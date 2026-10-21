#!/usr/bin/env python3
"""Line-follower simulator.

Two ways to drive the virtual car:
  FIRMWARE mode  runs YOUR C code (src/firmware/App: robot.c, calib.c ...) on the PC against
                 simulated drivers. Servo-steered car, 8 digital sensors -> one byte mask.
  PID mode       a built-in differential-drive PID demo (no firmware needed).

Left: map canvas. Right: button panel.
  1. Draw the track (Draw line / Erase), or load a PNG (Load PNG button, --map, or drop a file).
  2. Set START (= where the sensor bar sits), GOAL and HEADING with the tool buttons, then click the map.
  3. Run / Pause. The car stops at the goal.

Run:   python sim/simulator.py                    (finds ../src/firmware/App by itself)
       python sim/simulator.py --firmware path/to/src/firmware/App
       python sim/simulator.py --pid              (built-in PID demo)
       python sim/simulator.py --map maps/sample_course.png
       python sim/simulator.py --selftest         (headless check, no window)
"""
import argparse
import math
import os
import random
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

W, H = 1000, 700            # map size in pixels; 1 px = SCALE.mm_per_px millimetres
PANEL_W = 250
CAR_MAX_ANGLE = 35.0        # steering limit in degrees
SERVO_RATE = 400.0          # steering speed, degrees per second
MOTOR_TAU = 0.05            # motor lag (s)
N = 8                       # sensors in the bar (QTR-8A)
DT = 0.005                  # PID-mode control period (200 Hz)
TICK = 0.001                # firmware-mode tick: the 1 ms timer interrupt
V_MAX_MM = 500.0            # PID-mode speed at PWM 255 (mm/s)
ADC_WHITE, ADC_BLACK = 100, 900   # QTR-8A: floor reads LOW, black line reads HIGH
DIP_RUN = 4                 # DIP = 001 -> "run without gate bar" in loop()

SCALE = types.SimpleNamespace(mm_per_px=1.0)
DIM = types.SimpleNamespace(
    car_len=100, car_wid=90, wheelbase=170, sensor_fwd=45, sensor_gap=9.5, line_w=18,
    vmax=1000.0,        # firmware mode: speed at 100 % (mm/s)
    steer_gain=1.0,     # firmware mode: steering degrees per handle() unit
    brake_ms=60.0,      # firmware mode: time constant of the motor brake (ms)
    coast_ms=250.0)     # firmware mode: time constant when motors are simply set to 0 (ms)
PID_DEFAULTS = dict(car_len=100, car_wid=90, sensor_fwd=45, sensor_gap=9.5, line_w=18)
FW_DEFAULTS = dict(car_len=250, car_wid=140, wheelbase=170, sensor_fwd=210, sensor_gap=9.5,
                   line_w=19, vmax=1000.0, steer_gain=1.0, brake_ms=60.0, coast_ms=250.0)
FW_SCALE = 3.0              # firmware mode: 3 mm per pixel -> the map is 3 m x 2.1 m


def px(mm):
    return mm / SCALE.mm_per_px


def apply_defaults(mode):
    for k, v in (FW_DEFAULTS if mode == "fw" else PID_DEFAULTS).items():
        setattr(DIM, k, v)
    SCALE.mm_per_px = FW_SCALE if mode == "fw" else 1.0


# --------------------------------------------------------------------------
# PID controller (differential drive demo): mirror of the first version of this project
# --------------------------------------------------------------------------
class Controller:
    def __init__(self):
        self.kp, self.ki, self.kd = 80.0, 0.0, 8.0
        self.d_filter, self.i_limit = 0.35, 40.0
        self.base_pwm, self.max_pwm = 140, 255
        self.corner_slow, self.spin_pwm = 0.30, 110
        self.lost_level, self.line_reads_high, self.dt = 0.30, True, DT
        self.min_v = [ADC_WHITE] * N
        self.max_v = [ADC_BLACK] * N
        self.reset_state()
        self.last_side = 1.0

    def reset_state(self):
        self.integral = self.last_err = self.d_term = 0.0
        self.lost = False
        self.error = 0.0

    def activation(self, i, raw):
        span = self.max_v[i] - self.min_v[i]
        if span < 20:
            return 0.0
        a = min(1.0, max(0.0, (raw - self.min_v[i]) / span))
        return a if self.line_reads_high else 1.0 - a

    def position(self, raw):
        s = w = 0.0
        for i in range(N):
            a = self.activation(i, raw[i])
            s += a
            w += a * (i - 0.5 * (N - 1))
        found = s >= self.lost_level
        return (w / s if found else 0.0), found

    def update(self, raw):
        e, found = self.position(raw)
        self.lost = not found
        if self.lost:
            left = int(self.last_side * self.spin_pwm)
            return left, -left
        self.error = e
        if e > 0.3:
            self.last_side = 1.0
        elif e < -0.3:
            self.last_side = -1.0
        self.integral += e * self.dt * self.ki
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        raw_d = (e - self.last_err) / self.dt
        self.d_term += self.d_filter * (raw_d - self.d_term)
        self.last_err = e
        u = self.kp * e + self.integral + self.kd * self.d_term
        base = self.base_pwm * (1.0 - self.corner_slow * (abs(e) / (0.5 * (N - 1))))
        m = float(self.max_pwm)
        return int(max(-m, min(m, base + u))), int(max(-m, min(m, base - u)))


# --------------------------------------------------------------------------
# Car. (x, y) = centre of the rear axle (diff-drive demo: centre of the axle).
# start = (x, y, heading) of the SENSOR BAR centre; the axle is placed behind it.
# --------------------------------------------------------------------------
class Car:
    def __init__(self, x, y, th):
        self.start = (x, y, th)
        self.reset()

    def reset(self):
        sx, sy, th = self.start
        self.th = th
        self.x = sx - math.cos(th) * px(DIM.sensor_fwd)
        self.y = sy - math.sin(th) * px(DIM.sensor_fwd)
        self.vl = self.vr = 0.0
        self.delta = 0.0            # steering angle in degrees, positive = left
        self.trail = []

    def sensor_center(self):
        return (self.x + math.cos(self.th) * px(DIM.sensor_fwd),
                self.y + math.sin(self.th) * px(DIM.sensor_fwd))

    def sensor_points(self):
        fx, fy = math.cos(self.th), math.sin(self.th)
        lx, ly = math.sin(self.th), -math.cos(self.th)       # left of heading (screen y is down)
        cx, cy = self.sensor_center()
        pts = []
        for i in range(N):                                    # i = 0 is the LEFT-most sensor
            off = px(((N - 1) / 2 - i) * DIM.sensor_gap)
            pts.append((cx + lx * off, cy + ly * off))
        return pts

    def step(self, pwm_l, pwm_r, dt):
        """Differential drive (PID demo)."""
        a = dt / (MOTOR_TAU + dt)
        vmax = px(V_MAX_MM)
        self.vl += a * (pwm_l / 255.0 * vmax - self.vl)
        self.vr += a * (pwm_r / 255.0 * vmax - self.vr)
        v = (self.vl + self.vr) / 2.0
        omega = (self.vl - self.vr) / px(DIM.car_wid)         # faster left wheel -> turn right
        self.th += omega * dt
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt

    def step_servo(self, steer_units, pct_l, pct_r, brake_l, brake_r, dt):
        """Rear-wheel drive + servo steering (kinematic bicycle model). Positive steer = left."""
        target = max(-CAR_MAX_ANGLE, min(CAR_MAX_ANGLE, steer_units * DIM.steer_gain))
        lim = SERVO_RATE * dt
        self.delta += max(-lim, min(lim, target - self.delta))
        vmax = px(DIM.vmax)

        def gain(pct, brake):
            if pct == 0:                                      # target 0: brake or coast
                tau = DIM.brake_ms if brake else DIM.coast_ms
                return dt / (tau / 1000.0 + dt)
            return dt / (MOTOR_TAU + dt)

        tl = max(-100, min(100, pct_l)) / 100.0 * vmax
        tr = max(-100, min(100, pct_r)) / 100.0 * vmax
        self.vl += gain(pct_l, brake_l) * (tl - self.vl)
        self.vr += gain(pct_r, brake_r) * (tr - self.vr)
        v = (self.vl + self.vr) / 2.0
        omega = v * math.tan(math.radians(self.delta)) / px(DIM.wheelbase)
        self.th -= omega * dt                                  # left turn = heading decreases
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt


# --------------------------------------------------------------------------
# Sensors
# --------------------------------------------------------------------------
def sample_brightness(track, x, y):
    """Average brightness (0 = black .. 1 = white) of a small spot around (x, y)."""
    d = max(1.0, px(3.0))
    tot = 0.0
    for dx, dy in ((0, 0), (d, 0), (-d, 0), (0, d), (0, -d)):
        xi, yi = int(x + dx), int(y + dy)
        if 0 <= xi < W and 0 <= yi < H:
            c = track.get_at((xi, yi))
            tot += (c[0] + c[1] + c[2]) / 3.0
        else:
            tot += 255.0
    return tot / 5.0 / 255.0


def read_sensors(track, car, noise=12):
    """Fake ADC values for the PID demo (high = line)."""
    out = []
    for (x, y) in car.sensor_points():
        g = sample_brightness(track, x, y)
        out.append(int(ADC_BLACK + g * (ADC_WHITE - ADC_BLACK) + random.uniform(-noise, noise)))
    return out


def read_mask(track, car):
    """Firmware sensor byte: bit0 = right-most sensor ... bit7 = left-most, 1 = on the line."""
    mask = 0
    for i, (x, y) in enumerate(car.sensor_points()):
        if sample_brightness(track, x, y) < 0.5:
            mask |= 1 << (N - 1 - i)
    return mask


# --------------------------------------------------------------------------
# Maps
# --------------------------------------------------------------------------
def line_px():
    return max(2, int(round(px(DIM.line_w))))


def make_blank(pg):
    s = pg.Surface((W, H))
    s.fill((255, 255, 255))
    return s


def make_oval(pg):
    s = make_blank(pg)
    pg.draw.ellipse(s, (0, 0, 0), pg.Rect(150, 100, 700, 500), line_px())
    return s


def make_wavy(pg):
    """Closed loop with 3 lobes: r(theta) = 230 + 55*sin(3*theta)."""
    s = make_blank(pg)
    lw = line_px()
    pts = []
    for k in range(361):
        t = math.radians(k)
        r = 230 + 55 * math.sin(3 * t)
        pts.append((500 + r * math.cos(t) * 1.35, 350 + r * math.sin(t)))
    for a, b in zip(pts, pts[1:]):
        pg.draw.line(s, (0, 0, 0), a, b, lw)
        pg.draw.circle(s, (0, 0, 0), (int(b[0]), int(b[1])), lw // 2)
    return s


def make_course(pg):
    """Straight, cross line, 90 deg left corner, a second cross line, 90 deg right corner, straight.
    Start (100, 600) heading east, goal (880, 280). Meant for firmware mode (3 mm per pixel)."""
    s = make_blank(pg)
    lw = line_px()
    blk = (0, 0, 0)

    def seg(a, b):
        pg.draw.line(s, blk, a, b, lw)
        pg.draw.circle(s, blk, a, lw // 2)
        pg.draw.circle(s, blk, b, lw // 2)

    seg((60, 600), (600, 600))      # approach
    seg((600, 600), (600, 280))     # after the left corner
    seg((600, 280), (900, 280))     # after the right corner
    seg((470, 560), (470, 640))     # cross line before the left corner
    seg((520, 420), (680, 420))     # a second cross line on the vertical part
    return s


COURSE_START = (100, 600, 0.0)
COURSE_GOAL = (880, 280)


WAVY_START = (500 + 230 * 1.35, 350, math.pi / 2)   # right-most point, heading down


def load_png(pg, path, invert=False):
    img = pg.image.load(path).convert()
    iw, ih = img.get_size()
    sc = min(W / iw, H / ih)
    img = pg.transform.smoothscale(img, (max(1, int(iw * sc)), max(1, int(ih * sc))))
    s = make_blank(pg)
    s.blit(img, ((W - img.get_width()) // 2, (H - img.get_height()) // 2))
    return invert_surface(pg, s) if invert else s


def invert_surface(pg, s):
    n = pg.Surface((W, H))
    n.fill((255, 255, 255))
    n.blit(s, (0, 0), special_flags=pg.BLEND_RGB_SUB)
    return n


def estimate_line_width(pg, track):
    """Rough line width in px (None if the map has no line): erode 1 px at a time until nothing is left."""
    def dark(surf):
        return pg.mask.from_threshold(surf, (0, 0, 0, 255), (100, 100, 100, 255)).count()
    if dark(track) == 0:
        return None
    cur = track.copy()
    for n in range(1, 60):
        out = cur.copy()
        steps = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if n % 2 == 0:                      # alternate cross / square -> roughly round erosion
            steps += [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        for dx, dy in steps:
            out.blit(cur, (dx, dy), special_flags=pg.BLEND_RGB_MAX)
        cur = out
        if dark(cur) == 0:
            return max(2, 2 * n - 1)
    return 60


# --------------------------------------------------------------------------
# Headless self-tests
# --------------------------------------------------------------------------
def on_line_at_sensor_bar(track, car):
    cx, cy = car.sensor_center()
    return 0 <= cx < W and 0 <= cy < H and track.get_at((int(cx), int(cy)))[0] < 128


def run_headless(pg, track, car, ctl, seconds=40.0):
    car.reset()
    ctl.reset_state()
    on_line = lost = 0
    steps = int(seconds / DT)
    for _ in range(steps):
        raw = read_sensors(track, car)
        l, r = ctl.update(raw)
        car.step(l, r, DT)
        lost += ctl.lost
        on_line += on_line_at_sensor_bar(track, car)
    inside = 0 <= car.x < W and 0 <= car.y < H
    return 100.0 * on_line / steps, 100.0 * lost / steps, inside


def run_headless_fw(track, car, fw, seconds=40.0, centre=(500, 350)):
    """Drive the car with the real firmware. Returns (% of time line seen, laps, state histogram, car)."""
    car.reset()
    fw.reset()
    seen = 0
    cases = {}
    steps = int(seconds / TICK)
    prev, turned = None, 0.0
    k = 0
    for k in range(steps):
        mask = read_mask(track, car)
        out = fw.step(mask, 4 if 10 <= k < 40 else 0, DIP_RUN, k, TICK)
        car.step_servo(out.servo, out.speed_l, out.speed_r, out.brake_l, out.brake_r, TICK)
        seen += mask != 0
        name = fw.runcase_name(out.runcase)
        cases[name] = cases.get(name, 0) + 1
        if k % 20 == 0:
            cx, cy = car.sensor_center()
            ang = math.atan2(cy - centre[1], cx - centre[0])
            if prev is not None:
                d = ang - prev
                turned += (d + math.pi) % (2 * math.pi) - math.pi
            prev = ang
        if not (0 <= car.x < W and 0 <= car.y < H):
            break
    n = max(1, k + 1)
    return 100.0 * seen / n, abs(turned) / (2 * math.pi), cases, car


def selftest(fw_dir=None, exclude=None):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    import pygame as pg
    pg.init()
    ok = True
    apply_defaults("pid")
    os.makedirs("maps", exist_ok=True)
    maps = {"sample_oval": (make_oval(pg), (500, 109, 0.0)),
            "sample_wavy": (make_wavy(pg), WAVY_START)}
    for name, (track, start) in maps.items():
        pg.image.save(track, os.path.join("maps", name + ".png"))
        car, ctl = Car(*start), Controller()
        random.seed(1)
        on, lost, inside = run_headless(pg, track, car, ctl)
        print(f"[PID]      {name}: on-line {on:5.1f}%  line-lost {lost:4.1f}%  stayed-in-map {inside}")
        ok &= on > 80 and inside
    if fw_dir:
        import fw_bridge, fw_full
        try:
            fw = fw_full.FullFirmware(fw_dir, exclude or ())
        except fw_bridge.FirmwareError as exc:
            print(exc)
            return 1
        apply_defaults("fw")
        track, car = make_oval(pg), Car(500, 109, 0.0)
        seen, laps, cases, car = run_headless_fw(track, car, fw, 40.0)
        top = ", ".join(f"{k}:{v}" for k, v in sorted(cases.items(), key=lambda kv: -kv[1])[:4])
        print(f"[FIRMWARE] oval: line seen {seen:5.1f}%  laps {laps:4.1f} in 40 s  states {top}")
        ok &= seen > 95 and laps >= 1.0
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# --------------------------------------------------------------------------
# Interactive app: map canvas on the left, button panel on the right
# --------------------------------------------------------------------------
GOAL_R = 30               # goal counts as reached inside this radius (px)
GREEN, RED = (30, 160, 60), (210, 50, 50)
PANEL_BG, BTN_BG, BTN_HOT = (238, 238, 238), (210, 214, 220), (50, 120, 230)

SIZE_SPECS = [   # key, label, step, min, max, modes ("both" / "fw" / "pid")
    ("car_len", "car length", 5, 40, 400, "both"),
    ("car_wid", "car width", 5, 40, 300, "both"),
    ("wheelbase", "wheelbase", 5, 60, 350, "fw"),
    ("sensor_fwd", "sensor ahead", 5, 0, 300, "both"),
    ("sensor_gap", "sensor pitch", 0.5, 4, 30, "both"),
    ("line_w", "line width", 1, 4, 60, "both"),
    ("scale", "map mm/px", 0.5, 0.5, 10, "both"),
    ("vmax", "max mm/s", 100, 200, 5000, "fw"),
    ("steer_gain", "deg per unit", 0.1, 0.2, 3.0, "fw"),
    ("brake_ms", "brake ms", 10, 10, 500, "fw"),
]
SPEC = {s[0]: s for s in SIZE_SPECS}


def pick_image_file():
    """Native file dialog (tkinter ships with Python on Windows/macOS). None if unavailable."""
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        path = filedialog.askopenfilename(
            title="Choose a map image",
            filetypes=[("Images", "*.png *.jpg *.jpeg *.bmp"), ("All files", "*.*")])
        root.destroy()
        return path or None
    except Exception:
        return None


class Btn:
    def __init__(self, pg, label, rect, on_click, tool=None):
        self.label, self.rect, self.on_click, self.tool = label, pg.Rect(rect), on_click, tool

    def text(self):
        return self.label() if callable(self.label) else self.label


class App:
    def __init__(self, pg, track, fw=None, mode="pid", note=""):
        self.pg = pg
        self.track = track
        self.fw = fw
        self.mode = "fw" if (mode == "fw" and fw) else "pid"
        self.heading = 0.0
        self.car = Car(500, 109, self.heading)
        self.ctl = Controller()
        self.tool = "draw"
        self.goal = None
        self.running = False
        self.finished = None        # None | "goal" | "fail"
        self.msg, self.msg_until = "", 0
        self.acc = self.sim_t = self.dist = self.lost_t = 0.0
        self.speed_mult = 1
        self.mask = 0
        self.out = None
        self.held = None
        self.last = None
        self.font = pg.font.SysFont("consolas,menlo,dejavusansmono,monospace", 14)
        self.bold = pg.font.SysFont("consolas,menlo,dejavusansmono,monospace", 14, bold=True)
        self.buttons = self._build_buttons()
        if note:
            self.say(note, 10)

    # ---------------- panel layout ----------------
    def _build_buttons(self):
        pg = self.pg
        self.labels, self.tune_rows = [], []
        bs, x0, w, y = [], W + 12, PANEL_W - 24, 5
        BH, PITCH = 22, 25

        def header(text):
            nonlocal y
            self.labels.append((text, x0, y))
            y += 16

        def row(items):
            nonlocal y
            n, gap = len(items), 6
            bw = (w - gap * (n - 1)) / n
            for i, (label, fn, tool) in enumerate(items):
                bs.append(Btn(pg, label, (x0 + i * (bw + gap), y, bw, BH), fn, tool))
            y += PITCH

        def adjust(key, step):
            nonlocal y
            bs.append(Btn(pg, "-", (x0, y, 34, BH), lambda k=key, s=step: self.tune(k, -s)))
            bs.append(Btn(pg, "+", (x0 + w - 34, y, 34, BH), lambda k=key, s=step: self.tune(k, s)))
            self.tune_rows.append((key, y))
            y += PITCH

        header("TOOLS (click on the map)")
        row([("Draw line", lambda: self.set_tool("draw"), "draw"),
             ("Erase", lambda: self.set_tool("erase"), "erase")])
        row([("Set START", lambda: self.set_tool("start"), "start"),
             ("Set GOAL", lambda: self.set_tool("goal"), "goal")])
        row([("Set HEADING", lambda: self.set_tool("heading"), "heading")])
        header("CAR")
        row([("Run / Pause", self.toggle_run, None), ("Reset car", self.reset_car, None)])
        row([("Auto heading", self.auto_heading, None)])
        row([("Turn -15 deg", lambda: self.rotate_heading(-15), None),
             ("Turn +15 deg", lambda: self.rotate_heading(15), None)])
        row([(lambda: f"Sim speed x{self.speed_mult}", self.cycle_speed, None)])
        if self.fw:
            row([(lambda: "Mode: FIRMWARE (C code)" if self.mode == "fw" else "Mode: PID demo",
                  self.toggle_mode, None)])
        header("MAP")
        row([("Clear", self.clear_map, None), ("Invert", self.invert_map, None)])
        row([("Load PNG", self.load_png_dialog, None), ("Save PNG", self.save_png, None)])
        row([("Clear trail", lambda: self.car.trail.clear(), None)])
        if self.mode == "pid":
            header("PID TUNING")
            for key, step in (("kp", 5), ("kd", 1), ("base", 10)):
                adjust(key, step)
        header("SIZE / MODEL")
        for key, label, step, lo, hi, modes in SIZE_SPECS:
            if modes in ("both", self.mode):
                adjust(key, step)
        self.status_y = y + 4
        return bs

    # ---------------- helpers ----------------
    def say(self, text, secs=5.0):
        self.msg, self.msg_until = text, self.pg.time.get_ticks() + int(secs * 1000)

    def dark_at(self, x, y):
        if 0 <= x < W and 0 <= y < H:
            return self.track.get_at((int(x), int(y)))[0] < 128
        return False

    def set_tool(self, t):
        self.tool = t
        self.say({"draw": "Draw: hold left mouse on the map",
                  "erase": "Erase: hold left mouse on the map",
                  "start": "Start: click where the SENSOR BAR begins (snaps to the line)",
                  "goal": "Goal: click where the car should stop",
                  "heading": "Heading: click the point the car should face"}[t])

    def toggle_run(self):
        if self.finished:
            self.reset_car()
        self.running = not self.running
        self.acc = 0.0

    def cycle_speed(self):
        self.speed_mult = {1: 2, 2: 4, 4: 8, 8: 1}[self.speed_mult]

    def toggle_mode(self):
        self.mode = "pid" if self.mode == "fw" else "fw"
        apply_defaults(self.mode)
        self.buttons = self._build_buttons()
        self.reset_car()
        self.say("FIRMWARE mode: your C code drives the car" if self.mode == "fw"
                 else "PID demo mode (differential drive)")

    def reset_car(self):
        self.car.start = (self.car.start[0], self.car.start[1], self.heading)
        self.car.reset()
        self.ctl.reset_state()
        if self.mode == "fw":
            self.fw.reset()
        self.running = False
        self.finished = None
        self.mask = 0
        self.out = None
        self.acc = self.sim_t = self.dist = self.lost_t = 0.0

    def rotate_heading(self, deg):
        self.heading += math.radians(deg)
        self.reset_car()

    def auto_heading(self):
        """Point the car along the drawn line next to the start (towards the goal if set)."""
        sx, sy = self.car.start[:2]
        pts = [(sx + dx, sy + dy) for dy in range(-40, 41, 2) for dx in range(-40, 41, 2)
               if dx * dx + dy * dy <= 1600 and self.dark_at(sx + dx, sy + dy)]
        if len(pts) < 8:
            self.say("No line near the start - move START onto the line")
            return
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        sxx = sum((p[0] - mx) ** 2 for p in pts)
        syy = sum((p[1] - my) ** 2 for p in pts)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
        a = 0.5 * math.atan2(2 * sxy, sxx - syy)
        ref = math.atan2(self.goal[1] - sy, self.goal[0] - sx) if self.goal else self.heading
        if math.cos(a - ref) < 0:
            a += math.pi
        self.heading = a
        self.reset_car()
        self.say("Heading set along the line")

    def clear_map(self):
        self.track = make_blank(self.pg)
        self.reset_car()

    def invert_map(self):
        self.track = invert_surface(self.pg, self.track)

    def refresh_line_width(self):
        est = estimate_line_width(self.pg, self.track)
        if est:
            DIM.line_w = round(est * SCALE.mm_per_px, 1)

    def load_png_path(self, path):
        try:
            self.track = load_png(self.pg, path)
            self.refresh_line_width()
            self.reset_car()
            self.say(f"Loaded {os.path.basename(path)} (line about {DIM.line_w:g} mm at "
                     f"{SCALE.mm_per_px:g} mm/px)", 7)
        except Exception as exc:
            self.say(f"Could not load: {exc}", 6)

    def load_png_dialog(self):
        path = pick_image_file()
        if path:
            self.load_png_path(path)
        else:
            self.say("No file chosen. You can also drop a PNG on the window.", 6)

    def save_png(self):
        os.makedirs("maps", exist_ok=True)
        self.pg.image.save(self.track, "maps/drawn_track.png")
        self.say("Saved maps/drawn_track.png")

    def tune(self, name, delta):
        c = self.ctl
        if name == "kp":
            c.kp = max(0.0, c.kp + delta)
        elif name == "kd":
            c.kd = max(0.0, c.kd + delta)
        elif name == "base":
            c.base_pwm = int(min(255, max(40, c.base_pwm + delta)))
        elif name == "scale":
            _, _, _, lo, hi, _ = SPEC[name]
            SCALE.mm_per_px = round(min(hi, max(lo, SCALE.mm_per_px + delta)), 2)
            self.refresh_line_width()
            self.reset_car()
            self.say(f"Map is now {W * SCALE.mm_per_px:g} x {H * SCALE.mm_per_px:g} mm; "
                     "lines keep their pixel width", 6)
        elif name in SPEC:
            _, _, _, lo, hi, _ = SPEC[name]
            old = getattr(DIM, name)
            new = round(min(hi, max(lo, old + delta)), 2)
            if name == "line_w" and new != old:
                self.resize_line(px(new - old))        # redraw the map's lines at the new width
            setattr(DIM, name, new)
            if name in ("sensor_fwd", "car_len", "wheelbase"):
                self.reset_car()
            if name == "line_w":
                self.say(f"Line width {new:g} mm applied to the whole map")

    def value_text(self, key):
        if key in SPEC:
            val = SCALE.mm_per_px if key == "scale" else getattr(DIM, key)
            return f"{SPEC[key][1]} {val:g}"
        return {"kp": f"kp = {self.ctl.kp:.0f}", "kd": f"kd = {self.ctl.kd:.0f}",
                "base": f"speed = {self.ctl.base_pwm}"}[key]

    def resize_line(self, delta_px):
        """Make every dark line wider (delta > 0) or thinner (delta < 0) by about |delta| px."""
        pg = self.pg
        r = int(round(abs(delta_px) / 2.0))
        if r <= 0:
            return
        flag = pg.BLEND_RGB_MIN if delta_px > 0 else pg.BLEND_RGB_MAX
        out = self.track.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if (dx or dy) and dx * dx + dy * dy <= r * r + 1:
                    out.blit(self.track, (dx, dy), special_flags=flag)
        self.track = out

    def snap_to_line(self, x, y, radius=30):
        if self.dark_at(x, y):
            return x, y
        best, bd = None, radius * radius + 1
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                d = dx * dx + dy * dy
                if d < bd and self.dark_at(x + dx, y + dy):
                    best, bd = (x + dx, y + dy), d
        if best:
            return best
        self.say("START is not on a line - draw a line there or move it", 5)
        return x, y

    # ---------------- mouse tools ----------------
    def apply_tool(self, tool, pos, first):
        pg = self.pg
        x, y = pos
        if tool in ("draw", "erase"):
            lw = line_px()
            color, width = ((0, 0, 0), lw) if tool == "draw" else ((255, 255, 255), lw * 3)
            a = pos if first or self.last is None else self.last
            pg.draw.line(self.track, color, a, pos, width)
            pg.draw.circle(self.track, color, pos, max(1, width // 2))
            self.last = pos
        elif tool == "start":
            sx, sy = self.snap_to_line(x, y)
            self.car.start = (sx, sy, self.heading)
            self.reset_car()
        elif tool == "goal":
            self.goal = (x, y)
            self.finished = None
        elif tool == "heading":
            sx, sy = self.car.start[:2]
            if math.hypot(x - sx, y - sy) > 4:
                self.heading = math.atan2(y - sy, x - sx)
                self.reset_car()

    def handle_event(self, ev):
        """Returns False when the app should quit."""
        pg = self.pg
        if ev.type == pg.QUIT:
            return False
        if ev.type == pg.DROPFILE:
            self.load_png_path(ev.file)
        elif ev.type == pg.MOUSEBUTTONDOWN and ev.button in (1, 3):
            if ev.pos[0] >= W:
                if ev.button == 1:
                    for b in self.buttons:
                        if b.rect.collidepoint(ev.pos):
                            b.on_click()
                            break
            else:
                self.held = self.tool if ev.button == 1 else "erase"
                self.last = None
                self.apply_tool(self.held, ev.pos, True)
        elif ev.type == pg.MOUSEBUTTONUP:
            self.held = None
            self.last = None
        elif ev.type == pg.MOUSEMOTION and self.held and ev.pos[0] < W:
            self.apply_tool(self.held, ev.pos, False)
        elif ev.type == pg.KEYDOWN:
            if ev.key == pg.K_ESCAPE:
                return False
            if ev.key == pg.K_SPACE:
                self.toggle_run()
            elif ev.key == pg.K_r:
                self.reset_car()
            elif ev.key == pg.K_c:
                self.clear_map()
        return True

    # ---------------- simulation ----------------
    def step_pid(self, dt):
        car = self.car
        raw = read_sensors(self.track, car)
        l, r = self.ctl.update(raw)
        car.step(l, r, dt)
        self.lost_now = self.ctl.lost

    def step_fw(self, dt):
        car = self.car
        t_ms = int(round(self.sim_t * 1000))
        mask = read_mask(self.track, car)
        buttons = 4 if 10 <= t_ms < 40 else 0          # a short press of SW3 starts the run
        out = self.fw.step(mask, buttons, DIP_RUN, t_ms, dt)
        car.step_servo(out.servo, out.speed_l, out.speed_r, out.brake_l, out.brake_r, dt)
        self.mask, self.out = mask, out
        self.lost_now = mask == 0

    def update(self, dt_real):
        if not self.running:
            self.acc = 0.0
            return
        dt = TICK if self.mode == "fw" else DT
        self.acc = min(self.acc + dt_real * self.speed_mult, 0.25)
        car = self.car
        while self.running and self.acc >= dt:
            self.acc -= dt
            px0, py0 = car.x, car.y
            if self.mode == "fw":
                self.step_fw(dt)
            else:
                self.step_pid(dt)
            self.sim_t += dt
            self.dist += math.hypot(car.x - px0, car.y - py0) * SCALE.mm_per_px
            self.lost_t = self.lost_t + dt if self.lost_now else 0.0
            cx, cy = car.sensor_center()
            if not car.trail or math.hypot(cx - car.trail[-1][0], cy - car.trail[-1][1]) > 4:
                car.trail.append((cx, cy))
                if len(car.trail) > 4000:
                    car.trail.pop(0)
            if self.goal and math.hypot(cx - self.goal[0], cy - self.goal[1]) < GOAL_R:
                self.running, self.finished = False, "goal"
            elif self.lost_t > 3.0:
                self.running, self.finished = False, "fail"
                self.say("Stopped: line lost for 3 s", 8)
            elif not (0 <= car.x < W and 0 <= car.y < H):
                self.running, self.finished = False, "fail"
                self.say("Stopped: car left the map", 8)

    # ---------------- drawing ----------------
    def draw_arrow(self, screen, x, y, th, length, color):
        pg = self.pg
        ex, ey = x + math.cos(th) * length, y + math.sin(th) * length
        pg.draw.line(screen, color, (x, y), (ex, ey), 3)
        for s in (-1, 1):
            a = th + math.pi + s * 0.45
            pg.draw.line(screen, color, (ex, ey), (ex + math.cos(a) * 12, ey + math.sin(a) * 12), 3)

    def text(self, screen, s, x, y, color=(20, 20, 20), bold=False, bg=None):
        screen.blit((self.bold if bold else self.font).render(s, True, color, bg), (x, y))

    @staticmethod
    def oriented_rect(cx, cy, th, length, width):
        fx, fy = math.cos(th), math.sin(th)
        lx, ly = math.sin(th), -math.cos(th)
        hl, hw = length / 2, width / 2
        return [(cx - fx * hl + lx * hw, cy - fy * hl + ly * hw),
                (cx + fx * hl + lx * hw, cy + fy * hl + ly * hw),
                (cx + fx * hl - lx * hw, cy + fy * hl - ly * hw),
                (cx - fx * hl - lx * hw, cy - fy * hl - ly * hw)]

    def draw_car(self, screen):
        pg, car = self.pg, self.car
        th = car.th
        fx, fy = math.cos(th), math.sin(th)
        lx, ly = math.sin(th), -math.cos(th)
        if self.mode == "fw":
            wb, wd = px(DIM.wheelbase), px(DIM.car_wid)
            rear = px((DIM.car_len - DIM.wheelbase) / 3.0)
            front = px(DIM.car_len) - wb - rear
            c0, c1 = -rear, wb + front                      # body from rear edge to front edge
            mid = (c0 + c1) / 2
            body = self.oriented_rect(car.x + fx * mid, car.y + fy * mid, th, c1 - c0, wd * 0.8)
            pg.draw.polygon(screen, (230, 90, 60), body, 3)
            wl, ww = px(45), max(3, px(18))
            for side in (-1, 1):                              # rear wheels
                pg.draw.polygon(screen, (60, 60, 60), self.oriented_rect(
                    car.x + lx * side * wd / 2, car.y + ly * side * wd / 2, th, wl, ww))
            steer_th = th - math.radians(car.delta)           # left (+) = heading decreases
            for side in (-1, 1):                              # front wheels, turned by the servo
                pg.draw.polygon(screen, (60, 60, 60), self.oriented_rect(
                    car.x + fx * wb + lx * side * wd / 2, car.y + fy * wb + ly * side * wd / 2,
                    steer_th, wl, ww))
        else:
            hl, hw = px(DIM.car_len) / 2, px(DIM.car_wid) / 2
            corners = [(car.x - fx * hl + lx * hw, car.y - fy * hl + ly * hw),
                       (car.x + fx * hl + lx * hw, car.y + fy * hl + ly * hw),
                       (car.x + fx * hl - lx * hw, car.y + fy * hl - ly * hw),
                       (car.x - fx * hl - lx * hw, car.y - fy * hl - ly * hw)]
            pg.draw.polygon(screen, (230, 90, 60), corners, 3)
        pts = car.sensor_points()
        pg.draw.line(screen, (120, 120, 120), pts[0], pts[-1], 1)
        for (x, y) in pts:
            on = sample_brightness(self.track, x, y) < 0.5
            pg.draw.circle(screen, (0, 200, 0) if on else (150, 150, 150), (int(x), int(y)), 4)

    def draw(self, screen):
        pg, car = self.pg, self.car
        screen.blit(self.track, (0, 0))
        if len(car.trail) > 1:
            pg.draw.lines(screen, (90, 160, 255), False, car.trail, 2)
        if self.goal:
            pg.draw.circle(screen, RED, self.goal, GOAL_R, 3)
            pg.draw.circle(screen, RED, self.goal, 4)
            self.text(screen, "GOAL", self.goal[0] - 16, self.goal[1] - GOAL_R - 18, RED, True)
        sx, sy, _ = car.start
        pg.draw.circle(screen, GREEN, (int(sx), int(sy)), 9, 3)
        self.draw_arrow(screen, sx, sy, self.heading, 55, GREEN)
        self.text(screen, "START", sx - 20, sy + 14, GREEN, True)
        self.draw_car(screen)

        # ---- panel ----
        pg.draw.rect(screen, PANEL_BG, (W, 0, PANEL_W, H))
        pg.draw.line(screen, (150, 150, 150), (W, 0), (W, H), 2)
        for s, x, y in self.labels:
            self.text(screen, s, x, y, (90, 90, 90), True)
        for b in self.buttons:
            hot = b.tool is not None and b.tool == self.tool
            pg.draw.rect(screen, BTN_HOT if hot else BTN_BG, b.rect, border_radius=5)
            t = (self.bold if hot else self.font).render(b.text(), True, (255, 255, 255) if hot else (20, 20, 20))
            screen.blit(t, t.get_rect(center=b.rect.center))
        for name, y in self.tune_rows:
            self.text(screen, self.value_text(name), W + 12 + 40, y + 3)
        y = self.status_y
        lines = []
        if self.finished == "goal":
            self.text(screen, "GOAL REACHED", W + 12, y, GREEN, True)
            lines.append(f"{self.sim_t:.1f} s   {self.dist / 1000:.2f} m")
        elif self.finished == "fail":
            self.text(screen, "RUN FAILED", W + 12, y, RED, True)
        else:
            state = "RUNNING" if self.running else "PAUSED"
            self.text(screen, f"{state}  t={self.sim_t:.1f}s", W + 12, y, (20, 20, 20), True)
        if self.mode == "fw":
            o = self.out
            if o is not None:
                lines += [f"case {self.fw.runcase_name(o.runcase)}",
                          f"sensor {self.mask:08b}",
                          f"servo {o.servo_us}us  L {o.speed_l:+.0f} R {o.speed_r:+.0f}",
                          f"tick {o.tick_ms} ms  TIM1 {o.tim1_us/1000:.2f} ms"]
        else:
            lines.append(f"error {self.ctl.error:+.2f}" + ("  LINE LOST" if self.ctl.lost else ""))
        for i, t in enumerate(lines):
            self.text(screen, t, W + 12, y + 17 + i * 16)
        if self.msg and pg.time.get_ticks() < self.msg_until:
            ly2 = y + 22 + len(lines) * 16
            line = ""
            for wd in self.msg.split():
                if len(line) + len(wd) + 1 > 28:
                    self.text(screen, line, W + 12, ly2, (150, 60, 0))
                    ly2 += 16
                    line = wd
                else:
                    line = (line + " " + wd).strip()
            self.text(screen, line, W + 12, ly2, (150, 60, 0))
        hint = "SPACE run | R reset | C clear | right-drag erase | drop PNG to load"
        self.text(screen, hint, 8, H - 20, (60, 60, 60), bg=(255, 255, 255))


def main():
    ap = argparse.ArgumentParser(description="Line-follower simulator")
    ap.add_argument("--map", help="PNG track to load (dark line on light floor)")
    ap.add_argument("--invert", action="store_true", help="invert PNG (white line on black)")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--firmware", metavar="DIR",
                    help="your src/ folder (the one with MDK-ARM/*.uvprojx); default: ../src next to sim/")
    ap.add_argument("--pid", action="store_true", help="use the built-in PID demo, not your firmware")
    ap.add_argument("--fw-exclude", nargs="*", metavar="FILE",
                    help="C files to leave out of the PC build")
    ap.add_argument("--loop-us", type=int, default=50, help="firmware: CPU time of one loop() pass, us")
    ap.add_argument("--brake-always", action="store_true", help="brake pins brake even while PWM is non-zero")
    ap.add_argument("--servo-center", type=float, help="servo pulse (us) where the real wheels point straight (default 1472)")
    ap.add_argument("--car-length", type=float, help="car length, mm")
    ap.add_argument("--car-width", type=float, help="car width / wheel track, mm")
    ap.add_argument("--wheelbase", type=float, help="rear axle to front axle, mm (firmware mode)")
    ap.add_argument("--sensor-ahead", type=float, help="rear axle to sensor bar, mm")
    ap.add_argument("--sensor-pitch", type=float, help="sensor spacing, mm (QTR-8A: 9.5)")
    ap.add_argument("--line-width", type=float, help="line width for drawing, mm")
    ap.add_argument("--mm-per-px", type=float, help="map scale: millimetres per pixel")
    ap.add_argument("--max-speed", type=float, help="firmware mode: speed at 100%% motor power, mm/s")
    ap.add_argument("--servo-gain", type=float, help="firmware mode: steering degrees per handle() unit")
    ap.add_argument("--brake-ms", type=float, help="firmware mode: motor brake time constant, ms")
    args = ap.parse_args()

    import fw_bridge, fw_full, fw_build
    fw_dir = Path(args.firmware).expanduser() if args.firmware else fw_build.autodetect()

    if args.selftest:
        sys.exit(selftest(fw_dir if (args.firmware and not args.pid) else None, args.fw_exclude))

    fw, note = None, ""
    if fw_dir and not args.pid:
        try:
            fw = fw_full.FullFirmware(fw_dir, args.fw_exclude or (), loop_us=args.loop_us)
            fw.brake_always = args.brake_always
            if args.servo_center:
                fw.SERVO_MECH_CENTER = args.servo_center
            print(f"Firmware: {fw.root} ({len(fw.sources)} source files, built {fw.lib_path.name})")
            print("  main.c:", "uses its own robot loop" if not fw.glue else
                  "has no robot_init/loop() call -> sim calls them like the commented-out lines")
        except fw_bridge.FirmwareError as exc:
            print(f"\n{exc}\n")
            note = "Firmware build failed - see the console. Using the PID demo."
    elif not args.pid:
        note = "No firmware found (expected ../src with MDK-ARM/*.uvprojx). Using the PID demo."
    mode = "fw" if fw else "pid"

    apply_defaults(mode)
    for val, key in ((args.car_length, "car_len"), (args.car_width, "car_wid"),
                     (args.wheelbase, "wheelbase"), (args.sensor_ahead, "sensor_fwd"),
                     (args.sensor_pitch, "sensor_gap"), (args.line_width, "line_w"),
                     (args.max_speed, "vmax"), (args.servo_gain, "steer_gain"),
                     (args.brake_ms, "brake_ms")):
        if val:
            setattr(DIM, key, val)
    if args.mm_per_px:
        SCALE.mm_per_px = args.mm_per_px

    import pygame as pg
    pg.init()
    screen = pg.display.set_mode((W + PANEL_W, H))
    pg.display.set_caption("Line follower simulator")
    track = load_png(pg, args.map, args.invert) if args.map else make_oval(pg)
    app = App(pg, track, fw, mode, note)
    if args.map and not args.line_width:
        app.refresh_line_width()
    clock = pg.time.Clock()
    alive = True
    while alive:
        for ev in pg.event.get():
            alive = app.handle_event(ev) and alive
        app.update(clock.tick(60) / 1000.0)
        app.draw(screen)
        pg.display.flip()
    if app.mode == "pid":
        c = app.ctl
        print("\n// PID demo values:")
        print(f"  kp = {c.kp:.1f}, ki = {c.ki:.1f}, kd = {c.kd:.1f}, base = {c.base_pwm}")
    print(f"// Geometry used (mm): car {DIM.car_len:g} x {DIM.car_wid:g}, wheelbase {DIM.wheelbase:g}, "
          f"sensors {DIM.sensor_fwd:g} ahead, pitch {DIM.sensor_gap:g}, line {DIM.line_w:g}, "
          f"map {SCALE.mm_per_px:g} mm/px")
    pg.quit()


if __name__ == "__main__":
    main()
