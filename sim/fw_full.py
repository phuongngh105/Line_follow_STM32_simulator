"""Run the user's complete STM32 source tree (src/firmware) on the PC. See fw_build.py."""
import ctypes, os, shutil, types
from pathlib import Path
from fw_bridge import FirmwareError, find_compiler, parse_runcases
import fw_build


class SimOut(ctypes.Structure):
    _fields_ = [(n, ctypes.c_int32) for n in (
        "pwm_l", "pwm_r", "arr", "dir_l", "dir_r", "brake_l", "brake_r",
        "servo_ccr", "servo_center", "servo_arr", "runcase", "running", "line_lost", "cnt1")] + \
        [(n, ctypes.c_uint32) for n in ("odr_a", "odr_b", "odr_c", "tick_ms", "tim1_us")] + \
        [("robot_ready", ctypes.c_int32), ("finished", ctypes.c_int32)]


class FullFirmware:
    ADC_BLACK, ADC_WHITE = 3300, 300

    def __init__(self, path, exclude=(), glue=None, loop_us=50, call_cost=0.5):
        root = fw_build.find_root(path)
        if root is None:
            raise FirmwareError(f"No MDK-ARM/*.uvprojx found under {path}")
        self.root = root
        self.lib_path, self.glue, self.sources = fw_build.build(root, exclude, glue)
        cc = find_compiler()
        if os.name == "nt" and cc and hasattr(os, "add_dll_directory"):
            try: os.add_dll_directory(str(Path(shutil.which(cc[0]) or cc[0]).resolve().parent))
            except OSError: pass
        self.lib = None
        self._copies = 0
        self.loop_us, self.call_cost = loop_us, call_cost
        self.names = parse_runcases(root / "App" / "robot.h")
        self.out = SimOut()
        self.reset()

    def _load(self):
        """A fresh copy of the library = fresh .data/.bss, i.e. a power-cycled MCU."""
        self._unload()
        self._copies += 1
        cp = self.lib_path.with_name(f"{self.lib_path.stem}_run{os.getpid()}_{self._copies}{self.lib_path.suffix}")
        shutil.copyfile(self.lib_path, cp)
        try:
            self.lib = ctypes.CDLL(str(cp))
        except OSError as e:
            raise FirmwareError(f"Could not load {self.lib_path}: {e}")
        self._cp = cp
        L = self.lib
        L.sim_start.argtypes = [ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p]
        L.sim_start.restype = ctypes.c_int
        L.sim_error.restype = ctypes.c_char_p
        L.sim_set_adc.argtypes = [ctypes.c_void_p]
        L.sim_inputs.argtypes = [ctypes.c_uint8, ctypes.c_uint8]
        L.sim_run.argtypes = [ctypes.c_uint32, ctypes.POINTER(SimOut)]

    def _unload(self):
        if self.lib is None:
            return
        h, cp = self.lib._handle, self._cp
        self.lib = None
        try:
            if os.name == "nt":
                ctypes.windll.kernel32.FreeLibrary.argtypes = [ctypes.c_void_p]
                ctypes.windll.kernel32.FreeLibrary(h)
            else:
                import _ctypes
                _ctypes.dlclose(h)
        except Exception:
            pass
        try:
            cp.unlink()
        except OSError:
            pass

    def __del__(self):
        try:
            self._unload()
        except Exception:
            pass

    def reset(self):
        self._load()
        if self.lib.sim_start(1 if self.glue else 0, self.loop_us, int(self.call_cost * 10), None) != 0:
            raise FirmwareError(self.lib.sim_error().decode())
        self.set_sensors(0); self.inputs(0, 0)
        self.run(1000)
        self.calibrate()

    def set_sensors(self, mask):
        """mask bit0 = rightmost sensor, 1 = black line (the user's sensor_value[0] is the leftmost)."""
        arr = (ctypes.c_uint16 * 8)(*[self.ADC_BLACK if (mask >> (7 - i)) & 1 else self.ADC_WHITE for i in range(8)])
        self.lib.sim_set_adc(arr)

    def inputs(self, buttons, dip):
        """buttons bit0..2 = SW1..SW3 pressed; dip 0..7 (dip1 = MSB)."""
        self.lib.sim_inputs(buttons & 7, dip & 7)

    def run(self, us):
        self.lib.sim_run(int(us), ctypes.byref(self.out))
        return self.out

    def runcase_name(self, v):
        return self.names.get(int(v), str(int(v)))

    def _wait(self, cond, ms=3000):
        for _ in range(ms // 10):
            self.run(10_000)
            if cond(self.lib.sim_debug()):
                return True
        return False

    def calibrate(self, dip_run=4):
        """Drives the user's OWN calibration mode (DIP=000): SW1 on all-black, SW2 on all-white,
        then switches the DIP to the run mode. Takes ~1 s of simulated time."""
        self.inputs(0, 0); self.set_sensors(0)
        self._wait(lambda d: (d >> 4) == 0 and (d & 7) == 0)
        for mask, btn in ((0xFF, 1), (0x00, 2)):
            self.set_sensors(mask); self.inputs(btn, 0); self.run(60_000)
            self.inputs(0, 0)
            self._wait(lambda d: (d & 7) == 0)          # the code has finished its delay + beep
        self.inputs(0, dip_run)
        self._wait(lambda d: (d >> 4) == dip_run)

    SERVO_MECH_CENTER = 1472     # servo pulse (us) at which the real wheels point straight
    brake_always = False         # True: brake pins brake even while the PWM is non-zero

    def step(self, mask, buttons, dip, t_ms, dt):
        """One physics tick: feed sensors/buttons, let the user's code run for dt seconds,
        and return what the car's hardware outputs are (signed motor %, steering units)."""
        self.set_sensors(mask)
        self.inputs(buttons, dip)
        o = self.run(round(dt * 1e6))
        arr = max(1, o.arr)
        dl = min(100.0, o.pwm_l * 100.0 / arr)
        dr = min(100.0, o.pwm_r * 100.0 / arr)
        r = types.SimpleNamespace()
        r.speed_l = dl if o.dir_l else -dl
        r.speed_r = dr if o.dir_r else -dr
        r.brake_l = bool(o.brake_l and (self.brake_always or dl == 0))
        r.brake_r = bool(o.brake_r and (self.brake_always or dr == 0))
        r.servo = (o.servo_ccr - self.SERVO_MECH_CENTER) / 10.0 if o.servo_ccr else 0.0
        r.servo_us = o.servo_ccr
        r.servo_center = o.servo_center
        r.runcase, r.running, r.cnt1 = o.runcase, o.running, o.cnt1
        r.tick_ms, r.tim1_us = o.tick_ms, o.tim1_us
        return r
