"""Build and run the car's real C firmware on the PC (firmware-in-the-loop).

The application files (robot.c, calib.c, sensor_debounce.c ...) are compiled together with
simulated drivers from sim/fw_host/ into a shared library, then driven from Python with ctypes.

A C compiler is needed: gcc / clang (MSYS2, MinGW, WinLibs ...), or the easiest on Windows:
    python -m pip install ziglang
"""
import ctypes
import hashlib
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
HOST = HERE / "fw_host"
# check_line.c re-defines functions that robot.c already defines (check_crossline ...), so it
# cannot be part of the same build. Override with --fw-exclude / --fw-include.
DEFAULT_EXCLUDE = {"check_line.c"}


class FirmwareError(Exception):
    pass


def autodetect_dir():
    """<repo>/src/firmware/App next to the sim/ folder."""
    d = HERE.parent / "src" / "firmware" / "App"
    return d if (d / "robot.c").is_file() else None


def find_compiler():
    cands = []
    env = os.environ.get("CC")
    if env:
        cands.append(shlex.split(env))
    for name in ("gcc", "clang", "cc"):
        p = shutil.which(name)
        if p:
            cands.append([p])
    zig = shutil.which("zig")
    if zig:
        cands.append([zig, "cc"])
    try:
        import ziglang  # noqa: F401  (pip install ziglang)
        cands.append([sys.executable, "-m", "ziglang", "cc"])
    except ImportError:
        pass
    return cands[0] if cands else None


def _lib_suffix():
    if os.name == "nt":
        return ".dll"
    return ".dylib" if sys.platform == "darwin" else ".so"


def source_files(fw_dir, exclude):
    files = sorted(p for p in Path(fw_dir).glob("*.c") if p.name not in exclude)
    if not any(p.name == "robot.c" for p in files):
        raise FirmwareError(f"robot.c not found in {fw_dir}")
    return files


def build(fw_dir, exclude=None, extra_includes=()):
    fw_dir = Path(fw_dir).resolve()
    exclude = DEFAULT_EXCLUDE if exclude is None else set(exclude)
    srcs = source_files(fw_dir, exclude) + [HOST / "sim_stubs.c", HOST / "sim_bridge.c"]
    headers = sorted(fw_dir.glob("*.h")) + sorted((HOST / "include").glob("*.h"))

    h = hashlib.sha1()
    for p in srcs + headers:
        st = p.stat()
        h.update(f"{p}|{st.st_size}|{int(st.st_mtime)}".encode())
    h.update(",".join(sorted(exclude)).encode())
    tag = h.hexdigest()[:10]

    out_dir = HERE / ".build"
    try:
        out_dir.mkdir(exist_ok=True)
    except OSError:
        out_dir = Path(tempfile.mkdtemp(prefix="linesim_"))
    out = out_dir / f"sim_fw_{tag}{_lib_suffix()}"
    if out.exists():
        return out

    cc = find_compiler()
    if cc is None:
        raise FirmwareError(
            "No C compiler found. Install one of:\n"
            "  - python -m pip install ziglang        (easiest, no admin needed)\n"
            "  - MSYS2 / WinLibs (gcc), or LLVM (clang), and make sure it is on PATH")
    cmd = list(cc) + ["-shared", "-O1", "-w", "-std=gnu11", "-o", str(out)]
    if os.name != "nt":
        cmd.append("-fPIC")
    elif "gcc" in Path(cc[0]).name.lower():
        cmd.append("-static-libgcc")
    cmd += ["-I", str(HOST / "include"), "-I", str(fw_dir)]
    for inc in extra_includes:
        cmd += ["-I", str(inc)]
    cmd += [str(p) for p in srcs]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        msg = (r.stderr or r.stdout).strip()
        raise FirmwareError(
            "Firmware build failed.\n  command: " + " ".join(cmd[:3]) + " ...\n" + msg + "\n\n"
            "If the error is about a missing function or header, add a small stub for it in\n"
            f"  {HOST / 'include'}  and  {HOST / 'sim_stubs.c'}")
    for old in out_dir.glob("sim_fw_*" + _lib_suffix()):
        if old != out:
            try:
                old.unlink()
            except OSError:
                pass
    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        pass  # directory added at load time
    return out


def parse_runcases(robot_h):
    """{number: NAME} from the `typedef enum {...} run_case;` block of robot.h."""
    try:
        text = Path(robot_h).read_text(errors="replace")
    except OSError:
        return {}
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    m = re.search(r"typedef\s+enum\s*\{(.*?)\}\s*run_case\s*;", text, flags=re.S)
    names = {}
    if m:
        for name, val in re.findall(r"\b([A-Za-z_]\w*)\s*=\s*(\d+)", m.group(1)):
            names[int(val)] = name
    return names


class SimOut(ctypes.Structure):
    _fields_ = [(n, ctypes.c_int32) for n in (
        "servo", "speed_l", "speed_r", "brake_l", "brake_r",
        "runcase", "running", "line_lost", "cnt1", "beeps")]


class Firmware:
    def __init__(self, fw_dir, exclude=None):
        self.fw_dir = Path(fw_dir).resolve()
        self.lib_path = build(self.fw_dir, exclude)
        cc = find_compiler()
        if os.name == "nt" and cc and hasattr(os, "add_dll_directory"):
            exe = shutil.which(cc[0]) or cc[0]
            try:
                os.add_dll_directory(str(Path(exe).resolve().parent))
            except OSError:
                pass
        try:
            self.lib = ctypes.CDLL(str(self.lib_path))
        except OSError as exc:
            raise FirmwareError(f"Could not load {self.lib_path}: {exc}")
        self.lib.sim_init.restype = ctypes.c_int
        self.lib.sim_step.restype = None
        self.lib.sim_step.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
                                      ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(SimOut)]
        self.names = parse_runcases(self.fw_dir / "robot.h")
        self.out = SimOut()
        self.reset()

    def reset(self):
        self.lib.sim_init()

    def step(self, mask, buttons, dip, t_ms, loops=1):
        self.lib.sim_step(mask & 0xFF, buttons & 0xFF, dip & 0xFF, t_ms & 0xFFFFFFFF,
                          loops, ctypes.byref(self.out))
        return self.out

    def runcase_name(self, value):
        return self.names.get(int(value), str(int(value)))
