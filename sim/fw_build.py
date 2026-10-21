"""Build the user's WHOLE src/ tree (per the Keil project) into a PC shared library."""
import hashlib, os, re, shlex, shutil, subprocess, sys, tempfile
from pathlib import Path
from fw_bridge import find_compiler, _lib_suffix, FirmwareError

HERE = Path(__file__).resolve().parent
HOST = HERE / "fw_host2"
VENDOR_HAL = "stm32f1xx_hal"           # vendor HAL .c files are replaced by the shim


def find_root(p):
    """Directory that holds MDK-ARM/*.uvprojx (the 'firmware' folder)."""
    p = Path(p).resolve()
    for c in [p, p / "firmware", p / "src" / "firmware", p.parent, p.parent.parent] + list(p.glob("*/firmware")):
        if list(c.glob("MDK-ARM/*.uvprojx")):
            return c
    return None


def autodetect():
    return find_root(HERE.parent / "src") or find_root(HERE.parent)


def parse_project(root):
    px = next(root.glob("MDK-ARM/*.uvprojx"))
    t = px.read_text(encoding="utf8", errors="replace")
    base = px.parent
    srcs = []
    for name, typ, path in re.findall(r"<FileName>(.*?)</FileName>\s*<FileType>(\d+)</FileType>\s*<FilePath>(.*?)</FilePath>", t):
        if typ != "1":
            continue
        f = (base / path.replace("\\", "/")).resolve()
        if "Drivers" in f.parts or not f.is_file():
            continue
        srcs.append(f)
    inc = []
    for m in re.findall(r"<IncludePath>(.*?)</IncludePath>", t):
        for part in m.split(";"):
            if part:
                inc.append((base / part.replace("\\", "/")).resolve())
    defs = []
    for m in re.findall(r"<Define>(.*?)</Define>", t):
        defs += [d for d in re.split(r"[,\s]+", m) if d]
    return px, srcs, inc, defs


def has_real_main(main_c):
    t = Path(main_c).read_text(errors="replace")
    t = re.sub(r"/\*.*?\*/", "", t, flags=re.S)
    t = re.sub(r"//[^\n]*", "", t)
    return bool(re.search(r"\brobot_init\s*\(", t) and re.search(r"\bloop\s*\(", t))


def alias_dir(srcs, inc, out):
    """Case-insensitive include forwarding (servo.c includes "Servo.h")."""
    allh = {}
    for d in inc:
        for h in d.glob("*.h"):
            allh.setdefault(h.name.lower(), h)
    made = []
    for s in srcs:
        for h in re.findall(r'#\s*include\s+"([^"]+)"', s.read_text(errors="replace")):
            if any((d / h).is_file() for d in inc + [s.parent]):
                continue
            real = allh.get(Path(h).name.lower())
            if real:
                out.mkdir(parents=True, exist_ok=True)
                (out / h).write_text(f'#include "{real.as_posix()}"\n')
                made.append(h)
    return made


def build(root, exclude=(), force_glue=None):
    root = Path(root)
    px, srcs, inc, defs = parse_project(root)
    srcs = [s for s in srcs if s.name not in set(exclude)]
    main_c = next((s for s in srcs if s.name == "main.c"), None)
    if main_c is None:
        raise FirmwareError("main.c is not in the Keil project")
    glue = (not has_real_main(main_c)) if force_glue is None else force_glue
    others = [s for s in srcs if s != main_c]
    files = others + [HOST / "sim_core.c", HOST / "sim_os.c", HOST / "sim_bridge.c", HOST / "sim_main_wrap.c"]
    hdrs = [p for d in inc for p in d.glob("*.h") if "Drivers" not in p.parts] + list(HOST.glob("*.h"))

    h = hashlib.sha1()
    for p in files + [main_c] + hdrs:
        st = p.stat()
        h.update(f"{p}|{st.st_size}|{int(st.st_mtime)}".encode())
    h.update(",".join(sorted(exclude)).encode())
    tag = h.hexdigest()[:10]
    out_dir = HERE / ".build"
    try:
        out_dir.mkdir(exist_ok=True)
    except OSError:
        out_dir = Path(tempfile.mkdtemp(prefix="linesim_"))
    out = out_dir / f"sim_full_{tag}{_lib_suffix()}"
    if out.exists():
        return out, glue, srcs

    cc = find_compiler()
    if cc is None:
        raise FirmwareError("No C compiler found. Run:  python -m pip install ziglang")
    ali = out_dir / f"inc_{tag}"
    alias_dir(others + [main_c], inc, ali)
    cmd = list(cc) + ["-shared", "-O1", "-w", "-std=gnu11", "-fno-strict-aliasing", "-o", str(out)]
    if os.name != "nt":
        cmd += ["-fPIC", "-fvisibility=default"]
    cmd += ["-include", str(HOST / "sim_prelude.h")]
    cmd += [f"-D{d}" for d in defs]
    cmd += [f'-DSIM_MAIN_FILE="{main_c.as_posix()}"']
    for d in [ali] + inc + [main_c.parent]:
        cmd += ["-I", str(d)]
    cmd += [str(f) for f in files]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise FirmwareError("Build of your src/ failed:\n" + (r.stderr or r.stdout).strip()[:4000])
    for old in out_dir.glob("inc_*"):
        if old != ali:
            shutil.rmtree(old, ignore_errors=True)
    for old in out_dir.glob("sim_full_*" + _lib_suffix()):
        if old != out and "_run" not in old.name:
            try: old.unlink()
            except OSError: pass
    return out, glue, srcs
