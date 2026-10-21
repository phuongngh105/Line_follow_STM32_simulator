# Line Follow STM32 Simulator

A Python simulator for testing a line-following car with either the built-in PID demo or STM32 firmware.

## Install

```powershell Cái này phải chạy để dịch từ c của STM32 sang Python/This is must run
python -m pip install pygame-ce ziglang
```

## Run

From this directory, start the simulator in PID demo mode:

```powershell
python sim\simulator.py
```

To build and run a firmware project, pass the directory containing `MDK-ARM\*.uvprojx`:

```powershell
python sim\simulator.py --firmware "D:\path\to\src\firmware"
```

The firmware source is compiled for the PC by Zig; the simulator does not use a prebuilt STM32 `.hex` or `.bin` file. See [sim/README.md](sim/README.md) for details and options.

Run the headless checks with:

```powershell
python sim\simulator.py --selftest
```
