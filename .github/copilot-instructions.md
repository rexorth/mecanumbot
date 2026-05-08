# Copilot Instructions

## Project Overview

MecanumBot is a bare-metal C firmware for a mecanum-wheeled robot running on a **Raspberry Pi Pico 2 (RP2350)**. It uses the **Pico SDK** and **TinyUSB** in USB host mode to read Xbox controller input via the `tusb_xinput` submodule, then drives four stepper motors through a PWM-based interrupt control loop.

## Task Planning and Problem Solving

- Before each task, you must first complete the following steps:
  1. Provide a full plan of your changes.
  2. Provide a list of behaviors that you'll change.
- Before you add any code, always check if you can just re-use
  or re-configure any existing code to achieve the result.
- Always focus on simplicity and precision and not comprehensiveness.
- When writing tests, focus on the happy path and only the most
  important edge cases.
- Before adding a new test, always make sure that a similar test
  doesn't exist already.
  
## Build

This is a CMake project targeting the RP2350. The Pico SDK must be available (set `PICO_SDK_PATH` or enable `PICO_SDK_FETCH_FROM_GIT`).

```sh
mkdir -p build && cd build
cmake ..
make
```

The output binary is `build/MecanumBot.uf2` (or `.elf` for debug).

## Debug & Serial

- UART0 on GPIO 0/1 at 115200 baud is used for stdio (`pico_enable_stdio_uart` is on, USB stdio is off).
- Serial monitor: `sudo minicom -D /dev/ttyACM0 -b 115200`
- OpenOCD for debug: `sudo src/openocd -s tcl -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000"`
- VS Code is configured with cortex-debug and cmake-tools for on-target debugging.

## Architecture

### Control Loop

A **PWM interrupt** (`pwm_wrap_isr`) fires every `CONTROL_TICK_US` (200µs) and calls `control_tick()` in `main.c`. This is the real-time motor control path — it must remain fast and non-blocking. The control tick:

1. Computes per-wheel speeds from the current `MotionInput` (x, y, rotation) using mecanum kinematics
2. Normalizes speeds so no wheel exceeds `MAX_WHEEL_SPEED`
3. Accumulates time per wheel and toggles step/direction GPIO pins when the step delay elapses

### Input Pipeline

Xbox controller → TinyUSB host task (`tuh_task()` in main loop) → `tuh_xinput_report_received_cb` → `handle_gamepad_input()` → updates global `currentInput` (MotionInput struct with x, y, w fields).

The left stick controls translation (x/y), the right stick X-axis controls rotation (w). A deadzone of ±1500 is applied before normalizing to m/s.

### Motor Control

Each wheel is a `WheelState` struct holding speed, step delay, accumulated time, direction, and GPIO pin assignments. Stepper motors are driven by toggling step pins at computed intervals. `default_direction` encodes the physical mounting orientation (left wheels vs right wheels spin opposite directions for forward motion).

**GPIO pin mapping:**
- Front Left: Step=3, Dir=2
- Rear Left: Step=5, Dir=4
- Front Right: Step=7, Dir=6
- Rear Right: Step=9, Dir=8
- PWM control timer: GPIO 15

### Source Files

- `main.c` — All robot logic: kinematics, motor control, gamepad handling, initialization
- `hid_app.c`, `cdc_app.c`, `msc_app.c` — TinyUSB host class callbacks (from TinyUSB examples, mostly boilerplate)
- `tusb_xinput/` — Git submodule providing the Xbox controller USB host driver
- `tusb_config.h` — TinyUSB configuration (host mode, XInput enabled)

## Conventions

- C11 standard, compiled with `-O0 -g` (debug build)
- Physical constants (wheel diameter, track radius, steps per revolution) are `#define` macros in `main.c`
- Logging uses TinyUSB's `TU_LOG1()` macro (outputs over UART)
- No RTOS — bare-metal with a single main loop calling `tuh_task()` and an ISR for motor control
- Wheel speed units are meters per second throughout
