# RP2040W (Pico W) servo control

Experimental, open-loop-only control for classical RC servos (single
wire, 50 Hz frame, 1000-2000 us pulse) from a Raspberry Pi Pico W,
built as a progression of three MicroPython scripts of increasing
efficiency. There is no encoder feedback or position control loop
here yet -- this is purely about generating and scaling the servo
pulse signal as efficiently as possible, as a foundation for closing
the loop later and for driving 6 independent axes.

## Prerequisites

* A Raspberry Pi Pico W flashed with an official MicroPython build
  **>= 1.21** (`rp2.DMA` was added in that release). Get it from
  https://micropython.org/download/RPI_PICO_W/.
* `mpremote` (`pip install mpremote`) or Thonny to copy files onto the
  board and run them.
* A standard hobby RC servo.

## Wiring (all three steps)

* Servo signal -> Pico GPIO2 (step 3 also reserves GPIO3-GPIO7 for up
  to 5 more axes, unconnected for now).
* Servo power (typically 5-6V) from a separate supply/BEC, **not**
  the Pico's 3V3 or VBUS -- servos can draw well beyond what the
  Pico's own regulator can supply under load.
* Share ground between the Pico and the servo supply.
* Do not use GPIO 23, 24, 25 or 29 anywhere in this project -- they
  are wired to the Pico W's on-board CYW43 wireless chip.

## Running each step

```
mpremote cp RP2040W/MicroPython/motion_profile.py :
mpremote cp RP2040W/MicroPython/step1_basic_pwm.py :
mpremote run RP2040W/MicroPython/step1_basic_pwm.py
```

Step 2 also needs `motion_profile.py`. Step 3 needs `servo_frame.py`
instead. Copy the relevant files alongside each step before running
it. All three sweep the same servo from 1000 to 2000 us over ~1 second,
then hold.

## The three steps

| Step | File | Mechanism | What it demonstrates |
|---|---|---|---|
| 1 | `step1_basic_pwm.py` | `machine.PWM.duty_ns()`, updated in a blocking Python loop | Confirms the servo and wiring work, using only what stock MicroPython already provides for RC-style pulses. Holding a position is free once set (hardware PWM keeps outputting it); only a *sweep* busy-waits the CPU. |
| 2 | `step2_dma_pwm.py` | `machine.PWM` + `rp2.DMA` | A precomputed sweep is written straight into the PWM slice's compare register by DMA, paced by that same slice's own 50 Hz wrap. CPU starts the sweep and is free until it's done; the slice then keeps holding the last position exactly like step 1. |
| 3 | `step3_pio_servo.py` | `rp2.PIO` (custom program) + `rp2.DMA` (read-side ring buffer) | One state machine drives up to 6 servos (`GPIO2..GPIO7`) from a single small per-frame event list; one DMA channel replays that frame forever via ring-buffer wraparound until a new position is commanded. Consolidates 6 axes onto 1 SM + 1 DMA channel instead of 6 PWM slices + 6 DMA channels. |

Each step is a strict superset in capability of the previous one; run
them in order. This project's existing SAMD21 firmware already knows
how to *decode* an RC-style pulse as a command input
(`DCServoCommunicationHandlerWithPwmInterface`, see the top-level
README's "How to switch to PWM interface version"); these scripts
generate that same kind of pulse as output instead, to drive a stock
servo directly from the RP2040.

## Scaling to 6 axes

* Steps 1/2: one PWM slice + one GPIO per axis. The RP2040 has 8
  slices (16 channels), so 6 axes fit easily, each with its own DMA
  channel for step 2.
* Step 3: `rc_servo_pio` already drives 6 consecutive GPIOs
  (`out(pins, 6)`) from one state machine; `servo_frame.build_frame()`
  already accepts up to 6 pulse widths. Nothing needs to change to go
  from 1 to 6 axes here -- just pass a 6-element list to
  `set_pulses()`. A second controller board's worth of headroom is
  left over: 6 of the 8 PIO state machines and 1 of 12 DMA channels
  are used, leaving 2 SMs and 11 DMA channels free.

## Known limitations

* Open loop only: nothing here reads back servo position.
* Step 3's per-event instruction overhead (~3 PIO cycles) is measured
  and compensated for exactly (see `servo_frame.py`), but that
  assumes the 1 MHz PIO clock (`PIO_FREQ_HZ`) is left unchanged.
* None of the three steps persist across a board reset or survive a
  Ctrl-D soft reset cleanly mid-move; call `.stop()` (or power-cycle)
  before starting a new instance if a previous run was interrupted.
