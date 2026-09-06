"""
Step 1: Baseline classical RC servo control using stock MicroPython.

No DMA, no PIO -- just machine.PWM's duty_ns(), which is exactly what
it's for: a 50 Hz frame with a 1000-2000 us high pulse is the standard
RC servo position command. This project's SAMD21 build already knows
how to *decode* such a pulse as a command input
(DCServoCommunicationHandlerWithPwmInterface, see the "How to switch
to PWM interface version" section of the top-level README); here the
RP2040 generates that same kind of pulse train instead, to drive a
stock RC servo directly.

Once duty_ns() is set, the PWM slice free-runs in hardware and holds
the position with no CPU involvement -- the limitation this baseline
has is only during a *sweep*: moving smoothly between two positions
means a blocking Python loop calling duty_ns() repeatedly, which busy
-waits the CPU for the sweep's duration. Fine for one axis; won't
scale to running 6 independent sweeps at once (steps 2 and 3).

Wiring (RP2040W / Pico W):
  Servo signal -> GPIO2. Servo power (5-6V) from a separate BEC/
  supply, NOT the Pico's 3V3/VBUS -- servos can draw well beyond what
  the Pico's regulator can supply under load. Share ground between
  the Pico and the servo supply. Avoid GPIO 23/24/25/29 anywhere in
  this project: they are wired to the Pico W's on-board wireless chip.

Run with: mpremote run step1_basic_pwm.py
"""
from machine import Pin, PWM
import time
from motion_profile import linear_profile

SERVO_GPIO = 2
PWM_FREQ_HZ = 50
MIN_PULSE_US = 1000
MAX_PULSE_US = 2000


class BasicServoAxis:
    def __init__(self, gpio, pwm_freq=PWM_FREQ_HZ):
        self._pwm = PWM(Pin(gpio), freq=pwm_freq)

    def set_pulse_us(self, pulse_us):
        pulse_us = max(MIN_PULSE_US, min(MAX_PULSE_US, pulse_us))
        self._pwm.duty_ns(pulse_us * 1000)


def main():
    axis = BasicServoAxis(SERVO_GPIO)
    profile = linear_profile(MIN_PULSE_US, MAX_PULSE_US, duration_s=1.0, tick_hz=100)
    print("Sweeping %d..%d us over %d ticks..." % (MIN_PULSE_US, MAX_PULSE_US, len(profile)))
    for pulse_us in profile:
        axis.set_pulse_us(pulse_us)
        time.sleep_ms(10)
    print("Done, holding final position (hardware PWM keeps it with no CPU help).")


if __name__ == "__main__":
    main()
