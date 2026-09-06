"""
Step 1: Baseline open-loop H-bridge control using stock MicroPython.

No DMA, no PIO -- just machine.PWM, mirroring the two-pin H-bridge
drive scheme already used for the ItsyBitsy/SAMD21 build
(HBridge2WirePwm in ArduinoSketch/PwmHandler.cpp): one pin is PWM'd
for "forward", the other for "reverse", and both are held at 0 to
coast.

This establishes that the wiring and the H-bridge itself work, and is
the baseline the later DMA/PIO steps are measured against: the CPU is
fully occupied for the whole move (a plain Python loop), which is
fine for one axis but won't scale to 6 axes moving independently.

Wiring (RP2040W / Pico W):
  H-bridge (e.g. DRV8833, TB6612FNG) IN1 -> GPIO2, IN2 -> GPIO3.
  Motor supply comes from the H-bridge's own VM pin, NOT from the
  Pico's 3V3/VBUS. Share ground between the Pico and the H-bridge.
  Avoid GPIO 23/24/25/29 anywhere in this project: they are wired to
  the Pico W's on-board wireless chip.

Run with: mpremote run step1_basic_pwm.py
"""
from machine import Pin, PWM
import time
from motion_profile import trapezoid_profile, split_signed

MOTOR_GPIO_A = 2
MOTOR_GPIO_B = 3
PWM_FREQ_HZ = 20000  # matches the 20 kHz default used in ArduinoSketch/config/default.h
MAX_DUTY = 1023  # keeps the same 10-bit command range as the existing firmware


class BasicServoAxis:
    def __init__(self, gpio_a, gpio_b, pwm_freq=PWM_FREQ_HZ):
        self._pwm_a = PWM(Pin(gpio_a), freq=pwm_freq, duty_u16=0)
        self._pwm_b = PWM(Pin(gpio_b), freq=pwm_freq, duty_u16=0)

    def set_output(self, duty):
        mag_a, mag_b = split_signed(duty, MAX_DUTY)
        self._pwm_a.duty_u16(mag_a * 65535 // MAX_DUTY)
        self._pwm_b.duty_u16(mag_b * 65535 // MAX_DUTY)

    def coast(self):
        self.set_output(0)


def main():
    axis = BasicServoAxis(MOTOR_GPIO_A, MOTOR_GPIO_B)
    profile = trapezoid_profile(peak_duty=600, accel_s=0.5, cruise_s=1.0, decel_s=0.5, tick_hz=100)
    print("Running blocking ramp, %d ticks..." % len(profile))
    for duty in profile:
        axis.set_output(duty)
        time.sleep_ms(10)
    axis.coast()
    print("Done, coasting.")


if __name__ == "__main__":
    main()
