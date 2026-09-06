"""Shared open-loop motion helpers for the RP2040W servo steps.

No closed-loop feedback yet: these just produce a commanded duty-cycle
vs. time table (a simple trapezoidal speed profile) that steps 2 and 3
play back through DMA. `split_signed` mirrors the sign/magnitude split
used by the existing ServoProject H-bridge driver
(HBridge2WirePwm::setOutput in ArduinoSketch/PwmHandler.cpp): a
positive command drives one leg, a negative command drives the other,
and the idle leg is held at 0.
"""


def split_signed(duty, max_duty=1023):
    """Clamp duty to +/-max_duty and split it into (leg_neg, leg_pos)
    magnitudes, exactly one of which is non-zero."""
    if duty > max_duty:
        duty = max_duty
    elif duty < -max_duty:
        duty = -max_duty
    if duty >= 0:
        return 0, duty
    return -duty, 0


def trapezoid_profile(peak_duty, accel_s, cruise_s, decel_s, tick_hz):
    """Return a list of signed duty samples (one per 1/tick_hz seconds)
    ramping linearly up to peak_duty, holding, then ramping back down.
    peak_duty may be negative to run the move in reverse."""
    n_accel = max(1, round(accel_s * tick_hz))
    n_cruise = max(0, round(cruise_s * tick_hz))
    n_decel = max(1, round(decel_s * tick_hz))

    profile = [round(peak_duty * (i + 1) / n_accel) for i in range(n_accel)]
    profile.extend([peak_duty] * n_cruise)
    profile.extend(round(peak_duty * (n_decel - i - 1) / n_decel) for i in range(n_decel))
    return profile
