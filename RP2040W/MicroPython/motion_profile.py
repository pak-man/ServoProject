"""Shared open-loop motion helper for the RP2040W RC servo steps.

No closed-loop feedback yet: this just produces a commanded
pulse-width vs. time table (a linear sweep between two positions)
that steps 2 and 3 play back through DMA instead of jumping the servo
straight to a new position.
"""


def linear_profile(start_us, end_us, duration_s, tick_hz):
    """Return a list of pulse widths in microseconds (one per
    1/tick_hz seconds) ramping linearly from start_us to end_us."""
    n = max(1, round(duration_s * tick_hz))
    return [round(start_us + (end_us - start_us) * i / n) for i in range(n + 1)]
