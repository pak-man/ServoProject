"""Builds one 20 ms multi-servo frame as a sorted edge list for the
rc_servo_pio program in step3_pio_servo.py.

Rather than one PWM slice per axis, a single PIO state machine can
hold up to 6 servo outputs high or low simultaneously (out(pins, 6)
drives 6 consecutive GPIOs from one word) and switch them at
arbitrary times within the shared 20 ms frame. So instead of a duty
value per axis, the state machine is fed a short sequence of
(pin_mask, delay) events: which pins are high, and for how long,
until the next pin changes state.

Word format consumed by the PIO program (32 bits, LSB first):
  bits[5:0]  = pin mask (bit i = GPIO base+i is high)
  bits[31:6] = delay in PIO clock cycles until the next event
"""

FRAME_US = 20000
NUM_CHANNELS = 6
_EVENT_OVERHEAD_CYCLES = 3  # 2x `out` + 1 extra `jmp` cycle at the end of each countdown, see step3_pio_servo.py


def build_frame(pulses_us, frame_us=FRAME_US, num_channels=NUM_CHANNELS):
    """pulses_us: list of up to num_channels pulse widths in
    microseconds; 0 or None means that channel is inactive (held low
    all frame). One PIO clock cycle is assumed to be 1 us (see
    PIO_CLKDIV in step3_pio_servo.py). Returns a list of packed
    32-bit words forming one repeating frame."""
    pulses = (list(pulses_us) + [0] * num_channels)[:num_channels]
    pulses = [min(p, frame_us - 1) if p else 0 for p in pulses]

    events = sorted(set(p for p in pulses if p > 0))
    mask = sum(1 << i for i, p in enumerate(pulses) if p > 0)

    words = []
    t = 0
    for t_next in events:
        words.append(_pack(mask, t_next - t))
        mask &= ~sum(1 << i for i, p in enumerate(pulses) if p == t_next)
        t = t_next
    words.append(_pack(mask, frame_us - t))
    return words


def _pack(mask, delay_us):
    delay_cycles = max(0, delay_us - _EVENT_OVERHEAD_CYCLES)
    return ((delay_cycles & 0x3FFFFFF) << 6) | (mask & 0x3F)


def pad_frame(words, size):
    """Pad a build_frame() result to exactly `size` words by splitting
    its final (mask=0) low segment across the extra slots, so the
    total frame duration is unchanged. Needed because DMA's read-side
    ring-buffer wraparound (used in step3_pio_servo.py for zero-CPU
    continuous replay) requires a power-of-two-sized region."""
    if len(words) > size:
        raise ValueError("frame needs %d words, more than size=%d" % (len(words), size))
    if len(words) == size:
        return words
    *head, last = words
    # last's delay already had _EVENT_OVERHEAD_CYCLES subtracted once; recover
    # the true remaining microseconds before splitting it, so each of the new
    # fill words can have that same per-event overhead subtracted again.
    total_us = (last >> 6) + _EVENT_OVERHEAD_CYCLES
    n_fill = size - len(head)
    base, extra = divmod(total_us, n_fill)
    return head + [_pack(0, base + (1 if i < extra else 0)) for i in range(n_fill)]
