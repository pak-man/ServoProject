"""
Step 2: DMA-paced PWM sweep for a single RC servo axis.

Builds on step1_basic_pwm.py: the RP2040's hardware PWM slice still
generates the 50 Hz servo pulse train, but the per-tick pulse-width
updates that step 1 did in a blocking Python loop are now generated
by the DMA controller, writing a precomputed sweep profile straight
into the PWM slice's compare (CC) register. The CPU only has to start
the transfer; it is then free for the rest of the sweep. When the
buffer runs out, the slice keeps outputting the last CC value it was
given -- exactly like step 1's "hold with no CPU help" -- so no
special end-of-move handling is needed here either.

The DMA is paced by the *same* slice's own wrap DREQ: since RC servos
only care about one pulse per 20 ms frame, updating the target once
per frame (i.e. at the PWM's own 50 Hz) is exactly the right rate --
no separate timing-only "pacer" slice is needed here (contrast a
faster switching waveform, which would need the update rate decoupled
from the switching rate).

Converting a desired pulse width in nanoseconds to a raw CC count
needs the slice's actual clock divider, which is read back from the
hardware register instead of assumed, since MicroPython's internal
frequency-to-divider choice for a given freq=50 isn't otherwise
exposed. Register layout (PWM_CHx_DIV INT/FRAC fields, CC offset,
DREQ_PWM_WRAP* numbering) comes from the RP2040 datasheet section
4.5.2 (PWM) and the DREQ table in section 2.5.3. Requires MicroPython
>= 1.21 (rp2.DMA was added then).

Wiring: same servo as step 1, signal -> GPIO2.

Run with: mpremote run step2_dma_pwm.py
"""
import array
import time

from machine import Pin, PWM, mem32
from rp2 import DMA

from motion_profile import linear_profile

SERVO_GPIO = 2
PWM_FREQ_HZ = 50
MIN_PULSE_US = 1000
MAX_PULSE_US = 2000
TICK_HZ = 50  # one update per 20 ms servo frame

PWM_BASE = 0x40050000
_SLICE_STRIDE = 0x14
_CC_OFFSET = 0x0C
_DIV_OFFSET = 0x04
_DREQ_PWM_WRAP0 = 24  # RP2040 datasheet table 124: DREQ_PWM_WRAP0..7 = 24..31
_SYSCLK_HZ = 125_000_000  # MicroPython's default RP2040 clk_sys


def _slice_of(gpio):
    return (gpio >> 1) & 0x7


def _channel_of(gpio):
    return gpio & 0x1  # 0 = A (low half of CC), 1 = B (high half)


def _cc_addr(slice_num):
    return PWM_BASE + slice_num * _SLICE_STRIDE + _CC_OFFSET


def _read_divider(slice_num):
    raw = mem32[PWM_BASE + slice_num * _SLICE_STRIDE + _DIV_OFFSET]
    integer = (raw >> 4) & 0xFF
    frac = raw & 0xF
    if integer == 0:
        integer = 256  # datasheet 4.5.2.2: DIV_INT == 0 behaves as a divide by 256
    return integer + frac / 16.0


class DmaServoAxis:
    def __init__(self, gpio, pwm_freq=PWM_FREQ_HZ):
        self._pwm = PWM(Pin(gpio), freq=pwm_freq)
        self._slice = _slice_of(gpio)
        self._channel = _channel_of(gpio)
        self._tick_ns = _read_divider(self._slice) * 1e9 / _SYSCLK_HZ
        self._dreq = _DREQ_PWM_WRAP0 + self._slice
        self._dma = DMA()
        self._buf = None

    def _pack(self, pulse_us_samples):
        # CC register: bits[15:0] = channel A compare, bits[31:16] = channel B.
        buf = array.array("I", [0] * len(pulse_us_samples))
        shift = 16 if self._channel else 0
        for i, pulse_us in enumerate(pulse_us_samples):
            counts = int(pulse_us * 1000 / self._tick_ns)
            buf[i] = counts << shift
        return buf

    def play(self, pulse_us_samples, blocking=True):
        self._buf = self._pack(pulse_us_samples)
        self._dma.config(
            read=self._buf,
            write=_cc_addr(self._slice),
            count=len(self._buf),
            ctrl=self._dma.pack_ctrl(
                size=2,  # 32-bit transfers (both CC channels written each time)
                inc_read=True,
                inc_write=False,  # always the same CC register
                treq_sel=self._dreq,
            ),
            trigger=True,
        )
        if blocking:
            while self._dma.active():
                time.sleep_ms(1)

    def stop(self):
        self._dma.active(0)


def main():
    axis = DmaServoAxis(SERVO_GPIO)
    profile = linear_profile(MIN_PULSE_US, MAX_PULSE_US, duration_s=1.0, tick_hz=TICK_HZ)
    print("Playing %d samples via DMA..." % len(profile))
    axis.play(profile)
    print("Sweep complete, holding last position (hardware PWM, no CPU help).")
    axis.stop()


if __name__ == "__main__":
    main()
