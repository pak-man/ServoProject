"""
Step 3: one PIO state machine driving up to 6 RC servos, DMA-refreshed
with no CPU involvement between commands.

Steps 1/2 use one hardware PWM slice per servo, which already scales
comfortably to 6 axes on its own (up to 16 channels across 8 slices).
The reason to move to PIO here isn't running out of PWM slices -- it's
consolidating all 6 axes onto a *single* state machine and a *single*
DMA channel with frame-synchronised edges, freeing the other 7 PWM
slices and 11 DMA channels for whatever else a 6-axis controller needs
(current sensing, status LEDs, a 7th input, ...), which also mirrors
how a real multi-channel motor/servo PIO driver (e.g. pybricks' RP2040
port) consolidates channels onto few state machines.

Design: rc_servo_pio drives 6 consecutive GPIOs from one word per
event (out(pins, 6)), each followed by a delay count (out(x, 26)) --
see servo_frame.py for how a full 20 ms frame's pulses are turned into
a short list of (mask, delay) events. Unlike a hardware PWM slice,
this PIO program does *not* hold its output for free once fed: it
actively re-pulls a new event every time one finishes, so if the DMA
ever stopped mid-frame the state machine would simply stall. To get
the same "set once, keeps going" behaviour as steps 1/2, the one DMA
channel here uses the RP2040 DMA's read-side ring buffer: the frame
buffer is padded to a fixed 8-word (32-byte) power-of-two region and
the transfer count is set enormous, so DMA re-reads that same 8-word
frame forever, re-triggering the PIO once per 20 ms, until
set_pulses() is called again to swap in a new frame.

Wiring: GPIO2..GPIO7 = channel 0..5 (only channel 0 / GPIO2 has a
servo attached for this test; the rest are simply inactive/low).

Run with: mpremote run step3_pio_servo.py
"""
import array
import time

from machine import Pin
from rp2 import PIO, StateMachine, DMA, asm_pio

from motion_profile import linear_profile
from servo_frame import build_frame, pad_frame, NUM_CHANNELS

SERVO_GPIO_BASE = 2
MIN_PULSE_US = 1000
MAX_PULSE_US = 2000
PIO_FREQ_HZ = 1_000_000  # 1 PIO cycle = 1 us, matches servo_frame.py's time base
FRAME_WORDS = 8  # next power of two >= (NUM_CHANNELS events + 1 final segment)

PIO0_BASE = 0x50200000
PIO1_BASE = 0x50300000
_TXF0_OFFSET = 0x10


def _pio_txf_addr(pio_id, sm_num):
    base = PIO0_BASE if pio_id == 0 else PIO1_BASE
    return base + _TXF0_OFFSET + 4 * sm_num


def _pio_tx_dreq(pio_id, sm_num):
    return pio_id * 8 + sm_num


@asm_pio(
    out_init=(PIO.OUT_LOW,) * NUM_CHANNELS,
    out_shiftdir=PIO.SHIFT_RIGHT,
    autopull=True,
    pull_thresh=32,
)
def rc_servo_pio():
    pull(block)
    wrap_target()
    out(pins, NUM_CHANNELS)
    out(x, 26)
    label("delay")
    jmp(x_dec, "delay")
    wrap()


class PioServoAxes:
    def __init__(self, gpio_base, sm_id=0, pio_id=0):
        self._pio_id = pio_id
        self._sm_id = sm_id
        self._sm = StateMachine(
            pio_id * 4 + sm_id,
            rc_servo_pio,
            freq=PIO_FREQ_HZ,
            out_base=Pin(gpio_base),
        )
        self._sm.active(1)
        self._dma = DMA()
        self._buf = None

    def set_pulses(self, pulses_us):
        """pulses_us: list of up to NUM_CHANNELS pulse widths in us;
        missing/0 entries are inactive channels."""
        self._dma.active(0)
        words = pad_frame(build_frame(pulses_us), FRAME_WORDS)
        self._buf = array.array("I", words)
        self._dma.config(
            read=self._buf,
            write=_pio_txf_addr(self._pio_id, self._sm_id),
            count=0xFFFFFFFF,  # effectively forever, until the next set_pulses()/stop()
            ctrl=self._dma.pack_ctrl(
                size=2,  # 32-bit transfers
                inc_read=True,
                inc_write=False,  # always the same TX FIFO register
                ring_sel=True,  # ring applies to the read address...
                ring_size=5,  # ...wrapping every 2**5 = 32 bytes = FRAME_WORDS
                treq_sel=_pio_tx_dreq(self._pio_id, self._sm_id),
            ),
            trigger=True,
        )

    def stop(self):
        self._dma.active(0)
        self._sm.active(0)


def main():
    axes = PioServoAxes(SERVO_GPIO_BASE)
    profile = linear_profile(MIN_PULSE_US, MAX_PULSE_US, duration_s=1.0, tick_hz=50)
    print("Sweeping channel 0 over %d frames via PIO + ring-buffered DMA..." % len(profile))
    for pulse_us in profile:
        axes.set_pulses([pulse_us])
        time.sleep_ms(20)
    print("Done. Channel 0 keeps refreshing at 50 Hz with no further CPU help.")
    time.sleep(2)
    axes.stop()


if __name__ == "__main__":
    main()
