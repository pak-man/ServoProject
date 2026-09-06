"""
Step 3: PIO-generated complementary H-bridge PWM, still fed by DMA.

Hardware PWM slices (step 2) have no dead-time / break-before-make
support, so a fast direction reversal can momentarily overlap the two
H-bridge legs. This moves waveform generation into a small PIO
program that drives both legs from a single state machine (via
side-set) and inserts a fixed dead-time gap every switching period.
That also frees the hardware PWM slices entirely and only costs one
of the RP2040's 8 PIO state machines per axis -- 6 axes fit in 6 of
the 8 SMs (2 blocks x 4 SMs each), which the 8 PWM slices used two at
a time (step 2) would not have allowed.

PIO program, one switching period:
    dead time (both legs low)
    -> drive leg A (GPIO_BASE)   high for `x` cycles (0 if unused)
    -> drive leg B (GPIO_BASE+1) high for `y` cycles (0 if unused)
Exactly one of x/y is non-zero per the HBridge2WirePwm-style command
split (see motion_profile.split_signed), so the two legs are never
driven at once. A fresh (x, y) pair is autopulled from the TX FIFO
once every period (2x `out ... ,16` = 32 bits = pull_thresh); DMA
keeps that FIFO fed from a precomputed profile, paced by the state
machine's own TX DREQ, so a finished/underrun move simply stalls the
state machine rather than glitching the outputs.

Note: unlike step 2's separate pacer slice, here the switching period
IS the motion-profile sample period. PIO only has two scratch
registers (x, y) and both are consumed by the dead-time/on-time
counters every period, so there is no spare register to "hold last
value" the way a single-channel PWM idiom can (see the classic
pico-examples pwm.pio, which relies on `pull(noblock)` falling back
to scratch x for exactly that reason). A 5 kHz nominal switching rate
keeps the profile buffer small while staying a normal brushed-motor
drive frequency; because the loop length depends on whichever of x/y
is active, the *actual* switching frequency varies somewhat with
commanded duty (from ~5 kHz near full duty to a few times that near
zero) -- fine for open-loop torque/speed control, not a fixed-carrier
PWM.

Register-level bits (PIO base addresses/offsets, TX DREQ numbering)
come from the RP2040 datasheet: section 3 (PIO) and the DREQ table in
section 2.5.3.

Wiring: same as steps 1/2, IN1 -> GPIO2, IN2 -> GPIO3. The two pins
must be consecutive GPIOs: PIO side-set claims a contiguous pin block.

Run with: mpremote run step3_pio_servo.py
"""
import array
import time

from machine import Pin
from rp2 import PIO, StateMachine, DMA, asm_pio

from motion_profile import trapezoid_profile, split_signed

MOTOR_GPIO_BASE = 2  # GPIO2 = leg A (side-set bit 0), GPIO3 = leg B (side-set bit 1)
MAX_DUTY = 1023
DEAD_TIME_CYCLES = 16
PIO_CLKDIV = 24.0  # 125 MHz / 24 ~= 5.2 MHz PIO clock -> ~5 kHz nominal switching period
SWITCH_HZ_NOMINAL = 5000

PIO0_BASE = 0x50200000
PIO1_BASE = 0x50300000
_TXF0_OFFSET = 0x10


def _pio_txf_addr(pio_id, sm_num):
    base = PIO0_BASE if pio_id == 0 else PIO1_BASE
    return base + _TXF0_OFFSET + 4 * sm_num


def _pio_tx_dreq(pio_id, sm_num):
    return pio_id * 8 + sm_num


@asm_pio(
    sideset_init=(PIO.OUT_LOW, PIO.OUT_LOW),
    out_shiftdir=PIO.SHIFT_RIGHT,
    autopull=True,
    pull_thresh=32,
)
def hbridge_pwm():
    pull(block).side(0)
    wrap_target()
    out(x, 16).side(0)[DEAD_TIME_CYCLES - 1]
    out(y, 16).side(0)
    jmp(not_x, "skip_a").side(0)
    label("delay_a")
    jmp(x_dec, "delay_a").side(1)
    label("skip_a")
    jmp(not_y, "skip_b").side(0)
    label("delay_b")
    jmp(y_dec, "delay_b").side(2)
    label("skip_b")
    wrap()


class PioServoAxis:
    def __init__(self, gpio_base, sm_id=0, pio_id=0, clkdiv=PIO_CLKDIV):
        self._pio_id = pio_id
        self._sm_id = sm_id
        self._sm = StateMachine(
            pio_id * 4 + sm_id,
            hbridge_pwm,
            freq=int(125_000_000 / clkdiv),
            sideset_base=Pin(gpio_base),
        )
        self._sm.active(1)
        self._dma = DMA()
        self._buf = None

    @staticmethod
    def _pack(duty_samples):
        buf = array.array("I", [0] * len(duty_samples))
        for i, d in enumerate(duty_samples):
            leg_a, leg_b = split_signed(d, MAX_DUTY)
            buf[i] = (leg_b << 16) | leg_a
        return buf

    def play(self, duty_samples, blocking=True):
        self._buf = self._pack(duty_samples)
        self._dma.config(
            read=self._buf,
            write=_pio_txf_addr(self._pio_id, self._sm_id),
            count=len(self._buf),
            ctrl=self._dma.pack_ctrl(
                size=2,  # 32-bit transfers
                inc_read=True,
                inc_write=False,  # always the same TX FIFO register
                treq_sel=_pio_tx_dreq(self._pio_id, self._sm_id),
            ),
            trigger=True,
        )
        if blocking:
            while self._dma.active():
                time.sleep_ms(1)

    def stop(self):
        self._dma.active(0)
        self._sm.active(0)


def main():
    axis = PioServoAxis(MOTOR_GPIO_BASE)
    profile = trapezoid_profile(
        peak_duty=600, accel_s=0.5, cruise_s=1.0, decel_s=0.5, tick_hz=SWITCH_HZ_NOMINAL
    )
    print("Playing %d samples via DMA -> PIO..." % len(profile))
    axis.play(profile)
    print("Move complete. Stopping state machine.")
    axis.stop()


if __name__ == "__main__":
    main()
