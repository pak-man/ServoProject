"""
Step 2: DMA-paced PWM playback for a single H-bridge axis.

Builds on step1_basic_pwm.py: the RP2040's hardware PWM slice still
generates the 20 kHz motor switching waveform, but the per-tick duty
updates that step 1 did in a blocking Python loop are now generated
by the DMA controller, writing a precomputed motion profile straight
into the PWM slice's compare (CC) register. The CPU only has to start
the transfer; it is then free for the rest of the move, which is what
lets several axes move at once later on.

A second, otherwise-unused PWM slice is free-run purely as a timing
source: its wrap event fires a DREQ at the motion-profile update rate
(TICK_HZ), decoupling "how often the commanded duty changes" from
"how fast the H-bridge switches" (PWM_FREQ_HZ). Requires MicroPython
>= 1.21 (rp2.DMA was added then).

Register-level bits (PWM CC register layout/offsets, DREQ_PWM_WRAP*
numbers) come from the RP2040 datasheet: section 4.5.2 (PWM) and the
DREQ table in section 2.5.3.

Wiring: same H-bridge as step 1, IN1 -> GPIO2, IN2 -> GPIO3 (PWM
slice 1, channels A/B: slice = (gpio >> 1) & 7, channel = gpio & 1).
Pacer output (GPIO6) does not need to be physically connected.

Run with: mpremote run step2_dma_pwm.py
"""
from machine import Pin, PWM, mem32
from rp2 import DMA
import time
from motion_profile import trapezoid_profile, split_signed

MOTOR_GPIO_A = 2
MOTOR_GPIO_B = 3
PACER_GPIO = 6
PWM_FREQ_HZ = 20000
TICK_HZ = 1000  # motion-profile update rate
MAX_DUTY = 1023

PWM_BASE = 0x40050000
_SLICE_STRIDE = 0x14
_CC_OFFSET = 0x0C
_TOP_OFFSET = 0x10
_DREQ_PWM_WRAP0 = 24  # RP2040 datasheet table 124: DREQ_PWM_WRAP0..7 = 24..31


def _slice_of(gpio):
    return (gpio >> 1) & 0x7


def _cc_addr(slice_num):
    return PWM_BASE + slice_num * _SLICE_STRIDE + _CC_OFFSET


def _read_top(slice_num):
    return mem32[PWM_BASE + slice_num * _SLICE_STRIDE + _TOP_OFFSET]


class DmaServoAxis:
    def __init__(self, gpio_a, gpio_b, pacer_gpio, pwm_freq=PWM_FREQ_HZ, tick_hz=TICK_HZ):
        if _slice_of(gpio_a) != _slice_of(gpio_b) or (gpio_a & 1) == (gpio_b & 1):
            raise ValueError("gpio_a/gpio_b must be the A/B pair of one PWM slice")

        self._pwm_a = PWM(Pin(gpio_a), freq=pwm_freq, duty_u16=0)
        self._pwm_b = PWM(Pin(gpio_b), freq=pwm_freq, duty_u16=0)
        self._slice = _slice_of(gpio_a)
        self._a_is_low_half = (gpio_a & 1) == 0
        self._top = _read_top(self._slice)  # actual compare range chosen by MicroPython for pwm_freq

        # Free-running slice used only for its wrap DREQ.
        self._pacer = PWM(Pin(pacer_gpio), freq=tick_hz, duty_u16=0)
        self._pacer_dreq = _DREQ_PWM_WRAP0 + _slice_of(pacer_gpio)

        self._dma = DMA()
        self._buf = None

    def _pack(self, duty_samples):
        # CC register: bits[15:0] = channel A compare, bits[31:16] = channel B.
        import array
        buf = array.array("I", [0] * len(duty_samples))
        for i, d in enumerate(duty_samples):
            mag_neg, mag_pos = split_signed(d, MAX_DUTY)
            lo, hi = (mag_neg, mag_pos) if self._a_is_low_half else (mag_pos, mag_neg)
            lo = lo * self._top // MAX_DUTY
            hi = hi * self._top // MAX_DUTY
            buf[i] = (hi << 16) | lo
        return buf

    def play(self, duty_samples, blocking=True):
        self._buf = self._pack(duty_samples)
        self._dma.config(
            read=self._buf,
            write=_cc_addr(self._slice),
            count=len(self._buf),
            ctrl=self._dma.pack_ctrl(
                size=2,  # 32-bit transfers
                inc_read=True,
                inc_write=False,  # always the same CC register
                treq_sel=self._pacer_dreq,
            ),
            trigger=True,
        )
        if blocking:
            while self._dma.active():
                time.sleep_ms(1)

    def stop(self):
        self._dma.active(0)
        self._pwm_a.duty_u16(0)
        self._pwm_b.duty_u16(0)


def main():
    axis = DmaServoAxis(MOTOR_GPIO_A, MOTOR_GPIO_B, PACER_GPIO)
    profile = trapezoid_profile(peak_duty=600, accel_s=0.5, cruise_s=1.0, decel_s=0.5, tick_hz=TICK_HZ)
    print("Playing %d samples via DMA..." % len(profile))
    axis.play(profile)
    print("Move complete, holding last duty. Stopping.")
    axis.stop()


if __name__ == "__main__":
    main()
