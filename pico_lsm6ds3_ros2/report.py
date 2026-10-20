# SPDX-License-Identifier: Apache-2.0
"""The firmware's report as it arrives over evdev, and its conversion to SI units.

No ROS and no device here, so the tests can run anywhere.

Firmware: https://github.com/teknoman117/pico-lsm6ds3-hid (src/report.c, src/report.h). Each
HID report becomes one evdev packet: an EV_ABS event for every field whose value changed,
then SYN_REPORT. The input core sends nothing for an unchanged field, so the assembler keeps
the last value of every field and emits a complete report at each SYN_REPORT.
"""

import math
from dataclasses import dataclass

# evdev event types and codes (linux/input-event-codes.h)
EV_SYN = 0x00
EV_ABS = 0x03
SYN_REPORT = 0
SYN_DROPPED = 3

ABS_X = 0x00
ABS_Y = 0x01
ABS_Z = 0x02
ABS_RX = 0x03
ABS_RY = 0x04
ABS_RZ = 0x05
ABS_THROTTLE = 0x06
ABS_RUDDER = 0x07
ABS_WHEEL = 0x08
ABS_MISC = 0x28

# The firmware's mapping (README "Using it on Linux"; the HID usages in src/report.c).
CODE_GYRO = (ABS_RX, ABS_RY, ABS_RZ)       # Generic Desktop Rx/Ry/Rz: gyro sums
CODE_ACCEL = (ABS_X, ABS_Y, ABS_Z)         # Generic Desktop X/Y/Z: accel sums
CODE_TIMESTAMP = ABS_MISC                  # Sensor page Timestamp: window midpoint, us mod 2^31
CODE_SEQUENCE = ABS_RUDDER                 # Generic Desktop Dial: +1 per report, mod 2^31
CODE_TEMPERATURE = ABS_THROTTLE            # Generic Desktop Slider: OUT_TEMP raw
CODE_FLAGS = ABS_WHEEL                     # Generic Desktop Wheel: REPORT_FLAG_*
ALL_CODES = CODE_GYRO + CODE_ACCEL + (CODE_TIMESTAMP, CODE_SEQUENCE, CODE_TEMPERATURE,
                                      CODE_FLAGS)

# src/report.h
FLAG_SAMPLE_MISSED = 0x01  # a sample gap > 1.5 periods in this window
FLAG_USB_BUSY = 0x02       # an earlier report was replaced before it was sent
FLAG_ERROR = 0x04          # sensor missing, unknown or failed read-back; data is 0

WRAP = 1 << 31             # timestamp and sequence wrap

# src/config.h: IMU_AVERAGE_N, IMU_GY_FS (+/-500 dps), IMU_XL_FS (+/-4 g)
DEFAULT_SAMPLES_PER_REPORT = 8
DEFAULT_GYRO_MDPS_PER_LSB = 17.50
DEFAULT_ACCEL_MG_PER_LSB = 0.122
STANDARD_GRAVITY = 9.80665

# Temperature: 25 degC + raw / sensitivity (firmware README)
TEMPERATURE_LSB_PER_DEGC = {'LSM6DS3': 16.0, 'LSM6DS3TR-C': 256.0}


@dataclass
class RawReport:
    """One firmware report, as integers."""

    gyro: tuple       # sums of N raw samples, sensor axes
    accel: tuple      # sums of N raw samples, sensor axes
    timestamp: int    # Pico us, window midpoint, mod 2^31
    sequence: int     # mod 2^31
    temperature: int  # OUT_TEMP raw (last sample of the window)
    flags: int
    host_time: float  # kernel timestamp of the SYN_REPORT, seconds (CLOCK_REALTIME)


class ReportAssembler:
    """Turns a stream of evdev events into complete reports.

    initial: the current value of every field (from EVIOCGABS), so a field that never changes
    after opening the device still has its value.
    """

    def __init__(self, initial=None):
        self.values = {code: 0 for code in ALL_CODES}
        if initial:
            self.values.update({c: v for c, v in initial.items() if c in self.values})
        self.dropped = False

    def feed(self, etype, code, value, host_time):
        """Feed one event. Returns a RawReport at SYN_REPORT, else None."""
        if etype == EV_ABS:
            if code in self.values:
                self.values[code] = value
            return None
        if etype != EV_SYN:
            return None
        if code == SYN_DROPPED:
            # The kernel's buffer overflowed: events up to the next SYN_REPORT are lost and
            # the values are stale. Skip that packet; the caller resyncs from EVIOCGABS.
            self.dropped = True
            return None
        if code != SYN_REPORT:
            return None
        if self.dropped:
            self.dropped = False
            return None
        v = self.values
        return RawReport(
            gyro=tuple(v[c] for c in CODE_GYRO),
            accel=tuple(v[c] for c in CODE_ACCEL),
            timestamp=v[CODE_TIMESTAMP],
            sequence=v[CODE_SEQUENCE],
            temperature=v[CODE_TEMPERATURE],
            flags=v[CODE_FLAGS],
            host_time=host_time,
        )

    def resync(self, values):
        """Replace the stored values (after SYN_DROPPED), from EVIOCGABS."""
        self.values.update({c: v for c, v in values.items() if c in self.values})


class Scaler:
    """Sums of raw counts to SI units: sum / N * sensitivity."""

    def __init__(self, samples_per_report=DEFAULT_SAMPLES_PER_REPORT,
                 gyro_mdps_per_lsb=DEFAULT_GYRO_MDPS_PER_LSB,
                 accel_mg_per_lsb=DEFAULT_ACCEL_MG_PER_LSB,
                 temperature_lsb_per_degc=16.0):
        if samples_per_report <= 0:
            raise ValueError('samples_per_report must be positive')
        self.gyro_scale = gyro_mdps_per_lsb * 1e-3 * math.pi / 180.0 / samples_per_report
        self.accel_scale = accel_mg_per_lsb * 1e-3 * STANDARD_GRAVITY / samples_per_report
        self.temperature_lsb_per_degc = temperature_lsb_per_degc

    def gyro(self, sums):
        """rad/s"""
        return tuple(s * self.gyro_scale for s in sums)

    def accel(self, sums):
        """m/s^2"""
        return tuple(s * self.accel_scale for s in sums)

    def temperature(self, raw):
        """degC"""
        return 25.0 + raw / self.temperature_lsb_per_degc


class SequenceTracker:
    """Counts reports lost between consecutive sequence numbers (mod 2^31)."""

    # A step this large is the counter going backwards (the Pico restarted), not a gap.
    MAX_GAP = 1 << 30

    def __init__(self):
        self.last = None
        self.lost = 0
        self.gaps = 0
        self.restarts = 0

    def reset(self):
        self.last = None

    def update(self, sequence):
        """Returns the number of reports missing before this one (0 normally)."""
        missing = 0
        if self.last is not None:
            step = (sequence - self.last) % WRAP
            if step == 0:
                missing = 0   # a repeat; not expected from the firmware
            elif step > self.MAX_GAP:
                self.restarts += 1
            else:
                missing = step - 1
        self.last = sequence
        if missing:
            self.gaps += 1
            self.lost += missing
        return missing


def describe_flags(flags):
    names = []
    if flags & FLAG_SAMPLE_MISSED:
        names.append('sample missed')
    if flags & FLAG_USB_BUSY:
        names.append('report dropped (USB busy)')
    if flags & FLAG_ERROR:
        names.append('sensor error')
    rest = flags & ~(FLAG_SAMPLE_MISSED | FLAG_USB_BUSY | FLAG_ERROR)
    if rest:
        names.append(f'unknown flags 0x{rest:02x}')
    return ', '.join(names)
