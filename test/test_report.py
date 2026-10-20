# SPDX-License-Identifier: Apache-2.0
import math

import pytest

from pico_lsm6ds3_ros2.report import (
    ABS_MISC, ABS_RUDDER, ABS_RX, ABS_RY, ABS_RZ, ABS_THROTTLE, ABS_WHEEL, ABS_X, ABS_Y, ABS_Z,
    EV_ABS, EV_SYN, FLAG_ERROR, FLAG_SAMPLE_MISSED, SYN_DROPPED, SYN_REPORT, WRAP,
    ReportAssembler, Scaler, SequenceTracker, describe_flags)


def test_gyro_scaling():
    s = Scaler()  # 8 samples, 17.50 mdps/LSB
    # 8 samples of 1000 counts = 17.5 dps
    (x, y, z) = s.gyro((8000, -8000, 0))
    assert x == pytest.approx(math.radians(17.5))
    assert y == pytest.approx(-math.radians(17.5))
    assert z == 0.0


def test_accel_scaling_one_g():
    s = Scaler()  # 0.122 mg/LSB: 1 g = 8197 counts, 65574 in a sum of 8
    (_, _, z) = s.accel((0, 0, 8 * 8197))
    assert z == pytest.approx(9.80665, rel=1e-3)


def test_custom_scaling():
    s = Scaler(samples_per_report=4, gyro_mdps_per_lsb=70.0, accel_mg_per_lsb=0.488)
    assert s.gyro((4,))[0] == pytest.approx(math.radians(0.070))
    assert s.accel((4,))[0] == pytest.approx(0.488e-3 * 9.80665)


def test_temperature():
    assert Scaler(temperature_lsb_per_degc=16.0).temperature(-80) == pytest.approx(20.0)
    assert Scaler(temperature_lsb_per_degc=256.0).temperature(512) == pytest.approx(27.0)


def test_bad_divisor():
    with pytest.raises(ValueError):
        Scaler(samples_per_report=0)


def packet(asm, values, t):
    for code, value in values.items():
        assert asm.feed(EV_ABS, code, value, t) is None
    return asm.feed(EV_SYN, SYN_REPORT, 0, t)


def test_assembler_keeps_unchanged_values():
    asm = ReportAssembler({ABS_X: 1, ABS_Y: 2, ABS_Z: 3, ABS_WHEEL: 0})
    r = packet(asm, {ABS_RX: 10, ABS_RY: 20, ABS_RZ: 30, ABS_MISC: 1000, ABS_RUDDER: 5,
                     ABS_THROTTLE: -7}, 1.5)
    assert r.gyro == (10, 20, 30)
    assert r.accel == (1, 2, 3)          # never sent: the initial values
    assert (r.timestamp, r.sequence, r.temperature, r.flags) == (1000, 5, -7, 0)
    assert r.host_time == 1.5
    # Only the timestamp, sequence and gyro x change: the rest keep their last values.
    r = packet(asm, {ABS_RX: 11, ABS_MISC: 5800, ABS_RUDDER: 6}, 1.6)
    assert r.gyro == (11, 20, 30)
    assert r.accel == (1, 2, 3)
    assert (r.timestamp, r.sequence, r.temperature) == (5800, 6, -7)


def test_assembler_skips_dropped_packet():
    asm = ReportAssembler()
    asm.feed(EV_ABS, ABS_RX, 5, 0.0)
    assert asm.feed(EV_SYN, SYN_DROPPED, 0, 0.0) is None
    assert asm.dropped
    asm.feed(EV_ABS, ABS_RX, 6, 0.0)
    assert asm.feed(EV_SYN, SYN_REPORT, 0, 0.0) is None   # the packet after SYN_DROPPED
    assert not asm.dropped
    asm.resync({ABS_RX: 9})
    assert packet(asm, {ABS_MISC: 1}, 0.1).gyro[0] == 9


def test_assembler_ignores_other_codes():
    asm = ReportAssembler()
    assert asm.feed(EV_ABS, 0x10, 99, 0.0) is None      # ABS_HAT0X: not ours
    assert asm.feed(1, 30, 1, 0.0) is None              # EV_KEY
    assert packet(asm, {}, 0.0).gyro == (0, 0, 0)


def test_sequence_gaps_and_wrap():
    seq = SequenceTracker()
    assert seq.update(10) == 0
    assert seq.update(11) == 0
    assert seq.update(14) == 2
    assert (seq.gaps, seq.lost) == (1, 2)
    seq.reset()
    seq.update(WRAP - 2)
    assert seq.update(WRAP - 1) == 0
    assert seq.update(0) == 0                  # wraps mod 2^31
    assert seq.update(2) == 1


def test_sequence_restart():
    seq = SequenceTracker()
    seq.update(5000)
    assert seq.update(0) == 0                  # backwards: the Pico restarted, not a gap
    assert seq.restarts == 1
    assert seq.lost == 0


def test_describe_flags():
    assert describe_flags(0) == ''
    assert 'sample missed' in describe_flags(FLAG_SAMPLE_MISSED)
    assert 'sensor error' in describe_flags(FLAG_ERROR | 0x80)
    assert 'unknown' in describe_flags(0x80)
