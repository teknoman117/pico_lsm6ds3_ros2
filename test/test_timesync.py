# SPDX-License-Identifier: Apache-2.0
import random

import pytest

from pico_lsm6ds3_ros2.timesync import WRAP, TimestampMapper

PERIOD_US = 4756          # the measured report period: 8 samples of the sensor's fast clock


def simulate(mapper, n, start_raw=0, offset=1.8e9, drift_ppm=0.0, seed=1, latency_max=1.5e-3,
             start_host_dev=0.0):
    """Feed n reports. Returns [(true host time of the window midpoint, stamp, arrival)]."""
    rng = random.Random(seed)
    out = []
    for i in range(n):
        dev_us = i * PERIOD_US
        raw = (start_raw + dev_us) % WRAP
        dev_s = start_host_dev + dev_us * 1e-6
        true_host = offset + dev_s * (1.0 + drift_ppm * 1e-6)
        latency = 2e-4 + rng.random() * latency_max     # never below 0.2 ms
        arrival = true_host + latency
        stamp = mapper.update(raw, arrival)
        out.append((true_host, stamp, arrival))
    return out


def test_unwrap():
    m = TimestampMapper()
    assert m.unwrap(WRAP - 1000) == 0
    assert m.unwrap(WRAP - 10) == 990
    assert m.unwrap(5) == 1005          # across the 2^31 wrap
    assert m.unwrap(5005) == 6005


def test_unwrap_rejects_backwards_jump():
    m = TimestampMapper()
    m.unwrap(1_000_000)
    assert m.unwrap(500_000) is None    # backwards: a huge step mod 2^31


def test_stamps_follow_device_clock_not_arrival_jitter():
    m = TimestampMapper()
    out = simulate(m, 3000)
    # After the first second, every stamp is within (min latency + a little) of the truth,
    # and the steps are the device's period, not the arrival jitter.
    errs = [s - t for t, s, _ in out[300:]]
    assert max(errs) < 0.25e-3 + 0.05e-3
    assert min(errs) >= 0.0
    steps = [b[1] - a[1] for a, b in zip(out[300:], out[301:])]
    arrival_steps = [b[2] - a[2] for a, b in zip(out[300:], out[301:])]
    spread = max(steps) - min(steps)
    assert spread < 30e-6
    assert max(arrival_steps) - min(arrival_steps) > 1e-3


def test_never_later_than_arrival_and_monotonic():
    m = TimestampMapper()
    out = simulate(m, 2000, drift_ppm=-60.0)
    assert all(s <= a + 1e-9 for _, s, a in out)
    assert all(b[1] > a[1] for a, b in zip(out, out[1:]))


@pytest.mark.parametrize('drift_ppm', [-60.0, 40.0])
def test_tracks_drift(drift_ppm):
    m = TimestampMapper(window=5.0, drift_window=60.0)
    out = simulate(m, int(120 / (PERIOD_US * 1e-6)), drift_ppm=drift_ppm)
    assert m.drift_ppm == pytest.approx(drift_ppm, abs=5.0)
    # Without drift tracking a 5 s minimum lags by up to 5 s * drift = 0.3 ms; with it, the
    # error stays near the minimum latency (0.2 ms).
    errs = [s - t for t, s, _ in out[-12000:]]
    assert max(errs) < 0.35e-3
    assert min(errs) > 0.0


def test_across_wrap():
    m = TimestampMapper()
    out = simulate(m, 2000, start_raw=WRAP - 2_000_000)   # wraps after ~2 s
    errs = [s - t for t, s, _ in out[300:]]
    assert max(errs) < 0.3e-3


def test_restart_on_device_reset():
    m = TimestampMapper()
    simulate(m, 500, start_raw=50_000_000)
    resets = m.resets
    out = simulate(m, 500, start_raw=0, start_host_dev=500 * PERIOD_US * 1e-6, seed=2)
    assert m.resets == resets + 1
    assert abs(out[-1][1] - out[-1][0]) < 2e-3


def test_restart_on_host_clock_step():
    m = TimestampMapper()
    simulate(m, 500)
    resets = m.resets
    # The host clock jumps forward 2 s: arrivals are much later than the stamps.
    out = simulate(m, 500, start_raw=500 * PERIOD_US, offset=1.8e9 + 2.0,
                   start_host_dev=500 * PERIOD_US * 1e-6)
    assert m.resets == resets + 1
    assert abs(out[-1][1] - out[-1][0]) < 2e-3
