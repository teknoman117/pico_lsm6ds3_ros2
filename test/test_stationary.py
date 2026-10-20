# SPDX-License-Identifier: Apache-2.0
import random

from pico_lsm6ds3_ros2.stationary import (GyroBiasEstimator, ImuStationaryDetector,
                                          OdomStationaryDetector)

DT = 1.0 / 208.0
BIAS = (0.026, -0.149, 0.003)


def rest(rng, t, rate=(0.0, 0.0, 0.0), accel_noise=0.01):
    gyro = tuple(b + r + rng.gauss(0.0, 0.003) for b, r in zip(BIAS, rate))
    accel = (rng.gauss(0, accel_noise), rng.gauss(0, accel_noise),
             9.81 + rng.gauss(0, accel_noise))
    return gyro, accel


def test_imu_detector_rest_and_motion():
    rng = random.Random(0)
    d = ImuStationaryDetector(window=1.0)
    t = 0.0
    results = []
    for i in range(int(3 / DT)):
        g, a = rest(rng, t)
        results.append(d.update(t, g, a))
        t += DT
    assert not results[0]                  # the window is not full yet
    assert all(results[int(1.0 / DT) + 1:])
    # A bump: a sharp gyro and accel spike.
    g, a = rest(rng, t)
    assert not d.update(t, (g[0] + 1.0, g[1], g[2]), (a[0] + 3.0, a[1], a[2]))
    t += DT
    # It stays non-stationary until the bump has left the window.
    still = []
    for i in range(int(2 / DT)):
        g, a = rest(rng, t)
        still.append(d.update(t, g, a))
        t += DT
    assert not any(still[:int(0.9 / DT)])
    assert still[-1]


def test_imu_detector_slow_steady_turn():
    # A slow steady turn has no more variance than rest: only the mean check sees it.
    rng = random.Random(1)
    d = ImuStationaryDetector(window=1.0, gyro_mean=0.01)
    t = 0.0
    for i in range(int(2 / DT)):
        g, a = rest(rng, t, rate=(0.0, 0.0, 0.05))
        no_bias = d.update(t, g, a)
        t += DT
    assert no_bias                         # without a bias reference: looks stationary
    assert not d.stationary(BIAS)          # with one: turning at 0.05 rad/s


def test_odom_detector():
    d = OdomStationaryDetector(linear=0.005, angular=0.01, hold=1.0, timeout=0.5)
    assert not d.stationary(0.0)
    t = 0.0
    while t < 0.9:
        d.update(t, 0.0, 0.0, 0.0)
        t += 0.02
    assert not d.stationary(t)             # not still for 1 s yet
    while t < 1.5:
        d.update(t, 0.001, 0.0, 0.002)
        t += 0.02
    assert d.stationary(t)
    assert not d.stationary(t + 1.0)       # odometry stopped arriving
    d.update(t, 0.2, 0.0, 0.0)             # moving
    assert not d.stationary(t)


def test_bias_estimator_startup_and_tracking():
    rng = random.Random(2)
    b = GyroBiasEstimator(init_duration=2.5, time_constant=5.0)
    t = 0.0
    done_at = None
    for i in range(int(3 / DT)):
        g, _ = rest(rng, t)
        if b.update(t, g, True):
            done_at = t
        t += DT
    assert b.initialized and 2.5 <= done_at < 2.6
    assert all(abs(x - y) < 0.001 for x, y in zip(b.bias, BIAS))
    # The bias shifts by 0.01 rad/s on z; at rest the estimate follows.
    for i in range(int(30 / DT)):
        g, _ = rest(rng, t)
        b.update(t, (g[0], g[1], g[2] + 0.01), True)
        t += DT
    assert abs(b.bias[2] - (BIAS[2] + 0.01)) < 0.001
    # Not stationary: the estimate holds.
    held = list(b.bias)
    for i in range(200):
        b.update(t, (1.0, 1.0, 1.0), False)
        t += DT
    assert b.bias == held


def test_bias_estimator_restarts_on_motion():
    rng = random.Random(3)
    b = GyroBiasEstimator(init_duration=2.5)
    t = 0.0
    for i in range(int(2 / DT)):
        b.update(t, rest(rng, t)[0], True)
        t += DT
    b.update(t, (0.5, 0.5, 0.5), False)    # moved before 2.5 s of rest
    t += DT
    for i in range(int(2 / DT)):
        b.update(t, rest(rng, t)[0], True)
        t += DT
    assert not b.initialized               # needs 2.5 s of continuous rest
