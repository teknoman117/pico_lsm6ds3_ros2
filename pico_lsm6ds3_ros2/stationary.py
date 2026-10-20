# SPDX-License-Identifier: Apache-2.0
"""Stationary detection and the gyro bias estimate. No ROS here."""

import math
from collections import deque


class ImuStationaryDetector:
    """Stationary when, over the last `window` seconds, every gyro and accel axis's standard
    deviation is below its threshold and (once a bias is known) every gyro axis's mean is within
    `gyro_mean` of the bias.

    The mean check catches slow, steady rotation, which has no more variance than rest.
    """

    def __init__(self, window=1.0, gyro_std=0.01, accel_std=0.05, gyro_mean=0.01):
        self.window = window
        self.gyro_var = gyro_std ** 2
        self.accel_var = accel_std ** 2
        self.gyro_mean = gyro_mean
        self.samples = deque()
        self._sums()

    def _sums(self):
        self.s = [0.0] * 6
        self.ss = [0.0] * 6
        for _, x in self.samples:
            for i in range(6):
                self.s[i] += x[i]
                self.ss[i] += x[i] * x[i]
        self.updates = 0

    def reset(self):
        self.samples.clear()
        self._sums()

    def update(self, t, gyro, accel, bias=None):
        """t: seconds. Returns whether the device is stationary now."""
        x = tuple(gyro) + tuple(accel)
        samples = self.samples
        if samples and t < samples[-1][0]:
            self.reset()  # time went backwards (a new time mapping)
        samples.append((t, x))
        for i in range(6):
            self.s[i] += x[i]
            self.ss[i] += x[i] * x[i]
        while samples[0][0] < t - self.window:
            _, old = samples.popleft()
            for i in range(6):
                self.s[i] -= old[i]
                self.ss[i] -= old[i] * old[i]
        self.updates += 1
        if self.updates >= 4096:
            self._sums()  # drop the rounding error the running sums collect
        return self.stationary(bias)

    def full(self):
        """The window holds (nearly) `window` seconds of samples."""
        s = self.samples
        return len(s) >= 2 and s[-1][0] - s[0][0] >= 0.9 * self.window

    def mean(self, i):
        return self.s[i] / len(self.samples)

    def var(self, i):
        n = len(self.samples)
        m = self.s[i] / n
        return max(0.0, self.ss[i] / n - m * m)

    def stationary(self, bias=None):
        if not self.full():
            return False
        for i in range(3):
            if self.var(i) > self.gyro_var or self.var(3 + i) > self.accel_var:
                return False
            if bias is not None and abs(self.mean(i) - bias[i]) > self.gyro_mean:
                return False
        return True


class OdomStationaryDetector:
    """Stationary when the odometry's twist has been near zero for `hold` seconds and the last
    message is at most `timeout` old. Times are host seconds."""

    def __init__(self, linear=0.005, angular=0.01, hold=1.0, timeout=0.5):
        self.linear = linear
        self.angular = angular
        self.hold = hold
        self.timeout = timeout
        self.still_since = None
        self.last_msg = None

    def update(self, t, vx, vy, wz):
        self.last_msg = t
        still = math.hypot(vx, vy) <= self.linear and abs(wz) <= self.angular
        if not still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = t

    def stationary(self, t):
        if self.last_msg is None or self.still_since is None:
            return False
        if t - self.last_msg > self.timeout:
            return False
        return t - self.still_since >= self.hold


class GyroBiasEstimator:
    """The gyro bias: averaged over `init_duration` seconds of continuous rest at startup, then
    followed while stationary with a first-order filter of time constant `time_constant`
    (0 disables the updates)."""

    def __init__(self, init_duration=2.5, time_constant=10.0):
        self.init_duration = init_duration
        self.time_constant = time_constant
        self.bias = [0.0, 0.0, 0.0]
        self.initialized = False
        self._clear()

    def _clear(self):
        self.acc = [0.0, 0.0, 0.0]
        self.count = 0
        self.start = None
        self.last_t = None

    def update(self, t, gyro, stationary):
        """Returns True when this sample completed the initial estimate."""
        if not self.initialized:
            if not stationary:
                self._clear()
                return False
            if self.start is None:
                self.start = t
            for i in range(3):
                self.acc[i] += gyro[i]
            self.count += 1
            if t - self.start >= self.init_duration and self.count > 1:
                self.bias = [a / self.count for a in self.acc]
                self.initialized = True
                self.last_t = t
                return True
            return False

        last, self.last_t = self.last_t, t
        if not stationary or self.time_constant <= 0.0 or last is None:
            return False
        dt = min(max(t - last, 0.0), 0.1)
        k = dt / self.time_constant
        for i in range(3):
            self.bias[i] += k * (gyro[i] - self.bias[i])
        return False
