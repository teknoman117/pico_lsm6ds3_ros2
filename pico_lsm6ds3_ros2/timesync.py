# SPDX-License-Identifier: Apache-2.0
"""Maps the Pico's microsecond timestamps to host (ROS) time.

Each report carries the midpoint of its sampling window on the Pico's crystal clock, in us,
mod 2^31. The host knows when the report arrived (the kernel's evdev timestamp, on the same
clock as ROS time). For every report

    offset = host_receive_time - device_time = true offset + latency

where the latency (USB polling, at most 1 ms, plus the half window and scheduling) is never
negative and varies from report to report. Its minimum is the least delayed report, so the
lower envelope of the offsets is the clock offset plus a constant (the shortest latency). The
two clocks also run at slightly different rates (tens of ppm), so the envelope is a line, not a
constant:

    1. Unwrap the 2^31 wrap of the device time.
    2. Track the minimum offset over a sliding window (`window`, a few seconds), and the minimum
       of each `block` (1 s) over a longer `drift_window` (a minute).
    3. With enough blocks, fit a line (offset vs. device time) through the block minima by least
       squares for the slope (the drift), then lower it until no recent offset lies below it
       (the envelope). Until then, use the sliding-window minimum.
    4. stamp = device_time + offset estimate. A report that arrives earlier than the estimate
       allows lowers the estimate at once, so a stamp is never later than its arrival.

The stamps are then as smooth as the Pico's clock: the arrival jitter does not reach them.
"""

from collections import deque

WRAP = 1 << 31


class _SlidingMin:
    """Minimum of (t, value) pairs over the last `span` of t (monotonic deque)."""

    def __init__(self, span):
        self.span = span
        self.items = deque()

    def clear(self):
        self.items.clear()

    def push(self, t, value):
        items = self.items
        while items and items[-1][1] >= value:
            items.pop()
        items.append((t, value))
        while items[0][0] < t - self.span:
            items.popleft()

    def min(self):
        return self.items[0] if self.items else None


class TimestampMapper:
    """
    window: seconds, the sliding minimum used until the drift is known.
    drift_window: seconds of block minima the drift fit uses.
    block: seconds per block minimum.
    min_blocks: block minima needed before fitting the drift.
    max_step: seconds; a device-time jump larger than this (or backwards) restarts the mapping
        (the Pico restarted, or a long outage).
    max_latency: seconds; an arrival this much later than its stamp restarts the mapping (the
        host clock stepped forward, or the estimate is stale).
    """

    def __init__(self, window=5.0, drift_window=60.0, block=1.0, min_blocks=5,
                 max_step=1.0, max_latency=0.5):
        self.window = window
        self.drift_window = drift_window
        self.block = block
        self.min_blocks = min_blocks
        self.max_step_us = int(max_step * 1e6)
        self.max_latency = max_latency
        self.resets = 0            # restarts forced by the data (not reset() calls)
        self.reset()

    def _restart(self):
        self.resets += 1
        self.reset()

    def reset(self):
        """Forget everything (a new connection)."""
        self.last_raw = None
        self.device_us = 0          # unwrapped device time since the first report, us
        self.host0 = None           # host time of the first report (keeps floats small)
        self.recent = _SlidingMin(self.window)
        self.blocks = deque()       # (device s, offset s) minima, one per block
        self.block_start = None
        self.block_min = None
        self.slope = 0.0            # drift: d(offset)/d(device time)
        self.intercept = None       # offset at device time 0, seconds
        self.last_stamp = None
        self.latency = 0.0          # host receive time - stamp, last report
        self.count = 0

    @property
    def drift_ppm(self):
        return self.slope * 1e6

    def offset_at(self, device_s):
        """The estimated host - device offset at a device time (seconds since the first
        report), relative to host0."""
        return self.intercept + self.slope * device_s

    def unwrap(self, raw):
        """Unwrapped device time in us since the first report, or None after a restart."""
        if self.last_raw is None:
            self.last_raw = raw
            return 0
        step = (raw - self.last_raw) % WRAP
        if step > self.max_step_us:
            return None
        self.last_raw = raw
        self.device_us += step
        return self.device_us

    def update(self, raw_timestamp, host_time):
        """raw_timestamp: the report's timestamp field (us mod 2^31). host_time: arrival, s.

        Returns the stamp in host seconds. After a restart (see max_step, max_latency) the
        mapping starts again from this report."""
        if self.host0 is None:
            self.host0 = host_time
        dev_us = self.unwrap(raw_timestamp)
        if dev_us is None:
            self._restart()
            return self.update(raw_timestamp, host_time)
        dev = dev_us * 1e-6
        host = host_time - self.host0
        offset = host - dev
        self.count += 1

        self.recent.push(dev, offset)
        self._block(dev, offset)

        if self.intercept is None:
            self.intercept = offset
        est = self.offset_at(dev)
        if offset < est:
            # Arrived earlier than the estimate allows: lower the line through this report.
            self.intercept = offset - self.slope * dev
            est = offset

        stamp = dev + est
        if host - stamp > self.max_latency:
            # The host clock stepped forward (or the estimate went stale): start again.
            self._restart()
            return self.update(raw_timestamp, host_time)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            stamp = self.last_stamp + 1e-6
        self.last_stamp = stamp
        self.latency = host - stamp
        return self.host0 + stamp

    def _block(self, dev, offset):
        if self.block_start is None:
            self.block_start = dev
        if self.block_min is None or offset < self.block_min[1]:
            self.block_min = (dev, offset)
        if dev - self.block_start < self.block:
            return
        self.blocks.append(self.block_min)
        while self.blocks and self.blocks[0][0] < dev - self.drift_window:
            self.blocks.popleft()
        self.block_start = dev
        self.block_min = None
        self._fit()

    def _fit(self):
        """Re-estimate the line once per block."""
        if len(self.blocks) < self.min_blocks:
            # Not enough history for a slope: the sliding minimum, as a constant.
            self.slope = 0.0
            self.intercept = self.recent.min()[1]
            return
        n = len(self.blocks)
        mt = sum(t for t, _ in self.blocks) / n
        mo = sum(o for _, o in self.blocks) / n
        stt = sum((t - mt) ** 2 for t, _ in self.blocks)
        if stt <= 0.0:
            return
        slope = sum((t - mt) * (o - mo) for t, o in self.blocks) / stt
        # Lower envelope: the line with this slope that no recent offset lies below. The
        # sliding window's minimum keeps it anchored to the last few seconds.
        candidates = list(self.blocks)[-max(1, int(self.window / self.block)):]
        candidates.append(self.recent.min())
        self.slope = slope
        self.intercept = min(o - slope * t for t, o in candidates)
