# SPDX-License-Identifier: Apache-2.0
"""ROS 2 driver for the Pico + LSM6DS3 USB HID IMU (evdev).

Publishes sensor_msgs/Imu on imu/data_raw (no orientation), sensor_msgs/Temperature on
imu/temperature and diagnostics on /diagnostics. See README.md.
"""

import errno
import glob
import os
import select
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Temperature

from .report import (ALL_CODES, EV_ABS, FLAG_ERROR, FLAG_SAMPLE_MISSED, FLAG_USB_BUSY,
                     TEMPERATURE_LSB_PER_DEGC, ReportAssembler, Scaler, SequenceTracker,
                     describe_flags)
from .stationary import GyroBiasEstimator, ImuStationaryDetector, OdomStationaryDetector
from .timesync import TimestampMapper

DEVICE_GLOB = '/dev/input/by-id/usb-2e8a_LSM6DS3*-event-*'
STATIONARY_SOURCES = ('imu', 'odom', 'both')

# Measured on the first device (E46788D3671F3E29), at rest on a desk, two 60 s runs at ~210 Hz
# (rounded up between them): the variance of
# each axis (per report, after averaging 8 samples). See README, "Covariance".
DEFAULT_GYRO_VARIANCE = [3.2e-06, 1.0e-05, 2.7e-06]        # (rad/s)^2
DEFAULT_ACCEL_VARIANCE = [1.6e-04, 1.2e-04, 1.2e-04]       # (m/s^2)^2


def find_device(path):
    """The configured path, or the first LSM6DS3 under /dev/input/by-id."""
    if path:
        return path
    found = sorted(glob.glob(DEVICE_GLOB))
    return found[0] if found else None


def to_time_msg(seconds):
    ns = int(round(seconds * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class ImuDriver(Node):

    def __init__(self):
        super().__init__('pico_lsm6ds3_driver')
        p = self._declare
        self.device_param = p('device', '', 'evdev node; empty: the first ' + DEVICE_GLOB)
        self.frame_id = p('frame_id', 'imu_link', 'frame_id of the messages')
        n = p('samples_per_report', 8, 'samples summed per report (firmware IMU_AVERAGE_N)')
        gyro_s = p('gyro_sensitivity_mdps', 17.50, 'gyro mdps/LSB (firmware IMU_GY_FS: 500 dps)')
        accel_s = p('accel_sensitivity_mg', 0.122, 'accel mg/LSB (firmware IMU_XL_FS: 4 g)')
        self.temp_sens_param = p('temperature_sensitivity', 0.0,
                                 'LSB/degC; 0: from the device name (LSM6DS3 16, LSM6DS3TR-C 256)')
        self.gyro_var = p('angular_velocity_variance', DEFAULT_GYRO_VARIANCE,
                          'diagonal of angular_velocity_covariance, (rad/s)^2, x y z')
        self.accel_var = p('linear_acceleration_variance', DEFAULT_ACCEL_VARIANCE,
                           'diagonal of linear_acceleration_covariance, (m/s^2)^2, x y z')
        self.zero_fuzz = p('zero_fuzz', True, 'set fuzz and flat to 0 on axes that have them')
        self.reconnect_interval = p('reconnect_interval', 1.0, 'seconds between open attempts')
        self.publish_temperature = p('publish_temperature', True, 'publish imu/temperature')
        self.temperature_rate = p('temperature_rate', 1.0, 'imu/temperature rate, Hz')

        sync_window = p('time_sync_window', 5.0, 'seconds: the sliding minimum of the offset')
        drift_window = p('time_sync_drift_window', 60.0, 'seconds: the clock drift fit')

        self.stationary_source = p('stationary_source', 'imu',
                                   'what decides "at rest" for the gyro bias: imu, odom or both')
        if self.stationary_source not in STATIONARY_SOURCES:
            raise ValueError(f'stationary_source must be one of {STATIONARY_SOURCES}')
        st_window = p('stationary_window', 1.0, 'imu: seconds of samples judged together')
        st_gyro_std = p('stationary_gyro_std', 0.01, 'imu: max gyro std per axis, rad/s')
        st_accel_std = p('stationary_accel_std', 0.05, 'imu: max accel std per axis, m/s^2')
        st_gyro_mean = p('stationary_gyro_mean', 0.01,
                         'imu: max |gyro mean - bias| per axis, rad/s (slow steady turns)')
        odom_topic = p('odom_topic', '/omni_wheel_drive_controller/odom', 'odom: the topic')
        odom_lin = p('odom_linear_threshold', 0.005, 'odom: max speed at rest, m/s')
        odom_ang = p('odom_angular_threshold', 0.01, 'odom: max turn rate at rest, rad/s')
        odom_hold = p('odom_hold_time', 1.0, 'odom: seconds at rest before it counts')
        odom_timeout = p('odom_timeout', 0.5, 'odom: older than this, not at rest, s')

        bias_init = p('bias_init_duration', 2.5, 'seconds of rest averaged for the startup bias')
        self.bias_init_timeout = p('bias_init_timeout', 10.0,
                                   'seconds; publish without a bias estimate after this')
        bias_tau = p('bias_time_constant', 10.0,
                     'seconds; the bias follows the gyro at rest with this time constant; '
                     '0 freezes it after startup')

        self.scaler = Scaler(n, gyro_s, accel_s)
        self.timesync = TimestampMapper(window=sync_window, drift_window=drift_window)
        self.sequence = SequenceTracker()
        self.imu_still = ImuStationaryDetector(st_window, st_gyro_std, st_accel_std,
                                               st_gyro_mean)
        self.odom_still = OdomStationaryDetector(odom_lin, odom_ang, odom_hold, odom_timeout)
        self.bias = GyroBiasEstimator(bias_init, bias_tau)

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos)
        self.temp_pub = (self.create_publisher(Temperature, 'imu/temperature', 10)
                         if self.publish_temperature else None)
        if self.stationary_source in ('odom', 'both'):
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        # State shared with the diagnostics (read under the GIL; plain values only)
        self.device_path = None
        self.device_name = None
        self.connected = False
        self.fuzz_found = {}
        self.fuzz_zeroed = False
        self.reports = 0
        self.published = 0
        self.flag_counts = {FLAG_SAMPLE_MISSED: 0, FLAG_USB_BUSY: 0, FLAG_ERROR: 0}
        self.last_report_host = None
        self.last_temperature = None
        self.stationary = False
        self.start_wall = None
        self.last_flags = 0
        self._diag_prev = (time.monotonic(), 0, 0, 0, 0, 0)

        self.updater = Updater(self)
        self.updater.setHardwareID('pico_lsm6ds3')
        self.updater.add('IMU', self._diagnose)

        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name='imu_reader', daemon=True)
        self.thread.start()

    # --- parameters -------------------------------------------------------------------------

    def _declare(self, name, default, description):
        return self.declare_parameter(
            name, default, ParameterDescriptor(description=description)).value

    # --- odometry ---------------------------------------------------------------------------

    def _on_odom(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        tw = msg.twist.twist
        self.odom_still.update(t, tw.linear.x, tw.linear.y, tw.angular.z)

    # --- device -----------------------------------------------------------------------------

    def _run(self):
        import evdev  # here, so the tests and --help work without it
        warned = False
        while not self.stop.is_set() and rclpy.ok():
            path = find_device(self.device_param)
            dev = None
            try:
                if path is None:
                    raise FileNotFoundError(errno.ENOENT, 'no device matches', DEVICE_GLOB)
                dev = evdev.InputDevice(path)
            except OSError as e:
                if not warned:
                    self.get_logger().warning(
                        f'cannot open the IMU ({e}); retrying every '
                        f'{self.reconnect_interval:g} s')
                    warned = True
                self.stop.wait(self.reconnect_interval)
                continue
            warned = False
            try:
                self._stream(dev, path)
            except OSError as e:
                if e.errno in (errno.ENODEV, errno.EIO, errno.ENOENT):
                    self.get_logger().warning(f'IMU disconnected ({e}); waiting for it')
                else:
                    self.get_logger().error(f'IMU read failed ({e}); reopening')
            finally:
                self.connected = False
                try:
                    dev.close()
                except OSError:
                    pass
            self.stop.wait(self.reconnect_interval)

    def _check_axes(self, dev):
        """Report fuzz/flat (the kernel's hysteresis), and zero them if allowed."""
        caps = dict(dev.capabilities().get(EV_ABS, []))
        missing = [c for c in ALL_CODES if c not in caps]
        if missing:
            raise OSError(errno.EINVAL, f'{dev.path} ({dev.name}) lacks ABS codes {missing}; '
                                        'is it the LSM6DS3 firmware?')
        self.fuzz_found = {c: (caps[c].fuzz, caps[c].flat) for c in ALL_CODES
                           if caps[c].fuzz or caps[c].flat}
        self.fuzz_zeroed = False
        if not self.fuzz_found:
            return
        self.get_logger().warning(
            f'axes with fuzz/flat (code: (fuzz, flat)): {self.fuzz_found}. The kernel drops '
            'small changes on these. Fix the firmware: declare the HID application collection '
            'a Multi-axis Controller (Generic Desktop 0x08), not a Gamepad/Joystick.')
        if not self.zero_fuzz:
            return
        try:
            for c in self.fuzz_found:
                dev.set_absinfo(c, fuzz=0, flat=0)
            self.fuzz_zeroed = True
            self.get_logger().info('fuzz and flat set to 0 (EVIOCSABS)')
        except OSError as e:
            self.get_logger().error(
                f'cannot zero fuzz ({e}): the node needs write access to {dev.path}. '
                'Small gyro and accel changes will be lost.')

    def _initial_values(self, dev):
        return {c: dev.absinfo(c).value for c in ALL_CODES}

    def _stream(self, dev, path):
        self._check_axes(dev)
        name = dev.name
        sens = self.temp_sens_param or TEMPERATURE_LSB_PER_DEGC.get(name, 16.0)
        self.scaler.temperature_lsb_per_degc = sens
        self.device_path, self.device_name = path, name
        self.updater.setHardwareID(f'{name} {dev.uniq or ""} {path}'.strip())
        self.get_logger().info(f'opened {path} -> {os.path.realpath(path)} ({name})')

        assembler = ReportAssembler(self._initial_values(dev))
        self.timesync.reset()
        self.sequence.reset()
        self.imu_still.reset()
        self.connected = True
        self.start_wall = time.monotonic()
        next_temp = 0.0

        while not self.stop.is_set():
            r, _, _ = select.select([dev.fd], [], [], 0.5)
            if not r:
                if not os.path.exists(path):
                    raise OSError(errno.ENODEV, 'device node removed')
                continue
            for ev in dev.read():
                dropped = assembler.dropped
                report = assembler.feed(ev.type, ev.code, ev.value, ev.sec + ev.usec * 1e-6)
                if dropped and not assembler.dropped:
                    # SYN_DROPPED: the kernel's buffer overflowed. Its packet is skipped; take
                    # the current values from the device (evdev documentation).
                    self.get_logger().warning('evdev buffer overflow (SYN_DROPPED)',
                                              throttle_duration_sec=5.0)
                    assembler.resync(self._initial_values(dev))
                if report is not None:
                    self._handle(report)
                    if self.temp_pub is not None and report.host_time >= next_temp \
                            and not report.flags & FLAG_ERROR:
                        next_temp = report.host_time + 1.0 / max(self.temperature_rate, 1e-3)
                        self._publish_temperature(report)

    # --- reports ----------------------------------------------------------------------------

    def _handle(self, r):
        log = self.get_logger()
        self.reports += 1
        self.last_report_host = r.host_time
        restarts = self.sequence.restarts
        missing = self.sequence.update(r.sequence)
        if missing:
            log.warning(f'{missing} report(s) lost (sequence gap); {self.sequence.lost} so far',
                        throttle_duration_sec=5.0)
        if self.sequence.restarts != restarts:
            log.warning('sequence went backwards: the Pico restarted', throttle_duration_sec=5.0)
        self.last_flags = r.flags
        for bit in self.flag_counts:
            if r.flags & bit:
                self.flag_counts[bit] += 1
        if r.flags:
            msg = f'firmware flags 0x{r.flags:02x}: {describe_flags(r.flags)}'
            if r.flags & FLAG_ERROR:
                log.error(msg + '; data is invalid, not published', throttle_duration_sec=5.0)
            else:
                log.warning(msg, throttle_duration_sec=5.0)

        stamp = self.timesync.update(r.timestamp, r.host_time)
        if r.flags & FLAG_ERROR:
            return

        gyro = self.scaler.gyro(r.gyro)
        accel = self.scaler.accel(r.accel)
        bias = self.bias.bias if self.bias.initialized else None
        imu_still = self.imu_still.update(stamp, gyro, accel, bias)
        if self.stationary_source == 'imu':
            still = imu_still
        else:
            odom_still = self.odom_still.stationary(r.host_time)
            still = odom_still if self.stationary_source == 'odom' else odom_still and imu_still
        self.stationary = still
        if self.bias.update(stamp, gyro, still):
            b = self.bias.bias
            v = [self.imu_still.var(i) for i in range(6)]
            log.info('gyro bias (rad/s): x %.5f y %.5f z %.5f. At rest over the last %.1f s, '
                     'variance gyro (rad/s)^2 %.2e %.2e %.2e, accel (m/s^2)^2 %.2e %.2e %.2e'
                     % (b[0], b[1], b[2], self.imu_still.window, *v))

        if not self.bias.initialized:
            if time.monotonic() - self.start_wall < self.bias_init_timeout:
                return
            log.warning('no gyro bias yet (the IMU has not been at rest for '
                        f'{self.bias.init_duration:g} s): publishing uncorrected rates',
                        throttle_duration_sec=10.0)

        b = self.bias.bias if self.bias.initialized else (0.0, 0.0, 0.0)
        msg = Imu()
        msg.header.stamp = to_time_msg(stamp)
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = gyro[0] - b[0]
        msg.angular_velocity.y = gyro[1] - b[1]
        msg.angular_velocity.z = gyro[2] - b[2]
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        for i in range(3):
            msg.angular_velocity_covariance[4 * i] = float(self.gyro_var[i])
            msg.linear_acceleration_covariance[4 * i] = float(self.accel_var[i])
        self.imu_pub.publish(msg)
        self.published += 1

    def _publish_temperature(self, r):
        t = self.scaler.temperature(r.temperature)
        self.last_temperature = t
        msg = Temperature()
        msg.header.stamp = to_time_msg(self.timesync.host0 + self.timesync.last_stamp)
        msg.header.frame_id = self.frame_id
        msg.temperature = t
        msg.variance = 0.0
        self.temp_pub.publish(msg)

    # --- diagnostics ------------------------------------------------------------------------

    def _diagnose(self, stat):
        now = time.monotonic()
        t0, reports0, lost0, missed0, busy0, err0 = self._diag_prev
        fc = self.flag_counts
        cur = (now, self.reports, self.sequence.lost, fc[FLAG_SAMPLE_MISSED], fc[FLAG_USB_BUSY],
               fc[FLAG_ERROR])
        self._diag_prev = cur
        dt = max(now - t0, 1e-3)
        rate = (cur[1] - reports0) / dt
        d_lost, d_missed, d_busy, d_err = (cur[2] - lost0, cur[3] - missed0, cur[4] - busy0,
                                           cur[5] - err0)

        if not self.connected:
            stat.summary(DiagnosticStatus.ERROR, 'device not connected')
        elif d_err:
            stat.summary(DiagnosticStatus.ERROR, 'firmware reports a sensor error')
        elif rate < 1.0:
            stat.summary(DiagnosticStatus.ERROR, 'no reports')
        elif d_lost or d_missed or d_busy:
            stat.summary(DiagnosticStatus.WARN,
                         f'lost {d_lost} reports, {d_missed} missed samples, '
                         f'{d_busy} USB busy since the last update')
        elif self.fuzz_found and not self.fuzz_zeroed:
            stat.summary(DiagnosticStatus.WARN, 'axes have fuzz; small changes are dropped')
        elif not self.bias.initialized:
            stat.summary(DiagnosticStatus.WARN, 'estimating the gyro bias (keep it still)')
        else:
            stat.summary(DiagnosticStatus.OK, 'ok')

        b = self.bias.bias
        ts = self.timesync
        stat.add('device', f'{self.device_path} ({self.device_name})')
        stat.add('rate (Hz)', f'{rate:.1f}')
        stat.add('reports', str(self.reports))
        stat.add('published', str(self.published))
        stat.add('reports lost (sequence gaps)', str(self.sequence.lost))
        stat.add('sequence gaps', str(self.sequence.gaps))
        stat.add('flag: sample missed', str(fc[FLAG_SAMPLE_MISSED]))
        stat.add('flag: USB busy', str(fc[FLAG_USB_BUSY]))
        stat.add('flag: error', str(fc[FLAG_ERROR]))
        stat.add('fuzz/flat found', str(self.fuzz_found or 'none'))
        stat.add('gyro bias (rad/s)', f'{b[0]:.5f} {b[1]:.5f} {b[2]:.5f}')
        stat.add('gyro bias estimated', str(self.bias.initialized))
        stat.add('stationary', str(self.stationary))
        stat.add('stationary source', self.stationary_source)
        stat.add('clock drift (ppm)', f'{ts.drift_ppm:.2f}')
        stat.add('latency (ms)', f'{ts.latency * 1e3:.3f}')
        stat.add('time sync restarts', str(ts.resets))
        if self.last_temperature is not None:
            stat.add('temperature (degC)', f'{self.last_temperature:.2f}')
        return stat

    def destroy_node(self):
        self.stop.set()
        self.thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
