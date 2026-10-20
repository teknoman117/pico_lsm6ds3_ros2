# SPDX-License-Identifier: Apache-2.0
"""Measure the IMU's noise for the covariance parameters.

Keep the IMU still (on the robot: wheels and arm stopped), then:

    ros2 run pico_lsm6ds3_ros2 measure_noise --ros-args -p duration:=60.0

Subscribes to imu/data_raw, and prints the mean and variance of each axis and the parameter
lines for angular_velocity_variance and linear_acceleration_variance.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class NoiseMeter(Node):

    def __init__(self):
        super().__init__('imu_measure_noise')
        self.duration = self.declare_parameter('duration', 60.0).value
        self.n = 0
        self.s = [0.0] * 6
        self.ss = [0.0] * 6
        self.first = None
        self.last = None
        self.done = False
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, 100)
        self.get_logger().info(f'collecting imu/data_raw for {self.duration:g} s; keep it still')

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.first is None:
            self.first = t
        self.last = t
        w, a = msg.angular_velocity, msg.linear_acceleration
        for i, x in enumerate((w.x, w.y, w.z, a.x, a.y, a.z)):
            self.s[i] += x
            self.ss[i] += x * x
        self.n += 1
        if t - self.first >= self.duration:
            self.done = True

    def report(self):
        n = self.n
        if n < 2:
            print('no messages on imu/data_raw')
            return
        mean = [s / n for s in self.s]
        var = [max(0.0, ss / n - m * m) for ss, m in zip(self.ss, mean)]
        span = self.last - self.first
        print(f'{n} messages over {span:.1f} s = {(n - 1) / span:.1f} Hz')
        print('gyro  mean rad/s  %+.6f %+.6f %+.6f' % tuple(mean[:3]))
        print('gyro  std  rad/s  %.6f %.6f %.6f' % tuple(math.sqrt(v) for v in var[:3]))
        print('accel mean m/s^2  %+.4f %+.4f %+.4f   |a| = %.4f'
              % (*mean[3:], math.sqrt(sum(m * m for m in mean[3:]))))
        print('accel std  m/s^2  %.5f %.5f %.5f' % tuple(math.sqrt(v) for v in var[3:]))
        print('parameters:')
        print('  angular_velocity_variance: [%.2e, %.2e, %.2e]' % tuple(var[:3]))
        print('  linear_acceleration_variance: [%.2e, %.2e, %.2e]' % tuple(var[3:]))


def main(args=None):
    rclpy.init(args=args)
    node = NoiseMeter()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
