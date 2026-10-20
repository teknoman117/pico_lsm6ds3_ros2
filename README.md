# pico_lsm6ds3_ros2

ROS 2 (Jazzy) driver for a Raspberry Pi Pico (or Pico 2) with an LSM6DS3 / LSM6DS3TR-C IMU,
running the [pico-lsm6ds3-hid](https://github.com/teknoman117/pico-lsm6ds3-hid) firmware,
which shows up on the host as a USB HID device. The package has three parts:

- **`driver`** reads the device through evdev and publishes `sensor_msgs/Imu` on
  `imu/data_raw` (angular velocity and linear acceleration, no orientation). The stamps come
  from the Pico's clock, and the node removes the gyro bias. It also publishes the temperature
  and `/diagnostics`.
- **Fusion:** [`imu_filter_madgwick`](https://github.com/CCNYRoboticsLab/imu_tools) computes
  full 3D orientation from the gyro and accelerometer, with no magnetometer, and publishes
  `imu/data`.
- **`validation.launch.py`** runs both with RViz. Tilt or turn the device, and the axes in RViz
  move with it.

It was written for a LeKiwi robot, where the IMU's gyro corrects the heading errors that come
from omni-wheel slip.

## Device

- The firmware is [teknoman117/pico-lsm6ds3-hid](https://github.com/teknoman117/pico-lsm6ds3-hid),
  with its default `src/config.h`. That config sets gyro and accel to 1666 Hz, gyro to
  ±500 dps (17.50 mdps/LSB) and accel to ±4 g (0.122 mg/LSB). It sends one report per 8
  samples, at about 208 Hz nominal. One unit measured 210.3 Hz, because the sensor's own clock
  runs about 1% fast.
- The device must appear as `/dev/input/by-id/usb-2e8a_LSM6DS3*-event-*` (vendor 2e8a, product
  name `LSM6DS3` or `LSM6DS3TR-C`). With `device` left empty, the driver opens the first one it
  finds.
- The node needs read access to the event node. On most distributions that means being in the
  `input` group. It needs write access only to zero fuzz (below).

### HID fields and evdev codes

These come from the firmware's report descriptor (`src/report.c`). Each HID report becomes one
evdev packet:

| evdev code | HID usage | Field |
|---|---|---|
| `ABS_RX`, `ABS_RY`, `ABS_RZ` (3, 4, 5) | Generic Desktop Rx/Ry/Rz, int32 | gyro X/Y/Z: sum of 8 raw samples |
| `ABS_X`, `ABS_Y`, `ABS_Z` (0, 1, 2) | Generic Desktop X/Y/Z, int32 | accel X/Y/Z: sum of 8 raw samples |
| `ABS_MISC` (0x28) | Sensor page Timestamp (0x0529), 0..2^31-1 | window midpoint, Pico µs mod 2^31 |
| `ABS_RUDDER` (7) | Generic Desktop Dial | sequence, +1 per report, mod 2^31 |
| `ABS_THROTTLE` (6) | Generic Desktop Slider, int16 | `OUT_TEMP` raw, last sample of the window |
| `ABS_WHEEL` (8) | Generic Desktop Wheel, 0..255 | flags: bit 0 sample missed, bit 1 USB busy (a report was replaced), bit 2 sensor error (data is 0) |

The application collection is a Multi-axis Controller (Generic Desktop 0x08).

- Scaling: `value / samples_per_report × sensitivity`. Gyro: 17.50 mdps/LSB, converted to
  rad/s. Accel: 0.122 mg/LSB, times 9.80665, giving m/s².
- Temperature: `25 °C + raw / 16` (LSM6DS3), or `raw / 256` (LSM6DS3TR-C). The divisor is
  chosen by the device name.

### evdev, not hidraw

The fields map one-to-one onto ABS codes, so evdev carries everything the report has. Using
evdev also has these advantages:

- The event node is accessible to the `input` group. The hidraw node is root-only unless you
  add a udev rule.
- Each packet comes with a kernel timestamp, taken when the USB transfer completed, on the
  clock ROS uses (`CLOCK_REALTIME`). That makes a better arrival time than the moment a Python
  read returns.
- Reconnecting is simple.

evdev has two known pitfalls, and the driver handles both:

1. **Unchanged values send no event.** The input core suppresses an ABS event whose value
   equals the previous one. The node keeps the last value of every axis, seeded from
   `EVIOCGABS` when the device opens, and assembles a complete report at each `SYN_REPORT`.
   The sequence and timestamp change in every report, so every report produces a
   `SYN_REPORT`. After `SYN_DROPPED` (a kernel buffer overflow), it skips that packet and
   re-reads the values.
2. **Fuzz.** For Gamepad and Joystick collections, `hid-input` gives each axis a fuzz of
   range/256, and the input core then drops small changes. This firmware declares a
   Multi-axis Controller, so all of its axes have fuzz 0 and flat 0 (checked on a real device
   with `evtest`). The node still checks at startup. If any axis has fuzz or flat, it logs a
   warning and, with `zero_fuzz` set (the default), sets both to 0 with `EVIOCSABS`. That needs
   the event node opened read-write.

## Topics

| Topic | Type | |
|---|---|---|
| `imu/data_raw` | `sensor_msgs/Imu` | driver: rates and accelerations, `orientation_covariance[0] = -1` |
| `imu/temperature` | `sensor_msgs/Temperature` | driver: the sensor's temperature, `temperature_rate` Hz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | driver: the status `pico_lsm6ds3_driver: IMU` |
| `imu/data` | `sensor_msgs/Imu` | Madgwick: `imu/data_raw` plus orientation |
| `/tf` | | Madgwick, with `publish_tf:=true`: `fixed_frame` → `frame_id` |
| `odom_topic` (subscribed) | `nav_msgs/Odometry` | with `stationary_source` set to `odom` or `both` |

`imu/data_raw` is published reliable, depth 50.

## Driver parameters

| Parameter | Default | |
|---|---|---|
| `device` | `''` | evdev node. Empty: the first `/dev/input/by-id/usb-2e8a_LSM6DS3*-event-*`. Use the by-id path, not `eventN` |
| `frame_id` | `imu_link` | |
| `samples_per_report` | `8` | the firmware's `IMU_AVERAGE_N` |
| `gyro_sensitivity_mdps` | `17.5` | mdps/LSB, from the firmware's `IMU_GY_FS` (±500 dps) |
| `accel_sensitivity_mg` | `0.122` | mg/LSB, from the firmware's `IMU_XL_FS` (±4 g) |
| `temperature_sensitivity` | `0.0` | LSB/°C. 0: chosen by the device name (16 or 256) |
| `angular_velocity_variance` | `[3.2e-6, 1.0e-5, 2.7e-6]` | (rad/s)², the diagonal of the covariance (see Covariance) |
| `linear_acceleration_variance` | `[1.6e-4, 1.2e-4, 1.2e-4]` | (m/s²)² |
| `stationary_source` | `imu` | what decides "at rest" for the gyro bias: `imu`, `odom` or `both` |
| `stationary_window` | `1.0` | imu: seconds of samples judged together |
| `stationary_gyro_std` | `0.01` | imu: maximum gyro standard deviation per axis, rad/s |
| `stationary_accel_std` | `0.05` | imu: maximum accel standard deviation per axis, m/s² |
| `stationary_gyro_mean` | `0.01` | imu: maximum \|window mean − bias\| per gyro axis, rad/s (catches slow, steady turns) |
| `odom_topic` | `/omni_wheel_drive_controller/odom` | odom: the wheel odometry |
| `odom_linear_threshold`, `odom_angular_threshold` | `0.005`, `0.01` | odom: m/s and rad/s counted as zero |
| `odom_hold_time` | `1.0` | odom: seconds near zero before it counts as rest |
| `odom_timeout` | `0.5` | odom: an older message counts as not at rest |
| `bias_init_duration` | `2.5` | seconds of continuous rest averaged for the startup bias |
| `bias_init_timeout` | `10.0` | publish without a bias estimate (with a warning) if none is ready after this |
| `bias_time_constant` | `10.0` | seconds. At rest, the bias follows the gyro with this time constant; 0 freezes it after startup |
| `time_sync_window` | `5.0` | seconds: the sliding minimum of the clock offset |
| `time_sync_drift_window` | `60.0` | seconds: the clock drift fit |
| `zero_fuzz` | `true` | zero fuzz and flat if the kernel set them |
| `publish_temperature`, `temperature_rate` | `true`, `1.0` | |
| `reconnect_interval` | `1.0` | seconds between attempts to open the device |

`config/driver.yaml` lists the tunable parameters. `imu.launch.py` loads it, so the driver
starts from that file's values, and its arguments then override `device`, `frame_id` and
`stationary_source`.

### Gyro bias and stationary detection

The LSM6DS3's zero-rate offset is large, and it changes at every power-up. One unit read
(+1.48, −8.52, +0.16) °/s on X/Y/Z, then (+1.32, −6.99, +4.87) °/s after being plugged in
again. Left uncorrected, that
would make the fused orientation spin. Madgwick without a magnetometer has nothing to correct
yaw with, so the driver removes the bias itself:

- **At startup**, it averages `bias_init_duration` seconds of continuous rest. Nothing is
  published until that is done. The first estimate is ready about 3.5 s after the device
  opens: 1 s for the stationary window to fill, then 2.5 s of averaging. If the IMU never
  comes to rest, the driver publishes uncorrected rates after `bias_init_timeout`, with a
  warning.
- **Afterwards**, while stationary, the bias follows the gyro with a first-order filter
  (`bias_time_constant`), so slow drift with temperature is tracked.
- **Stationary** means one of three things, set by `stationary_source`:
  - `imu`: from the IMU itself. Over the last `stationary_window`, every gyro and accel axis's
    standard deviation is under its threshold, and every gyro axis's mean is within
    `stationary_gyro_mean` of the current bias. A slow, steady turn has no more variance than
    rest, so the mean check is what catches it.
  - `odom`: the wheel odometry's twist has been near zero for `odom_hold_time`, and its last
    message is fresh.
  - `both`: both of the above must agree. This is the safer choice on a robot.

  On a workstation there is no odometry, so `imu` (the default) works there on its own.

### Covariance

The covariances are the variance of each axis per message, at rest. To measure them, keep the
IMU still (on the robot: wheels and arm stopped, servos powered as in use) with the driver
running, then run:

```bash
ros2 run pico_lsm6ds3_ros2 measure_noise --ros-args -p duration:=60.0
```

It prints the means, the standard deviations, and the parameter lines to paste into
`config/driver.yaml`. The driver also logs the variance over the last second when it finishes
its startup bias estimate.

The defaults were measured on the first unit, at rest on a desk:

- Gyro standard deviation: 1.8, 3.0 and 1.6 mrad/s (0.10, 0.17 and 0.09 °/s).
- Accel standard deviation: about 0.012 m/s².

On the robot, vibration will raise both numbers, so measure there.

## Timestamps

The stamp of each message is the midpoint of its sampling window on the Pico's crystal clock,
mapped to ROS time. Arrival time is never used as the stamp:

1. **Unwrap.** The device timestamp is µs mod 2^31, which wraps every 35.8 min. The driver
   accumulates the steps mod 2^31. A step back, or one forward of more than 1 s, means the
   Pico restarted, so the mapping starts again.
2. **Offset.** For each report, `offset = host_receive_time − device_time`. The receive time is
   the kernel's evdev timestamp. The offset equals the true clock offset plus the delivery
   latency, which is never negative. The lower envelope of the offsets therefore tracks the
   clock offset, at the shortest latency. Before there is enough data to fit drift, the driver
   uses the minimum over a sliding `time_sync_window` (5 s).
3. **Drift.** The two clocks run at slightly different rates: one Pico measured −57 to −63 ppm
   against an NTP-disciplined host. That is 0.3 ms over a 5 s window. The driver keeps the
   minimum of each 1 s block over `time_sync_drift_window` (60 s) and fits a line through those
   minima for the slope. It then lowers the line until no recent offset lies below it. The fit
   is redone once per block.
4. `stamp = device_time + offset estimate`. A report that arrives earlier than the estimate
   allows lowers the line at once, so a stamp is never later than its arrival. Stamps are kept
   strictly increasing. An arrival more than 0.5 s after its stamp (a host clock step) restarts
   the mapping.

Measured result: over 60 s, the steps between stamps vary by 0.8 µs (standard deviation),
against 0.94 ms for the arrival times. Stamps sit about 0.9 ms before the kernel's receive
time.

### Sequence and flags

- **Sequence gaps** (lost reports) are counted and logged as a throttled warning.
- **Firmware flags:**
  - sample missed (bit 0) and USB busy, meaning a report was dropped (bit 1): counted, logged
    as a throttled warning, and reported in the diagnostics.
  - sensor error (bit 2): the data is zero, so the report is not published, and the
    diagnostics go to ERROR.
- **Diagnostics** also report the rate, bias, clock drift, latency, fuzz and connection state.
- **Reconnect:** if the device is unplugged, the driver logs it and retries every
  `reconnect_interval`. When the device returns, it resumes with a fresh time mapping and keeps
  the bias estimate.

## Fusion

Fusion uses `imu_filter_madgwick`, configured in `config/madgwick.yaml`. It takes the gyro and
accelerometer with `use_mag: false` and `world_frame: enu`, and keeps gravity in the
acceleration (`remove_gravity_vector: false`).

- **`gain: 0.02`** is lower than the default 0.1. The accelerometer pulls roll and pitch toward
  gravity at about `gain` rad/s. At 0.1, the robot's own acceleration would leak into roll and
  pitch: 0.5 m/s² reads as a 3° tilt. The bias-corrected gyro is good enough to lean on. Yaw
  has no reference, so it is the integrated gyro whatever the gain.
- **`zeta: 0`**, because the driver removes the bias.
- **dt from the stamps.** The stamps are smooth, so the integration step is too.
- **`publish_tf`** is a launch argument. Use `true` for standalone validation and `false` on
  the robot, where `robot_localization` will own TF.

`imu_complementary_filter` would also work. The Madgwick filter was chosen because it fits:
its accelerometer correction does not affect yaw, it accepts the variable dt the stamps give,
and `rviz_imu_plugin` comes from the same repository.

## Launch

```bash
# driver + fusion
ros2 launch pico_lsm6ds3_ros2 imu.launch.py [device:=/dev/input/by-id/...] [frame_id:=imu_link] \
    [publish_tf:=false] [fixed_frame:=odom] [stationary_source:=imu|odom|both]

# standalone validation: the above with publish_tf:=true, a static world -> odom and RViz
ros2 launch pico_lsm6ds3_ros2 validation.launch.py [device:=...] [rviz:=true]
```

`validation.launch.py` builds this TF chain:

- `world` → `odom`: a static, identity transform.
- `odom` → `imu_link`: Madgwick's fused orientation.

RViz's fixed frame is `world`. It shows the TF axes of `imu_link`, and the `rviz_imu_plugin`
display of `imu/data`: a box, axes, and the acceleration vector, which points up at rest. At
start, yaw is wherever the device's x axis points, and roll and pitch are the device's tilt.
Turn the device about its z axis and the axes turn the same amount. Tilt it and they tilt.

### In the LeKiwi workspace (Docker)

The [lekiwi_ros2_ws](https://github.com/teknoman117/lekiwi_ros2_ws) workstation image includes
this package. Its `imu` Compose profile runs `validation.launch.py` on the workstation, with
no robot needed:

```bash
docker/setup-env.sh workstation       # once: finds IMU_DEVICE
docker compose -f docker/compose.workstation.yaml --profile imu up
```

## Tests

```bash
colcon test --packages-select pico_lsm6ds3_ros2 && colcon test-result --verbose
```

The tests are pure Python (pytest) and need no device. They cover the scaling, report assembly
from evdev events, sequence gaps, the timestamp unwrap, offset and drift estimation, stationary
detection, and the bias estimate.

## License

Apache-2.0; see `LICENSE`. The firmware is a separate project with its own license (MPL-2.0).
