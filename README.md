# witimu

CycloneDDS publisher/subscriber for the WitMotion BWT901CL IMU, with torso-orientation calibration for the Unitree G1 robot.

## Topics

| Topic        | Type            | Fields                                   |
|--------------|-----------------|------------------------------------------|
| `Acc`        | `AccelMsg`      | `ax, ay, az` (m/s²), `temperature` (°C) |
| `Gyro`       | `GyroMsg`       | `wx, wy, wz` (°/s), `voltage` (V)       |
| `Quat`       | `QuaternionMsg` | `q0, q1, q2, q3` (w, x, y, z) — raw IMU |
| `TorsoQuat`  | `TorsoQuatMsg`  | `q0, q1, q2, q3` (w, x, y, z) — calibrated torso |

## Requirements

```
cyclonedds
pyserial
numpy
scipy
unitree_sdk2py   # only needed for calibrate.py
```

## Usage

### IMU Publisher

Reads from the IMU over serial and publishes raw `Acc`, `Gyro`, and `Quat` topics.

```bash
./run_publisher.sh
```

Or as a standalone script (adjust port as needed):

```python
from imu_publisher import ImuPublisher

pub = ImuPublisher(port="/dev/ttyUSB0")
pub.run()                  # runs until Ctrl+C
pub.run(duration=10.0)     # runs for 10 seconds
```

### IMU Listener

Subscribes to all three raw topics. Properties return tuples on each access.

```python
from imu_listener import ImuListener

listener = ImuListener()

ax, ay, az   = listener.acc      # tuple[float, float, float] | None
wx, wy, wz   = listener.ang_vel  # tuple[float, float, float] | None
w, x, y, z   = listener.quat     # tuple[float, float, float, float] | None
```

Run standalone to print live data:

```bash
./run_listener.sh
```

### Calibration

Calibrates the torso IMU mounting offset against the G1 robot's pelvis IMU and waist forward kinematics. Requires `imu_publisher` to be running and the robot to be connected via `rt/lowstate`.

Keep the robot stationary while calibrating:

```bash
python calibrate.py [--samples N] [--duration T] [--output PATH]
```

| Argument     | Default          | Description                                  |
|--------------|------------------|----------------------------------------------|
| `--samples`  | `20`             | Number of samples to collect                 |
| `--duration` | `2.0`            | Collection window in seconds                 |
| `--output`   | `./offset`       | Base path for output (writes `.npy`+`.json`) |

Outputs `offset.npy` (mean quaternion) and `offset.json` (full statistics). These are required by `TorsoQuatPublisher`.

### Torso Quaternion Publisher

Loads the calibration offset and re-publishes the corrected torso orientation to the `TorsoQuat` topic at 200 Hz. Requires `imu_publisher` to be running and `offset.npy` to exist.

```python
from torso_quat_publisher import TorsoQuatPublisher

pub = TorsoQuatPublisher()
pub.run()   # runs until Ctrl+C
```

Run standalone (also runs calibration first):

```bash
python torso_quat_publisher.py
```

### Torso Quaternion Listener

Subscribes to `TorsoQuat` and exposes the calibrated torso orientation.

```python
from torso_quat_listener import TorsoQuatListener

listener = TorsoQuatListener()

w, x, y, z              = listener.quat   # tuple[float, float, float, float] | None
yaw, pitch, roll         = listener.euler  # ZYX degrees | None
```

Run standalone to print live calibrated data:

```bash
python torso_quat_listener.py
```

## Service (autostart)

Install the publishers as systemd user services that restart every 2 seconds on failure:

```bash
./scripts/install_imu_publisher_service.bash
./scripts/install_torso_quat_publisher_service.bash
```

```bash
journalctl --user -u imu_publisher -f            # IMU publisher logs
journalctl --user -u torso_quat_publisher -f     # torso quat publisher logs

systemctl --user stop imu_publisher              # stop
systemctl --user disable imu_publisher           # remove from autostart
```

## Network config

`cyclonedds.xml` binds CycloneDDS to the `192.168.123.x` interface. All publishers and listeners load it automatically via `CYCLONEDDS_URI`. To use a different interface, edit the `NetworkInterface address` in `cyclonedds.xml`.
