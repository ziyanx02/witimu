"""
Calibrate the torso IMU (witimu / BWT901CL) against the G1 robot's forward kinematics.

Kinematic chain (from URDF):
    pelvis
      └─[waist_yaw_joint,  axis Z] waist_yaw_link
          └─[waist_roll_joint, axis X] waist_roll_link
              └─[waist_pitch_joint, axis Y] torso_link

The pelvis IMU (from unitree_sdk2py rt/lowstate) gives R_pelvis in world frame.
Composing with the three waist joints gives R_torso_fk (expected torso orientation).
The witimu reports R_imu (measured torso orientation).

The mounting offset is defined as:
    R_imu = R_torso_fk @ R_mount
    =>  R_mount = inv(R_torso_fk) @ R_imu

After calibration, the corrected torso orientation from any new IMU reading is:
    R_torso = R_imu @ inv(R_mount)

Usage
-----
    python calibrate.py [--duration T] [--samples N] [--output PATH]

    Collects N evenly-spaced samples over T seconds (default: 20 samples / 2 s),
    prints mean and std, and saves offset.npy + offset.json.
"""

import argparse
import json
import os
import re
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation

_dir = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault("CYCLONEDDS_URI", f"file://{_dir}/cyclonedds.xml")

try:
    from .imu_listener import ImuListener
except ImportError:
    from imu_listener import ImuListener

# ── unitree_sdk2py ────────────────────────────────────────────────────────────
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowState_hg

# ── G1 waist joint indices in the full 29-DOF motor array ────────────────────
WAIST_YAW_IDX   = 12
WAIST_ROLL_IDX  = 13
WAIST_PITCH_IDX = 14


# ── Network helpers ───────────────────────────────────────────────────────────

def find_interface_by_ip(prefix: str = "192.168.123") -> str:
    output = subprocess.check_output(["ifconfig"], text=True)
    for iface_block in output.split("\n\n"):
        lines = iface_block.strip().splitlines()
        if not lines:
            continue
        iface_name = lines[0].split(":")[0]
        for line in lines:
            match = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", line)
            if match and match.group(1).startswith(prefix):
                return iface_name
    raise RuntimeError(f"No interface found with IP starting with {prefix}")


# ── Kinematics ────────────────────────────────────────────────────────────────

def fk_torso(pelvis_quat_wxyz: np.ndarray,
             q_yaw: float, q_roll: float, q_pitch: float) -> Rotation:
    """
    Compute the torso orientation in world frame via forward kinematics.

    Parameters
    ----------
    pelvis_quat_wxyz : (4,) array, [w, x, y, z]
        Pelvis IMU quaternion from unitree rt/lowstate.
    q_yaw, q_roll, q_pitch : float
        Waist joint angles in radians.
    """
    w, x, y, z = pelvis_quat_wxyz
    R_pelvis = Rotation.from_quat([x, y, z, w])           # scipy scalar-last

    R_yaw   = Rotation.from_rotvec([0.0,    0.0,    q_yaw])   # axis Z
    R_roll  = Rotation.from_rotvec([q_roll, 0.0,    0.0])     # axis X
    R_pitch = Rotation.from_rotvec([0.0,    q_pitch, 0.0])    # axis Y

    return R_pelvis * R_yaw * R_roll * R_pitch


def quat_mean(quats_xyzw: np.ndarray) -> np.ndarray:
    """
    Compute the mean quaternion from an (N, 4) array [x, y, z, w].

    Uses eigendecomposition of the outer-product matrix (Markley et al. 2007).
    """
    Q = quats_xyzw.copy()
    # Resolve double-cover: flip any quaternion that points away from q[0]
    signs = np.sign(Q @ Q[0])
    signs[signs == 0] = 1.0
    Q *= signs[:, None]
    M = Q.T @ Q
    _, eigenvectors = np.linalg.eigh(M)
    mean_q = eigenvectors[:, -1]          # largest eigenvalue → last column
    if mean_q[3] < 0:
        mean_q = -mean_q
    return mean_q / np.linalg.norm(mean_q)


# ── Unitree low-state subscriber ──────────────────────────────────────────────

class UnitreeStateReader:
    """Reads pelvis IMU + motor states from unitree_sdk2py rt/lowstate."""

    def __init__(self) -> None:
        self._msg: LowState_hg | None = None
        self._received = False

    def init(self) -> None:
        try:
            ChannelFactoryInitialize(0, find_interface_by_ip())
        except Exception:
            pass
        sub = ChannelSubscriber("rt/lowstate", LowState_hg)
        sub.Init(self._handler, 10)
        print("Waiting for rt/lowstate ...")
        while not self._received:
            time.sleep(0.1)
        print("rt/lowstate received.")

    def _handler(self, msg: LowState_hg) -> None:
        self._msg = msg
        self._received = True

    @property
    def pelvis_quat_wxyz(self) -> np.ndarray:
        """Pelvis IMU quaternion [w, x, y, z]."""
        assert self._msg is not None
        return np.array(self._msg.imu_state.quaternion)

    @property
    def waist_angles(self) -> tuple:
        """(yaw, roll, pitch) waist joint angles in radians."""
        assert self._msg is not None
        ms = self._msg.motor_state
        return (ms[WAIST_YAW_IDX].q, ms[WAIST_ROLL_IDX].q, ms[WAIST_PITCH_IDX].q)


def wait_for_imu(listener: ImuListener, timeout: float = 10.0) -> None:
    """Block until the ImuListener has received at least one Quat sample."""
    t0 = time.time()
    while listener.quat is None:
        if time.time() - t0 > timeout:
            raise TimeoutError("No witimu Quat data received within timeout.")
        time.sleep(0.05)
    print("witimu Quat data received.")


# ── Calibration routine ───────────────────────────────────────────────────────

def calibrate(n_samples: int = 20, duration: float = 2.0,
              output: str = "offset") -> dict:
    """
    Collect N synchronized samples evenly spaced over `duration` seconds,
    compute the torso IMU mounting offset, and report mean ± std.

    Parameters
    ----------
    n_samples : int
        Number of timestamps to sample (default 20).
    duration : float
        Total collection window in seconds (default 2.0).
    output : str
        Base path (no extension) for offset.npy and offset.json.
    """
    unitree = UnitreeStateReader()
    unitree.init()

    witimu = ImuListener()
    wait_for_imu(witimu)

    interval = duration / n_samples
    print(f"\nCollecting {n_samples} samples over {duration:.1f} s "
          f"(every {interval*1000:.0f} ms) — keep the robot stationary ...")

    offsets_xyzw = []

    for i in range(n_samples):
        t_start = time.time()

        pelvis_q = unitree.pelvis_quat_wxyz
        yaw, roll, pitch = unitree.waist_angles
        R_torso_fk = fk_torso(pelvis_q, yaw, roll, pitch)

        imu_q = witimu.quat    # (w, x, y, z) per ImuListener convention
        if imu_q is None:
            print(f"  [sample {i+1:2d}] no witimu data, skipping")
            time.sleep(interval)
            continue

        w, x, y, z = imu_q
        R_imu = Rotation.from_quat([x, y, z, w])

        R_mount = R_torso_fk.inv() * R_imu        # R_mount = R_torso_fk^-1 @ R_imu
        q = R_mount.as_quat()                      # [x, y, z, w]
        offsets_xyzw.append(q)

        euler = R_mount.as_euler("ZYX", degrees=True)
        print(f"  [{i+1:2d}/{n_samples}]  "
              f"yaw={euler[0]:+7.3f}°  pitch={euler[1]:+7.3f}°  roll={euler[2]:+7.3f}°")

        elapsed = time.time() - t_start
        time.sleep(max(0.0, interval - elapsed))

    if not offsets_xyzw:
        raise RuntimeError("No valid samples collected.")

    offsets_xyzw = np.array(offsets_xyzw)         # (N, 4)

    # ── Mean ──────────────────────────────────────────────────────────────────
    mean_q    = quat_mean(offsets_xyzw)
    R_mean    = Rotation.from_quat(mean_q)
    mean_euler = R_mean.as_euler("ZYX", degrees=True)   # [yaw, pitch, roll]

    # ── Std: angular deviation of each sample from the mean, per axis ─────────
    R_samples = Rotation.from_quat(offsets_xyzw)
    # relative rotation from mean to each sample → rotvec ≈ axis-angle error
    rel_rotvecs = (R_mean.inv() * R_samples).as_rotvec(degrees=True)  # (N, 3) in deg
    std_rotvec  = rel_rotvecs.std(axis=0)                              # (3,) per axis
    std_deg     = float(np.linalg.norm(std_rotvec))                    # scalar angular std

    # ── Print summary ─────────────────────────────────────────────────────────
    print("\n" + "─" * 55)
    print(f"  Samples collected : {len(offsets_xyzw)}")
    print(f"  Mean  quaternion  [x y z w]: "
          f"[{mean_q[0]:+.4f}  {mean_q[1]:+.4f}  {mean_q[2]:+.4f}  {mean_q[3]:+.4f}]")
    print(f"  Mean  euler (ZYX) : "
          f"yaw={mean_euler[0]:+.3f}°  pitch={mean_euler[1]:+.3f}°  roll={mean_euler[2]:+.3f}°")
    print(f"  Std   per-axis    : "
          f"yaw={std_rotvec[2]:+.3f}°  pitch={std_rotvec[1]:+.3f}°  roll={std_rotvec[0]:+.3f}°")
    print(f"  Std   angular     : {std_deg:.3f}°")
    print("─" * 55)

    result = {
        "n_samples": len(offsets_xyzw),
        "mean_quat_xyzw": mean_q.tolist(),
        "mean_euler_zyx_deg": {
            "yaw":   float(mean_euler[0]),
            "pitch": float(mean_euler[1]),
            "roll":  float(mean_euler[2]),
        },
        "std_euler_zyx_deg": {
            "yaw":   float(std_rotvec[2]),
            "pitch": float(std_rotvec[1]),
            "roll":  float(std_rotvec[0]),
        },
        "std_angular_deg": float(std_deg),
    }

    # ── Save ──────────────────────────────────────────────────────────────────
    npy_path  = output + ".npy"
    json_path = output + ".json"

    np.save(npy_path, mean_q)
    with open(json_path, "w") as f:
        json.dump(result, f, indent=2)

    print(f"\nSaved  {npy_path}")
    print(f"Saved  {json_path}")

    return result


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calibrate torso IMU (witimu) against G1 forward kinematics."
    )
    parser.add_argument("--samples",  type=int,   default=20,
                        help="Number of timestamps to sample (default: 20).")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Collection window in seconds (default: 2.0).")
    parser.add_argument("--output",   type=str,
                        default=os.path.join(_dir, "offset"),
                        help="Output base path (no extension). "
                             "Writes <path>.npy and <path>.json.")
    args = parser.parse_args()

    calibrate(n_samples=args.samples, duration=args.duration, output=args.output)
