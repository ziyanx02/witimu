"""
Torso-orientation publisher for the BWT901CL witimu.

Loads the pre-computed mounting offset (offset.npy written by calibrate.py),
subscribes to the "Quat" DDS topic (raw IMU quaternion from imu_publisher),
and publishes the corrected torso quaternion to the "TorsoQuat" DDS topic.

Topic published: "TorsoQuat"  →  TorsoQuatMsg(q0=w, q1=x, q2=y, q3=z)

Requires:
  - offset.npy produced by calibrate.py
  - imu_publisher active on the same DDS domain
"""

import os
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.topic import Topic
from cyclonedds.idl import IdlStruct

_dir = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault("CYCLONEDDS_URI", f"file://{_dir}/cyclonedds.xml")

OFFSET_PATH = os.path.join(_dir, "offset.npy")

try:
    from .imu_listener import ImuListener
except ImportError:
    from imu_listener import ImuListener

try:
    from .calibrate import calibrate
except ImportError:
    from calibrate import calibrate

# ── DDS IDL type ──────────────────────────────────────────────────────────────

@dataclass
class TorsoQuatMsg(IdlStruct, typename="TorsoQuat"):
    q0: float  # w
    q1: float  # x
    q2: float  # y
    q3: float  # z


# ── Publisher ─────────────────────────────────────────────────────────────────

class TorsoQuatPublisher:
    """
    Loads the stored mounting offset on startup, then continuously publishes
    the corrected torso quaternion.

    Parameters
    ----------
    domain_id   : CycloneDDS domain (must match imu_publisher).
    offset_path : Path to the .npy file written by calibrate.py.
    rate_hz     : Publishing rate.
    """

    TOPIC = "TorsoQuat"

    def __init__(self, domain_id: int = 0,
                 offset_path: str = OFFSET_PATH,
                 rate_hz: float = 200.0):
        self._rate_hz = rate_hz

        # Load offset [x, y, z, w] (scipy scalar-last convention)
        if not os.path.exists(offset_path):
            raise FileNotFoundError(
                f"Offset file not found: {offset_path}\n"
                "Run calibrate.py first to generate it."
            )
        mean_q = np.load(offset_path)
        self._R_mount_inv = Rotation.from_quat(mean_q).inv()

        euler = Rotation.from_quat(mean_q).as_euler("ZYX", degrees=True)
        print(f"Loaded offset from {offset_path}")
        print(f"  Offset (ZYX):  yaw={euler[0]:+.3f}°  "
              f"pitch={euler[1]:+.3f}°  roll={euler[2]:+.3f}°\n")

        self._imu = ImuListener(domain_id=domain_id)

        dp           = DomainParticipant(domain_id)
        _pub         = Publisher(dp)
        self._writer = DataWriter(_pub, Topic(dp, self.TOPIC, TorsoQuatMsg))

    # ── Correction helper ─────────────────────────────────────────────────────

    def _correct(self, imu_q_wxyz: tuple) -> TorsoQuatMsg:
        """Apply R_mount correction:  R_torso = R_imu * R_mount.inv()"""
        w, x, y, z = imu_q_wxyz
        R_imu   = Rotation.from_quat([x, y, z, w])
        R_torso = R_imu * self._R_mount_inv
        xo, yo, zo, wo = R_torso.as_quat()      # scipy returns scalar-last
        return TorsoQuatMsg(q0=wo, q1=xo, q2=yo, q3=zo)

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self) -> None:
        """Publish corrected torso quat indefinitely."""
        interval = 1.0 / self._rate_hz
        print(f"Publishing '{self.TOPIC}' at {self._rate_hz:.0f} Hz  (Ctrl+C to stop)\n")
        try:
            while True:
                t_start = time.time()
                q = self._imu.quat
                if q is not None:
                    self._writer.write(self._correct(q))
                elapsed = time.time() - t_start
                time.sleep(max(0.0, interval - elapsed))
        except KeyboardInterrupt:
            print("\nStopped.")


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    calibrate()
    pub = TorsoQuatPublisher()
    pub.run()
