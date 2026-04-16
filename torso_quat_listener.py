"""
Test listener for the "TorsoQuat" CycloneDDS topic.

Prints the calibrated torso quaternion and its ZYX Euler angles (°) at ~10 Hz.
Run this while torso_quat_publisher is active to verify calibration output.
"""

import os
import time

from scipy.spatial.transform import Rotation
from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic

_dir = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault("CYCLONEDDS_URI", f"file://{_dir}/cyclonedds.xml")

try:
    from .torso_quat_publisher import TorsoQuatMsg
except ImportError:
    from torso_quat_publisher import TorsoQuatMsg


class TorsoQuatListener:
    """
    Subscribes to "TorsoQuat" and exposes the latest sample.

    Properties
    ----------
    quat  : (w, x, y, z) or None
    euler : (yaw_deg, pitch_deg, roll_deg) ZYX or None
    """

    def __init__(self, domain_id: int = 0):
        self._msg: TorsoQuatMsg | None = None

        dp   = DomainParticipant(domain_id)
        sub  = Subscriber(dp)

        class _L(Listener):
            def on_data_available(inner_self, reader):
                for sample in reader.take():
                    if hasattr(sample, "sample_info") and not sample.sample_info.valid_data:
                        continue
                    self._msg = sample

        self._reader = DataReader(sub, Topic(dp, "TorsoQuat", TorsoQuatMsg), listener=_L())

    @property
    def quat(self) -> tuple[float, float, float, float] | None:
        if self._msg is None:
            return None
        m = self._msg
        return (m.q0, m.q1, m.q2, m.q3)   # (w, x, y, z)

    @property
    def euler(self) -> tuple[float, float, float] | None:
        q = self.quat
        if q is None:
            return None
        w, x, y, z = q
        yaw, pitch, roll = Rotation.from_quat([x, y, z, w]).as_euler("ZYX", degrees=True)
        return (yaw, pitch, roll)


if __name__ == "__main__":
    listener = TorsoQuatListener()
    print("Listening for TorsoQuat — Ctrl+C to stop.\n")

    try:
        while True:
            time.sleep(0.1)

            q = listener.quat
            e = listener.euler
            if q is None:
                print("[TorsoQuat] waiting ...")
                continue

            w, x, y, z = q
            yaw, pitch, roll = e
            print(
                f"[TorsoQuat]  "
                f"w={w:+7.4f}  x={x:+7.4f}  y={y:+7.4f}  z={z:+7.4f}  │  "
                f"yaw={yaw:+7.2f}°  pitch={pitch:+7.2f}°  roll={roll:+7.2f}°"
            )

    except KeyboardInterrupt:
        print("\nStopped.")
