"""
CycloneDDS listener for BWT901CL IMU data.

Uses on_data_available callbacks — purely event-driven, no polling loop.
cyclonedds.xml is loaded automatically from the package directory.
"""

import os
import time

from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic

try:
    from .imu_publisher import AccelMsg, GyroMsg, QuaternionMsg
except ImportError:
    from imu_publisher import AccelMsg, GyroMsg, QuaternionMsg

_dir = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault("CYCLONEDDS_URI", f"file://{_dir}/cyclonedds.xml")


def _make_listener(setter):
    """Return a Listener that calls setter(sample) on each valid received sample."""
    class _L(Listener):
        def on_data_available(self, reader):
            for sample in reader.take():
                if hasattr(sample, "sample_info") and not sample.sample_info.valid_data:
                    continue
                setter(sample)
    return _L()


class ImuListener:
    """
    Subscribes to Acc, Gyro, and Quat topics.
    Attributes are updated in-place by CycloneDDS callbacks — no thread needed.

    Properties:
        acc     (tuple[float, float, float] | None): latest (ax, ay, az) in m/s²
        ang_vel (tuple[float, float, float] | None): latest (wx, wy, wz) in °/s
        quat    (tuple[float, float, float, float] | None): latest (w, x, y, z)
    """

    def __init__(self, domain_id: int = 0):
        self._acc:     AccelMsg      | None = None
        self._ang_vel: GyroMsg       | None = None
        self._quat:    QuaternionMsg | None = None

        self._dp  = DomainParticipant(domain_id)
        self._sub = Subscriber(self._dp)

        self._rd_acc  = DataReader(self._sub, Topic(self._dp, "Acc",  AccelMsg),
                                   listener=_make_listener(lambda s: setattr(self, "_acc", s)))
        self._rd_gyro = DataReader(self._sub, Topic(self._dp, "Gyro", GyroMsg),
                                   listener=_make_listener(lambda s: setattr(self, "_ang_vel", s)))
        self._rd_quat = DataReader(self._sub, Topic(self._dp, "Quat", QuaternionMsg),
                                   listener=_make_listener(lambda s: setattr(self, "_quat", s)))

    @property
    def acc(self) -> tuple[float, float, float] | None:
        if self._acc is None:
            return None
        return (self._acc.ax, self._acc.ay, self._acc.az)

    @property
    def ang_vel(self) -> tuple[float, float, float] | None:
        if self._ang_vel is None:
            return None
        return (self._ang_vel.wx, self._ang_vel.wy, self._ang_vel.wz)

    @property
    def quat(self) -> tuple[float, float, float, float] | None:
        if self._quat is None:
            return None
        return (self._quat.q0, self._quat.q1, self._quat.q2, self._quat.q3)


if __name__ == "__main__":
    listener = ImuListener()
    print("Listening — Ctrl+C to stop.\n")

    try:
        while True:
            time.sleep(0.1)

            if listener.acc:
                ax, ay, az = listener.acc
                print(f"[Acc ] ax={ax:8.4f}  ay={ay:8.4f}  az={az:8.4f}")
            if listener.ang_vel:
                wx, wy, wz = listener.ang_vel
                print(f"[Gyro] wx={wx:8.4f}  wy={wy:8.4f}  wz={wz:8.4f}")
            if listener.quat:
                w, x, y, z = listener.quat
                print(f"[Quat] w={w:7.4f}  x={x:7.4f}  y={y:7.4f}  z={z:7.4f}")
            print()

    except KeyboardInterrupt:
        print("\nStopped.")
