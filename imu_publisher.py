"""
CycloneDDS publisher for BWT901CL IMU data.

Topics published:
  - "Acc"  → AccelMsg      (acceleration)
  - "Gyro" → GyroMsg       (angular velocity)
  - "Quat" → QuaternionMsg (orientation)
"""

import math
import os
import dataclasses
from dataclasses import dataclass

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.topic import Topic
from cyclonedds.idl import IdlStruct

try:
    from .utils.BWT901C import AccelData, AngleData, GyroData, QuaternionData, BWT901CLReader
except ImportError:
    from utils.BWT901C import AccelData, AngleData, GyroData, QuaternionData, BWT901CLReader

_dir = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault("CYCLONEDDS_URI", f"file://{_dir}/cyclonedds.xml")


# ── Quaternion helpers ────────────────────────────────────────────────────────

def euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> QuaternionData:
    """Convert roll/pitch/yaw (degrees, ZYX intrinsic) to a unit quaternion [w, x, y, z]."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)

    q0 = cr * cp * cy + sr * sp * sy  # w
    q1 = sr * cp * cy - cr * sp * sy  # x
    q2 = cr * sp * cy + sr * cp * sy  # y
    q3 = cr * cp * sy - sr * sp * cy  # z
    return QuaternionData(q0, q1, q2, q3)


def _apply_z_rotation(q: QuaternionData, angle_deg: float) -> QuaternionData:
    """Pre-multiply q by a pure Z-axis rotation of angle_deg degrees (q_offset * q * q_offset^T)."""
    half = math.radians(angle_deg) / 2
    cz, sz = math.cos(half), math.sin(half)
    # q_offset = [cz, 0, 0, sz]
    w_ = cz * q.q0 - sz * q.q3
    x_ = cz * q.q1 - sz * q.q2
    y_ = cz * q.q2 + sz * q.q1
    z_ = cz * q.q3 + sz * q.q0
    w = w_ * cz + z_ * sz
    x = x_ * cz - y_ * sz
    y = y_ * cz + x_ * sz
    z = z_ * cz - w_ * sz
    return QuaternionData(w, x, y, z)


# ── DDS IDL types ─────────────────────────────────────────────────────────────

@dataclass
class AccelMsg(IdlStruct, typename="Acc"):
    ax: float
    ay: float
    az: float
    temperature: float


@dataclass
class GyroMsg(IdlStruct, typename="Gyro"):
    wx: float
    wy: float
    wz: float
    voltage: float


@dataclass
class QuaternionMsg(IdlStruct, typename="Quat"):
    q0: float
    q1: float
    q2: float
    q3: float


# Maps sensor dataclass type → DDS message type (field names are identical)
_MSG_TYPE: dict = {
    AccelData:      AccelMsg,
    GyroData:       GyroMsg,
    QuaternionData: QuaternionMsg,
}


# ── Publisher service ──────────────────────────────────────────────────────────

class ImuPublisher:
    def __init__(self, port: str, baudrate: int = 115200, domain_id: int = 0,
                 rotation_offset_deg: float = -90.0):
        """
        Args:
            rotation_offset_deg: Z-axis rotation (degrees) applied when converting
                Euler angles to quaternion. Default 90° maps sensor +Y → robot +X
                and sensor -X → robot +Y.
        """
        self.rotation_offset_deg = rotation_offset_deg
        self.reader = BWT901CLReader(port=port, baudrate=baudrate)

        self.dp = DomainParticipant(domain_id)
        pub = Publisher(self.dp)

        self.writers = {
            AccelData:      DataWriter(pub, Topic(self.dp, "Acc",  AccelMsg)),
            GyroData:       DataWriter(pub, Topic(self.dp, "Gyro", GyroMsg)),
            QuaternionData: DataWriter(pub, Topic(self.dp, "Quat", QuaternionMsg)),
        }

    def _publish(self, results: dict):
        # Derive quaternion from angle data with rotation offset
        if "QuaternionData" not in results and "AngleData" in results:
            a = results["AngleData"]
            q = euler_to_quaternion(a.roll, a.pitch, a.yaw)
            results = dict(results)
            results["QuaternionData"] = _apply_z_rotation(q, self.rotation_offset_deg)

        for data in results.values():
            dtype = type(data)
            writer = self.writers.get(dtype)
            if writer is None:
                continue  # skip unregistered types (TimeData, AngleData, MagData)
            writer.write(_MSG_TYPE[dtype](**dataclasses.asdict(data)))

    def run(self, duration: float = 0.0):
        """
        Stream IMU data and publish to DDS topics.

        Args:
            duration: How many seconds to run. 0 means run indefinitely.
        """
        self.reader.connect()
        print("ImuPublisher running — Ctrl+C to stop.\n")
        try:
            self.reader.stream(duration=duration or float("inf"),
                               callback=self._publish, rate_hz=200)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            self.reader.disconnect()


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    PORT = "/dev/torsoimu"   # adjust to your device

    publisher = ImuPublisher(port=PORT)
    publisher.run()          # run until Ctrl+C
