import serial
import struct
import time
from dataclasses import dataclass
from typing import Optional

# ── Constants ────────────────────────────────────────────────────────────────
HEADER = 0x55
PACKET_LEN = 11  # each packet is 11 bytes

PACKET_TIME   = 0x50
PACKET_ACCEL  = 0x51
PACKET_GYRO   = 0x52
PACKET_ANGLE  = 0x53
PACKET_MAG    = 0x54

G = 9.8  # m/s²

# ── Data Classes ─────────────────────────────────────────────────────────────
@dataclass
class TimeData:
    year: int
    month: int
    day: int
    hour: int
    minute: int
    second: int
    millisecond: int

    def __str__(self):
        return (f"Time: 20{self.year:02d}-{self.month:02d}-{self.day:02d} "
                f"{self.hour:02d}:{self.minute:02d}:{self.second:02d}.{self.millisecond:03d}")

@dataclass
class AccelData:
    ax: float  # m/s²
    ay: float
    az: float
    temperature: float  # °C

    def __str__(self):
        return (f"Accel (m/s²) → X: {self.ax:8.4f}  Y: {self.ay:8.4f}  Z: {self.az:8.4f} "
                f"| Temp: {self.temperature:.2f}°C")

@dataclass
class GyroData:
    wx: float  # °/s
    wy: float
    wz: float
    voltage: float  # V

    def __str__(self):
        return (f"Gyro  (°/s)  → X: {self.wx:8.4f}  Y: {self.wy:8.4f}  Z: {self.wz:8.4f} "
                f"| Batt: {self.voltage:.2f}V")

@dataclass
class AngleData:
    roll: float   # °
    pitch: float  # °
    yaw: float    # °
    version: int

    def __str__(self):
        return (f"Angle (°)    → Roll: {self.roll:8.4f}  Pitch: {self.pitch:8.4f}  "
                f"Yaw: {self.yaw:8.4f}")

@dataclass
class MagData:
    hx: int  # raw units
    hy: int
    hz: int
    temperature: float  # °C

    def __str__(self):
        return (f"Mag   (raw)  → X: {self.hx:6d}  Y: {self.hy:6d}  Z: {self.hz:6d} "
                f"| Temp: {self.temperature:.2f}°C")

@dataclass
class QuaternionData:
    q0: float
    q1: float
    q2: float
    q3: float

    def __str__(self):
        return (f"Quat         → Q0: {self.q0:7.4f}  Q1: {self.q1:7.4f}  "
                f"Q2: {self.q2:7.4f}  Q3: {self.q3:7.4f}")


# ── Parser ────────────────────────────────────────────────────────────────────
def to_signed_short(high: int, low: int) -> int:
    """Combine high/low bytes into a signed 16-bit integer."""
    value = (high << 8) | low
    return struct.unpack('h', struct.pack('H', value))[0]

def verify_checksum(packet: bytes) -> bool:
    """Verify the packet checksum (sum of bytes 0-9, masked to 8 bits)."""
    return (sum(packet[:10]) & 0xFF) == packet[10]

def parse_packet(packet: bytes):
    """Parse a single 11-byte packet and return the appropriate data object."""
    if len(packet) != PACKET_LEN or packet[0] != HEADER:
        return None

    if not verify_checksum(packet):
        print(f"  [WARNING] Checksum mismatch for packet type 0x{packet[1]:02X}")
        return None

    ptype = packet[1]

    if ptype == PACKET_TIME:
        yy, mm, dd, hh, mn, ss, msl, msh = packet[2:10]
        ms = (msh << 8) | msl
        return TimeData(yy, mm, dd, hh, mn, ss, ms)

    elif ptype == PACKET_ACCEL:
        axl, axh, ayl, ayh, azl, azh, tl, th = packet[2:10]
        ax = to_signed_short(axh, axl) / 32768 * 16 * G
        ay = to_signed_short(ayh, ayl) / 32768 * 16 * G
        az = to_signed_short(azh, azl) / 32768 * 16 * G
        temp = to_signed_short(th, tl) / 100
        return AccelData(ax, ay, az, temp)

    elif ptype == PACKET_GYRO:
        wxl, wxh, wyl, wyh, wzl, wzh, vl, vh = packet[2:10]
        wx = to_signed_short(wxh, wxl) / 32768 * 2000
        wy = to_signed_short(wyh, wyl) / 32768 * 2000
        wz = to_signed_short(wzh, wzl) / 32768 * 2000
        voltage = ((vh << 8) | vl) / 100
        return GyroData(wx, wy, wz, voltage)

    elif ptype == PACKET_ANGLE:
        rl, rh, pl, ph, yl, yh, vl, vh = packet[2:10]
        roll  = to_signed_short(rh, rl)  / 32768 * 180
        pitch = to_signed_short(ph, pl)  / 32768 * 180
        yaw   = to_signed_short(yh, yl)  / 32768 * 180
        version = (vh << 8) | vl
        return AngleData(roll, pitch, yaw, version)

    elif ptype == PACKET_MAG:
        hxl, hxh, hyl, hyh, hzl, hzh, tl, th = packet[2:10]
        hx = to_signed_short(hxh, hxl)
        hy = to_signed_short(hyh, hyl)
        hz = to_signed_short(hzh, hzl)
        temp = to_signed_short(th, tl) / 100
        return MagData(hx, hy, hz, temp)

    return None


# ── Reader ────────────────────────────────────────────────────────────────────
class BWT901CLReader:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()

    def connect(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout
        )
        print(f"Connected to {self.port} @ {self.baudrate} baud")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected.")

    def _read_packets(self):
        """Read available bytes, extract complete 11-byte packets."""
        raw = self.ser.read(self.ser.in_waiting or 1)
        if raw:
            self._buf.extend(raw)

        packets = []
        while len(self._buf) >= PACKET_LEN:
            # Sync to header byte 0x55
            if self._buf[0] != HEADER:
                self._buf.pop(0)
                continue
            packet = bytes(self._buf[:PACKET_LEN])
            self._buf = self._buf[PACKET_LEN:]
            packets.append(packet)
        return packets

    def read_once(self) -> dict:
        """Block until one full set of readings arrives, return as dict."""
        results = {}
        while True:
            for packet in self._read_packets():
                data = parse_packet(packet)
                if data is not None:
                    results[type(data).__name__] = data
            # Return when we have at least one reading
            if results:
                return results

    def stream(self, duration: float = 10.0, callback=None, rate_hz: float = 1000.0):
        """
        Stream sensor data for `duration` seconds.
        Optional `callback(data_dict)` is called each cycle.
        `rate_hz` controls the polling rate (default 1000 Hz).
        """
        print(f"\nStreaming for {duration}s  (Ctrl+C to stop early)\n")
        interval = 1.0 / rate_hz
        t_end = time.time() + duration
        try:
            while time.time() < t_end:
                results = {}
                for packet in self._read_packets():
                    data = parse_packet(packet)
                    if data is not None:
                        results[type(data).__name__] = data

                if results:
                    if callback:
                        callback(results)
                    else:
                        _default_print(results)

                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nStopped by user.")


# ── Helpers ───────────────────────────────────────────────────────────────────
def _default_print(results: dict):
    print("─" * 75)
    for data in results.values():
        print(data)


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # ── CHANGE THIS to your actual serial port ──
    # Windows: "COM3", "COM4", ...
    # Linux:   "/dev/ttyUSB0", "/dev/rfcomm0" (Bluetooth)
    # macOS:   "/dev/tty.HC-06-DevB"
    PORT = "/dev/torsoimu"

    reader = BWT901CLReader(port=PORT)

    try:
        reader.connect()
        reader.stream(duration=30.0)   # stream for 30 seconds
    finally:
        reader.disconnect()