"""8-byte measured big-yaw feedback, independent of ROS and serial I/O.

A5 5A | float32 LE (rad, turret relative to chassis) | CRC16/MODBUS LE.
CRC covers only the four payload bytes. The MCU sends at 200 Hz while online.
"""

import math
import struct

from .protocol import SOF, crc16_modbus


FRAME_SIZE = 8
# -pi rounds slightly below mathematical -pi when encoded as float32.
MIN_ANGLE = struct.unpack('<f', struct.pack('<f', -math.pi))[0]


def decode_frame(frame: bytes) -> float:
    if len(frame) != FRAME_SIZE or frame[:2] != SOF:
        raise ValueError('expected an 8-byte big-yaw frame beginning A5 5A')
    payload = frame[2:6]
    if crc16_modbus(payload) != struct.unpack('<H', frame[6:8])[0]:
        raise ValueError('big-yaw CRC-16/MODBUS mismatch')
    angle = struct.unpack('<f', payload)[0]
    if not math.isfinite(angle) or not MIN_ANGLE <= angle < math.pi:
        raise ValueError('big-yaw angle must be finite and in [-pi, pi) (float32)')
    return angle


class BigYawParser:
    """Reassemble split frames and resync bytewise after noise or invalid data."""

    def __init__(self):
        self._buffer = bytearray()

    def reset(self):
        self._buffer.clear()

    def feed(self, data: bytes) -> list[float]:
        self._buffer.extend(data)
        angles = []
        while True:
            start = self._buffer.find(SOF)
            if start < 0:
                self._buffer[:] = b'\xa5' if self._buffer[-1:] == b'\xa5' else b''
                break
            del self._buffer[:start]
            if len(self._buffer) < FRAME_SIZE:
                break
            try:
                angle = decode_frame(bytes(self._buffer[:FRAME_SIZE]))
            except ValueError:
                del self._buffer[0]
                continue
            angles.append(angle)
            del self._buffer[:FRAME_SIZE]
        return angles
