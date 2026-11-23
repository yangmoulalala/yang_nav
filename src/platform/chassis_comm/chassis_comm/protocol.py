"""Fixed 17-byte, little-endian chassis control frames.

CRC-16/MODBUS covers bytes [2:15], excluding SOF and the CRC itself.
This module has no ROS or serial dependency and can also test an MCU receiver.
"""

from dataclasses import dataclass
import math
import struct


SOF = b'\xa5\x5a'
PAYLOAD = struct.Struct('<fffB')
FRAME_SIZE = 17


@dataclass(frozen=True)
class Command:
    vx: float
    vy: float
    yaw: float  # Target rotation relative to the current body heading (rad).
    action_flag: int = 0  # Rising edge requests the hole-passage action.


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def encode_frame(command: Command) -> bytes:
    if not all(math.isfinite(v) for v in (command.vx, command.vy, command.yaw)):
        raise ValueError('vx, vy and yaw must be finite')
    if command.action_flag not in (0, 1):
        raise ValueError('action_flag must be 0 or 1')
    try:
        payload = PAYLOAD.pack(command.vx, command.vy, command.yaw, command.action_flag)
    except (OverflowError, struct.error) as exc:
        raise ValueError('command cannot be represented as float32/uint8') from exc
    return SOF + payload + struct.pack('<H', crc16_modbus(payload))


def decode_frame(frame: bytes) -> Command:
    if len(frame) != FRAME_SIZE or frame[:2] != SOF:
        raise ValueError('expected a 17-byte frame beginning A5 5A')
    payload = frame[2:15]
    if crc16_modbus(payload) != struct.unpack('<H', frame[15:17])[0]:
        raise ValueError('CRC-16/MODBUS mismatch')
    command = Command(*PAYLOAD.unpack(payload))
    # CRC correctness alone must not admit NaN, infinity or an undefined action.
    encode_frame(command)
    return command


class FrameParser:
    """Yield only validated commands; recover after noise, drops or bad CRCs."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[Command]:
        self._buffer.extend(data)
        commands = []
        while True:
            start = self._buffer.find(SOF)
            if start < 0:
                # Preserve a split SOF, but never retain arbitrary noise.
                self._buffer[:] = b'\xa5' if self._buffer[-1:] == b'\xa5' else b''
                break
            del self._buffer[:start]
            if len(self._buffer) < FRAME_SIZE:
                break
            try:
                command = decode_frame(bytes(self._buffer[:FRAME_SIZE]))
            except ValueError:
                # A valid SOF may occur inside the rejected candidate.
                del self._buffer[0]
                continue
            commands.append(command)
            del self._buffer[:FRAME_SIZE]
        return commands
