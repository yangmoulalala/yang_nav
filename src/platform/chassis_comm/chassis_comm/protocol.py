"""MiniPC navigation frames; CRC16/MODBUS covers SOF + payload.

No ROS or serial dependencies. The navigation bridge sends A3 and receives A4.
"""

from dataclasses import dataclass
import math
import struct


@dataclass(frozen=True)
class Command:
    """A3: radar-frame velocity, relative radar turn (rad), world position (m)."""

    vx: float
    vy: float
    yaw: float  # Desired turn relative to current radar +X; CCW positive, rad.
    is_through_hole: int = 0
    x: float = 0.0
    y: float = 0.0

    @property
    def action_flag(self):
        """Compatibility for the existing simulation-only edge command state."""
        return self.is_through_hole


@dataclass(frozen=True)
class NavigationTarget:
    yaw: float  # Measured chassis +X -> radar +X angle; CCW positive, rad.
    x: float
    y: float

    @property
    def intent(self):
        return 'home' if self.x == 0 and self.y == 0 else 'target'


# The navigation API keeps vx/vy/yaw first for callers; the wire puts the flag first.
FORMATS = {
    0xA3: (struct.Struct('<Bfffff'), Command),
    0xA4: (struct.Struct('<fff'), NavigationTarget),
}
FRAME_SIZES = {header: layout.size + 3 for header, (layout, _) in FORMATS.items()}
FRAME_SIZE = FRAME_SIZES[0xA3]
DISCARD_SIZES = {0xA1: 17, 0xA2: 24}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def encode_frame(message) -> bytes:
    if isinstance(message, Command):
        header = 0xA3
        flags = (message.is_through_hole,)
        floats = (message.vx, message.vy, message.yaw, message.x, message.y)
    elif isinstance(message, NavigationTarget):
        header = 0xA4
        flags = ()
        floats = (message.yaw, message.x, message.y)
    else:
        raise ValueError('unsupported MiniPC message type')
    if any(flag not in (0, 1) for flag in flags):
        raise ValueError('protocol flags must be 0 or 1')
    if not all(math.isfinite(value) for value in floats):
        raise ValueError('protocol floats must be finite')
    try:
        data = bytes([header]) + FORMATS[header][0].pack(*flags, *floats)
    except (OverflowError, struct.error) as exc:
        raise ValueError('message cannot be represented as float32/uint8') from exc
    return data + struct.pack('<H', crc16_modbus(data))


def decode_frame(frame: bytes):
    if not frame or frame[0] not in FORMATS or len(frame) != FRAME_SIZES[frame[0]]:
        raise ValueError('expected a complete A3/A4 navigation frame')
    if crc16_modbus(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
        raise ValueError('CRC-16/MODBUS mismatch')
    layout, cls = FORMATS[frame[0]]
    values = layout.unpack(frame[1:-2])
    if cls is Command:
        flag, vx, vy, yaw, x, y = values
        message = Command(vx, vy, yaw, flag, x, y)
    else:
        message = cls(*values)
    encode_frame(message)  # Reject CRC-valid NaN/Inf and undefined flags too.
    return message


class FrameParser:
    """Drop one byte per unknown SOF or bad CRC and consume valid frames whole.

    Retired envelopes with valid CRC are discarded without decoding their payload.
    """

    def __init__(self, headers=None):
        self._headers = set(FORMATS if headers is None else headers)
        if not self._headers <= FORMATS.keys():
            raise ValueError('unknown frame header')
        self._buffer = bytearray()

    def reset(self):
        self._buffer.clear()

    def feed(self, data: bytes) -> list:
        self._buffer.extend(data)
        messages = []
        while self._buffer:
            header = self._buffer[0]
            size = FRAME_SIZES.get(header, DISCARD_SIZES.get(header))
            if size is None:
                del self._buffer[0]
                continue
            if len(self._buffer) < size:
                break
            if header in DISCARD_SIZES:
                valid = crc16_modbus(self._buffer[:size - 2]) == int.from_bytes(self._buffer[size - 2:size], 'little')
                del self._buffer[:size if valid else 1]
                continue
            try:
                message = decode_frame(bytes(self._buffer[:size]))
            except ValueError:
                del self._buffer[0]
                continue
            if header in self._headers:
                messages.append(message)
            del self._buffer[:size]
        return messages
