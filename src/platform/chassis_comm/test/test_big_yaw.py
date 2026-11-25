"""Feedback wire format, float32 boundaries and serial stream recovery."""

import math
import struct

import pytest

from chassis_comm.big_yaw import BigYawParser, FRAME_SIZE, MIN_ANGLE, decode_frame
from chassis_comm.protocol import crc16_modbus


def frame(angle):
    payload = struct.pack('<f', angle)
    return b'\xaa\x55' + payload + struct.pack('<H', crc16_modbus(payload))


def test_fixed_vectors_payload_crc_and_endianness():
    assert FRAME_SIZE == 8
    assert decode_frame(bytes.fromhex('aa 55 00 00 00 00 00 24')) == 0.0
    assert decode_frame(bytes.fromhex('aa 55 00 00 80 3f 21 f4')) == 1.0
    assert decode_frame(bytes.fromhex('aa 55 00 00 80 bf 20 54')) == -1.0
    payload = struct.pack('<f', 0.5)
    with pytest.raises(ValueError, match='CRC'):
        decode_frame(b'\xaa\x55' + payload
                     + struct.pack('<H', crc16_modbus(b'\xaa\x55' + payload)))
    with pytest.raises(ValueError, match='CRC'):
        decode_frame(bytes.fromhex('aa 55 00 00 80 3f f4 21'))


@pytest.mark.parametrize('angle', [float('nan'), float('inf'), -float('inf'),
                                  math.pi, -3.142, 4.0, -4.0])
def test_crc_valid_invalid_angle_is_rejected(angle):
    with pytest.raises(ValueError, match='angle'):
        decode_frame(frame(angle))


def test_float32_minus_pi_is_valid_and_plus_pi_is_excluded():
    assert decode_frame(frame(-math.pi)) == MIN_ANGLE
    assert decode_frame(frame(3.1415925)) == pytest.approx(3.1415925)
    assert decode_frame(frame(-0.0)) == 0.0


def test_all_single_bit_errors_and_wrong_lengths_are_rejected():
    original = frame(-1.25)
    for offset in range(FRAME_SIZE):
        for bit in range(8):
            damaged = bytearray(original)
            damaged[offset] ^= 1 << bit
            with pytest.raises(ValueError):
                decode_frame(damaged)
    for data in (original[:-1], original + b'\x00'):
        with pytest.raises(ValueError):
            decode_frame(data)


@pytest.mark.parametrize('chunk_size', range(1, 18))
def test_stream_resync_and_partial_header(chunk_size):
    bad = bytearray(frame(0.5))
    bad[3] ^= 0x80
    stream = (b'noise\xaa' + frame(2.0)[:5] + bytes(bad) + frame(float('nan'))
              + b'\xaa' + frame(1.0) + frame(-2.0) + b'end\xaa')
    parser = BigYawParser()
    received = []
    for offset in range(0, len(stream), chunk_size):
        received.extend(parser.feed(stream[offset:offset + chunk_size]))
    assert received == [1.0, -2.0]
    assert parser.feed(frame(0.5)[1:]) == [0.5]


def test_reset_discards_an_old_partial_frame():
    parser = BigYawParser()
    assert parser.feed(frame(1.0)[:5]) == []
    parser.reset()
    assert parser.feed(frame(1.0)[5:]) == []
    assert parser.feed(frame(-1.0)) == [-1.0]
