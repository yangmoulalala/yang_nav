import struct

import pytest

from chassis_comm.protocol import (
    Command, FRAME_SIZE, FrameParser, crc16_modbus, decode_frame, encode_frame,
)


def test_modbus_standard_vector():
    assert crc16_modbus(b'123456789') == 0x4B37
    assert crc16_modbus(b'') == 0xFFFF


def test_fixed_wire_vector_and_offsets():
    frame = encode_frame(Command(1.0, -2.0, 0.5, 1))
    assert frame == bytes.fromhex('a5 5a 0000803f 000000c0 0000003f 01 5bd6')
    assert len(frame) == FRAME_SIZE == 17
    assert frame[2:6] == struct.pack('<f', 1.0)
    assert frame[6:10] == struct.pack('<f', -2.0)
    assert frame[10:14] == struct.pack('<f', 0.5)
    assert frame[14] == 1
    assert struct.unpack('<H', frame[15:])[0] == crc16_modbus(frame[2:15])
    assert crc16_modbus(frame[:15]) != crc16_modbus(frame[2:15])
    assert decode_frame(frame) == Command(1.0, -2.0, 0.5, 1)


def test_zero_vector():
    assert encode_frame(Command(0, 0, 0)) == bytes.fromhex('a55a' + '00' * 13 + '03eb')


@pytest.mark.parametrize('field', ['vx', 'vy', 'yaw'])
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf'), 1e100])
def test_unrepresentable_float_is_rejected(field, value):
    values = dict(vx=0.0, vy=0.0, yaw=0.0)
    values[field] = value
    with pytest.raises(ValueError):
        encode_frame(Command(**values))


@pytest.mark.parametrize('flag', [-1, 2, 255, 256])
def test_undefined_actions_are_rejected(flag):
    with pytest.raises(ValueError):
        encode_frame(Command(0, 0, 0, flag))


def test_each_single_bit_error_is_rejected():
    frame = encode_frame(Command(0.125, -0.75, -1.25, 1))
    for offset in range(FRAME_SIZE):
        for bit in range(8):
            damaged = bytearray(frame)
            damaged[offset] ^= 1 << bit
            with pytest.raises(ValueError):
                decode_frame(damaged)
    for short in (frame[:-1], frame + b'\x00'):
        with pytest.raises(ValueError):
            decode_frame(short)


def test_crc_valid_invalid_payload_is_rejected():
    for payload in (struct.pack('<fffB', float('nan'), 0, 0, 0),
                    struct.pack('<fffB', 0, 0, 0, 2)):
        with pytest.raises(ValueError):
            decode_frame(b'\xa5\x5a' + payload + struct.pack('<H', crc16_modbus(payload)))


@pytest.mark.parametrize('chunk_size', range(1, 20))
def test_stream_resync_with_noise_truncated_frame_and_bad_crc(chunk_size):
    first, second = Command(1, -2, 0.5, 1), Command(-0.125, 0.25, -0.75, 0)
    bad = bytearray(encode_frame(first))
    bad[8] ^= 0x40
    # Includes overlapping SOF, a truncated candidate and back-to-back frames.
    stream = (b'noise\xa5' + encode_frame(first)[:9] + bytes(bad)
              + b'\xa5' + encode_frame(first) + encode_frame(second) + b'end\xa5')
    parser = FrameParser()
    received = []
    for offset in range(0, len(stream), chunk_size):
        received.extend(parser.feed(stream[offset:offset + chunk_size]))
    assert received == [first, second]
    assert parser.feed(encode_frame(second)[1:]) == [second]  # trailing split SOF


def test_partial_or_corrupt_input_does_not_publish_a_command():
    parser = FrameParser()
    frame = encode_frame(Command(0.25, 0.5, 1.5))
    assert parser.feed(frame[:16]) == []
    assert parser.feed(frame[16:]) == [decode_frame(frame)]
    bad = bytearray(frame)
    bad[14] ^= 1
    assert parser.feed(bad) == []
    assert parser.feed(frame) == [decode_frame(frame)]
