"""Offline illustration only: overlay a recorded relative TF track on an annotated terrain PNG."""

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw


def read_track(bag, offset, duration):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    import yaml

    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if not any(entry['topic_metadata']['name'] == '/tf' and entry['message_count'] > 0
               for entry in metadata['topics_with_message_count']):
        raise ValueError('录包没有 /tf，不能用地图定位输出代替独立里程计')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf']))
    start = metadata['starting_time']['nanoseconds_since_epoch'] + int(offset * 1e9)
    end = start + int(duration * 1e9)
    track = []
    previous = None
    while reader.has_next():
        _, payload, received = reader.read_next()
        if received < start:
            continue
        if received >= end:
            break
        for item in deserialize_message(payload, TFMessage).transforms:
            if (item.header.frame_id, item.child_frame_id) != ('lio_odom', 'gimbal_nav'):
                continue
            stamp = item.header.stamp.sec * 10**9 + item.header.stamp.nanosec
            if previous is not None and stamp < previous:
                raise ValueError('所选时间段有时钟回退，请改用单一连续段')
            if stamp == previous:
                continue
            previous = stamp
            rotation = item.transform.rotation
            position = item.transform.translation
            norm = math.hypot(rotation.x, rotation.y, rotation.z, rotation.w)
            if not math.isclose(norm, 1, abs_tol=0.001) or not all(map(math.isfinite, (position.x, position.y))):
                raise ValueError('录包位姿无效')
            yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                             1 - 2 * (rotation.y ** 2 + rotation.z ** 2))
            track.append((position.x, position.y, yaw))
    if len(track) < 2:
        raise ValueError('所选时间段缺少连续的 lio_odom → gimbal_nav 样本')
    return track


def overlay(terrain, track):
    meta = json.loads((terrain / 'terrain.json').read_text())
    points = meta['landmarks']
    if meta['frame'] != 'map' or 'match' not in points or not isinstance(meta.get('match_yaw_deg'), (int, float)):
        raise ValueError('需要 map 系参考点和显式 yaw')
    origin_x, origin_y = meta['origin']
    scale = meta['resolution']
    if not math.isfinite(scale) or scale <= 0 or not math.isfinite(meta['match_yaw_deg']):
        raise ValueError('地图参数无效')
    with Image.open(terrain / 'terrain.png') as source:
        if list(source.size) != meta['size']:
            raise ValueError('PNG 与地图栅格大小不一致')
        image = source.convert('RGB')

    def pixel(east, north):
        return math.floor((east - origin_x) / scale), image.height - 1 - math.floor((north - origin_y) / scale)

    anchor_x, anchor_y = points['match']
    first_x, first_y, first_yaw = track[0]
    theta = math.radians(meta['match_yaw_deg']) - first_yaw
    cosine, sine = math.cos(theta), math.sin(theta)
    path = [pixel(anchor_x + cosine * (east - first_x) - sine * (north - first_y),
                  anchor_y + sine * (east - first_x) + cosine * (north - first_y)) for east, north, _ in track]
    draw = ImageDraw.Draw(image)
    draw.line(path, fill='cyan', width=3)
    for name, position in points.items():
        col, row = pixel(*position)
        if not (0 <= col < image.width and 0 <= row < image.height):
            raise ValueError(f'标注 {name} 在地形图范围外')
        draw.ellipse((col - 3, row - 3, col + 3, row + 3), fill='yellow', outline='black')
        draw.text((col + 5, row - 12 if name == 'match' else row + 2), name, fill='white', stroke_width=1, stroke_fill='black')
    draw.rectangle((0, 0, image.width, 22), fill='black')
    draw.text((5, 4), 'OFFLINE / assumed anchor / cyan: relative track / not a localization result', fill='white')
    return image


def main():
    parser = argparse.ArgumentParser(description='把录包时间段的第一帧假设为参考位姿，仅生成离线图片，不发布 ROS 消息。')
    parser.add_argument('bag', type=Path)
    parser.add_argument('terrain', type=Path)
    parser.add_argument('png', type=Path)
    parser.add_argument('--offset', type=float, required=True, help='从录包开始计的秒数；首个有效样本作为假设起点')
    parser.add_argument('--duration', type=float, default=30, help='选择一段无里程计重置的记录（秒）')
    args = parser.parse_args()
    if not (math.isfinite(args.offset) and args.offset >= 0 and math.isfinite(args.duration) and args.duration > 0):
        parser.error('时间段无效')
    try:
        track = read_track(args.bag, args.offset, args.duration)
        image = overlay(args.terrain, track)
        with args.png.open('xb') as output:
            image.save(output, format='PNG')
    except (OSError, ValueError, TypeError, KeyError, RuntimeError) as error:
        parser.exit(1, f'{error}\n')
    print(f'{len(track)} samples, {args.png} (offline, assumed anchor)')


if __name__ == '__main__':
    main()
