import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Event
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from .server import Console, read_landmarks
from .settings import Settings, TERRAIN_FILES
from .terrain import render_match_reference


def test_stop_interrupts_wait_and_closes_only_managed_processes(tmp_path):
    ros = Mock()
    ros.node_names.return_value = {'/backend_node'}
    waiting = Event()

    def snapshot():
        waiting.set()
        return {}

    ros.snapshot.side_effect = snapshot
    console = Console(tmp_path, ros, 8766)
    nav = console.processes
    owned = Mock()
    owned.child.poll.return_value = None
    nav.entries['optimizer'] = owned
    nav.launch = Mock()
    with ThreadPoolExecutor(max_workers=2) as pool:
        starting = pool.submit(console.action, '/api/process/navigation/start', {})
        assert waiting.wait(2)
        stopping = pool.submit(console.action, '/api/stack/stop', {})
        with pytest.raises(ValueError, match='启动已终止'):
            starting.result(timeout=2)
        assert '关闭' in stopping.result(timeout=2)
    nav.launch.assert_not_called()
    owned.stop.assert_called_once()
    assert not nav.cancel_start.is_set()
    assert console.action('/api/tune', {'profile': 'sim', 'values': {}})


def test_favorites_persist_as_platform_specific_pairs(tmp_path):
    prior = tmp_path / 'prior'
    (prior / 'clouds').mkdir(parents=True)
    (prior / 'keyframes.txt').touch()
    terrain = tmp_path / 'terrain'
    terrain.mkdir()
    for name in TERRAIN_FILES:
        (terrain / name).touch()
    settings = Settings(tmp_path)
    pair = dict(prior_map=str(prior), terrain_dir=str(terrain))
    for _ in range(2):
        settings.favorite(dict(pair, profile='real', remove=False))
    assert Settings(tmp_path).data['real']['favorites'] == [pair]
    assert settings.data['sim']['favorites'] == []
    assert settings.profile == 'sim'
    settings.favorite(dict(pair, profile='real', remove=True))
    assert not Settings(tmp_path).data['real']['favorites']


def test_preview_never_uses_landmarks_from_a_different_prior(tmp_path):
    prior = tmp_path / 'prior'
    meta = tmp_path / 'terrain.json'
    meta.write_text(json.dumps(dict(frame='map', source=str(prior), landmarks={'match': [0, 0]})))
    assert read_landmarks(meta, prior, 1, 1)['landmarks'] == {'match': [0, 0]}
    assert read_landmarks(meta, Path('/different-map'), 1, 1) is None


def test_reference_overlay_uses_map_origin_and_flipped_image_row(tmp_path):
    source = np.zeros((100, 120, 3), dtype='uint8')
    assert cv2.imwrite(str(tmp_path / 'terrain.png'), source)
    grid = [0.5, [-10., -20.], [120, 100]]
    data = render_match_reference(tmp_path, grid, [0.25, -14.75])
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    assert image[89, 20].min() == 255
    assert not image[10, 20].any()
    with pytest.raises(ValueError, match='范围外'):
        render_match_reference(tmp_path, grid, [111., 222.])
