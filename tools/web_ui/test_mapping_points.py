import pytest

from .settings import Settings


def test_mapping_points_persist_per_profile(tmp_path):
    settings = Settings(tmp_path)
    points = {'home': [0, -2.5], 'target': [3.5, 0]}
    settings.setup({'profile': 'real', 'mapping_points': points})
    settings.setup({'profile': 'real', 'mode': 'mapping'})
    loaded = Settings(tmp_path)
    assert loaded.active['mapping_points'] == points
    assert loaded.data['sim']['mapping_points'] == {}
    loaded.setup({'profile': 'real', 'mapping_points': {}})
    assert Settings(tmp_path).active['mapping_points'] == {}


@pytest.mark.parametrize('points', [None, {'unknown': [0, 0]}, {'home': [2]},
                                    {'target': [False, 0]}, {'target': [float('inf'), 0]}])
def test_invalid_mapping_points_leave_config_unchanged(tmp_path, points):
    settings = Settings(tmp_path)
    with pytest.raises(ValueError):
        settings.setup({'profile': 'real', 'mapping_points': points})
    assert not settings.file.exists()
