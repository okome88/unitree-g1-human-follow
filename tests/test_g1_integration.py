"""Optional asset-dependent tests, explicitly skipped when assets have not been fetched."""
from pathlib import Path
import numpy as np
import pytest
from cooperative_carry.config import load_config
from cooperative_carry.environment import CarryEnvironment

pytestmark = pytest.mark.skipif(not Path('.cache/robots/unitree_g1/scene_g1.xml').is_file(),
                                reason="Run g1-carry fetch-assets for G1 integration tests")


def test_g1_unloaded_walk():
    config = load_config('configs/walk_g1.toml')
    config.duration = 5
    env = CarryEnvironment(config)
    while not env.done:
        result = env.step()
    assert result.reason == 'time_limit'
    assert result.metrics['robot_x'] > .3
    assert np.isfinite(env.data.qpos).all()


def test_g1_light_carry():
    env = CarryEnvironment(load_config('configs/carry_g1.toml'))
    payload_geom = env.model.geom('payload_box').id
    assert all(payload_geom not in (c.geom1, c.geom2) for c in env.data.contact)
    while not env.done:
        result = env.step()
    assert result.reason == 'goal_reached'
    assert result.metrics['payload_tilt_deg'] < 10
    assert result.metrics['grasp_error'] < .1
