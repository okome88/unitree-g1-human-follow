from functools import partial
import os
from pathlib import Path
import unittest
import importlib.util

import mujoco
import numpy as np
if importlib.util.find_spec('jupedsim') is None or importlib.util.find_spec('shapely') is None:
    raise unittest.SkipTest('Install requirements-station.txt for station tests')
from shapely.geometry import Point

from human_follow.station import blend_people, observation
from human_follow.station_world import StationLayout, StationCrowd, decorate_station
from human_follow.sim import scene
from human_follow.planner import Follower


class StationTests(unittest.TestCase):
    def setUp(self):
        self.layout = StationLayout()

    def test_visibility_geometry_people_and_fov(self):
        self.assertTrue(self.layout.visible(np.zeros(3), [4, 0]))
        self.assertFalse(self.layout.visible(np.zeros(3), [10, 6], sensor_range=20))
        self.assertFalse(self.layout.visible(np.zeros(3), [4, 0], [[2, 0, 0, 0, .25]]))
        self.assertFalse(self.layout.visible(np.zeros(3), [-1, 0]))
        self.assertFalse(self.layout.visible(np.zeros(3), [9, 0]))

    def test_hidden_target_does_not_leak_truth(self):
        target = (np.array([12., 2.]), np.array([.4, .2]), 1.)
        obs = observation(self.layout, np.zeros(3), target, [], 100., 4.)
        self.assertFalse(obs.visible)
        np.testing.assert_array_equal(obs.position, 0)
        np.testing.assert_array_equal(obs.velocity, 0)
        np.testing.assert_array_equal(Follower().command(np.zeros(3), obs), 0)

    def test_spawn_has_no_cross_room_interpolation(self):
        result = blend_people([None], [[-1, 2, .8, 0, .25]], .2)
        np.testing.assert_allclose(result[0][:2], [-1, 2])
        self.assertEqual(blend_people([[25, 2, .8, 0, .25]], [None], .5), [None])

    def test_layout_obstacles_and_clearance(self):
        self.assertGreater(self.layout.clearance([0, 0]), 1.)
        self.assertLess(self.layout.clearance([5, 3]), 0.)
        near = self.layout.obstacles(np.array([10., 0.]), [])
        self.assertGreater(len(near), 0)
        for fixture in self.layout.fixtures:
            self.assertTrue(any(np.linalg.norm(np.array(o[:2])-[fixture.x, fixture.y]) <= o[4]+1e-9
                                for o in self.layout.discs))

    def test_crowd_seed_and_motion_with_robot_proxy(self):
        a = StationCrowd(self.layout, count=20, seed=5)
        b = StationCrowd(self.layout, count=20, seed=5)
        for _ in range(15):
            a.step([0, 0]); b.step([0, 0])
        np.testing.assert_allclose(a.target()[0], b.target()[0])
        self.assertGreater(a.target()[0][0], 2.3)
        for pa, pb in zip(a.people(), b.people()):
            np.testing.assert_allclose(pa, pb)
            self.assertGreater(self.layout.walkable.boundary.distance(Point(*pa[:2])), .24)
        self.assertEqual(a.shadow_update_failures, 0)

    def test_target_pause(self):
        c = StationCrowd(self.layout, count=0)
        c.step([0, 0], stop_at=0., stop_for=1.)
        np.testing.assert_allclose(c.target()[0], [2, 0], atol=1e-8)

    def test_arrival_wave_spawns_over_time(self):
        c = StationCrowd(self.layout, count=20, pattern='arrival-wave')
        self.assertEqual(sum(p is not None for p in c.people()), 10)
        for _ in range(140):
            c.step([0, 0])
        self.assertGreater(sum(p is not None for p in c.people()), 10)

    @unittest.skipUnless(os.environ.get('G1_STATION_RUNTIME'), 'optional official scene integration')
    def test_official_dofs_and_fixture_contacts(self):
        model, _, _ = scene(Path(os.environ['G1_STATION_RUNTIME']), decorate=partial(
            decorate_station, count=50, seed=42))
        self.assertEqual((model.nu, model.nq, model.nv), (29, 36, 35))
        data = mujoco.MjData(model)
        data.qpos[:3] = [10., 1.5, .8]
        mujoco.mj_forward(model, data)
        collisions = [(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom1)) or '',
                       mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom2)) or '')
                      for c in data.contact if c.dist < 0]
        self.assertTrue(any('station_collision_gate_' in x or 'station_collision_gate_' in y
                            for x, y in collisions))


if __name__ == '__main__':
    unittest.main()
