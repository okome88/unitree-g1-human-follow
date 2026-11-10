import tempfile
import os
import hashlib
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from human_follow.follow_bench import observed_target, prepare_config, scan_obstacles, run_trial, verify_files
from human_follow.planner import Follower


class BenchmarkTests(unittest.TestCase):
    def test_pinned_file_verification_allows_crlf_and_rejects_changes(self):
        content = b'original\n'
        digest = hashlib.sha1(b'blob 9\0' + content).hexdigest()
        tree = f'100644 blob {digest}\tworld.yaml\0'.encode()
        with tempfile.TemporaryDirectory() as directory, patch(
                'human_follow.follow_bench.subprocess.check_output', return_value=tree):
            root = Path(directory)
            (root / 'world.yaml').write_bytes(b'original\r\n')
            verify_files(root, ['world.yaml'])
            (root / 'world.yaml').write_bytes(b'modified\n')
            with self.assertRaises(ValueError):
                verify_files(root, ['world.yaml'])

    @unittest.skipUnless(os.environ.get('FOLLOW_BENCH_ROOT'), 'optional upstream integration')
    def test_repeated_environment_has_consistent_motion_logs(self):
        os.environ['MPLBACKEND'] = 'Agg'
        import irsim
        source = Path(os.environ['FOLLOW_BENCH_ROOT']) / 'socialRPF/example/robot_person_following/config_target_traj/uTurn_back_1.yaml'
        with tempfile.TemporaryDirectory() as directory:
            reports = [run_trial(irsim, source, Path(directory) / str(i), 0, 20, 125, 1.5, .4)
                       for i in range(2)]
        for report in reports:
            self.assertGreater(report['metrics']['avg_velocity'], .1)
            self.assertGreater(report['metrics']['path_length'], .5)
        self.assertEqual(reports[0]['metrics'], reports[1]['metrics'])

    def test_lost_target_has_no_ground_truth(self):
        target, heading = observed_target([], .7)
        self.assertFalse(target.visible)
        np.testing.assert_array_equal(target.position, [0, 0])
        np.testing.assert_array_equal(Follower().command([0, 0, 0], target), [0, 0, 0])
        self.assertEqual(heading, .7)

    def test_scan_world_transform_and_moving_person_replacement(self):
        human = SimpleNamespace(role='human', state=np.array([[1.], [3.], [0.]]),
                                velocity_xy=np.array([[.2], [-.1]]), radius=.3)
        scan = dict(ranges=[2., 4., np.inf], angle_min=0., angle_max=np.pi,
                    range_min=0., range_max=5.)
        obs = scan_obstacles(scan, np.array([1., 1., np.pi / 2]), [human])
        self.assertEqual(len(obs), 2)
        np.testing.assert_allclose(obs[0], [-3., 1., 0, 0, .05], atol=1e-10)
        np.testing.assert_allclose(obs[1], [1., 3., .2, -.1, .3])

    def test_hidden_target_does_not_remove_lidar_obstacle(self):
        scan = dict(ranges=[2.], angle_min=0., angle_max=0., range_max=5.)
        self.assertEqual(len(scan_obstacles(scan, np.zeros(3), [])), 1)

    def test_explicit_profile_preserves_scenario(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'world.yaml'
            source.write_text('world: {step_time: 0.1}\nrobot: [{}]\ntarget: [{state: [5, 10, 0], behavior: {name: orca}}]\n')
            config = prepare_config(source, .4)
            self.assertEqual(config['robot'][0]['vel_max'], [.8, .7])
            self.assertEqual(config['target'][0]['state'], [5, 10, 0])
            self.assertEqual(config['target'][0]['behavior'], dict(name='orca', vxmax=.4, vymax=.4))
            self.assertEqual(config['robot'][0]['shape']['radius'], .35)

    def test_lateral_limit_applied_before_rollout(self):
        planner = Follower(seed=0)
        planner.low[1] = planner.high[1] = 0.
        target, _ = observed_target([SimpleNamespace(role='target',
            state=np.array([[2.], [1.], [0.]]), velocity_xy=np.array([[.4], [0.]]))], 0.)
        command = planner.command(np.zeros(3), target)
        self.assertEqual(command[1], 0.)
        self.assertGreaterEqual(command[0], 0.)


if __name__ == '__main__':
    unittest.main()
