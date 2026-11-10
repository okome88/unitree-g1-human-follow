import unittest
import numpy as np
from human_follow.planner import Follower, Person
from human_follow.policy import OfficialPolicy
from human_follow.sim import target_at


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.follower = Follower()
        self.target = Person(np.array([2., 0.]), np.array([.4, 0.]))

    def test_loss_and_stale_observation_stop(self):
        for visible, age in [(False, 0), (True, .31), (True, -1)]:
            self.target.visible, self.target.age = visible, age
            np.testing.assert_array_equal(self.follower.command([0, 0, 0], self.target), 0)
            self.assertEqual(self.follower.reason, "target_lost")

    def test_nonfinite_stop(self):
        self.target.position[0] = np.nan
        np.testing.assert_array_equal(self.follower.command([0, 0, 0], self.target), 0)

    def test_too_close_stop(self):
        self.target.position[0] = .5
        np.testing.assert_array_equal(self.follower.command([0, 0, 0], self.target), 0)
        self.assertEqual(self.follower.reason, "target_too_close")

    def test_obstacle_stop(self):
        np.testing.assert_array_equal(self.follower.command([0, 0, 0], self.target,
                                                            [[.4, 0, 0, 0, .3]]), 0)
        self.assertEqual(self.follower.reason, "obstacle_too_close")

    def test_limits_and_acceleration(self):
        previous = np.zeros(3)
        for _ in range(30):
            command = self.follower.command([0, 0, 0], self.target)
            self.assertTrue(np.all(command >= self.follower.low))
            self.assertTrue(np.all(command <= self.follower.high))
            self.assertTrue(np.all(np.abs(command - previous) <= np.array([.08, .06, .12]) + 1e-9))
            previous = command

    def test_body_frame_rotation(self):
        self.target.position = np.array([0., 2.])
        self.target.velocity = np.array([0., .4])
        self.target.heading = np.pi / 2
        command = self.follower.command([0, 0, np.pi / 2], self.target)
        self.assertGreater(command[0], 0)

    def test_closed_loop_straight(self):
        pose = np.zeros(3)
        distances = []
        for k in range(200):
            self.target.position = np.array([2 + .4 * k * .1, 0])
            cmd = self.follower.command(pose, self.target)
            c, s = np.cos(pose[2]), np.sin(pose[2])
            pose += .1 * np.array([c*cmd[0]-s*cmd[1], s*cmd[0]+c*cmd[1], cmd[2]])
            distances.append(np.linalg.norm(pose[:2] - self.target.position))
        self.assertLess(abs(distances[-1] - 1.5), .35)
        self.assertGreater(min(distances), .85)

    def test_seed_reproducibility(self):
        a, b = Follower(7), Follower(7)
        np.testing.assert_array_equal(a.command([0, 0, 0], self.target), b.command([0, 0, 0], self.target))

    def test_scenarios(self):
        self.assertEqual(np.linalg.norm(target_at(12, "stop", .4).velocity), 0)
        self.assertFalse(target_at(11, "loss", .4).visible)
        self.assertTrue(target_at(14, "loss", .4).visible)
        self.assertGreater(target_at(12, "turn", .4).heading, 0)


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = OfficialPolicy()

    def test_observation_contract(self):
        p = self.policy
        obs = p.observation([1, 2, 3], [0, 0, -1], [9, -9, 9], p.default, np.zeros(29))[0]
        np.testing.assert_allclose(obs[:9], [1, 2, 3, 0, 0, -1, 1, -.5, 1])
        np.testing.assert_allclose(obs[9:11], [np.sin(2*np.pi/30), np.cos(2*np.pi/30)], atol=1e-6)
        np.testing.assert_array_equal(obs[11:], 0)

    def test_standing_phase(self):
        p = self.policy
        obs = p.observation([0]*3, [0, 0, -1], [0]*3, p.default, np.zeros(29))
        np.testing.assert_array_equal(obs[0, 9:11], 0)

    def test_actual_onnx_inference(self):
        p = self.policy
        output = p.act([0]*3, [0, 0, -1], [.4, 0, 0], p.default, np.zeros(29))
        self.assertEqual(output.shape, (29,))
        self.assertTrue(np.isfinite(output).all())
        obs = p.observation([0]*3, [0, 0, -1], [.4, 0, 0], p.default, np.zeros(29))
        np.testing.assert_array_equal(obs[0, -29:], p.last)


if __name__ == "__main__":
    unittest.main()
