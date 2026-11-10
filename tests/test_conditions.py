import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from human_follow.conditions import conditions, pedestrians_at
from human_follow.sim import target_at


class ConditionsTests(unittest.TestCase):
    def test_legacy_defaults(self):
        self.assertEqual(len(conditions(SimpleNamespace(scenario="crossing"))[2]), 1)
        self.assertEqual(conditions(SimpleNamespace(scenario="turn"))[2], [])

    def test_turn_and_crowd_are_independent(self):
        distance, rate, routes = conditions(SimpleNamespace(scenario="turn", distance=2.,
                        turn_rate=-.1, pedestrians=3, pedestrian_path="parallel"))
        self.assertEqual((distance, rate, len(routes)), (2., -.1, 3))
        self.assertLess(target_at(5, "turn", .4, rate).position[1], 0)

    def test_zero_turn_is_straight(self):
        np.testing.assert_allclose(target_at(7, "turn", .4, 0).position,
                                   target_at(7, "straight", .4).position)

    def test_paths(self):
        for path, expected in [("crossing", [0, .25]), ("parallel", [.25, 0]),
                               ("oncoming", [-.25, 0]), ("stationary", [0, 0])]:
            routes = conditions(SimpleNamespace(scenario="turn", pedestrians=2, pedestrian_path=path))[2]
            self.assertEqual(routes[0][2:4], expected)
            after = pedestrians_at(6, routes)
            np.testing.assert_allclose(np.array(after[0][:2]) - routes[0][:2], 4*np.array(expected))

    def test_invalid_settings(self):
        for name, value in [("distance", .5), ("distance", float("nan")), ("turn_rate", .4),
                            ("pedestrians", -1), ("pedestrians", 21), ("pedestrian_speed", float("inf"))]:
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                conditions(SimpleNamespace(scenario="straight", **{name: value}))

    def test_custom_routes_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "routes.json"
            args = SimpleNamespace(scenario="turn", pedestrian_routes=path)
            path.write_text(json.dumps([[4, -2, 0, .25, .3]]))
            self.assertEqual(len(conditions(args)[2]), 1)
            for invalid in [[[]], [[0, 0, 2, 0, .3]], [[0, 0, 0, 0, -1]], {}]:
                path.write_text(json.dumps(invalid))
                with self.assertRaises(ValueError):
                    conditions(args)
