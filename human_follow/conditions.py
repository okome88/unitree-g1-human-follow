"""User-configurable simulation conditions, independent of following method."""
import json
import math
from pathlib import Path


def conditions(args):
    distance = getattr(args, "distance", 1.5)
    turn_rate = getattr(args, "turn_rate", .1)
    count = getattr(args, "pedestrians", None)
    count = (1 if args.scenario == "crossing" else 0) if count is None else count
    path = getattr(args, "pedestrian_path", "crossing")
    speed = getattr(args, "pedestrian_speed", .25)
    if not math.isfinite(distance) or not 1.2 <= distance <= 3:
        raise ValueError("distance must be in [1.2, 3.0] m")
    if not math.isfinite(turn_rate) or abs(turn_rate) > .3:
        raise ValueError("turn-rate must be in [-0.3, 0.3] rad/s")
    if not isinstance(count, int) or not 0 <= count <= 20:
        raise ValueError("pedestrians must be an integer in [0, 20]")
    if not math.isfinite(speed) or not 0 <= speed <= 1.5:
        raise ValueError("pedestrian-speed must be in [0, 1.5] m/s")
    routes = []
    for i in range(count):
        if path == "crossing":
            route = [4 + 2*i, -2., 0., speed, .3]
        elif path == "parallel":
            route = [3 + 2*i, 2., speed, 0., .3]
        elif path == "oncoming":
            route = [6 + 2*i, 1.5, -speed, 0., .3]
        elif path == "stationary":
            route = [4 + 2*i, 1.5, 0., 0., .3]
        else:
            raise ValueError("Unknown pedestrian path")
        routes.append(route)
    route_file = getattr(args, "pedestrian_routes", None)
    if route_file:
        if getattr(args, "pedestrians", None) is not None:
            raise ValueError("Use pedestrian-routes OR pedestrians, not both")
        routes = json.loads(Path(route_file).read_text(encoding="utf-8"))
        if not isinstance(routes, list) or len(routes) > 20:
            raise ValueError("Route file must contain a list of at most 20 routes")
        for route in routes:
            if (not isinstance(route, list) or len(route) != 5 or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in route)):
                raise ValueError("Each route must be finite [x, y, vx, vy, radius]")
            if not .1 <= route[4] <= 1 or math.hypot(route[2], route[3]) > 1.5:
                raise ValueError("Route radius must be 0.1–1 m and speed <= 1.5 m/s")
    return distance, turn_rate, routes


def pedestrians_at(t, routes):
    elapsed = max(0., t - 2.)
    return [[x + vx*elapsed, y + vy*elapsed, vx, vy, radius]
            for x, y, vx, vy, radius in routes]
