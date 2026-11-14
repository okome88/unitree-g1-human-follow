"""Simulator-independent wxyz rotations and wrench transfer, tested without Isaac."""
import numpy as np


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-10:
        raise ValueError('Expected finite nonzero wxyz quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def point_state(link_position, rotation, offset, com_position, com_velocity, angular_velocity):
    point = link_position + rotation @ offset
    velocity = com_velocity + np.cross(angular_velocity, point - com_position)
    return point, velocity


def spring(target, target_velocity, position, velocity, stiffness, damping, maximum):
    force = stiffness * (target-position) + damping * (target_velocity-velocity)
    return force * min(1.0, maximum / max(float(np.linalg.norm(force)), 1e-12))


def wrench_at_com(force, point, com):
    return force, np.cross(point-com, force)
