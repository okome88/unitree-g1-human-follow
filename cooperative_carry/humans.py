"""Scripted intent; physical hand force is applied separately by the backend."""
import numpy as np
from .interfaces import register_human


class ScriptedHuman:
    def __init__(self, config):
        self.config = config

    def reset(self):
        pass

    def target(self, time):
        c = self.config
        elapsed = np.clip(time - c.start_time, 0, c.stop_time - c.start_time)
        active = c.start_time < time < c.stop_time
        x = c.speed * elapsed
        y = 0.15 * np.sin(0.7 * elapsed) if c.kind == "curve" else 0.0
        vy = 0.105 * np.cos(0.7 * elapsed) if c.kind == "curve" else 0.0
        return np.array([x, y, 0.0]), np.array([c.speed, vy, 0.0]) * active


class KeyboardHuman:
    """Viewer keys W/S, A/D change velocity, space stops. Simulation thread owns state."""
    def __init__(self, config):
        from queue import SimpleQueue
        self.keys = SimpleQueue()
        self.reset()

    def reset(self):
        while not self.keys.empty():
            self.keys.get_nowait()
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.time = 0.0

    def key(self, keycode):
        self.keys.put(keycode)

    def target(self, time):
        while not self.keys.empty():
            key = self.keys.get_nowait()
            if key == 32:
                self.velocity[:] = 0
            elif key in (87, 83, 65, 68):
                index = 0 if key in (87, 83) else 1
                self.velocity[index] = np.clip(self.velocity[index] +
                    (0.05 if key in (87, 65) else -0.05), -0.3, 0.3)
        self.position += self.velocity * max(0, time - self.time)
        self.time = time
        return self.position.copy(), self.velocity.copy()


register_human("straight", ScriptedHuman)
register_human("curve", ScriptedHuman)
register_human("keyboard", KeyboardHuman)
