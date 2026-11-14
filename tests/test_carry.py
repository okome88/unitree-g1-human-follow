from dataclasses import replace
import numpy as np
import pytest
from cooperative_carry.config import Config, HumanConfig, load_config
from cooperative_carry.controllers import Admittance
from cooperative_carry.environment import CarryEnvironment, spring_force
from cooperative_carry.interfaces import Observation
from cooperative_carry.humans import ScriptedHuman


def test_invalid_config():
    for config in (Config(timestep=.003), Config(duration=-1), Config(duration=float('nan'))):
        with pytest.raises(ValueError):
            config.validate()


def test_spring_direction_and_force_bound():
    force = spring_force(np.array([1., 0, 0]), np.zeros(3), np.zeros(3), np.zeros(3), 300, 12, 50)
    np.testing.assert_allclose(force, [50, 0, 0])
    force = spring_force(np.zeros(3), np.zeros(3), np.zeros(3), np.array([1., 0, 0]), 300, 12, 50)
    assert force[0] < 0


def test_controller_frame_and_limits():
    cfg = Config().controller
    controller = Admittance(cfg)
    rotation = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    obs = Observation(0., np.zeros(3), rotation, np.array([100., 0, 0]))
    for _ in range(300):
        command = controller.command(obs, .02)
        assert np.linalg.norm(command[:2]) <= cfg.max_speed + 1e-10
    assert command[1] < 0 and abs(command[0]) < 1e-10
    controller.reset()
    np.testing.assert_equal(controller.velocity, [0, 0])


def test_human_stop():
    driver = ScriptedHuman(HumanConfig())
    position, velocity = driver.target(100)
    np.testing.assert_allclose(velocity, [0, 0, 0])
    np.testing.assert_allclose(position, [.15 * 14, 0, 0])


def test_reset_reproducibility_and_reaction():
    env = CarryEnvironment(Config(robot="surrogate", duration=1))
    # Translate payload away from robot: interaction force must pull robot forwards.
    index = env.model.joint("payload_free").qposadr[0]
    env.data.qpos[index] += .01
    import mujoco
    mujoco.mj_forward(env.model, env.data)
    env.apply_interaction()
    assert env.robot_force[0] > 0
    payload_dof = env.model.joint("payload_free").dofadr[0]
    # Internal robot-payload forces cancel; human supplies the external net force.
    np.testing.assert_allclose(env.data.qfrc_applied[0] + env.data.qfrc_applied[payload_dof],
                               env.human_force[0], atol=1e-12)
    def rollout():
        env.reset(seed=3)
        for _ in range(20):
            env.step()
        return env.data.qpos.copy()
    np.testing.assert_allclose(rollout(), rollout(), atol=1e-12)


def test_surrogate_carry_and_logical_termination():
    env = CarryEnvironment(load_config("configs/carry_surrogate.toml"))
    while not env.done:
        result = env.step()
    assert result.reason == "goal_reached"
    assert result.metrics["payload_tilt_deg"] < 10
    with pytest.raises(RuntimeError):
        env.step()


def test_payload_mass_affects_static_support():
    vertical = []
    for mass in (.5, 1.5):
        cfg = Config(robot="surrogate", human=HumanConfig(speed=0), duration=1)
        cfg.payload.mass = mass
        env = CarryEnvironment(cfg)
        while not env.done:
            result = env.step()
        vertical.append(result.metrics["human_vertical_force"] + result.metrics["robot_vertical_force"])
    np.testing.assert_allclose(vertical, np.array([.5, 1.5]) * 9.81, rtol=.05)
