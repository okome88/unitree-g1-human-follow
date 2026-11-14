"""CPU tests for contracts shared across simulators, NOT Isaac runtime smoke tests."""
from types import SimpleNamespace
import numpy as np
import pytest
from isaac_carry.math_utils import rotation_matrix, point_state, spring, wrench_at_com
from isaac_carry.policy import Policy, JOINT_NAMES, TORQUE_LIMITS
from cooperative_carry.policy import OfficialG1


def test_wxyz_rotation_and_point_velocity():
    rotation = rotation_matrix([np.sqrt(.5),0,0,np.sqrt(.5)])
    np.testing.assert_allclose(rotation @ [1,0,0],[0,1,0],atol=1e-12)
    point,velocity = point_state(np.zeros(3),rotation,np.array([1,0,0]),
        np.zeros(3),np.array([2,0,0]),np.array([0,0,1]))
    np.testing.assert_allclose(point,[0,1,0],atol=1e-12)
    np.testing.assert_allclose(velocity,[1,0,0],atol=1e-12)
    with pytest.raises(ValueError):
        rotation_matrix([0,0,0,0])


def test_force_pair_and_off_center_torque():
    force = spring(np.array([1,0,0]),np.zeros(3),np.zeros(3),np.zeros(3),300,12,50)
    np.testing.assert_allclose(force,[50,0,0])
    f,t = wrench_at_com(force,np.array([0,1,0]),np.zeros(3))
    np.testing.assert_allclose(t,[0,0,-50])
    opposite,opposite_t = wrench_at_com(-force,np.array([0,1,0]),np.zeros(3))
    np.testing.assert_allclose(f+opposite,0)
    np.testing.assert_allclose(t+opposite_t,0)


def test_official_policy_parity_with_mujoco_adapter():
    # Test actual ONNX inference and observation order against the existing adapter.
    # A tiny model/data facade supplies the adapter's state, not a fake Isaac runtime.
    model = SimpleNamespace(actuator_trnid=np.column_stack([np.arange(29),np.zeros(29)]).astype(int),
        jnt_qposadr=np.arange(29),jnt_dofadr=np.arange(29),joint=lambda i:SimpleNamespace(name=JOINT_NAMES[i]))
    reference = OfficialG1(model,'models/unitree_official/g1_velocity',.02)
    candidate = Policy('models/unitree_official/g1_velocity',.02)
    rng = np.random.default_rng(42)
    for command in ([0,0,0],[.15,0,0],[.2,.1,.15],[0,0,0]):
        q = reference.default+rng.normal(0,.01,29)
        dq = rng.normal(0,.05,29)
        gyro = rng.normal(0,.01,3)
        rotation = rotation_matrix([.99,.01,.02,.03])
        data = SimpleNamespace(qpos=q,qvel=dq,body=lambda _:SimpleNamespace(xmat=rotation.ravel()),
                               sensor=lambda _:SimpleNamespace(data=gyro))
        reference.update(model,data,command)
        candidate.update(gyro,rotation.T @ [0,0,-1],command,q,dq)
        np.testing.assert_allclose(candidate.target,reference.target,atol=1e-7)
    torque = candidate.torques(np.ones(29)*100,np.zeros(29))
    assert np.all(np.abs(torque) <= TORQUE_LIMITS)


def test_policy_rejects_wrong_rate_and_nonfinite_input():
    with pytest.raises(ValueError):
        Policy('models/unitree_official/g1_velocity',.01)
    policy = Policy('models/unitree_official/g1_velocity')
    with pytest.raises(ValueError):
        policy.update([np.nan,0,0],[0,0,-1],[0,0,0],policy.default,np.zeros(29))
