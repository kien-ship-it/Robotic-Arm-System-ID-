"""Unit tests for robot_sysid.solver module."""

import numpy as np
import pytest

from robot_sysid.simulator import RegressorData
from robot_sysid.solver import IdentifiedParams, identify_parameters


def test_identify_parameters_without_friction():
    """Test parameter identification without friction terms."""
    # Create synthetic data with known parameters
    # θ = [m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
    theta_true = np.array([1.0, 0.1, 0.2, 0.3, 0.01, 0.001, 0.002, 0.015, 0.003, 0.005])
    
    # Create a simple regressor matrix (10 columns)
    n_samples = 100
    Y = np.random.randn(n_samples, 10)
    
    # Compute torques from true parameters
    tau = Y @ theta_true
    
    # Create RegressorData
    data = RegressorData(
        Y=Y,
        tau=tau,
        time=np.linspace(0, 1, n_samples),
        condition_number=10.0,
        matrix_rank=10,
    )
    
    # Identify parameters
    params = identify_parameters(data, joint_name="test_joint", include_friction=False)
    
    # Verify results
    assert params.joint_name == "test_joint"
    assert params.mass == pytest.approx(theta_true[0], abs=1e-8)
    np.testing.assert_allclose(params.com_moment, theta_true[1:4], atol=1e-8)
    np.testing.assert_allclose(params.inertia, theta_true[4:10], atol=1e-8)
    assert params.coulomb_friction is None
    assert params.viscous_friction is None
    assert params.rmse < 1e-10  # Should be near-zero for noiseless data
    assert params.condition_number == 10.0
    assert params.n_base_params == 10


def test_identify_parameters_with_friction():
    """Test parameter identification with friction terms."""
    # Create synthetic data with known parameters including friction
    # θ = [m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Fc, Fv]
    theta_true = np.array([
        1.0, 0.1, 0.2, 0.3,  # mass and com_moment
        0.01, 0.001, 0.002, 0.015, 0.003, 0.005,  # inertia
        0.4, 0.1  # friction
    ])
    
    # Create a simple regressor matrix (12 columns)
    n_samples = 100
    Y = np.random.randn(n_samples, 12)
    
    # Compute torques from true parameters
    tau = Y @ theta_true
    
    # Create RegressorData
    data = RegressorData(
        Y=Y,
        tau=tau,
        time=np.linspace(0, 1, n_samples),
        condition_number=15.0,
        matrix_rank=12,
    )
    
    # Identify parameters
    params = identify_parameters(data, joint_name="test_joint", include_friction=True)
    
    # Verify results
    assert params.joint_name == "test_joint"
    assert params.mass == pytest.approx(theta_true[0], abs=1e-8)
    np.testing.assert_allclose(params.com_moment, theta_true[1:4], atol=1e-8)
    np.testing.assert_allclose(params.inertia, theta_true[4:10], atol=1e-8)
    assert params.coulomb_friction == pytest.approx(theta_true[10], abs=1e-8)
    assert params.viscous_friction == pytest.approx(theta_true[11], abs=1e-8)
    assert params.rmse < 1e-10  # Should be near-zero for noiseless data
    assert params.condition_number == 15.0
    assert params.n_base_params == 12


def test_identify_parameters_dimension_mismatch():
    """Test that dimension mismatch raises ValueError."""
    # Create regressor with 10 columns
    Y = np.random.randn(100, 10)
    tau = np.random.randn(100)
    
    data = RegressorData(
        Y=Y,
        tau=tau,
        time=np.linspace(0, 1, 100),
        condition_number=10.0,
        matrix_rank=10,
    )
    
    # Try to identify with include_friction=True (expects 12 columns)
    with pytest.raises(ValueError, match="Regressor matrix has 10 columns, expected 12"):
        identify_parameters(data, joint_name="test_joint", include_friction=True)


def test_identified_params_to_dict():
    """Test serialization to JSON-compatible dict."""
    params = IdentifiedParams(
        mass=1.5,
        com_moment=np.array([0.1, 0.2, 0.3]),
        inertia=np.array([0.01, 0.001, 0.002, 0.015, 0.003, 0.005]),
        coulomb_friction=0.4,
        viscous_friction=0.1,
        rmse=0.001,
        condition_number=123.45,
        n_base_params=12,
        joint_name="joint6",
    )
    
    d = params.to_dict()
    
    # Verify structure
    assert d["joint_name"] == "joint6"
    assert d["mass"] == 1.5
    assert d["center_of_mass_moment"] == [0.1, 0.2, 0.3]
    assert d["inertia_tensor"]["Ixx"] == 0.01
    assert d["inertia_tensor"]["Ixy"] == 0.001
    assert d["inertia_tensor"]["Ixz"] == 0.002
    assert d["inertia_tensor"]["Iyy"] == 0.015
    assert d["inertia_tensor"]["Iyz"] == 0.003
    assert d["inertia_tensor"]["Izz"] == 0.005
    assert d["coulomb_friction"] == 0.4
    assert d["viscous_friction"] == 0.1
    assert d["rmse"] == 0.001
    assert d["condition_number"] == 123.45
    assert d["n_base_params"] == 12


def test_identified_params_from_dict():
    """Test deserialization from dict."""
    d = {
        "joint_name": "joint6",
        "mass": 1.5,
        "center_of_mass_moment": [0.1, 0.2, 0.3],
        "inertia_tensor": {
            "Ixx": 0.01,
            "Ixy": 0.001,
            "Ixz": 0.002,
            "Iyy": 0.015,
            "Iyz": 0.003,
            "Izz": 0.005,
        },
        "coulomb_friction": 0.4,
        "viscous_friction": 0.1,
        "rmse": 0.001,
        "condition_number": 123.45,
        "n_base_params": 12,
    }
    
    params = IdentifiedParams.from_dict(d)
    
    # Verify values
    assert params.joint_name == "joint6"
    assert params.mass == 1.5
    np.testing.assert_array_equal(params.com_moment, np.array([0.1, 0.2, 0.3]))
    np.testing.assert_array_equal(
        params.inertia, np.array([0.01, 0.001, 0.002, 0.015, 0.003, 0.005])
    )
    assert params.coulomb_friction == 0.4
    assert params.viscous_friction == 0.1
    assert params.rmse == 0.001
    assert params.condition_number == 123.45
    assert params.n_base_params == 12


def test_identified_params_round_trip():
    """Test that to_dict() and from_dict() are inverses."""
    params_original = IdentifiedParams(
        mass=1.5,
        com_moment=np.array([0.1, 0.2, 0.3]),
        inertia=np.array([0.01, 0.001, 0.002, 0.015, 0.003, 0.005]),
        coulomb_friction=0.4,
        viscous_friction=0.1,
        rmse=0.001,
        condition_number=123.45,
        n_base_params=12,
        joint_name="joint6",
    )
    
    # Round-trip: to_dict -> from_dict
    d = params_original.to_dict()
    params_recovered = IdentifiedParams.from_dict(d)
    
    # Verify all fields match
    assert params_recovered.joint_name == params_original.joint_name
    assert params_recovered.mass == params_original.mass
    np.testing.assert_array_equal(params_recovered.com_moment, params_original.com_moment)
    np.testing.assert_array_equal(params_recovered.inertia, params_original.inertia)
    assert params_recovered.coulomb_friction == params_original.coulomb_friction
    assert params_recovered.viscous_friction == params_original.viscous_friction
    assert params_recovered.rmse == params_original.rmse
    assert params_recovered.condition_number == params_original.condition_number
    assert params_recovered.n_base_params == params_original.n_base_params


def test_identified_params_round_trip_without_friction():
    """Test round-trip with None friction values."""
    params_original = IdentifiedParams(
        mass=1.5,
        com_moment=np.array([0.1, 0.2, 0.3]),
        inertia=np.array([0.01, 0.001, 0.002, 0.015, 0.003, 0.005]),
        coulomb_friction=None,
        viscous_friction=None,
        rmse=0.001,
        condition_number=123.45,
        n_base_params=10,
        joint_name="joint6",
    )
    
    # Round-trip: to_dict -> from_dict
    d = params_original.to_dict()
    params_recovered = IdentifiedParams.from_dict(d)
    
    # Verify all fields match
    assert params_recovered.joint_name == params_original.joint_name
    assert params_recovered.mass == params_original.mass
    np.testing.assert_array_equal(params_recovered.com_moment, params_original.com_moment)
    np.testing.assert_array_equal(params_recovered.inertia, params_original.inertia)
    assert params_recovered.coulomb_friction is None
    assert params_recovered.viscous_friction is None
    assert params_recovered.rmse == params_original.rmse
    assert params_recovered.condition_number == params_original.condition_number
    assert params_recovered.n_base_params == params_original.n_base_params
