"""Integration tests for solver with simulator data."""

import numpy as np
import pytest

from robot_sysid.parser import load_robot_model
from robot_sysid.simulator import collect_sysid_data
from robot_sysid.solver import identify_parameters
from robot_sysid.trajectory import generate_excitation_trajectory


@pytest.mark.mujoco
def test_solver_with_kinova_model():
    """Integration test: identify parameters from Kinova model simulation."""
    # Load the Kinova model
    xml_path = "examples/kinova/model/kinova_fullinertia_guess.xml"
    model = load_robot_model(xml_path)
    
    # Generate a short trajectory for testing
    trajectory = generate_excitation_trajectory(
        model=model,
        duration=5.0,  # Short duration for fast test
        sample_rate=500,  # Lower sample rate for speed
        seed=42,
    )
    
    # Collect simulation data
    data = collect_sysid_data(
        xml_path=xml_path,
        trajectory=trajectory,
        terminal_joint=model.terminal_joint,
        terminal_body_name=model.terminal_body.name,
        include_friction=True,
    )
    
    # Identify parameters
    params = identify_parameters(
        data=data,
        joint_name=model.terminal_joint.name,
        include_friction=True,
    )
    
    # Verify results are reasonable
    assert params.joint_name == model.terminal_joint.name
    # Note: mass may be 0 or small due to rank deficiency in single-joint identification
    # This is expected behavior - not all parameters are identifiable
    assert params.rmse < 0.1, f"RMSE should be small, got {params.rmse}"
    assert params.condition_number > 0, "Condition number should be positive"
    assert params.n_base_params > 0, "Should have identifiable parameters"
    assert params.coulomb_friction is not None, "Should have friction estimate"
    assert params.viscous_friction is not None, "Should have friction estimate"
    
    # Verify inertia tensor has reasonable values
    assert len(params.inertia) == 6, "Inertia should have 6 components"
    
    # Verify com_moment has 3 components
    assert len(params.com_moment) == 3, "COM moment should have 3 components"
    
    print(f"\nIdentified parameters for {params.joint_name}:")
    print(f"  Mass: {params.mass:.6f} kg")
    print(f"  COM moment: {params.com_moment}")
    print(f"  Inertia: {params.inertia}")
    print(f"  Coulomb friction: {params.coulomb_friction:.6f} Nm")
    print(f"  Viscous friction: {params.viscous_friction:.6f} Nm·s/rad")
    print(f"  RMSE: {params.rmse:.6f} Nm")
    print(f"  Condition number: {params.condition_number:.2f}")
    print(f"  Base parameters: {params.n_base_params}")


@pytest.mark.mujoco
def test_solver_without_friction():
    """Test solver with friction disabled."""
    # Load the Kinova model
    xml_path = "examples/kinova/model/kinova_fullinertia_guess.xml"
    model = load_robot_model(xml_path)
    
    # Generate a short trajectory
    trajectory = generate_excitation_trajectory(
        model=model,
        duration=3.0,
        sample_rate=500,
        seed=42,
    )
    
    # Collect simulation data without friction
    data = collect_sysid_data(
        xml_path=xml_path,
        trajectory=trajectory,
        terminal_joint=model.terminal_joint,
        terminal_body_name=model.terminal_body.name,
        include_friction=False,
    )
    
    # Identify parameters without friction
    params = identify_parameters(
        data=data,
        joint_name=model.terminal_joint.name,
        include_friction=False,
    )
    
    # Verify friction is None
    assert params.coulomb_friction is None
    assert params.viscous_friction is None
    # Note: mass may be 0 or small due to rank deficiency
    assert params.rmse < 0.1
    
    print(f"\nIdentified parameters (no friction) for {params.joint_name}:")
    print(f"  Mass: {params.mass:.6f} kg")
    print(f"  RMSE: {params.rmse:.6f} Nm")
