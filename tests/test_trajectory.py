"""Unit tests for the trajectory generator module.

Tests verify:
- Trajectory shape and metadata correctness
- Joint limit compliance (positions within limits)
- Velocity limit compliance
- Analytical derivatives match numerical derivatives
- Terminal joint has dominant excitation amplitude
- Configurable parameters (duration, sample_rate, seed)
"""

from __future__ import annotations

import numpy as np
import pytest

from robot_sysid.parser import JointInfo, BodyInfo, RobotModel
from robot_sysid.trajectory import (
    Trajectory,
    generate_excitation_trajectory,
    DEFAULT_VELOCITY_LIMIT,
)


def _make_joint(
    name: str,
    lower: float = -2.41,
    upper: float = 2.41,
) -> JointInfo:
    """Create a JointInfo with sensible defaults for testing."""
    return JointInfo(
        name=name,
        axis=np.array([0.0, 0.0, 1.0]),
        body_name=f"body_{name}",
        site_name=f"site_{name}",
        range=(lower, upper),
        force_range=(-10.0, 10.0),
    )


def _make_body(name: str, parent: str | None = None) -> BodyInfo:
    """Create a BodyInfo with sensible defaults for testing."""
    return BodyInfo(
        name=name,
        mass=1.0,
        com=np.zeros(3),
        inertia=np.array([0.01, 0.0, 0.0, 0.01, 0.0, 0.01]),
        parent=parent,
    )


def _make_model(n_joints: int = 7) -> RobotModel:
    """Create a RobotModel with n_joints for testing.

    The last joint is the terminal joint.
    """
    joints = [_make_joint(f"joint{i}") for i in range(n_joints)]
    bodies = []
    for i in range(n_joints):
        parent = f"body_joint{i - 1}" if i > 0 else "world"
        bodies.append(_make_body(f"body_joint{i}", parent=parent))

    terminal_joint = joints[-1]
    terminal_body = bodies[-1]

    return RobotModel(
        xml_path="test.xml",
        joints=joints,
        bodies=bodies,
        terminal_joint=terminal_joint,
        terminal_body=terminal_body,
        n_joints=n_joints,
    )


class TestTrajectoryShape:
    """Tests for trajectory output shape and metadata."""

    def test_default_parameters(self):
        model = _make_model(7)
        traj = generate_excitation_trajectory(model, seed=42)

        assert traj.n_joints == 7
        assert traj.dt == pytest.approx(0.001)
        assert traj.position.shape == (15000, 7)
        assert traj.velocity.shape == (15000, 7)
        assert traj.acceleration.shape == (15000, 7)
        assert traj.time.shape == (15000,)

    def test_custom_duration_and_rate(self):
        model = _make_model(3)
        traj = generate_excitation_trajectory(
            model, duration=5.0, sample_rate=500, seed=42
        )

        assert traj.n_joints == 3
        assert traj.dt == pytest.approx(0.002)
        assert traj.position.shape == (2500, 3)
        assert traj.time.shape == (2500,)
        assert traj.time[0] == pytest.approx(0.0)

    def test_single_joint(self):
        model = _make_model(1)
        traj = generate_excitation_trajectory(model, duration=2.0, seed=42)

        assert traj.n_joints == 1
        assert traj.position.shape[1] == 1

    def test_seed_reproducibility(self):
        model = _make_model(4)
        traj1 = generate_excitation_trajectory(model, seed=123)
        traj2 = generate_excitation_trajectory(model, seed=123)

        np.testing.assert_array_equal(traj1.position, traj2.position)
        np.testing.assert_array_equal(traj1.velocity, traj2.velocity)
        np.testing.assert_array_equal(traj1.acceleration, traj2.acceleration)

    def test_different_seeds_differ(self):
        model = _make_model(4)
        traj1 = generate_excitation_trajectory(model, seed=1)
        traj2 = generate_excitation_trajectory(model, seed=2)

        assert not np.allclose(traj1.position, traj2.position)


class TestJointLimits:
    """Tests for joint position limit compliance."""

    def test_positions_within_limits(self):
        model = _make_model(7)
        traj = generate_excitation_trajectory(model, seed=42)

        for j, joint in enumerate(model.joints):
            lower, upper = joint.range
            assert np.all(traj.position[:, j] >= lower), (
                f"Joint {j} position below lower limit {lower}"
            )
            assert np.all(traj.position[:, j] <= upper), (
                f"Joint {j} position above upper limit {upper}"
            )

    def test_narrow_joint_limits(self):
        """Trajectory should respect even narrow joint limits."""
        joints = [
            _make_joint("j0", lower=-0.5, upper=0.5),
            _make_joint("j1", lower=-0.3, upper=0.3),
            _make_joint("j2", lower=-0.1, upper=0.1),
        ]
        bodies = [
            _make_body("body_j0", parent="world"),
            _make_body("body_j1", parent="body_j0"),
            _make_body("body_j2", parent="body_j1"),
        ]
        model = RobotModel(
            xml_path="test.xml",
            joints=joints,
            bodies=bodies,
            terminal_joint=joints[-1],
            terminal_body=bodies[-1],
            n_joints=3,
        )
        traj = generate_excitation_trajectory(model, seed=42)

        for j, joint in enumerate(model.joints):
            lower, upper = joint.range
            assert np.all(traj.position[:, j] >= lower)
            assert np.all(traj.position[:, j] <= upper)


class TestVelocityLimits:
    """Tests for velocity limit compliance."""

    def test_velocities_within_limits(self):
        model = _make_model(7)
        traj = generate_excitation_trajectory(model, seed=42)

        vel_limit = DEFAULT_VELOCITY_LIMIT
        for j in range(model.n_joints):
            assert np.all(traj.velocity[:, j] >= -vel_limit), (
                f"Joint {j} velocity below lower limit"
            )
            assert np.all(traj.velocity[:, j] <= vel_limit), (
                f"Joint {j} velocity above upper limit"
            )


class TestAnalyticalDerivatives:
    """Tests that analytical derivatives match numerical derivatives."""

    def test_velocity_matches_numerical_derivative(self):
        model = _make_model(4)
        traj = generate_excitation_trajectory(
            model, duration=5.0, sample_rate=1000, seed=42
        )

        # Numerical derivative of position
        dt = traj.dt
        numerical_vel = np.diff(traj.position, axis=0) / dt

        # Compare interior points (skip first/last for finite diff boundary)
        # The analytical velocity at point i should match (pos[i+1] - pos[i]) / dt
        # to within O(dt) tolerance
        for j in range(traj.n_joints):
            # Use central difference for better accuracy
            central_vel = (traj.position[2:, j] - traj.position[:-2, j]) / (2 * dt)
            analytical_vel = traj.velocity[1:-1, j]

            # Tolerance: O(dt²) for central difference
            tol = dt**2 * 1e4  # generous tolerance for numerical diff
            np.testing.assert_allclose(
                analytical_vel, central_vel, atol=tol,
                err_msg=f"Velocity mismatch for joint {j}",
            )

    def test_acceleration_matches_numerical_derivative(self):
        model = _make_model(4)
        traj = generate_excitation_trajectory(
            model, duration=5.0, sample_rate=1000, seed=42
        )

        dt = traj.dt
        for j in range(traj.n_joints):
            # Central difference of velocity
            central_acc = (traj.velocity[2:, j] - traj.velocity[:-2, j]) / (2 * dt)
            analytical_acc = traj.acceleration[1:-1, j]

            tol = dt**2 * 1e4
            np.testing.assert_allclose(
                analytical_acc, central_acc, atol=tol,
                err_msg=f"Acceleration mismatch for joint {j}",
            )


class TestTerminalJointDominance:
    """Tests that the terminal joint has dominant excitation amplitude."""

    def test_terminal_joint_largest_amplitude(self):
        model = _make_model(7)
        traj = generate_excitation_trajectory(model, seed=42)

        # Find terminal joint index
        terminal_idx = None
        for i, joint in enumerate(model.joints):
            if joint.name == model.terminal_joint.name:
                terminal_idx = i
                break

        # Peak-to-peak amplitude of terminal joint
        terminal_ptp = np.ptp(traj.position[:, terminal_idx])

        # All non-terminal joints should have smaller peak-to-peak
        for j in range(model.n_joints):
            if j == terminal_idx:
                continue
            non_terminal_ptp = np.ptp(traj.position[:, j])
            assert terminal_ptp > non_terminal_ptp, (
                f"Terminal joint ptp ({terminal_ptp:.4f}) not greater than "
                f"joint {j} ptp ({non_terminal_ptp:.4f})"
            )

    def test_terminal_dominance_with_equal_ranges(self):
        """Even with equal joint ranges, terminal should dominate."""
        model = _make_model(3)
        traj = generate_excitation_trajectory(model, seed=42)

        terminal_ptp = np.ptp(traj.position[:, -1])
        for j in range(model.n_joints - 1):
            assert terminal_ptp > np.ptp(traj.position[:, j])

    def test_single_joint_is_terminal(self):
        """With a single joint, it is both terminal and the only joint."""
        model = _make_model(1)
        traj = generate_excitation_trajectory(model, seed=42)

        # Should still produce non-trivial excitation
        assert np.ptp(traj.position[:, 0]) > 0.1
