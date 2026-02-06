"""Trajectory generator for system identification excitation.

Generates multi-frequency sinusoidal excitation trajectories with analytical
velocity and acceleration profiles. The terminal joint receives primary
(large-amplitude) excitation while non-terminal joints receive low-amplitude
excitation to improve regressor conditioning.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np

from robot_sysid.parser import RobotModel


# Default velocity limit when not available from the model (rad/s)
DEFAULT_VELOCITY_LIMIT = 2.0

# Incommensurate base frequencies (Hz) to avoid periodicity.
# These are chosen so no frequency is a rational multiple of another.
BASE_FREQUENCIES = np.array([0.1, 0.15, 0.23, 0.37, 0.51])

# Amplitude ratio for non-terminal joints relative to terminal joint
NON_TERMINAL_AMPLITUDE_RATIO = 0.15


@dataclass
class Trajectory:
    """Excitation trajectory with position, velocity, and acceleration profiles.

    Attributes:
        position: Joint positions array of shape (n_samples, n_joints).
        velocity: Joint velocities array of shape (n_samples, n_joints).
        acceleration: Joint accelerations array of shape (n_samples, n_joints).
        time: Time array of shape (n_samples,).
        dt: Time step in seconds.
        n_joints: Number of joints.
    """

    position: np.ndarray      # (n_samples, n_joints)
    velocity: np.ndarray      # (n_samples, n_joints)
    acceleration: np.ndarray  # (n_samples, n_joints)
    time: np.ndarray          # (n_samples,)
    dt: float
    n_joints: int


def _compute_sinusoid_components(
    time: np.ndarray,
    n_sinusoids: int,
    frequencies: np.ndarray,
    amplitudes: np.ndarray,
    phases: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute position, velocity, and acceleration for a sum of sinusoids.

    Position:     q(t) = sum_i A_i * sin(2π * f_i * t + φ_i)
    Velocity:     dq/dt = sum_i A_i * 2π * f_i * cos(2π * f_i * t + φ_i)
    Acceleration: d²q/dt² = -sum_i A_i * (2π * f_i)² * sin(2π * f_i * t + φ_i)

    Args:
        time: Time array of shape (n_samples,).
        n_sinusoids: Number of sinusoidal components.
        frequencies: Frequencies in Hz, shape (n_sinusoids,).
        amplitudes: Amplitudes, shape (n_sinusoids,).
        phases: Phase offsets in radians, shape (n_sinusoids,).

    Returns:
        Tuple of (position, velocity, acceleration) arrays, each shape (n_samples,).
    """
    position = np.zeros_like(time)
    velocity = np.zeros_like(time)
    acceleration = np.zeros_like(time)

    for i in range(n_sinusoids):
        omega = 2.0 * np.pi * frequencies[i]
        arg = omega * time + phases[i]

        position += amplitudes[i] * np.sin(arg)
        velocity += amplitudes[i] * omega * np.cos(arg)
        acceleration -= amplitudes[i] * omega**2 * np.sin(arg)

    return position, velocity, acceleration


def _select_frequencies(rng: np.random.Generator, n_sinusoids: int) -> np.ndarray:
    """Select incommensurate frequencies from the base set.

    If n_sinusoids <= len(BASE_FREQUENCIES), selects a subset.
    Otherwise, generates additional frequencies in the 0.1–0.6 Hz range.

    Args:
        rng: NumPy random generator.
        n_sinusoids: Number of frequencies to select.

    Returns:
        Array of frequencies in Hz, shape (n_sinusoids,).
    """
    if n_sinusoids <= len(BASE_FREQUENCIES):
        indices = rng.choice(len(BASE_FREQUENCIES), size=n_sinusoids, replace=False)
        return BASE_FREQUENCIES[indices].copy()
    else:
        # Use all base frequencies plus additional random ones
        extra = rng.uniform(0.1, 0.6, size=n_sinusoids - len(BASE_FREQUENCIES))
        return np.concatenate([BASE_FREQUENCIES.copy(), extra])


def _compute_amplitude_scale(
    joint_range: tuple[float, float],
    n_sinusoids: int,
    is_terminal: bool,
    frequencies: np.ndarray,
    velocity_limit: float = DEFAULT_VELOCITY_LIMIT,
) -> float:
    """Compute the per-sinusoid amplitude scale for a joint.

    The total amplitude (sum of all sinusoid amplitudes) should stay within
    the joint's position range AND the resulting velocities should stay within
    the velocity limit. For the terminal joint, we use ~70% of the available
    range. For non-terminal joints, we use a fraction of that.

    The velocity of a sum of sinusoids q(t) = sum A_i sin(ω_i t + φ_i) has
    max velocity bounded by sum(A_i * ω_i). We constrain this to stay within
    the velocity limit.

    Args:
        joint_range: (lower, upper) position limits in radians.
        n_sinusoids: Number of sinusoidal components.
        is_terminal: Whether this is the terminal (primary) joint.
        frequencies: Array of frequencies in Hz for each sinusoid.
        velocity_limit: Maximum allowed velocity in rad/s.

    Returns:
        Per-sinusoid amplitude scale.
    """
    half_range = (joint_range[1] - joint_range[0]) / 2.0

    if is_terminal:
        # Use ~70% of the half-range, divided among sinusoids
        total_amplitude_pos = 0.7 * half_range
    else:
        # Non-terminal: small fraction of the half-range
        total_amplitude_pos = NON_TERMINAL_AMPLITUDE_RATIO * 0.7 * half_range

    per_sinusoid_from_pos = total_amplitude_pos / n_sinusoids

    # Velocity constraint: max velocity = sum(A_i * ω_i)
    # If each A_i = A (equal amplitude), max_vel = A * sum(ω_i)
    # So A <= velocity_limit * safety_margin / sum(ω_i)
    omega_sum = np.sum(2.0 * np.pi * frequencies)
    safety_margin = 0.85  # leave 15% margin
    per_sinusoid_from_vel = (velocity_limit * safety_margin) / omega_sum

    # Take the more restrictive constraint
    return min(per_sinusoid_from_pos, per_sinusoid_from_vel)


def generate_excitation_trajectory(
    model: RobotModel,
    duration: float = 15.0,
    sample_rate: int = 1000,
    seed: int | None = None,
) -> Trajectory:
    """Generate sinusoidal excitation trajectory for system identification.

    The terminal joint receives primary excitation (large amplitude sum of
    sinusoids), while non-terminal joints receive low-amplitude excitation
    to improve the condition number of the regressor matrix.

    Velocities and accelerations are computed analytically as exact
    derivatives of the position sinusoids.

    After computing the raw sinusoidal trajectory, positions are clamped to
    joint limits and velocities are clamped to velocity limits. The analytical
    derivatives are of the unclamped trajectory — clamping is a safety check,
    not a trajectory modification (amplitudes are chosen to mostly stay within
    limits).

    Args:
        model: Robot model with joint information and limits.
        duration: Trajectory duration in seconds (default 15.0).
        sample_rate: Samples per second (default 1000).
        seed: Random seed for reproducibility (default None).

    Returns:
        Trajectory dataclass with position, velocity, acceleration, and time
        arrays.
    """
    rng = np.random.default_rng(seed)

    n_samples = int(duration * sample_rate)
    dt = 1.0 / sample_rate
    time = np.linspace(0.0, duration, n_samples, endpoint=False)

    n_joints = model.n_joints
    position = np.zeros((n_samples, n_joints))
    velocity = np.zeros((n_samples, n_joints))
    acceleration = np.zeros((n_samples, n_joints))

    # Find the terminal joint index
    terminal_idx = _find_terminal_joint_index(model)

    for j, joint in enumerate(model.joints):
        is_terminal = (j == terminal_idx)

        # Number of sinusoids: 3–5
        n_sinusoids = rng.integers(3, 6)  # [3, 5] inclusive

        # Select incommensurate frequencies in 0.1–0.6 Hz
        frequencies = _select_frequencies(rng, n_sinusoids)

        # Random phases
        phases = rng.uniform(0.0, 2.0 * np.pi, size=n_sinusoids)

        # Compute per-sinusoid amplitude (respecting both position and velocity limits)
        amp_scale = _compute_amplitude_scale(
            joint.range, n_sinusoids, is_terminal, frequencies
        )
        # Add slight random variation to each sinusoid's amplitude
        amplitudes = amp_scale * rng.uniform(0.7, 1.3, size=n_sinusoids)

        # Compute the sinusoidal components
        pos_j, vel_j, acc_j = _compute_sinusoid_components(
            time, n_sinusoids, frequencies, amplitudes, phases
        )

        # Center the trajectory within the joint range
        joint_center = (joint.range[0] + joint.range[1]) / 2.0
        pos_j += joint_center

        position[:, j] = pos_j
        velocity[:, j] = vel_j
        acceleration[:, j] = acc_j

    # Clamp positions to joint limits
    position, n_pos_clamped = _clamp_positions(position, model)
    if n_pos_clamped > 0:
        warnings.warn(
            f"Clamped {n_pos_clamped} position values to joint limits",
            stacklevel=2,
        )

    # Clamp velocities to velocity limits
    velocity, n_vel_clamped = _clamp_velocities(velocity, model)
    if n_vel_clamped > 0:
        warnings.warn(
            f"Clamped {n_vel_clamped} velocity values to velocity limits",
            stacklevel=2,
        )

    return Trajectory(
        position=position,
        velocity=velocity,
        acceleration=acceleration,
        time=time,
        dt=dt,
        n_joints=n_joints,
    )


def _find_terminal_joint_index(model: RobotModel) -> int:
    """Find the index of the terminal joint in the model's joint list.

    Args:
        model: Robot model.

    Returns:
        Index of the terminal joint in model.joints.
    """
    for i, joint in enumerate(model.joints):
        if joint.name == model.terminal_joint.name:
            return i
    # Fallback: last joint
    return model.n_joints - 1


def _clamp_positions(
    position: np.ndarray, model: RobotModel
) -> tuple[np.ndarray, int]:
    """Clamp position values to joint limits.

    Args:
        position: Position array of shape (n_samples, n_joints).
        model: Robot model with joint limits.

    Returns:
        Tuple of (clamped_position, n_clamped) where n_clamped is the total
        number of values that were modified.
    """
    clamped = position.copy()
    n_clamped = 0

    for j, joint in enumerate(model.joints):
        lower, upper = joint.range
        below = clamped[:, j] < lower
        above = clamped[:, j] > upper
        n_clamped += int(np.sum(below)) + int(np.sum(above))
        clamped[:, j] = np.clip(clamped[:, j], lower, upper)

    return clamped, n_clamped


def _clamp_velocities(
    velocity: np.ndarray, model: RobotModel
) -> tuple[np.ndarray, int]:
    """Clamp velocity values to velocity limits.

    Uses DEFAULT_VELOCITY_LIMIT since JointInfo does not store velocity limits.

    Args:
        velocity: Velocity array of shape (n_samples, n_joints).
        model: Robot model.

    Returns:
        Tuple of (clamped_velocity, n_clamped) where n_clamped is the total
        number of values that were modified.
    """
    clamped = velocity.copy()
    n_clamped = 0

    for j in range(model.n_joints):
        vel_limit = DEFAULT_VELOCITY_LIMIT
        below = clamped[:, j] < -vel_limit
        above = clamped[:, j] > vel_limit
        n_clamped += int(np.sum(below)) + int(np.sum(above))
        clamped[:, j] = np.clip(clamped[:, j], -vel_limit, vel_limit)

    return clamped, n_clamped
