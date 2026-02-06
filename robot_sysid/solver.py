"""Parameter identification solver for robot system identification.

Solves the least-squares problem θ = pinv(Y) × τ to identify inertial
parameters and friction coefficients from regressor data.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from robot_sysid.simulator import RegressorData


@dataclass
class IdentifiedParams:
    """Identified inertial parameters and friction coefficients.

    Attributes:
        mass: Mass of the terminal link (kg).
        com_moment: First moment of mass [hx, hy, hz] = m × [cx, cy, cz] (kg·m).
        inertia: Inertia tensor [Ixx, Ixy, Ixz, Iyy, Iyz, Izz] (kg·m²).
        coulomb_friction: Coulomb friction coefficient (N·m), or None if not identified.
        viscous_friction: Viscous friction coefficient (N·m·s/rad), or None if not identified.
        rmse: Root mean squared error of torque reconstruction (N·m).
        condition_number: Condition number of the regressor matrix.
        n_base_params: Number of identifiable base parameters (matrix rank).
        joint_name: Name of the terminal joint.
    """

    mass: float
    com_moment: np.ndarray  # [hx, hy, hz]
    inertia: np.ndarray  # [Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
    coulomb_friction: float | None
    viscous_friction: float | None
    rmse: float
    condition_number: float
    n_base_params: int
    joint_name: str

    def to_dict(self) -> dict:
        """Serialize to JSON-compatible dict.

        Converts numpy arrays to lists for JSON serialization.

        Returns:
            Dictionary with all parameter values in JSON-compatible format.
        """
        return {
            "joint_name": self.joint_name,
            "mass": float(self.mass),
            "center_of_mass_moment": self.com_moment.tolist(),
            "inertia_tensor": {
                "Ixx": float(self.inertia[0]),
                "Ixy": float(self.inertia[1]),
                "Ixz": float(self.inertia[2]),
                "Iyy": float(self.inertia[3]),
                "Iyz": float(self.inertia[4]),
                "Izz": float(self.inertia[5]),
            },
            "coulomb_friction": (
                float(self.coulomb_friction)
                if self.coulomb_friction is not None
                else None
            ),
            "viscous_friction": (
                float(self.viscous_friction)
                if self.viscous_friction is not None
                else None
            ),
            "rmse": float(self.rmse),
            "condition_number": float(self.condition_number),
            "n_base_params": int(self.n_base_params),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "IdentifiedParams":
        """Deserialize from dict.

        Converts lists back to numpy arrays.

        Args:
            d: Dictionary with parameter values (from to_dict() or JSON).

        Returns:
            IdentifiedParams object with values from the dictionary.
        """
        # Extract inertia tensor from nested dict
        inertia_dict = d["inertia_tensor"]
        inertia = np.array(
            [
                inertia_dict["Ixx"],
                inertia_dict["Ixy"],
                inertia_dict["Ixz"],
                inertia_dict["Iyy"],
                inertia_dict["Iyz"],
                inertia_dict["Izz"],
            ]
        )

        return cls(
            joint_name=d["joint_name"],
            mass=float(d["mass"]),
            com_moment=np.array(d["center_of_mass_moment"]),
            inertia=inertia,
            coulomb_friction=(
                float(d["coulomb_friction"])
                if d["coulomb_friction"] is not None
                else None
            ),
            viscous_friction=(
                float(d["viscous_friction"])
                if d["viscous_friction"] is not None
                else None
            ),
            rmse=float(d["rmse"]),
            condition_number=float(d["condition_number"]),
            n_base_params=int(d["n_base_params"]),
        )


def identify_parameters(
    data: RegressorData, joint_name: str, include_friction: bool = True
) -> IdentifiedParams:
    """Solve θ = pinv(Y) × τ and return identified parameters.

    Uses least-squares to solve for the parameter vector θ from the regressor
    matrix Y and torque vector τ. The parameter vector contains:
    - 10 inertial parameters: [m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
    - 2 friction parameters (if include_friction=True): [Fc, Fv]

    The solution is computed via np.linalg.lstsq with rcond=1e-10 for robust
    least-squares that handles rank-deficient matrices gracefully.

    Args:
        data: RegressorData containing the regressor matrix Y and torque vector τ.
        joint_name: Name of the terminal joint being identified.
        include_friction: Whether friction parameters are included in the regressor
            (default True). Must match the include_friction flag used in
            collect_sysid_data().

    Returns:
        IdentifiedParams containing the identified mass, center of mass moment,
        inertia tensor, friction coefficients (if applicable), RMSE, condition
        number, and matrix rank.

    Raises:
        ValueError: If the regressor matrix has unexpected shape or if
            include_friction doesn't match the regressor dimensions.
    """
    Y = data.Y
    tau = data.tau

    # Validate regressor dimensions
    expected_cols = 12 if include_friction else 10
    if Y.shape[1] != expected_cols:
        raise ValueError(
            f"Regressor matrix has {Y.shape[1]} columns, expected {expected_cols}. "
            f"include_friction={include_friction} requires {expected_cols} columns."
        )

    # Solve least-squares: θ = pinv(Y) × τ
    # Using lstsq with rcond=1e-10 for robust least-squares
    theta, residuals, rank, singular_values = np.linalg.lstsq(
        Y, tau, rcond=1e-10
    )

    # Split parameter vector
    mass = theta[0]
    com_moment = theta[1:4]  # [hx, hy, hz]
    inertia = theta[4:10]  # [Ixx, Ixy, Ixz, Iyy, Iyz, Izz]

    if include_friction:
        coulomb_friction = theta[10]
        viscous_friction = theta[11]
    else:
        coulomb_friction = None
        viscous_friction = None

    # Compute reconstruction RMSE
    tau_predicted = Y @ theta
    rmse = float(np.sqrt(np.mean((tau - tau_predicted) ** 2)))

    return IdentifiedParams(
        mass=float(mass),
        com_moment=com_moment,
        inertia=inertia,
        coulomb_friction=coulomb_friction,
        viscous_friction=viscous_friction,
        rmse=rmse,
        condition_number=data.condition_number,
        n_base_params=data.matrix_rank,
        joint_name=joint_name,
    )
