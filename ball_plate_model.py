from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm


@dataclass(frozen=True)
class BallPlateParams:
    """Physical parameters of the ball-on-plate model."""

    g: float = 9.81
    rolling_gain: float = 5.0 / 7.0
    damping: float = 0.25
    max_tilt_rad: float = np.deg2rad(10.0)
    max_position_m: float = 0.18
    max_velocity_mps: float = 0.8


def continuous_dynamics(
    state: np.ndarray,
    control: np.ndarray,
    disturbance: np.ndarray | None = None,
    params: BallPlateParams = BallPlateParams(),
) -> np.ndarray:
    """Nonlinear continuous-time dynamics.

    State:
        x = [p_x, p_y, v_x, v_y], where p_x/p_y are ball positions on the plate.
    Control:
        [theta_x, theta_y], commanded plate tilts in radians.
    Disturbance:
        [dx, dy], additive acceleration disturbance in m/s^2.
    """

    x, y, vx, vy = np.asarray(state, dtype=float)
    theta_x, theta_y = np.clip(
        np.asarray(control, dtype=float),
        -params.max_tilt_rad,
        params.max_tilt_rad,
    )
    if disturbance is None:
        disturbance = np.zeros(2)
    disturbance = np.asarray(disturbance, dtype=float)

    accel_gain = params.rolling_gain * params.g
    return np.array(
        [
            vx,
            vy,
            accel_gain * np.sin(theta_x) - params.damping * vx + disturbance[0],
            accel_gain * np.sin(theta_y) - params.damping * vy + disturbance[1],
        ]
    )


def rk4_step(
    state: np.ndarray,
    control: np.ndarray,
    dt: float,
    disturbance: np.ndarray | None = None,
    params: BallPlateParams = BallPlateParams(),
) -> np.ndarray:
    """One fixed-step RK4 integration step for the nonlinear model."""

    k1 = continuous_dynamics(state, control, disturbance, params)
    k2 = continuous_dynamics(state + 0.5 * dt * k1, control, disturbance, params)
    k3 = continuous_dynamics(state + 0.5 * dt * k2, control, disturbance, params)
    k4 = continuous_dynamics(state + dt * k3, control, disturbance, params)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def linearize_at_equilibrium(params: BallPlateParams = BallPlateParams()) -> tuple[np.ndarray, np.ndarray]:
    """Linearize the nonlinear model at x = [p_x, p_y, v_x, v_y] = [0, 0, 0, 0], theta=0."""

    gain = params.rolling_gain * params.g
    a_cont = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, -params.damping, 0.0],
            [0.0, 0.0, 0.0, -params.damping],
        ]
    )
    b_cont = np.array(
        [
            [0.0, 0.0],
            [0.0, 0.0],
            [gain, 0.0],
            [0.0, gain],
        ]
    )
    return a_cont, b_cont


def discretize_linear_model(
    a_cont: np.ndarray,
    b_cont: np.ndarray,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Zero-order-hold discretization using a block matrix exponential."""

    n_states, n_inputs = b_cont.shape
    block = np.zeros((n_states + n_inputs, n_states + n_inputs))
    block[:n_states, :n_states] = a_cont
    block[:n_states, n_states:] = b_cont
    block_exp = expm(block * dt)
    return block_exp[:n_states, :n_states], block_exp[:n_states, n_states:]


def controllability_matrix(a_mat: np.ndarray, b_mat: np.ndarray) -> np.ndarray:
    """Return [B, AB, ..., A^(n-1)B]."""

    n_states = a_mat.shape[0]
    return np.hstack([np.linalg.matrix_power(a_mat, i) @ b_mat for i in range(n_states)])


def controllability_rank(a_mat: np.ndarray, b_mat: np.ndarray) -> int:
    return int(np.linalg.matrix_rank(controllability_matrix(a_mat, b_mat)))


def default_disturbance(time_s: float) -> np.ndarray:
    """A repeatable disturbance profile for the disturbed simulation."""

    bias = np.array([0.025, -0.015])
    pulse = np.array([0.0, 0.0])
    if 2.5 <= time_s <= 3.5:
        pulse += np.array([0.35, 0.0])
    if 5.0 <= time_s <= 5.7:
        pulse += np.array([0.0, -0.30])
    return bias + pulse
