import cvxpy as cp
import numpy as np
from scipy.linalg import solve_discrete_are


def dlqr(a_mat: np.ndarray, b_mat: np.ndarray, q_mat: np.ndarray, r_mat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Infinite-horizon discrete LQR gain for x = [p_x, p_y, v_x, v_y], u = -Kx."""

    p_mat = solve_discrete_are(a_mat, b_mat, q_mat, r_mat)
    gain = np.linalg.solve(b_mat.T @ p_mat @ b_mat + r_mat, b_mat.T @ p_mat @ a_mat)
    return gain, p_mat


class LinearMPC:
    """Quadratic MPC for the discrete model with x = [p_x, p_y, v_x, v_y]."""

    def __init__(
        self,
        a_mat: np.ndarray,
        b_mat: np.ndarray,
        q_mat: np.ndarray,
        r_mat: np.ndarray,
        p_mat: np.ndarray,
        horizon: int,
        u_max: np.ndarray,
        x_max: np.ndarray,
    ) -> None:
        self.a_mat = np.asarray(a_mat, dtype=float)
        self.b_mat = np.asarray(b_mat, dtype=float)
        self.q_mat = np.asarray(q_mat, dtype=float)
        self.r_mat = np.asarray(r_mat, dtype=float)
        self.p_mat = np.asarray(p_mat, dtype=float)
        self.horizon = int(horizon)
        self.u_max = np.asarray(u_max, dtype=float)
        self.x_max = np.asarray(x_max, dtype=float)

        self.n_states, self.n_inputs = self.b_mat.shape
        self._build_problem()

    def _build_problem(self) -> None:
        self.x0 = cp.Parameter(self.n_states)
        self.x_ref = cp.Parameter(self.n_states, value=np.zeros(self.n_states))
        self.x = cp.Variable((self.n_states, self.horizon + 1))
        self.u = cp.Variable((self.n_inputs, self.horizon))

        objective = 0
        constraints = [self.x[:, 0] == self.x0]

        for k in range(self.horizon):
            state_error = self.x[:, k] - self.x_ref
            objective += cp.quad_form(state_error, self.q_mat)
            objective += cp.quad_form(self.u[:, k], self.r_mat)
            constraints += [
                self.x[:, k + 1] == self.a_mat @ self.x[:, k] + self.b_mat @ self.u[:, k],
                self.u[:, k] <= self.u_max,
                self.u[:, k] >= -self.u_max,
                self.x[:, k] <= self.x_max,
                self.x[:, k] >= -self.x_max,
            ]

        terminal_error = self.x[:, self.horizon] - self.x_ref
        objective += cp.quad_form(terminal_error, self.p_mat)
        constraints += [
            self.x[:, self.horizon] <= self.x_max,
            self.x[:, self.horizon] >= -self.x_max,
        ]

        self.problem = cp.Problem(cp.Minimize(objective), constraints)

    def control(self, state: np.ndarray, reference: np.ndarray | None = None) -> tuple[np.ndarray, str]:
        self.x0.value = np.asarray(state, dtype=float)
        self.x_ref.value = np.zeros(self.n_states) if reference is None else np.asarray(reference, dtype=float)

        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
        except cp.SolverError:
            self.problem.solve(warm_start=True, verbose=False)

        if self.problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            return np.zeros(self.n_inputs), self.problem.status

        return np.asarray(self.u[:, 0].value, dtype=float), self.problem.status
