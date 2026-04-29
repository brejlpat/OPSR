from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ball_plate_model import (
    BallPlateParams,
    controllability_rank,
    default_disturbance,
    discretize_linear_model,
    linearize_at_equilibrium,
    rk4_step,
)
from controllers import LinearMPC, dlqr


OUTPUT_DIR = Path("results")


def saturate_control(control: np.ndarray, params: BallPlateParams) -> np.ndarray:
    return np.clip(control, -params.max_tilt_rad, params.max_tilt_rad)


def simulate(
    controller_name: str,
    controller,
    initial_state: np.ndarray,
    dt: float,
    t_final: float,
    params: BallPlateParams,
    disturbed: bool,
) -> dict[str, np.ndarray]:
    n_steps = int(round(t_final / dt))
    times = np.linspace(0.0, t_final, n_steps + 1)
    states = np.zeros((n_steps + 1, 4))
    controls = np.zeros((n_steps, 2))
    disturbances = np.zeros((n_steps, 2))
    statuses: list[str] = []

    states[0] = initial_state
    for k in range(n_steps):
        time_s = times[k]
        disturbance = default_disturbance(time_s) if disturbed else np.zeros(2)

        if controller_name == "lqr":
            raw_control = -controller @ states[k]
            status = "analytic"
        else:
            raw_control, status = controller.control(states[k])

        control = saturate_control(raw_control, params)
        next_state = rk4_step(states[k], control, dt, disturbance, params)

        states[k + 1] = next_state
        controls[k] = control
        disturbances[k] = disturbance
        statuses.append(status)

    return {
        "times": times,
        "states": states,
        "controls": controls,
        "disturbances": disturbances,
        "statuses": np.array(statuses, dtype=object),
    }


def save_result(name: str, result: dict[str, np.ndarray]) -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)
    n_steps = result["controls"].shape[0]
    data = np.column_stack(
        [
            result["times"][:n_steps],
            result["states"][:n_steps],
            result["controls"],
            result["disturbances"],
        ]
    )
    header = "t,x,y,vx,vy,theta_x,theta_y,dist_x,dist_y"
    np.savetxt(OUTPUT_DIR / f"{name}.csv", data, delimiter=",", header=header, comments="")


def plot_result(name: str, result: dict[str, np.ndarray], params: BallPlateParams) -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)
    times = result["times"]
    states = result["states"]
    controls_deg = np.rad2deg(result["controls"])
    disturbances = result["disturbances"]
    control_times = times[:-1]

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    fig.suptitle(name.replace("_", " "))

    axes[0].plot(times, states[:, 0], label="x")
    axes[0].plot(times, states[:, 1], label="y")
    axes[0].axhline(params.max_position_m, color="tab:red", linestyle="--", linewidth=0.9)
    axes[0].axhline(-params.max_position_m, color="tab:red", linestyle="--", linewidth=0.9)
    axes[0].set_ylabel("poloha [m]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(times, states[:, 2], label="vx")
    axes[1].plot(times, states[:, 3], label="vy")
    axes[1].axhline(params.max_velocity_mps, color="tab:red", linestyle="--", linewidth=0.9)
    axes[1].axhline(-params.max_velocity_mps, color="tab:red", linestyle="--", linewidth=0.9)
    axes[1].set_ylabel("rychlost [m/s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    axes[2].step(control_times, controls_deg[:, 0], where="post", label="theta_x")
    axes[2].step(control_times, controls_deg[:, 1], where="post", label="theta_y")
    axes[2].axhline(np.rad2deg(params.max_tilt_rad), color="tab:red", linestyle="--", linewidth=0.9)
    axes[2].axhline(-np.rad2deg(params.max_tilt_rad), color="tab:red", linestyle="--", linewidth=0.9)
    axes[2].set_ylabel("naklon [deg]")
    axes[2].set_xlabel("cas [s]")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / f"{name}_timeseries.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 6))
    limit = params.max_position_m
    ax.plot(states[:, 0], states[:, 1], label="trajektorie")
    ax.scatter(states[0, 0], states[0, 1], marker="o", color="tab:orange", label="start")
    ax.scatter(states[-1, 0], states[-1, 1], marker="x", color="tab:green", label="konec")
    ax.plot(0.0, 0.0, marker="+", color="black", markersize=10, label="reference")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(name.replace("_", " "))
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / f"{name}_trajectory.png", dpi=160)
    plt.close(fig)

    if np.max(np.abs(disturbances)) > 0.0:
        fig, ax = plt.subplots(figsize=(9, 3))
        ax.step(control_times, disturbances[:, 0], where="post", label="d_x")
        ax.step(control_times, disturbances[:, 1], where="post", label="d_y")
        ax.set_xlabel("cas [s]")
        ax.set_ylabel("porucha [m/s^2]")
        ax.set_title(f"poruchy: {name.replace('_', ' ')}")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(OUTPUT_DIR / f"{name}_disturbance.png", dpi=160)
        plt.close(fig)


def plot_comparison(results: dict[str, dict[str, np.ndarray]]) -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    groups = [
        ("without_disturbance", "bez poruch"),
        ("with_disturbance", "s poruchami"),
    ]
    for suffix, title in groups:
        fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        fig.suptitle(f"Porovnani regulatoru: {title}")

        for controller in ("lqr", "mpc"):
            name = f"{controller}_{suffix}"
            result = results[name]
            times = result["times"]
            states = result["states"]
            position_norm = np.linalg.norm(states[:, :2], axis=1)
            max_control_component_deg = np.rad2deg(np.max(np.abs(result["controls"]), axis=1))

            axes[0].plot(times, position_norm, label=controller.upper())
            axes[1].step(times[:-1], max_control_component_deg, where="post", label=controller.upper())

        axes[0].set_ylabel("||p|| [m]")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="best")
        axes[1].set_ylabel("max |theta_i| [deg]")
        axes[1].set_xlabel("cas [s]")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc="best")
        fig.tight_layout()
        fig.savefig(OUTPUT_DIR / f"comparison_{suffix}.png", dpi=160)
        plt.close(fig)


def summarize_result(name: str, result: dict[str, np.ndarray]) -> str:
    states = result["states"]
    controls = result["controls"]
    final_pos_norm = np.linalg.norm(states[-1, :2])
    max_pos_abs = np.max(np.abs(states[:, :2]))
    max_tilt_deg = np.rad2deg(np.max(np.abs(controls)))
    return (
        f"{name}: final |position|={final_pos_norm:.5f} m, "
        f"max |position coordinate|={max_pos_abs:.5f} m, "
        f"max |tilt|={max_tilt_deg:.2f} deg"
    )


def main() -> None:
    params = BallPlateParams()
    dt = 0.05
    t_final = 8.0
    initial_state = np.array([0.12, -0.08, 0.0, 0.0])

    a_cont, b_cont = linearize_at_equilibrium(params)
    a_disc, b_disc = discretize_linear_model(a_cont, b_cont, dt)
    rank_cont = controllability_rank(a_cont, b_cont)
    rank_disc = controllability_rank(a_disc, b_disc)

    q_mat = np.diag([180.0, 180.0, 8.0, 8.0])
    r_mat = np.diag([0.8, 0.8])
    k_lqr, p_lqr = dlqr(a_disc, b_disc, q_mat, r_mat)

    mpc = LinearMPC(
        a_disc,
        b_disc,
        q_mat,
        r_mat,
        p_lqr,
        horizon=25,
        u_max=np.array([params.max_tilt_rad, params.max_tilt_rad]),
        x_max=np.array(
            [
                params.max_position_m,
                params.max_position_m,
                params.max_velocity_mps,
                params.max_velocity_mps,
            ]
        ),
    )

    print("Continuous controllability rank:", rank_cont)
    print("Discrete controllability rank:", rank_disc)
    print("LQR gain K:")
    print(k_lqr)

    scenarios = [
        ("lqr_without_disturbance", "lqr", k_lqr, False),
        ("lqr_with_disturbance", "lqr", k_lqr, True),
        ("mpc_without_disturbance", "mpc", mpc, False),
        ("mpc_with_disturbance", "mpc", mpc, True),
    ]

    all_results = {}
    for file_name, controller_name, controller, disturbed in scenarios:
        result = simulate(controller_name, controller, initial_state, dt, t_final, params, disturbed)
        all_results[file_name] = result
        save_result(file_name, result)
        plot_result(file_name, result, params)
        print(summarize_result(file_name, result))
    plot_comparison(all_results)


if __name__ == "__main__":
    main()
