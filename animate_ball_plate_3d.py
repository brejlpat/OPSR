from argparse import ArgumentParser
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from ball_plate_model import BallPlateParams


RESULTS_DIR = Path("results")


def load_result(name: str) -> np.ndarray:
    path = RESULTS_DIR / f"{name}.csv"
    if not path.exists():
        raise FileNotFoundError(f"Missing simulation result: {path}")
    return np.genfromtxt(path, delimiter=",", names=True)


def plate_z(x_grid: np.ndarray, y_grid: np.ndarray, theta_x: float, theta_y: float) -> np.ndarray:
    """Height of a tilted plate.

    Positive theta_x lowers the plate in +p_x direction and positive theta_y lowers it in
    +p_y direction, which is consistent with the acceleration signs in the model.
    """

    return -theta_x * x_grid - theta_y * y_grid


def ball_surface(
    p_x: float,
    p_y: float,
    theta_x: float,
    theta_y: float,
    radius: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    u = np.linspace(0.0, 2.0 * np.pi, 18)
    v = np.linspace(0.0, np.pi, 10)
    x_sphere = p_x + radius * np.outer(np.cos(u), np.sin(v))
    y_sphere = p_y + radius * np.outer(np.sin(u), np.sin(v))
    z_center = plate_z(p_x, p_y, theta_x, theta_y) + radius
    z_sphere = z_center + radius * np.outer(np.ones_like(u), np.cos(v))
    return x_sphere, y_sphere, z_sphere


def make_animation(name: str, every: int = 2, fps: int = 20) -> Path:
    params = BallPlateParams()
    data = load_result(name)

    times = data["t"][::every]
    p_x = data["x"][::every]
    p_y = data["y"][::every]
    theta_x = data["theta_x"][::every]
    theta_y = data["theta_y"][::every]

    plate_limit = params.max_position_m
    grid = np.linspace(-plate_limit, plate_limit, 9)
    x_grid, y_grid = np.meshgrid(grid, grid)
    ball_radius = 0.015
    z_limit = plate_limit * params.max_tilt_rad + 2.5 * ball_radius

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    fig.suptitle(name.replace("_", " "))

    ax.set_xlim(-plate_limit, plate_limit)
    ax.set_ylim(-plate_limit, plate_limit)
    ax.set_zlim(-z_limit, z_limit)
    ax.set_xlabel("p_x [m]")
    ax.set_ylabel("p_y [m]")
    ax.set_zlabel("z [m]")
    ax.set_box_aspect((1, 1, 0.35))
    ax.view_init(elev=26, azim=-55)

    time_text = ax.text2D(0.03, 0.95, "", transform=ax.transAxes)
    path_line, = ax.plot([], [], [], color="tab:blue", linewidth=2.0, label="trajektorie")
    ax.scatter([0.0], [0.0], [ball_radius], marker="+", color="black", s=80, label="reference")
    ax.legend(loc="upper right")

    artists: dict[str, object | None] = {"plate": None, "ball": None}

    def update(frame: int):
        if artists["plate"] is not None:
            artists["plate"].remove()
        if artists["ball"] is not None:
            artists["ball"].remove()

        z_grid = plate_z(x_grid, y_grid, theta_x[frame], theta_y[frame])
        artists["plate"] = ax.plot_surface(
            x_grid,
            y_grid,
            z_grid,
            color="lightgray",
            alpha=0.65,
            edgecolor="gray",
            linewidth=0.4,
            shade=False,
        )

        x_ball, y_ball, z_ball = ball_surface(
            p_x[frame],
            p_y[frame],
            theta_x[frame],
            theta_y[frame],
            ball_radius,
        )
        artists["ball"] = ax.plot_surface(
            x_ball,
            y_ball,
            z_ball,
            color="tab:orange",
            edgecolor="none",
            shade=True,
        )

        path_z = plate_z(p_x[: frame + 1], p_y[: frame + 1], theta_x[frame], theta_y[frame])
        path_line.set_data(p_x[: frame + 1], p_y[: frame + 1])
        path_line.set_3d_properties(path_z + 0.003)
        time_text.set_text(f"t = {times[frame]:.2f} s")
        return artists["plate"], artists["ball"], path_line, time_text

    animation = FuncAnimation(fig, update, frames=len(times), interval=1000 / fps, blit=False)

    RESULTS_DIR.mkdir(exist_ok=True)
    output_path = RESULTS_DIR / f"{name}_3d.gif"
    animation.save(output_path, writer=PillowWriter(fps=fps))

    # Save a final frame as a quick static preview for reports.
    update(len(times) - 1)
    fig.savefig(RESULTS_DIR / f"{name}_3d_final.png", dpi=160)
    plt.close(fig)
    return output_path


def main() -> None:
    parser = ArgumentParser(description="Create a 3D ball-on-plate animation from simulation CSV results.")
    parser.add_argument(
        "scenario",
        nargs="?",
        default="mpc_with_disturbance",
        help="Result name without .csv, for example mpc_with_disturbance.",
    )
    parser.add_argument("--all", action="store_true", help="Generate animations for all four default scenarios.")
    parser.add_argument("--every", type=int, default=2, help="Use every N-th simulation sample.")
    parser.add_argument("--fps", type=int, default=20, help="Animation frames per second.")
    args = parser.parse_args()

    scenarios = [
        "lqr_without_disturbance",
        "lqr_with_disturbance",
        "mpc_without_disturbance",
        "mpc_with_disturbance",
    ]
    names = scenarios if args.all else [args.scenario]

    for name in names:
        output_path = make_animation(name, every=max(1, args.every), fps=args.fps)
        print(f"Saved {output_path}")


if __name__ == "__main__":
    main()
