"""Visualisation for 2D topology optimisation results."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot_density(
    densities: np.ndarray,
    nelx: int,
    nely: int,
    title: str = "Topology Optimisation Result",
    save_path: str | None = None,
):
    """Plot the density field as a greyscale image.

    Black = solid material, white = void.
    """
    grid = 1 - densities.reshape(nelx, nely).T
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.imshow(grid, cmap="gray", interpolation="none", origin="upper")
    ax.set_title(title)
    ax.set_xlabel("x elements")
    ax.set_ylabel("y elements")
    ax.set_aspect("equal")
    fig.savefig(save_path or "density.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved density plot to {save_path or 'density.png'}")


def plot_convergence(
    history: list[float],
    title: str = "Compliance Convergence",
    save_path: str | None = None,
):
    """Plot compliance versus iteration."""
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(history, "b-", linewidth=1.5)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Compliance")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    fig.savefig(save_path or "convergence.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved convergence plot to {save_path or 'convergence.png'}")


def plot_monocoque(
    densities: np.ndarray,
    nelx: int,
    nely: int,
    save_path: str | None = None,
):
    """Plot monocoque cross-section with annotations."""
    grid = 1 - densities.reshape(nelx, nely).T
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(grid, cmap="gray", interpolation="none", origin="upper")
    ax.set_title("Monocoque Cross-Section (half, viewed from front)")
    ax.set_xlabel("Centreline <-- elements --> Sill outer edge")
    ax.set_ylabel("Scuttle top <-- elements --> Floor")
    ax.set_aspect("equal")

    # Annotate regions
    ax.annotate("Cockpit\n(void)", xy=(nelx * 0.25, nely * 0.45),
                fontsize=10, color="blue", ha="center", va="center",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.8))
    ax.annotate("Suspension\npickup", xy=(nelx - 2, nely - 2),
                fontsize=8, color="red", ha="right", va="bottom",
                bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8))

    path = save_path or "monocoque_section.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved monocoque plot to {path}")


if __name__ == "__main__":
    import sys

    from car_solver.solver2d import cantilever_beam, mbb_beam, plate_with_hole, monocoque_cross_section
    from car_solver.config import load_config

    def on_iter(it, densities, compliance, change):
        if it % 10 == 0:
            print(f"  Iteration {it:3d}: compliance={compliance:.4f}, change={change:.4f}")

    mode = sys.argv[1] if len(sys.argv) > 1 else "all"

    if mode in ("all", "cantilever"):
        print("Running cantilever beam optimisation (120x40, vf=0.30)...")
        densities, history = cantilever_beam(on_iteration=on_iter)
        print(f"Converged in {len(history)} iterations")
        print(f"Final compliance: {history[-1]:.4f}")
        plot_density(densities, 120, 40, "Cantilever Beam - 2D SIMP")
        plot_convergence(history)

    if mode in ("all", "mbb"):
        print("\nRunning MBB beam optimisation (180x60, vf=0.30)...")
        densities, history = mbb_beam(on_iteration=on_iter)
        print(f"Converged in {len(history)} iterations")
        print(f"Final compliance: {history[-1]:.4f}")
        plot_density(densities, 180, 60, "MBB Beam - 2D SIMP", "mbb_density.png")
        plot_convergence(history, "MBB Compliance Convergence", "mbb_convergence.png")

    if mode in ("all", "plate"):
        print("\nRunning plate with hole optimisation (80x80, vf=0.40)...")
        densities, history = plate_with_hole(nelx=80, nely=80, on_iteration=on_iter)
        print(f"Converged in {len(history)} iterations")
        print(f"Final compliance: {history[-1]:.4f}")
        nelx_p, nely_p = 80, 80
        plot_density(densities, nelx_p, nely_p, "Plate with Hole - 2D SIMP", "plate_density.png")
        plot_convergence(history, "Plate Compliance Convergence", "plate_convergence.png")

    if mode in ("all", "monocoque"):
        print("\nRunning monocoque cross-section optimisation...")
        cfg = load_config()
        densities, history, nelx, nely = monocoque_cross_section(cfg, on_iteration=on_iter)
        print(f"Converged in {len(history)} iterations")
        print(f"Final compliance: {history[-1]:.4f}")
        print(f"Grid: {nelx}x{nely} elements")
        plot_monocoque(densities, nelx, nely)
        plot_convergence(history, "Monocoque Compliance Convergence", "monocoque_convergence.png")
