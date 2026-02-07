"""Generate a geometric diagram for predictive scan blocked intervals."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    output_dir = Path(__file__).resolve().parents[1] / "assets"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "predictive_scan_blocked_interval.png"

    fig, ax = plt.subplots(figsize=(6, 6))

    # Base geometry values (not to scale, just illustrative).
    theta_c = np.deg2rad(35)
    alpha = np.deg2rad(30)
    r_self = 1.0
    r_block = 0.55

    origin = np.array([0.0, 0.0])
    q = np.array([0.9, 0.6])
    a = r_self * np.array([np.cos(theta_c - alpha), np.sin(theta_c - alpha)])
    b = r_self * np.array([np.cos(theta_c + alpha), np.sin(theta_c + alpha)])

    # Draw circles
    circle_self = plt.Circle(origin, r_self, fill=False, linestyle="--", color="#555")
    circle_block = plt.Circle(q, r_block, fill=False, linestyle="-", color="#e07a5f")
    ax.add_patch(circle_self)
    ax.add_patch(circle_block)

    # Draw rays and labels
    ax.plot([0, q[0]], [0, q[1]], color="#3d405b", linewidth=2, label="中心角 θc")
    ax.plot([0, a[0]], [0, a[1]], color="#81b29a", linewidth=2)
    ax.plot([0, b[0]], [0, b[1]], color="#81b29a", linewidth=2)

    # Arc for blocked interval
    arc_angles = np.linspace(theta_c - alpha, theta_c + alpha, 100)
    ax.plot(r_self * np.cos(arc_angles), r_self * np.sin(arc_angles), color="#f2cc8f", linewidth=4, label="阻塞角区间")

    # Points
    ax.scatter([origin[0], q[0], a[0], b[0]], [origin[1], q[1], a[1], b[1]], color="#222")

    # Labels
    ax.text(origin[0] - 0.05, origin[1] - 0.08, "O", fontsize=12)
    ax.text(q[0] + 0.03, q[1] + 0.02, "q(t)", fontsize=12)
    ax.text(a[0] + 0.03, a[1] - 0.02, "A", fontsize=12)
    ax.text(b[0] + 0.03, b[1] + 0.02, "B", fontsize=12)
    ax.text(q[0] - 0.18, q[1] + 0.08, "R", color="#e07a5f", fontsize=11)
    ax.text(0.4, 0.1, "θc", color="#3d405b", fontsize=11)
    ax.text(0.65, 0.55, "α", color="#81b29a", fontsize=11)

    ax.set_aspect("equal")
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.axis("off")
    ax.set_title("Predictive Scan: 中心角/半角/阻塞区间示意", fontsize=12)

    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
