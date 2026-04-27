"""Acrobot animation, saved as GIF.

The animation shows both links, the upright target, and a small overlay
with current time, energy, control torque, and (for the adaptive
scenario) the friction parameter estimate vs true value.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


def render(acrobot, result, save_path, n_frames=300, fps=24):
    """Render a swing-up animation to a GIF.

    ``acrobot`` is used only to look up link lengths.  Frames are
    sub-sampled from the simulation so that the GIF size stays modest.
    """
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    n_total = len(result["t"])
    idx = np.linspace(0, n_total - 1, n_frames).astype(int)
    t = result["t"][idx]
    q1 = result["q1"][idx]
    q2 = result["q2"][idx]
    u = result["u"][idx]
    E = result["E"][idx]
    b_hat = result["b_hat"][idx] if result["is_adaptive"] else None

    l1, l2 = acrobot.l1, acrobot.l2
    x1 = l1 * np.cos(q1)
    y1 = l1 * np.sin(q1)
    x2 = x1 + l2 * np.cos(q1 + q2)
    y2 = y1 + l2 * np.sin(q1 + q2)

    R = l1 + l2 + 0.5
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-R, R)
    ax.set_ylim(-R, R)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    title = "Adaptive friction control" if result["is_adaptive"] else "Energy-based swing-up"
    ax.set_title(f"Acrobot: {title}")

    target_x = [0.0, 0.0, 0.0]
    target_y = [0.0, l1, l1 + l2]
    ax.plot(target_x, target_y, color="lightgray", linestyle="--", linewidth=2,
            label="target (upright)")

    line1, = ax.plot([], [], "o-", color="royalblue", linewidth=4, markersize=8, label="Link 1")
    line2, = ax.plot([], [], "o-", color="crimson", linewidth=4, markersize=8, label="Link 2")
    base = plt.Rectangle((-0.06, -0.06), 0.12, 0.12, color="black")
    ax.add_patch(base)
    ax.legend(loc="upper right", fontsize=9)

    text_block = ax.text(
        0.02, 0.98, "", transform=ax.transAxes, fontfamily="monospace",
        fontsize=10, verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.85, edgecolor="lightgray"),
    )

    def init():
        line1.set_data([], [])
        line2.set_data([], [])
        text_block.set_text("")
        return line1, line2, text_block

    def step(i):
        line1.set_data([0.0, x1[i]], [0.0, y1[i]])
        line2.set_data([x1[i], x2[i]], [y1[i], y2[i]])
        info = (
            f"t      = {t[i]:6.2f} s\n"
            f"E      = {E[i]:6.2f} J  (Er = {result['E_upright']:.2f})\n"
            f"tau_2  = {u[i]:+6.2f} N*m"
        )
        if result["is_adaptive"]:
            info += f"\nb_hat  = {b_hat[i]:6.3f}  (true {result['b_true']:.2f})"
        text_block.set_text(info)
        return line1, line2, text_block

    ani = FuncAnimation(fig, step, init_func=init, frames=n_frames, interval=1000 / fps, blit=True)
    ani.save(save_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
