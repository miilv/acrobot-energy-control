import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D


def create_animation(acrobot, results, fps=30, max_frames=300):
    """Animate the acrobot swing-up with a ghost overlay of the upright target.

    Forward kinematics (angle measured from horizontal):
        joint 1:  (l1*cos(q1),  l1*sin(q1))
        tip:      (j1_x + l2*cos(q1+q2),  j1_y + l2*sin(q1+q2))
    At q1 = pi/2 both links point straight up on screen.
    """
    t = results["t"]
    q1, q2 = results["q1"], results["q2"]

    # Forward kinematics for every output time step
    j1_x = acrobot.l1 * np.cos(q1)
    j1_y = acrobot.l1 * np.sin(q1)
    tip_x = j1_x + acrobot.l2 * np.cos(q1 + q2)
    tip_y = j1_y + acrobot.l2 * np.sin(q1 + q2)

    # Upright reference ghost (q1=pi/2, q2=0)
    ref_j1 = (0.0, acrobot.l1)
    ref_tip = (0.0, acrobot.l1 + acrobot.l2)

    # Sub-sample frames
    step = max(1, len(t) // max_frames)
    frame_idx = list(range(0, len(t), step))

    lim = acrobot.l1 + acrobot.l2 + 0.5
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Acrobot: Energy-Based Swing-Up + LQR Stabilization")

    # Ghost
    ax.plot(
        [0, ref_j1[0], ref_tip[0]], [0, ref_j1[1], ref_tip[1]],
        "o--", color="gray", alpha=0.25, lw=3, markersize=6,
    )

    # Pivot
    ax.plot(0, 0, "ks", markersize=10, zorder=5)

    # Links
    (link1,) = ax.plot([], [], "o-", lw=5, color="royalblue", markersize=8)
    (link2,) = ax.plot([], [], "o-", lw=5, color="crimson", markersize=8)

    # Legend
    ax.legend(
        handles=[
            Line2D([0], [0], color="royalblue", lw=4, marker="o", label="Link 1"),
            Line2D([0], [0], color="crimson", lw=4, marker="o", label="Link 2"),
            Line2D([0], [0], color="gray", lw=3, ls="--", alpha=0.4, label="Target"),
        ],
        loc="upper right",
    )

    # Text overlays
    kw = dict(transform=ax.transAxes, fontsize=10, fontfamily="monospace",
              verticalalignment="top")
    time_txt = ax.text(0.02, 0.97, "", **kw)
    energy_txt = ax.text(0.02, 0.93, "", **kw)
    torque_txt = ax.text(0.02, 0.89, "", **kw)

    def init():
        link1.set_data([], [])
        link2.set_data([], [])
        time_txt.set_text("")
        energy_txt.set_text("")
        torque_txt.set_text("")
        return link1, link2, time_txt, energy_txt, torque_txt

    def animate(i):
        link1.set_data([0, j1_x[i]], [0, j1_y[i]])
        link2.set_data([j1_x[i], tip_x[i]], [j1_y[i], tip_y[i]])
        time_txt.set_text(f"t = {t[i]:6.2f} s")
        energy_txt.set_text(
            f"E = {results['E'][i]:6.1f} J   (Er = {results['E_upright']:.1f} J)"
        )
        torque_txt.set_text(f"\u03c4\u2082 = {results['u'][i]:7.1f} N\u00b7m")
        return link1, link2, time_txt, energy_txt, torque_txt

    ani = FuncAnimation(
        fig, animate, frames=frame_idx,
        interval=1000 / fps, blit=True, init_func=init,
    )
    plt.close(fig)
    return ani
