"""Plotting routines.  Each function saves one figure into ``figures/``.

The visual style follows Project 1's plot conventions: title, labelled
axes with units, legend when more than one curve, switch-time vline
where applicable, grid on.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def _ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def plot_state_trajectories(result, save_path):
    """Joint angles and velocities versus time."""
    _ensure_dir(os.path.dirname(save_path))
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    t = result["t"]
    axes[0].plot(t, result["q1"] - np.pi / 2, label=r"$q_1 - \pi/2$ (link 1)")
    axes[0].plot(t, result["q2"], label=r"$q_2$ (link 2)")
    axes[0].set_ylabel("Angle [rad]")
    axes[0].set_title("Joint angles: convergence to upright equilibrium")
    axes[0].grid(True, alpha=0.4)
    axes[0].legend()

    axes[1].plot(t, result["dq1"], label=r"$\dot q_1$")
    axes[1].plot(t, result["dq2"], label=r"$\dot q_2$")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Angular velocity [rad/s]")
    axes[1].set_title("Joint velocities: damped after LQR switch")
    axes[1].grid(True, alpha=0.4)
    axes[1].legend()

    if result["switch_time"] is not None:
        for ax in axes:
            ax.axvline(result["switch_time"], color="green", linestyle=":",
                       alpha=0.7, label=f"LQR switch (t={result['switch_time']:.1f} s)")
        axes[0].legend()
        axes[1].legend()

    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_energy(result, save_path):
    """Total mechanical energy versus time, with target line."""
    _ensure_dir(os.path.dirname(save_path))
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(result["t"], result["E"], label=r"$E(t)$", linewidth=1.7)
    ax.axhline(result["E_upright"], color="red", linestyle="--",
               label=fr"$E_r = {result['E_upright']:.2f}$ J")
    if result["switch_time"] is not None:
        ax.axvline(result["switch_time"], color="green", linestyle=":",
                   label=f"LQR switch (t={result['switch_time']:.1f} s)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Total mechanical energy [J]")
    ax.set_title("Energy convergence: controller pumps energy toward $E_r$, then LQR holds it")
    ax.grid(True, alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_lyapunov(result, save_path):
    """Adaptive Lyapunov function V(t).

    For the adaptive scenario this includes the parameter-error term
    (b_hat - b)^2 / (2 gamma); for non-adaptive scenarios the term is
    omitted so the curve stays comparable to Project 1.
    """
    _ensure_dir(os.path.dirname(save_path))
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(result["t"], result["V"], color="purple", linewidth=1.6,
            label=r"$V(t)$")
    if result["switch_time"] is not None:
        ax.axvline(result["switch_time"], color="green", linestyle=":",
                   label=f"LQR switch (t={result['switch_time']:.1f} s)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$V$")
    ax.set_yscale("log")
    if result["is_adaptive"]:
        ax.set_title(r"Adaptive Lyapunov $V = \frac{1}{2}(E-E_r)^2 + "
                     r"\frac{1}{2}k_D \dot q_2^2 + \frac{1}{2}k_P q_2^2 + "
                     r"\frac{1}{2\gamma}(\hat b_2 - b_2)^2$")
    else:
        ax.set_title(r"Lyapunov $V = \frac{1}{2}(E-E_r)^2 + "
                     r"\frac{1}{2}k_D \dot q_2^2 + \frac{1}{2}k_P q_2^2$")
    ax.grid(True, which="both", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_friction_estimate(result, save_path):
    """Adaptive parameter estimate b_hat(t) compared with true b."""
    if not result["is_adaptive"]:
        return
    _ensure_dir(os.path.dirname(save_path))
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(result["t"], result["b_hat"], color="darkorange", linewidth=1.7,
            label=r"$\hat b_2(t)$ (estimate)")
    ax.axhline(result["b_true"], color="red", linestyle="--",
               label=fr"$b_2 = {result['b_true']:.2f}$ (true)")
    if result["switch_time"] is not None:
        ax.axvline(result["switch_time"], color="green", linestyle=":",
                   label=f"LQR switch (t={result['switch_time']:.1f} s)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"$\hat b_2$ [N$\cdot$m$\cdot$s/rad]")
    ax.set_title(r"Friction parameter adaptation: $\hat b_2$ tracks $b_2$ online")
    ax.grid(True, alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_control(result, save_path, u_max=50.0):
    """Control torque tau_2 versus time, with saturation lines."""
    _ensure_dir(os.path.dirname(save_path))
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(result["t"], result["u"], color="navy", linewidth=1.4,
            label=r"$\tau_2(t)$")
    ax.axhline(u_max, color="gray", linestyle=":", alpha=0.7,
               label=fr"$\pm u_{{max}} = {u_max:g}$ N$\cdot$m")
    ax.axhline(-u_max, color="gray", linestyle=":", alpha=0.7)
    if result["switch_time"] is not None:
        ax.axvline(result["switch_time"], color="green", linestyle=":",
                   label=f"LQR switch (t={result['switch_time']:.1f} s)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Torque $\tau_2$ [N$\cdot$m]")
    ax.set_title("Control input: stays within saturation limits throughout")
    ax.grid(True, alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_phase_portrait(result, save_path):
    """Phase portrait of joint 2 with time-coloured trajectory."""
    _ensure_dir(os.path.dirname(save_path))
    fig, ax = plt.subplots(figsize=(8, 7))
    pts = np.array([result["q2"], result["dq2"]]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    norm = plt.Normalize(result["t"].min(), result["t"].max())
    lc = LineCollection(segs, cmap="plasma", norm=norm, linewidth=1.6)
    lc.set_array(result["t"][:-1])
    ax.add_collection(lc)
    ax.scatter(result["q2"][0], result["dq2"][0], color="green", s=120,
               marker="o", label="start", zorder=5)
    ax.scatter(result["q2"][-1], result["dq2"][-1], color="red", s=120,
               marker="x", label="end", zorder=5)
    ax.set_xlim(result["q2"].min() - 0.1, result["q2"].max() + 0.1)
    ax.set_ylim(result["dq2"].min() - 0.5, result["dq2"].max() + 0.5)
    ax.set_xlabel(r"$q_2$ [rad]")
    ax.set_ylabel(r"$\dot q_2$ [rad/s]")
    ax.set_title("Phase portrait of link 2: trajectory spirals to the upright fixed point")
    ax.grid(True, alpha=0.4)
    ax.legend()
    cbar = fig.colorbar(lc, ax=ax)
    cbar.set_label("Time [s]")
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_tracking_error(result, save_path):
    """Weighted state-error norm on a log scale."""
    _ensure_dir(os.path.dirname(save_path))
    e0 = (result["q1"] - np.pi / 2 + np.pi) % (2.0 * np.pi) - np.pi
    err = (np.abs(e0) + np.abs(result["q2"])
           + 0.1 * np.abs(result["dq1"]) + 0.1 * np.abs(result["dq2"]))
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.semilogy(result["t"], np.maximum(err, 1e-12), color="darkred", linewidth=1.5)
    if result["switch_time"] is not None:
        ax.axvline(result["switch_time"], color="green", linestyle=":",
                   label=f"LQR switch (t={result['switch_time']:.1f} s)")
        ax.legend()
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Weighted state-error norm")
    ax.set_title("Tracking error: exponential decay after the LQR switch")
    ax.grid(True, which="both", alpha=0.4)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_comparison_energy(results_by_name, save_path):
    """Overlay total energy E(t) for several scenarios on one figure."""
    _ensure_dir(os.path.dirname(save_path))
    fig, ax = plt.subplots(figsize=(11, 5))
    colors = {"no_friction": "steelblue", "friction_no_adapt": "crimson",
              "friction_adaptive": "seagreen"}
    labels = {"no_friction": "no friction (P1 baseline)",
              "friction_no_adapt": "friction, no adaptation",
              "friction_adaptive": "friction, adaptive (ours)"}
    for name, r in results_by_name.items():
        ax.plot(r["t"], r["E"], color=colors.get(name, None),
                linewidth=1.5, label=labels.get(name, name))
    Er = next(iter(results_by_name.values()))["E_upright"]
    ax.axhline(Er, color="black", linestyle="--", alpha=0.6,
               label=fr"$E_r = {Er:.2f}$ J")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Total mechanical energy [J]")
    ax.set_title("Comparison: energy convergence under different control strategies")
    ax.grid(True, alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_comparison_error(results_by_name, save_path):
    """Overlay weighted error norm for several scenarios."""
    _ensure_dir(os.path.dirname(save_path))
    fig, ax = plt.subplots(figsize=(11, 5))
    colors = {"no_friction": "steelblue", "friction_no_adapt": "crimson",
              "friction_adaptive": "seagreen"}
    labels = {"no_friction": "no friction (P1 baseline)",
              "friction_no_adapt": "friction, no adaptation",
              "friction_adaptive": "friction, adaptive (ours)"}
    for name, r in results_by_name.items():
        e0 = (r["q1"] - np.pi / 2 + np.pi) % (2.0 * np.pi) - np.pi
        err = (np.abs(e0) + np.abs(r["q2"])
               + 0.1 * np.abs(r["dq1"]) + 0.1 * np.abs(r["dq2"]))
        ax.semilogy(r["t"], np.maximum(err, 1e-12), color=colors.get(name, None),
                    linewidth=1.5, label=labels.get(name, name))
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Weighted state-error norm (log scale)")
    ax.set_title("Comparison: only the adaptive controller drives the error to zero")
    ax.grid(True, which="both", alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
