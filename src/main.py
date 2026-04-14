"""Entry point: simulate the acrobot swing-up, generate plots and animation.

Usage (from the project root directory):
    python -m src.main              # full run (plots + animation)
    python -m src.main --no-anim    # skip animation (faster)
"""

import argparse
from pathlib import Path

import numpy as np

from configs.params import PhysicalParams, ControlParams, SimParams
from src.system import Acrobot
from src.controller import EnergySwingUpController, LQRController
from src.simulation import simulate
from src.visualization import generate_all_plots
from src.animation import create_animation

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(description="Acrobot swing-up simulation")
    parser.add_argument("--no-anim", action="store_true",
                        help="Skip GIF generation (faster)")
    args = parser.parse_args()

    phys = PhysicalParams()
    ctrl = ControlParams()
    sim = SimParams()

    # --- Build system ---
    acrobot = Acrobot(phys)

    kD_min = acrobot.solvability_bound()
    print(f"Solvability bound:  kD > {kD_min:.3f}")
    print(f"Using kD = {ctrl.kD}  (margin {ctrl.kD - kD_min:.3f})")
    assert ctrl.kD > kD_min, (
        f"kD = {ctrl.kD} does not exceed solvability bound {kD_min:.3f}"
    )

    # --- Build controllers ---
    swing_up = EnergySwingUpController(acrobot, ctrl.kD, ctrl.kP, ctrl.kV, ctrl.u_max)

    Q = np.diag(ctrl.Q_diag)
    R = np.array([[ctrl.R_val]])
    lqr = LQRController(acrobot, Q, R)
    print(f"LQR gain K = {np.array2string(lqr.K, precision=3)}")

    # --- Simulate ---
    print(f"Simulating {sim.t_final} s from x0 = {sim.x0} ...")
    results = simulate(
        acrobot, swing_up, lqr, ctrl.u_max, ctrl.switch_threshold,
        sim.x0, sim.t_final, sim.dt,
    )

    if results["switch_time"] is not None:
        print(f"Switched to LQR at t = {results['switch_time']:.2f} s")
    else:
        print("Warning: LQR switch did not occur within the simulation window.")

    # --- Plots ---
    figures_dir = PROJECT_ROOT / "figures"
    print(f"Generating plots in {figures_dir}/ ...")
    generate_all_plots(results, ctrl.u_max, figures_dir)
    print("Plots saved.")

    # --- Animation ---
    if not args.no_anim:
        anim_dir = PROJECT_ROOT / "animations"
        anim_dir.mkdir(parents=True, exist_ok=True)
        gif_path = anim_dir / "acrobot_swingup.gif"
        print(f"Creating animation: {gif_path} ...")
        ani = create_animation(acrobot, results)
        ani.save(str(gif_path), writer="pillow", fps=30)
        print("Animation saved.")
    else:
        print("Skipping animation (--no-anim).")

    print("Done.")


if __name__ == "__main__":
    main()
