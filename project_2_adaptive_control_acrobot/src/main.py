"""Entry point: runs all scenarios, writes plots and animations.

Scenarios
---------
``no_friction``       Project-1 controller on the frictionless plant.
                      Sanity baseline (should match Project 1 results).
``friction_no_adapt`` Project-1 controller on the friction plant.
                      Demonstrates that ignoring friction breaks the
                      swing-up (mandatory comparison for Project 2+).
``friction_adaptive`` Adaptive certainty-equivalence controller on the
                      friction plant.  The team's main result.
"""

import argparse
import os
from pathlib import Path

import numpy as np

from configs.params import (
    AdaptiveParams,
    ControlParams,
    FrictionParams,
    PhysicalParams,
    SimParams,
)
from src.system import Acrobot
from src.controller import (
    AdaptiveSwingUpController,
    EnergySwingUpController,
    LQRController,
)
from src.simulation import simulate
from src import visualization, animation


ROOT = Path(__file__).resolve().parents[1]
FIG_DIR = ROOT / "figures"
ANIM_DIR = ROOT / "animations"
SCENARIOS = ("no_friction", "friction_no_adapt", "friction_adaptive")


def _build_lqr(acrobot, ctrl, b2_for_lqr):
    Q = np.diag(ctrl.Q_diag)
    R = np.array([[ctrl.R_val]])
    return LQRController(acrobot, Q, R, b2_for_lqr=b2_for_lqr)


def run_no_friction(phys, ctrl, sim):
    """Frictionless Project 1 baseline."""
    acro = Acrobot(phys, b2=0.0)
    swing = EnergySwingUpController(acro, ctrl.kD, ctrl.kP, ctrl.kV, ctrl.u_max)
    lqr = _build_lqr(acro, ctrl, b2_for_lqr=0.0)
    out = simulate(
        acro, swing, lqr, ctrl.u_max, ctrl.switch_threshold_p1,
        x0=(-1.4, 0.0, 0.0, 0.0), t_final=sim.t_final, dt=sim.dt,
    )
    return acro, out


def run_friction_no_adapt(phys, fric, ctrl, sim):
    """Project 1 controller on the friction plant — should fail."""
    acro = Acrobot(phys, b2=fric.b2_true)
    swing = EnergySwingUpController(acro, ctrl.kD, ctrl.kP, ctrl.kV, ctrl.u_max)
    lqr = _build_lqr(acro, ctrl, b2_for_lqr=0.0)
    out = simulate(
        acro, swing, lqr, ctrl.u_max, ctrl.switch_threshold,
        x0=sim.x0, t_final=sim.t_final, dt=sim.dt,
    )
    return acro, out


def run_friction_adaptive(phys, fric, ctrl, ad, sim):
    """Certainty-equivalence adaptive controller on the friction plant."""
    acro = Acrobot(phys, b2=fric.b2_true)
    swing = AdaptiveSwingUpController(
        acro, ctrl.kD, ctrl.kP, ctrl.kV, ctrl.u_max,
        gamma=ad.gamma, b_hat_0=ad.b2_hat_0,
    )
    lqr = _build_lqr(acro, ctrl, b2_for_lqr=fric.b2_true)
    out = simulate(
        acro, swing, lqr, ctrl.u_max, ctrl.switch_threshold,
        x0=sim.x0, t_final=sim.t_final, dt=sim.dt,
    )
    return acro, out


def write_scenario_artifacts(name, acro, result, ctrl, no_anim=False):
    """Save per-scenario plots (and animation unless skipped)."""
    base = FIG_DIR / name
    base.mkdir(parents=True, exist_ok=True)
    visualization.plot_state_trajectories(result, str(base / "states.png"))
    visualization.plot_energy(result, str(base / "energy.png"))
    visualization.plot_lyapunov(result, str(base / "lyapunov.png"))
    visualization.plot_control(result, str(base / "control.png"), u_max=ctrl.u_max)
    visualization.plot_phase_portrait(result, str(base / "phase_portrait.png"))
    visualization.plot_tracking_error(result, str(base / "tracking_error.png"))
    if result["is_adaptive"]:
        visualization.plot_friction_estimate(result, str(base / "friction_estimate.png"))
    if not no_anim:
        ANIM_DIR.mkdir(parents=True, exist_ok=True)
        animation.render(acro, result, str(ANIM_DIR / f"{name}.gif"))


def write_comparison_plots(results):
    visualization.plot_comparison_energy(results, str(FIG_DIR / "comparison_energy.png"))
    visualization.plot_comparison_error(results, str(FIG_DIR / "comparison_error.png"))


def _summary(name, result):
    sw = result["switch_time"]
    sw_str = f"{sw:.2f} s" if sw is not None else "never"
    e0 = (result["q1"][-1] - np.pi / 2 + np.pi) % (2.0 * np.pi) - np.pi
    err = (abs(e0) + abs(result["q2"][-1])
           + 0.1 * abs(result["dq1"][-1]) + 0.1 * abs(result["dq2"][-1]))
    msg = (f"  {name:22s}  switch = {sw_str:>9s}   final V = {result['V'][-1]:.4g}   "
           f"final |error| = {err:.4g}")
    if result["is_adaptive"]:
        msg += f"   b_hat_end = {result['b_hat'][-1]:.3f}  (true {result['b_true']:.2f})"
    print(msg)


def main():
    parser = argparse.ArgumentParser(description="Adaptive friction control of an acrobot.")
    parser.add_argument("--scenario", choices=SCENARIOS + ("all",), default="all",
                        help="Which scenario to run (default: all).")
    parser.add_argument("--no-anim", action="store_true",
                        help="Skip GIF animation rendering (much faster).")
    args = parser.parse_args()

    phys = PhysicalParams()
    fric = FrictionParams()
    ctrl = ControlParams()
    ad = AdaptiveParams()
    sim = SimParams()

    print(f"True friction b2 = {fric.b2_true} N*m*s/rad")
    print(f"Adaptation gain gamma = {ad.gamma}")
    print(f"Energy-shape gains kD={ctrl.kD}, kP={ctrl.kP}, kV={ctrl.kV}\n")

    runs = {}
    if args.scenario in ("all", "no_friction"):
        acro, out = run_no_friction(phys, ctrl, sim)
        write_scenario_artifacts("no_friction", acro, out, ctrl, no_anim=args.no_anim)
        runs["no_friction"] = out
    if args.scenario in ("all", "friction_no_adapt"):
        acro, out = run_friction_no_adapt(phys, fric, ctrl, sim)
        write_scenario_artifacts("friction_no_adapt", acro, out, ctrl, no_anim=args.no_anim)
        runs["friction_no_adapt"] = out
    if args.scenario in ("all", "friction_adaptive"):
        acro, out = run_friction_adaptive(phys, fric, ctrl, ad, sim)
        write_scenario_artifacts("friction_adaptive", acro, out, ctrl, no_anim=args.no_anim)
        runs["friction_adaptive"] = out

    if args.scenario == "all":
        write_comparison_plots(runs)

    print("\nResults summary:")
    for name in SCENARIOS:
        if name in runs:
            _summary(name, runs[name])

    print(f"\nFigures: {FIG_DIR}")
    if not args.no_anim:
        print(f"Animations: {ANIM_DIR}")


if __name__ == "__main__":
    main()
