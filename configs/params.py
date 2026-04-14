from dataclasses import dataclass


@dataclass
class PhysicalParams:
    """Physical parameters of the acrobot."""

    m1: float = 1.0    # mass of link 1 [kg]
    m2: float = 1.0    # mass of link 2 [kg]
    l1: float = 1.0    # length of link 1 [m]
    l2: float = 2.0    # length of link 2 [m]
    lc1: float = 0.5   # distance to center of mass of link 1 [m]
    lc2: float = 1.0   # distance to center of mass of link 2 [m]
    I1: float = 0.083  # moment of inertia of link 1 [kg*m^2]
    I2: float = 0.33   # moment of inertia of link 2 [kg*m^2]
    g: float = 9.8     # gravitational acceleration [m/s^2]


@dataclass
class ControlParams:
    """Controller tuning parameters."""

    kD: float = 35.8    # energy-shaping damping gain (must exceed solvability bound ~35.74)
    kP: float = 61.2    # energy-shaping position gain
    kV: float = 66.3    # energy-shaping velocity damping gain
    u_max: float = 50.0  # torque saturation limit [N*m]
    switch_threshold: float = 0.04  # weighted state-error norm for LQR activation
    Q_diag: tuple = (10.0, 10.0, 1.0, 1.0)  # LQR state cost diagonal entries
    R_val: float = 5.0  # LQR input cost (larger → less aggressive gains)


@dataclass
class SimParams:
    """Simulation parameters."""

    t_final: float = 30.0  # simulation duration [s]
    dt: float = 0.005      # output time step [s]
    x0: tuple = (-1.4, 0.0, 0.0, 0.0)  # initial state [q1, q2, dq1, dq2]
