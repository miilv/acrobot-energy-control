import numpy as np
from scipy.linalg import solve_continuous_are


class EnergySwingUpController:
    """Energy-based swing-up controller (Xin & Kaneda, 2007).

    Drives the total energy E toward the upright energy E_r using the
    Lyapunov function  V = (1/2)(E - E_r)^2 + (1/2)kD*dq2^2 + (1/2)kP*q2^2,
    yielding V_dot = -kV * dq2^2 <= 0.

    Control law:
        tau2 = -[ (kV*dq2 + kP*q2)*Delta + kD*(M21*(H1+G1) - M11*(H2+G2)) ]
               / [ kD*M11 + (E - Er)*Delta ]
    """

    def __init__(self, acrobot, kD, kP, kV, u_max):
        self.acrobot = acrobot
        self.kD = kD
        self.kP = kP
        self.kV = kV
        self.u_max = u_max

    def compute(self, state):
        q1, q2, dq1, dq2 = state
        ac = self.acrobot

        E_err = ac.total_energy(q1, q2, dq1, dq2) - ac.E_upright

        M = ac.mass_matrix(q2)
        M11, M12, M21, M22 = M[0, 0], M[0, 1], M[1, 0], M[1, 1]
        Delta = M11 * M22 - M12 * M21

        C = ac.coriolis(q2, dq1, dq2)
        G = ac.gravity(q1, q2)

        numerator = (self.kV * dq2 + self.kP * q2) * Delta + self.kD * (
            M21 * (C[0] + G[0]) - M11 * (C[1] + G[1])
        )
        denominator = self.kD * M11 + E_err * Delta

        u = -numerator / denominator
        return np.clip(u, -self.u_max, self.u_max)


class LQRController:
    """Linear-quadratic regulator for stabilization near the upright equilibrium.

    Solves the continuous-time algebraic Riccati equation for the linearized
    acrobot dynamics, yielding optimal gain K such that u = -K x.
    """

    def __init__(self, acrobot, Q, R):
        A, B = acrobot.linearize_at_upright()
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.solve(R, B.T @ P).flatten()  # K = R^{-1} B^T P, shape (4,)
        self.x_ref = np.array([np.pi / 2, 0.0, 0.0, 0.0])

    def compute(self, state):
        x_err = np.array(state) - self.x_ref
        x_err[0] = (x_err[0] + np.pi) % (2.0 * np.pi) - np.pi  # wrap to [-pi, pi]
        return float(-self.K @ x_err)


class SwitchingController:
    """Two-phase controller: energy swing-up followed by LQR stabilization.

    Switches from swing-up to LQR when the weighted state-error norm
    falls below a threshold.  The switch is one-way (once in LQR, stays in LQR).
    """

    def __init__(self, swing_up, lqr, threshold, u_max):
        self.swing_up = swing_up
        self.lqr = lqr
        self.threshold = threshold
        self.u_max = u_max
        self.switched = False
        self.switch_time = None

    def _state_error_norm(self, state):
        e0 = (state[0] - np.pi / 2 + np.pi) % (2.0 * np.pi) - np.pi
        return abs(e0) + abs(state[1]) + 0.1 * abs(state[2]) + 0.1 * abs(state[3])

    def compute(self, state, t=None):
        if not self.switched and self._state_error_norm(state) < self.threshold:
            self.switched = True
            self.switch_time = t

        if self.switched:
            u = self.lqr.compute(state)
        else:
            u = self.swing_up.compute(state)

        return np.clip(u, -self.u_max, self.u_max)

    def reset(self):
        """Reset switching state for trajectory replay."""
        self.switched = False
        self.switch_time = None
