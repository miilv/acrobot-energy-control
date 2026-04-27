import numpy as np
from scipy.linalg import solve_continuous_are


class EnergySwingUpController:
    """Project-1 energy-based swing-up controller (no friction compensation).

    Uses the Lyapunov candidate
        V = (1/2)(E - E_r)^2 + (1/2) kD dq2^2 + (1/2) kP q2^2.
    The control law that yields V_dot = -kV dq2^2 (under zero friction)
    is

        tau_2 = - [ (kV dq2 + kP q2) Delta + kD (M21 (H1+G1) - M11 (H2+G2)) ]
                 / [ kD M11 + (E - E_r) Delta ].

    This controller is used both as a baseline (it FAILS when the plant
    has unknown friction) and as the nominal controller inside the
    adaptive design.
    """

    def __init__(self, acrobot, kD, kP, kV, u_max, b_feedforward=0.0):
        self.acrobot = acrobot
        self.kD = kD
        self.kP = kP
        self.kV = kV
        self.u_max = u_max
        # Optional fixed friction feedforward: when ``b_feedforward`` equals
        # the true plant b2 this exactly cancels the friction term and
        # recovers the frictionless V_dot identity.  Leave at 0.0 for the
        # naive baseline.
        self.b_feedforward = float(b_feedforward)

    def compute(self, state):
        return self._tau(state, b_hat=self.b_feedforward)

    def _tau(self, state, b_hat):
        q1, q2, dq1, dq2 = state
        ac = self.acrobot

        E_err = ac.total_energy(q1, q2, dq1, dq2) - ac.E_upright

        M = ac.mass_matrix(q2)
        M11, M12, M21, M22 = M[0, 0], M[0, 1], M[1, 0], M[1, 1]
        Delta = M11 * M22 - M12 * M21

        C = ac.coriolis(q2, dq1, dq2)
        G = ac.gravity(q1, q2)

        # b_hat * dq2 is added to the row-2 dynamics (same place real friction
        # enters), so the controller compensates it the same way it
        # compensates Coriolis + gravity.
        H2_eff = C[1] + G[1] + b_hat * dq2

        numerator = (
            (self.kV * dq2 + self.kP * q2) * Delta
            + self.kD * (M21 * (C[0] + G[0]) - M11 * H2_eff)
        )
        denominator = self.kD * M11 + E_err * Delta

        u = -numerator / denominator
        return float(np.clip(u, -self.u_max, self.u_max))


class AdaptiveSwingUpController:
    """Certainty-equivalence adaptive swing-up controller.

    Augments the swing-up Lyapunov function with a parameter-error term
    (b_hat - b)^2 / (2 gamma).  Choosing the adaptation law

        d(b_hat)/dt = - gamma * (E - E_r) * dq2^2

    yields, after substituting the certainty-equivalence control law,
    the same dissipation identity as the frictionless P1 controller:

        V_dot = - kV dq2^2 <= 0.

    LaSalle then gives convergence of the state to the largest
    invariant set on which dq2 = 0; under persistence of excitation,
    b_hat -> b.

    The estimate ``b_hat`` is held by the controller.  For event-driven
    integration the simulator augments the ODE state with b_hat and
    queries ``adapt_rate`` for its derivative; ``compute`` is then
    given an explicit b_hat so the simulator and controller agree.
    """

    def __init__(
        self,
        acrobot,
        kD,
        kP,
        kV,
        u_max,
        gamma,
        b_hat_0=0.0,
    ):
        self._nominal = EnergySwingUpController(acrobot, kD, kP, kV, u_max)
        self.acrobot = acrobot
        self.gamma = float(gamma)
        self.b_hat_0 = float(b_hat_0)
        self.b_hat = self.b_hat_0  # mutable internal state, kept in sync by sim

    @property
    def kD(self):
        return self._nominal.kD

    @property
    def kP(self):
        return self._nominal.kP

    @property
    def kV(self):
        return self._nominal.kV

    @property
    def u_max(self):
        return self._nominal.u_max

    def reset(self):
        self.b_hat = self.b_hat_0

    def compute(self, state, b_hat=None):
        """Adaptive torque using the supplied (or stored) friction estimate."""
        b = self.b_hat if b_hat is None else float(b_hat)
        return self._nominal._tau(state, b_hat=b)

    def adapt_rate(self, state):
        """d(b_hat)/dt at the supplied state."""
        q1, q2, dq1, dq2 = state[:4]
        E_err = self.acrobot.total_energy(q1, q2, dq1, dq2) - self.acrobot.E_upright
        return -self.gamma * E_err * dq2 * dq2

    def lyapunov(self, state, b_hat=None, b_true=None):
        """Adaptive Lyapunov function value V(s, b_hat).

        If ``b_true`` is provided the parameter-error term uses
        (b_hat - b_true); otherwise the term is omitted (useful when
        plotting the certainty-equivalence component only).
        """
        q1, q2, dq1, dq2 = state[:4]
        b = self.b_hat if b_hat is None else float(b_hat)
        E_err = self.acrobot.total_energy(q1, q2, dq1, dq2) - self.acrobot.E_upright
        V_nom = 0.5 * E_err**2 + 0.5 * self.kD * dq2**2 + 0.5 * self.kP * q2**2
        if b_true is None:
            return V_nom
        return V_nom + 0.5 * (b - b_true) ** 2 / self.gamma


class LQRController:
    """LQR stabilization about the upright equilibrium.

    The linearization can optionally include the controller's current
    friction estimate as extra damping; this lets a single LQR design
    track the post-handover plant when adaptation has roughly converged.
    """

    def __init__(self, acrobot, Q, R, b2_for_lqr=0.0):
        A, B = acrobot.linearize_at_upright(b2_for_lqr=b2_for_lqr)
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.solve(R, B.T @ P).flatten()
        self.x_ref = np.array([np.pi / 2, 0.0, 0.0, 0.0])

    def compute(self, state):
        x_err = np.array(state[:4]) - self.x_ref
        x_err[0] = (x_err[0] + np.pi) % (2.0 * np.pi) - np.pi
        return float(-self.K @ x_err)


def state_error_norm(state):
    """Weighted state-error norm relative to the upright equilibrium."""
    e0 = (state[0] - np.pi / 2 + np.pi) % (2.0 * np.pi) - np.pi
    return abs(e0) + abs(state[1]) + 0.1 * abs(state[2]) + 0.1 * abs(state[3])
