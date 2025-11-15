from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from .pid import PI


@dataclass
class MotorParamsBLDC:
    R: float = 0.5
    L: float = 0.0002
    Kt: float = 0.06         # N·m/A
    Ke: float = 0.06         # V·s/rad
    J: float = 2.0e-4
    B: float = 1.0e-4
    Vbus: float = 24.0
    Imax: float = 10.0


@dataclass
class GainsBLDC:
    spd_kp: float = 0.01
    spd_ki: float = 2.0
    cur_kp: float = 0.8
    cur_ki: float = 120.0


class BLDCQSim:
    """Simplified BLDC q-axis model with vector-like current control.

    Electrical: di/dt = (v - Ke*omega - R*i) / L
    Mechanical: dω/dt = (Kt*i - B*ω - Tload) / J
    Control: speed PI → i_ref; current PI → v (clamped to ±Vbus)
    """

    def __init__(self, params: MotorParamsBLDC, gains: GainsBLDC):
        self.p = params
        self.g = gains
        self.i = 0.0
        self.omega = 0.0
        self.pi_spd = PI(self.g.spd_kp, self.g.spd_ki, -self.p.Imax, self.p.Imax)
        self.pi_i = PI(self.g.cur_kp, self.g.cur_ki, -self.p.Vbus, self.p.Vbus)
        self.i_ref = 0.0

    def reset(self):
        self.i = 0.0
        self.omega = 0.0
        self.pi_spd.reset(); self.pi_i.reset()

    def step(self, dt: float, mode: str, rpm_target: float, torque_ref: float, t_load: float):
        # Reference
        if mode == "torque":
            self.i_ref = max(min(torque_ref / self.p.Kt, self.p.Imax), -self.p.Imax)
        else:
            omega_ref = rpm_target * 2.0 * 3.141592653589793 / 60.0
            spd_err = omega_ref - self.omega
            self.i_ref = self.pi_spd.step(spd_err, dt)

        # Current PI → voltage command
        i_err = self.i_ref - self.i
        v = self.pi_i.step(i_err, dt)

        # Electrical dynamics
        di_dt = (v - self.p.Ke * self.omega - self.p.R * self.i) / self.p.L
        self.i += di_dt * dt

        # Mechanical
        Te = self.p.Kt * self.i
        domega_dt = (Te - self.p.B * self.omega - t_load) / self.p.J
        self.omega += domega_dt * dt

        return self.omega, self.i, v, Te

