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
    p: int = 4               # pole pairs (for electrical speed)
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


class BLDCTrapSim:
    """Trapezoidal (six-step style) BLDC controller on a q-axis plant.

    Uses a simple speed PI to generate a duty in [-1, 1]. The commanded
    phase voltage is v = duty * Vbus. This approximates six-step behavior
    without electrical angle/commutation tables in this simplified model.
    """

    def __init__(self, params: MotorParamsBLDC, gains: GainsBLDC):
        self.p = params
        self.g = gains
        self.i = 0.0
        self.omega = 0.0
        self.pi_spd = PI(self.g.spd_kp, self.g.spd_ki, -1.0, 1.0)  # output as duty
        self.theta_e = 0.0  # electrical angle [rad]

    def reset(self):
        self.i = 0.0
        self.omega = 0.0
        self.pi_spd.reset()

    def _emf_norm(self, theta: float) -> float:
        """Trapezoidal back-EMF normalized to [-1,1] for a single effective phase.
        0..pi/6: ramp 0->1, pi/6..5pi/6: 1, 5pi/6..7pi/6: ramp 1->-1,
        7pi/6..11pi/6: -1, 11pi/6..2pi: ramp -1->0
        """
        import math
        th = theta % (2.0 * math.pi)
        pi = math.pi
        if 0.0 <= th < pi/6:
            return th / (pi/6)
        if pi/6 <= th < 5*pi/6:
            return 1.0
        if 5*pi/6 <= th < 7*pi/6:
            return 1.0 - 2.0 * (th - 5*pi/6) / (pi/3)
        if 7*pi/6 <= th < 11*pi/6:
            return -1.0
        # 11pi/6 .. 2pi
        return -1.0 + (th - 11*pi/6) / (pi/6)

    def step(self, dt: float, mode: str, rpm_target: float, torque_ref: float, t_load: float):
        # Duty command
        if mode == "torque":
            # Rough mapping torque→duty via current and Ohm's law/back-EMF drop
            i_ref = max(min(torque_ref / self.p.Kt, self.p.Imax), -self.p.Imax)
            # Estimate v needed to achieve i_ref in one step (very crude)
            v_cmd = self.p.R * i_ref + self.p.Ke * self.omega
            duty = max(min(v_cmd / self.p.Vbus, 1.0), -1.0)
        else:
            omega_ref = rpm_target * 2.0 * 3.141592653589793 / 60.0
            spd_err = omega_ref - self.omega
            duty = self.pi_spd.step(spd_err, dt)

        v = duty * self.p.Vbus

        # Electrical angle update
        omega_e = self.p.p * self.omega
        self.theta_e = (self.theta_e + omega_e * dt) % (2.0 * 3.141592653589793)
        e_norm = self._emf_norm(self.theta_e)

        # Electrical dynamics
        di_dt = (v - self.p.Ke * self.omega * e_norm - self.p.R * self.i) / self.p.L
        self.i += di_dt * dt

        # Mechanical
        Te = self.p.Kt * self.i
        domega_dt = (Te - self.p.B * self.omega - t_load) / self.p.J
        self.omega += domega_dt * dt

        return self.omega, self.i, v, Te, duty


class BLDCSixStepSim:
    """3‑phase six‑step BLDC with simple commutation table and trapezoidal EMF.

    - Phase currents: i_a, i_b, i_c (star connection, no neutral).
    - Phase EMFs: e_a = Ke*ω*f(θe), e_b = Ke*ω*f(θe-2π/3), e_c = Ke*ω*f(θe+2π/3).
    - Voltages: per sector, two phases are driven ±Vbus, one is floating; we
      subtract average to get neutral‑referenced phase voltages.
    - Control: speed PI → duty ∈ [0,1]; torque mode maps torque→duty crudely.
    """

    def __init__(self, params: MotorParamsBLDC, gains: GainsBLDC):
        self.p = params
        self.g = gains
        self.ia = 0.0; self.ib = 0.0; self.ic = 0.0
        self.omega = 0.0
        self.theta_e = 0.0
        self.pi_spd = PI(self.g.spd_kp, self.g.spd_ki, 0.0, 1.0)  # duty 0..1

    @staticmethod
    def _trap(th: float) -> float:
        import math
        th = th % (2.0 * math.pi)
        pi = math.pi
        if 0.0 <= th < pi/6: return th/(pi/6)
        if pi/6 <= th < 5*pi/6: return 1.0
        if 5*pi/6 <= th < 7*pi/6: return 1.0 - 2.0*(th-5*pi/6)/(pi/3)
        if 7*pi/6 <= th < 11*pi/6: return -1.0
        return -1.0 + (th-11*pi/6)/(pi/6)

    def _sector(self, th: float) -> int:
        import math
        return int((th % (2.0*math.pi)) // (math.pi/3))  # 0..5

    def reset(self):
        self.ia = self.ib = self.ic = 0.0
        self.omega = 0.0
        self.theta_e = 0.0
        self.pi_spd.reset()

    def step(self, dt: float, mode: str, rpm_target: float, torque_ref: float, t_load: float):
        import math
        # Duty generation
        if mode == "torque":
            i_ref = max(min(torque_ref / self.p.Kt, self.p.Imax), -self.p.Imax)
            v_est = self.p.R * i_ref + self.p.Ke * self.omega
            duty = max(0.0, min(abs(v_est) / max(self.p.Vbus, 1e-6), 1.0))
        else:
            omega_ref = rpm_target * 2.0 * math.pi / 60.0
            spd_err = omega_ref - self.omega
            duty = self.pi_spd.step(spd_err, dt)

        # Electrical angle and EMFs
        omega_e = self.p.p * self.omega
        self.theta_e = (self.theta_e + omega_e * dt) % (2.0 * math.pi)
        ea = self.p.Ke * self.omega * self._trap(self.theta_e)
        eb = self.p.Ke * self.omega * self._trap(self.theta_e - 2.0*math.pi/3.0)
        ec = self.p.Ke * self.omega * self._trap(self.theta_e + 2.0*math.pi/3.0)

        # Commutation voltages per sector (A,B,C): (+V, -V, Z)
        sec = self._sector(self.theta_e)
        pattern = [
            (1, -1, 0),  # 0: A+,B-,Cz
            (1, 0, -1),  # 1: A+,C-,Bz
            (0, 1, -1),  # 2: B+,C-,Az
            (-1, 1, 0),  # 3: A-,B+,Cz
            (-1, 0, 1),  # 4: A-,C+,Bz
            (0, -1, 1),  # 5: B-,C+,Az
        ][sec]
        va_t = duty * self.p.Vbus * pattern[0]
        vb_t = duty * self.p.Vbus * pattern[1]
        vc_t = duty * self.p.Vbus * pattern[2]
        v_avg = (va_t + vb_t + vc_t) / 3.0
        va = va_t - v_avg; vb = vb_t - v_avg; vc = vc_t - v_avg

        # Phase dynamics
        dia = (va - ea - self.p.R * self.ia) / self.p.L
        dib = (vb - eb - self.p.R * self.ib) / self.p.L
        dic = (vc - ec - self.p.R * self.ic) / self.p.L
        self.ia += dia * dt; self.ib += dib * dt; self.ic += dic * dt

        # Torque approximation from phase contributions
        Te = self.p.Kt * (self._trap(self.theta_e) * self.ia + self._trap(self.theta_e - 2.0*math.pi/3.0) * self.ib + self._trap(self.theta_e + 2.0*math.pi/3.0) * self.ic)

        domega = (Te - self.p.B * self.omega - t_load) / self.p.J
        self.omega += domega * dt

        return self.omega, (self.ia, self.ib, self.ic), (va, vb, vc), Te, duty
