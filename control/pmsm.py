from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple
import math

from .pid import PI
from .foc import foc_decouple, voltage_limit
from .svpwm import svpwm_duty


@dataclass
class MotorParams:
    R: float = 0.5
    Ld: float = 0.0002
    Lq: float = 0.0002
    Kt: float = 0.06        # N·m/A
    p: int = 4              # pole pairs
    J: float = 2.0e-4
    B: float = 1.0e-4
    Vbus: float = 24.0
    Imax: float = 10.0

    @property
    def psi(self) -> float:
        # Kt = 1.5 * p * psi
        return self.Kt / (1.5 * self.p)


@dataclass
class Gains:
    spd_kp: float = 0.01
    spd_ki: float = 2.0
    id_kp: float = 0.8
    id_ki: float = 120.0
    iq_kp: float = 0.8
    iq_ki: float = 120.0


class PMSMSim:
    def __init__(self, params: MotorParams, gains: Gains):
        self.p = params
        self.g = gains

        # States
        self.id = 0.0
        self.iq = 0.0
        self.omega = 0.0  # mechanical rad/s
        self.theta_e = 0.0  # electrical angle [rad]

        # Controllers
        self.pi_spd = PI(self.g.spd_kp, self.g.spd_ki, -self.p.Imax, self.p.Imax)
        self.pi_id = PI(self.g.id_kp, self.g.id_ki, -self.p.Vbus, self.p.Vbus)
        self.pi_iq = PI(self.g.iq_kp, self.g.iq_ki, -self.p.Vbus, self.p.Vbus)

        # References
        self.id_ref = 0.0
        self.iq_ref = 0.0

    def reset(self):
        self.id = 0.0
        self.iq = 0.0
        self.omega = 0.0
        self.theta_e = 0.0
        self.pi_spd.reset(); self.pi_id.reset(); self.pi_iq.reset()

    def step(self, dt: float, mode: str, rpm_target: float, torque_ref: float, t_load: float) -> Tuple[float, float, float, float, float, float, float]:
        # Electrical speed and angle
        omega_e = self.p.p * self.omega
        self.theta_e = (self.theta_e + omega_e * dt) % (2.0 * math.pi)

        # Compute current refs
        if mode == "torque":
            # iq_ref from torque (ignore reluctance torque for SPMSM)
            self.iq_ref = torque_ref / (1.5 * self.p.p * self.p.psi)
            # clamp
            self.iq_ref = max(min(self.iq_ref, self.p.Imax), -self.p.Imax)
        else:  # speed
            omega_ref = rpm_target * 2.0 * math.pi / 60.0
            spd_err = omega_ref - self.omega
            self.iq_ref = self.pi_spd.step(spd_err, dt)

        # Current errors
        id_err = self.id_ref - self.id
        iq_err = self.iq_ref - self.iq

        # PI voltages
        vd_pi = self.pi_id.step(id_err, dt)
        vq_pi = self.pi_iq.step(iq_err, dt)

        # Decoupling
        vd, vq = foc_decouple(vd_pi, vq_pi, omega_e, self.p.Ld, self.p.Lq, self.id, self.iq, self.p.psi)

        # Voltage limit
        vd, vq, vmag = voltage_limit(vd, vq, self.p.Vbus)

        # Electrical dynamics
        did_dt = (vd + omega_e * self.p.Lq * self.iq - self.p.R * self.id) / self.p.Ld
        diq_dt = (vq - omega_e * (self.p.Ld * self.id + self.p.psi) - self.p.R * self.iq) / self.p.Lq
        self.id += did_dt * dt
        self.iq += diq_dt * dt

        # Torque and mechanical dynamics
        Te = 1.5 * self.p.p * (self.p.psi * self.iq + (self.p.Ld - self.p.Lq) * self.id * self.iq)
        domega_dt = (Te - self.p.B * self.omega - t_load) / self.p.J
        self.omega += domega_dt * dt

        # Diagnostics for UI (alpha-beta, phases, PWM, errors)
        cos_t = math.cos(self.theta_e); sin_t = math.sin(self.theta_e)
        # dq -> alpha-beta
        v_alpha = cos_t * vd - sin_t * vq
        v_beta = sin_t * vd + cos_t * vq
        i_alpha = cos_t * self.id - sin_t * self.iq
        i_beta = sin_t * self.id + cos_t * self.iq
        # Back-EMF for SPMSM
        e_alpha = -self.p.psi * omega_e * sin_t
        e_beta = self.p.psi * omega_e * cos_t
        # Phase reconstruction (inverse Clarke)
        sqrt3_2 = math.sqrt(3) / 2.0
        i_a = i_alpha
        i_b = -0.5 * i_alpha + sqrt3_2 * i_beta
        i_c = -0.5 * i_alpha - sqrt3_2 * i_beta
        v_a = v_alpha
        v_b = -0.5 * v_alpha + sqrt3_2 * v_beta
        v_c = -0.5 * v_alpha - sqrt3_2 * v_beta
        # Duty via SVPWM (0..1) and sector
        d_a, d_b, d_c, sector = svpwm_duty(v_alpha, v_beta, self.p.Vbus)
        duty_a, duty_b, duty_c = d_a, d_b, d_c
        pwm_ratio = min(1.0, vmag / (self.p.Vbus / math.sqrt(3.0)))

        self.extra = {
            "theta_e": self.theta_e, "omega_e": omega_e,
            "v_alpha": v_alpha, "v_beta": v_beta,
            "i_alpha": i_alpha, "i_beta": i_beta,
            "e_alpha": e_alpha, "e_beta": e_beta,
            "i_a": i_a, "i_b": i_b, "i_c": i_c,
            "v_a": v_a, "v_b": v_b, "v_c": v_c,
            "duty_a": duty_a, "duty_b": duty_b, "duty_c": duty_c,
            "pwm": pwm_ratio, "id_err": id_err, "iq_err": iq_err, "sector": sector,
        }

        return self.omega, self.id, self.iq, vd, vq, Te, vmag
