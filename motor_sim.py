"""
PMSM dq‑FOC simulation based on Motor_simul.md notes.

Electrical (dq frame, rotor‑aligned):
  di_d/dt = (v_d + ω_e L_q i_q - R i_d) / L_d
  di_q/dt = (v_q - ω_e (L_d i_d + ψ) - R i_q) / L_q

Mechanical:
  dω_m/dt = (T_e - B ω_m - T_load) / J
  T_e = 1.5 p (ψ i_q + (L_d - L_q) i_d i_q)
  ω_e = p ω_m

Control:
- Speed PI → i_q,ref (torque producing)
- i_d PI and i_q PI with decoupling → v_d, v_q
- Voltage vector limited to Vbus/√3 (sine/SVPWM fundamental limit)

Usage:
  python motor_sim.py --mode speed --rpm 2000 --duration 1.5 --dt 0.0005
  python motor_sim.py --mode torque --torque 0.08
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import csv
from typing import Optional


@dataclass
class MotorParams:
    R: float = 0.5            # Ohm (phase equivalent)
    L: float = 0.0002         # H (legacy, used if Ld/Lq not set)
    Ld: float | None = None   # H (d-axis inductance)
    Lq: float | None = None   # H (q-axis inductance)
    Kt: float = 0.06          # N·m/A (used to derive ψ)
    p: int = 4                # pole pairs
    J: float = 2.0e-4         # kg·m^2 (inertia)
    B: float = 1.0e-4         # N·m·s/rad (viscous friction)
    Vbus: float = 24.0        # V (DC link)
    Imax: float = 10.0        # A (current limit)


@dataclass
class PI:
    kp: float
    ki: float
    out_min: float
    out_max: float
    integ: float = 0.0

    def reset(self) -> None:
        self.integ = 0.0

    def step(self, error: float, dt: float) -> float:
        # PI with simple anti-windup via clamping integrator
        self.integ += self.ki * error * dt
        # Pre-clamp integrator to avoid runaway
        if self.integ > self.out_max:
            self.integ = self.out_max
        elif self.integ < self.out_min:
            self.integ = self.out_min

        u = self.kp * error + self.integ
        # Output clamp
        if u > self.out_max:
            u = self.out_max
        elif u < self.out_min:
            u = self.out_min
        return u


@dataclass
class SimConfig:
    duration: float = 1.0     # s
    dt: float = 0.0005        # s
    # Mode: 'speed' (speed PI → Iq_ref) or 'torque' (Iq_ref from torque)
    mode: str = "speed"
    rpm_target: float = 2000  # rpm (used in speed mode)
    torque_ref: float = 0.05  # N·m (used in torque mode)
    t_load_step: float = 0.3  # s (when load torque is applied)
    t_load: float = 0.05      # N·m (load torque after step)
    csv_path: Optional[str] = None
    id_ref: float = 0.0       # A (d-axis current target)


def rpm_to_rad_s(rpm: float) -> float:
    return rpm * 2.0 * 3.141592653589793 / 60.0


def rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 60.0 / (2.0 * 3.141592653589793)


def simulate(params: MotorParams, cfg: SimConfig):
    # Effective inductances
    Ld = params.Ld if params.Ld is not None else params.L
    Lq = params.Lq if params.Lq is not None else params.L

    # Flux linkage (derived from Kt): Kt = 1.5 p ψ  => ψ = Kt / (1.5 p)
    psi = params.Kt / (1.5 * params.p)

    # Controllers (tuned conservatively)
    # Speed PI outputs iq_ref (A)
    spd_pi = PI(kp=0.01, ki=2.0, out_min=-params.Imax, out_max=params.Imax)
    # Current PIs output vd, vq (V)
    id_pi = PI(kp=0.8, ki=120.0, out_min=-params.Vbus, out_max=params.Vbus)
    iq_pi = PI(kp=0.8, ki=120.0, out_min=-params.Vbus, out_max=params.Vbus)

    t = 0.0
    n_steps = int(cfg.duration / cfg.dt)

    # States
    id_ = 0.0     # A
    iq_ = 0.0     # A
    omega = 0.0   # rad/s (mechanical)

    # Targets
    omega_ref = rpm_to_rad_s(cfg.rpm_target)

    # Logs
    log = []  # list of tuples

    for k in range(n_steps + 1):
        # Load torque profile
        t_load = cfg.t_load if t >= cfg.t_load_step else 0.0

        # Electrical speed
        omega_e = params.p * omega

        # Determine current references
        if cfg.mode == "torque":
            # Use id_ref as provided; compute iq_ref from torque (ignore reluctance part by default)
            # For SPMSM (Ld≈Lq), T ≈ 1.5 p ψ iq
            iq_ref = cfg.torque_ref / (1.5 * params.p * psi)
            # Clamp
            if iq_ref > params.Imax:
                iq_ref = params.Imax
            elif iq_ref < -params.Imax:
                iq_ref = -params.Imax
            spd_err = omega_ref - omega  # logged only
        else:
            spd_err = omega_ref - omega
            iq_ref = spd_pi.step(spd_err, cfg.dt)

        id_ref = cfg.id_ref

        # Current control with decoupling (standard convention)
        id_err = id_ref - id_
        iq_err = iq_ref - iq_

        vd_pi = id_pi.step(id_err, cfg.dt)
        vq_pi = iq_pi.step(iq_err, cfg.dt)

        # Decoupling/feedforward terms
        vd = vd_pi - omega_e * Lq * iq_
        vq = vq_pi + omega_e * (Ld * id_ + psi)

        # Voltage limit (sine/SVPWM fundamental)
        import math

        vlim = params.Vbus / math.sqrt(3.0)
        vmag = math.hypot(vd, vq)
        if vmag > vlim and vmag > 1e-9:
            scale = vlim / vmag
            vd *= scale
            vq *= scale

        # Electrical plant integration
        did_dt = (vd + omega_e * Lq * iq_ - params.R * id_) / Ld
        diq_dt = (vq - omega_e * (Ld * id_ + psi) - params.R * iq_) / Lq
        id_ += did_dt * cfg.dt
        iq_ += diq_dt * cfg.dt
        # Soft current clamp to keep numerics bounded
        lim = max(5.0 * params.Imax, params.Imax + 1.0)
        if id_ > lim:
            id_ = lim
        elif id_ < -lim:
            id_ = -lim
        if iq_ > lim:
            iq_ = lim
        elif iq_ < -lim:
            iq_ = -lim

        # Electromagnetic torque
        Te = 1.5 * params.p * (psi * iq_ + (Ld - Lq) * id_ * iq_)

        # Mechanical dynamics
        domega_dt = (Te - params.B * omega - t_load) / params.J
        omega += domega_dt * cfg.dt

        # Log (preserve legacy columns: i→iq, v_cmd→|v|, i_err→iq_err)
        log.append(
            (
                t,
                omega,
                rad_s_to_rpm(omega),
                iq_,
                iq_ref,
                vmag,
                t_load,
                spd_err,
                iq_err,
                id_,
                id_ref,
                vd,
                vq,
                Te,
            )
        )

        t += cfg.dt

    return log


def write_csv(path: str, log) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "t_s",
            "omega_rad_s",
            "speed_rpm",
            "iq_a",
            "iq_ref_a",
            "v_mag_v",
            "t_load_nm",
            "speed_err",
            "iq_err",
            "id_a",
            "id_ref_a",
            "vd_v",
            "vq_v",
            "Te_nm",
        ])
        w.writerows(log)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PMSM dq‑FOC simulation")
    p.add_argument("--mode", choices=["speed", "torque"], default="speed", help="Control mode")
    p.add_argument("--rpm", type=float, default=2000.0, help="Target speed in rpm (speed mode)")
    p.add_argument("--torque", type=float, default=0.05, help="Reference torque in N·m (torque mode)")
    p.add_argument("--duration", type=float, default=1.5, help="Simulation time [s]")
    p.add_argument("--dt", type=float, default=0.0005, help="Time step [s]")
    p.add_argument("--t_load_step", type=float, default=0.3, help="Load step time [s]")
    p.add_argument("--t_load", type=float, default=0.05, help="Load torque after step [N·m]")
    p.add_argument("--id_ref", type=float, default=0.0, help="d-axis current reference [A]")
    p.add_argument("--csv", type=str, default=None, help="Optional CSV output path")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = SimConfig(
        duration=args.duration,
        dt=args.dt,
        mode=args.mode,
        rpm_target=args.rpm,
        torque_ref=args.torque,
        t_load_step=args.t_load_step,
        t_load=args.t_load,
        csv_path=args.csv,
        id_ref=args.id_ref,
    )

    params = MotorParams()
    log = simulate(params, cfg)

    # Summary
    t_s, omega, speed_rpm, iq, iq_ref, v_mag, t_load, spd_err, iq_err, id_, id_ref, vd, vq, Te = log[-1]
    mode_desc = (
        f"mode=speed, target={args.rpm:.0f} rpm" if args.mode == "speed" else f"mode=torque, target={args.torque:.3f} N·m"
    )
    print(
        f"Final: {mode_desc}; speed={speed_rpm:.1f} rpm, iq={iq:.2f} A, |v|={v_mag:.2f} V, load={t_load:.3f} N·m, Te={Te:.3f} N·m"
    )

    if cfg.csv_path:
        write_csv(cfg.csv_path, log)
        print(f"Saved CSV: {cfg.csv_path}")


if __name__ == "__main__":
    main()
