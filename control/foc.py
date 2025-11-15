from __future__ import annotations

import math


def foc_decouple(vd_pi: float, vq_pi: float, omega_e: float, Ld: float, Lq: float, id_: float, iq_: float, psi: float) -> tuple[float, float]:
    """Apply standard dq decoupling/feedforward terms.

    vd = vd_pi - ωe * Lq * iq
    vq = vq_pi + ωe * (Ld * id + ψ)
    """
    vd = vd_pi - omega_e * Lq * iq_
    vq = vq_pi + omega_e * (Ld * id_ + psi)
    return vd, vq


def voltage_limit(vd: float, vq: float, vbus: float) -> tuple[float, float, float]:
    """Limit voltage vector magnitude to Vbus/sqrt(3) and return (vd, vq, |v|)."""
    vlim = vbus / math.sqrt(3.0)
    mag = math.hypot(vd, vq)
    if mag > vlim and mag > 1e-12:
        s = vlim / mag
        vd *= s
        vq *= s
        mag = vlim
    return vd, vq, mag

