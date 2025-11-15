from __future__ import annotations

import math
from typing import Tuple


def svpwm_duty(v_alpha: float, v_beta: float, vbus: float) -> Tuple[float, float, float, int]:
    """Compute SVPWM duty cycles (0..1) and sector from αβ voltages.

    This uses the classic two-active-vector + zero-vector approach scaled to the
    hexagon with Vbus/√3 as the linear modulation limit. Returns (d_a,d_b,d_c,sector).
    sector is 1..6 following 60° regions.
    """
    # Normalize to dimensionless reference using Vlim = Vbus/√3
    vlim = max(vbus / math.sqrt(3.0), 1e-9)
    x = v_alpha / vlim
    y = v_beta / vlim

    # Determine sector (1..6) based on space vector angle
    ang = math.atan2(y, x)  # -pi..pi
    if ang < 0:
        ang += 2.0 * math.pi
    sector = int(ang // (math.pi / 3.0)) + 1
    if sector > 6:
        sector = 6

    # Clarke transform inverse to 3-phase references (unnormalized)
    # Use αβ→abc mapping for centered modulation, then scale to 0..1
    v_a = x
    v_b = -0.5 * x + (math.sqrt(3.0) / 2.0) * y
    v_c = -0.5 * x - (math.sqrt(3.0) / 2.0) * y

    # Map to duty 0..1 using centered PWM (not full zero-sequence injection)
    # Add common-mode to center within 0..1
    v_min = min(v_a, v_b, v_c)
    v_max = max(v_a, v_b, v_c)
    v_cm = -0.5 * (v_max + v_min)
    d_a = max(0.0, min(1.0, 0.5 + 0.5 * (v_a + v_cm)))
    d_b = max(0.0, min(1.0, 0.5 + 0.5 * (v_b + v_cm)))
    d_c = max(0.0, min(1.0, 0.5 + 0.5 * (v_c + v_cm)))

    return d_a, d_b, d_c, sector

