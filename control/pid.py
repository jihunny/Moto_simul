from __future__ import annotations

from dataclasses import dataclass


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
        self.integ += self.ki * error * dt
        # Clamp integrator
        if self.integ > self.out_max:
            self.integ = self.out_max
        elif self.integ < self.out_min:
            self.integ = self.out_min
        u = self.kp * error + self.integ
        if u > self.out_max:
            return self.out_max
        if u < self.out_min:
            return self.out_min
        return u

