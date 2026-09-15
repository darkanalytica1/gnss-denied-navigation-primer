"""Extended Kalman filter with Mahalanobis innovation gating.

    predict:  x- = f(x, u)          P- = F P F^T + Q
    update:   y  = z - h(x-)        S  = H P- H^T + R
              d2 = y^T S^-1 y       accept only if d2 <= chi2_ppf(alpha, m)
              K  = P- H^T S^-1      x = x- + K y      P = (I - K H) P- (I - K H)^T + K R K^T

The Joseph form of the covariance update is used because it stays symmetric
and positive semi-definite under rounding, which matters when gating decisions
depend on P being honest.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from . import linalg as la
from .stats import chi2_ppf


@dataclass(frozen=True)
class UpdateResult:
    accepted: bool
    d2: float
    threshold: float
    innovation: tuple

    @property
    def gated(self) -> bool:
        return not self.accepted


class EKF:
    def __init__(self, x0: Sequence[float], P0: la.Matrix):
        self.x = [float(v) for v in x0]
        self.P = [list(map(float, row)) for row in P0]

    @property
    def n(self) -> int:
        return len(self.x)

    def predict(self, f: Callable[[la.Vector], la.Vector], F: la.Matrix, Q: la.Matrix) -> None:
        self.x = f(self.x)
        self.P = la.symmetrize(la.add(la.mul(la.mul(F, self.P), la.transpose(F)), Q))

    def innovation(self, z: Sequence[float], h: Callable[[la.Vector], la.Vector], H: la.Matrix, R: la.Matrix):
        y = la.vsub(z, h(self.x))
        S = la.add(la.mul(la.mul(H, self.P), la.transpose(H)), R)
        return y, S

    def update(
        self,
        z: Sequence[float],
        h: Callable[[la.Vector], la.Vector],
        H: la.Matrix,
        R: la.Matrix,
        gate_prob: Optional[float] = 0.99,
    ) -> UpdateResult:
        """Measurement update. gate_prob=None disables gating (accept everything)."""
        y, S = self.innovation(z, h, H, R)
        S_inv = la.inv(S)
        d2 = la.quad_form(y, S_inv)
        threshold = chi2_ppf(gate_prob, len(z)) if gate_prob is not None else float("inf")
        if d2 > threshold:
            return UpdateResult(False, d2, threshold, tuple(y))
        K = la.mul(la.mul(self.P, la.transpose(H)), S_inv)
        self.x = la.vadd(self.x, la.mulv(K, y))
        I_KH = la.sub(la.eye(self.n), la.mul(K, H))
        self.P = la.symmetrize(
            la.add(la.mul(la.mul(I_KH, self.P), la.transpose(I_KH)), la.mul(la.mul(K, R), la.transpose(K)))
        )
        return UpdateResult(True, d2, threshold, tuple(y))


def mahalanobis_d2(innovation: Sequence[float], S: la.Matrix) -> float:
    """Squared Mahalanobis distance of an innovation, d2 = y^T S^-1 y."""
    return la.quad_form(list(innovation), la.inv(S))
