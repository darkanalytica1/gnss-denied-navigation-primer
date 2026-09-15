"""Accuracy and consistency metrics for a navigation run."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from . import linalg as la
from .stats import chi2_ppf, percentile


def position_errors(est: Sequence[Sequence[float]], truth: Sequence[Sequence[float]]) -> list:
    return [math.hypot(e[0] - t[0], e[1] - t[1]) for e, t in zip(est, truth)]


def rmse(errors: Sequence[float]) -> float:
    """root mean square of 2D position errors (2D RMS, sometimes called DRMS)."""
    return math.sqrt(sum(e * e for e in errors) / len(errors))


def cep(errors: Sequence[float], q: float = 0.5) -> float:
    """Empirical circular error: radius containing fraction q of the errors.

    Empirical percentiles are used because real navigation errors are biased
    and heavy-tailed, so the circular-normal factors (1.1774 sigma, 2.4477 sigma)
    only apply to zero-mean Gaussian errors.
    """
    return percentile(list(errors), q)


def nees(error: Sequence[float], P: la.Matrix) -> float:
    """Normalised estimation error squared, e^T P^-1 e. Its mean should equal the dimension."""
    return la.quad_form(list(error), la.inv(P))


def nees_bounds(dim: int, n_samples: int, prob: float = 0.95) -> tuple:
    """Two-sided acceptance interval for the mean NEES of n independent samples."""
    lo = chi2_ppf((1 - prob) / 2, dim * n_samples) / n_samples
    hi = chi2_ppf(1 - (1 - prob) / 2, dim * n_samples) / n_samples
    return lo, hi


@dataclass(frozen=True)
class Summary:
    name: str
    rmse_m: float
    cep50_m: float
    cep95_m: float
    max_m: float
    final_m: float
    mean_nees: float
    fixes_accepted: int
    fixes_rejected: int

    @staticmethod
    def header() -> str:
        return f"{'scenario':<22}{'RMSE':>9}{'CEP50':>9}{'CEP95':>9}{'max':>9}{'final':>9}{'NEES':>10}{'acc':>6}{'rej':>6}"

    def row(self) -> str:
        return (
            f"{self.name:<22}{self.rmse_m:9.1f}{self.cep50_m:9.1f}{self.cep95_m:9.1f}"
            f"{self.max_m:9.1f}{self.final_m:9.1f}{self.mean_nees:10.1f}{self.fixes_accepted:6d}{self.fixes_rejected:6d}"
        )
