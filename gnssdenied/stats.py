"""Chi-square distribution and circular error helpers, standard library only."""
from __future__ import annotations

import math
from typing import Sequence


def _gammainc_lower_regularized(a: float, x: float) -> float:
    """P(a, x) = gamma(a, x) / Gamma(a), series or continued fraction (Numerical Recipes 6.2)."""
    if x <= 0:
        return 0.0
    gln = math.lgamma(a)
    if x < a + 1:
        term = 1.0 / a
        total = term
        ap = a
        for _ in range(500):
            ap += 1
            term *= x / ap
            total += term
            if abs(term) < abs(total) * 1e-14:
                break
        return total * math.exp(-x + a * math.log(x) - gln)
    b = x + 1 - a
    c = 1.0 / 1e-300
    d = 1.0 / b
    h = d
    for i in range(1, 500):
        an = -i * (i - a)
        b += 2
        d = an * d + b
        d = 1e-300 if abs(d) < 1e-300 else d
        c = b + an / c
        c = 1e-300 if abs(c) < 1e-300 else c
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1) < 1e-14:
            break
    return 1.0 - math.exp(-x + a * math.log(x) - gln) * h


def chi2_cdf(x: float, dof: int) -> float:
    return _gammainc_lower_regularized(dof / 2.0, x / 2.0)


def chi2_ppf(p: float, dof: int) -> float:
    """Inverse chi-square CDF by bisection. chi2_ppf(0.99, 2) = 9.21."""
    if not 0 < p < 1:
        raise ValueError("p must be in (0, 1)")
    if dof == 2:
        return -2.0 * math.log(1.0 - p)
    lo, hi = 0.0, max(10.0, 10.0 * dof)
    while chi2_cdf(hi, dof) < p:
        hi *= 2
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if chi2_cdf(mid, dof) < p:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


CEP50_FACTOR = math.sqrt(2 * math.log(2))     # 1.1774
R95_FACTOR = math.sqrt(-2 * math.log(0.05))   # 2.4477


def cep_circular_normal(sigma: float, q: float = 0.5) -> float:
    """Radius containing fraction q of a zero-mean circular normal with per-axis sigma.

    q = 0.5 gives CEP = 1.1774 sigma; q = 0.95 gives R95 = 2.4477 sigma.
    """
    return sigma * math.sqrt(-2 * math.log(1 - q))


def percentile(values: Sequence[float], q: float) -> float:
    """Linear-interpolated percentile, q in [0, 1]."""
    if not values:
        raise ValueError("empty sequence")
    s = sorted(values)
    pos = q * (len(s) - 1)
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (pos - lo)
