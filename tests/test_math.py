import math
import random

import pytest

from gnssdenied import EKF, cep, cep_circular_normal, chi2_cdf, chi2_ppf, mahalanobis_d2, nees, nees_bounds, rmse
from gnssdenied import linalg as la


def test_chi2_ppf_known_values():
    assert chi2_ppf(0.99, 2) == pytest.approx(9.21, abs=0.005)
    assert chi2_ppf(0.95, 2) == pytest.approx(5.991, abs=0.005)
    assert chi2_ppf(0.99, 3) == pytest.approx(11.345, abs=0.005)
    assert chi2_ppf(0.95, 1) == pytest.approx(3.841, abs=0.005)


def test_chi2_cdf_inverts_ppf():
    for dof in (1, 2, 4, 10):
        for p in (0.1, 0.5, 0.9, 0.999):
            assert chi2_cdf(chi2_ppf(p, dof), dof) == pytest.approx(p, abs=1e-6)


def test_handbook_gate_example():
    S = la.diag([15.0 ** 2, 15.0 ** 2])
    assert mahalanobis_d2([10, -15], S) == pytest.approx(1.44, abs=0.005)
    assert mahalanobis_d2([30, 40], S) == pytest.approx(11.11, abs=0.01)
    assert mahalanobis_d2([2000, 0], S) == pytest.approx(17778, rel=1e-3)
    assert mahalanobis_d2([10, -15], S) <= chi2_ppf(0.99, 2) < mahalanobis_d2([30, 40], S)


def test_accelerometer_bias_drift():
    b = 1e-3 * 9.80665  # 1 mg
    assert 0.5 * b * 60 ** 2 == pytest.approx(17.7, abs=0.1)
    assert 0.5 * b * 300 ** 2 == pytest.approx(441, abs=1)


def test_inverse_and_quad_form():
    A = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 2.0]]
    I = la.mul(A, la.inv(A))
    for i in range(3):
        for j in range(3):
            assert I[i][j] == pytest.approx(1.0 if i == j else 0.0, abs=1e-12)
    with pytest.raises(ValueError):
        la.inv([[1.0, 2.0], [2.0, 4.0]])


def test_cep_factors():
    assert cep_circular_normal(1.0) == pytest.approx(1.1774, abs=1e-4)
    assert cep_circular_normal(1.0, 0.95) == pytest.approx(2.4477, abs=1e-4)


def test_empirical_cep_matches_theory_for_gaussian():
    rng = random.Random(3)
    sigma = 5.0
    errs = [math.hypot(rng.gauss(0, sigma), rng.gauss(0, sigma)) for _ in range(20000)]
    assert cep(errs, 0.5) == pytest.approx(1.1774 * sigma, rel=0.03)
    assert cep(errs, 0.95) == pytest.approx(2.4477 * sigma, rel=0.03)
    assert rmse(errs) == pytest.approx(math.sqrt(2) * sigma, rel=0.03)


def test_nees_mean_is_dimension_for_consistent_errors():
    rng = random.Random(5)
    P = [[4.0, 1.0], [1.0, 9.0]]
    # draw e ~ N(0, P) via Cholesky
    l11 = 2.0
    l21 = 0.5
    l22 = math.sqrt(9.0 - l21 ** 2)
    vals = []
    for _ in range(5000):
        a, b = rng.gauss(0, 1), rng.gauss(0, 1)
        vals.append(nees([l11 * a, l21 * a + l22 * b], P))
    mean = sum(vals) / len(vals)
    lo, hi = nees_bounds(2, len(vals))
    assert lo < mean < hi


def test_ekf_gate_rejects_and_accepts():
    ekf = EKF([0.0, 0.0], la.diag([200.0, 200.0]))
    H = la.eye(2)
    R = la.diag([25.0, 25.0])
    h = lambda x: x
    bad = ekf.update([30.0, 40.0], h, H, R)       # S = 225 I
    assert not bad.accepted and bad.d2 == pytest.approx(11.11, abs=0.01)
    assert ekf.x == [0.0, 0.0]
    good = ekf.update([10.0, -15.0], h, H, R)
    assert good.accepted
    assert ekf.x[0] == pytest.approx(10 * 200 / 225)
    assert ekf.P[0][0] == pytest.approx(200 * 25 / 225)


def test_ekf_without_gate_accepts_anything():
    ekf = EKF([0.0, 0.0], la.diag([1.0, 1.0]))
    r = ekf.update([1000.0, 0.0], lambda x: x, la.eye(2), la.diag([1.0, 1.0]), gate_prob=None)
    assert r.accepted
