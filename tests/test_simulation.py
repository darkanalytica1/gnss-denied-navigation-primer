import random
from dataclasses import replace

import pytest

from gnssdenied import Config, run, standard_scenarios
from gnssdenied import linalg as la
from gnssdenied.ekf import EKF
from gnssdenied.sim import predict, transition_matrix


@pytest.fixture(scope="module")
def sc():
    return standard_scenarios(Config(), seed=7)


def test_closed_form_predict_matches_generic_ekf():
    cfg = Config()
    dt = cfg.dt_s
    rng = random.Random(1)
    A = [[rng.gauss(0, 1) for _ in range(4)] for _ in range(4)]
    P = la.add(la.mul(A, la.transpose(A)), la.eye(4))
    a, b = EKF([1, 2, 0.1, 0.2], P), EKF([1, 2, 0.1, 0.2], P)
    v = (3.0, -1.0)
    predict(a, cfg, v)
    qp, qb = (cfg.vio_noise_m_s * dt) ** 2, cfg.vio_bias_walk ** 2 * dt
    b.predict(lambda x: [x[0] + (v[0] - x[2]) * dt, x[1] + (v[1] - x[3]) * dt, x[2], x[3]],
              transition_matrix(dt), la.diag([qp, qp, qb, qb]))
    assert a.x == b.x
    for ra, rb in zip(a.P, b.P):
        for x, y in zip(ra, rb):
            assert x == pytest.approx(y, abs=1e-12)


def test_deterministic_for_a_seed():
    cfg = replace(Config(), duration_s=120)
    r1, r2 = run("a", cfg, seed=3), run("b", cfg, seed=3)
    assert r1.est == r2.est


def test_vio_alone_drifts(sc):
    errs = sc["vio_only"].errors()
    assert errs[-1] > 30
    assert errs[-1] > 5 * errs[len(errs) // 20]


def test_gated_fusion_beats_vio_and_ungated(sc):
    gated = sc["vpr_gated"].summary()
    assert gated.rmse_m < 15
    assert gated.rmse_m < sc["vio_only"].summary().rmse_m / 4
    assert gated.rmse_m < sc["vpr_no_gate"].summary().rmse_m / 4


def test_gate_rejects_all_injected_outliers(sc):
    log = sc["vpr_gated"].vpr_log
    outliers = [x for x in log if x[4]]
    assert len(outliers) >= 3
    assert all(not x[3] for x in outliers)
    good_rejected = sum(1 for x in log if not x[4] and not x[3])
    assert good_rejected <= 0.05 * len(log)


def test_gated_filter_is_roughly_consistent(sc):
    assert 0.5 < sc["vpr_gated"].summary().mean_nees < 4.0


def test_gate_alone_is_captured_by_slow_carry_off(sc):
    naive = sc["spoof_naive"]
    spoofed = [x for x in naive.gnss_log if x[4]]
    accepted = sum(1 for x in spoofed if x[3])
    assert accepted > 0.8 * len(spoofed)          # the gate lets the drag through
    assert naive.summary(start_s=300).final_m > 200


def test_monitor_detects_after_onset_and_recovers(sc):
    m = sc["spoof_monitored"]
    assert m.detection_time_s is not None
    assert m.cfg.spoof_start_s < m.detection_time_s < m.cfg.spoof_start_s + 300
    assert m.summary().final_m < 30
    assert m.summary(start_s=300).rmse_m < sc["spoof_naive"].summary(start_s=300).rmse_m / 5


@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
def test_monitor_has_no_false_alarm_before_onset(seed):
    cfg = replace(Config(), duration_s=Config().spoof_start_s - 1)
    r = run("nominal", cfg, seed=seed, use_gnss=True, monitor=True)
    assert r.detection_time_s is None
