"""A 2D navigation simulation: truth, drifting VIO, absolute fixes and spoofing.

Everything here is synthetic and seeded. The numbers are chosen to make the
mechanisms visible, not to represent any particular sensor or product.

State of the navigation filter: [px, py, bx, by], position (m) and the VIO
velocity bias (m/s). VIO reports velocity = true velocity + bias + noise, so
dead reckoning drifts without bound; absolute fixes make the bias observable.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field, replace
from typing import List, Optional

from . import linalg as la
from .ekf import EKF
from .metrics import Summary, cep, position_errors, nees, rmse


# --------------------------------------------------------------------------- config

@dataclass(frozen=True)
class Config:
    duration_s: float = 900.0
    dt_s: float = 0.2
    speed_m_s: float = 15.0
    # VIO error model
    vio_noise_m_s: float = 0.10
    vio_bias0_m_s: tuple = (0.06, -0.04)
    vio_bias_walk: float = 0.004          # m/s per sqrt(s)
    # Visual place recognition (VPR) absolute fixes
    vpr_period_s: float = 10.0
    vpr_sigma_m: float = 12.0
    vpr_availability: float = 0.8         # texture, light, map coverage
    vpr_outlier_prob: float = 0.10        # confidently wrong matches
    vpr_outlier_range_m: tuple = (150.0, 2000.0)
    # GNSS fixes (used in the spoofing scenario)
    gnss_period_s: float = 1.0
    gnss_sigma_m: float = 3.0
    # Spoofing carry-off
    spoof_start_s: float = 300.0
    spoof_accel_m_s2: float = 0.003
    spoof_max_rate_m_s: float = 1.0
    spoof_heading_deg: float = 60.0
    # Filter
    gate_prob: Optional[float] = 0.99
    # Consistency monitor
    monitor_window_s: float = 60.0
    monitor_bias_bound_m_s: float = 0.12
    monitor_prob: float = 0.999
    monitor_votes: int = 3                 # consecutive coherent disagreements needed to flag GNSS
    monitor_inflate_m: float = 150.0       # position sigma added on detection
    monitor_inflate_bias_m_s: float = 1.0  # bias sigma added on detection (the drag was absorbed there)


# --------------------------------------------------------------------------- truth

def truth_trajectory(cfg: Config) -> List[tuple]:
    """Racetrack survey pattern: straight legs joined by constant-rate turns.

    Returns a list of (t, x, y, vx, vy).
    """
    legs = [(0.0, 120.0), (None, 20.0), (180.0, 120.0), (None, 20.0)]  # (heading deg, duration s)
    out = []
    x = y = 0.0
    heading = 0.0
    t = 0.0
    steps = int(round(cfg.duration_s / cfg.dt_s))
    schedule = []
    while sum(d for _, d in schedule) < cfg.duration_s + 1:
        schedule.extend(legs)
    seg, seg_t = 0, 0.0
    for _ in range(steps + 1):
        target, dur = schedule[seg]
        if target is None:
            prev_heading = schedule[seg - 1][0]
            heading = prev_heading + 180.0 * (seg_t / dur)
        else:
            heading = target
        rad = math.radians(heading)
        vx, vy = cfg.speed_m_s * math.cos(rad), cfg.speed_m_s * math.sin(rad)
        out.append((t, x, y, vx, vy))
        x += vx * cfg.dt_s
        y += vy * cfg.dt_s
        t += cfg.dt_s
        seg_t += cfg.dt_s
        if seg_t >= dur - 1e-9:
            seg, seg_t = seg + 1, 0.0
    return out


# --------------------------------------------------------------------------- sensors

@dataclass
class Measurements:
    vio: List[tuple] = field(default_factory=list)          # (vx, vy) per step
    vpr: dict = field(default_factory=dict)                  # step -> (x, y, is_outlier)
    gnss: dict = field(default_factory=dict)                 # step -> (x, y, is_spoofed)
    spoof_offset: dict = field(default_factory=dict)         # step -> (dx, dy)


def generate_measurements(cfg: Config, truth: List[tuple], rng: random.Random, with_gnss: bool = False) -> Measurements:
    m = Measurements()
    bx, by = cfg.vio_bias0_m_s
    walk = cfg.vio_bias_walk * math.sqrt(cfg.dt_s)
    vpr_every = int(round(cfg.vpr_period_s / cfg.dt_s))
    gnss_every = int(round(cfg.gnss_period_s / cfg.dt_s))
    h = math.radians(cfg.spoof_heading_deg)
    for k, (t, x, y, vx, vy) in enumerate(truth):
        bx += rng.gauss(0, walk)
        by += rng.gauss(0, walk)
        m.vio.append((vx + bx + rng.gauss(0, cfg.vio_noise_m_s), vy + by + rng.gauss(0, cfg.vio_noise_m_s)))
        if k > 0 and k % vpr_every == 0 and rng.random() < cfg.vpr_availability:
            if rng.random() < cfg.vpr_outlier_prob:
                r = rng.uniform(*cfg.vpr_outlier_range_m)
                a = rng.uniform(0, 2 * math.pi)
                m.vpr[k] = (x + r * math.cos(a), y + r * math.sin(a), True)
            else:
                m.vpr[k] = (x + rng.gauss(0, cfg.vpr_sigma_m), y + rng.gauss(0, cfg.vpr_sigma_m), False)
        if with_gnss and k > 0 and k % gnss_every == 0:
            ts = t - cfg.spoof_start_s
            if ts > 0:
                t_cap = cfg.spoof_max_rate_m_s / cfg.spoof_accel_m_s2
                if ts <= t_cap:
                    d = 0.5 * cfg.spoof_accel_m_s2 * ts * ts
                else:
                    d = 0.5 * cfg.spoof_accel_m_s2 * t_cap * t_cap + cfg.spoof_max_rate_m_s * (ts - t_cap)
                dx, dy = d * math.cos(h), d * math.sin(h)
            else:
                dx = dy = 0.0
            m.spoof_offset[k] = (dx, dy)
            m.gnss[k] = (x + dx + rng.gauss(0, cfg.gnss_sigma_m), y + dy + rng.gauss(0, cfg.gnss_sigma_m), ts > 0)
    return m


# --------------------------------------------------------------------------- filter model

def make_filter(cfg: Config) -> EKF:
    return EKF([0.0, 0.0, 0.0, 0.0], la.diag([4.0, 4.0, 0.1 ** 2, 0.1 ** 2]))


def predict(ekf: EKF, cfg: Config, v_meas: tuple) -> None:
    """Propagate [px, py, bx, by] with the VIO velocity as control input.

    x- = f(x, u) = [px + (vx - bx) dt, py + (vy - by) dt, bx, by]
    F  = [[I, -dt I], [0, I]],   Q = diag(qp, qp, qb, qb)

    With P = [[A, B], [B^T, C]] in 2x2 blocks, F P F^T + Q expands to
    A' = A - dt (B + B^T) + dt^2 C,  B' = B - dt C,  C' = C.
    This is exactly EKF.predict with that F (a test checks it), written out
    because it is about ten times faster in pure Python.
    """
    dt = cfg.dt_s
    x = ekf.x
    ekf.x = [x[0] + (v_meas[0] - x[2]) * dt, x[1] + (v_meas[1] - x[3]) * dt, x[2], x[3]]
    P = ekf.P
    qp = (cfg.vio_noise_m_s * dt) ** 2
    qb = cfg.vio_bias_walk ** 2 * dt
    n = [[0.0] * 4 for _ in range(4)]
    for i in range(2):
        for j in range(2):
            n[i][j] = P[i][j] - dt * (P[i][j + 2] + P[i + 2][j]) + dt * dt * P[i + 2][j + 2]
            n[i][j + 2] = P[i][j + 2] - dt * P[i + 2][j + 2]
            n[i + 2][j + 2] = P[i + 2][j + 2]
    for i in range(2):
        for j in range(2):
            n[j + 2][i] = n[i][j + 2]
    n[0][0] += qp
    n[1][1] += qp
    n[2][2] += qb
    n[3][3] += qb
    ekf.P = n


def transition_matrix(dt: float) -> la.Matrix:
    return [[1, 0, -dt, 0], [0, 1, 0, -dt], [0, 0, 1, 0], [0, 0, 0, 1]]


H_POS = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]


def h_pos(x):
    return [x[0], x[1]]


# --------------------------------------------------------------------------- consistency monitor

class ConsistencyMonitor:
    """Two checks on GNSS that do not rely on the filter's own innovations.

    A slow carry-off can be absorbed into the filter's bias estimate, so the
    innovation gate never fires. These checks compare GNSS with evidence the
    spoofer does not control:

    1. Displacement test. GNSS displacement over a sliding window must match raw
       VIO displacement, within fix noise plus the worst plausible VIO bias
       times the window length. Catches jumps and fast drags.
    2. Independent witness test. Each VPR fix is compared with the latest GNSS
       fix; if k of the last n comparisons disagree beyond a chi-square bound,
       GNSS is declared inconsistent. Catches slow drags once the offset exceeds
       what the two noise levels explain. Voting tolerates occasional VPR
       outliers.
    """

    def __init__(self, cfg: Config):
        from .stats import chi2_ppf

        self.cfg = cfg
        self.window_steps = int(round(cfg.monitor_window_s / cfg.dt_s))
        self.threshold = chi2_ppf(cfg.monitor_prob, 2)
        self.vio_pos = [(0.0, 0.0)]
        self.gnss: dict = {}
        self.last_gnss = None
        self.votes: list = []
        self.reason: Optional[str] = None

    def step_vio(self, v: tuple) -> None:
        px, py = self.vio_pos[-1]
        self.vio_pos.append((px + v[0] * self.cfg.dt_s, py + v[1] * self.cfg.dt_s))

    def check_gnss(self, k: int, z: tuple) -> bool:
        self.gnss[k] = z
        self.last_gnss = z
        k0 = k - self.window_steps
        if k0 not in self.gnss:
            return False
        dz = (z[0] - self.gnss[k0][0], z[1] - self.gnss[k0][1])
        dv = (self.vio_pos[k][0] - self.vio_pos[k0][0], self.vio_pos[k][1] - self.vio_pos[k0][1])
        sigma2 = 2 * self.cfg.gnss_sigma_m ** 2 + (self.cfg.monitor_bias_bound_m_s * self.cfg.monitor_window_s) ** 2
        d2 = ((dz[0] - dv[0]) ** 2 + (dz[1] - dv[1]) ** 2) / sigma2
        if d2 > self.threshold:
            self.reason = "displacement"
            return True
        return False

    def check_witness(self, z_vpr: tuple) -> bool:
        """Flag GNSS when the last n VPR fixes all disagree with it, and agree with each other.

        A single confidently wrong VPR match disagrees with GNSS too, but in a
        random direction and by a random amount. A spoofed GNSS track produces
        a disagreement that is persistent and coherent: the same offset, growing
        slowly. Requiring coherence stops clustered VPR outliers from causing
        false alarms.
        """
        if self.last_gnss is None:
            return False
        sigma2 = self.cfg.vpr_sigma_m ** 2 + self.cfg.gnss_sigma_m ** 2
        r = (z_vpr[0] - self.last_gnss[0], z_vpr[1] - self.last_gnss[1])
        inconsistent = (r[0] ** 2 + r[1] ** 2) / sigma2 > self.threshold
        n = self.cfg.monitor_votes
        self.votes = (self.votes + [r if inconsistent else None])[-n:]
        if len(self.votes) < n or any(v is None for v in self.votes):
            return False
        pair_sigma2 = 2 * self.cfg.vpr_sigma_m ** 2
        coherent = all(
            ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) / pair_sigma2 <= self.threshold
            for i, a in enumerate(self.votes) for b in self.votes[i + 1:]
        )
        if coherent:
            self.reason = "independent witness"
            return True
        return False

    def vpr_pair_consistent(self, k_prev: int, z_prev: tuple, k: int, z: tuple) -> bool:
        """Do two VPR fixes move the way VIO says the vehicle moved between them?"""
        dt = (k - k_prev) * self.cfg.dt_s
        dz = (z[0] - z_prev[0], z[1] - z_prev[1])
        dv = (self.vio_pos[k][0] - self.vio_pos[k_prev][0], self.vio_pos[k][1] - self.vio_pos[k_prev][1])
        sigma2 = 2 * self.cfg.vpr_sigma_m ** 2 + (self.cfg.monitor_bias_bound_m_s * dt) ** 2
        return ((dz[0] - dv[0]) ** 2 + (dz[1] - dv[1]) ** 2) / sigma2 <= self.threshold


# --------------------------------------------------------------------------- runner

@dataclass
class Run:
    name: str
    cfg: Config
    truth: List[tuple]
    est: List[tuple]
    cov: List[la.Matrix]
    vpr_log: List[tuple]        # (t, x, y, accepted, is_outlier, d2)
    gnss_log: List[tuple]       # (t, x, y, accepted, is_spoofed, d2)
    detection_time_s: Optional[float] = None
    detection_reason: Optional[str] = None

    def errors(self) -> List[float]:
        return position_errors(self.est, [(p[1], p[2]) for p in self.truth])

    def summary(self, start_s: float = 0.0) -> Summary:
        all_errs = self.errors()
        idx = [i for i, p in enumerate(self.truth) if p[0] >= start_s]
        errs = [all_errs[i] for i in idx]
        ne = [
            nees((self.est[i][0] - self.truth[i][1], self.est[i][1] - self.truth[i][2]),
                 [row[:2] for row in self.cov[i][:2]])
            for i in idx
        ]
        logs = [r for r in self.vpr_log + self.gnss_log if r[0] >= start_s]
        return Summary(
            name=self.name,
            rmse_m=rmse(errs),
            cep50_m=cep(errs, 0.5),
            cep95_m=cep(errs, 0.95),
            max_m=max(errs),
            final_m=errs[-1],
            mean_nees=sum(ne) / len(ne),
            fixes_accepted=sum(1 for r in logs if r[3]),
            fixes_rejected=sum(1 for r in logs if not r[3]),
        )


def run(
    name: str,
    cfg: Config,
    seed: int = 7,
    use_vpr: bool = True,
    use_gnss: bool = False,
    monitor: bool = False,
) -> Run:
    """Simulate one scenario. The same seed gives identical truth and sensor data across runs."""
    rng = random.Random(seed)
    truth = truth_trajectory(cfg)
    meas = generate_measurements(cfg, truth, rng, with_gnss=True)
    ekf = make_filter(cfg)
    mon = ConsistencyMonitor(cfg) if monitor else None
    gnss_trusted = True
    detection = None
    reacquire = False
    last_vpr = None
    est, cov, vpr_log, gnss_log = [], [], [], []
    R_vpr = la.diag([cfg.vpr_sigma_m ** 2] * 2)
    R_gnss = la.diag([cfg.gnss_sigma_m ** 2] * 2)

    def distrust(t):
        nonlocal gnss_trusted, detection, reacquire
        gnss_trusted = False
        reacquire = True
        detection = t
        ekf.P[0][0] += cfg.monitor_inflate_m ** 2
        ekf.P[1][1] += cfg.monitor_inflate_m ** 2
        ekf.P[2][2] += cfg.monitor_inflate_bias_m_s ** 2
        ekf.P[3][3] += cfg.monitor_inflate_bias_m_s ** 2

    for k, p in enumerate(truth):
        if k > 0:
            predict(ekf, cfg, meas.vio[k - 1])
            if mon:
                mon.step_vio(meas.vio[k - 1])
        if use_gnss and k in meas.gnss:
            gx, gy, spoofed = meas.gnss[k]
            if mon and gnss_trusted and mon.check_gnss(k, (gx, gy)):
                distrust(p[0])
            if gnss_trusted:
                res = ekf.update((gx, gy), h_pos, H_POS, R_gnss, cfg.gate_prob)
                gnss_log.append((p[0], gx, gy, res.accepted, spoofed, res.d2))
            else:
                gnss_log.append((p[0], gx, gy, False, spoofed, float("nan")))
        if use_vpr and k in meas.vpr:
            vx, vy, outlier = meas.vpr[k]
            if mon and use_gnss and gnss_trusted and mon.check_witness((vx, vy)):
                distrust(p[0])
            if reacquire:
                # With P inflated the gate is wide, so one wrong match could capture the
                # filter. Re-anchor only on a fix that agrees with the previous one via VIO.
                ok = last_vpr is not None and mon.vpr_pair_consistent(last_vpr[0], last_vpr[1], k, (vx, vy))
                last_vpr = (k, (vx, vy))
                if not ok:
                    vpr_log.append((p[0], vx, vy, False, outlier, float("nan")))
                    est.append((ekf.x[0], ekf.x[1]))
                    cov.append([row[:] for row in ekf.P])
                    continue
                reacquire = False
            res = ekf.update((vx, vy), h_pos, H_POS, R_vpr, cfg.gate_prob)
            vpr_log.append((p[0], vx, vy, res.accepted, outlier, res.d2))
            if mon and not reacquire:
                last_vpr = (k, (vx, vy))
        est.append((ekf.x[0], ekf.x[1]))
        cov.append([row[:] for row in ekf.P])
    return Run(name, cfg, truth, est, cov, vpr_log, gnss_log, detection, mon.reason if mon else None)


def standard_scenarios(cfg: Config = Config(), seed: int = 7) -> dict:
    """The five runs used by the demo, the docs and the tests."""
    return {
        "vio_only": run("VIO only", cfg, seed, use_vpr=False),
        "vpr_no_gate": run("VIO + VPR, no gate", replace(cfg, gate_prob=None), seed),
        "vpr_gated": run("VIO + VPR, gated", cfg, seed),
        "spoof_naive": run("Spoof, gate only", cfg, seed, use_gnss=True),
        "spoof_monitored": run("Spoof, monitored", cfg, seed, use_gnss=True, monitor=True),
    }
