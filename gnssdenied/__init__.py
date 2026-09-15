"""gnssdenied: a small, dependency-free primer on navigating without trusting GNSS."""
from .ekf import EKF, UpdateResult, mahalanobis_d2
from .metrics import Summary, cep, position_errors, nees, nees_bounds, rmse
from .sim import Config, ConsistencyMonitor, Run, run, standard_scenarios, truth_trajectory
from .stats import CEP50_FACTOR, R95_FACTOR, cep_circular_normal, chi2_cdf, chi2_ppf

__all__ = [
    "EKF", "UpdateResult", "mahalanobis_d2", "Summary", "cep", "position_errors", "nees", "nees_bounds",
    "rmse", "Config", "ConsistencyMonitor", "Run", "run", "standard_scenarios", "truth_trajectory",
    "CEP50_FACTOR", "R95_FACTOR", "cep_circular_normal", "chi2_cdf", "chi2_ppf",
]
__version__ = "0.1.0"
