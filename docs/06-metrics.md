# 06 · Metrics: RMSE, CEP and NEES

Accuracy is a distribution, and a filter's covariance is a claim that can be tested. Three metrics cover both.

## 6.1 RMSE

$$\text{RMSE} = \sqrt{\frac{1}{N}\sum_{k=1}^{N} \lVert \hat{p}_k - p_k \rVert^2}$$

Two-dimensional root mean square position error (sometimes called DRMS). Sensitive to large errors, which is useful: one capture event dominates it.

## 6.2 CEP50 and CEP95

The circular error probable is the radius of a circle, centred on the truth, that contains a given fraction of the errors.

For a zero-mean circular normal distribution with per-axis standard deviation $\sigma$:

$$\text{CEP}_{50} = \sqrt{2\ln 2}\,\sigma = 1.1774\,\sigma, \qquad R_{95} = \sqrt{-2\ln 0.05}\,\sigma = 2.4477\,\sigma$$

Navigation errors are rarely zero-mean or Gaussian: drift creates bias, and wrong fixes create heavy tails. The code therefore computes **empirical percentiles** of the 2D position error (`metrics.cep(errors, 0.5)` and `cep(errors, 0.95)`). The analytic factors are kept in `stats.cep_circular_normal` and a test checks that the empirical and analytic values agree on Gaussian data.

Always report CEP50 together with CEP95 and the maximum. A system with a good CEP50 and a terrible CEP95 is one whose failures are rare and large, which is exactly what matters operationally.

## 6.3 NEES: is the filter honest about its uncertainty?

$$\text{NEES}_k = e_k^\top P_k^{-1} e_k, \qquad e_k = \hat{p}_k - p_k$$

If the filter is consistent, NEES follows a chi-square distribution with $n$ degrees of freedom and its mean equals $n$ (here $n = 2$, 2D position). `metrics.nees_bounds(n, N)` gives a two-sided acceptance interval for the mean over $N$ independent samples.

| Mean NEES | Interpretation |
|---|---|
| near $n$ | the covariance describes the actual error |
| much greater than $n$ | over-confident: the filter believes it is more accurate than it is; its gate will reject correct fixes |
| much less than $n$ | conservative: safe but wastes information; its gate is wider than needed |

In the demo, the gated VIO + VPR filter has a mean NEES between 1 and 2: consistent, slightly conservative. The spoofed, gate-only filter has a mean NEES in the tens of thousands: it is confidently wrong, which is the definition of the spoofing threat. NEES requires ground truth, so it is a test and evaluation metric, not something a vehicle can compute in flight.

Consecutive samples along one trajectory are correlated, so the chi-square bounds are indicative when applied to a single run; averaging over independent Monte Carlo runs gives a proper test.

## Sources

Bar-Shalom, Li and Kirubarajan (2001) for NEES and consistency testing; Groves (2013) and Kaplan and Hegarty (2017) for accuracy conventions. See [SOURCES.md](SOURCES.md).
