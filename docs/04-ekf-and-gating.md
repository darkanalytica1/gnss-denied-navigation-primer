# 04 · The extended Kalman filter and Mahalanobis innovation gating

![Fusion loop](../assets/fusion-loop.svg)

## 4.1 The equations

Predict, driven by the relative source $u_k$:

$$\hat{x}_k^- = f(\hat{x}_{k-1}, u_k), \qquad P_k^- = F_k P_{k-1} F_k^\top + Q_k$$

Innovation, when an absolute fix $z_k$ arrives:

$$y_k = z_k - h(\hat{x}_k^-), \qquad S_k = H_k P_k^- H_k^\top + R_k$$

Gate:

$$d_k^2 = y_k^\top S_k^{-1} y_k \;\le\; \chi^2_{m,\alpha}$$

Update, only if the gate passes:

$$K_k = P_k^- H_k^\top S_k^{-1}, \qquad \hat{x}_k = \hat{x}_k^- + K_k y_k, \qquad P_k = (I - K_k H_k) P_k^- (I - K_k H_k)^\top + K_k R_k K_k^\top$$

| Symbol | Meaning |
|---|---|
| $\hat{x}$, $P$ | state estimate and its covariance |
| $f$, $F$ | motion model and its Jacobian |
| $Q$ | process noise: how much the prediction is allowed to be wrong per step |
| $z$, $h$, $H$ | measurement, measurement model and its Jacobian |
| $R$ | measurement noise covariance |
| $y$, $S$ | innovation and its covariance |
| $K$ | Kalman gain |
| $d^2$ | squared Mahalanobis distance of the innovation |
| $\chi^2_{m,\alpha}$ | chi-square quantile for $m$ measurement dimensions at confidence $\alpha$ |

The covariance update uses the Joseph form, which stays symmetric and positive semi-definite under rounding. That matters here because gating decisions depend on $P$ being honest.

In this repository's model $f$ and $h$ are linear, so the EKF reduces to a Kalman filter. The code keeps the general structure (function plus Jacobian) so a non-linear model can be dropped in.

## 4.2 Why Mahalanobis, not Euclidean

A 30 m innovation means different things depending on how uncertain the filter is. Dividing by $S$ measures the innovation in standard deviations along each direction, accounting for correlation. Under the hypothesis that the fix is correct and the filter is consistent, $d^2$ follows a chi-square distribution with $m$ degrees of freedom, so the gate has a known false-rejection rate: $1 - \alpha$.

For a 2D fix, $\chi^2_{2,\alpha} = -2\ln(1-\alpha)$, so the 99 % gate is **9.21** and the 95 % gate is 5.99.

## 4.3 Worked example

A 2D fix with $S = \text{diag}(15^2, 15^2)$ m², 99 % gate:

| Innovation | $d^2$ | Decision |
|---|---|---|
| [10, −15] m | $(100 + 225)/225 = 1.44$ | accepted |
| [30, 40] m | $2500/225 = 11.1$ | rejected |
| [2000, 0] m | $\approx 17{,}800$ | rejected, however good the image match looked |

The gate radius in this case is $\sqrt{9.21} \times 15 = 45.5$ m.

```python
from gnssdenied import mahalanobis_d2, chi2_ppf
from gnssdenied import linalg as la
S = la.diag([15**2, 15**2])
mahalanobis_d2([30, 40], S), chi2_ppf(0.99, 2)    # (11.11, 9.21)
```

## 4.4 Failure modes: the gate is only as honest as the covariance

| Situation | Effect |
|---|---|
| $P$ inflated (large $Q$, long gap with no fixes) | Gate widens; a wrong fix or a spoofed position can pass |
| $P$ over-confident (small $Q$, optimistic $R$) | Gate narrows; correct fixes are rejected, the filter ignores reality and diverges |
| Slow, consistent drag on the absolute source | Innovations stay small at every step because the filter follows; the gate never fires ([05](05-spoofing-carry-off.md)) |
| Many consecutive rejections | Either the source is bad or the filter is; the gate alone cannot tell which |

The last row is why real systems pair gating with monitors that do not depend on the filter's own state, and with explicit logic for re-acquisition after a period of rejections.

## 4.5 In the code

- `gnssdenied/ekf.py`: `EKF.predict`, `EKF.update(..., gate_prob=0.99)` returning an `UpdateResult` with `accepted`, `d2` and `threshold`.
- `gnssdenied/sim.py`: the navigation model with state $[p_x, p_y, b_x, b_y]$; `predict` is the closed-form expansion of $F P F^\top + Q$ for this model, checked against the generic EKF by a test.
- `gnssdenied/stats.py`: `chi2_ppf` and `chi2_cdf` without SciPy.

## Assumptions and limits

- Gaussian noise with known covariances. Real measurement noise is heavy-tailed and its covariance is estimated, not known.
- A fixed gate probability. Some systems adapt it, or use a sequential probability ratio test over several fixes.
- Forward filtering only. Factor-graph smoothers re-optimise past states and can recover from a late-detected fault more gracefully.
