# 02 · Drift: inertial and visual-inertial odometry

Dead reckoning integrates motion. Integration turns small, constant errors into large, growing ones. That is the whole problem in one sentence.

## 2.1 Inertial drift from a bias

A constant accelerometer bias $b$ integrated twice gives a position error

$$e(t) = \tfrac{1}{2}\,b\,t^2$$

An uncorrected 1 mg bias ($9.8 \times 10^{-3}$ m/s²) gives about **18 m after 60 s** and **441 m after 300 s**, before gyro errors are considered. A gyro bias is worse: it tilts the estimated gravity vector, which leaks into level-plane acceleration and grows as $t^3$.

## 2.2 Visual-inertial odometry (VIO)

VIO tracks image features between frames and fuses them with inertial measurements to estimate the camera's motion. It slows drift dramatically compared with inertial alone because vision observes velocity directly. It is still **relative** navigation: it measures motion, not position, so small velocity errors integrate into position errors that grow without a ceiling.

Typical VIO failure conditions: low texture (water, snow, fresh asphalt, featureless fields), darkness without infrared, smoke and dust, motion blur, dynamic scenes, and aggressive manoeuvres that break feature tracking.

## 2.3 The error model used in the simulation

The simulation represents VIO as a velocity sensor with a slowly wandering bias:

$$v_{meas} = v_{true} + b + n, \qquad b_{k+1} = b_k + w_k$$

| Term | Value in `Config` | Meaning |
|---|---|---|
| $n$ | $\sigma = 0.10$ m/s | white velocity noise per sample |
| $b_0$ | (0.06, −0.04) m/s | initial bias |
| $w$ | 0.004 m/s per $\sqrt{\text{s}}$ | bias random walk |

Integrated over 900 s this gives tens to a hundred metres of drift, depending on the seed. The values are illustrative and chosen to make drift visible on a figure. They are not the performance of any real VIO system, which depends on camera, IMU, altitude, speed, scene and algorithm.

## 2.4 Why the filter estimates the bias

The navigation filter carries the VIO bias in its state, $x = [p_x, p_y, b_x, b_y]$. Between absolute fixes the bias is unobservable and the covariance grows. Each accepted absolute fix constrains position, and successive fixes constrain how position moved relative to what VIO reported, which is the bias. A well-estimated bias means the filter drifts much more slowly during the next gap.

This is also the mechanism a slow spoofer exploits: a gradual drag in GNSS position looks, to the filter, exactly like a VIO bias to be learned. See [04](04-ekf-and-gating.md) and [05](05-spoofing-carry-off.md).

## Assumptions and limits

- The model is 2D, with velocity-level VIO errors only. Scale drift, heading drift and attitude coupling are not modelled.
- Real VIO covariance is often over-confident; a filter that trusts it blindly will gate out correct fixes. The simulation uses the true noise parameters, which is the best case.
