# 05 · Spoofing carry-off and consistency monitoring

## 5.1 The scenario

The vehicle flies with VIO, VPR every 10 s and GNSS at 1 Hz. At $t = 300$ s a simulated carry-off begins: the GNSS position is displaced along a fixed bearing, starting with a gentle acceleration (0.003 m/s²) that is capped at 1 m/s of drag. There is no jump. Each individual GNSS fix is only slightly different from what the filter expects.

The simulation models the **effect** on the position solution, not the radio signal. It contains nothing about how a spoofer is built.

## 5.2 Why the gate alone is captured

GNSS arrives every second with a 3 m noise level, so the filter trusts it strongly. When the reported position starts to slide, the innovation at each step is small, the fix passes the gate, and the filter moves a little. The filter has a place to put a steady disagreement between GNSS and VIO: the **VIO bias** state. So it learns a false bias, and its innovations stay small.

Then a second effect appears. The honest witness, VPR, starts to disagree with the filter by more than its gate allows, and **correct VPR fixes are rejected**. The estimator now defends the lie against the truth. With the default seed the gate-only filter ends the run about 430 m from the true position while its GNSS innovations look healthy.

## 5.3 Two monitors that do not use the filter's own innovations

`ConsistencyMonitor` in `gnssdenied/sim.py` checks GNSS against evidence that the spoofer does not control:

**Displacement test.** Over a sliding window of $W = 60$ s, GNSS displacement must match raw VIO displacement:

$$r = (z_k - z_{k-W}) - (p^{VIO}_k - p^{VIO}_{k-W}), \qquad \frac{\lVert r \rVert^2}{2\sigma_{GNSS}^2 + (b_{max} W)^2} \le \chi^2_{2,0.999}$$

where $b_{max}$ is the largest VIO bias considered plausible (0.12 m/s). Raw VIO is used, not the filter's bias-corrected estimate, precisely because the filter's bias has been corrupted. This test catches jumps and faster drags, and is limited by the VIO bias bound: a drag slower than the plausible VIO bias is indistinguishable from VIO error over one window.

**Independent witness test.** Each VPR fix is compared with the latest GNSS fix:

$$\frac{\lVert z^{VPR} - z^{GNSS} \rVert^2}{\sigma_{VPR}^2 + \sigma_{GNSS}^2} > \chi^2_{2,0.999}$$

GNSS is flagged only when the last three comparisons all disagree **and their disagreement vectors agree with each other**. A wrong VPR match also disagrees with GNSS, but in a random direction; a spoofed track produces a persistent, coherent offset. The coherence requirement is what keeps clustered VPR outliers from causing false alarms.

## 5.4 What happens on detection

1. GNSS is excluded from the filter.
2. The position and bias covariances are inflated, because the filter has absorbed the drag into both.
3. **Re-acquisition.** An inflated covariance means a wide gate, and a wide gate can accept a wrong VPR match and lock onto it. So the filter re-anchors only on a VPR fix that moves consistently, via VIO, with the previous VPR fix. After one such fix is accepted, normal gating resumes.

Step 3 was added after an earlier version of the simulation diverged on one seed: detection worked, but the very next VPR fix was an outlier, the wide gate accepted it, and the estimate never recovered. That failure is the covariance-honesty problem from [04](04-ekf-and-gating.md) in its most practical form.

## 5.5 Results with the default configuration

Reproduce with `python -m gnssdenied.demo`. The tests assert the qualitative outcomes across seeds: the gate-only filter is captured, the monitor flags GNSS after onset and never before it, and the monitored filter's error after onset is a small fraction of the captured filter's.

## Assumptions and limits

- **Detection delay is real.** A slow drag is flagged only once its offset exceeds what the VPR and GNSS noise levels explain, which takes minutes at these settings; the estimate is already tens of metres off by then. Faster detection needs better independent sensors, not a tighter threshold.
- A spoofer that also degrades VPR (for example by forcing flight over featureless terrain) or that coincides with VIO failure defeats both monitors.
- The monitors assume the VPR reference map is correct and unbiased.
- The carry-off profile is one example. Other drag profiles change detection time; the thresholds here are not tuned for any operational system.
