# 03 · Visual place recognition: absolute fixes that can be wrong

## 3.1 What VPR provides

Visual place recognition (VPR) matches downward or oblique imagery against an onboard georeferenced reference (orthophotos, satellite imagery, or a prior map) and returns an **absolute** position. Unlike odometry it does not drift. Unlike GNSS it does not depend on a radio signal an adversary can reach.

The engineering challenge is not the average case but the tails: VPR can be **confidently wrong**. Repetitive terrain (fields, forests, identical buildings), seasonal change, outdated maps, shadows and low altitude produce matches that score well and are hundreds of metres or kilometres off.

## 3.2 VPR proposes, the estimator disposes

A VPR match is a candidate, not a fact. The estimator decides whether to believe it by asking whether the fix is plausible given everything else it knows: where odometry says the vehicle has been, and how uncertain that is. A fix that implies the vehicle jumped 2 km sideways in ten seconds is rejected however good the image similarity score looked.

## 3.3 The fix model used in the simulation

| Parameter | Value | Meaning |
|---|---|---|
| `vpr_period_s` | 10 s | attempt interval |
| `vpr_availability` | 0.8 | fraction of attempts that return a fix (texture, light, map coverage) |
| `vpr_sigma_m` | 12 m | per-axis noise of a correct fix |
| `vpr_outlier_prob` | 0.10 | fraction of returned fixes that are wrong matches |
| `vpr_outlier_range_m` | 150 to 2,000 m | size of a wrong match, random direction |

Outliers are drawn independently. Real VPR errors are often correlated (the same misleading area produces several wrong matches in a row), which is harder; see limits below.

## 3.4 What the simulation shows

With seed 7 and the defaults, every injected wrong match is rejected by the gate, and gated fusion holds 2D RMSE near 6 m against about 54 m for VIO alone. Accepting every fix without gating is worse than not using VPR at all, because a single 1 km outlier pulls the estimate far off and the filter then takes several fixes to recover. Run `python -m gnssdenied.demo` to reproduce the table in the README.

## 3.5 Questions to ask of any VPR claim

1. Accuracy as a distribution (CEP50, CEP95, maximum), not a single number.
2. Under which conditions: altitude, speed, terrain type, season, light, map age and resolution.
3. The rate of wrong matches, and how large they are.
4. Whether the navigation system can reject a wrong match, and what happens after it rejects several in a row.

## Assumptions and limits

- Fixes are 2D position only, with a known noise level. Real systems output a match confidence that must be mapped to a covariance, which is hard to calibrate.
- Wrong matches are independent. Correlated outliers can be accepted if the filter has already been pulled by one of them.
- No altitude, attitude or camera geometry is modelled.
