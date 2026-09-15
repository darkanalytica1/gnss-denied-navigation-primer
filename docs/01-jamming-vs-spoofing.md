# 01 · Jamming versus spoofing

`ASSESSMENT` A jammed receiver knows it is lost. A spoofed receiver is confident and wrong. The two attacks need different engineering answers, and a requirement that says only "GNSS-denied" leaves the more dangerous one unspecified.

## 1.1 Why GNSS is easy to deny

The minimum received GPS L1 C/A power at the Earth's surface is −158.5 dBW, or **−128.5 dBm** (IS-GPS-200). Thermal noise in the signal's 2.046 MHz bandwidth is

$$N = -174 + 10\log_{10}(2.046 \times 10^6) = -110.9\ \text{dBm}$$

so the signal sits about 18 dB below noise. The receiver recovers it by correlating against the known spreading code, a processing gain of roughly 46 dB relative to the 50 bit/s navigation data. A jammer only needs to add enough in-band power to push the post-correlation carrier-to-noise density $C/N_0$ below what the tracking loops can hold.

A 1 W jammer with an omnidirectional antenna, 10 km from a receiver, delivers about −86 dBm at L1: a J/S of about 42 dB. The arithmetic is worked in the companion repository [rf-link-budget-toolkit](https://github.com/darkanalytica1/rf-link-budget-toolkit/blob/main/docs/04-gnss-jamming.md).

## 1.2 Jamming: an availability failure

Jamming raises the noise floor until the receiver cannot acquire or track satellites. The receiver reports no fix, or a degraded one with collapsing $C/N_0$. A well-designed flight controller detects this and changes mode: hold, return on dead reckoning, land, or continue on a GNSS-independent source.

This is the **honest failure**. It is disruptive, but the system knows it happened.

## 1.3 Spoofing: an integrity failure

Spoofing transmits counterfeit GNSS signals so the receiver computes a false position, velocity and time while still reporting a valid, healthy fix.

| Class | Mechanism | Typical tell |
|---|---|---|
| Simplistic | Transmit a synthetic constellation at higher power | Sudden jump in position or time, power step |
| Intermediate (carry-off) | Match code phase and Doppler of the real signals, overpower slightly so the tracking loops lock onto the counterfeit, then drag the solution away slowly | Small power rise at capture; afterwards a plausible, gradual drift |
| Sophisticated | Several synchronised transmitters that also fake arrival directions | Very few; needs independent sensors |
| Meaconing | Record and rebroadcast genuine signals with a delay | Delay, wrong geometry; defeats message authentication because content is genuine |

This is the **dangerous failure**. Nothing downstream is alerted, so the aircraft flies confidently to the wrong place.

## 1.4 Detecting spoofing: layers of physical consistency

No single test is sufficient. Each forces the spoofer to be consistent in one more dimension.

| Detector | Tests | Blind spot |
|---|---|---|
| RAIM (receiver autonomous integrity monitoring) | Pseudorange residuals are mutually consistent | Assumes few independent faults; misses a consistent fake constellation |
| Power, AGC and $C/N_0$ monitoring | Absolute power, gain control, suspiciously uniform $C/N_0$ | A careful low-power carry-off |
| Clock monitoring | Steps in clock bias or drift | A spoofer that models the clock |
| Angle of arrival (multi-antenna) | Signals arrive from ephemeris directions | Multi-transmitter spoofers |
| Signal quality monitoring | Correlation peak distortion during capture | Only works during the capture phase |
| Navigation message authentication (for example Galileo OSNMA) | Navigation data is cryptographically signed | Meaconing (content is genuine) |
| **Independent navigation cross-check** | GNSS agrees with inertial and visual motion and with absolute visual fixes | Needs sensors whose errors are independent of GNSS |

The last row is the one this repository simulates. It is also the only layer that remains available to a small platform with one antenna and a civil receiver.

## 1.5 The design principle

GNSS should stop being the only witness. Navigation is framed as several sources with **independent failure modes** and an estimator that decides which to believe:

- **Relative** sources (inertial, visual-inertial odometry) are smooth and cannot be spoofed by radio, but drift without bound.
- **Absolute** sources (GNSS, visual place recognition against a map) do not drift, but can be unavailable or wrong.
- The **estimator** fuses them and rejects what is statistically inconsistent.

Claims should be stated as behaviour under stated conditions, never as immunity.

## Assumptions and limits

- The table of detectors is indicative, after Psiaki and Humphreys (2016); effectiveness depends on receiver design and attacker care.
- Signal power figures are specification minimums; real received power is usually a few dB higher.
- Transmitting on GNSS frequencies without authorisation is illegal in most jurisdictions. Nothing here describes how to build or operate a jammer or spoofer.
