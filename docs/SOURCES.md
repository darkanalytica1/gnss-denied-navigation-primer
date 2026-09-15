# Sources

Formulas and concepts come from public standards, textbooks and peer-reviewed papers. The simulation, its parameters, the consistency monitors and all numbers produced by the code are original and synthetic. No measured result from any system or organisation is used.

## Standards and specifications

1. IS-GPS-200, *Navstar GPS Space Segment / Navigation User Segment Interfaces*. L1 frequency and minimum received C/A power.
2. US Department of Defense, *GPS Standard Positioning Service Performance Standard*, 5th ed., 2020. Accuracy conventions.
3. European GNSS Open Service Signal-In-Space ICD, and EUSPA documentation for Galileo Open Service Navigation Message Authentication (OSNMA).

## Textbooks

4. P. D. Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed., Artech House, 2013. Inertial error growth, integration architectures, integrity monitoring.
5. E. D. Kaplan and C. J. Hegarty (eds.), *Understanding GPS/GNSS: Principles and Applications*, 3rd ed., Artech House, 2017. Signal structure, $C/N_0$, interference.
6. Y. Bar-Shalom, X. R. Li and T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001. Kalman filter, innovation gating, NEES and consistency tests.
7. S. Thrun, W. Burgard and D. Fox, *Probabilistic Robotics*, MIT Press, 2005. EKF and robust estimation.
8. R. G. Brown and P. Y. C. Hwang, *Introduction to Random Signals and Applied Kalman Filtering*, 4th ed., Wiley, 2012. Joseph-form covariance update, random-walk models.

## Papers

9. M. L. Psiaki and T. E. Humphreys, "GNSS Spoofing and Detection", *Proceedings of the IEEE*, 104(6), 2016. Spoofing classes, carry-off, detection layers.
10. T. E. Humphreys et al., "Assessing the Spoofing Threat: Development of a Portable GPS Civilian Spoofer", *Proc. ION GNSS*, 2008. The intermediate (carry-off) spoofer concept, cited at concept level only.
11. C. Forster, L. Carlone, F. Dellaert and D. Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", *IEEE Transactions on Robotics*, 33(1), 2017. VIO.
12. S. Lowry et al., "Visual Place Recognition: A Survey", *IEEE Transactions on Robotics*, 32(1), 2016. VPR and perceptual aliasing.
13. W. H. Press et al., *Numerical Recipes*, 3rd ed., Cambridge University Press, 2007, section 6.2. Incomplete gamma function used for the chi-square CDF.

## Confidence

| Claim type | Confidence | Why |
|---|---|---|
| Filter equations, chi-square gates, CEP and NEES definitions | High | Canonical textbooks; reproduced by unit tests |
| GNSS signal power and noise figures | High | Interface specification |
| Spoofing classes and detector blind spots | Medium to high | Peer-reviewed survey; effectiveness depends on implementation |
| Sensor parameters in `Config` | Illustrative | Chosen to make mechanisms visible, not to represent any product |
| Simulation outcomes (RMSE, detection time) | Synthetic | Valid only for the stated model and seeds |
