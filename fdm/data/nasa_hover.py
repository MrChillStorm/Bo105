"""Full-scale Bo 105 rotor in hover on NASA Ames' Rotor Test Apparatus, 425 rpm, shaft tilted 10 deg forward, 1/rev
flapping nulled with cyclic. NASA TM-85892 (Peterson and Warmbrodt 1984) appendix B: collective at 0.7 R (deg),
thrust and power coefficients over solidity (0.07); TM-103867 (Peterson 1991) fig. 5: steady torsion moment at
r/R = 0.34 (in lb), digitised."""
# (theta at 0.7 R deg, CT/sigma, CP/sigma), runs 10.7, 10.6, 8.8, 9.7, 9.5, 9.4, 9.3 at 425 rpm (rho 1.195-1.203 kg/m^3)
HOVER = [(3.3, -0.000044, 0.001087), (6.7, 0.019913, 0.001842), (8.9, 0.041104, 0.003093),
         (11.2, 0.061961, 0.004791), (13.0, 0.078788, 0.006412), (14.9, 0.104688, 0.009573), (16.5, 0.119362, 0.012141)]
SIGMA = 0.07
# the tunnel's air speed recorded at each of those points (appendix A, VKNOTS, kt): with the access doors open the rotor
# still drove some air through the test section, at its 10 deg forward shaft tilt
TUNNEL_KT = [3.9, 4.3, 0.0, 6.3, 0.0, 11.0, 6.7]
SHAFT_TILT_DEG = 10.0
TORSION_034 = [(0.010, -320.0), (0.025, -425.0), (0.035, -450.0), (0.060, -480.0), (0.065, -490.0), (0.075, -470.0), (0.10, -500.0)]
