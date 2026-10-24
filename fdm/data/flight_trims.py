"""Bo 105 level-flight trims from flight tests, digitised (fdm/digitize.py and a calibrated grid on each panel).

DLR: the DLR Bo 105-S123 as published by Padfield, Helicopter Flight Dynamics, 3rd ed. (2018), figs. 4.6a, 4.10a-d and
4.11 ("flight test" against Helisim): 2200 kg; sea level standard assumed (Helisim's conditions). Blade pitch
harmonics in Helisim's convention: theta = theta0 + theta_tw r/R + theta1s sin psi + theta1c cos psi, theta0 at the
shaft; theta1s < 0 is forward cyclic, theta1c > 0 left; rotor torque in kN m (fig. 4.11's right axis: 44.4 rad/s).

MBB: the Bo 105 V-4, flight 108, as plotted in USAAMRDL TR-75-50 fig. 16 against C-81: 4500 lb, 5000 ft density
altitude, CG 3.1 in forward; collective at 0.7 R (the shaft's less 5.6 deg), cyclic as swashplate angles: longitudinal
+ forward, lateral + right; pitch attitude. (Its tail rotor collective, plotted on an axis whose zero is near the
hover, is left out.)

WEILAND: power required, 4409 lb, mid CG, sea level standard: Weiland, AHS Forum 1968 (the prototype), as plotted in
TR-75-50 fig. 27, hp.
"""
# (kt, value)
DLR_THETA0 = [(0, 14.22), (8.5, 13.78), (12.5, 13.51), (19.9, 13.06), (25.6, 12.65), (34.1, 12.29), (50.1, 12.09),
              (60.3, 12.16), (70.6, 12.58), (80.2, 13.06), (100.2, 14.49)]
DLR_THETA1S = [(0, 1.90), (7.2, 1.54), (14.3, 1.12), (16.6, 0.00), (24.1, -0.11), (35.0, -0.52), (45.3, -0.86),
               (55.3, -1.18), (64.2, -1.34), (76.5, -1.70), (84.8, -2.15), (104.3, -3.17)]
DLR_THETA1C = [(0, 1.11), (4.0, 1.31), (10.8, 1.55), (13.4, 2.34), (20.5, 2.68), (24.4, 2.85), (31.3, 2.72),
               (39.8, 2.38), (50.3, 2.07), (59.9, 1.72), (70.2, 1.49), (79.5, 1.30), (99.4, 1.15)]
DLR_THETA0T = [(2.3, 10.75), (2.8, 10.13), (12.2, 9.26), (15.0, 8.95), (22.9, 5.18), (31.2, 3.47), (40.2, 2.57),
               (70.5, 3.37), (82.7, 3.85), (101.4, 4.58)]
DLR_PITCH = [(0, 2.65), (6.5, 2.82), (9.8, 2.42), (17.4, 2.53), (19.6, 3.03), (28.6, 3.16), (39.5, -0.23),
             (49.3, 0.04), (61.1, -0.72), (68.7, -1.89), (78.8, -2.28), (95.7, -4.33)]
DLR_TORQUE_KNM = [(4.1, 8.92), (6.7, 8.65), (8.0, 7.86), (11.9, 7.61), (18.9, 7.06), (20.9, 6.50), (24.8, 6.23),
                  (32.6, 5.66), (47.3, 5.01), (60.5, 5.27), (70.0, 5.48), (79.1, 6.09), (100.0, 8.02)]
DLR_MASS_KG = 2200.0

MBB_COLL07 = [(0, 9.51), (32, 8.56), (44, 8.60), (52, 7.44), (66, 7.89), (77, 8.65), (87, 9.26), (97, 9.55),
              (107.5, 10.47), (118, 11.35)]
MBB_B1S = [(0, -0.16), (33, 2.05), (44, 2.39), (55, 2.39), (66, 2.50), (78, 2.89), (88, 3.18), (98, 3.75), (108, 4.48),
           (118, 5.18)]
MBB_A1S = [(0, -0.70), (33, -2.24), (44, -1.70), (55, -1.63), (66, -1.39), (78, -1.19), (88, -0.89), (98, -0.67),
           (108, -0.31), (118, -0.11)]
MBB_PITCH = [(0, 3.2), (33, 1.2), (44, 0.1), (55, 0.3), (66, -0.5), (78, -1.05), (88, -1.85), (98, -3.1), (108, -5.1),
             (119, -5.75)]
MBB_GW, MBB_DENSITY_ALT, MBB_CG_FWD = 4500.0, 5000.0, 3.1

WEILAND_HP = [(0, 465), (20, 415), (40, 318), (60, 274), (80, 306), (100, 429), (120, 555), (140, 683)]
WEILAND_GW = 4409.0
