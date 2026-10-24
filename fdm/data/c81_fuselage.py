"""Bo 105 fuselage aerodynamics (with the fin, without the horizontal stabiliser and the rotor), wind axes.

Measured: Boeing Vertol's 1/4-scale static model of the Bo 105 (BVWT 039, D212-10005-1, 1970) as plotted in
USAAMRDL TR-75-50 fig. 14 (sheets 1-3), digitised here: lift, drag and pitching moment against the tunnel's pitch
angle (-15..15 deg) at 0, 10 and 45 deg of yaw; side force, rolling and yawing moment against yaw (-50..50 deg) at
zero pitch. Forces over dynamic pressure in ft^2, moments in ft^3, about the data reference point FS 100.39, BL 0,
WL -1.86 [TR card 21]. C-81 sign conventions: yaw psi + with the nose right of the wind (wind from the left, sideslip
negative in JSBSim's sense); side force + to the right; rolling moment + right side down; yawing moment + nose right;
pitching moment + nose up; lift + up; drag + along the wind.

Also kept: the C-81 nominal-angle coefficients Boeing fitted to these data [TR cards 23-2E], which reproduce the
zero-yaw curves within the reading accuracy (they are used for the lift at zero yaw, a polynomial in pitch).

Beyond the tests (|pitch| > 20 deg, |yaw| > 50 deg) the coefficients are faired into a bluff-body cross-flow model
whose areas and centres come from the FlightGear 3-D model (fdm/acread.py): planform 8.39 m^2 (90.3 ft^2) with its
centroid at FS 107.8, side 9.60 m^2 (103.3 ft^2) at FS 120.5, WL 5.4 (fuselage, tail boom, fin, cowlings, skids
excluded), normal-force coefficients 0.9 (planform) and 0.85 (side) [Hoerner, Fluid-Dynamic Drag, ch. 3 and 13: blunt
bodies and their cross-flow drag].
"""
import math

FS_REF, BL_REF, WL_REF = 100.39, 0.0, -1.86

# ---------------------------------------------------------------- C-81 nominal-angle coefficients [TR cards 23-2E]
L0, L_TH, L_TH2, L_TH3 = -2.8103, 0.569222, -0.000192, -0.000166      # ft^2, per deg^n
L_PSI, L_PSI2, L_PSITH, L_PSI2TH, L_PSITH2 = 0.026886, 0.003745, 0.00013, 0.000052, -0.000003
D0, D_PSI, D_PSI2 = 9.1025, 0.017826, 0.028820
D_TH, D_PSITH, D_PSI2TH, D_TH2, D_PSITH2, D_TH3 = -0.158123, 0.000702, 0.000085, 0.006891, -0.000023, -0.000177
M0, M_PSI, M_PSI2, M_TH = -29.3444, 0.056265, -0.006806, 1.575644
M_PSITH, M_PSI2TH, M_TH2, M_PSITH2, M_TH3 = -0.011048, -0.001690, 0.031490, 0.000402, 0.010432

# ---------------------------------------------------------------- digitised wind tunnel data [TR fig. 14]
PITCH = [-15, -10, -5, 0, 5, 10, 15]
DRAG = {0: [12.4, 11.2, 10.1, 9.2, 8.6, 8.3, 7.7],          # D/q, ft^2
        10: [15.8, 14.3, 13.0, 11.8, 11.4, 11.1, 11.0],
        45: [56.2, 56.4, 56.6, 57.0, 57.3, 57.4, 57.6]}
LIFT45 = [-5.1, -2.6, 0.4, 4.4, 6.9, 8.4, 13.9]               # L/q at 45 deg yaw
PMOM = {0: [-74.0, -55.6, -39.0, -29.9, -20.3, -2.7, 31.4],  # M/q, ft^3
        45: [-33.0, -26.0, -32.0, -36.4, -42.0, -41.0, -24.0]}
YAW = [-45, -40, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 40, 47]
SIDE = [-50.4, -42.0, -32.5, -28.3, -20.1, -13.0, -7.6, -2.5, 1.9, 6.7, 11.9, 16.8, 22.9, 29.3, 34.0, 42.0, 48.8]
ROLL = [-65.0, -59.6, -46.0, -39.0, -28.7, -18.2, -9.1, -1.5, 7.3, 16.7, 25.8, 32.7, 39.3, 46.5, 52.7, 58.2, 62.0]
YAWM = [125.0, 109.0, 73.0, 49.0, 24.0, 0.0, -10.0, -19.6, -29.0, -37.7, -46.0, -56.0, -67.0, -85.0, -103.0, -132.0, -158.0]

# ---------------------------------------------------------------- cross-flow model beyond the tests
A_TOP, X_TOP = 90.3, 107.8               # ft^2, centroid FS (in)
A_SIDE, X_SIDE, Z_SIDE = 103.3, 120.5, 5.4
CN_TOP, CN_SIDE = 0.9, 0.85


def lift0(th):
    """L/q at zero yaw, the C-81 polynomial (deg)"""
    return L0 + L_TH * th + L_TH2 * th * th + L_TH3 * th ** 3


def _interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    for i in range(len(xs) - 1):
        if x <= xs[i + 1]:
            f = (x - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + f * (ys[i + 1] - ys[i])
    return ys[-1]


def _fair(f_lo, x_lo, f_hi, x_hi, x):
    """blend smoothly (cosine) from f_lo at x_lo to f_hi at x_hi"""
    t = min(1.0, max(0.0, (x - x_lo) / (x_hi - x_lo)))
    w = 0.5 - 0.5 * math.cos(math.pi * t)
    return (1 - w) * f_lo + w * f_hi


ALPHA = [-90, -75, -60, -45, -35, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 35, 45, 60, 75, 90]
PSI_ABS = [0, 10, 20, 30, 45, 60, 75, 90]
PSI = [-90, -75, -60, -50, -45, -40, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 40, 45, 50, 60, 75, 90]


def crossflow_alpha(a):
    """lift, drag (wind axes) and pitching moment of the body in pitch at zero yaw, from the cross-flow normal force
    on the planform (up for flow from below, a > 0) and the measured axial drag at zero pitch"""
    ar = math.radians(a)
    n = CN_TOP * A_TOP * math.sin(ar) * abs(math.sin(ar))
    ax = DRAG[0][3] * math.cos(ar) * abs(math.cos(ar))
    lift = n * math.cos(ar) - ax * math.sin(ar)
    drag = n * math.sin(ar) + ax * math.cos(ar)
    # Munk's potential-flow moment (its slope the measured one at zero pitch, M_TH) and the normal force acting at the
    # planform's centroid, aft of the reference point (nose down for flow from below)
    pm = M0 * math.cos(ar) + 0.5 * math.degrees(M_TH) * math.sin(2 * ar) - n * (X_TOP - FS_REF) / 12.0
    return lift, drag, pm


def tables():
    """L0(a), D0(a), M(a, |psi|) ... over +-90 deg: dict of lists for genbo105.fuselage()"""
    L, D, Mz = [], [], []
    for a in ALPHA:
        cl, cd, cm = crossflow_alpha(a)
        if abs(a) <= 15:
            L.append(lift0(a)); D.append(_interp(PITCH, DRAG[0], a)); Mz.append(_interp(PITCH, PMOM[0], a))
        else:
            e = 15 if a > 0 else -15
            lo = (lift0(e), _interp(PITCH, DRAG[0], e), _interp(PITCH, PMOM[0], e))
            L.append(_fair(lo[0], 15, cl, 45, abs(a))); D.append(_fair(lo[1], 15, cd, 45, abs(a))); Mz.append(_fair(lo[2], 15, cm, 45, abs(a)))
    # yaw increments at zero pitch (even in psi): drag and lift; the pitching moment as a table in (alpha, |psi|)
    dD, dL = [], []
    for p in PSI_ABS:
        if p <= 10:
            dD.append(_interp([0, 10], [0.0, DRAG[10][3] - DRAG[0][3]], p))
            dL.append(L_PSI2 * p * p)
        elif p <= 45:
            dD.append(_interp([10, 45], [DRAG[10][3] - DRAG[0][3], DRAG[45][3] - DRAG[0][3]], p))
            dL.append(_interp([10, 45], [L_PSI2 * 100, LIFT45[3] - lift0(0)], p))
        else:
            # body side force grows to the cross-flow value at 90 deg; wind-axis drag at 90 is that side force
            d90 = CN_SIDE * A_SIDE - DRAG[0][3]
            dD.append(_fair(DRAG[45][3] - DRAG[0][3], 45, d90, 90, p))
            dL.append(_fair(LIFT45[3] - lift0(0), 45, -lift0(0), 90, p))
    Mtab = []
    for a in ALPHA:
        row = []
        for p in PSI_ABS:
            m0 = Mz[ALPHA.index(a)]
            if abs(a) <= 15 and p <= 45:
                m45 = _interp(PITCH, PMOM[45], a)
                row.append(_interp([0, 10, 45], [m0, m0, m45], p))
            elif p <= 45:
                m45 = _fair(_interp(PITCH, PMOM[45], 15 if a > 0 else -15), 15, m0 * 0.5, 45, abs(a))
                row.append(_interp([0, 10, 45], [m0, m0, m45], p))
            else:
                m45 = row[PSI_ABS.index(45)]
                row.append(_fair(m45, 45, 0.0, 90, p))
        Mtab.append(row)
    # lateral-directional at zero pitch, odd in psi: measured to 45-47 deg, then the body's cross-flow side force
    # (converted to wind axes) and its moments about the side area's centroid
    Y, R, N = [], [], []
    yb90 = CN_SIDE * A_SIDE
    for p in PSI:
        s = 1 if p >= 0 else -1
        pa = abs(p)
        if pa <= 45:
            Y.append(_interp(YAW, SIDE, p)); R.append(_interp(YAW, ROLL, p)); N.append(_interp(YAW, YAWM, p))
        else:
            # wind-axis side force falls to the body's axial force component at 90 deg (none)
            y45, r45, n45 = (_interp(YAW, f, 45 * s) for f in (SIDE, ROLL, YAWM))
            Y.append(_fair(y45, 45, 0.0, 90, pa))
            R.append(_fair(r45, 45, s * yb90 * (Z_SIDE - WL_REF) / 12.0, 90, pa))
            N.append(_fair(n45, 45, -s * yb90 * (X_SIDE - FS_REF) / 12.0, 90, pa))
    return dict(alpha=ALPHA, psi_abs=PSI_ABS, psi=PSI, L0=L, D0=D, dL=dL, dD=dD, M=Mtab, Y=Y, R=R, N=N)


if __name__ == '__main__':
    t = tables()
    print('alpha  L0     D0     M(psi 0)')
    for i, a in enumerate(t['alpha']):
        print('%5d %6.1f %6.1f %7.1f' % (a, t['L0'][i], t['D0'][i], t['M'][i][0]))
    print('psi    dD     dL'); [print('%5d %6.1f %6.1f' % (p, t['dD'][i], t['dL'][i])) for i, p in enumerate(t['psi_abs'])]
    print('psi    Y      R      N'); [print('%5d %6.1f %6.1f %7.1f' % (p, t['Y'][i], t['R'][i], t['N'][i])) for i, p in enumerate(t['psi'])]
