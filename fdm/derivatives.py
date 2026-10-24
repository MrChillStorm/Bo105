#!/usr/bin/env python3
"""Quasi-steady 6-DOF derivatives and modes at 80 kt against the DLR Bo 105's flight identifications
[data/ar280_bo105.py: AGARD AR-280 tables 6.2.8-6.2.13, six organisations; Kaletka et al. ERF 1989 table 2].

The model (the identification flights' loading, aircraft bo105-ar280) is trimmed in level flight at 80 kt, 3000 ft
density altitude (the tests' [AR table 6.2.2]); then, the airframe frozen and the rotor running, each body velocity,
rate and control is moved either way of trim and the aerodynamic forces and moments (rotor included) are averaged
over two revolutions once the rotor and its wake have settled: central differences, in the identification's units and
axes (body, at the CG; specific moments with its inertias, Ixz = 0). Controls in % of travel as DLR's: longitudinal
+ aft, lateral + right, pedal + right, collective + up, each 100 % the full travel of the model's control.
The modes follow from the 8-state linear model (u v w p q r phi theta) with the trim's kinematic and gravity terms.

  python3 derivatives.py"""
import math, sys
import numpy as np
import harness as H
import bo105data as U
from data import ar280_bo105 as AR

G = 9.80665
FT = 0.3048
LBF = 4.448222
INERTIA = (1700.0, 4200.0, 4200.0)          # kg m^2 [AR table 6.2.2]
MASS = 2200.0
V_KT, ALT_FT = 80.0, 3000.0
STEP = {'u': 1.0, 'v': 1.0, 'w': 0.5, 'p': 0.05, 'q': 0.05, 'r': 0.05, 'col': 2.0, 'lon': 2.0, 'lat': 2.0, 'ped': 2.0}
KEYS = ['forces/fbx-aero-lbs', 'forces/fby-aero-lbs', 'forces/fbz-aero-lbs',
        'moments/l-aero-lbsft', 'moments/m-aero-lbsft', 'moments/n-aero-lbsft']


def loads(f, trim, d, settle=1.0, revs=2):
    """forces (N) and moments (N m) at the CG with the state and control offsets d (SI, % for controls)"""
    coll, lon, lat, ped = trim['x'][:4]
    H.controls(f, coll + 0.01 * d.get('col', 0), lon - 0.02 * d.get('lon', 0), lat + 0.02 * d.get('lat', 0), ped + 0.02 * d.get('ped', 0))
    f['ic/theta-deg'] = trim['theta']; f['ic/phi-deg'] = trim['phi']; f['ic/psi-true-deg'] = 0.0
    f['ic/h-sl-ft'] = ALT_FT; f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
    f['ic/u-fps'] = trim['u'] + d.get('u', 0) / FT
    f['ic/v-fps'] = trim['v'] + d.get('v', 0) / FT
    f['ic/w-fps'] = trim['w'] + d.get('w', 0) / FT
    f['ic/p-rad_sec'] = d.get('p', 0); f['ic/q-rad_sec'] = d.get('q', 0); f['ic/r-rad_sec'] = d.get('r', 0)
    f.run_ic()
    dt = f['simulation/dt']
    for _ in range(int(settle / dt)):
        f.run()
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / dt))
    w = np.sin(np.pi * (np.arange(n) + 0.5) / n) ** 2
    w /= w.sum()
    acc = np.zeros(6)
    for i in range(n):
        f.run()
        acc += w[i] * np.array([f[k] for k in KEYS])
    acc[:3] *= LBF
    acc[3:] *= LBF * FT
    return acc


def model_derivatives():
    f = H.load(model='bo105-ar280')
    x, res, ex = H.trim(f, V_KT, alt_ft=ALT_FT)
    f['rotor/accel-hold'] = 1           # the rotor keeps the trim's accelerations: quasi-steady, no body acceleration
    trim = dict(x=x, theta=f['attitude/theta-deg'], phi=f['attitude/phi-deg'],
                u=f['velocities/u-fps'], v=f['velocities/v-fps'], w=f['velocities/w-fps'], res=float(np.abs(res).max()))
    D = {}
    for var, h in STEP.items():
        dF = (loads(f, trim, {var: h}) - loads(f, trim, {var: -h})) / (2 * h)
        for i, ax in enumerate('XYZ'):
            D[ax + var] = dF[i] / MASS
        for i, ax in enumerate('LMN'):
            D[ax + var] = dF[3 + i] / INERTIA[i]
    return D, trim


def modes(D, trim):
    """eigenvalues of the 8-state model (u v w p q r phi theta), SI"""
    U0, V0, W0 = trim['u'] * FT, trim['v'] * FT, trim['w'] * FT
    th, ph = math.radians(trim['theta']), math.radians(trim['phi'])
    g = lambda k: D.get(k) or 0.0
    A = np.zeros((8, 8))
    for i, ax in enumerate('XYZLMN'):
        for j, var in enumerate('uvwpqr'):
            A[i, j] = g(ax + var)
    A[0, 4] -= W0; A[0, 5] += V0; A[0, 7] = -G * math.cos(th)
    A[1, 3] += W0; A[1, 5] -= U0; A[1, 6] = G * math.cos(th) * math.cos(ph); A[1, 7] = -G * math.sin(th) * math.sin(ph)
    A[2, 3] -= V0; A[2, 4] += U0; A[2, 6] = -G * math.cos(th) * math.sin(ph); A[2, 7] = -G * math.sin(th) * math.cos(ph)
    A[6, 3] = 1.0; A[6, 4] = math.sin(ph) * math.tan(th); A[6, 5] = math.cos(ph) * math.tan(th)
    A[7, 4] = math.cos(ph); A[7, 5] = -math.sin(ph)
    return np.linalg.eigvals(A)


def describe(ev):
    out = []
    for e in sorted(ev, key=lambda z: (abs(z.imag) < 1e-9, -abs(z))):
        if e.imag > 1e-9:
            wn = abs(e); out.append('[%.2f, %.2f]' % (-e.real / wn, wn))
        elif abs(e.imag) < 1e-9:
            out.append('(%.2f)' % -e.real)
    return out


def table(D):
    rows = []
    for k, vals in AR.DERIV.items():
        ident = [v for v in vals if v is not None]
        rows.append((k, D.get(k, float('nan')), vals, min(ident), max(ident)))
    return rows


if __name__ == '__main__':
    D, trim = model_derivatives()
    print('80 kt, %.0f ft, 2200 kg: trim pitch %.2f deg, roll %.2f deg, residual %.3f' % (ALT_FT, trim['theta'], trim['phi'], trim['res']))
    print('%-6s %9s | %s | identified range' % ('deriv', 'model', ' '.join('%8s' % o for o in AR.ORGS)))
    for k, m, vals, lo, hi in table(D):
        inside = 'in' if lo - 1e-9 <= m <= hi + 1e-9 else ('low' if m < lo else 'high')
        print('%-6s %9.4f | %s | %8.4f .. %8.4f  %s' % (k, m, ' '.join('%8s' % ('-' if v is None else '%.4g' % v) for v in vals), lo, hi, inside))
    ev = modes(D, trim)
    print('model modes:', ' '.join(describe(ev)))
    for mode, vals in AR.MODES.items():
        print('%-10s %s' % (mode, '  '.join('%s %s' % (o, v) for o, v in zip(AR.ORGS, vals))))
