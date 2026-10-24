#!/usr/bin/env python3
"""The blades' lag damping in the hover against the full-scale rotor's on NASA Ames' rotor test apparatus
[TM-85892 (Peterson and Warmbrodt 1984) fig. 6: decay coefficient of the regressing in-plane mode against CT/sigma at
425 rpm, curve fit through the points; the apparatus' stiff, heavy support leaves essentially the isolated blade's lag
mode]. The model: airframe frozen, rotor speed held, cyclic zero; one blade's lag is kicked and a damped sinusoid
(plus the trim's harmonics of the rotor speed) is fitted to its lag over the next 2.5 s.

  python3 lag_damping.py [root]"""
import sys, math
import numpy as np
from scipy.optimize import least_squares
import harness as H
from data import nasa_hover as NH

# TM-85892 fig. 6, 425 rpm: the curve fit, decay coefficient (1/s) against CT/sigma
TM85892_SIGMA = [(0.0, -0.64), (0.02, -0.54), (0.04, -0.58), (0.06, -0.80), (0.08, -1.13), (0.10, -1.65), (0.12, -2.35)]
KICK = 0.004           # tip lag / R


def decay(f, coll, settle=2.0, span=2.5):
    H.controls(f, coll, 0.0, 0.0, 0.0)
    f['ic/vt-kts'] = 0.0; f['ic/theta-deg'] = 0.0; f['ic/phi-deg'] = 0.0
    f['ic/h-sl-ft'] = 1000.0; f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
    f.run_ic()
    dt = f['simulation/dt']
    ct = []
    for i in range(int(settle / dt)):
        f.run()
        ct.append(f['aero/rotor/thrust-lbs'])
    f['rotor/b0/ql'] = f['rotor/b0/ql'] + KICK
    t, y = [], []
    for i in range(int(span / dt)):
        f.run()
        t.append(i * dt); y.append(f['rotor/b0/ql'])
    t, y = np.array(t), np.array(y)
    W = f['rotor/omega-rad_sec']
    T = float(np.mean(ct[-int(0.5 / dt):]))
    rho = f['atmosphere/rho-slugs_ft3']
    import bo105data as U
    ct_s = T / (rho * math.pi * U.R ** 2 * (W * U.R) ** 2) / (U.NB * U.CHORD / (math.pi * U.R))

    def model(x):
        a0, s, w, A, ph = x[:5]
        h = x[5:]
        r = a0 + A * np.exp(s * t) * np.cos(w * t + ph)
        for n in range(1, 5):
            r = r + h[2 * n - 2] * np.cos(n * W * t) + h[2 * n - 1] * np.sin(n * W * t)
        return r
    x0 = [y.mean(), -1.0, 0.66 * W, KICK, 0.0] + [0.0] * 8
    sol = least_squares(lambda x: model(x) - y, x0)
    return ct_s, sol.x[1], sol.x[2] / W, float(np.sqrt(np.mean(sol.fun ** 2)))


if __name__ == '__main__':
    root = sys.argv[1] if len(sys.argv) > 1 else None
    f = H.load(root=root)
    f['ctl/init'] = 1
    H.freeze(f)
    print(' coll   CT/sigma   decay 1/s (test fit)   lag freq /rev   fit rms')
    for coll in (0.30, 0.40, 0.50, 0.55, 0.62, 0.70):
        cts, s, nu, rms = decay(f, coll)
        print('%5.2f   %7.4f   %7.3f  (%6.3f)        %6.3f        %.1e' % (coll, cts, s, np.interp(cts, *zip(*TM85892_SIGMA)), nu, rms))
