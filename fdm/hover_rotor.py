#!/usr/bin/env python3
"""The isolated rotor in hover against NASA's full-scale test [data/nasa_hover.py]: thrust and power against the
collective at 0.7 R, and the blade's steady torsion moment at 0.34 R against thrust. The model's whole aircraft is
held far above the ground at 425 rpm and sea level density, at each point's recorded tunnel speed with the shaft tilted
10 deg forward into it as in the test (the airframe 7 deg nose down: its own shaft is tilted 3 deg); its cyclic nulls
the first harmonic flapping, as the test's did."""
import math, sys
import numpy as np
import harness as H
import bo105data as U
from data import nasa_hover as NH

OMEGA_TEST = 425 * math.pi / 30


def point(f, th07, V_kt=0.0, seconds=3.0):
    root = th07 - U.TWIST * 0.7
    pitch = -(NH.SHAFT_TILT_DEG - math.degrees(U.SHAFT_TILT))
    coll = (root - U.COLL_ROOT[0]) / (U.COLL_ROOT[1] - U.COLL_ROOT[0])
    lon, lat = 0.0, 0.0
    for it in range(4):                  # null the 1/rev flapping with the cyclic
        H.controls(f, coll, lon, lat, 0.0)
        f['ic/vt-kts'] = V_kt; f['ic/gamma-deg'] = 0; f['ic/alpha-deg'] = pitch if V_kt > 0 else 0; f['ic/beta-deg'] = 0
        f['ic/h-sl-ft'] = 300; f['ic/terrain-elevation-ft'] = -10000
        f.run_ic()
        f['rotor/omega-rad_sec'] = OMEGA_TEST
        n = int(seconds / f['simulation/dt']); acc = np.zeros(5); m = 0
        for i in range(n):
            f.run()
            if i > n // 2:
                acc += [f['aero/rotor/thrust-lbs'], f['aero/rotor/power-hp'], f['aero/rotor/a1-deg'], f['aero/rotor/b1-deg'],
                        np.mean([f['rotor/b%d/torsion-034-inlb' % k] for k in range(4)])]; m += 1
        T, P, a1, b1, mt = acc / m
        if abs(a1) < 0.05 and abs(b1) < 0.05:
            break
        lon += a1 / (U.B1S_RANGE[1] - U.B1S_RANGE[0]) * 2 * 1.0
        lat -= b1 / (U.A1S_RANGE[1] - U.A1S_RANGE[0]) * 2 * 1.0
    rho = f['atmosphere/rho-slugs_ft3']; A = math.pi * U.R ** 2; VT = OMEGA_TEST * U.R
    s = NH.SIGMA
    ct = T / (rho * A * VT ** 2); cp = P * 550 / (rho * A * VT ** 3)
    return ct / s, cp / s, mt, a1, b1


if __name__ == '__main__':
    f = H.load()
    H.freeze(f)
    f['drive/hold-rpm'] = 1
    print(' th0.7  tunnel  CT/s model  test     CP/s model  test     FM model  test    torsion 0.34R model  test')
    for (th, ct_t, cp_t), V in zip(NH.HOVER, NH.TUNNEL_KT):
        cts, cps, mt, a1, b1 = point(f, th, V)
        fm = lambda ct, cp: (ct * NH.SIGMA) ** 1.5 / (math.sqrt(2) * cp * NH.SIGMA) if ct > 0 else float('nan')
        mt_t = float(np.interp(cts, *zip(*NH.TORSION_034)))
        print('%5.1f  %4.1f kt  %.4f    %.4f   %.5f   %.5f   %.3f  %.3f   %7.0f  %7.0f   (a1 %.2f b1 %.2f)' % (
            th, V, cts, ct_t, cps, cp_t, fm(cts, cps), fm(ct_t, cp_t), mt, mt_t, a1, b1))
