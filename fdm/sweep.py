#!/usr/bin/env python3
"""Level-flight trims against the flight tests [data/flight_trims.py]: the DLR Bo 105-S123 (2200 kg, sea level, as
Padfield shows it) and MBB's V-4 (4500 lb, 5000 ft density altitude, TR-75-50 fig. 16).

  python3 sweep.py [dlr|mbb]       prints model and test side by side; results to stdout as a table"""
import math, sys
import numpy as np
import harness as H
import bo105data as U
from data import flight_trims as FT

PH = math.radians(U.SWASH_PHASE)


def blade_harmonics(A1s, B1s):
    """swashplate cyclic (with the control phasing) -> blade pitch harmonics in Helisim's convention"""
    th1c = -A1s * math.cos(PH) - B1s * math.sin(PH)
    th1s = A1s * math.sin(PH) - B1s * math.cos(PH)
    return th1s, th1c


def run(case, speeds, alt_ft):
    f = H.load(model=case)
    try:                                    # the fin blockage's side force (none when built with --no-tr-fin)
        f['aero/emp/fin/dFy-lbs']; fin = ('aero/emp/fin/dFy-lbs',)
    except KeyError:
        fin = ()
    rows = []
    x0 = None
    for V in speeds:
        x, r, e = H.trim(f, V, alt_ft=alt_ft, x0=x0, extra=('aero/rotor/power-hp', 'aero/rotor/tr/power-hp', 'aero/rotor/torque-ftlb',
                                                           'drive/loss-ftlb', 'aero/rotor/thrust-lbs', 'aero/rotor/tr/thrust-lbs') + fin)
        x0 = x
        th0, B1s, A1s, thTR = H.deg(x)
        th1s, th1c = blade_harmonics(A1s, B1s)
        mr_hp, tr_hp, q, loss, T, Ttr = e[:6]
        loss_hp = loss * f['rotor/omega-rad_sec'] / 550
        rows.append(dict(V=V, th0=th0, B1s=B1s, A1s=A1s, th1s=th1s, th1c=th1c, thTR=thTR, pitch=f['attitude/theta-deg'],
                         roll=f['attitude/phi-deg'], mr_hp=mr_hp, tr_hp=tr_hp, tot_hp=mr_hp + tr_hp + loss_hp,
                         q_knm=q * 1.35582 / 1000, T=T, Ttr=Ttr, fin=float(e[6]) if fin else 0.0, res=float(np.abs(r).max()), pct=H.pct(x)))
    return rows


def interp(data, V):
    xs, ys = zip(*data)
    return float(np.interp(V, xs, ys)) if xs[0] - 3 <= V <= xs[-1] + 3 else float('nan')


if __name__ == '__main__':
    which = sys.argv[1] if len(sys.argv) > 1 else 'dlr'
    if which == 'dlr':
        speeds = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120]
        rows = run('bo105', speeds, 0.0)
        print('   V | theta0 m/t     | th1s m/t      | th1c m/t      | thTR m/t      | pitch m/t     | MR torque kNm m/t (total) | roll  res')
        for r in rows:
            V = r['V']
            print('%4.0f | %5.2f %5.2f  | %5.2f %5.2f  | %5.2f %5.2f  | %5.2f %5.2f  | %5.2f %5.2f  | %5.2f %5.2f (%5.2f)      | %5.2f %.3f' % (
                V, r['th0'], interp(FT.DLR_THETA0, V), r['th1s'], interp(FT.DLR_THETA1S, V), r['th1c'], interp(FT.DLR_THETA1C, V),
                r['thTR'], interp(FT.DLR_THETA0T, V), r['pitch'], interp(FT.DLR_PITCH, V), r['q_knm'], interp(FT.DLR_TORQUE_KNM, V),
                r['tot_hp'] * 745.7 / 1000 / 44.4, r['roll'], r['res']))
    else:
        speeds = [0, 20, 33, 44, 55, 66, 78, 88, 98, 108, 118]
        rows = run('bo105-mbb', speeds, FT.MBB_DENSITY_ALT)
        print('   V | coll0.7 m/t   | B1s m/t       | A1s m/t       | pitch m/t     | MR hp  total hp | roll  res')
        for r in rows:
            V = r['V']
            print('%4.0f | %5.2f %5.2f  | %5.2f %5.2f  | %5.2f %5.2f  | %5.2f %5.2f  | %5.0f  %5.0f    | %5.2f %.3f' % (
                V, r['th0'] - 5.6, interp(FT.MBB_COLL07, V), r['B1s'], interp(FT.MBB_B1S, V), r['A1s'], interp(FT.MBB_A1S, V),
                r['pitch'], interp(FT.MBB_PITCH, V), r['mr_hp'], r['tot_hp'], r['roll'], r['res']))
