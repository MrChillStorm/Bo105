#!/usr/bin/env python3
"""Rotor speed, shaft torque and collective lag after a collective 3-2-1-1 in the hover, against DLR's flight test
[data/hg_fig6.json: Hamers and von Gruenhagen, ERF 1997 paper 91, fig. 6; the DLR Bo 105-S123].

The model is trimmed in the hover at 2200 kg (the S123's AGARD AR-280 loading), 1000 ft, with the governor's beep
trim set so that it settles near the test's rotor speed. Then the rotor speed and the airframe are let go and the
recorded collective is applied as increments from its first value, at 0.16 deg of blade pitch per % (DLR's percent of
the full travel, 6 to 22 deg at the root). Pitch and roll are held by a cyclic feedback on attitude and rate, as
DLR's own run "with attitude stabilisation"; pedals stay at their trim (the pilot's own pedal and cyclic corrections
after about 6 s are not known, so yaw is compared only before them).

  python3 engine_response.py [--plot out.png]"""
import sys, os, json, math
import numpy as np
import harness as H
import bo105data as U
import engines250

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, 'data', 'hg_fig6.json')))
NM = 1.35582
SETTLE = 3.0           # s with the airframe free before the record starts
# stick (-1..1) per deg and per deg/s, the rates through a first-order lag: unfiltered rate feedback reaches the
# regressing lag mode (2.4 Hz, air resonance) and drives it unstable, as it would on the aircraft
K_ATT, K_RATE, TAU_RATE = 0.015, 0.008, 0.15


def run(root=None, props=None):
    """props: {property: value} set after loading (e.g. the governor's eng/gov-k, eng/gov-tau, eng/coll-ff)"""
    f = H.load(root=root)
    for k, val in (props or {}).items():
        f[k] = val
    # trimmed at the test's rotor speed (the governor's datum starts at its equilibrium there)
    nr = 100 * D['rotor_speed_rads'][0] / U.OMEGA
    f['/controls/engines/power-trim'] = (nr - 100) / engines250.TRIM_PCT
    x, res, ex = H.trim(f, 0.0, alt_ft=1000.0)
    coll, lon, lat, ped = x[:4]
    th0, ph0 = f['attitude/theta-deg'], f['attitude/phi-deg']
    f['drive/hold-rpm'] = 0.0
    H.freeze(f, False)
    f['ctl/init'] = 0
    dt = f['simulation/dt']
    t_data = np.array(D['t'])
    c_data = np.array(D['collective_pct'])
    rec = []
    rf = np.zeros(2)
    n0 = int(round(SETTLE / dt))
    for i in range(n0 + int(round(t_data[-1] / dt)) + 1):
        t = i * dt - SETTLE
        dc = (np.interp(t, t_data, c_data) - c_data[0]) / 100.0 if t >= 0 else 0.0
        rf += dt / TAU_RATE * (np.degrees([f['velocities/p-rad_sec'], f['velocities/q-rad_sec']]) - rf)
        H.controls(f, coll + dc,
                   lon + K_ATT * (f['attitude/theta-deg'] - th0) + K_RATE * rf[1],
                   lat - K_ATT * (f['attitude/phi-deg'] - ph0) - K_RATE * rf[0], ped)
        f.run()
        if t >= -1e-9:
            rec.append((t, f['rotor/omega-rad_sec'], f['drive/mr-torque-ftlb'] * NM,
                        sum(f['rotor/b%d/ql' % k] for k in range(U.NB)) / U.NB, f['velocities/r-rad_sec'],
                        -f['accelerations/Nz'] * 9.80665, f['eng[0]/torque-pct'], f['eng[0]/ng-pct']))
    r = np.array(rec)
    m = {k: np.interp(t_data, r[:, 0], r[:, j]) for j, k in enumerate(
        ('rotor_speed_rads', 'shaft_torque_nm', 'coll_lag_rad', 'yaw_rate_rads', 'vert_acc_ms2', 'tq_pct', 'ng_pct'), 1)}
    return m, float(np.abs(res).max()), x


COMPARE = (('rotor_speed_rads', 'rotor speed, rad/s', 15.0), ('shaft_torque_nm', 'shaft torque, N m', 15.0),
           ('coll_lag_rad', 'collective lag, rad (model: tip / R)', 15.0), ('yaw_rate_rads', 'yaw rate, rad/s', 6.0),
           ('vert_acc_ms2', 'vertical specific force, m/s2', 15.0))


def table(m):
    t = np.array(D['t'])
    rows = []
    for k, label, tmax in COMPARE:
        y, z = np.array(D[k]), m[k]
        w = t <= tmax
        dy, dz = y - y[0], z - z[0]
        rows.append((label, y[0], z[0], dy[w].min(), dz[w].min(), dy[w].max(), dz[w].max(), float(np.sqrt(np.mean((dz - dy)[w] ** 2)))))
    return rows


if __name__ == '__main__':
    m, res, x = run()
    print('collective 3-2-1-1 in the hover, DLR flight test vs model; trim residual %.3f' % res)
    print('%-38s %10s %10s | %9s %9s | %9s %9s | %8s' % ('', 'test t=0', 'model', 'test min', 'model', 'test max', 'model', 'RMS inc'))
    for row in table(m):
        print('%-38s %10.4g %10.4g | %+9.4g %+9.4g | %+9.4g %+9.4g | %8.4g' % row)
    if '--plot' in sys.argv:
        import matplotlib; matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        t = np.array(D['t'])
        fig, axs = plt.subplots(6, 1, figsize=(7, 11), sharex=True)
        axs[0].plot(t, D['collective_pct'], 'k'); axs[0].set_ylabel('collective %')
        for ax, (k, label, tmax) in zip(axs[1:], COMPARE):
            ax.plot(t, D[k], 'k', label='flight test')
            ax.plot(t, np.array(m[k]) - m[k][0] + D[k][0], 'C3', label='model (increment)')
            ax.set_ylabel(label, fontsize=8)
        axs[1].legend(fontsize=8); axs[-1].set_xlabel('t, s')
        fig.tight_layout(); fig.savefig(sys.argv[sys.argv.index('--plot') + 1], dpi=110)
