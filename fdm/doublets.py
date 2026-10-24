#!/usr/bin/env python3
"""Rate responses to the DLR Bo 105's verification doublets at 80 kt against the flight records
[data/ar280_doublets.json: AGARD AR-280 figs. 6.2.17 and 6.2.20, four runs: longitudinal, lateral, pedal, collective].

For each run the model (the identification flights' loading, bo105-ar280) is trimmed at 80 kt, 3000 ft, then flown
free with the recorded increments of all four controls from their values in the run's quiet window (each 100 % the full
travel of the model's control: longitudinal + aft, lateral + right, pedal + right, collective + up), rotor speed
governed, no stability augmentation (the Bo 105-S123 flew these without). Rates are compared as increments from their
values in the same quiet window, over each run's record (14 to 26 s). Inputs and rates are taken on the figures' common
80 s axis (their run separators differ by up to 0.2 s), each run from a quiet window before its doublet.

  python3 doublets.py [--plot out.png]"""
import sys, os, json, math
import numpy as np
from multiprocessing import Pool
import harness as H

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, 'data', 'ar280_doublets.json')))
RUNS = ('lon', 'lat', 'ped', 'col')
GEAR = {'lon': -0.02, 'lat': 0.02, 'ped': 0.02, 'col': 0.01}     # model control per %
V_KT, ALT_FT = 80.0, 3000.0


# each run's start on the common time axis (the input figure's separator, s) and its quiet window before the doublet
# (s from that start; the first 0.25 s of a run is lost under the separator in both figures)
START = [D['inputs']['segments'][k] for k in range(4)]
END = [D['inputs']['segments'][k + 1] for k in range(4)]
QUIET = [(0.3, 0.9), (0.25, 0.5), (0.3, 0.9), (0.0, 0.2)]


def segment(k, rec):
    """the run's record from the start of its quiet window, on the common time axis"""
    t = np.array(D[rec]['t'])
    t0 = START[k] + QUIET[k][0]
    m = (t >= t0) & (t <= END[k] - 0.3)
    return t[m] - t0, {n: np.array(D[rec][n])[m] for n in D[rec] if n not in ('t', 'segments')}


def base(k, t, y):
    return float(np.median(y[t <= QUIET[k][1] - QUIET[k][0]]))


def simulate(k):
    ti, u = segment(k, 'inputs')
    f = H.load(model='bo105-ar280')
    x, res, ex = H.trim(f, V_KT, alt_ft=ALT_FT)
    coll, lon, lat, ped = x[:4]
    f['drive/hold-rpm'] = 0.0
    H.freeze(f, False)
    f['ctl/init'] = 0
    du = {n: u[n] - base(k, ti, u[n]) for n in RUNS}
    dt = f['simulation/dt']
    rec = []
    for i in range(int(ti[-1] / dt) + 1):
        tt = i * dt
        d = {n: float(np.interp(tt, ti, du[n])) * GEAR[n] for n in RUNS}
        H.controls(f, coll + d['col'], lon + d['lon'], lat + d['lat'], ped + d['ped'])
        f.run()
        rec.append((tt, f['velocities/p-rad_sec'], f['velocities/q-rad_sec'], f['velocities/r-rad_sec']))
    return k, np.array(rec), float(np.abs(res).max())


def compare(runs):
    rows = []
    for k, r, res in sorted(runs):
        tr, y = segment(k, 'rates')
        for j, ax in enumerate('pqr', 1):
            test = y[ax] - base(k, tr, y[ax])
            mod = np.interp(tr, r[:, 0], r[:, j])
            mod = mod - base(k, tr, mod)
            w = tr <= min(10.0, tr[-1])
            pk_t = test[w][np.argmax(np.abs(test[w]))]
            pk_m = mod[w][np.argmax(np.abs(mod[w]))]
            rows.append((RUNS[k], ax, pk_t, pk_m, float(np.sqrt(np.mean((mod[w] - test[w]) ** 2))), tr, test, mod))
    return rows


if __name__ == '__main__':
    with Pool(4) as pool:
        runs = pool.map(simulate, range(4))
    rows = compare(runs)
    print('80 kt doublets, rate increments (rad/s) over the first 10 s: largest value in the test and the model, RMS of the difference')
    for run, ax, a, b, e, *_ in rows:
        print('%-4s %s   test %+.3f   model %+.3f   rms %.3f' % (run, ax, a, b, e))
    if '--plot' in sys.argv:
        import matplotlib; matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(3, 4, figsize=(13, 7), sharex='col')
        for i, (run, ax, a, b, e, tr, test, mod) in enumerate(rows):
            c, rr = i // 3, i % 3
            axs[rr, c].plot(tr, test, 'k', lw=1, label='flight')
            axs[rr, c].plot(tr, mod, 'C3', lw=1, label='model')
            if rr == 0: axs[rr, c].set_title(run + ' doublet')
            if c == 0: axs[rr, c].set_ylabel('%s, rad/s' % ax)
        axs[0, 0].legend(fontsize=8)
        for c in range(4): axs[2, c].set_xlabel('t, s')
        fig.tight_layout(); fig.savefig(sys.argv[sys.argv.index('--plot') + 1], dpi=100)
