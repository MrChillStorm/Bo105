#!/usr/bin/env python3
"""Roll rate response to lateral stick at 80 kt against the DLR Bo 105's identified transfer function
[Kaletka, Tischler, von Gruenhagen, Fletcher, 15th ERF (1989) paper 66, section 6.1: the "baseline" 4th/7th order
model fitted to the flight frequency response from 1 to 30 rad/s:
    phi / dlat = 2.457 [0.447, 3.237] [0.045, 14.94] e^(-0.017 s) / ( s [0.317, 2.858] [0.021, 14.98] [0.450, 13.142] )
with [zeta, w] = s^2 + 2 zeta w s + w^2, dlat in % of travel, phi in rad: the Dutch roll (2.86 rad/s), the lag/air
resonance (15.0 rad/s, zeta 0.021) and the coupled body-roll / regressing flap mode (13.1 rad/s, zeta 0.45)].

The model (the identification flights' loading, bo105-ar280) is trimmed at 80 kt, 3000 ft, then flown free with a
logarithmic lateral stick sweep of 1.5 % from 0.7 to 30 rad/s over 60 s (rotor speed free, governed); pitch attitude
is held loosely through the longitudinal stick (rates low-passed, as engine_response.py), the other axes are free.
The frequency response is the ratio of the cross- to the input auto-spectrum, smoothed over 1/12 decade.

  python3 freqresp.py [--plot out.png]"""
import sys, math
import numpy as np
import harness as H

V_KT, ALT_FT = 80.0, 3000.0
AMP = 1.5                      # % lateral stick
W0, W1, T = 0.7, 30.0, 60.0
K_ATT, K_RATE, TAU_RATE = 0.010, 0.005, 0.15
GEAR_LAT = 0.02                # model lateral stick (-1..1) per %


def tischler(w):
    """p / dlat (rad/s per %) of the identified transfer function at frequencies w (rad/s)"""
    s = 1j * np.asarray(w)
    q = lambda z, wn: s * s + 2 * z * wn * s + wn * wn
    phi = 2.457 * q(0.447, 3.237) * q(0.045, 14.94) * np.exp(-0.017 * s) / (s * q(0.317, 2.858) * q(0.021, 14.98) * q(0.450, 13.142))
    return s * phi


def sweep(root=None, model='bo105-ar280'):
    f = H.load(root=root, model=model)
    x, res, ex = H.trim(f, V_KT, alt_ft=ALT_FT)
    coll, lon, lat, ped = x[:4]
    th0 = f['attitude/theta-deg']
    f['drive/hold-rpm'] = 0.0
    H.freeze(f, False)
    f['ctl/init'] = 0
    dt = f['simulation/dt']
    n = int(T / dt)
    t = np.arange(n) * dt
    k = math.log(W1 / W0) / T
    phase = W0 * (np.exp(k * t) - 1) / k
    taper = np.minimum(1, np.minimum(t / 2.0, (T - t) / 2.0))
    d = AMP * np.sin(phase) * taper
    p = np.zeros(n)
    qf = 0.0
    for i in range(n):
        qf += dt / TAU_RATE * (math.degrees(f['velocities/q-rad_sec']) - qf)
        H.controls(f, coll, lon + K_ATT * (f['attitude/theta-deg'] - th0) + K_RATE * qf, lat + GEAR_LAT * d[i], ped)
        f.run()
        p[i] = f['velocities/p-rad_sec']
    return t, d, p - p.mean(), float(np.abs(res).max())


def response(t, d, p, w_out):
    dt = t[1] - t[0]
    win = np.hanning(len(t))
    D = np.fft.rfft(d * win); P = np.fft.rfft(p * win)
    wf = 2 * math.pi * np.fft.rfftfreq(len(t), dt)
    Sdp, Sdd = np.conj(D) * P, np.abs(D) ** 2
    out = []
    for w in w_out:
        m = (wf > w * 10 ** (-1 / 24)) & (wf < w * 10 ** (1 / 24))
        out.append(Sdp[m].sum() / Sdd[m].sum())
    return np.array(out)


if __name__ == '__main__':
    t, d, p, res = sweep()
    w = np.logspace(math.log10(1.0), math.log10(25.0), 30)
    Hm, Ht = response(t, d, p, w), tischler(w)
    print('80 kt, lateral stick sweep (trim residual %.3f): p/dlat, dB and deg, model vs identified' % res)
    for wi, a, b in zip(w, Hm, Ht):
        print('%6.2f rad/s   %6.1f %6.1f dB   %7.1f %7.1f deg' % (wi, 20 * np.log10(abs(a)), 20 * np.log10(abs(b)),
                                                                  np.degrees(np.angle(a)), np.degrees(np.angle(b))))
    if '--plot' in sys.argv:
        import matplotlib; matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(7, 7), sharex=True)
        wd = np.logspace(0, math.log10(30), 300)
        ax[0].semilogx(wd, 20 * np.log10(abs(tischler(wd))), 'k', label='identified (Kaletka et al. 1989)')
        ax[0].semilogx(w, 20 * np.log10(abs(Hm)), 'C3o-', ms=3, label='model')
        ax[1].semilogx(wd, np.degrees(np.unwrap(np.angle(tischler(wd)))), 'k')
        ax[1].semilogx(w, np.degrees(np.unwrap(np.angle(Hm))), 'C3o-', ms=3)
        ax[0].set_ylabel('p / dlat, dB (rad/s per %)'); ax[1].set_ylabel('phase, deg'); ax[1].set_xlabel('rad/s')
        ax[0].legend(fontsize=8)
        fig.tight_layout(); fig.savefig(sys.argv[sys.argv.index('--plot') + 1], dpi=110)
