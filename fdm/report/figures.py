"""Figures for the report (vector PDF) from report/results/*.json.  python3 report/figures.py"""
import sys, os, json, math
HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
import numpy as np
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import bo105data as U
from data import flight_trims as FT
from mktables import tischler

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['STIX Two Text', 'STIXGeneral', 'DejaVu Serif'], 'mathtext.fontset': 'stix',
                     'font.size': 8.5, 'axes.labelsize': 8.5, 'axes.titlesize': 9, 'legend.fontsize': 7.5, 'xtick.labelsize': 7.5,
                     'ytick.labelsize': 7.5, 'axes.linewidth': 0.6, 'lines.linewidth': 1.1, 'grid.linewidth': 0.3, 'grid.alpha': 0.5,
                     'axes.grid': True, 'figure.dpi': 150, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
                     'axes.spines.top': False, 'axes.spines.right': False})
C_MOD, C_TEST, C_REF = '#b2182b', '#222222', '#7a7a7a'
W1, W2 = 3.35, 6.9          # single / double column width (in)
OUT = os.path.join(HERE, 'fig')
RES = os.path.join(HERE, 'results')


def res(name):
    return json.load(open(os.path.join(RES, name + '.json')))


def save(fig, name):
    fig.savefig(os.path.join(OUT, name + '.pdf')); plt.close(fig)


def tag(ax, s):
    ax.set_title('(%s)' % s, loc='left', fontsize=8.5, pad=3)


def test_pts(ax, data, label='flight test'):
    x, y = zip(*data)
    ax.plot(x, y, 'o', mfc='none', mec=C_TEST, ms=3.6, mew=0.8, label=label)


def trim_dlr():
    rows = res('trims_dlr')
    V = [r['V'] for r in rows]
    panels = [('th0', FT.DLR_THETA0, r'$\theta_0$ (shaft), deg'), ('th1s', FT.DLR_THETA1S, r'$\theta_{1s}$, deg'),
              ('th1c', FT.DLR_THETA1C, r'$\theta_{1c}$, deg'), ('thTR', FT.DLR_THETA0T, r'tail rotor $\theta_{0T}$, deg'),
              ('pitch', FT.DLR_PITCH, r'pitch attitude $\theta$, deg'), ('q_knm', FT.DLR_TORQUE_KNM, 'main rotor torque, kN m')]
    fig, axs = plt.subplots(2, 3, figsize=(W2, 3.9))
    bai = res('trims_dlr_bailey') if os.path.exists(os.path.join(RES, 'trims_dlr_bailey.json')) else None
    for i, (ax, (k, data, lab)) in enumerate(zip(axs.flat, panels)):
        test_pts(ax, data)
        ax.plot(V, [r[k] for r in rows], '-', color=C_MOD, label='model')
        if k == 'thTR' and bai:
            ax.plot([r['V'] for r in bai], [r[k] for r in bai], '--', color=C_REF, lw=0.9, label='Bailey\'s closed form')
            ax.legend(loc='upper right', frameon=False)
        ax.set_ylabel(lab); tag(ax, 'abcdef'[i])
        ax.set_xlim(-3, 123)
        if i >= 3:
            ax.set_xlabel('airspeed, kt')
    axs[0, 1].legend(loc='upper right', frameon=False)
    fig.tight_layout(w_pad=1.2, h_pad=0.6)
    save(fig, 'trim_dlr')


def trim_mbb():
    rows = res('trims_mbb')
    V = [r['V'] for r in rows]
    panels = [(lambda r: r['th0'] - 5.6, FT.MBB_COLL07, r'collective at 0.7$R$, deg'), (lambda r: r['B1s'], FT.MBB_B1S, r'$B_{1s}$ (+ forward), deg'),
              (lambda r: r['A1s'], FT.MBB_A1S, r'$A_{1s}$ (+ right), deg'), (lambda r: r['pitch'], FT.MBB_PITCH, r'pitch attitude, deg')]
    fig, axs = plt.subplots(1, 4, figsize=(W2, 1.85))
    for i, (ax, (g, data, lab)) in enumerate(zip(axs, panels)):
        test_pts(ax, data)
        ax.plot(V, [g(r) for r in rows], '-', color=C_MOD, label='model')
        ax.set_ylabel(lab); ax.set_xlabel('airspeed, kt'); tag(ax, 'abcd'[i])
    axs[3].legend(loc='lower left', frameon=False)
    fig.tight_layout(w_pad=0.8)
    save(fig, 'trim_mbb')


def hover():
    h = res('hover')
    s = h['sigma']
    rows = h['rows']
    fm = lambda ct, cp: (ct * s) ** 1.5 / (math.sqrt(2) * cp * s)
    th = [r['th07'] for r in rows]
    fig, axs = plt.subplots(1, 4, figsize=(W2, 1.95))
    a = axs[0]
    a.plot(th, [r['ct_test'] for r in rows], 'o', mfc='none', mec=C_TEST, ms=3.6, mew=0.8, label='test')
    a.plot(th, [r['ct'] for r in rows], '-', color=C_MOD, label='model')
    a.set_xlabel(r'collective at 0.7$R$, deg'); a.set_ylabel(r'$C_T/\sigma$'); a.legend(frameon=False, loc='lower right')
    a = axs[1]
    a.plot([r['ct_test'] for r in rows], [1e3 * r['cp_test'] for r in rows], 'o', mfc='none', mec=C_TEST, ms=3.6, mew=0.8)
    a.plot([r['ct'] for r in rows], [1e3 * r['cp'] for r in rows], '-', color=C_MOD)
    a.set_xlabel(r'$C_T/\sigma$'); a.set_ylabel(r'$10^3\,C_P/\sigma$')
    a = axs[2]
    a.plot([r['ct_test'] for r in rows if r['ct_test'] > 0.01], [fm(r['ct_test'], r['cp_test']) for r in rows if r['ct_test'] > 0.01], 'o',
           mfc='none', mec=C_TEST, ms=3.6, mew=0.8)
    a.plot([r['ct'] for r in rows], [fm(r['ct'], r['cp']) for r in rows], '-', color=C_MOD)
    a.set_xlabel(r'$C_T/\sigma$'); a.set_ylabel('figure of merit'); a.set_ylim(0, 0.85)
    a = axs[3]
    test_pts(a, h['torsion_test'])
    a.plot([r['ct'] for r in rows], [r['torsion'] for r in rows], '-', color=C_MOD)
    a.set_xlabel(r'$C_T/\sigma$'); a.set_ylabel(r'torsion at 0.34$R$, in lb')
    for i, ax in enumerate(axs):
        tag(ax, 'abcd'[i])
    fig.tight_layout(w_pad=0.6)
    save(fig, 'hover')


def blades():
    md, lg = res('modes'), res('lagdamp')
    fig, axs = plt.subplots(1, 2, figsize=(W1, 1.75))
    a = axs[0]
    r = np.array(md['r']) / md['R']
    for k, ls, c, lab in (('f', '-', C_MOD, 'flap'), ('l', '--', C_TEST, 'lag')):
        y = np.array(md['shapes'][k]); a.plot(r, y / y[-1], ls, color=c, label=lab)
    a.set_xlabel('$r/R$'); a.set_ylabel('first mode shape'); a.legend(frameon=False, loc='upper left')
    a = axs[1]
    x, y = zip(*lg['test'])
    a.plot(x, y, '-', color=C_REF, lw=0.9, label='test fit')
    a.plot([q['ct_sigma'] for q in lg['rows']], [q['decay'] for q in lg['rows']], 'o-', color=C_MOD, ms=2.5, label='model')
    a.set_xlabel(r'$C_T/\sigma$'); a.set_ylabel(r'lag decay, s$^{-1}$'); a.legend(frameon=False, loc='lower left')
    for i, ax in enumerate(axs):
        tag(ax, 'ab'[i])
    fig.tight_layout(w_pad=0.6)
    save(fig, 'blades')


def freq():
    fr = res('freqresp')
    w = np.array(fr['w']); Hm = np.array([complex(*h) for h in fr['H']])
    wd = np.logspace(0, math.log10(25), 400)
    Ht = tischler(wd)
    fig, axs = plt.subplots(2, 1, figsize=(W1, 3.0), sharex=True)
    axs[0].semilogx(wd, 20 * np.log10(np.abs(Ht)), '-', color=C_TEST, lw=0.9, label='identified (Kaletka et al.)')
    axs[0].semilogx(w, 20 * np.log10(np.abs(Hm)), 'o-', color=C_MOD, ms=2.2, lw=0.8, label='model')
    ph_t = np.degrees(np.unwrap(np.angle(Ht))); ph_m = np.degrees(np.unwrap(np.angle(Hm)))
    axs[1].semilogx(wd, ph_t, '-', color=C_TEST, lw=0.9)
    axs[1].semilogx(w, ph_m, 'o-', color=C_MOD, ms=2.2, lw=0.8)
    axs[0].set_ylabel(r'$|p/\delta_a|$, dB (rad/s per %)'); axs[1].set_ylabel('phase, deg'); axs[1].set_xlabel('frequency, rad/s')
    axs[0].legend(frameon=False, loc='lower left')
    for i, ax in enumerate(axs):
        tag(ax, 'ab'[i])
    fig.tight_layout(h_pad=0.4)
    save(fig, 'freq')


def doublets(rows):
    title = dict(lon='longitudinal', lat='lateral', ped='pedal', col='collective')
    fig, axs = plt.subplots(3, 4, figsize=(W2, 4.3), sharex='col')
    for i, (run, ax, pt, pm, rms, tr, test, mod) in enumerate(rows):
        c, rr = i // 3, i % 3
        a = axs[rr, c]
        a.plot(tr, np.degrees(test), '-', color=C_TEST, lw=0.9, label='flight')
        a.plot(tr, np.degrees(mod), '-', color=C_MOD, lw=0.9, label='model')
        if rr == 0:
            a.set_title(title[run] + ' doublet')
        if c == 0:
            a.set_ylabel('$%s$, deg/s' % ax)
        if rr == 2:
            a.set_xlabel('time, s')
        a.axvspan(10, tr[-1], color='0.93', lw=0)
    axs[0, 0].legend(frameon=False, loc='lower left')
    fig.tight_layout(w_pad=0.5, h_pad=0.3)
    save(fig, 'doublets')


def engine():
    e = res('engine')
    D = json.load(open(os.path.join(FDM, 'data', 'hg_fig6.json')))
    t = np.array(D['t'])
    m = e['m']
    panels = [('rotor_speed_rads', 'rotor speed, rad/s', 1), ('shaft_torque_nm', 'shaft torque, kN m', 1e-3),
              ('coll_lag_rad', r'collective lag, $10^3\times$', 1e3), ('vert_acc_ms2', r'vertical specific force, m/s$^2$', 1),
              ('yaw_rate_rads', 'yaw rate, deg/s', 180 / math.pi)]
    fig, axs = plt.subplots(2, 3, figsize=(W2, 3.6), sharex=True)
    a = axs[0, 0]
    a.plot(t, D['collective_pct'], '-', color=C_TEST, lw=0.9)
    a.set_ylabel('collective, %'); tag(a, 'a')
    for i, (ax, (k, lab, sc)) in enumerate(zip(list(axs.flat)[1:], panels)):
        y = np.array(D[k]); z = np.array(m[k])
        ax.plot(t, sc * y, '-', color=C_TEST, lw=0.9, label='flight test')
        ax.plot(t, sc * (z - z[0] + y[0]), '-', color=C_MOD, lw=0.9, label='model (increment)')
        ax.set_ylabel(lab); tag(ax, 'bcdef'[i])
        if k == 'yaw_rate_rads':
            ax.axvspan(6, t[-1], color='0.92', lw=0)
    h, l = axs[0, 1].get_legend_handles_labels()
    fig.legend(h, l, frameon=False, loc='upper center', ncol=2, bbox_to_anchor=(0.5, 1.03))
    for ax in axs[1]:
        ax.set_xlabel('time, s')
    fig.tight_layout(w_pad=0.8, h_pad=0.4)
    save(fig, 'engine')


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    import mktables
    trim_dlr(); trim_mbb(); hover(); blades(); freq(); engine()
    doublets(mktables.doublets())
    print('figures in', OUT)
