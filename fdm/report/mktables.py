#!/usr/bin/env python3
"""The report's tables (tab/tab_*.tex) and the numbers quoted in its text (tab/numbers.tex, as \\N... macros), from
report/results/*.json (mkresults.py) and the data they are compared with."""
import os, sys, json, math, re
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
import bo105data as U
import engines250 as E
from data import flight_trims as FT, ar280_bo105 as AR, nasa_hover as NH
TAB = os.path.join(HERE, 'tab')
NUM = {}


def R(name):
    return json.load(open(os.path.join(HERE, 'results', name + '.json')))


def num(name, value, fmt='%.1f'):
    assert re.fullmatch('[A-Za-z]+', name), name
    s = (fmt % value) if not isinstance(value, str) else value
    NUM[name] = '$-$' + s[1:] if s.startswith('-') else s


def write(name, body):
    open(os.path.join(TAB, 'tab_%s.tex' % name), 'w').write(body)


def interp(data, V):
    xs, ys = zip(*data)
    return float(np.interp(V, xs, ys)) if xs[0] - 3 <= V <= xs[-1] + 3 else float('nan')


def f(x, fmt='%+.2f'):
    return '--' if x is None or (isinstance(x, float) and math.isnan(x)) else (fmt % x).replace('-', '$-$')


def tabular(cols, head, rows, foot=''):
    return ('\\begin{tabular}{@{}%s@{}}\n\\toprule\n%s \\\\\n\\midrule\n' % (cols, head) +
            ''.join(r + ' \\\\\n' for r in rows) + foot + '\\bottomrule\n\\end{tabular}\n')


# ---------------------------------------------------------------- the generated model
def model_table():
    root = os.path.join(FDM, 'root', 'aircraft', 'bo105', 'Systems')
    files = [('bo105-aero.xml', 'the aerodynamic functions: rotor segments, wake, fuselage, stabiliser, tail rotor; the six force and moment axes', 'function'),
             ('bo105-rotor.xml', 'blade modes, rotor azimuth, Peters--He wake states, tail rotor teeter and inflow, torsion', 'fcs'),
             ('bo105-drive.xml', 'engines, fuel control and governors, rotor speed, freewheels, rotor brake, fuel', 'fcs'),
             ('bo105-controls.xml', 'pilot controls, rigging, boost servos', 'fcs'),
             ('bo105-outputs.xml', 'properties for the 3-D model and the instruments', 'fcs')]
    rows, nfun, nfcs = [], 0, 0
    for fn, what, kind in files:
        s = open(os.path.join(root, fn)).read()
        if kind == 'function':
            n = len(re.findall(r'<function name=', s)); nfun += n
        else:
            n = len(re.findall(r'<(?:fcs_function|pure_gain|summer|lag_filter|integrator|switch|kinematic|actuator|'
                               r'lead_lag_filter|second_order_filter|washout_filter|deadband|aerosurface_scale|pid|'
                               r'sensor|distributor|linear_actuator|accelerometer|gyro|magnetometer)\b', s)); nfcs += n
        rows.append('\\texttt{%s} & %s & %d' % (fn.replace('bo105-', '').replace('.xml', ''), what, n))
    num('nFunc', nfun, '%d'); num('nFcs', nfcs, '%d')
    write('model', '\\begin{tabularx}{\\columnwidth}{@{}lXr@{}}\n\\toprule\nFile & Content & n \\\\\n\\midrule\n' +
          ''.join(r + ' \\\\\n' for r in rows) + '\\bottomrule\n\\end{tabularx}\n')


def params_table():
    md = R('modes')
    rows = [('Gross weight (default, DLR S123)', '2200\\,kg (4850\\,lb)', 'AR'),
            ('Main rotor radius, blades, chord', '%.2f\\,ft (4.91\\,m), %d, %.3f\\,ft' % (U.R, U.NB, U.CHORD), 'TR'),
            ('Solidity, twist, precone', '%.3f, $%.0f^\\circ$, $%.1f^\\circ$' % (U.NB * U.CHORD / (math.pi * U.R), U.TWIST, math.degrees(U.PRECONE)), 'TR, PT'),
            ('Rotor speed, 100\\,\\%', '%.1f\\,rad/s (424\\,rpm)' % U.OMEGA, 'EA, HG'),
            ('Shaft tilt, control phasing', '$%.0f^\\circ$ forward, $%.0f^\\circ$' % (math.degrees(U.SHAFT_TILT), -U.SWASH_PHASE), 'TR'),
            ('Section; segments', 'NACA 23012 (C-81); %d, from 0.2\\,$R$' % U.NSEG, 'TR'),
            ('Blade modes, per rev', 'flap %.2f, lag %.2f, torsion %.2f' % (md['model']['flap'], md['model']['lag'], md['model']['torsion']), 'TR, PT'),
            ('Blade weight, flap inertia', '115.0\\,lb, 162.1\\,slug\\,ft$^2$', 'TR'),
            ('Tail rotor: radius, blades, chord', '%.3f\\,ft, %d (teetering), %.3f\\,ft' % (U.R_TR, U.NB_TR, U.CHORD_TR), 'TR, AR'),
            ('Tail rotor: gear ratio, $\\delta_3$', '%.2f, $%.0f^\\circ$' % (U.GEAR_TR, math.degrees(U.DELTA3_TR)), 'TR, HF'),
            ('Tail rotor: section; segments', 'NACA 0012 (C-81 equations); %d, from 0.2\\,$R$' % U.NSEG_TR, 'TR'),
            ('Stabiliser area, aspect ratio', '%.2f\\,ft$^2$, %.2f, end plates' % (U.S_H, U.AR_H), 'TR, EA'),
            ('Hub, tail rotor hub (FS, BL, WL)', '(%.1f, 0, %.1f), (%.0f, %.1f, %.1f)\\,in' % (U.FS_MR, U.WL_MR, U.FS_TR, U.BL_TR, U.WL_TR), 'TR, HF'),
            ('Default CG (FS, WL)', '(%.1f, %.1f)\\,in' % (U.FS_CG, U.WL_CG), 'HF'),
            ('Inertias $I_x, I_y, I_z, I_{xz}$', '1433, 4973, 4099, 660\\,kg\\,m$^2$', 'AR'),
            ('Engines', '2 $\\times$ Allison 250-C20B, 298.8\\,kW at 100\\,\\% torque', 'EA'),
            ('Drive: rotor inertia', '%.0f\\,slug\\,ft$^2$ (blades %.0f)' % (E.J_ROTOR, E.J_BLADES), 'TR, est.'),
            ('Simulation rate', '960\\,Hz', '')]
    write('params', '\\begin{tabularx}{\\columnwidth}{@{}>{\\raggedright\\arraybackslash}X>{\\raggedright\\arraybackslash}p{3.55cm}l@{}}\n'
          '\\toprule\nQuantity & Value & Source \\\\\n\\midrule\n' + ''.join('%s & %s & %s \\\\\n' % r for r in rows) +
          '\\bottomrule\n\\end{tabularx}\n')


# ---------------------------------------------------------------- level-flight trims
def trims():
    dl = R('trims_dlr')
    rows, d = [], {k: [] for k in ('th0', 'th1s', 'th1c', 'thTR', 'pitch', 'tq')}
    for r in dl:
        V = r['V']
        e = dict(th0=r['th0'] - interp(FT.DLR_THETA0, V), th1s=r['th1s'] - interp(FT.DLR_THETA1S, V),
                 th1c=r['th1c'] - interp(FT.DLR_THETA1C, V), thTR=r['thTR'] - interp(FT.DLR_THETA0T, V),
                 pitch=r['pitch'] - interp(FT.DLR_PITCH, V))
        t = interp(FT.DLR_TORQUE_KNM, V)
        e['tq'] = 100 * (r['q_knm'] / t - 1) if not math.isnan(t) else float('nan')
        if V > 100:
            continue
        for k in d:
            if not math.isnan(e[k]):
                d[k].append((V, e[k]))
        rows.append('%.0f & %s & %s & %s & %s & %s & %s' % (V, f(e['th0']), f(e['th1s']), f(e['th1c']), f(e['thTR']), f(e['pitch']),
                                                          f(e['tq'], '%+.0f')))
    write('trim_dlr', tabular('rrrrrrr', 'kt & $\\theta_0$ & $\\theta_{1s}$ & $\\theta_{1c}$ & $\\theta_{0T}$ & $\\theta$ & $Q$, \\%', rows))
    rng = lambda k, lo=0: (min(v for V, v in d[k] if V >= lo), max(v for V, v in d[k] if V >= lo))
    num('DthzMin', rng('th0')[0], '%.1f'); num('DthzMax', rng('th0')[1], '%.1f')
    num('DlonMin', -rng('th1s')[1], '%.1f'); num('DlonMax', -rng('th1s')[0], '%.1f')       # forward of the test, deg
    num('DlonMaxHi', -min(v for V, v in d['th1s'] if V >= 60), '%.1f')
    num('DlatMin', rng('th1c')[0], '%.1f'); num('DlatMax', rng('th1c')[1], '%.1f')
    num('DtrMax', rng('thTR')[1], '%.1f'); num('DtrHi', max(abs(v) for V, v in d['thTR'] if V >= 60), '%.1f')
    num('DtrMaxV', max(d['thTR'], key=lambda p: p[1])[0], '%.0f')
    num('DpitchMax', max(abs(v) for V, v in d['pitch']), '%.1f')
    num('DtqMin', rng('tq')[0], '%.0f'); num('DtqMax', rng('tq')[1], '%.0f')
    num('DtqLow', max(abs(v) for V, v in d['tq'] if V <= 60), '%.0f')
    num('DrollMin', min(r['roll'] for r in dl if r['V'] <= 100), '%.1f'); num('DrollMax', max(r['roll'] for r in dl if r['V'] <= 100), '%.1f')
    # the tail rotor's pitch with either model: in the hover (against the test's at 2.3 kt), through the transition, at speed
    for tag, rows_ in (('Be', dl), ('Ba', R('trims_dlr_bailey'))):
        e = {r['V']: r['thTR'] - interp(FT.DLR_THETA0T, r['V']) for r in rows_ if r['V'] <= 100}
        num('Tr%sHover' % tag, rows_[0]['thTR'] - FT.DLR_THETA0T[0][1], '%+.1f')
        num('Tr%sTransMin' % tag, min(e[V] for V in (20, 30, 40)), '%.1f'); num('Tr%sTransMax' % tag, max(e[V] for V in (20, 30, 40)), '%.1f')
        num('Tr%sHi' % tag, max(abs(e[V]) for V in e if V >= 60), '%.1f')
        num('Tr%sHoverPow' % tag, rows_[0]['tr_hp'], '%.0f')
    num('TrHoverTest', FT.DLR_THETA0T[0][1], '%.1f'); num('TrBeHoverDeg', dl[0]['thTR'], '%.1f')
    # the fin blockage: its force against the tail rotor's thrust, and the pitch it adds (against the model without it)
    pct = {r['V']: -100 * r['fin'] / r['Ttr'] for r in dl}
    num('FinHoverLb', -dl[0]['fin'], '%.0f'); num('FinHoverPct', pct[0], '%.1f')
    vpk = max((V for V in pct if V <= 100), key=lambda V: pct[V])
    num('FinPeakPct', pct[vpk], '%.0f'); num('FinPeakV', vpk, '%.0f'); num('FinPeakLb', -dl[[r['V'] for r in dl].index(vpk)]['fin'], '%.0f')
    num('FinHiPct', max(pct[V] for V in pct if 60 <= V <= 100), '%.0f')
    dth = {a['V']: b['thTR'] - a['thTR'] for a, b in zip(R('trims_dlr_nofin'), dl)}
    num('FinDthHover', dth[0], '%.2f'); num('FinDthMax', max(dth.values()), '%.1f')
    num('FinDthMaxV', max(dth, key=dth.get), '%.0f'); num('FinDthHi', max(dth[V] for V in dth if 60 <= V <= 100), '%.2f')
    h0 = dl[0]
    num('DhoverTq', h0['q_knm'], '%.2f')
    num('DhoverTqTest', FT.DLR_TORQUE_KNM[0][1], '%.2f'); num('DhoverTqTestV', FT.DLR_TORQUE_KNM[0][0], '%.0f')
    mb = R('trims_mbb')
    rows, d = [], {k: [] for k in ('coll', 'B1s', 'A1s', 'pitch')}
    for r in mb:
        V = r['V']
        e = dict(coll=r['th0'] - 5.6 - interp(FT.MBB_COLL07, V), B1s=r['B1s'] - interp(FT.MBB_B1S, V), A1s=r['A1s'] - interp(FT.MBB_A1S, V),
                 pitch=r['pitch'] - interp(FT.MBB_PITCH, V))
        for k in d:
            d[k].append((V, e[k]))
        rows.append('%.0f & %s & %s & %s & %s & %.0f' % (V, f(e['coll']), f(e['B1s']), f(e['A1s']), f(e['pitch']), r['tot_hp']))
    write('trim_mbb', tabular('rrrrrr', 'kt & $\\theta_{.7}$ & $B_{1s}$ & $A_{1s}$ & $\\theta$ & hp', rows))
    for k, n in (('coll', 'Mcoll'), ('B1s', 'Mlon'), ('A1s', 'Mlat'), ('pitch', 'Mpitch')):
        num(n, max(abs(v) for V, v in d[k]), '%.1f')
    num('MlonHi', max(abs(v) for V, v in d['B1s'] if V >= 55), '%.1f')
    w = [(r['V'], r['tot_hp']) for r in mb]
    num('MpowMin', min(w, key=lambda p: p[1])[1], '%.0f'); num('MpowMinV', min(w, key=lambda p: p[1])[0], '%.0f')


# ---------------------------------------------------------------- the isolated rotor in hover
def hover():
    h = R('hover')
    s = h['sigma']
    fm = lambda ct, cp: (ct * s) ** 1.5 / (math.sqrt(2) * cp * s) if ct > 0 else float('nan')
    rows = []
    m_ct = [r['ct'] for r in h['rows']]; m_cp = [r['cp'] for r in h['rows']]; m_th = [r['th07'] for r in h['rows']]
    dcp, dth = [], []
    for r in h['rows']:
        ct_t, cp_t = r['ct_test'], r['cp_test']
        cp_m = float(np.interp(ct_t, m_ct, m_cp)) if ct_t > 0.03 else float('nan')     # the model's power at the test's thrust
        th_m = float(np.interp(ct_t, m_ct, m_th)) if ct_t > 0.03 else float('nan')
        if not math.isnan(cp_m):
            dcp.append(100 * (cp_m / cp_t - 1)); dth.append(r['th07'] - th_m)
        rows.append('%.1f & %s & %s & %s & %s & %s & %s' % (r['th07'], f(max(r['ct_test'], 0.0), '%.4f'), f(r['ct'], '%.4f'), f(cp_m * 1000, '%.2f'), f(cp_t * 1000, '%.2f'),
                                                             f(fm(ct_t, cp_m), '%.3f'), f(fm(ct_t, cp_t), '%.3f')))
    write('hover', tabular('rrrrrrr', '$\\theta_{.7}$ & \\multicolumn{2}{c}{$C_T/\\sigma$} & \\multicolumn{2}{c}{$10^3 C_P/\\sigma$ at test $C_T$} & '
                           '\\multicolumn{2}{c}{FM} \\\\\n deg & test & model & model & test & model & test', rows))
    num('HcpMin', -max(dcp), '%.0f'); num('HcpMax', -min(dcp), '%.0f')
    num('HthMin', min(dth), '%.1f'); num('HthMax', max(dth), '%.1f')
    fms = [fm(r['ct'], r['cp']) for r in h['rows']]
    num('HfmMod', max(fms), '%.2f'); num('HfmTest', max(fm(r['ct_test'], r['cp_test']) for r in h['rows'] if r['ct_test'] > 0), '%.2f')
    tors = [(r['ct'], r['torsion']) for r in h['rows'] if r['ct'] > 0.02]
    num('HtorsMod', np.interp(0.07, *zip(*tors)), '%.0f'); num('HtorsTest', np.interp(0.07, *zip(*h['torsion_test'])), '%.0f')


# ---------------------------------------------------------------- blade dynamics
def blades():
    md = R('modes')
    t4, pt, y = md['table4'], md['pt'], md['y71']
    rows = [('Flap 1', '%.3f' % t4['flap'][0], '%.3f' % md['model']['flap'], '%.3f' % y['flap1'], '%.2f' % pt['flap1']),
            ('Lag 1', '%.3f' % t4['lag'][0], '%.3f' % md['model']['lag'], '%.3f' % y['lag1'], '%.2f' % pt['lag1']),
            ('Torsion 1', '%.3f' % t4['torsion'][0], '%.3f' % md['model']['torsion'], '%.3f' % y['torsion1'], '%.2f' % pt['torsion1']),
            ('Flap 2', '%.3f' % t4['flap'][1], '--', '%.3f' % y['flap2'], '%.2f' % pt['flap2']),
            ('Lag 2', '%.3f' % t4['lag'][1], '--', '%.3f' % y['lag2'], '%.2f' % pt['lag2']),
            ('Flap 3', '%.3f' % t4['flap'][2], '--', '%.3f' % y['flap3'], '%.2f' % pt['flap3'])]
    write('blades', tabular('lrrrr', 'Mode & beam & model & Y-71~\\cite{tr} & analysis~\\cite{pt}', ['%s & %s & %s & %s & %s' % r for r in rows]))
    num('BflapMod', md['model']['flap'], '%.2f'); num('BlagMod', md['model']['lag'], '%.2f'); num('BtorsMod', md['model']['torsion'], '%.2f')
    num('BlagBeam', t4['lag'][0], '%.2f'); num('BlagScale', md['lag_root_scale'], '%.2f')
    lg = R('lagdamp')
    test = lambda c: float(np.interp(c, *zip(*lg['test'])))
    rows = ['%.4f & %s & %s & %.3f' % (r['ct_sigma'], f(r['decay'], '%.2f'), f(test(r['ct_sigma']), '%.2f'), r['nu']) for r in lg['rows']]
    write('lag', tabular('rrrr', '$C_T/\\sigma$ & \\multicolumn{2}{c}{decay, s$^{-1}$} & $\\nu_\\zeta$ \\\\\n & model & test fit & /rev', rows))
    near = min(lg['rows'], key=lambda r: abs(r['ct_sigma'] - 0.07))
    num('LagMod', near['decay'], '%.2f'); num('LagTest', test(near['ct_sigma']), '%.2f'); num('LagCt', near['ct_sigma'], '%.3f')
    hi = lg['rows'][-1]
    num('LagModHi', hi['decay'], '%.2f'); num('LagTestHi', test(hi['ct_sigma']), '%.2f'); num('LagCtHi', hi['ct_sigma'], '%.3f')
    lo = lg['rows'][0]
    num('LagModLo', lo['decay'], '%.2f'); num('LagTestLo', test(lo['ct_sigma']), '%.2f'); num('LagCtLo', lo['ct_sigma'], '%.3f')
    num('LagNu', np.mean([r['nu'] for r in lg['rows']]), '%.2f')


# ---------------------------------------------------------------- derivatives and modes at 80 kt
SELECT = ['Xu', 'Xw', 'Xq', 'Xlon', 'Zu', 'Zw', 'Zq', 'Zcol', 'Zlon', 'Mu', 'Mw', 'Mp', 'Mq', 'Mcol', 'Mlon', 'Mlat',
          'Yv', 'Yp', 'Yr', 'Ylat', 'Yped', 'Lv', 'Lp', 'Lq', 'Lr', 'Llat', 'Llon', 'Lcol', 'Lped', 'Nv', 'Np', 'Nq', 'Nr',
          'Nped', 'Nlat', 'Ncol']


def derivs():
    dv = R('derivs')
    D = dv['D']
    inside, n, rows = 0, 0, []
    for k in SELECT:
        vals = [v for v in AR.DERIV[k] if v is not None]
        lo, hi = min(vals), max(vals)
        m = D[k]
        ok = lo <= m <= hi
        mark = '' if ok else ('$\\downarrow$' if m < lo else '$\\uparrow$')
        dlr = AR.DERIV[k][2]
        tex = '$%s_{%s}$' % (k[0], {'col': '\\delta_c', 'lon': '\\delta_e', 'lat': '\\delta_a', 'ped': '\\delta_r'}.get(k[1:], k[1:]))
        rows.append('%s & %s%s & %s & %s .. %s' % (tex, f(m, '%.3f'), mark, f(dlr, '%.3f'), f(lo, '%.3f'), f(hi, '%.3f')))
    for k, vals in AR.DERIV.items():
        vv = [v for v in vals if v is not None]
        if len(vv) >= 4:
            n += 1
            inside += min(vv) <= D[k] <= max(vv)
    num('DerIn', inside, '%d'); num('DerN', n, '%d')
    half = (len(rows) + 1) // 2
    L, Rr = rows[:half], rows[half:] + [''] * (2 * half - len(rows))
    body = ['%s & %s' % (a, b if b else '& & &') for a, b in zip(L, Rr)]
    write('derivs', tabular('lrrr@{\\hspace{8mm}}lrrr', '& model & DLR & six identifications & & model & DLR & six identifications', body))
    for k in ('Mq', 'Lp', 'Nr', 'Mw', 'Lv', 'Nv', 'Mlon', 'Llat', 'Nped', 'Zw', 'Zcol', 'Np', 'Mp', 'Llon', 'Lcol', 'Mcol'):
        num('d' + k, D[k], '%.3f')
        vals = [v for v in AR.DERIV[k] if v is not None]
        num('d' + k + 'Lo', min(vals), '%.3f'); num('d' + k + 'Hi', max(vals), '%.3f'); num('d' + k + 'Dlr', AR.DERIV[k][2], '%.3f')
    # the steady rate per % of cyclic, control power over damping
    dl = lambda k: AR.DERIV[k][2]
    num('SsRoll', -D['Llat'] / D['Lp'] * 100, '%.1f'); num('SsRollDlr', -dl('Llat') / dl('Lp') * 100, '%.1f')
    num('SsPitch', -D['Mlon'] / D['Mq'] * 100, '%.1f'); num('SsPitchDlr', -dl('Mlon') / dl('Mq') * 100, '%.1f')
    # modes
    ev = [complex(*e) for e in dv['eig']]
    osc = sorted([e for e in ev if e.imag > 1e-6], key=lambda e: -abs(e))
    real = sorted([e.real for e in ev if abs(e.imag) <= 1e-6])
    zw = lambda e: (-e.real / abs(e), abs(e))
    names = {'phugoid': None, 'dutch roll': None}
    for e in osc:
        z, w = zw(e)
        if w < 1.0:
            names['phugoid'] = (z, w)
        elif w < 5:
            names['dutch roll'] = (z, w)
        else:
            names['roll/pitch'] = (z, w)
    fm = lambda x: ('[%.2f, %.2f]' % tuple(x)).replace('-', '$-$') if isinstance(x, (tuple, list)) else ('%.2f' % x).replace('-', '$-$')
    rows = []
    for mode, model in (('phugoid', names['phugoid']), ('dutch roll', names['dutch roll'])):
        rows.append('%s & %s & %s' % (mode, fm(model), ', '.join(fm(v) for v in AR.MODES[mode])))
    rr = [-x for x in real]
    rows.append('roll / pitch & %s & %s' % ((fm(names['roll/pitch']) if 'roll/pitch' in names else ', '.join('%.2f' % x for x in rr if x > 1.5)),
                                            'roll ' + ', '.join(fm(v) for v in AR.MODES['roll'] if v is not None) +
                                            '; pitch ' + ', '.join(fm(v) for v in AR.MODES['pitch 1'] if v is not None)))
    p2 = [x for x in rr if 0.1 < x < 1.5]; sp = [x for x in rr if abs(x) <= 0.1]
    rows.append('pitch (heave) & %s & %s' % (', '.join('%.2f' % x for x in p2), ', '.join(fm(v) for v in AR.MODES['pitch 2'])))
    rows.append('spiral & %s & %s' % (', '.join(fm(x) for x in sp), ', '.join(fm(v) for v in AR.MODES['spiral'])))
    write('modes', '\\begin{tabularx}{\\columnwidth}{@{}lrX@{}}\n\\toprule\nMode & model & identified (AFDD, CERT, DLR, Glasgow, NAE, NLR) \\\\\n'
          '\\midrule\n' + ''.join(r + ' \\\\\n' for r in rows) + '\\bottomrule\n\\end{tabularx}\n')
    num('MdrZeta', names['dutch roll'][0], '%.2f'); num('MdrW', names['dutch roll'][1], '%.2f')
    num('MdrPct', 100 * (names['dutch roll'][1] / max(v[1] for v in AR.MODES['dutch roll']) - 1), '%.0f')
    num('MphZeta', names['phugoid'][0], '%.2f'); num('MphW', names['phugoid'][1], '%.2f')
    if 'roll/pitch' in names:
        num('MrpZeta', names['roll/pitch'][0], '%.2f'); num('MrpW', names['roll/pitch'][1], '%.2f')
    num('MpTwo', p2[0] if p2 else float('nan'), '%.2f')
    num('Dtheta', dv['trim']['theta'], '%.1f'); num('Dphi', dv['trim']['phi'], '%.1f')


# ---------------------------------------------------------------- frequency response, doublets, rotor speed
def tischler(w):
    s = 1j * np.asarray(w)
    q = lambda z, wn: s * s + 2 * z * wn * s + wn * wn
    return s * 2.457 * q(0.447, 3.237) * q(0.045, 14.94) * np.exp(-0.017 * s) / (s * q(0.317, 2.858) * q(0.021, 14.98) * q(0.450, 13.142))


def freq():
    fr = R('freqresp')
    w = np.array(fr['w']); Hm = np.array([complex(*h) for h in fr['H']]); Ht = tischler(w)
    m = w <= 25
    dmag = 20 * np.log10(np.abs(Hm[m])) - 20 * np.log10(np.abs(Ht[m]))
    dph = np.degrees(np.unwrap(np.angle(Hm[m]))) - np.degrees(np.unwrap(np.angle(Ht[m])))
    lo = w[m] <= 12
    num('FmagRms', np.sqrt(np.mean(dmag[lo] ** 2)), '%.1f'); num('FmagMax', np.max(np.abs(dmag[lo])), '%.1f')
    num('FphRms', np.sqrt(np.mean(dph[lo] ** 2)), '%.0f'); num('FphMax', np.max(np.abs(dph[lo])), '%.0f')
    hi = w[m] > 15
    num('FphHi', np.mean(dph[hi]), '%.0f')
    num('FgainLow', 20 * np.log10(np.mean(np.abs(Hm[(w >= 1) & (w <= 2)]))), '%.1f')
    num('FgainLowT', 20 * np.log10(np.mean(np.abs(Ht[(w >= 1) & (w <= 2)]))), '%.1f')


def doublets():
    import doublets as DB
    runs = []
    for k in range(4):
        d = R('doublet%d' % k)
        runs.append((d['k'], np.array(d['rec']), d['res']))
    rows = DB.compare(runs)
    out, onaxis = [], []
    title = dict(lon='longitudinal', lat='lateral', ped='pedal', col='collective')
    on = dict(lon='q', lat='p', ped='r', col='q')
    tab = {}
    for run, ax, pt, pm, rms, *_ in rows:
        tab.setdefault(run, {})[ax] = (rms, pt, pm)
        if ax == on[run]:
            onaxis.append(rms)
    for run in ('lon', 'lat', 'ped', 'col'):
        r = tab[run]
        out.append('%s & %s & %s & %s' % (title[run], *[('\\textbf{%.3f}' if ax == on[run] else '%.3f') % r[ax][0] for ax in 'pqr']))
    write('doublets', tabular('lrrr', 'Doublet & $p$ & $q$ & $r$', out))
    num('DbOn', np.mean(onaxis), '%.3f')
    num('DbOnDeg', math.degrees(np.mean(onaxis)), '%.1f')
    allr = [v[0] for r in tab.values() for v in r.values()]
    num('DbAll', math.degrees(np.mean(allr)), '%.1f')
    num('DbPedR', math.degrees(tab['ped']['r'][0]), '%.1f')
    num('DbPedPk', math.degrees(tab['ped']['r'][2]), '%.0f'); num('DbPedPkT', math.degrees(tab['ped']['r'][1]), '%.0f')
    num('DbLonPk', math.degrees(tab['lon']['q'][2]), '%.1f'); num('DbLonPkT', math.degrees(tab['lon']['q'][1]), '%.1f')
    return rows


def engine():
    e = R('engine')
    labels = ['rotor speed, rad/s', 'shaft torque, kN\\,m', 'lag, tip/$R\\times10^3$', 'yaw rate, deg/s (6\\,s)',
              'vert.\\ sp.\\ force, m/s$^2$']
    scale = [1, 1e-3, 1e3, 180 / math.pi, 1]
    rows = []
    for lab, sc, row in zip(labels, scale, e['table']):
        _, y0, z0, tmin, mmin, tmax, mmax, rms = row
        rows.append('%s & %s & %s & %s & %s & %.3g' % (lab, f(tmin * sc, '%+.2f'), f(mmin * sc, '%+.2f'), f(tmax * sc, '%+.2f'), f(mmax * sc, '%+.2f'), rms * sc))
    write('engine', tabular('lrrrrr', '& \\multicolumn{2}{c}{lowest} & \\multicolumn{2}{c}{highest} & rms \\\\\nIncrement & test & model & test & model & error', rows))
    t = e['table']
    num('EnrMinT', t[0][3], '%.2f'); num('EnrMinM', t[0][4], '%.2f'); num('EnrMaxT', t[0][5], '%.2f'); num('EnrMaxM', t[0][6], '%.2f')
    num('EnrRms', t[0][7], '%.2f'); num('EnrPct', 100 * t[0][7] / U.OMEGA, '%.2f')
    num('EtqMinT', t[1][3] / 1000, '%.1f'); num('EtqMinM', t[1][4] / 1000, '%.1f'); num('EtqMaxT', t[1][5] / 1000, '%.1f'); num('EtqMaxM', t[1][6] / 1000, '%.1f')
    num('EazRms', t[4][7], '%.2f')


if __name__ == '__main__':
    os.makedirs(TAB, exist_ok=True)
    model_table(); params_table(); trims(); hover(); blades(); derivs(); freq(); doublets(); engine()
    open(os.path.join(TAB, 'numbers.tex'), 'w').write(''.join('\\newcommand{\\N%s}{%s}\n' % (k, v) for k, v in sorted(NUM.items())))
    print('wrote %d tables and %d numbers' % (len([n for n in os.listdir(TAB) if n.startswith('tab_')]), len(NUM)))
