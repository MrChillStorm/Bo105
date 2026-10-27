#!/usr/bin/env python3
"""Rotating modes of the Bo 105 main rotor blade from its mass and stiffness distribution [TR-75-50 table 4].

The blade is a cantilever from the shaft (Boeing's Y-71 stations run from r = 0.05 in to the tip), with the masses
lumped at the stations and the stiffness constant between them. Flap and lag bending are Euler-Bernoulli beam
elements (cubic Hermite) with the centrifugal tension's geometric stiffness, and for lag the centrifugal softening;
torsion has linear elements on the control system's stiffness at the root, with the propeller moment of the blade's
pitch inertia. Flap, lag and torsion are uncoupled (no pitch, CG or shear centre offsets).

  python3 blademodes.py           frequencies against the published ones, and the generalized properties

modes() returns the first flap, lag and torsion modes and the integrals the flight model needs, in ft, slug, s.
"""
import math, sys
import numpy as np
from scipy.linalg import eigh
import bo105data as U

IN = 1 / 12.0      # in -> ft
SLUG = 12.0        # lb s^2/in -> slug


def blade(lag_root_scale=1.0, flap_root_scale=1.0, gj_scale=None):
    """nodes root to tip: r (in), lumped mass (lb s^2/in), pitch inertia (lb in s^2); element stiffness (lb in^2)"""
    r = np.array(U.Y71_R[::-1])
    m = np.array(U.Y71_M[::-1])
    ix = np.array(U.Y71_IX[::-1])
    # the stiffness listed at a station is that of the segment to the next station outboard: in root-to-tip order,
    # element j (node j to j + 1) carries node j's value
    eif = np.array([x * 1e6 for x in U.Y71_EIF[::-1][:-1]])
    eil = np.array([x * 1e6 for x in U.Y71_EIL[::-1][:-1]])
    gj = np.array([x * 1e6 for x in U.Y71_GJ[::-1][:-1]]) * (torsion_scale() if gj_scale is None else gj_scale)
    # the hub region (r/R < 0.06, the two innermost elements) is where TM-103867 tuned the stiffness to MBB's
    # measured frequencies
    hub = r[1:] <= 0.06 * U.R * 12 + 1e-6
    eil = np.where(hub, eil * lag_root_scale, eil)
    eif = np.where(hub, eif * flap_root_scale, eif)
    return r, m, ix, eif, eil, gj


_GJ_SCALE = None


def torsion_scale():
    """scale of the blade's torsional stiffness that gives TORSION_TARGET /rev at 425 rpm (1 without a target)"""
    global _GJ_SCALE
    if _GJ_SCALE is None:
        if not U.TORSION_TARGET:
            _GJ_SCALE = 1.0
        else:
            om = 425 * math.pi / 30
            lo, hi = 0.2, 10.0
            for _ in range(60):
                mid = math.sqrt(lo * hi)
                r, m, ix, eif, eil, gj = blade(gj_scale=mid)
                Kt, Mt = torsion(r, ix, gj, U.K_CONTROL * 12)
                f = solve(Kt + om ** 2 * Mt, Mt, 1)[0][0] / om
                lo, hi = (mid, hi) if f < U.TORSION_TARGET else (lo, mid)
            _GJ_SCALE = math.sqrt(lo * hi)
    return _GJ_SCALE


def tension(r, m, omega):
    """centrifugal tension in each element (lb): the lumped masses outboard of it"""
    mr = m * r
    return np.array([omega ** 2 * mr[j + 1:].sum() for j in range(len(r) - 1)])


def bending(r, m, ei, omega, inplane):
    """stiffness (structural, geometric per unit tension) and mass matrices, clamped root; DOFs (w, w') per node"""
    n = len(r)
    K = np.zeros((2 * n, 2 * n)); Kg = np.zeros((2 * n, 2 * n)); Mm = np.zeros((2 * n, 2 * n))
    T = tension(r, m, 1.0)
    for j in range(n - 1):
        L = r[j + 1] - r[j]
        ke = ei[j] / L ** 3 * np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L * L, -6 * L, 2 * L * L],
                                        [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L * L, -6 * L, 4 * L * L]])
        kg = T[j] / (30 * L) * np.array([[36, 3 * L, -36, 3 * L], [3 * L, 4 * L * L, -3 * L, -L * L],
                                         [-36, -3 * L, 36, -3 * L], [3 * L, -L * L, -3 * L, 4 * L * L]])
        idx = [2 * j, 2 * j + 1, 2 * j + 2, 2 * j + 3]
        K[np.ix_(idx, idx)] += ke; Kg[np.ix_(idx, idx)] += kg
    for j in range(n):
        Mm[2 * j, 2 * j] = m[j]
    soft = np.zeros_like(Mm)
    if inplane:
        soft = Mm.copy()          # in plane the centrifugal force also pulls the displaced mass outwards
    free = list(range(2, 2 * n))  # root node clamped
    sub = lambda A: A[np.ix_(free, free)]
    return sub(K), sub(Kg), sub(Mm), sub(soft)


def torsion(r, ix, gj, k_root):
    n = len(r)
    K = np.zeros((n, n)); Mm = np.diag(ix)
    for j in range(n - 1):
        k = gj[j] / (r[j + 1] - r[j])
        K[j:j + 2, j:j + 2] += k * np.array([[1, -1], [-1, 1]])
    K[0, 0] += k_root
    return K, Mm


def solve(K, M, n=3):
    """lowest n modes of K v = w^2 M v with a diagonal M: the massless DOFs (slopes; rotary inertia is zero) are
    condensed out statically, which is exact for them"""
    d = np.diag(M)
    a = np.where(d > 1e-9 * d.max())[0]; b = np.where(d <= 1e-9 * d.max())[0]
    if len(b):
        Kbb_inv_Kba = np.linalg.solve(K[np.ix_(b, b)], K[np.ix_(b, a)])
        Kr = K[np.ix_(a, a)] - K[np.ix_(a, b)] @ Kbb_inv_Kba
    else:
        Kr = K
    w, Va = eigh(Kr, M[np.ix_(a, a)])
    V = np.zeros((len(d), n))
    V[a] = Va[:, :n]
    if len(b):
        V[b] = -Kbb_inv_Kba @ Va[:, :n]
    return np.sqrt(np.maximum(w[:n], 0)), V


def frequencies(omega=U.OMEGA, lag_root_scale=1.0, flap_root_scale=1.0, n=3):
    r, m, ix, eif, eil, gj = blade(lag_root_scale, flap_root_scale)
    Kf, Kgf, Mf, _ = bending(r, m, eif, omega, False)
    Kl, Kgl, Ml, Sl = bending(r, m, eil, omega, True)
    Kt, Mt = torsion(r, ix, gj, U.K_CONTROL * 12)
    wf, Vf = solve(Kf + omega ** 2 * Kgf, Mf, n)
    wl, Vl = solve(Kl + omega ** 2 * (Kgl - Sl), Ml, n)
    wt, Vt = solve(Kt + omega ** 2 * Mt, Mt, n)
    return (wf, Vf), (wl, Vl), (wt, Vt)


def lag_scale(target=U.LAG_TARGET, omega=425 * math.pi / 30):
    """scale of the hub region's in-plane stiffness that gives the target first lag frequency (per rev)"""
    if not target:
        return 1.0
    lo, hi = 0.001, 1.0
    for _ in range(60):
        mid = math.sqrt(lo * hi)
        f = frequencies(omega, mid)[1][0][0] / omega
        lo, hi = (mid, hi) if f < target else (lo, mid)
    return math.sqrt(lo * hi)


def shape(r, V, k, rr):
    """bending mode k at radius rr (in) from nodal (w, w') with the root clamped: value and slope (Hermite)"""
    n = len(r)
    full = np.zeros(2 * n); full[2:] = V[:, k]
    j = min(max(np.searchsorted(r, rr) - 1, 0), n - 2)
    L = r[j + 1] - r[j]; x = (rr - r[j]) / L
    h = [1 - 3 * x * x + 2 * x ** 3, L * (x - 2 * x * x + x ** 3), 3 * x * x - 2 * x ** 3, L * (-x * x + x ** 3)]
    dh = [(-6 * x + 6 * x * x) / L, 1 - 4 * x + 3 * x * x, (6 * x - 6 * x * x) / L, -2 * x + 3 * x * x]
    d = full[2 * j: 2 * j + 4]
    return float(np.dot(h, d)), float(np.dot(dh, d))


def modes(omega=U.OMEGA):
    """First flap, lag (hub stiffness tuned to LAG_TARGET) and torsion modes at the nominal rotor speed, normalized to
    a tip deflection of R (bending: the coordinate is the tip deflection over R, rad-like) and a tip twist of 1 rad, with
    the integrals the blade equations and the hub loads use, in ft, slug, s:
      M q'' + (K + W^2 Kcf - W^2 soft) q = sum(phi F) + ...      (soft: the lag mode's centrifugal softening)"""
    ks = lag_scale_coupled()
    r, m, ix, eif, eil, gj = blade(ks)
    (wf, Vf), (wl, Vl), (wt, Vt) = frequencies(omega, ks)
    Kf, Kgf, Mf, _ = bending(r, m, eif, omega, False)
    Kl, Kgl, Ml, Sl = bending(r, m, eil, omega, True)
    Kt, Mt = torsion(r, ix, gj, U.K_CONTROL * 12)
    Rin = U.R * 12
    mass = m * SLUG                      # slug at the nodes
    rf = r * IN
    out = {'lag_root_scale': ks, 'r_in': r}
    for name, V, K, Kg in (('f', Vf, Kf, Kgf), ('l', Vl, Kl, Kgl)):
        v = V[:, 0] / shape(r, V, 0, Rin)[0] * Rin          # nodal (w, w') in inches for a tip deflection of R
        Vn = v[:, None]
        phi_ft = np.array([shape(r, Vn, 0, x)[0] for x in r]) * IN
        d = dict(V=Vn, phi_ft=phi_ft)
        d['M'] = float((mass * phi_ft ** 2).sum())                   # slug ft^2
        d['K'] = float(v @ K @ v) / 12                                # ft lb (structural)
        d['Kcf'] = float(v @ Kg @ v) / 12                             # slug ft^2: x W^2, centrifugal stiffening
        d['soft'] = d['M'] if name == 'l' else 0.0                    # x W^2, in-plane centrifugal softening
        d['S'] = float((mass * phi_ft).sum())                         # int m phi, slug ft
        d['Sr'] = float((mass * rf * phi_ft).sum())                   # int m r phi, slug ft^2
        out[name] = d
    v = Vt[:, 0] / Vt[-1, 0]
    out['t'] = dict(phi=v, I=float(v @ Mt @ v) / 12, K=float(v @ Kt @ v) / 12, Kprop=float(v @ Mt @ v) / 12)
    out['Mb'] = float(mass.sum()); out['Sr'] = float((mass * rf).sum()); out['Ib'] = float((mass * rf ** 2).sum())
    out['G1'] = float((mass * out['f']['phi_ft'] * out['l']['phi_ft']).sum())
    f, l, t = out['f'], out['l'], out['t']
    out['freq'] = dict(flap=math.sqrt((f['K'] + omega ** 2 * f['Kcf']) / f['M']) / omega,
                       lag=math.sqrt((l['K'] + omega ** 2 * (l['Kcf'] - l['soft'])) / l['M']) / omega,
                       torsion=math.sqrt((t['K'] + omega ** 2 * t['Kprop']) / t['I']) / omega)
    return out


def droop_residual(pcts, omega=U.OMEGA):
    """The blade's static droop under its weight (1 g along the shaft) beyond the first flap mode's, tip deflection / R
    (rad, down -), at the rotor speeds pcts (% of omega): the beam's own solution less the mode's. On the rotating mode's
    shape the blade is about three times too stiff at rest: its tip sags 0.11 m against the beam's 0.35 m, and the 2.5 deg
    of precone (0.21 m at the tip) held the parked blades up; the centrifugal stiffening takes over by about 40 %."""
    r, m, ix, eif, eil, gj = blade()
    K, Kg, Mm, _ = bending(r, m, eif, omega, False)
    f = modes(omega)['f']
    weight = np.zeros(K.shape[0]); weight[0::2] = -np.diag(Mm)[0::2] * U.G0 * 12     # lb at the nodes, down
    out = []
    for pct in pcts:
        W = omega * pct / 100
        beam = np.linalg.solve(K + W ** 2 * Kg, weight)[-2] / (U.R * 12)
        out.append(beam + f['S'] * U.G0 / (f['K'] + W ** 2 * f['Kcf']))
    return out


def phi(md, name, r_ft):
    """mode shape value and slope (per ft) at radius r_ft"""
    w, s = shape(md['r_in'], md[name]['V'], 0, r_ft * 12)
    return w * IN, s



def rotor_constants(seg_r, omega=U.OMEGA):
    """Everything the flight model needs from the modes, ft-slug-s: the generalized properties, the coupling integrals
    and the mode shapes at the aerodynamic segments' radii seg_r (ft)."""
    md = modes(omega)
    r_in, m = md['r_in'], np.array(U.Y71_M[::-1]) * SLUG
    ix = np.array(U.Y71_IX[::-1]) / 12.0                     # slug ft^2
    rf = r_in * IN
    f, l, t = md['f'], md['l'], md['t']
    # foreshortening integral G2 = int m phi_l(r) int_0^r phi_f'^2 on a fine grid (Hermite slopes)
    xs = np.linspace(r_in[0], r_in[-1], 2000)
    slope2 = np.array([shape(r_in, f['V'], 0, x)[1] ** 2 for x in xs])        # (in/in)^2 per q^2 ... tip = R in
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (slope2[1:] + slope2[:-1]) * np.diff(xs))]) * IN   # ft per q^2
    G2 = float(sum(mi * l['phi_ft'][i] * np.interp(r_in[i], xs, cum) for i, mi in enumerate(m)))
    phi_t = t['phi']
    c = U.CHORD
    seg = [dict(r=r, phi_f=phi(md, 'f', r)[0], dphi_f=phi(md, 'f', r)[1], phi_l=phi(md, 'l', r)[0],
                phi_t=float(np.interp(r * 12, r_in, phi_t))) for r in seg_r]
    Kc = coupled_stiffness(md['lag_root_scale'], md)
    return dict(
        Kc=Kc,
        Mf=f['M'], Kf=f['K'], Kcf_f=f['Kcf'], Sf=f['S'], Srf=f['Sr'],
        Ml=l['M'], Kl=l['K'], Kcf_l=l['Kcf'], Sl=l['S'], Srl=l['Sr'],
        It=t['I'], Kt=t['K'], Kprop_t=t['Kprop'],
        Mb=md['Mb'], Sr=md['Sr'], Ib=md['Ib'], G1=md['G1'], G2=G2,
        # torsion: inertial forcing through the CG's offset aft of the elastic axis, and the propeller moment of the
        # blade's rigid pitch (its pitch inertia, lumped)
        T1=float((m * phi_t).sum()), T2=float((m * phi_t * f['phi_ft']).sum()), T3=float((m * rf * phi_t).sum()),
        # centrifugal force on the coned, bent blade at the CG offset: sum m r phi_t phi_f' (slope of the flap mode)
        T4=float(sum(m[i] * rf[i] * phi_t[i] * shape(r_in, f['V'], 0, r_in[i])[1] for i in range(len(m)))),
        # internal torsion moment at 0.34 R per unit tip twist, in lb: GJ phi_t' there [TM-103867 measures it]
        Mt034=float(np.interp(0.34 * U.R * 12, 0.5 * (r_in[1:] + r_in[:-1]),
                              np.array([x * 1e6 for x in U.Y71_GJ[::-1][:-1]]) * np.diff(phi_t) / np.diff(r_in))),
        P1=float((ix * phi_t).sum()), P2=float((ix * phi_t * rf / U.R).sum()),
        d_cg=(U.X_CG - U.X_EA) * c, d_ac=(U.X_AC - U.X_EA) * c,
        seg=seg, freq=md['freq'], lag_root_scale=md['lag_root_scale'])


# ---------------------------------------------------------------- structural flap-lag coupling
R_PITCH_BEARING = 6.66      # in: the pitch horn's station [TR card 45]; outboard of it the blade pitches as a whole


def coupled(theta_root_deg, omega=U.OMEGA, lag_root_scale=1.0, n=4):
    """Coupled flap-lag bending modes with the blade pitched: outboard of the pitch bearing each element's principal
    axes turn with the local pitch (root pitch plus the linear twist), so the in-plane stiffness (25 times the flapwise)
    stiffens flapping and couples the two. DOFs per node (w, w', v, v'); returns frequencies (rad/s) and modes."""
    r, m, ix, eif, eil, gj = blade(lag_root_scale)
    nn = len(r)
    K = np.zeros((4 * nn, 4 * nn)); Kg = np.zeros_like(K); Mm = np.zeros_like(K); Sft = np.zeros_like(K)
    T = tension(r, m, 1.0)
    for j in range(nn - 1):
        L = r[j + 1] - r[j]
        B = np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L * L, -6 * L, 2 * L * L], [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L * L, -6 * L, 4 * L * L]]) / L ** 3
        G = np.array([[36, 3 * L, -36, 3 * L], [3 * L, 4 * L * L, -3 * L, -L * L], [-36, -3 * L, 36, -3 * L], [3 * L, -L * L, -3 * L, 4 * L * L]]) / (30 * L)
        rm = 0.5 * (r[j] + r[j + 1])
        th = math.radians(theta_root_deg + U.TWIST * rm / (U.R * 12)) if rm > R_PITCH_BEARING else 0.0
        c, s = math.cos(th), math.sin(th)
        eww = eil[j] * s * s + eif[j] * c * c
        evv = eil[j] * c * c + eif[j] * s * s
        ewv = (eil[j] - eif[j]) * s * c
        w = [4 * j, 4 * j + 1, 4 * j + 4, 4 * j + 5]; vv = [4 * j + 2, 4 * j + 3, 4 * j + 6, 4 * j + 7]
        K[np.ix_(w, w)] += eww * B; K[np.ix_(vv, vv)] += evv * B
        K[np.ix_(w, vv)] += ewv * B; K[np.ix_(vv, w)] += ewv * B
        Kg[np.ix_(w, w)] += T[j] * G; Kg[np.ix_(vv, vv)] += T[j] * G
    for j in range(nn):
        Mm[4 * j, 4 * j] = m[j]; Mm[4 * j + 2, 4 * j + 2] = m[j]; Sft[4 * j + 2, 4 * j + 2] = m[j]
    free = list(range(4, 4 * nn))
    sub = lambda A: A[np.ix_(free, free)]
    wf, V = solve(sub(K) + omega ** 2 * (sub(Kg) - sub(Sft)), sub(Mm), n)
    # classify: the share of kinetic energy in flap
    out = []
    for k in range(n):
        x = V[:, k]; wpart = sum(m[j + 1] * x[4 * j] ** 2 for j in range(nn - 1)); vpart = sum(m[j + 1] * x[4 * j + 2] ** 2 for j in range(nn - 1))
        out.append((wf[k] / omega, 'flap' if wpart > vpart else 'lag', wpart / (wpart + vpart)))
    return out


def coupled_stiffness(lag_root_scale, md):
    """The generalized bending stiffness of the flap and lag modes (md: modes() at zero pitch) with the structural
    coupling: K_xy(theta0) = A + B cos 2 theta0 + C sin 2 theta0 for xy = ff, ll, fl, theta0 the blade pitch at the
    pitch bearing (rad), ft lb. Exact in that form: the element stiffness is linear in cos^2, sin^2 and sin cos of
    theta0 plus the local twist."""
    r, m, ix, eif, eil, gj = blade(lag_root_scale)
    nn = len(r)
    vf = np.zeros(4 * nn); vl = np.zeros(4 * nn)
    vf[[4 * j for j in range(1, nn)]] = md['f']['V'][0::2, 0]; vf[[4 * j + 1 for j in range(1, nn)]] = md['f']['V'][1::2, 0]
    vl[[4 * j + 2 for j in range(1, nn)]] = md['l']['V'][0::2, 0]; vl[[4 * j + 3 for j in range(1, nn)]] = md['l']['V'][1::2, 0]

    def K_at(th0):
        K = np.zeros((4 * nn, 4 * nn))
        for j in range(nn - 1):
            L = r[j + 1] - r[j]
            Bm = np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L * L, -6 * L, 2 * L * L], [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L * L, -6 * L, 4 * L * L]]) / L ** 3
            rm = 0.5 * (r[j] + r[j + 1])
            th = th0 + math.radians(U.TWIST * rm / (U.R * 12)) if rm > R_PITCH_BEARING else 0.0
            c, s = math.cos(th), math.sin(th)
            w = [4 * j, 4 * j + 1, 4 * j + 4, 4 * j + 5]; vv = [4 * j + 2, 4 * j + 3, 4 * j + 6, 4 * j + 7]
            K[np.ix_(w, w)] += (eil[j] * s * s + eif[j] * c * c) * Bm
            K[np.ix_(vv, vv)] += (eil[j] * c * c + eif[j] * s * s) * Bm
            K[np.ix_(w, vv)] += (eil[j] - eif[j]) * s * c * Bm; K[np.ix_(vv, w)] += (eil[j] - eif[j]) * s * c * Bm
        return np.array([vf @ K @ vf, vl @ K @ vl, vf @ K @ vl]) / 12.0          # ft lb
    k0, k90, k45 = K_at(0.0), K_at(math.pi / 2), K_at(math.pi / 4)
    A = 0.5 * (k0 + k90); B = 0.5 * (k0 - k90); C = k45 - A
    return {name: (A[i], B[i], C[i]) for i, name in enumerate(('ff', 'll', 'fl'))}


def lag_scale_coupled(target=U.LAG_TARGET, theta_root=9.7 + 5.6, omega=425 * math.pi / 30):
    """hub region in-plane stiffness scale giving the target lag frequency with the blade pitched as in TM-103867's
    table 2 (collective 9.7 deg at 0.7 R)"""
    lo, hi = 0.01, 1.0
    for _ in range(50):
        mid = math.sqrt(lo * hi)
        f = [x for x in coupled(theta_root, omega, mid) if x[1] == 'lag'][0][0]
        lo, hi = (mid, hi) if f < target else (lo, mid)
    return math.sqrt(lo * hi)


if __name__ == '__main__':
    om = 425 * math.pi / 30
    print('blade weight %.2f lb (TR-75-50: 114.36), flap inertia %.1f slug ft2 (161.9)' % (
        sum(U.Y71_M) * 386.09, sum(m * r * r for m, r in zip(U.Y71_M, U.Y71_R)) / 12))
    for label, ks in (('table 4 as it stands', 1.0), ('hub in-plane stiffness tuned to %.2f/rev' % U.LAG_TARGET, lag_scale())):
        (wf, _), (wl, _), (wt, _) = frequencies(om, ks)
        print('%-40s flap %s  lag %s  torsion %s /rev (scale %.3f)' % (label, np.round(wf / om, 3), np.round(wl / om, 3), np.round(wt / om, 3), ks))
    print('published: Y-71 (TR-75-50)', U.FREQ_Y71)
    print('           TM-103867      ', U.FREQ_PT)
    (wf, _), (wl, _), (wt, _) = frequencies(1e-3, lag_scale())
    print('non-rotating: flap %.2f Hz, lag %.2f Hz, torsion %.2f Hz' % (wf[0] / 2 / math.pi, wl[0] / 2 / math.pi, wt[0] / 2 / math.pi))
    md = modes()
    for k in ('f', 'l'):
        d = md[k]
        print(k, 'M %.3f slug ft2  K %.0f ft lb  Kcf %.3f  S %.3f  Sr %.3f' % (d['M'], d['K'], d['Kcf'], d['S'], d['Sr']))
    print('t', md['t'])
    print('blade: M %.3f slug  Sr %.3f  Ib %.1f  G1 %.3f' % (md['Mb'], md['Sr'], md['Ib'], md['G1']))
    print('frequencies from the generalized properties at 100 %%: %s' % md['freq'])
