#!/usr/bin/env python3
"""The tail rotor's induced velocity at the fin (as the UH-60 model's trwake.py). Wake = skewed cylinder of ring
vortices (uniform loading, Heyson & Katzoff NACA TR 1319): rings parallel to the disc, their centres leaving the hub
along the wake's skew line, at chi from the disc's axis towards the in-plane flow. Evaluated over the fin's planform
and normalised by the velocity at the disc's centre (the model's uniform inflow), it gives the fin's sidewash from the
tail rotor at any skew.

The Bo 105's tail rotor pushes: on the left of the fin, thrust to the right, its wake leaving to the left, so the fin
stands in its inflow, upstream of the disc (12.5 in from it [TR card 52]); thrust reversed (left pedal), the wake
blows onto the fin. Hub frame as the model's: t the thrust axis (right), a0 aft, b0 = t x a0 (down); the wake leaves
along -t. Fin: the 3-D model's (a swept, tapered surface from the tail boom's end to the tail rotor gearbox, 0.35 m^2;
the flight manual's three-view agrees), in the plane BL 0, placed against the flight model's tail rotor hub as the
3-D model has it against its own; cut into spanwise strips.

  python3 trwake.py [--write]     strips' geometry [and recompute data/trwake_fin.json]
"""
import math, os, json
import numpy as np
from scipy.special import ellipk, ellipe
from scipy.integrate import quad_vec
import bo105data as U

# the 3-D model's fin (its own frame: x aft, z up, m): leading edge root to tip, trailing edge root to the gearbox
FIN_LE = [(4.980, 0.997), (5.748, 1.988)]
FIN_TE = [(5.490, 1.026), (5.971, 1.797), (5.900, 1.858), (5.827, 1.920), (5.763, 1.975), (5.748, 1.988)]
MODEL_HUB = (5.890, 1.910)          # its tail rotor hub
NSTRIP = 4
CHI = list(range(0, 181, 10))       # deg, wake skew from -t (0: straight down the axis, 90: edgewise, 180: up the axis)
PSI = list(range(0, 361, 30))       # deg, in-plane flow direction from a0 (aft) towards b0 (down)


def _x_at(line, z):
    zs = [q[1] for q in line]; xs = [q[0] for q in line]
    return float(np.interp(z, zs, xs))


def to_station(x, z):
    """the 3-D model's fin point -> (FS, WL), shifted with the model's hub onto the flight model's"""
    fs0, _, wl0 = U.model_to_station(MODEL_HUB[0], 0.0, MODEL_HUB[1])
    fs, _, wl = U.model_to_station(x, 0.0, z)
    return fs - fs0 + U.FS_TR, wl - wl0 + U.WL_TR


def planform():
    """(WL, leading edge FS, trailing edge FS) at the root and the tip, inches"""
    z0, z1 = FIN_LE[0][1], FIN_LE[-1][1]
    return tuple((to_station(_x_at(FIN_LE, z), z)[1], to_station(_x_at(FIN_LE, z), z)[0], to_station(_x_at(FIN_TE, z), z)[0])
                 for z in (z0, z1))


def area():
    """ft^2"""
    zz = np.linspace(FIN_LE[0][1], FIN_LE[-1][1], 400)
    c = [_x_at(FIN_TE, z) - _x_at(FIN_LE, z) for z in zz]
    return float(np.trapezoid(c, zz)) * 10.7639


def ring(r, z, R=1.0):
    """axial and radial velocity of a unit-circulation ring (radius R) at radial distance r, axial distance z"""
    a = (R + r) ** 2 + z * z
    m = min(4 * R * r / a, 1 - 1e-12)
    K, E = ellipk(m), ellipe(m)
    d = (R - r) ** 2 + z * z
    uz = (K + (R * R - r * r - z * z) / d * E) / (2 * math.pi * math.sqrt(a))
    ur = z / (2 * math.pi * r * math.sqrt(a)) * (-K + (R * R + r * r + z * z) / d * E) if r > 1e-9 else 0.0
    return uz, ur


def frame():
    t = np.array([0.0, 1.0, 0.0]); a0 = np.array([-1.0, 0.0, 0.0])
    return t, a0, np.cross(t, a0)


def induced(P, chi, psi, R=1.0):
    """induced air velocity (body axes, per unit circulation) at P (body axes, from the hub, in rotor radii)"""
    t, a0, b0 = frame()
    n = -t
    s_hat = math.cos(psi) * a0 + math.sin(psi) * b0
    w = math.cos(chi) * n + math.sin(chi) * s_hat

    def vel(s):
        d = P - s * w
        z = float(d @ n)
        rv = d - z * n
        r = float(np.linalg.norm(rv))
        uz, ur = ring(r, z, R)
        return uz * n + (ur * rv / r if r > 1e-9 else 0.0)

    # break points: where the rings' plane passes P, and where the cylinder's wall comes nearest
    pts = [x for x in (float(P @ n) / math.cos(chi) if abs(math.cos(chi)) > 1e-6 else None, float(P @ w)) if x is not None and 0 < x < 40]
    return quad_vec(vel, 0, 40, limit=400, points=pts or None)[0] + quad_vec(vel, 40, np.inf, limit=200)[0]


def centre(chi, psi):
    t = frame()[0]
    return float(induced(np.zeros(3), chi, psi) @ -t)


def fin_points(nc=6, ns=4):
    """per strip: [(P body ft from the hub, area ft2)] and the strip's (FS of the quarter chord, WL, area)"""
    z0, z1 = FIN_LE[0][1], FIN_LE[-1][1]
    strips = []
    for k in range(NSTRIP):
        pts, ar, fs_a, wl_a = [], 0.0, 0.0, 0.0
        for j in range(ns):
            z = z0 + (k + (j + 0.5) / ns) / NSTRIP * (z1 - z0)
            fs_le, wl = to_station(_x_at(FIN_LE, z), z)
            fs_te, _ = to_station(_x_at(FIN_TE, z), z)
            dh = (z1 - z0) / NSTRIP / ns * U.IN_PER_M
            for i in range(nc):
                fs = fs_le + (i + 0.5) / nc * (fs_te - fs_le)
                A = (fs_te - fs_le) / nc * dh / 144.0
                P = np.array([-(fs - U.FS_TR), 0.0 - U.BL_TR, -(wl - U.WL_TR)]) / 12.0
                pts.append((P, A)); ar += A
            fs_a += (fs_le + 0.25 * (fs_te - fs_le)) * (fs_te - fs_le) * dh / 144.0; wl_a += wl * (fs_te - fs_le) * dh / 144.0
        strips.append((pts, fs_a / ar, wl_a / ar, ar))
    return strips


def _case(args):
    c, p = args
    ch, ps = math.radians(min(c, 179.9) if c else 0.0), math.radians(p)
    ch = math.radians(89.9) if c == 90 else ch
    w0 = centre(ch, ps)
    return [(sum(induced(P / U.R_TR, ch, ps) * A for P, A in pts) / ar / w0)[:2].tolist() for pts, fs, wl, ar in fin_points()]


def tables():
    """per strip: (FS of the quarter chord, WL, area ft2) and k[x|y][chi][psi], the area-mean induced air velocity
    over the strip (body x and y) over the disc centre's"""
    from multiprocessing import Pool
    cases = [(c, p) for c in CHI for p in PSI]
    uniq = [(c, p if c not in (0, 180) else 0) for c, p in cases]
    with Pool() as pool:
        res = dict(zip(sorted(set(uniq)), pool.map(_case, sorted(set(uniq)))))
    out = []
    for k, (pts, fs, wl, ar) in enumerate(fin_points()):
        kx = [[res[(c, p if c not in (0, 180) else 0)][k][0] for p in PSI] for c in CHI]
        ky = [[res[(c, p if c not in (0, 180) else 0)][k][1] for p in PSI] for c in CHI]
        out.append(dict(fs=fs, wl=wl, area=ar, kx=kx, ky=ky))
    return out


DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'trwake_fin.json')


def load():
    """the tables, from data/trwake_fin.json (python3 trwake.py --write recomputes it)"""
    return json.load(open(DATA))


if __name__ == '__main__':
    import sys
    t, a0, b0 = frame()
    (w0, le0, te0), (w1, le1, te1) = planform()
    print('fin %.2f ft2 (%.3f m2): root WL %.1f FS %.1f-%.1f, tip WL %.1f FS %.1f-%.1f' % (area(), area() / 10.7639, w0, le0, te0, w1, le1, te1))
    for pts, fs, wl, ar in fin_points():
        P = sum(P * A for P, A in pts) / ar
        print('strip FS %.0f WL %.0f %.2f ft2: on the thrust side %.2f R, in-plane %.2f R' % (fs, wl, ar, P @ t / U.R_TR,
              math.hypot(P @ a0, P @ b0) / U.R_TR))
    if '--write' in sys.argv:
        json.dump(dict(chi=CHI, psi=PSI, strips=tables()), open(DATA, 'w'), indent=0)
        print('wrote', DATA)
