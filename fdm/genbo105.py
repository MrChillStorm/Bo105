#!/usr/bin/env python3
"""Generate the Bo 105 JSBSim model: elastic hingeless main rotor, teetering tail rotor, airframe.

  python3 genbo105.py OUTDIR [--fg]   writes OUTDIR/bo105-jsb.xml (or bo105.xml) and OUTDIR/Systems/bo105-*.xml

Main rotor: the 4 blades of the hingeless, soft in-plane rotor are simulated individually as elastic beams
cantilevered from the hub with 2.5 deg of precone. Each blade has its first flap and first lag bending modes,
computed from the blade's mass and stiffness distribution [TR-75-50 table 4] by fdm/blademodes.py, integrated in the
FCS at the FDM rate, and its first torsion mode, quasi-static. The blade is cut into 8 segments of equal annulus area
with the NACA 23012 C-81 tables [TR-75-50 table A-1], at their own angle of attack and Mach number, in the air
the full kinematics give them: the aircraft's motion, the rotor's rotation, the blade's bending velocities and the
induced inflow (Peters-He finite-state wake). The hub loads are summed from the blades' aerodynamic and inertial
forces (the forces of the blades' motion relative to the airframe, whose mass JSBSim carries rigidly), so the hub
moment of the hingeless rotor comes from the blades' root bending, not from a hinge offset.

Shaft axes: x forward, y right, z down along the shaft (3 deg forward tilt); the rotor turns anticlockwise seen from
above; azimuth psi from the tail. A blade at psi lies along e_r = (-cos psi, sin psi, 0), moves along
e_t = (sin psi, cos psi, 0); e_n = (0, 0, -1) is up. Body axes: JSBSim's (x forward, y right, z down).
"""
import math, os, sys
import numpy as np
import bo105data as U
import blademodes
import engines250 as engines
from xmlfun import *
from data import c81_23012 as AF
from data import c81_0012 as AF12
from data import c81_fuselage as FUS
from data import heyson_ground as HG
import peters_he as PH
import mrwake

G0 = 32.174
R2D = 180 / math.pi
# functions read by only one other are written into it (xmlfun.inline_single_use; --no-inline keeps them all named),
# but for these, which the test programme and the report read
INLINE = True
KEEP = {'aero/emp/fin/dFy-lbs', 'aero/rotor/a1-deg', 'aero/rotor/b1-deg', 'aero/rotor/power-hp', 'aero/rotor/thrust-lbs',
        'aero/rotor/torque-ftlb', 'aero/rotor/tr/power-hp', 'aero/rotor/tr/thrust-lbs', 'drive/loss-ftlb', 'drive/mr-torque-ftlb',
        'eng[0]/torque-pct'} | {'rotor/b%d/torsion-034-inlb' % k for k in range(4)}


# ---------------------------------------------------------------- section tables
def section_tables():
    """lift and drag on the lift table's angle rows, moment on its own; Mach columns of the drag table (the lift
    table is re-gridded to them)"""
    a_cl, cl = AF.table(AF.CL)
    a_cd, cd = AF.table(AF.CD)
    a_cm, cm = AF.table(AF.CM)
    mach = AF.CD_MACH
    cl_r = [[float(np.interp(m, AF.CL_MACH, row)) for m in mach] for row in cl]
    return mach, (a_cl, cl_r), (a_cd, cd), (a_cm, cm)


def _odd(xs, ys):
    return [-x for x in reversed(xs[1:])] + list(xs), [-y for y in reversed(ys[1:])] + list(ys)


def _even(xs, ys):
    return [-x for x in reversed(xs[1:])] + list(xs), list(reversed(ys[1:])) + list(ys)


def _mirror180(xs, ys, negate):
    """extend a table over -90..90 deg to +-180: reverse flow sees the section mirrored front to back"""
    k = -1 if negate else 1
    pts = dict(zip(xs, ys))
    for x, y in zip(xs, ys):
        if 0 <= x < 90:
            pts[180 - x] = k * y
        if -90 < x <= 0:
            pts[-180 - x] = k * y
    out = sorted(pts)
    return out, [pts[x] for x in out]


class Model:
    def __init__(s):
        s.g = Graph()
        s.ctl = FCS('Bo 105 controls')
        s.rot = FCS('Bo 105 rotor')
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = [], [], [], [], [], []
        s.aero = s.g
        s.air = FCS('Bo 105 airframe')
        segs, s.edges = U.segments()
        s.segs = segs
        s.bc = blademodes.rotor_constants([r for r, dr in segs])

    # ------------------------------------------------------------ controls
    def controls(s):
        """Pilot's controls to the swashplate and the tail rotor [TR cards 131-134]: no mixing, no augmentation (the
        basic Bo 105 CBS); hydraulic boost servos. FlightGear's trims shift the controls' centre."""
        c = s.ctl
        c.declare('/controls/engines/engine[0]/throttle', 0.0)
        c.declare('ctl/init', 0)
        c.channel('pilot')
        c.fn('ctl/init-fast', IFTHEN(LT(p('simulation/sim-time-sec'), v(1.0)), v(1), MIN(v(1), MAX(v(0), p('ctl/init')))))
        coll = c.fn('ctl/coll-norm', p('/controls/engines/engine[0]/throttle'), clip=('0', '1'),
                    desc='collective lever, 0 down .. 1 up (FlightGear: the throttle axis)')
        c.fn('ctl/coll-in', M(coll, v(U.COLL_IN)))
        # JSBSim's rudder is positive left (FlightGear hands it the negated pedal): negated back
        lon = c.fn('ctl/lon-norm', S(p('fcs/elevator-cmd-norm'), p('fcs/pitch-trim-cmd-norm')), clip=('-1', '1'), desc='+ forward')
        lat = c.fn('ctl/lat-norm', S(p('fcs/aileron-cmd-norm'), p('fcs/roll-trim-cmd-norm')), clip=('-1', '1'), desc='+ right')
        ped = c.fn('ctl/ped-norm', NEG(S(p('fcs/rudder-cmd-norm'), p('fcs/yaw-trim-cmd-norm'))), clip=('-1', '1'), desc='+ right')
        c.fn('ctl/lon-in', M(lon, v(U.LON_IN / 2)), desc='cyclic, inches forward of centre')
        c.fn('ctl/lat-in', M(lat, v(U.LAT_IN / 2)), desc='cyclic, inches right of centre')
        c.fn('ctl/ped-in', M(ped, v(U.PED_IN / 2)), desc='pedals, inches right of centre')
        c.channel('rigging')
        mid = lambda r: (r[0] + r[1]) / 2
        half = lambda r: (r[1] - r[0]) / 2
        c.fn('ctl/theta0-cmd-deg', S(v(U.COLL_ROOT[0]), M(v(U.COLL_ROOT[1] - U.COLL_ROOT[0]), coll)),
             desc='blade pitch at the shaft (the twist is zero there), deg')
        c.fn('ctl/B1s-cmd-deg', S(v(mid(U.B1S_RANGE)), M(v(half(U.B1S_RANGE)), lon)), desc='longitudinal cyclic, + forward')
        c.fn('ctl/A1s-cmd-deg', S(v(mid(U.A1S_RANGE)), M(v(half(U.A1S_RANGE)), lat)), desc='lateral cyclic, + right')
        c.fn('ctl/thetaTR-cmd-deg', D(v(mid(U.TR_PITCH_RANGE)), M(v(half(U.TR_PITCH_RANGE)), ped)),
             desc='tail rotor blade pitch, + pushing the tail right (anti-torque)')
        c.channel('servos')
        # hydraulic boost: no Bo 105 data; the UH-60's primary servo response (Ames' no-load test, TM-88360 app. A)
        for n in ('B1s', 'A1s', 'theta0', 'thetaTR'):
            c.second_order('ctl/%s-deg' % n, 'ctl/%s-cmd-deg' % n, (0, 0, 1), (0.0002081, 0.020, 1))
        c.channel('sensors')
        c.fn('sens/coll-pct', M(v(100), coll))

    # ------------------------------------------------------------ main rotor states (FCS)
    # ---- the blade modes' equations, shared by the rotor system (which steps them) and the hub loads (which take the
    # accelerations at the same instant as the deflections and the azimuth they are resolved with)
    def flap_forcing(s, k, c, sn, body=True):
        """flap mode forcing: aerodynamic, hub specific force, gyroscopic and (body) the airframe's angular acceleration,
        precone; c, sn: cosine and sine of the blade root's azimuth"""
        bc, W, bp = s.bc, p('rotor/omega-rad_sec'), U.PRECONE
        gyro = M(v(2), W, D(M(p('rotor/ps'), c), M(p('rotor/qs'), sn)))
        if body:
            gyro = S(gyro, M(p('rotor/pds'), sn), M(p('rotor/qds'), c))
        return S(p('aero/rotor/b%d/Qf' % k), M(v(bc['Sf']), p('rotor/Az')), M(v(bc['Srf']), gyro), M(v(-bp * bc['Srf']), SQR(W)))

    def lag_forcing(s, k, c, sn):
        at = S(M(sn, p('rotor/Ax')), M(c, p('rotor/Ay')))
        return S(p('aero/rotor/b%d/Ql' % k), NEG(M(v(s.bc['Sl']), at)))

    def flap_accel(s, b, Ff, qf, qfd, ql, qld):
        """Mf qf'' = Ff - Kf qf - Kfl ql - 2 z wf Mf qf' - cor ql' (b: the rotor system's blade prefix, for the stiffness
        and Coriolis terms it computes)"""
        bc = s.bc
        kf = p(b + '/kf')
        damp = M(v(2 * U.ZETA_STRUCT * bc['Mf']), SQ(MAX(v(1), Q(kf, v(bc['Mf'])))), qfd)
        return Q(S(Ff, NEG(M(kf, qf)), NEG(M(p(b + '/kfl'), ql)), NEG(damp), NEG(M(p(b + '/coriolis'), qld))), v(bc['Mf']))

    def lag_free(s, b, Fl, ql, qld, qf, qfd):
        """the lag equation's right-hand side but for the hub's angular acceleration about the shaft"""
        bc = s.bc
        kl = p(b + '/kl')
        damp = M(v(2 * U.ZETA_LAG * bc['Ml']), SQ(MAX(v(1), Q(kl, v(bc['Ml'])))), qld)
        return S(Fl, NEG(M(kl, ql)), NEG(M(p(b + '/kfl'), qf)), NEG(damp), M(p(b + '/coriolis'), qfd))

    def rotor_states(s):
        r = s.rot
        bc = s.bc
        ci, si = math.cos(U.SHAFT_TILT), math.sin(U.SHAFT_TILT)
        r.declare('rotor/omega-rad_sec', U.OMEGA)
        r.declare('rotor/psi-rad', 0.0)
        r.declare('rotor/tr/psi-rad', 0.0)
        for k in range(U.NB):
            for st in ('qf', 'qfd', 'ql', 'qld', 'qt', 'qfdd', 'qldd'):
                r.declare('rotor/b%d/%s' % (k, st), 0.0)
        r.declare('rotor/v0-fps', 0.0)
        for st, tg in ((U.PH_COS, 'c'), (U.PH_SIN, 's')):
            for i in range(len(st)):
                r.declare('rotor/ph/%s%d' % (tg, i), 0.0)
        if U.TR_MODEL == 'blades':
            for st in ('beta-rad', 'betadot-rad_sec', 'v0-fps'):
                r.declare('rotor/tr/' + st, 0.0)
        else:
            r.declare('rotor/tr/dw', 0.0)
        r.channel('shaft kinematics')
        r.fn('rotor/ps', S(M(v(ci), p('velocities/p-rad_sec')), M(v(si), p('velocities/r-rad_sec'))))
        r.fn('rotor/qs', p('velocities/q-rad_sec'))
        # the airframe's accelerations as the rotor feels them; rotor/accel-hold keeps them at their last values (for
        # quasi-steady derivatives with the airframe frozen, where JSBSim's accelerations are not the motion's)
        r.declare('rotor/accel-hold', 0)
        held = GT(p('rotor/accel-hold'), v(0.5))
        hold = lambda name, e: IFTHEN(held, p(name), e)
        r.fn('rotor/pds', hold('rotor/pds', S(M(v(ci), p('accelerations/pdot-rad_sec2')), M(v(si), p('accelerations/rdot-rad_sec2')))))
        r.fn('rotor/qds', hold('rotor/qds', p('accelerations/qdot-rad_sec2')))
        # the hub's specific force (ft/s2, body axes: forward, right, down): the CG's plus the airframe's angular
        # acceleration and centripetal terms at the hub's arm (it is 4.5 ft above the CG: rolling moves it sideways, the
        # blades' lag feels that as the airframe feels their in-plane forces at the same arm)
        hx, hy, hz = s.arm_cg(U.FS_MR, U.BL_MR, U.WL_MR)
        P_, Q_, R_ = p('velocities/p-rad_sec'), p('velocities/q-rad_sec'), p('velocities/r-rad_sec')
        Pd, Qd, Rd = p('accelerations/pdot-rad_sec2'), p('accelerations/qdot-rad_sec2'), p('accelerations/rdot-rad_sec2')
        wxr = (D(M(Q_, hz), M(R_, hy)), D(M(R_, hx), M(P_, hz)), D(M(P_, hy), M(Q_, hx)))
        cent = (D(M(Q_, wxr[2]), M(R_, wxr[1])), D(M(R_, wxr[0]), M(P_, wxr[2])), D(M(P_, wxr[1]), M(Q_, wxr[0])))
        ang = (D(M(Qd, hz), M(Rd, hy)), D(M(Rd, hx), M(Pd, hz)), D(M(Pd, hy), M(Qd, hx)))
        cg = (M(v(G0), p('accelerations/Nx')), M(v(G0), p('accelerations/Ny')), M(v(-G0), p('accelerations/Nz')))
        hub = [S(cg[i], ang[i], cent[i]) for i in range(3)]
        # to shaft axes (the inverse of main_rotor's to_b)
        r.fn('rotor/Az', hold('rotor/Az', D(M(v(ci), hub[2]), M(v(si), hub[0]))), desc='hub specific force along the shaft (down +)')
        r.fn('rotor/Ax', hold('rotor/Ax', S(M(v(ci), hub[0]), M(v(si), hub[2]))), desc='hub specific force along the shaft x (forward +)')
        r.fn('rotor/Ay', hold('rotor/Ay', hub[1]))
        r.fn('rotor/rds-in', hold('rotor/rds-in', p('accelerations/rdot-rad_sec2')))
        r.fn('rotor/pds-in', hold('rotor/pds-in', p('accelerations/pdot-rad_sec2')))
        dt = p('simulation/dt')
        W = p('rotor/omega-rad_sec')
        init = GT(p('ctl/init-fast'), v(0.5))
        bp = U.PRECONE
        r.channel('blade bending')
        # modal equations of the first flap (q_f) and lag (q_l) modes, coordinates = tip deflection / R:
        #   Mf qf'' = Qf + Sf Az + Srf (2 W (ps c - qs s) + pds s + qds c) - bp W^2 Srf
        #             - (Kff(theta) + W^2 Kcf_f) qf - Kfl(theta) ql - 2 z wf Mf qf' - 2 W (bp G1 + qf G2) ql'
        #   Ml ql'' = Ql - Sl a_t - Srl W'_abs - (Kll(theta) + W^2 (Kcf_l - Ml)) ql - Kfl(theta) qf - 2 z wl Ml ql'
        #             + 2 W (bp G1 + qf G2) qf'
        # (precone: the centrifugal force on the coned blade; G1, G2: Coriolis between flap and lag through the blade's
        # coning and its own bending)
        # the lag modes ride on the hub's angular acceleration and their accelerations load the drive: on this soft
        # in-plane rotor the first lag mode carries most of the blades' inertia about the shaft, so the rotor speed and
        # the four lag accelerations are solved together in each step (as DLR's SIMH [HG]: "resolved for rotor and
        # lead/lag accelerations"), the rotor speed's equation in engines250.rotor_speed
        init_fast = init
        # the blades start from their static deflection in the first frames of a run (not after a reset of the
        # initial conditions: the trims keep the rotor's state from one evaluation to the next)
        r.declare('rotor/initialized', 0)
        init = GT(r.fn('rotor/blade-init', LT(p('rotor/initialized'), v(0.5))), v(0.5))
        r.fn('rotor/initialized-next', GT(p('simulation/sim-time-sec'), v(0.02)), out='rotor/initialized')
        Kc = bc['Kc']
        for k in range(U.NB):
            b = 'rotor/b%d' % k
            psi = S(p('rotor/psi-rad'), v(k * math.pi / 2))
            c, sn = COS(psi), SIN(psi)
            qf, qfd, ql, qld = p(b + '/qf'), p(b + '/qfd'), p(b + '/ql'), p(b + '/qld')
            # bending stiffness with the structural flap-lag coupling: outboard of the pitch bearing the blade's
            # principal axes turn with its pitch (blademodes.coupled_stiffness), K = A + B cos 2 theta + C sin 2 theta
            th2 = M(v(2 / R2D), p('aero/rotor/b%d/theta-deg' % k))
            c2, s2 = r.fn(b + '/c2th', COS(th2)), r.fn(b + '/s2th', SIN(th2))
            ks = lambda n: S(v(Kc[n][0]), M(v(Kc[n][1]), c2), M(v(Kc[n][2]), s2))
            kf = r.fn(b + '/kf', S(ks('ff'), M(v(bc['Kcf_f']), SQR(W))), desc='flap stiffness, structural + centrifugal')
            kl = r.fn(b + '/kl', S(ks('ll'), M(v(bc['Kcf_l'] - bc['Ml']), SQR(W))), desc='lag stiffness, structural + centrifugal - softening')
            kfl = r.fn(b + '/kfl', ks('fl'), desc='structural flap-lag coupling')
            cor = r.fn(b + '/coriolis', M(v(2), W, S(v(bp * bc['G1']), M(v(bc['G2']), qf))))
            Ff = r.fn(b + '/Ff', s.flap_forcing(k, c, sn), desc='flap mode forcing: aerodynamic, hub acceleration, gyroscopic, precone')
            Fl = r.fn(b + '/Fl', s.lag_forcing(k, c, sn), desc='lag mode forcing: aerodynamic, hub acceleration in the plane')
            qfdd = r.fn(b + '/qfdd-calc', s.flap_accel(b, Ff, qf, qfd, ql, qld), desc='flap mode acceleration')
            r.fn(b + '/Fl-free', s.lag_free(b, Fl, ql, qld, qf, qfd), desc='lag mode: all but the hub\'s angular acceleration')
            # second-order explicit steps (Adams-Bashforth on the accelerations): the blade modes respond at 1/rev close to
            # their natural frequencies with strong aerodynamic damping, where a first-order step's half-step lag in the
            # damping turns the flapping's phase by several degrees at 240 Hz; in the first frames the blades start at
            # their static deflection
            det = D(M(kf, kl), SQR(kfl))
            qf_static = Q(D(M(Ff, kl), M(p(b + '/Fl'), kfl)), det)
            r.fn(b + '/ql-static', Q(D(M(p(b + '/Fl'), kf), M(Ff, kfl)), det))
            acc = r.fn(b + '/qfdd-ab', IFTHEN(init, qfdd, D(M(v(1.5), qfdd), M(v(0.5), p(b + '/qfdd')))))
            r.fn(b + '/qf-next', IFTHEN(init, qf_static, S(qf, M(dt, qfd), M(v(0.5), dt, dt, acc))), out=b + '/qf')
            r.fn(b + '/qfd-next', IFTHEN(init, v(0), S(qfd, M(acc, dt))), out=b + '/qfd')
            r.fn(b + '/qfdd-next', IFTHEN(init, v(0), qfdd), out=b + '/qfdd')
        # the hub's angular acceleration relative to the airframe, then its absolute one about the shaft (up): less the
        # airframe's about the shaft (down +)
        engines.rotor_speed(r, S(*[p('rotor/b%d/Fl-free' % k) for k in range(U.NB)]), bc['Srl'], bc['Ml'], init_fast)
        Wd = r.fn('rotor/omega-dot-abs', IFTHEN(init_fast, v(0), D(p('drive/omega-dot'), p('drive/r-dot-shaft'))))
        for k in range(U.NB):
            b = 'rotor/b%d' % k
            qld, ql = p(b + '/qld'), p(b + '/ql')
            qldd = r.fn(b + '/qldd-calc', Q(D(p(b + '/Fl-free'), M(v(bc['Srl']), Wd)), v(bc['Ml'])), desc='lag mode acceleration')
            acc = r.fn(b + '/qldd-ab', IFTHEN(init, qldd, D(M(v(1.5), qldd), M(v(0.5), p(b + '/qldd')))))
            r.fn(b + '/ql-next', IFTHEN(init, p(b + '/ql-static'), S(ql, M(dt, qld), M(v(0.5), dt, dt, acc))), out=b + '/ql')
            r.fn(b + '/qld-next', IFTHEN(init, v(0), S(qld, M(acc, dt))), out=b + '/qld')
            r.fn(b + '/qldd-next', IFTHEN(init, v(0), qldd), out=b + '/qldd')
        r.channel('blade torsion')
        # first torsion mode (3.3/rev), quasi-static: the aerodynamic moment about the elastic axis, the inertial forces
        # at the CG (aft of the elastic axis) and the propeller moment of the blade's rigid pitch, against the
        # torsional and control system stiffness and the propeller moment of the twist itself; relaxed by half a step
        kt = S(v(bc['Kt']), M(v(bc['Kprop_t']), SQR(W)))
        for k in range(U.NB):
            b = 'rotor/b%d' % k
            psi = S(p('rotor/psi-rad'), v(k * math.pi / 2))
            c, sn = COS(psi), SIN(psi)
            wr = D(M(p('rotor/qs'), sn), M(p('rotor/ps'), c))           # body rate along e_r
            qin = M(v(-bc['d_cg']), S(M(v(bc['T1']), p('rotor/Az')), M(v(-bc['T2']), p(b + '/qfdd')),
                                         M(v(-2 * bc['T3']), W, wr)))
            # the centrifugal force on the coned and bent blade has a component along the blade's normal (down) at the
            # CG, aft of the elastic axis: nose up, in proportion to the slope
            qcf = M(v(bc['d_cg']), SQR(W), S(v(U.PRECONE * bc['T3']), M(v(bc['T4']), p(b + '/qf'))))
            th_rig = M(v(1 / R2D), S(M(v(bc['P1']), p('aero/rotor/b%d/theta-deg' % k)), v(bc['P2'] * U.TWIST)))
            qprop = NEG(M(SQR(W), th_rig))
            target = Q(S(p('aero/rotor/b%d/Qt' % k), qin, qcf, qprop), kt)
            r.fn(b + '/qt-next', IFTHEN(init, target, S(p(b + '/qt'), M(v(0.5), D(target, p(b + '/qt'))))),
                 clip=('-0.3', '0.3'), out=b + '/qt')
            r.fn(b + '/torsion-034-inlb', M(v(bc['Mt034']), p(b + '/qt')), desc='internal torsion moment at 0.34 R, in lb (nose up +)')
        s.inflow_states()
        s.tail_rotor_states()
        # the azimuths advance last, so that every rotor equation of a step sees the blades where their states are
        r.channel('rotor azimuth')
        psn = S(p('rotor/psi-rad'), M(p('rotor/omega-rad_sec'), p('simulation/dt')))
        r.fn('rotor/psi-next', IFTHEN(GT(psn, v(2 * math.pi)), D(psn, v(2 * math.pi)), psn), out='rotor/psi-rad')
        psn = S(p('rotor/tr/psi-rad'), M(v(U.GEAR_TR), p('rotor/omega-rad_sec'), p('simulation/dt')))
        r.fn('rotor/tr/psi-next', IFTHEN(GT(psn, v(2 * math.pi)), D(psn, v(2 * math.pi)), psn), out='rotor/tr/psi-rad')

    def inflow_states(s):
        """Peters-He finite-state wake (as the UH-60 model): M a' + L^-1 V a = tau / 2 per state set, in the wake
        frame; the hub-frame harmonic pairs are turned into it, stepped semi-implicitly, and turned back."""
        r = s.rot
        r.channel('inflow')
        h = M(MAX(p('rotor/omega-rad_sec'), v(1.0)), p('simulation/dt'))
        cd, sd = p('aero/rotor/ph/cd'), p('aero/rotor/ph/sd')
        rot = {1: (cd, sd), 2: (r.fn('rotor/ph/cd2', D(SQR(cd), SQR(sd))), r.fn('rotor/ph/sd2', M(v(2), cd, sd)))}
        sets = {'c': U.PH_COS, 's': U.PH_SIN}
        pair = {}
        for i, st in enumerate(U.PH_COS):
            if st[1] > 0 and st in U.PH_SIN:
                pair[('c', i)] = U.PH_SIN.index(st); pair[('s', U.PH_SIN.index(st))] = i

        def wake(tg, i, prop):
            rh = sets[tg][i][1]
            if rh == 0 or (tg, i) not in pair:
                return p(prop % (tg, i))
            cr, sr = rot[rh]; o = 's' if tg == 'c' else 'c'; po = p(prop % (o, pair[(tg, i)]))
            me = p(prop % (tg, i))
            return S(M(me, cr), M(po, sr)) if tg == 'c' else D(M(me, cr), M(po, sr))
        for tg, st in sets.items():
            for i in range(len(st)):
                r.fn('rotor/ph/w%s%d' % (tg, i), wake(tg, i, 'rotor/ph/%s%d'))
                r.fn('rotor/ph/wtau-%s%d' % (tg, i), wake(tg, i, 'aero/rotor/ph/tau-%s%d'))
        for tg, st in sets.items():
            for i, (j, rh) in enumerate(st):
                k_ = Q(h, v(2 / math.pi * PH.H(j, rh)))
                a = lambda i_, kk: p('aero/rotor/ph/A%s%d-%d' % (tg, i_, kk))
                off = [M(a(i, kk), p('rotor/ph/w%s%d' % (tg, kk))) for kk in range(len(st)) if kk != i]
                num = S(p('rotor/ph/w%s%d' % (tg, i)), M(k_, D(M(v(0.5), p('rotor/ph/wtau-%s%d' % (tg, i))), S(*off) if off else v(0))))
                r.fn('rotor/ph/n%s%d' % (tg, i), Q(num, S(v(1), M(k_, a(i, i)))))
        for tg, st in sets.items():
            for i, (j, rh) in enumerate(st):
                if rh == 0 or (tg, i) not in pair:
                    r.fn('rotor/ph/%s%d-next' % (tg, i), p('rotor/ph/n%s%d' % (tg, i)), out='rotor/ph/%s%d' % (tg, i))
                else:
                    cr, sr = rot[rh]; o = 's' if tg == 'c' else 'c'; po = p('rotor/ph/n%s%d' % (o, pair[(tg, i)])); me = p('rotor/ph/n%s%d' % (tg, i))
                    r.fn('rotor/ph/%s%d-next' % (tg, i), D(M(me, cr), M(po, sr)) if tg == 'c' else S(M(po, sr), M(me, cr)), out='rotor/ph/%s%d' % (tg, i))
        r.fn('rotor/v0-out', MAX(v(0), M(v(math.sqrt(3) * U.R), p('rotor/ph/c0'), p('rotor/omega-rad_sec'))), out='rotor/v0-fps',
             desc='mean inflow, sqrt(3) x the first state, ft/s')

    def tail_rotor_states(s):
        if U.TR_MODEL == 'blades':
            return s.tail_rotor_blade_states()
        s.tail_rotor_bailey_states()

    def tail_rotor_blade_states(s):
        """The blade-element tail rotor's states (aerodynamics in tail_rotor_blades): the two blades' teetering, and the
        uniform inflow (Pitt-Peters' momentum, over the effective disc B^2 A, as the UH-60 model's):
        (8 / 3 pi) R dv/dt + 2 V' v = T / (rho A), V' the mass flow through the disc. Its axial part goes to nearly
        nothing when the rotor moves into its own wake (a right crosswind or a left yaw: the pusher's wake blows
        left); then the induced velocity grows and the thrust falls away, vortex ring state as momentum theory can't
        have it.

        Teeter: blade 0 flaps by beta towards the thrust side, blade 1 by -beta; their two flap equations about the
        centre (each as the UH-60 model's blades': aerodynamic moment, gyroscopic and centrifugal terms, the hub's
        angular acceleration) less one another give the teeter's, semi-implicit at the FDM rate (14 deg of azimuth a
        step at 2230 rpm):
          beta'' = (M0 - M1) / (2 I) - W^2 cos(beta) sin(beta) - 2 W w_r + w_e'"""
        r = s.rot
        dt = p('simulation/dt')
        W = M(v(U.GEAR_TR), p('rotor/omega-rad_sec'))
        init_fast = GT(p('ctl/init-fast'), v(0.5))
        r.channel('tail rotor teeter')
        # body rates in the hub frame (a0 aft, b0 = t x a0 down; t the thrust axis, right), inertial
        wa, wb = r.fn('rotor/tr/wa', NEG(p('velocities/p-rad_sec'))), r.fn('rotor/tr/wb', p('velocities/r-rad_sec'))
        wad, wbd = r.fn('rotor/tr/wad', NEG(p('accelerations/pdot-rad_sec2'))), r.fn('rotor/tr/wbd', p('accelerations/rdot-rad_sec2'))
        I = U.I_BLADE_TR
        psi = p('rotor/tr/psi-rad')
        c, sn = COS(psi), SIN(psi)
        be = p('rotor/tr/beta-rad')
        wr = S(M(c, wa), M(sn, wb)); wed = S(NEG(M(sn, wad)), M(c, wbd))
        bdd = r.fn('rotor/tr/betaddot', S(Q(D(p('aero/rotor/tr/b0/Mflap'), p('aero/rotor/tr/b1/Mflap')), v(2 * I)),
                                         NEG(M(SQR(W), COS(be), SIN(be))), NEG(M(v(2), W, wr)), wed),
                   desc='teeter acceleration: the blades\' aerodynamic moments, centrifugal stiffness, gyroscopic, hub angular acceleration')
        bd_new = S(p('rotor/tr/betadot-rad_sec'), M(bdd, dt))
        b_new = S(be, M(bd_new, dt))
        lim = U.TEETER_STOP_TR
        r.fn('rotor/tr/betadot-next', IFTHEN(LT(b_new, v(-lim)), MAX(bd_new, v(0)), IFTHEN(GT(b_new, v(lim)), MIN(bd_new, v(0)), bd_new)),
             out='rotor/tr/betadot-rad_sec')
        r.fn('rotor/tr/beta-next', IFTHEN(M(init_fast, LT(W, v(20))), v(0),
                                          MAX(v(-lim), MIN(v(lim), S(be, M(p('rotor/tr/betadot-rad_sec'), dt))))), out='rotor/tr/beta-rad')
        r.channel('tail rotor inflow')
        A = math.pi * U.R_TR ** 2 * U.TIP_LOSS_TR_BLADES ** 2
        v0 = p('rotor/tr/v0-fps')
        Vm = r.fn('rotor/tr/Vprime-fps', SQ(S(SQR(p('aero/rotor/tr/Vinplane-fps')), SQR(S(p('aero/rotor/tr/Vaxial-fps'), v0)), v(25.0))),
                  desc='mass flow velocity through the disc (with a 5 ft/s floor, as the main rotor)')
        # (it reverses with the thrust: full right pedal)
        r.fn('rotor/tr/v0-next', S(v0, M(dt, Q(D(Q(p('aero/rotor/tr/thrust-lbs'), M(p('atmosphere/rho-slugs_ft3'), v(A))),
                                                M(v(2), Vm, v0)), v(8 / (3 * math.pi) * U.R_TR)))), out='rotor/tr/v0-fps')

    def tail_rotor_bailey_states(s):
        """Bailey's closed-form tail rotor: its inflow ratio, one relaxed fixed-point step per frame"""
        r = s.rot
        r.channel('tail rotor')
        B = U.TIP_LOSS_TR
        mu, muz = p('aero/rotor/tr/mu'), p('aero/rotor/tr/muz')
        th = r.fn('rotor/tr/theta-rad', M(p('ctl/thetaTR-deg'), v(1 / R2D)))
        G = U.A_TR * U.SIGMA_TR / 2
        mu2 = SQR(mu)
        t31 = S(v(B ** 2 / 2), M(v(0.25), mu2)); t32 = S(v(B ** 3 / 3), M(v(B / 2), mu2))
        dw = p('rotor/tr/dw')
        B2 = B ** 2
        dwn = Q(M(v(G), S(M(muz, t31), M(th, t32))), S(M(v(2 * B2), SQ(S(mu2, SQR(D(muz, dw)), v(1e-6)))), M(v(G), t31)))
        r.fn('rotor/tr/dw-next', S(dw, M(v(0.5), D(dwn, dw))), out='rotor/tr/dw')

    # ------------------------------------------------------------ geometry helpers
    def arm_rp(s, fs, bl, wl):
        """body-axis vector (ft) from the aerodynamic reference point to a station"""
        return (Q(D(p('metrics/aero-rp-x-in'), v(fs)), v(12)), Q(D(v(bl), p('metrics/aero-rp-y-in')), v(12)), Q(D(p('metrics/aero-rp-z-in'), v(wl)), v(12)))

    def arm_cg(s, fs, bl, wl):
        return (Q(D(p('inertia/cg-x-in'), v(fs)), v(12)), Q(D(v(bl), p('inertia/cg-y-in')), v(12)), Q(D(p('inertia/cg-z-in'), v(wl)), v(12)))

    def wake(s, target, mirror=False):
        """the main rotor wake's induced air velocity over a target (ring-vortex tables, mrwake.py) against the wake's
        skew, the direction of the in-plane flight path and the longitudinal flapping: body-axis components to add to
        the target's velocity relative to the air (ft/s; the air moves with the wake, so they are subtracted). mirror:
        the target's mirror image across the plane of symmetry (the left end plate)."""
        g, a = s.g, s.aero
        if not a.has('aero/rotor/psi-flow-deg'):
            ps = ATAN2(p('aero/rotor/uhy'), p('aero/rotor/uhx'))
            a.fn('aero/rotor/psi-flow-deg', M(v(R2D), IFTHEN(LT(ps, v(0)), S(ps, v(2 * math.pi)), ps)),
                 'direction of the in-plane flight path at the hub, from the nose to the right')
            a.fn('aero/rotor/psi-flow-mirror-deg', D(v(360), p('aero/rotor/psi-flow-deg')))
        mw = mrwake.load(target)
        psi = 'aero/rotor/psi-flow-mirror-deg' if mirror else 'aero/rotor/psi-flow-deg'
        tag = target + ('-m' if mirror else '')
        out = []
        for i, a in enumerate('xyz'):
            tab = [[[c[i] for c in row] for row in t] for t in mw['k']]
            k = g.fn('aero/wake/%s/k%s' % (tag, a), T3('aero/rotor/chi-deg', psi, 'aero/rotor/a1-deg', mw['chi'], mw['psi'], mw['a1'], tab),
                     'main rotor induced air velocity over the %s, over its mean inflow' % target)
            sign = -1 if (mirror and a == 'y') else 1
            out.append(M(v(-sign), k, p('aero/rotor/vi-fps')))
        return out

    def add(s, F, Mo):
        for lst, x in zip((s.X, s.Y, s.Z, s.L, s.Mm, s.N), tuple(F) + tuple(Mo)):
            if x is not None:
                lst.append(x)

    # ------------------------------------------------------------ main rotor aerodynamics and hub loads
    def main_rotor(s):
        g = s.g
        bc = s.bc
        ci, si = math.cos(U.SHAFT_TILT), math.sin(U.SHAFT_TILT)
        to_s = lambda bx, by, bz: (S(M(v(ci), bx), M(v(si), bz)), by, S(M(v(-si), bx), M(v(ci), bz)))
        rx, ry, rz = s.arm_cg(U.FS_MR, U.BL_MR, U.WL_MR)
        rx, ry, rz = [g.fn('aero/rotor/hub-r%s-ft' % a, e) for a, e in zip('xyz', (rx, ry, rz))]
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        vb = (S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry))),
              S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz))),
              S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx))))
        Vhx, Vhy, Vhz = [g.fn('aero/rotor/Vh%s-fps' % a, e) for a, e in zip('xyz', to_s(*vb))]
        ps_a = g.fn('aero/rotor/ps-aero', S(M(v(ci), Pa), M(v(si), Ra)))
        qs_a = Qa
        rs_a = g.fn('aero/rotor/rs-aero', S(M(v(-si), Pa), M(v(ci), Ra)))
        Vip = g.fn('aero/rotor/Vinplane-fps', SQ(S(SQR(Vhx), SQR(Vhy), v(1e-6))))
        wdir = MIN(v(1), Q(Vip, v(5.0)))
        g.fn('aero/rotor/uhx', S(M(wdir, Q(Vhx, Vip)), D(v(1), wdir)))
        g.fn('aero/rotor/uhy', M(wdir, Q(Vhy, Vip)))
        v0 = p('rotor/v0-fps')
        Vn = g.fn('aero/rotor/Vnormal-fps', D(v0, Vhz), 'flow through the disc (down +)')
        g.fn('aero/rotor/Vprime-fps', SQ(S(SQR(Vip), SQR(Vn), v(25.0))), 'mass flow velocity (5 ft/s floor)')
        chi = g.fn('aero/rotor/chi-rad', ATAN2(Vip, MAX(Vn, v(0.5))), 'wake skew angle')
        g.fn('aero/rotor/chi-deg', M(v(R2D), chi))
        # ground effect on the induced velocity [as the UH-60 model]: hover value against the hub height (U.GE_K: scaled
        # to the Bo 105 flight manual's hover ceilings), carried into forward flight with Heyson's ratio [NASA TN D-234]
        hh = g.fn('aero/rotor/hub-agl-ft', S(p('position/h-agl-ft'), Q(D(v(U.WL_MR), p('inertia/cg-z-in')), v(12))))
        zr = g.fn('aero/rotor/ge-z-r', MAX(v(U.GE_ZR_MIN), Q(hh, v(U.R))))
        th_, ph_ = p('attitude/theta-rad'), p('attitude/phi-rad')
        dn = to_s(NEG(SIN(th_)), M(SIN(ph_), COS(th_)), M(COS(ph_), COS(th_)))
        wd = g.fn('aero/rotor/ge-wake-down', S(M(NEG(Vhx), dn[0]), M(NEG(Vhy), dn[1]), M(Vn, dn[2])))
        w2 = S(SQR(Vip), SQR(Vn))
        g.fn('aero/rotor/ge-chi-deg', M(v(R2D), ATAN2(SQ(MAX(v(0), D(w2, SQR(wd)))), wd)), 'wake angle from the ground normal')
        kh = g.fn('aero/rotor/ge-k-hover', tag('pow', S(v(1), Q(v(U.GE_K), SQR(zr))), v(U.GE_EXP)))
        g.fn('aero/rotor/kge', S(v(1), M(D(kh, v(1)), T2('aero/rotor/ge-z-r', 'aero/rotor/ge-chi-deg', HG.ZR, HG.CHI_DEG, HG.RATIO))),
             'induced velocity in / out of ground effect')
        g.fn('aero/rotor/vi-fps', M(p('aero/rotor/kge'), v0), 'uniform induced velocity with ground effect')
        # Peters-He: flow parameters and gain matrices against the wake skew X = tan(chi / 2)
        WR = g.fn('aero/rotor/ph/omega-r-fps', MAX(M(p('rotor/omega-rad_sec'), v(U.R)), v(30)))
        X = g.fn('aero/rotor/ph/X', Q(SIN(chi), S(v(1), COS(chi))))
        VT = g.fn('aero/rotor/ph/VT', Q(p('aero/rotor/Vprime-fps'), WR))
        mu, lm, la = Q(Vip, WR), Q(v0, WR), Q(Vn, WR)
        Vm = g.fn('aero/rotor/ph/V', MAX(Q(v(5), WR), Q(S(SQR(mu), M(S(la, lm), la)), VT)))
        Xs = [0.05 * i for i in range(21)]
        for st, tg, kind in ((U.PH_COS, 'c', 'c'), (U.PH_SIN, 's', 's')):
            Li = [np.linalg.inv(PH.Lmat(st, x, kind)) for x in Xs]
            for i in range(len(st)):
                for kk in range(len(st)):
                    Vk = VT if st[kk] == (1, 0) else Vm
                    g.fn('aero/rotor/ph/A%s%d-%d' % (tg, i, kk), M(T1('aero/rotor/ph/X', Xs, [float(L_[i, kk]) for L_ in Li]), Vk))
        nw = g.fn('aero/rotor/ph/vip-fps', SQ(S(SQR(Vhx), SQR(Vhy))))
        slow = LT(nw, v(0.5))
        g.fn('aero/rotor/ph/cd', IFTHEN(slow, v(1), Q(Vhx, MAX(nw, v(0.5)))))
        g.fn('aero/rotor/ph/sd', IFTHEN(slow, v(0), NEG(Q(Vhy, MAX(nw, v(0.5))))))
        ph_tau = {('c', i): [] for i in range(len(U.PH_COS))}; ph_tau.update({('s', i): [] for i in range(len(U.PH_SIN))})
        rho, a_snd = p('atmosphere/rho-slugs_ft3'), p('atmosphere/a-fps')
        W = p('rotor/omega-rad_sec')
        mach, (a_cl, cl_tab), (a_cd, cd_tab), (a_cm, cm_tab) = section_tables()
        hrp = s.arm_rp(U.FS_MR, U.BL_MR, U.WL_MR)
        hrp_s = [g.fn('aero/rotor/hubrp-%s-ft' % a, e) for a, e in zip('xyz', to_s(*hrp))]
        bp = U.PRECONE
        ph = math.radians(U.SWASH_PHASE)
        Fsum = {'x': [], 'y': [], 'z': []}; Msum = {'x': [], 'y': [], 'z': []}
        wrel = g.fn('aero/rotor/w-rel', D(W, rs_a), 'rotor speed less the body\'s yaw rate about the shaft, rad/s')
        shapes = sorted({st_ for sts in (U.PH_COS, U.PH_SIN) for st_ in sts})
        T_terms, Qa_terms, Qin_terms = [], [], []
        for k in range(U.NB):
            b = 'aero/rotor/b%d' % k
            st = 'rotor/b%d' % k
            qf, qfd, ql, qld, qt = (p(st + '/' + n) for n in ('qf', 'qfd', 'ql', 'qld', 'qt'))
            psi = g.fn(b + '/psi', S(p('rotor/psi-rad'), v(k * math.pi / 2)), 'azimuth of the blade root')
            ch, sh = g.fn(b + '/cpsi', COS(psi)), g.fn(b + '/spsi', SIN(psi))
            # the blade lags by about its tip lag / R: the aerodynamic directions and the pitch use that azimuth
            psib = g.fn(b + '/psib', S(psi, ql), 'azimuth of the lagged blade')
            c, sn = g.fn(b + '/cpsib', COS(psib)), g.fn(b + '/spsib', SIN(psib))
            th0 = g.fn(b + '/theta-deg', D(D(p('ctl/theta0-deg'), M(p('ctl/A1s-deg'), COS(S(psib, v(ph))))),
                                         M(p('ctl/B1s-deg'), SIN(S(psib, v(ph))))), 'rigid blade pitch at the shaft, deg')
            # the Peters-He states' induced velocity at the blade, by radial shape (jj, rh): the states of one shape (its
            # cosine and sine) projected on the lagged blade's azimuth and summed, so a segment reads each shape once
            trig = {1: (c, sn)}
            if any(rh == 2 for jj, rh in shapes):
                trig[2] = (g.fn(b + '/ph-c2', D(SQR(c), SQR(sn))), g.fn(b + '/ph-s2', M(v(2), c, sn)))
            tf = lambda tg, rh: v(1) if rh == 0 else trig[rh][0 if tg == 'c' else 1]
            vi_shape = {}
            for (jj, rh) in shapes:
                lam = S(*[M(p('rotor/ph/%s%d' % (tg, i)), tf(tg, rh)) if rh else p('rotor/ph/%s%d' % (tg, i))
                          for sts, tg in ((U.PH_COS, 'c'), (U.PH_SIN, 's')) for i, st_ in enumerate(sts) if st_ == (jj, rh)])
                vi_shape[(jj, rh)] = g.fn(b + '/vi-%d-%d' % (jj, rh), M(p('aero/rotor/kge'), WR, lam),
                                          'induced velocity of the inflow shape (%d, %d), ft/s' % (jj, rh))
            # a segment's velocity relative to the air is linear in its radius r and mode shapes phi_l, phi_f: in shaft axes
            # V = V0 + r V1 + phi_l V2 + phi_f V3 (the hub's motion, the rotation and the body's rates, the lag and flap
            # motion), so each blade's coefficients are worked out once, its in-plane ones turned onto the lagged blade's
            # directions (UT along its motion, UR outward), and a segment reads four of them for each component
            X = (Vhx, S(M(v(-bp), qs_a), M(wrel, sh)), S(M(wrel, ql, ch), M(qld, sh)), NEG(M(qs_a, qf)))
            Y = (Vhy, S(M(wrel, ch), M(v(bp), ps_a)), D(M(qld, ch), M(wrel, ql, sh)), M(ps_a, qf))
            UTc = [g.fn(b + '/UT%d' % n, S(M(xn, sn), M(yn, c))) for n, (xn, yn) in enumerate(zip(X, Y))]
            URc = [g.fn(b + '/UR%d' % n, D(M(yn, sn), M(xn, c))) for n, (xn, yn) in enumerate(zip(X, Y))]
            Vz1 = g.fn(b + '/Vz1', S(M(ps_a, sh), M(qs_a, ch)))
            Vz2 = g.fn(b + '/Vz2', M(D(M(ps_a, ch), M(qs_a, sh)), ql))
            qf_terms, ql_terms, qt_terms = [], [], []
            FA, FB, FC, MU, QQ = [], [], [], [], []
            for j, (rr, dr) in enumerate(s.segs):
                e = b + '/s%d' % j
                sg = bc['seg'][j]
                fl_, ff_ = sg['phi_l'], sg['phi_f']
                slope = S(v(bp), M(v(sg['dphi_f']), qf))                  # flap slope, rad
                cb, sb = g.fn(e + '/cs', COS(slope)), g.fn(e + '/ss', SIN(slope))
                lin = lambda cf: S(cf[0], M(v(rr), cf[1]), M(v(fl_), cf[2]), M(v(ff_), cf[3]))
                UT = g.fn(e + '/UT', lin(UTc))
                UR = g.fn(e + '/UR', lin(URc), 'spanwise flow (outward), ft/s')
                Vz = S(Vhz, M(v(rr), Vz1), M(v(fl_), Vz2), NEG(M(v(ff_), qfd)),
                       NEG(S(*[M(v(PH.phi(jj, rh, rr / U.R)), vi_shape[(jj, rh)]) for (jj, rh) in shapes])))
                UP = g.fn(e + '/UP', NEG(S(M(UR, sb), M(Vz, cb))), 'flow up through the blade, ft/s')
                V2 = g.fn(e + '/V2', S(tag('pow', UT, v(2)), tag('pow', UP, v(2)), v(1e-4)))
                Vv = g.fn(e + '/V', SQ(V2))
                th = S(th0, v(U.TWIST * rr / U.R), M(v(R2D * sg['phi_t']), qt))
                ar = g.fn(e + '/alpha-raw', D(th, M(v(R2D), ATAN2(UP, UT))))
                g.fn(e + '/alpha-deg', IFTHEN(GT(ar, v(180)), D(ar, v(360)), IFTHEN(LT(ar, v(-180)), S(ar, v(360)), ar)))
                g.fn(e + '/mach', MIN(Q(Vv, a_snd), v(1.0)))
                cl = g.fn(e + '/cl', T2(e + '/alpha-deg', e + '/mach', a_cl, mach, cl_tab))
                cd = g.fn(e + '/cd', S(T2(e + '/alpha-deg', e + '/mach', a_cd, mach, cd_tab), v(U.DCD)))
                cm = g.fn(e + '/cm', T2(e + '/alpha-deg', e + '/mach', a_cm, mach, cm_tab))
                r_in, r_out = s.edges[j], s.edges[j + 1]
                fl = max(0.0, min(1.0, (U.TIP_LOSS * U.R - r_in) / (r_out - r_in)))
                Lk = g.fn(e + '/Lk', M(v(0.5 * U.CHORD * dr * fl), rho, Vv, cl), 'lift / V')
                Dk = g.fn(e + '/Dk', M(v(0.5 * U.CHORD * dr), rho, cd, SQ(S(V2, tag('pow', UR, v(2))))), 'drag / V')
                Ft = g.fn(e + '/Ft', NEG(S(M(Lk, UP), M(Dk, UT))), 'force along the blade motion, lbf')
                Fn = g.fn(e + '/Fn', D(M(Lk, UT), M(Dk, UP)), 'force normal to the blade (up), lbf')
                Fr = g.fn(e + '/Fr', NEG(M(Dk, UR)), 'spanwise drag, outward, lbf')
                # moment about the elastic axis, nose up: the section's own and its normal force at the aerodynamic centre
                Mea = g.fn(e + '/Mea', D(M(v(0.5 * U.CHORD ** 2 * dr), rho, V2, cm), M(v(bc['d_ac']), Fn)))
                A_ = g.fn(e + '/A', D(M(Fr, cb), M(Fn, sb)), 'along the blade\'s radius in the plane of rotation, lbf')
                C_ = g.fn(e + '/C', S(M(Fn, cb), M(Fr, sb)), 'up, lbf')
                g.fn(e + '/Fnc', M(Fn, cb), 'normal force\'s share along the shaft, for the inflow, lbf')
                FA.append(A_); FB.append(Ft); FC.append(C_)
                MU.append(D(M(v(rr), C_), M(S(v(bp * rr), M(v(ff_), qf)), A_)))  # flap moment about the hub, up
                QQ.append(D(M(v(rr), Ft), M(v(fl_), ql, A_)))                    # moment about the shaft (up), in-plane forces
                qf_terms.append(M(v(sg['phi_f']), C_)); ql_terms.append(M(v(sg['phi_l']), Ft)); qt_terms.append(M(v(sg['phi_t']), Mea))
            g.fn(b + '/Qf', S(*qf_terms), 'generalized aerodynamic force, flap mode, lbf ft')
            g.fn(b + '/Ql', S(*ql_terms), 'generalized aerodynamic force, lag mode')
            g.fn(b + '/Qt', S(*qt_terms), 'generalized aerodynamic moment, torsion mode')
            # the inflow's forcing: each shape's moment of the normal loads, once, shared by its cosine and sine states
            for (jj, rh) in shapes:
                sh_ = g.fn(b + '/Fn-%d-%d' % (jj, rh), S(*[M(v(PH.phi(jj, rh, rr / U.R)), p('aero/rotor/b%d/s%d/Fnc' % (k, j)))
                                                        for j, (rr, dr) in enumerate(s.segs)]))
                for sts, tg in ((U.PH_COS, 'c'), (U.PH_SIN, 's')):
                    for i, st_ in enumerate(sts):
                        if st_ == (jj, rh):
                            ph_tau[(tg, i)].append(M(tf(tg, rh), sh_))
            fa = g.fn(b + '/FA', S(*FA)); fb = g.fn(b + '/FB', S(*FB)); fc = g.fn(b + '/FC', S(*FC))
            mu_a = g.fn(b + '/Mflap-aero', S(*MU), 'aerodynamic flap moment about the hub, ft lbf')
            qa = g.fn(b + '/Q', S(*QQ), 'aerodynamic moment about the shaft, in the direction of rotation, ft lbf')
            Qa_terms.append(qa); T_terms.append(fc)
            # the blade's inertial forces of its motion relative to the airframe (hub frame of the root's azimuth)
            # the modes' accelerations now, from the loads and deflections of this instant (the rotor system's are a step
            # older: at 1/rev the blade's inertial and centrifugal flap moments nearly cancel, and a step between them
            # turns the hub moment); the flapping one less its airframe angular acceleration part (added mass, ROTOR_DI)
            qfdd = g.fn(b + '/qfdd', s.flap_accel(st, s.flap_forcing(k, ch, sh), qf, qfd, ql, qld))
            qfdd_free = g.fn(b + '/qfdd-free', s.flap_accel(st, s.flap_forcing(k, ch, sh, body=False), qf, qfd, ql, qld))
            qldd = g.fn(b + '/qldd', Q(D(s.lag_free(st, s.lag_forcing(k, ch, sh), ql, qld, qf, qfd), M(v(bc['Srl']), p('rotor/omega-dot-abs'))), v(bc['Ml'])))
            # (named, as the hub loads read each two or three times)
            wr = g.fn(b + '/wr', D(M(qs_a, sh), M(ps_a, ch)), 'body rate about the blade\'s in-plane normal, rad/s')
            fr = g.fn(b + '/fr-inertial', S(M(v(bc['Sl']), p('rotor/omega-dot-abs'), ql), M(v(2 * bc['Sl']), W, qld)))
            ft = g.fn(b + '/ft-inertial', D(M(v(bc['Sl']), SQR(W), ql), M(v(bc['Sl']), qldd)))
            fn = NEG(S(M(v(bc['Sf']), qfdd), M(v(2 * bc['Sr']), W, wr)))
            mu_i = g.fn(b + '/Mflap-inertial', NEG(S(M(v(bc['Srf']), qfdd_free), M(v(2 * bc['Ib']), W, wr),
                                                      M(SQR(W), S(v(bp * bc['Ib']), M(v(bc['Srf']), qf))))))
            Qin_terms.append(NEG(M(v(bc['Srl']), qldd)))
            mflap = g.fn(b + '/Mflap', S(mu_a, mu_i), 'root flap moment on the hub (blade up +), ft lbf')
            # to shaft axes: aero with the lagged azimuth, inertial with the root's
            Fx = g.fn(b + '/Fx', S(M(fa, NEG(c)), M(fb, sn), M(fr, NEG(ch)), M(ft, sh)))
            Fy = g.fn(b + '/Fy', S(M(fa, sn), M(fb, c), M(fr, sh), M(ft, ch)))
            Fz = g.fn(b + '/Fz', NEG(S(fc, fn)))
            Fsum['x'].append(Fx); Fsum['y'].append(Fy); Fsum['z'].append(Fz)
            # moments about the reference point: the hub force's arm, and the root moments (-Mflap e_t)
            Msum['x'].append(S(D(M(hrp_s[1], Fz), M(hrp_s[2], Fy)), NEG(M(mu_a, sn)), NEG(M(mu_i, sh))))
            Msum['y'].append(S(D(M(hrp_s[2], Fx), M(hrp_s[0], Fz)), NEG(M(mu_a, c)), NEG(M(mu_i, ch))))
            Msum['z'].append(D(M(hrp_s[0], Fy), M(hrp_s[1], Fx)))
        g.fn('aero/rotor/thrust-lbs', S(*T_terms), 'aerodynamic thrust along the shaft (up)')
        # Peters-He forcing: tau = (1/2pi or 1/pi) sum of the blades' normal loads x shape / (rho W^2 R^4), over the
        # effective disc B^2 A (tip loss in the momentum balance, as the UH-60 model)
        qn = MAX(M(rho, SQR(MAX(W, v(5))), v(U.R ** 4)), v(1e-6))
        for (tg, i), terms in ph_tau.items():
            rh = (U.PH_COS if tg == 'c' else U.PH_SIN)[i][1]
            kb = 1 / U.TIP_LOSS ** 2
            g.fn('aero/rotor/ph/tau-%s%d' % (tg, i), Q(M(v(kb / (2 * math.pi) if rh == 0 else kb / math.pi), S(*terms)), qn))
        Qa = g.fn('aero/rotor/torque-ftlb', NEG(S(*Qa_terms)), 'aerodynamic torque against the rotation, ft lbf')
        g.fn('aero/rotor/torque-inertial-ftlb', S(*Qin_terms), 'the blades\' lag accelerations\' moment on the hub, rotation +')
        g.fn('aero/rotor/power-hp', Q(M(Qa, W), v(550)))
        Fs = [g.fn('aero/rotor/F%s-s' % a, S(*Fsum[a])) for a in 'xyz']
        Ms = [g.fn('aero/rotor/M%s-s' % a, S(*Msum[a])) for a in 'xyz']
        # about the shaft the airframe gets the drive torque's reaction (z down: + yaws the nose right); the blades'
        # torque about the shaft goes into the rotor speed
        Mzs = g.fn('aero/rotor/Mz-airframe-s', S(Ms[2], p('drive/mr-torque-ftlb')))
        to_b = lambda x, y, z: (D(M(v(ci), x), M(v(si), z)), y, S(M(v(si), x), M(v(ci), z)))
        Fb = [g.fn('aero/rotor/F%s-lbs' % a, e) for a, e in zip('XYZ', to_b(*Fs))]
        Mb = [g.fn('aero/rotor/M%s-ftlb' % a, e) for a, e in zip('LMN', to_b(Ms[0], Ms[1], Mzs))]
        s.add(Fb, Mb)
        # flapping harmonics of the tip path (deg) for the wash maps and the 3-D model: tip height = bp R + R qf
        g.fn('aero/rotor/a0-deg', M(v(R2D / U.NB), S(*[S(v(bp), p('rotor/b%d/qf' % k)) for k in range(U.NB)])))
        g.fn('aero/rotor/a1-deg', M(v(-0.5 * R2D), S(*[M(p('rotor/b%d/qf' % k), p('aero/rotor/b%d/cpsi' % k)) for k in range(U.NB)])))
        g.fn('aero/rotor/b1-deg', M(v(-0.5 * R2D), S(*[M(p('rotor/b%d/qf' % k), p('aero/rotor/b%d/spsi' % k)) for k in range(U.NB)])))
        # hub moments in the shaft frame (body roll and pitch), for the tests
        g.fn('aero/rotor/hub-roll-ftlb', S(*[NEG(S(M(p('aero/rotor/b%d/Mflap-aero' % k), p('aero/rotor/b%d/spsib' % k)),
                                                    M(D(p('aero/rotor/b%d/Mflap' % k), p('aero/rotor/b%d/Mflap-aero' % k)), p('aero/rotor/b%d/spsi' % k))))
                                            for k in range(U.NB)]))
        g.fn('aero/rotor/hub-pitch-ftlb', S(*[NEG(S(M(p('aero/rotor/b%d/Mflap-aero' % k), p('aero/rotor/b%d/cpsib' % k)),
                                                     M(D(p('aero/rotor/b%d/Mflap' % k), p('aero/rotor/b%d/Mflap-aero' % k)), p('aero/rotor/b%d/cpsi' % k))))
                                             for k in range(U.NB)]))

    # ------------------------------------------------------------ tail rotor [Bailey's closed form, as GenHel's]
    def tail_rotor(s):
        if U.TR_MODEL == 'blades':
            return s.tail_rotor_blades()
        s.tail_rotor_bailey()

    def tr_hub_velocity(s):
        """the tail rotor hub's velocity relative to the air (body axes): flight, rotation, the main rotor's wake"""
        g = s.g
        rx, ry, rz = s.arm_cg(U.FS_TR, U.BL_TR, U.WL_TR)
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        w = s.wake('tr') if U.WAKE_ON_AIRFRAME else (v(0), v(0), v(0))
        return [g.fn('aero/rotor/tr/V%s-fps' % a, e) for a, e in zip('xyz', (
            S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry)), w[0]),
            S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz)), w[1]),
            S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx)), w[2])))]

    def tr_wake_on_fin(s, Vt, Va, Vb, vi):
        """the tail rotor wake's skew and direction and its induced velocity, for the fin (fin_in_tr_flow)"""
        g = s.g
        g.fn('aero/rotor/tr/vi-fps', vi, 'induced velocity at the disc')
        g.fn('aero/rotor/tr/chi-deg', M(v(180 / math.pi), ATAN2(p('aero/rotor/tr/Vinplane-fps'), S(Vt, vi))),
             'wake skew from the axis (0: straight along -t; over 90: blown back through the disc, onto the fin)')
        ps = ATAN2(NEG(Vb), NEG(Va))
        g.fn('aero/rotor/tr/psi-wake-deg', M(v(180 / math.pi), IFTHEN(LT(ps, v(0)), S(ps, v(2 * math.pi)), ps)),
             'direction the wake leaves in the disc plane, from aft towards down')

    def tail_rotor_blades(s):
        """Blade-element tail rotor (as the UH-60 model's): two blades teetering on the hub (states in
        tail_rotor_blade_states), with delta-3, each of NSEG_TR segments at its own angle of attack and Mach number in the
        NACA 0012 tables of C-81's equations (data/c81_0012.py), in the air at the hub plus its own motion and the
        induced inflow. Thrust, in-plane force and the blades' moments on the hub (the torque among them: the drive's
        reaction on the airframe) all come from the blades. On the left of the fin and pushing, it draws its air past the
        fin, whose own force in that flow is the fin blockage (fin_in_tr_flow). Its shaft's 4 deg toe [TR card 57] is
        left out.

        Hub frame: t the thrust axis (right), a0 aft (a blade at zero azimuth), b0 = t x a0 (down); a blade at azimuth
        psi lies along r = cos psi a0 + sin psi b0 and moves along e = -sin psi a0 + cos psi b0 (the top blade moves
        aft); flapped by beta (towards t) it lies along d = cos b r + sin b t, normal n = -sin b r + cos b t."""
        g = s.g
        t, a0, b0 = (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)
        dot = lambda u, x: S(*[M(v(cu), xe) for cu, xe in zip(u, x) if cu != 0])
        vb = s.tr_hub_velocity()
        Va = g.fn('aero/rotor/tr/Va-fps', dot(a0, vb)); Vb = g.fn('aero/rotor/tr/Vb-fps', dot(b0, vb))
        Vt = g.fn('aero/rotor/tr/Vaxial-fps', dot(t, vb), 'hub velocity along the thrust axis (right), relative to the air')
        g.fn('aero/rotor/tr/Vinplane-fps', SQ(S(SQR(Va), SQR(Vb), v(1e-6))))
        rates = (p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec'))
        wa, wb, wt = [g.fn('aero/rotor/tr/w%s' % n, dot(u, rates)) for n, u in (('a', a0), ('b', b0), ('t', t))]
        W = g.fn('aero/rotor/tr/omega-rad_sec', M(v(U.GEAR_TR), p('rotor/omega-rad_sec')))
        v0 = p('rotor/tr/v0-fps')
        if U.TR_FIN:
            s.tr_wake_on_fin(Vt, Va, Vb, v0)
        rho, a_snd = p('atmosphere/rho-slugs_ft3'), p('atmosphere/a-fps')
        segs, edges = U.segments(U.NSEG_TR, U.R_ROOT_TR, U.R_TR)
        a_rows, mach, cl_tab, cd_tab = AF12.tables()
        B = U.TIP_LOSS_TR_BLADES
        Fa, Fb, Ft_, Ma, Mb, Mt = [], [], [], [], [], []
        for k in range(U.NB_TR):
            b = 'aero/rotor/tr/b%d' % k
            sgn = 1 if k == 0 else -1                                 # teetering: blade 1 flaps the other way
            psi = g.fn(b + '/psi', S(p('rotor/tr/psi-rad'), v(k * math.pi)))
            c, sn = g.fn(b + '/cpsi', COS(psi)), g.fn(b + '/spsi', SIN(psi))
            beta = g.fn(b + '/beta-rad', M(v(sgn), p('rotor/tr/beta-rad')))
            bd = g.fn(b + '/betadot', M(v(sgn), p('rotor/tr/betadot-rad_sec')))
            cb, sb = g.fn(b + '/cbeta', COS(beta)), g.fn(b + '/sbeta', SIN(beta))
            vr = g.fn(b + '/Vr', S(M(c, Va), M(sn, Vb)))
            ve = g.fn(b + '/Ve', S(NEG(M(sn, Va)), M(c, Vb)))
            wr, we = S(M(c, wa), M(sn, wb)), g.fn(b + '/we', S(NEG(M(sn, wa)), M(c, wb)))
            vn = g.fn(b + '/Vn', S(NEG(M(sb, vr)), M(cb, S(Vt, v0))), 'hub velocity and inflow along the blade normal')
            vd = g.fn(b + '/Vd', S(M(cb, vr), M(sb, S(Vt, v0))), 'spanwise')
            rote = g.fn(b + '/rot-e', D(M(wt, cb), M(wr, sb)), 'body rotation: velocity along e per ft of radius')
            th0 = g.fn(b + '/theta-deg', D(p('ctl/thetaTR-deg'), M(v(math.tan(U.DELTA3_TR) * R2D), beta)),
                       'pitch: the pedals\' less tan(delta-3) x flap')
            fn_j, ft_j, fr_j, mf_j, mt_j = [], [], [], [], []
            for j, (rr, dr) in enumerate(segs):
                e = b + '/s%d' % j
                UT = g.fn(e + '/UT', S(ve, M(v(rr), rote), M(W, v(rr), cb)))
                UP = g.fn(e + '/UP', S(vn, NEG(M(v(rr), we)), M(v(rr), bd)), 'flow through the blade, from the thrust side +')
                V2 = g.fn(e + '/V2', S(SQR(UT), SQR(UP), v(1e-4)))
                Vv = g.fn(e + '/V', SQ(V2))
                th = S(th0, v(U.TWIST_TR * rr / U.R_TR)) if U.TWIST_TR else th0
                ar = g.fn(e + '/alpha-raw', D(th, M(v(R2D), ATAN2(UP, UT))))
                g.fn(e + '/alpha-deg', IFTHEN(GT(ar, v(180)), D(ar, v(360)), IFTHEN(LT(ar, v(-180)), S(ar, v(360)), ar)))
                g.fn(e + '/mach', MIN(Q(Vv, a_snd), v(1.0)))
                cl = g.fn(e + '/cl', T2(e + '/alpha-deg', e + '/mach', a_rows, mach, cl_tab))
                cd = g.fn(e + '/cd', T2(e + '/alpha-deg', e + '/mach', a_rows, mach, cd_tab))
                r_in, r_out = edges[j], edges[j + 1]
                fl = max(0.0, min(1.0, (B * U.R_TR - r_in) / (r_out - r_in)))
                Lk = g.fn(e + '/Lk', M(v(0.5 * U.CHORD_TR * dr * fl), rho, Vv, cl), 'lift / V')
                Dk = g.fn(e + '/Dk', M(v(0.5 * U.CHORD_TR * dr), rho, cd, SQ(S(V2, SQR(vd)))), 'drag / V')
                Fn = g.fn(e + '/Fn', D(M(Lk, UT), M(Dk, UP)), 'force along the blade normal (thrust side), lbf')
                Ft = g.fn(e + '/Ft', NEG(S(M(Lk, UP), M(Dk, UT))), 'force along the blade motion, lbf')
                fn_j.append(Fn); ft_j.append(Ft); fr_j.append(NEG(M(Dk, vd))); mf_j.append(M(v(rr), Fn)); mt_j.append(M(v(rr), Ft))
            Fn_k, Ft_k, Fr_k = g.fn(b + '/Fn', S(*fn_j)), g.fn(b + '/Ft', S(*ft_j)), g.fn(b + '/Fr', S(*fr_j))
            g.fn(b + '/Mflap', S(*mf_j), 'aerodynamic flap moment about the teeter axis, ft lbf (towards the thrust side +)')
            Mte = g.fn(b + '/Me', S(*mt_j), 'moment of the forces along the blade motion, ft lbf')
            # on the hub (teetering: no flap moment): forces, and r Ft (cos b t - sin b r)
            rad = g.fn(b + '/F-radial', D(M(Fr_k, cb), M(Fn_k, sb)))
            Fa.append(D(M(rad, c), M(Ft_k, sn))); Fb.append(S(M(rad, sn), M(Ft_k, c))); Ft_.append(S(M(Fn_k, cb), M(Fr_k, sb)))
            Ma.append(NEG(M(Mte, sb, c))); Mb.append(NEG(M(Mte, sb, sn))); Mt.append(M(Mte, cb))
        Fha, Fhb = g.fn('aero/rotor/tr/Fa-lbs', S(*Fa), 'in-plane force, aft'), g.fn('aero/rotor/tr/Fb-lbs', S(*Fb), 'in-plane force, down')
        T = g.fn('aero/rotor/tr/thrust-lbs', S(*Ft_), 'the blades\' thrust (right)')
        Mha, Mhb, Mht = [g.fn('aero/rotor/tr/M%s-ftlb' % n, S(*x)) for n, x in (('a', Ma), ('b', Mb), ('t', Mt))]
        Qtr = g.fn('aero/rotor/tr/torque-ftlb', NEG(Mht), 'aerodynamic torque (against the rotation)')
        g.fn('aero/rotor/tr/power-hp', Q(M(Qtr, W), v(550)))
        # body axes: x = -a, y = t, z = b
        Fx, Fy, Fz = [g.fn('aero/rotor/tr/F%s-lbs' % n, e) for n, e in zip('XYZ', (NEG(Fha), T, Fhb))]
        Mx, My, Mz = NEG(Mha), Mht, Mhb
        ax, ay, az = s.arm_rp(U.FS_TR, U.BL_TR, U.WL_TR)
        Mo = (S(D(M(ay, Fz), M(az, Fy)), Mx), S(D(M(az, Fx), M(ax, Fz)), My), S(D(M(ax, Fy), M(ay, Fx)), Mz))
        s.add((Fx, Fy, Fz), Mo)

    def tail_rotor_bailey(s):
        """Two-bladed teetering tail rotor on the left of the fin, thrust to the right (a pusher). Bailey's
        closed-form solution with uniform inflow (states in tail_rotor_states): teetering, it has no coning, so its
        delta-3 changes only the cyclic flapping, which the closed form leaves out."""
        g = s.g
        rx, ry, rz = s.arm_cg(U.FS_TR, U.BL_TR, U.WL_TR)
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        w = s.wake('tr') if U.WAKE_ON_AIRFRAME else (v(0), v(0), v(0))
        vb = [g.fn('aero/rotor/tr/V%s-fps' % a, e) for a, e in zip('xyz', (
            S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry)), w[0]),
            S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz)), w[1]),
            S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx)), w[2])))]
        Vax = g.fn('aero/rotor/tr/Vaxial-fps', vb[1], 'along the thrust axis (right)')
        Vip = g.fn('aero/rotor/tr/Vinplane-fps', SQ(S(SQR(vb[0]), SQR(vb[2]), v(1e-6))))
        ORv = g.fn('aero/rotor/tr/tipspeed-fps', M(v(U.GEAR_TR * U.R_TR), p('rotor/omega-rad_sec')))
        ORs = MAX(ORv, v(1))
        mu = g.fn('aero/rotor/tr/mu', Q(Vip, ORs))
        g.fn('aero/rotor/tr/muz', Q(NEG(Vax), ORs), 'flow through the disc towards the thrust side')
        dw = p('rotor/tr/dw')
        if U.TR_FIN:
            s.tr_wake_on_fin(Vax, NEG(vb[0]), vb[2], M(dw, ORv))
        lam = g.fn('aero/rotor/tr/lambda', D(p('aero/rotor/tr/muz'), dw))
        A = math.pi * U.R_TR ** 2
        rho = p('atmosphere/rho-slugs_ft3')
        ct = g.fn('aero/rotor/tr/ct', M(v(2 * U.TIP_LOSS_TR ** 2), dw, SQ(S(SQR(mu), SQR(lam)))))
        T = g.fn('aero/rotor/tr/thrust-lbs', M(v(A), rho, SQR(ORv), ct))
        cq = S(NEG(M(lam, ct)), M(v(U.SIGMA_TR * U.CD0_TR / 8), S(v(1), M(v(4.65), SQR(mu)))))
        g.fn('aero/rotor/tr/torque-ftlb', M(v(A * U.R_TR), rho, SQR(ORv), cq))
        g.fn('aero/rotor/tr/power-hp', Q(M(p('aero/rotor/tr/torque-ftlb'), p('rotor/omega-rad_sec'), v(U.GEAR_TR)), v(550)))
        ax, ay, az = s.arm_rp(U.FS_TR, U.BL_TR, U.WL_TR)
        s.add((None, T, None), (NEG(M(az, T)), None, M(ax, T)))

    def fin_in_tr_flow(s):
        """The fin blockage, as the UH-60 model's: the fin's own lift and drag in the tail rotor's flow. The 1/4-scale
        model's data carry the fin without a tail rotor [TR fig. 14], so the tail rotor's induced air velocity over the
        fin (ring-vortex tables, trwake.py: the pusher's inflow, or its wake with the thrust reversed) adds, strip by
        strip, the difference of the fin's forces with it and without it, in the local flow (with the main rotor's wake
        over the fin, mrwake.py)."""
        if not U.TR_FIN:
            return
        import trwake
        g = s.g
        tw = trwake.load()
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        kw = s.wake('fin') if U.WAKE_ON_AIRFRAME else (v(0), v(0), v(0))
        vtr = p('aero/rotor/tr/vi-fps')
        xs, cl, cd = U.fin_tables()
        xs_l, ys_l = _mirror180(*_odd(xs, cl), negate=True)
        xs_d, ys_d = _mirror180(*_even(xs, cd), negate=False)
        rho = p('atmosphere/rho-slugs_ft3')
        FY = []
        for i, st in enumerate(tw['strips']):
            tg = 'aero/emp/fin%d' % i
            rx, ry, rz = s.arm_cg(st['fs'], 0.0, st['wl'])
            Vx0 = g.fn(tg + '/Vx-fps', S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry)), kw[0]))
            Vy0 = g.fn(tg + '/Vy-fps', S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz)), kw[1]))
            ex = g.fn(tg + '/k-tr-x', T2('aero/rotor/tr/chi-deg', 'aero/rotor/tr/psi-wake-deg', tw['chi'], tw['psi'], st['kx']))
            ey = g.fn(tg + '/k-tr-y', T2('aero/rotor/tr/chi-deg', 'aero/rotor/tr/psi-wake-deg', tw['chi'], tw['psi'], st['ky']),
                      'the tail rotor\'s induced air velocity over the strip, body y, over its inflow')
            F = []
            for case, (Vx, Vy) in (('free', (Vx0, Vy0)), ('tr', (D(Vx0, M(ex, vtr)), D(Vy0, M(ey, vtr))))):
                e = '%s/%s' % (tg, case)
                Vx, Vy = g.fn(e + '/Vx-fps', Vx), g.fn(e + '/Vy-fps', Vy)
                Vn = g.fn(e + '/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vy))), v(0.01)))
                br = g.fn(e + '/beta-raw', S(M(v(R2D), ATAN2(Vy, Vx)), v(U.FIN_INC)))
                g.fn(e + '/beta-deg', IFTHEN(GT(br, v(180)), D(br, v(360)), IFTHEN(LT(br, v(-180)), S(br, v(360)), br)))
                c_l = g.fn(e + '/cl', T1(e + '/beta-deg', xs_l, ys_l))
                c_d = g.fn(e + '/cd', T1(e + '/beta-deg', xs_d, ys_d))
                qs = g.fn(e + '/qS', M(v(0.5 * st['area']), rho, SQR(Vn)))
                # lift (sb, -cb, 0): positive with the flow from the right pushes the tail left
                F.append((g.fn(e + '/Fx-lbs', M(qs, Q(D(M(c_l, Vy), M(c_d, Vx)), Vn))),
                          g.fn(e + '/Fy-lbs', M(qs, Q(NEG(S(M(c_l, Vx), M(c_d, Vy))), Vn)))))
            Fx = g.fn(tg + '/dFx-lbs', D(F[1][0], F[0][0]), 'the tail rotor\'s flow\'s share of the strip\'s force')
            Fy = g.fn(tg + '/dFy-lbs', D(F[1][1], F[0][1]))
            rx, ry, rz = s.arm_rp(st['fs'], 0.0, st['wl'])
            s.add((Fx, Fy, None), (NEG(M(rz, Fy)), M(rz, Fx), D(M(rx, Fy), M(ry, Fx))))
            FY.append(Fy)
        g.fn('aero/emp/fin/dFy-lbs', S(*FY), 'the fin blockage: side force from the tail rotor\'s flow (right +)')

    def airframe(s):
        """The fuselage, the hub's and the skids' drag, the empennage and the fin in the tail rotor's flow, in a system
        channel run every AIRFRAME_EXECRATE frames: their loads change slowly against the rotors', and in FlightGear
        every property read takes a lock. The channel sums them into six loads for the aerodynamic axes."""
        s.air.channel('airframe', execrate=U.AIRFRAME_EXECRATE)
        s.g = ChannelGraph(s.air)
        axes = (s.X, s.Y, s.Z, s.L, s.Mm, s.N)
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = [], [], [], [], [], []
        s.fuselage()
        s.empennage()
        s.fin_in_tr_flow()
        loads = [s.g.fn('aero/airframe/%s' % n, S(*terms) if terms else v(0))
                 for n, terms in zip(('FX-lbs', 'FY-lbs', 'FZ-lbs', 'ML-ftlb', 'MM-ftlb', 'MN-ftlb'), (s.X, s.Y, s.Z, s.L, s.Mm, s.N))]
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = axes
        s.g = s.aero
        s.add(loads[:3], loads[3:])

    # ------------------------------------------------------------ fuselage [TR fig. 14, c81_fuselage]
    def fuselage(s):
        """The 1/4-scale model's coefficients (with the fin) at their reference point, wind axes, in the local flow;
        flying backwards they are mirrored front to back (drag still opposes the motion; lift, pitching and rolling
        moments fade through zero)."""
        g = s.g
        t = FUS.tables()
        kw = s.wake('fus') if U.WAKE_ON_AIRFRAME else None
        Vx = g.fn('aero/fus/Vx-fps', p('velocities/u-aero-fps') if kw is None else S(p('velocities/u-aero-fps'), kw[0]))
        Vy = g.fn('aero/fus/Vy-fps', p('velocities/v-aero-fps') if kw is None else S(p('velocities/v-aero-fps'), kw[1]))
        Vz = g.fn('aero/fus/Vz-fps', p('velocities/w-aero-fps') if kw is None else S(p('velocities/w-aero-fps'), kw[2]))
        Vxz = g.fn('aero/fus/Vxz-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
        V = g.fn('aero/fus/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vy), SQR(Vz))), v(0.01)))
        g.fn('aero/fus/alpha-deg', M(v(R2D), ATAN2(Vz, ABS(Vx))), 'tunnel pitch angle (front-back mirrored)')
        g.fn('aero/fus/psi-deg', M(v(-R2D), ATAN2(Vy, Vxz)), 'tunnel yaw angle = -sideslip (nose right of the wind +)')
        g.fn('aero/fus/psi-abs-deg', ABS(p('aero/fus/psi-deg')))
        kf = g.fn('aero/fus/kfwd', MAX(v(-1), MIN(v(1), Q(Vx, v(3.0)))))
        q = g.fn('aero/fus/qbar-psf', M(v(0.5), p('atmosphere/rho-slugs_ft3'), SQR(V)))
        A_, P_, PA = 'aero/fus/alpha-deg', 'aero/fus/psi-deg', 'aero/fus/psi-abs-deg'
        Dq = g.fn('aero/fus/D-ft2', S(T1(A_, t['alpha'], t['D0']), T1(PA, t['psi_abs'], t['dD'])))
        Lq = g.fn('aero/fus/L-ft2', M(kf, S(T1(A_, t['alpha'], t['L0']), T1(PA, t['psi_abs'], t['dL']))))
        Mq = g.fn('aero/fus/M-ft3', M(kf, T2(A_, PA, t['alpha'], t['psi_abs'], t['M'])))
        Yq = g.fn('aero/fus/Y-ft2', T1(P_, t['psi'], t['Y']))
        Rq = g.fn('aero/fus/R-ft3', M(kf, T1(P_, t['psi'], t['R'])))
        Nq = g.fn('aero/fus/N-ft3', T1(P_, t['psi'], t['N']))
        ca, sa = Q(Vx, Vxz), Q(Vz, Vxz)
        cb, sb = Q(Vxz, V), Q(Vy, V)
        c1 = (Q(Vx, V), Q(Vy, V), Q(Vz, V))
        c2 = (M(ca, sb), NEG(cb), M(sa, sb))
        c3 = (NEG(sa), v(0), ca)
        F = [g.fn('aero/fus/F%s-lbs' % a, M(q, NEG(S(M(Dq, c1[i]), M(Yq, c2[i]), M(Lq, c3[i]))))) for i, a in enumerate('XYZ')]
        Mw = [g.fn('aero/fus/M%s-wind' % a, M(q, S(M(Rq, c1[i]), NEG(M(Mq, c2[i])), M(Nq, c3[i])))) for i, a in enumerate('LMN')]
        rx, ry, rz = s.arm_rp(FUS.FS_REF, FUS.BL_REF, FUS.WL_REF)
        Mo = (S(Mw[0], D(M(ry, F[2]), M(rz, F[1]))), S(Mw[1], D(M(rz, F[0]), M(rx, F[2]))), S(Mw[2], D(M(rx, F[1]), M(ry, F[0]))))
        Mo = [g.fn('aero/fus/%s-ftlb' % a, e) for a, e in zip('LMN', Mo)]
        s.add(F, Mo)
        # rotor hub, shaft and controls with the blades off: 4.2 ft^2 [TM-110356 aerodynamic tares, DRAGH at 0 deg]
        qh = g.fn('aero/hub/qbar-psf', M(v(0.5), p('atmosphere/rho-slugs_ft3'), S(SQR(p('velocities/u-aero-fps')), SQR(p('velocities/v-aero-fps')))))
        uv = SQ(S(SQR(p('velocities/u-aero-fps')), SQR(p('velocities/v-aero-fps')), v(1e-4)))
        Fx = g.fn('aero/hub/Fx-lbs', NEG(M(v(U.HUB_DRAG_FT2), qh, Q(p('velocities/u-aero-fps'), uv))))
        Fy = g.fn('aero/hub/Fy-lbs', NEG(M(v(U.HUB_DRAG_FT2), qh, Q(p('velocities/v-aero-fps'), uv))))
        hx, hy, hz = s.arm_rp(U.FS_MR, U.BL_MR, U.WL_MR + 4.0)
        s.add((Fx, Fy, None), (NEG(M(hz, Fy)), M(hz, Fx), D(M(hx, Fy), M(hy, Fx))))
        # skid landing gear: cross-flow drag of its tubes per body axis [U.GEAR_F], in the fuselage's local flow
        rho = p('atmosphere/rho-slugs_ft3')
        Fg = [g.fn('aero/gear/F%s-lbs' % a, M(v(-0.5 * f), rho, Vc, ABS(Vc))) for a, f, Vc in zip('XYZ', U.GEAR_F, (Vx, Vy, Vz))]
        gx, gy, gz = s.arm_rp(U.FS_GEAR, 0.0, U.WL_GEAR)
        s.add(Fg, (D(M(gy, Fg[2]), M(gz, Fg[1])), D(M(gz, Fg[0]), M(gx, Fg[2])), D(M(gx, Fg[1]), M(gy, Fg[0]))))

    # ------------------------------------------------------------ horizontal stabiliser and its end plates
    def empennage(s):
        """Each surface a 2-D strip in its own plane with lift and drag tables over +-180 deg, in the local flow
        (the main rotor's wake where s.stab_wash is given)."""
        g = s.g
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')

        def local(tag, fs, bl, wl, wash=None):
            rx, ry, rz = s.arm_cg(fs, bl, wl)
            w = wash or (v(0), v(0), v(0))
            vb = (S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry)), w[0]),
                  S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz)), w[1]),
                  S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx)), w[2]))
            return [g.fn('aero/emp/%s/V%s-fps' % (tag, a), e) for a, e in zip('xyz', vb)]
        # stabiliser: symmetric section, 3-D lift slope 0.0836/deg from the C-81 section's 0.103/deg at aspect ratio
        # 8.09 with end plates [TR cards 72, 77], stalling at 13 deg; flat plate beyond (normal force 1.2)
        xs, cl, cd = U.stab_tables()
        Vx, Vy, Vz = local('h', U.FS_H, U.BL_H, U.WL_H, s.wake('stab') if U.WAKE_ON_AIRFRAME else None)
        Vn = g.fn('aero/emp/h/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
        ah = g.fn('aero/emp/h/alpha-raw', S(M(v(R2D), ATAN2(Vz, Vx)), v(U.INC_H)))
        g.fn('aero/emp/h/alpha-deg', IFTHEN(GT(ah, v(180)), D(ah, v(360)), IFTHEN(LT(ah, v(-180)), S(ah, v(360)), ah)))
        c_l = g.fn('aero/emp/h/cl', T1('aero/emp/h/alpha-deg', *_mirror180(*_odd(xs, cl), negate=True)))
        c_d = g.fn('aero/emp/h/cd', T1('aero/emp/h/alpha-deg', *_mirror180(*_even(xs, cd), negate=False)))
        qs = g.fn('aero/emp/h/qS', M(v(0.5 * U.S_H), p('atmosphere/rho-slugs_ft3'), SQR(Vn)))
        Fx = g.fn('aero/emp/h/Fx-lbs', M(qs, Q(D(M(c_l, Vz), M(c_d, Vx)), Vn)))
        Fz = g.fn('aero/emp/h/Fz-lbs', M(qs, Q(NEG(S(M(c_l, Vx), M(c_d, Vz))), Vn)))
        rx, ry, rz = s.arm_rp(U.FS_H, U.BL_H, U.WL_H)
        s.add((Fx, None, Fz), (M(ry, Fz), D(M(rz, Fx), M(rx, Fz)), NEG(M(ry, Fx))))
        # end plates: two low aspect ratio vertical plates at the stabiliser's tips [EA: stabiliser with end plates]
        xs, cl, cd = U.endplate_tables()
        for side, bl in (('l', -U.BL_EP), ('r', U.BL_EP)):
            tg = 'ep' + side
            Vx, Vy, Vz = local(tg, U.FS_EP, bl, U.WL_EP, s.wake('ep', mirror=(side == 'l')) if U.WAKE_ON_AIRFRAME else None)
            Vn = g.fn('aero/emp/%s/V-fps' % tg, MAX(SQ(S(SQR(Vx), SQR(Vy))), v(0.01)))
            g.fn('aero/emp/%s/beta-deg' % tg, M(v(R2D), ATAN2(Vy, Vx)))
            c_l = g.fn('aero/emp/%s/cl' % tg, T1('aero/emp/%s/beta-deg' % tg, *_mirror180(*_odd(xs, cl), negate=True)))
            c_d = g.fn('aero/emp/%s/cd' % tg, T1('aero/emp/%s/beta-deg' % tg, *_mirror180(*_even(xs, cd), negate=False)))
            qs = g.fn('aero/emp/%s/qS' % tg, M(v(0.5 * U.S_EP), p('atmosphere/rho-slugs_ft3'), SQR(Vn)))
            Fx = g.fn('aero/emp/%s/Fx-lbs' % tg, M(qs, Q(D(M(c_l, Vy), M(c_d, Vx)), Vn)))
            Fy = g.fn('aero/emp/%s/Fy-lbs' % tg, M(qs, Q(NEG(S(M(c_l, Vx), M(c_d, Vy))), Vn)))
            rx, ry, rz = s.arm_rp(U.FS_EP, bl, U.WL_EP)
            s.add((Fx, Fy, None), (NEG(M(rz, Fy)), M(rz, Fx), D(M(rx, Fy), M(ry, Fx))))

    def axes(s):
        def ax(name, terms):
            return '<axis name="%s">%s</axis>' % (name, ''.join('<function>%s</function>' % t for t in terms))
        return '\n'.join((ax('X', s.X), ax('Y', s.Y), ax('Z', s.Z), ax('ROLL', s.L), ax('PITCH', s.Mm), ax('YAW', s.N)))

    # ------------------------------------------------------------ outputs for FlightGear (model, instruments)
    def outputs(s):
        """read once a frame: worked out every U.SLOW_EXECRATE frames"""
        o = FCS('Bo 105 outputs')
        o.channel('rotor', execrate=U.SLOW_EXECRATE)
        RPM = 60 / (2 * math.pi)
        W = p('rotor/omega-rad_sec')
        o.fn('out/rotor-rpm', M(v(RPM), W), out='/rotors/main/rpm')
        o.fn('out/rotor-rpm-pct', M(v(100 / U.OMEGA), W), out='/rotors/main/rpm-pct')
        o.fn('out/tail-rpm', M(v(RPM * U.GEAR_TR), W), out='/rotors/tail/rpm')
        wrap = lambda e: IFTHEN(GT(e, v(360)), D(e, v(360)), e)
        # the blades' droop under their weight beyond the first flap mode's, quasi-static (blademodes.droop_residual):
        # at rest the mode alone held the tips up on the precone; above 40 % rotor speed it is under 0.01 deg
        pcts = [0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 80, 100, 120]
        droop = o.fn('out/flap-droop-deg', M(v(-R2D / G0), p('rotor/Az'), T1('rotor/omega-rad_sec', [U.OMEGA * x / 100 for x in pcts],
                                                                             blademodes.droop_residual(pcts))),
                     desc='the blades\' static droop beyond the flap mode\'s, per the hub\'s load factor along the shaft')
        for k in range(U.NB):
            o.fn('out/b%d-position' % k, wrap(S(M(v(R2D), p('rotor/psi-rad')), v(90.0 * k))), out='/rotors/main/blade[%d]/position-deg' % k)
            o.fn('out/b%d-flap' % k, S(M(v(R2D), S(v(U.PRECONE), p('rotor/b%d/qf' % k))), droop), out='/rotors/main/blade[%d]/flap-deg' % k)
            o.fn('out/b%d-lag' % k, M(v(R2D), p('rotor/b%d/ql' % k)), out='/rotors/main/blade[%d]/lag-deg' % k)
            o.fn('out/b%d-pitch' % k, p('aero/rotor/b%d/theta-deg' % k), out='/rotors/main/blade[%d]/incidence-deg' % k)
        for k in range(U.NB_TR):
            o.fn('out/tr-b%d-position' % k, wrap(S(M(v(R2D), p('rotor/tr/psi-rad')), v(180.0 * k))), out='/rotors/tail/blade[%d]/position-deg' % k)
        if U.TR_MODEL == 'blades':
            for k in range(U.NB_TR):
                o.fn('out/tr-b%d-pitch' % k, p('aero/rotor/tr/b%d/theta-deg' % k), out='/rotors/tail/blade[%d]/incidence-deg' % k)
            o.fn('out/tr-teeter', M(v(R2D), p('rotor/tr/beta-rad')), out='/rotors/tail/teeter-deg')
        else:
            o.fn('out/tr-incidence', p('ctl/thetaTR-deg'), out=['/rotors/tail/blade[0]/incidence-deg', '/rotors/tail/blade[1]/incidence-deg'])
        o.fn('out/total-torque', p('aero/rotor/torque-ftlb'), out='/rotors/gear/total-torque')
        # the mast moment indicator's signal: the hub moment off the shaft axis, % of its red radial [U.MM_LIMIT_NM]
        o.fn('out/mast-moment-pct', M(v(100 * 1.35582 / U.MM_LIMIT_NM), SQ(S(SQR(p('aero/rotor/Mx-s')), SQR(p('aero/rotor/My-s'))))),
             out='/rotors/main/mast-moment-pct')
        o.channel('engines', execrate=U.SLOW_EXECRATE)
        for n in range(2):
            g, e = 'eng[%d]' % n, '/engines/engine[%d]' % n
            o.fn('out/%s/n1' % g, p(g + '/ng-pct'), out=e + '/n1-pct')
            o.fn('out/%s/n2' % g, p(g + '/np-pct'), out=e + '/n2-pct')
            o.fn('out/%s/tot' % g, p(g + '/tgt-degc'), out=e + '/tot-degc')
            o.fn('out/%s/trq' % g, p(g + '/torque-pct'), out=e + '/torque-pct')
            o.fn('out/%s/ff' % g, p(g + '/fuel-flow-pph'), out=e + '/fuel-flow-pph')
            o.fn('out/%s/run' % g, p(g + '/lit'), out=[e + '/running', e + '/on'])
        # the cockpit's gauges (MBB_Bo_105/Instruments): the dual torque meter (one needle property, % per engine) and the
        # power turbines' speed, which the gyro instruments' vacuum source reads as /engines/engine/rpm (33 290 rpm = 100 %)
        o.fn('out/torque-gauge', M(v(0.5), S(p('eng[0]/torque-pct'), p('eng[1]/torque-pct'))), out='/sim/model/bo105/torque-pct')
        o.fn('out/engine-rpm', M(v(332.9 / 2), S(p('eng[0]/np-pct'), p('eng[1]/np-pct'))), out='/engines/engine/rpm')
        # cockpit vibration (RMS g) for the view shake and the rattle [as the UH-60 model's]: the 6-30 Hz part of the
        # acceleration at the pilot seat, which the blades' 4-per-rev hub loads put there, plus the transition
        # vibration around translational lift that the inflow model cannot make (blade-vortex interaction, a band
        # about 0.9 of the hover induced velocity wide 0.5 of it; the level is an estimate)
        o.channel('vibration', execrate=U.SLOW_EXECRATE)
        hp = []
        for ax in 'xyz':
            o.washout('vib/%s-hp1' % ax, 'accelerations/a-pilot-%s-ft_sec2' % ax, 1 / (2 * math.pi * 6))
            hp.append(o.washout('vib/%s-hp' % ax, 'vib/%s-hp1' % ax, 1 / (2 * math.pi * 6)))
        o.fn('vib/ms', S(*[SQR(h) for h in hp]))
        o.lag('vib/ms-avg', 'vib/ms', 0.25)
        o.fn('vib/rotor-g', Q(SQ(MAX(v(0), p('vib/ms-avg'))), v(G0)))
        A = math.pi * U.R ** 2
        vh = SQ(MAX(v(1), Q(p('aero/rotor/thrust-lbs'), M(v(2 * A), p('atmosphere/rho-slugs_ft3')))))
        mu = Q(p('aero/rotor/Vinplane-fps'), MAX(vh, v(5)))
        o.fn('vib/etl', tag('exp', NEG(SQR(Q(D(mu, v(0.9)), v(0.5))))), desc='translational lift band')
        thrust = MAX(v(0), MIN(v(1.5), Q(p('aero/rotor/thrust-lbs'), v(U.GW_DESIGN))))
        o.fn('vib/bvi-g', M(v(0.05), p('vib/etl'), thrust, GT(W, v(0.5 * U.OMEGA))))
        # (faded in over the first seconds: the filters and the aircraft settling on its skids start from rest)
        fade = MAX(v(0), MIN(v(1), Q(D(p('simulation/sim-time-sec'), v(1.0)), v(3.0))))
        o.fn('vib/level-g', M(fade, D(v(1), p('ctl/init')), SQ(S(SQR(p('vib/rotor-g')), SQR(p('vib/bvi-g'))))),
             out='/sim/model/bo105/vibration-g')
        return o


# ---------------------------------------------------------------- mass, fuel and gear
STRUCTURE = [('NOSE', (-2.76, 0.0, 0.25)), ('CHIN_LEFT', (-2.3, -0.6, -0.5)), ('CHIN_RIGHT', (-2.3, 0.6, -0.5)),
             ('BELLY_LEFT', (0.8, -0.75, -0.62)), ('BELLY_RIGHT', (0.8, 0.75, -0.62)),
             ('ROOF_LEFT', (-1.0, -0.75, 1.15)), ('ROOF_RIGHT', (-1.0, 0.75, 1.15)),
             ('ROTOR_HEAD', (-U.MODEL_DX, 0.0, 1.93)), ('TAIL_BOOM', (3.5, 0.0, 0.5)),
             ('STAB_LEFT', (4.45, -1.09, 0.77)), ('STAB_RIGHT', (4.45, 1.09, 0.77)), ('FIN_TOP', (5.9, 0.0, 2.0))]
# skid tubes: the lowest points of the 3-D model's skid tubes near their ends (model frame)
SKIDS = [('LEFT_FRONT', (-1.65, -1.15, -1.026)), ('RIGHT_FRONT', (-1.65, 1.15, -1.026)),
         ('LEFT_REAR', (0.55, -1.15, -1.026)), ('RIGHT_REAR', (0.55, 1.15, -1.026))]
TAIL_SKID = (5.748, 0.0, 0.292)


# the blades' rotary inertia about the shaft's in-plane axes carried by their first flap modes, slug ft^2 (NB/2 Srf^2/Mf:
# 317 of the rigid blades' 324)
_bc = blademodes.rotor_constants([r for r, dr in U.segments()[0]])
ROTOR_DI = U.NB / 2 * _bc['Srf'] ** 2 / _bc['Mf']


def mass_config(cg=None, weight=None, inertia=None):
    """Empty weight, its CG and inertias such that the default loading is the DLR Bo 105-S123's (2200 kg, AR-280)
    at the nominal CG. For a test configuration everything sits at its CG."""
    I0 = inertia or (U.IXX, U.IYY, U.IZZ, U.IXZ)
    # the blades' flapping inertia that the flap modes carry about the shaft's in-plane axes is theirs, not the airframe's
    I0 = (I0[0] - ROTOR_DI * math.cos(U.SHAFT_TILT) ** 2, I0[1] - ROTOR_DI, I0[2] - ROTOR_DI * math.sin(U.SHAFT_TILT) ** 2, I0[3])
    if cg:
        pms = [(n, 0.0, mx, cg[0], cg[1], cg[2]) for n, w, mx, x, y, z in U.POINT_MASSES]
        tanks = [(w, cg[0], cg[1], cg[2]) for w, x, y, z in U.TANKS]
        return dict(empty=weight - sum(t[0] for t in tanks), cg=cg, I=I0, pms=pms, tanks=tanks)
    loads = [(w, x, y, z) for n, w, mx, x, y, z in U.POINT_MASSES] + list(U.TANKS)
    total = U.GW_DESIGN
    we = total - sum(l[0] for l in loads)
    ex = (total * U.FS_CG - sum(w * x for w, x, y, z in loads)) / we
    ez = (total * U.WL_CG - sum(w * z for w, x, y, z in loads)) / we
    ey = -sum(w * y for w, x, y, z in loads) / we

    def para(w, x, y, z):
        m = w / G0; dx, dy, dz = (x - U.FS_CG) / 12, y / 12, (z - U.WL_CG) / 12
        return m * (dy * dy + dz * dz), m * (dx * dx + dz * dz), m * (dx * dx + dy * dy), -m * dx * dz
    terms = [para(*l) for l in loads] + [para(we, ex, ey, ez)]
    I = tuple(It - sum(t[i] for t in terms) for i, It in enumerate(I0))
    return dict(empty=we, cg=(ex, ey, ez), I=I, pms=U.POINT_MASSES, tanks=U.TANKS)


def fdm_file(cg=None, weight=None, inertia=None):
    mc = mass_config(cg, weight, inertia)
    gear = []
    # skids: sliding contacts (no wheels: the rolling friction is the sliding one); about 3 cm static deflection
    for name, xyz in SKIDS:
        fs, bl, wl = U.model_to_station(*xyz)
        gear.append('''    <contact type="BOGEY" name="%s">
      <location unit="IN"><x>%.2f</x><y>%.2f</y><z>%.2f</z></location>
      <static_friction>0.6</static_friction><dynamic_friction>0.45</dynamic_friction><rolling_friction>0.45</rolling_friction>
      <spring_coeff unit="LBS/FT">12000</spring_coeff><damping_coeff unit="LBS/FT/SEC">900</damping_coeff>
      <max_steer unit="DEG">0</max_steer><brake_group>NONE</brake_group><retractable>0</retractable>
    </contact>''' % (name, fs, bl, wl))
    fs, bl, wl = U.model_to_station(*TAIL_SKID)
    gear.append('''    <contact type="STRUCTURE" name="TAIL_SKID">
      <location unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
      <static_friction>0.6</static_friction><dynamic_friction>0.5</dynamic_friction>
      <spring_coeff unit="LBS/FT">6000</spring_coeff><damping_coeff unit="LBS/FT/SEC">600</damping_coeff>
    </contact>''' % (fs, bl, wl))
    for name, xyz in STRUCTURE:
        fs, bl, wl = U.model_to_station(*xyz)
        gear.append('''    <contact type="STRUCTURE" name="%s">
      <location unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
      <static_friction>0.8</static_friction><dynamic_friction>0.6</dynamic_friction>
      <spring_coeff unit="LBS/FT">15000</spring_coeff><damping_coeff unit="LBS/FT/SEC">800</damping_coeff>
    </contact>''' % (name, fs, bl, wl))
    pml = [(n, w, x, y, z) for n, w, mx, x, y, z in mc['pms']] + \
          [('Fuel tank %d' % (i + 1), w, x, y, z) for i, (w, x, y, z) in enumerate(mc['tanks'])]
    pms = '\n'.join('''    <pointmass name="%s">
      <weight unit="LBS">%.1f</weight>
      <location unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
    </pointmass>''' % pm for pm in pml)
    I = mc['I']
    eye = U.model_to_station(*U.EYE_MODEL)
    return '''<?xml version="1.0"?>
<!-- MBB Bo 105: generated by fdm/genbo105.py -->
<fdm_config name="Bo 105" version="2.0" release="ALPHA">
  <fileheader>
    <author>generated</author>
    <description>MBB Bo 105, elastic hingeless main rotor blade by blade, from the C-81 data of USAAMRDL TR-75-50</description>
  </fileheader>
  <metrics>
    <wingarea unit="FT2">%.1f</wingarea>
    <wingspan unit="FT">%.2f</wingspan>
    <chord unit="FT">%.3f</chord>
    <location name="AERORP" unit="IN"><x>%.2f</x><y>0.0</y><z>%.2f</z></location>
    <location name="EYEPOINT" unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
    <location name="VRP" unit="IN"><x>%.2f</x><y>0.0</y><z>%.2f</z></location>
  </metrics>
  <mass_balance>
    <ixx unit="SLUG*FT2">%.0f</ixx><iyy unit="SLUG*FT2">%.0f</iyy><izz unit="SLUG*FT2">%.0f</izz><ixz unit="SLUG*FT2">%.0f</ixz>
    <emptywt unit="LBS">%.1f</emptywt>
    <location name="CG" unit="IN"><x>%.2f</x><y>%.2f</y><z>%.2f</z></location>
%s
  </mass_balance>
  <ground_reactions>
%s
  </ground_reactions>
  <system file="bo105-controls"/>
  <system file="bo105-rotor"/>
  <system file="bo105-airframe"/>
  <system file="bo105-drive"/>
  <system file="bo105-outputs"/>
  <aerodynamics file="Systems/bo105-aero.xml"/>
</fdm_config>
''' % ((math.pi * U.R ** 2, 2 * U.R, U.CHORD, U.FS_CG, U.WL_CG) + tuple(eye) + (U.FS_MODEL0, U.WL_MODEL0) + I + (mc['empty'],) + tuple(mc['cg']) +
       (pms, '\n'.join(gear)))


# offline test configurations: (file name, CG stations, gross weight lb, inertias slug ft^2)
TEST_CASES = [('bo105', None, None, None),
              ('bo105-c81', U.C81_CG, U.C81_GW, U.C81_I),     # the C-81 analysis' loading [TR cards 21-22]
              # MBB's V-4 trim flights [TR fig. 16]: 4500 lb, CG 3.1 in forward of the rotor reference axis (FS 100.4,
              # where the main and tail rotor drive shafts meet [TR p. 28]), C-81's inertias
              ('bo105-mbb', (U.FS_RRA - 3.1, 0.0, U.WL_CG), 4500.0, U.C81_I),
              # the DLR S123 of the AGARD identification tests [AR table 6.2.2]: 2200 kg, the manufacturer's inertias
              # about the CG for those flights (Ix 1700, Iy 4200, Iz 4200, Ixz 0 kg m^2)
              ('bo105-ar280', (U.FS_CG, 0.0, U.WL_CG), U.GW_DESIGN, tuple(i * U.KGM2 for i in (1700, 4200, 4200, 0)))]


def build(out, fg=False):
    mdl = Model()
    mdl.controls()
    mdl.rotor_states()
    mdl.main_rotor()
    mdl.tail_rotor()
    mdl.airframe()
    os.makedirs(os.path.join(out, 'Systems'), exist_ok=True)
    texts = {'bo105-controls.xml': mdl.ctl.xml(), 'bo105-rotor.xml': mdl.rot.xml(), 'bo105-airframe.xml': mdl.air.xml(),
             'bo105-drive.xml': engines.build().xml(), 'bo105-outputs.xml': mdl.outputs().xml(),
             'bo105-aero.xml': '<!-- generated by fdm/genbo105.py -->\n<aerodynamics>\n%s\n%s\n</aerodynamics>\n' % (mdl.g.xml(), mdl.axes())}
    if INLINE:
        inline_single_use(texts, extra=[fdm_file()], keep=KEEP)
    for name, t in texts.items():
        open(os.path.join(out, 'Systems', name), 'w').write('<?xml version="1.0"?>\n' + t)
    if fg:
        open(os.path.join(out, 'bo105.xml'), 'w').write(fdm_file())
        return len(mdl.g.items)
    for name, cg, w, I in TEST_CASES:
        open(os.path.join(out, name + '.xml'), 'w').write(fdm_file(cg, w, I))
    return len(mdl.g.items)


if __name__ == '__main__':
    args = [a for i, a in enumerate(sys.argv[1:], 1) if not a.startswith('--') and sys.argv[i - 1] != '--tr']
    out = args[0] if args else 'out'
    if '--tr' in sys.argv:
        U.TR_MODEL = sys.argv[sys.argv.index('--tr') + 1]
    if '--no-tr-fin' in sys.argv:
        U.TR_FIN = False
    if '--no-inline' in sys.argv:
        INLINE = False
    n = build(out, fg='--fg' in sys.argv)
    print('wrote', out, '-', n, 'aerodynamic functions')
