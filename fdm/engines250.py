"""Bo 105 drive system: two Allison 250-C20B engines, freewheel clutches, rotor brake and the rotor speed degree of
freedom, as a JSBSim system (FCS components, no Nasal). Built like the UH-60 model's T700 drive (fdm in ~/git/UH-60).

Sources:
  [EA]  EASA TCDS R.011: 100 % power turbine speed 6016 rpm (rotor 424 rpm); gas generator 100 % = 50 980 rpm
        (52 000 rpm = 102 %); all-engine take-off and maximum continuous power 2 x 257 kW (86 % torque) at TOT 810
        and 779 C; one engine inoperative 283 kW (95 %, 30 min), 313 kW (105 %, continuous), 328 kW (110 %, 2.5 min)
        at 105 % gas generator speed. So 100 % torque is 298.8 kW per engine at 100 % rotor speed.
  [HG]  Hamers and von Gruenhagen, ERF 1997: the 250-C20B's hydromechanical governors meter fuel against the gas
        generator speed, the power turbine speed (its governor trims the gas generator's reference) and the
        collective (feed forward); the power turbine governor proportional, without integral action.
  [AL]  Allison 250-C20B ratings: 420 shp (313 kW) take-off, specific fuel consumption 0.650 lb/shp/h there; flight
        idle about 60 % gas generator speed (typical 250-series data, not Bo 105 specific).

The engine is a nonlinear model of the same structure as the UH-60's: the fuel control holds a gas generator speed
reference within acceleration and deceleration schedules; the gas generator follows the fuel with a speed-dependent
lag; the power turbine's torque depends on the gas generator's speed and its own. The steady torque against gas
generator speed is shaped to the EASA points (110 % torque at 105 %, 86 % about 100 %) and to zero torque at idle;
corrected parameters (delta, theta) carry it to other altitudes and temperatures. The rotor speed is the hub's,
with the blades' rigid inertia about the shaft (their lag motion relative to it is the rotor model's).
"""
import math
from xmlfun import *
import bo105data as U

G0 = 32.174
OMEGA0 = U.OMEGA
KW_FTLB = 737.562           # (ft lb/s) per kW
Q100 = 298.8 * KW_FTLB / OMEGA0      # ft lb at the rotor shaft per engine for 100 % torque (4964)
# blades about the shaft (rigid; the lag modes ride on it) + hub, shafts, gearbox downstream of the clutches
J_BLADES = U.NB * 162.1              # slug ft^2 [TR table 3: 161.9 per blade]
J_HUB = 45.0                         # slug ft^2: titanium hub, mast, gearbox (estimate)
J_ENG = 4.0                          # per engine, referred to the rotor: power turbine and its gears (estimate)
J_ROTOR = J_BLADES + J_HUB
SHAFT_TILT = U.SHAFT_TILT
GEAR_TR = U.GEAR_TR
ACCESSORY_FTLB = 8.0 * KW_FTLB / OMEGA0   # accessories (hydraulics, generators), 8 kW (estimate)
GEARBOX_LOSS = 0.025                       # of the main rotor torque (estimate)

# steady torque at the rotor per engine, % of Q100, at 100 % power turbine speed, sea level standard, against
# corrected gas generator speed %
NG_Q = [0, 40, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 102, 105, 108]
Q_NG = [-60, -35, -12, -4, 2, 8, 15, 23, 33, 46, 62, 83, 93, 110, 127]
# fuel flow per engine, lb/h, corrected: idle about 70 lb/h, 273 lb/h at 420 shp (sfc 0.65) [AL]
NG_WF = [0, 20, 40, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 108]
WF_NG = [0, 15, 35, 58, 68, 78, 90, 104, 120, 140, 163, 192, 228, 274, 305]
# gas generator lag, s, against corrected speed (faster at high speed; slow spool-up and spool-down below idle)
NG_TAU = [0, 20, 40, 55, 60, 70, 80, 90, 100, 108]
TAU_NG = [5.0, 5.0, 4.0, 2.5, 1.6, 1.0, 0.7, 0.5, 0.4, 0.4]
# TOT / ambient temperature (both absolute) at steady state against corrected gas generator speed: 810 C at 105 %
# (take-off), about 460 C at idle
NG_TGT = [0, 20, 50, 60, 70, 80, 90, 95, 100, 105, 108]
TGT_NG = [1.0, 1.8, 2.3, 2.5, 2.6, 2.75, 2.95, 3.1, 3.3, 3.75, 3.95]
NG_IDLE = 60.0
NG_MAX = 105.0
NG_STARTER = 22.0
NG_DROPOUT = 58.0
NP_OVERSPEED = 120.0
TGT_LIMIT = 810.0
# the cockpit's power levers (FlightGear /controls/engines/engine[n]/power): 0 cut-off, 0.63 ground idle, 1 flight; the
# power turbine governor takes over near the flight detent
PL_IDLE, PL_FLY, PL_GOVERNED = 0.63, 1.0, 0.9
# power turbine governor [HG fig. 5: proportional on the rotor speed error with the collective's feed forward, a lag,
# a rate limit, fuel metered directly in flight]: gain (% gas generator speed per % rotor speed), lag (s) and feed
# forward (% per % collective) fitted to DLR's hover collective 3-2-1-1 (engine_response.py: rotor speed and shaft
# torque increments, 15 s; the flat optimum, the gas generator's lags as estimated). The rate limit is not identified
# (left open). The flight record cannot tell a pure droop governor from one with a slow reset; a 20 s reset takes the
# droop out in steady flight without changing the fit (RMS within 1 %).
GOV_K = 1.0
TAU_GOV = 0.1
COLL_FF = 0.1
GOV_TI = 20.0
TRIM_PCT = 10.0            # % power turbine speed per unit of beep trim or balance
NG_GOV = 95.0              # % gas generator speed at 100 % rotor speed, 50 % collective (about the hover's)
# The anticipation's slope is the fit's about the hover; held down to flat pitch it would leave the governor a droop
# of 11 % there (rotor at 111 % before the reset, 107 % peak after starts). Below COLL_BREAK it runs linearly to the
# gas generator speed the rotor needs at flat pitch, on the ground at 100 % (78.5 %, ISA sea level), as a compensator
# cam would; the hover and forward flight (40-60 % collective) stay on the fitted slope.
COLL_BREAK, NG_FLAT = 35.0, 78.5
GOV_RATE = 100.0           # % gas generator speed per s


def build(fcs_name='Bo 105 drive'):
    f = FCS(fcs_name)
    for n in range(2):
        e = '/controls/engines/engine[%d]' % n
        f.declare(e + '/power', 1.0)          # power lever: 0 cut-off, 0.63 idle, 1 flight
        f.declare(e + '/ignition', 0)
        f.declare(e + '/starter', 0)
        for st in ('ng-pct', 'np-rad_sec', 'lit', 'engaged', 'starter-on'):
            f.declare('eng[%d]/%s' % (n, st), 0.0)
        f.declare('eng[%d]/tgt-degc' % n, 15.0)
        f.declare('eng[%d]/ng-gov' % n, 90.0)
        f.declare('eng[%d]/gov-int' % n, 0.0)
    # the cockpit's power turbine speed trim (beep) and the engines' balance, each 1.0 = TRIM_PCT % (the FlightGear
    # aircraft's keys: n/N trim through propeller-pitch, m/M balance through mixture)
    f.declare('/controls/engines/power-trim', 0.0)
    f.declare('/controls/engines/power-balance', 0.0)
    f.declare('/consumables/fuel/tank[0]/level-lbs', 1000.0)
    f.declare('/consumables/fuel/tank[1]/level-lbs', 100.0)
    f.declare('/controls/rotor/brake', 0.0)
    f.declare('eng/init-running', 1)
    f.declare('eng/gov-k', GOV_K)
    f.declare('eng/gov-tau', TAU_GOV)
    f.declare('eng/coll-ff', COLL_FF)
    f.declare('eng/ng-loop-flight', 0.0)
    f.declare('eng/tau-scale', 1.0)
    f.declare('eng/gov-rate', GOV_RATE)
    f.declare('eng/gov-ti', GOV_TI)
    f.declare('/sim/freeze/fuel', 0)
    dt = p('simulation/dt')
    init = p('ctl/init-fast')
    f.channel('start state')
    run0 = f.fn('eng/running-at-start', MAX(p('eng/init-running'), GT(p('position/h-agl-ft'), v(50))))
    for n in range(2):
        e = '/controls/engines/engine[%d]' % n
        f.fn('eng[%d]/pl-init' % n, IFTHEN(GT(init, v(0.5)), IFTHEN(run0, v(PL_FLY), v(0)), p(e + '/power')), out=e + '/power')
    f.fn('drive/brake-init', IFTHEN(GT(init, v(0.5)), IFTHEN(run0, v(0), v(1)), p('/controls/rotor/brake')), out='/controls/rotor/brake')
    W = p('rotor/omega-rad_sec')
    clamp = lambda x, lo, hi: MAX(v(lo), MIN(v(hi), x))
    f.channel('ambient')
    th = f.fn('eng/theta', Q(p('atmosphere/T-R'), v(518.67)))
    dl = f.fn('eng/delta', Q(p('atmosphere/P-psf'), v(2116.22)))
    sth = f.fn('eng/sqrt-theta', SQ(th))
    dsth = f.fn('eng/delta-sqrt-theta', M(dl, sth))
    Tamb = f.fn('eng/tamb-k', Q(p('atmosphere/T-R'), v(1.8)))
    f.channel('rotor load')
    Qmr = p('aero/rotor/torque-ftlb')
    Qtr = M(v(GEAR_TR), p('aero/rotor/tr/torque-ftlb'))
    Wn = Q(W, v(OMEGA0))
    f.fn('drive/loss-ftlb', S(M(v(ACCESSORY_FTLB), SQR(Wn)), M(v(GEARBOX_LOSS), ABS(Qmr))), desc='accessories and gearbox losses')
    f.fn('drive/brake-ftlb', M(p('/controls/rotor/brake'), v(4000), Q(W, S(ABS(W), v(0.3)))), desc='rotor brake, smooth at standstill')
    Qload = f.fn('drive/load-ftlb', S(Qmr, Qtr, p('drive/loss-ftlb'), p('drive/brake-ftlb')))
    npc = f.fn('drive/nr-pct', M(v(100 / OMEGA0), W))
    for n in range(2):
        e = '/controls/engines/engine[%d]' % n
        g = 'eng[%d]' % n
        f.channel('engine %d' % (n + 1))
        ng, np_, tgt = p(g + '/ng-pct'), p(g + '/np-rad_sec'), p(g + '/tgt-degc')
        lit, eng, st = p(g + '/lit'), p(g + '/engaged'), p(g + '/starter-on')
        pl = p(e + '/power')
        ngc = f.fn(g + '/ngc-pct', Q(ng, sth))
        npp = f.fn(g + '/np-pct', M(v(100 / OMEGA0), np_))
        f.fn(g + '/starter-next', IFTHEN(GT(S(p(e + '/starter'), st), v(0.5)), IFTHEN(GT(ng, v(NG_DROPOUT)), v(0), v(1)), v(0)), out=g + '/starter-on')
        fuel_ok = f.fn(g + '/fuel-ok', GT(S(p('/consumables/fuel/tank[0]/level-lbs'), p('/consumables/fuel/tank[1]/level-lbs')), v(0.5)))
        lightable = M(GT(p(e + '/ignition'), v(0.5)), GT(pl, v(0.1)), GT(ng, v(8)), fuel_ok)
        keep = M(GT(pl, v(0.05)), GT(ng, v(8)), LT(npp, v(NP_OVERSPEED)), fuel_ok)
        f.fn(g + '/lit-next', IFTHEN(GT(init, v(0.5)), run0, M(keep, MAX(lit, lightable))), out=g + '/lit')
        # fuel control: gas generator reference from the power lever (idle .. full), less what the power turbine
        # governor takes off for rotor speed above its reference, plus the collective's feed forward
        # lever: from starting fuel (0.1, the starter's speed) up to the idle detent, then idle to maximum
        below = S(v(NG_STARTER), M(v((NG_IDLE - NG_STARTER) / (PL_IDLE - 0.1)), clamp(D(pl, v(0.1)), 0, PL_IDLE - 0.1)))
        above = S(v(NG_IDLE), M(v((NG_MAX - NG_IDLE) / (PL_FLY - PL_IDLE)), clamp(D(pl, v(PL_IDLE)), 0, PL_FLY - PL_IDLE)))
        ng_pl = f.fn(g + '/ng-pl', M(sth, IFTHEN(LT(pl, v(PL_IDLE)), below, above)))
        ref = S(v(100), M(v(TRIM_PCT), p('/controls/engines/power-trim')), M(v(TRIM_PCT * (2 * n - 1)), p('/controls/engines/power-balance')))
        # power turbine governor: proportional on the speed error with the collective's feed forward [HG], about a datum
        # (NG_GOV, % gas generator speed at the reference speed and 50 % collective) and a reset (eng[n]/gov-int) that,
        # with a reset time eng/gov-ti > 0, slowly takes out the droop while the governor governs (lever at flight, rotor
        # above 90 %; no wind-up through starts); started at the trim's equilibrium when running from the start
        err = D(ref, npc)
        gi = p(g + '/gov-int')
        if n == 0:
            c = p('sens/coll-pct')
            ff_break = M(p('eng/coll-ff'), v(COLL_BREAK - 50))
            f.fn('eng/coll-anticipation', IFTHEN(LT(c, v(COLL_BREAK)),
                                                 S(v(NG_FLAT - NG_GOV), M(Q(D(ff_break, v(NG_FLAT - NG_GOV)), v(COLL_BREAK)), MAX(v(0), c))),
                                                 M(p('eng/coll-ff'), D(c, v(50)))),
                 desc='the governor\'s collective anticipation, % gas generator speed')
        governing = M(lit, GT(pl, v(PL_GOVERNED)), GT(npc, v(90)))
        f.fn(g + '/gov-int-next', IFTHEN(GT(init, v(0.5)),
                                         M(run0, D(D(Q(p(g + '/ng-for-load'), sth), M(p('eng/gov-k'), err)), S(v(NG_GOV), p('eng/coll-anticipation')))),
                                         MAX(v(-15), MIN(v(15), S(gi, M(GT(p('eng/gov-ti'), v(0)), governing, dt, Q(p('eng/gov-k'), MAX(p('eng/gov-ti'), v(1))), err))))),
             out=g + '/gov-int')
        gov_ss = f.fn(g + '/ng-gov-ss', M(sth, S(v(NG_GOV), gi, M(p('eng/gov-k'), err), p('eng/coll-anticipation'))),
                      desc='power turbine governor and collective anticipation, steady')
        # the hydromechanical governor's own lag and its rate limit [HG fig. 5: 1 / (tau_gov s + 1) on the speed error and
        # the collective's feed forward, then a rate limiter on the fuel]; the lag also keeps the governor off the coupled
        # collective lag and drive train mode (2.2/rev)
        step = M(Q(dt, MAX(p('eng/gov-tau'), dt)), D(gov_ss, p(g + '/ng-gov')))
        lim = M(p('eng/gov-rate'), sth, dt)
        gov = f.fn(g + '/ng-gov-next', IFTHEN(GT(init, v(0.5)), gov_ss, S(p(g + '/ng-gov'), MAX(NEG(lim), MIN(lim, step)))),
                   out=g + '/ng-gov')
        flying = GT(pl, v(PL_GOVERNED))
        ngref = f.fn(g + '/ng-ref', IFTHEN(flying, MAX(M(v(NG_IDLE), sth), MIN(ng_pl, gov)), ng_pl))
        tgtlim = M(v(0.5), MAX(v(0), D(tgt, v(TGT_LIMIT + 15))))
        f.fn(g + '/ngref-c', Q(D(ngref, tgtlim), sth))
        wfss = lambda x: T1(x, NG_WF, WF_NG)
        # fuel command: the steady fuel flow for the reference, and a gas generator speed loop on the ground (starts and
        # idle); in flight the governor meters fuel directly and the gas generator follows with its own lag [HG fig. 5]
        kng = IFTHEN(flying, p('eng/ng-loop-flight'), v(25))
        wfc = f.fn(g + '/wf-cmd-c', S(T1(g + '/ngref-c', NG_WF, WF_NG), M(kng, D(p(g + '/ngref-c'), ngc))))
        wf_acc = S(M(v(1.35), wfss(g + '/ngc-pct')), v(30))
        wf_dec = M(v(0.55), wfss(g + '/ngc-pct'))
        wf = f.fn(g + '/fuel-flow-pph', M(lit, GT(pl, v(0.05)), dsth, MAX(wf_dec, MIN(wf_acc, wfc))), desc='lb/h')
        f.fn(g + '/wf-c', Q(wf, MAX(dsth, v(0.05))))
        tau = M(p('eng/tau-scale'), T1(g + '/ngc-pct', NG_TAU, TAU_NG))
        ng_lit = M(sth, T1(g + '/wf-c', WF_NG, NG_WF))
        ng_unlit = M(st, v(NG_STARTER))
        target = IFTHEN(lit, ng_lit, ng_unlit)
        tau_eff = IFTHEN(lit, tau, IFTHEN(st, v(3.0), v(8.0)))
        assist = M(st, lit, v(1.5), MAX(v(0), D(v(1), Q(ng, v(60)))))
        steady_ng = f.fn(g + '/ng-for-load', M(sth, T1(g + '/q-share-pct', Q_NG, NG_Q)))
        f.fn(g + '/q-share-pct', Q(M(v(50 / Q100), Qload), MAX(dsth, v(0.05))))
        f.fn(g + '/ng-next', IFTHEN(GT(init, v(0.5)), M(run0, MAX(M(v(NG_IDLE), sth), steady_ng)),
                                    MAX(v(0), S(ng, M(dt, S(Q(D(target, ng), tau_eff), assist))))), out=g + '/ng-pct')
        # power turbine torque at the rotor: the table at 100 % Np, rising as the turbine slows
        qtab = M(v(Q100 / 100), T1(g + '/ngc-pct', NG_Q, Q_NG))
        kq = M(v(0.012), MAX(v(0), S(qtab, v(0.4 * Q100))))
        q = f.fn(g + '/torque-ftlb', M(lit, dsth, S(qtab, M(kq, D(v(100), npp)))), desc='at the rotor shaft')
        f.fn(g + '/torque-pct', Q(M(v(100), MAX(v(0), M(q, eng))), v(Q100)))
        ratio = clamp(Q(S(p(g + '/wf-c'), v(5)), S(wfss(g + '/ngc-pct'), v(5))), 0.3, 2.5)
        tgt_ss = IFTHEN(lit, MAX(D(Tamb, v(273.15)), D(M(Tamb, T1(g + '/ngc-pct', NG_TGT, TGT_NG), SQ(ratio)), v(273.15))), D(Tamb, v(273.15)))
        f.fn(g + '/tgt-ss', tgt_ss)
        f.fn(g + '/tgt-next', IFTHEN(GT(init, v(0.5)), p(g + '/tgt-ss'), S(tgt, M(dt, Q(D(p(g + '/tgt-ss'), tgt), IFTHEN(lit, v(1.5), v(20)))))),
             out=g + '/tgt-degc')
    f.channel('fuel')
    frozen = GT(p('/sim/freeze/fuel'), v(0.5))
    burn = M(S(p('eng[0]/fuel-flow-pph'), p('eng[1]/fuel-flow-pph')), dt, v(1 / 3600.0))
    t0, t1 = '/consumables/fuel/tank[0]/level-lbs', '/consumables/fuel/tank[1]/level-lbs'
    # both engines draw from the supply tank; the transfer pumps keep it full from the main tank
    f.fn('fuel/burn-supply', IFTHEN(frozen, p(t1), MAX(v(0), D(p(t1), burn))), out=t1)
    f.fn('fuel/transfer', IFTHEN(frozen, p(t0), MAX(v(0), D(p(t0), MIN(MAX(v(0), D(v(U.SUPPLY_LBS), p(t1))), M(v(1.2), dt))))), out=t0)
    f.fn('fuel/fill-supply', IFTHEN(frozen, p(t1), MIN(v(U.SUPPLY_LBS), S(p(t1), IFTHEN(GT(p(t0), v(0.5)), MIN(MAX(v(0), D(v(U.SUPPLY_LBS), p(t1))), M(v(1.2), dt)), v(0))))), out=t1)
    f.fn('fuel/mass-main', p(t0), out='inertia/pointmass-weight-lbs[%d]' % U.PM_TANK0)
    f.fn('fuel/mass-supply', p(t1), out='inertia/pointmass-weight-lbs[%d]' % (U.PM_TANK0 + 1))
    f.channel('drive train')
    q0, q1 = p('eng[0]/torque-ftlb'), p('eng[1]/torque-ftlb')
    e0, e1 = p('eng[0]/engaged'), p('eng[1]/engaged')
    f.fn('drive/net-torque-ftlb', D(S(M(e0, q0), M(e1, q1)), Qload), desc='engines less the rotors and losses')
    f.declare('drive/hold-rpm', 0)
    wdot = p('drive/omega-dot')          # from rotor_speed, in the rotor system (it runs first)
    w_new = f.fn('drive/omega-new', IFTHEN(GT(p('drive/hold-rpm'), v(0.5)), W, MAX(v(0), S(W, M(dt, wdot)))))
    w_init = M(run0, M(v(OMEGA0 / 100), S(v(100), M(v(TRIM_PCT), p('/controls/engines/power-trim')))))
    for n, (q, en) in enumerate(((q0, e0), (q1, e1))):
        g = 'eng[%d]' % n
        np_ = p(g + '/np-rad_sec')
        free = MAX(v(0), S(np_, M(dt, Q(D(q, M(v(60), SQR(Q(np_, v(OMEGA0))))), v(J_ENG)))))
        f.fn(g + '/np-free', free)
        f.fn(g + '/engaged-next', IFTHEN(GT(init, v(0.5)), run0, IFTHEN(en, GT(q, v(0)), GT(p(g + '/np-free'), w_new))), out=g + '/engaged')
        f.fn(g + '/np-next', IFTHEN(GT(init, v(0.5)), w_init, IFTHEN(p(g + '/engaged'), w_new, p(g + '/np-free'))), out=g + '/np-rad_sec')
    f.fn('drive/omega-next', IFTHEN(GT(init, v(0.5)), w_init, w_new), out='rotor/omega-rad_sec')
    # with the rotor speed held (trims) the shaft gives the rotor what it takes: its aerodynamic torque and the blades'
    # lag accelerations' reaction
    held = GT(S(init, p('drive/hold-rpm')), v(0.5))
    f.fn('drive/mr-torque-ftlb', IFTHEN(held, D(Qmr, p('aero/rotor/torque-inertial-ftlb')),
                                        D(S(M(e0, q0), M(e1, q1)), S(Qtr, p('drive/loss-ftlb'), p('drive/brake-ftlb')))),
         desc='torque into the main rotor shaft; its reaction acts on the airframe')
    return f


def rotor_speed(r, lag_forcing, Srl, Ml, init):
    """The rotor speed's acceleration relative to the airframe (drive/omega-dot), in the rotor system r, solved with
    the blades' lag accelerations. Per blade Ml ql'' = F_k - Srl W'_abs (F_k: the rest of its lag equation), and
    J W' = Q_net - sum Srl ql_k'' + J_blades r'_s (the rotor keeps its own angular momentum: its speed relative to the
    airframe changes with the airframe's acceleration about the shaft, as the UH-60 model, Fletcher NASA TM 110362),
    with W'_abs = W' - r'_s, give
        (J - NB Srl^2 / Ml) W' = Q_net - Srl / Ml sum F_k + (J_blades - NB Srl^2 / Ml) r'_s.
    lag_forcing = sum F_k; Q_net is the drive system's, a frame old (the engines' torque and the rotors' load)."""
    Jl = U.NB * Srl ** 2 / Ml
    rds = r.fn('drive/r-dot-shaft', S(M(v(math.cos(SHAFT_TILT)), p('rotor/rds-in')), M(v(-math.sin(SHAFT_TILT)), p('rotor/pds-in'))),
               desc='airframe angular acceleration about the main rotor shaft (down +)')
    J = S(v(J_ROTOR - Jl), M(v(J_ENG), S(p('eng[0]/engaged'), p('eng[1]/engaged'))))
    still = GT(S(p('drive/hold-rpm'), init), v(0.5))
    return r.fn('drive/omega-dot', IFTHEN(still, v(0), Q(S(p('drive/net-torque-ftlb'), M(v(-Srl / Ml), lag_forcing), M(v(J_BLADES - Jl), rds)), J)),
                desc='rotor speed acceleration relative to the airframe, rad/s2')
