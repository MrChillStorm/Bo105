"""MBB Bo 105 data for the JSBSim model, with sources.

  [TR]  USAAMRDL TR-75-50 (Staley 1976): Boeing Vertol's C-81 model of the Bo 105 and MBB flight tests; input deck
        (table A-1, cards and airfoil table 666), blade mass and stiffness (tables 3, 4), modes (figs. 6-12)
  [HF]  NASA CR-3144 (Heffley et al. 1979), section III: Bo 105C descriptive data, loadings, Y-92 derivatives
  [AR]  AGARD AR-280 (1991), table 6.2.1: the DLR Bo 105-S123 of the system identification flight tests
  [PT]  NASA TM-103867 (Peterson 1991): blade frequencies (table 2), section axes (table 3), control stiffness
  [EA]  EASA TCDS R.011 issue 3 (2017), section 3: Bo 105 CBS, CBS-4, CBS-5 limits
  [HG]  Hamers and von Gruenhagen, ERF 1997 paper 91: Allison 250-C20B engine and governor in DLR's SIMH
  [FM]  Eurocopter BO 105 CB-5/CBS-5 flight manual: three-view (fig. 1-1), hover ceilings in and out of ground effect
        (figs. 5-8, 5-10), loading data and station diagram (section 6.3, fig. 6-1), cockpit (figs. 7-2, 7-3)

Frames: the stations of [TR] and [HF] in inches are the JSBSim structural frame: SL (FS) from the fuselage nose,
aft +; BL right +; WL up + from MBB's reference datum. MBB's stations in mm [FM fig. 6-1] start 745 mm ahead of the
nose: FS = (mm - 745) / 25.4 (the hub's FS 98.44 is the three-view's 2.50 m behind the nose).
"""
import math

DEG = math.pi / 180
G0 = 32.174

# ---------------------------------------------------------------- main rotor
R = 16.11               # ft [TR table 2, card 41]; 9.82 m [AR]
NB = 4
CHORD = 10.64 / 12      # ft [TR card 41]; 0.27 m [AR], 0.886 ft [PT]
TWIST = -8.0            # deg, linear from the shaft to the tip [TR card 41: total twist; HF]
PRECONE = 2.5 * DEG     # built-in coning of the hub [TR card 44, PT]
SHAFT_TILT = 3.0 * DEG  # forward [TR card 47, HF, AR]
OMEGA = 44.4            # rad/s, 100 % = 424 rpm [EA: 6016 rpm power turbine; HG: 44.4 rad/s]
R_ROOT = 3.22           # ft, inboard end of the aerodynamic blade: the C-81 hub extent, 0.2 R [TR card 43]
SWASH_PHASE = -10.0     # deg, control phasing: pitch = th0 - A1s cos(psi - 10) - B1s sin(psi - 10) [TR card 47; TM-110356]
FS_MR, BL_MR, WL_MR = 98.444, 0.0, 61.20   # hub (mast pivot) [TR card 42; HF: FS 98.44, WL 61.2]
TIP_LOSS = 0.97         # B (square tip; assumed as for the UH-60)
NSEG = 8                # aerodynamic segments of equal annulus area
# section drag increment over the smooth tables (production blade, tripped boundary layer; as the UH-60's SC1095,
# Bousman NASA TP-2003-212265): checked against the full-scale rotor's hover power [TM-85892] in the test programme
DCD = 0.0

# Blade structure [TR table 4]: Boeing's Y-71 model of the 23012 blade, stations from the tip; masses lumped at the
# stations (lb s^2/in), pitch inertia IX (lb in s^2) lumped, stiffness (1e6 lb in^2) of the segment from the station to
# the next one outboard; YM mass and YSC shear centre offsets from the pitch axis (in, + towards the leading edge)
Y71_R = [193.36, 190.40, 183.70, 174.30, 164.36, 154.70, 145.30, 135.36, 125.69, 116.02, 106.35, 96.69, 87.02,
         77.35, 67.68, 58.01, 48.34, 38.67, 29.01, 19.34, 9.67, 2.46, 0.05]
Y71_M = [0.000015, 0.00385] + [0.007732] * 15 + [0.008672, 0.007894, 0.030621, 0.094402, 0.036313, 0.0]
Y71_IX = [0.000097, 0.024123] + [0.048439] * 15 + [0.026055, 0.016095, 0.582690, 0.344870, 0.0, 0.0]
Y71_GJ = [None, 1.52] + [1.52] * 15 + [1.62, 2.16, 3.21, 4.09, 4.10, 4.10]
Y71_EIF = [None, 2.38] + [2.38] * 15 + [2.62, 3.22, 3.66, 13.60, 201.0, 850.0]
Y71_EIL = [None, 59.4] + [59.4] * 15 + [67.9, 38.0, 38.1, 88.6, 204.0, 885.0]
Y71_YM = [0.0, -0.011] + [-0.011] * 15 + [-0.594, 0.593, -0.088, -0.358, -0.670, 0.0]
Y71_YSC = [None, 0.744] + [0.744] * 15 + [0.734, 0.110, 0.460, 0.0, 0.0, 0.0]
# C-81 mass distribution, 20 strips of 5 % R from the shaft (lb/in) [TR table 3]: 114.36 lb, 161.9 slug ft^2
C81_W = [3.3503, 2.5024, 0.7737, 0.2821, 0.2889] + [0.3090] * 14 + [0.3084]
# Control system stiffness at the blade root (pitch links, swashplate, servos), ft lb/rad: the baseline of [PT] table 4
# (non-rotating rigid torsion 7.3/rev with I_theta 0.0795 slug ft^2)
K_CONTROL = 8220.0
# Section axes, fraction of chord from the leading edge [PT table 3]: elastic and feathering axes 15.6 % (17.95 % gave
# [PT] the better correlation of the in-plane loads, and is where Y-71 has the shear centre, 0.744 in ahead of 25 %), tension
# axis 21.1 %, CG 24.4 %; the aerodynamic centre at the quarter chord
X_EA, X_CG, X_AC = 0.1795, 0.244, 0.25
# Rotating frequencies, per rev at 425 rpm: the comprehensive analysis of [PT] table 2, its hub stiffness tuned to MBB's
# measured flap and lag frequencies; and Boeing's Y-71 [TR figs. 6-12] from table 4 as it stands
FREQ_PT = {'flap1': 1.12, 'lag1': 0.66, 'torsion1': 3.79, 'flap2': 2.78, 'flap3': 5.02, 'lag2': 4.33}
FREQ_Y71 = {'flap1': 1.125, 'lag1': 0.732, 'flap2': 2.78, 'torsion1': 3.176, 'lag2': 4.518, 'flap3': 5.007}
ZETA_STRUCT = 0.01      # structural damping ratio of every mode [TR deck, cards 16D]
# the first lag mode's: fitted to the isolated rotor's measured in-plane decay in the hover [TM-85892 fig. 6, 425 rpm:
# -0.97 1/s at CT/sigma 0.07; the model gives -0.77 with 0.01] (fdm/lag_damping.py)
ZETA_LAG = 0.017
# blade modes kept: first flap and first lag bending (integrated), first torsion (quasi-static)
LAG_TARGET = 0.66       # /rev: the lag frequency the root's in-plane stiffness is tuned to (None: table 4 as it stands)
# first torsion frequency, /rev at 425 rpm with the control system's stiffness: Boeing's Y-71 had 3.18 [TR fig. 9] and table
# 4 gives 3.32 here, where NASA's analyses with MBB/DLR's structural data found 3.56 [TM-85990 table 1] and 3.79
# [PT table 2, with the loads it reproduced]; the blade's torsional stiffness GJ is scaled to the latter (None: as it stands)
TORSION_TARGET = 3.79

# ---------------------------------------------------------------- tail rotor [TR cards 51-57, HF, AR]
R_TR = 3.115            # ft (1.90 m [EA, AR])
NB_TR = 2               # teetering (gimballed hub)
CHORD_TR = 7.05 / 12    # ft (0.179 m [AR])
TWIST_TR = 0.0
DELTA3_TR = 45.0 * DEG  # pitch-flap coupling [TR card 54]
W_BLADE_TR = 4.851      # lb
I_BLADE_TR = 0.487      # slug ft^2, each blade about the teeter axis
FS_TR, BL_TR, WL_TR = 335.0, -12.5, 68.7   # hub, on the left of the fin [TR card 52, HF]
GEAR_TR = 5.25          # tail rotor / main rotor speed: 233.1 rad/s [Padfield, as quoted by METU thesis 18442], 5.24 [HF];
GEAR_TR_TR7550 = 5.527  # the V4 / S50 aircraft of [TR] (2349 rpm at 425 rpm)
TOP_AFT_TR = True       # rotation: the blade at the top moves aft (advancing blade below the hub)
TIP_LOSS_TR = 0.92

# ---------------------------------------------------------------- controls [TR cards 131-134]
COLL_IN = 9.0           # collective lever travel, in
COLL_ROOT = (6.0, 22.0) # blade pitch at the shaft (twist zero), lever down / up, deg
LON_IN = 12.12          # longitudinal cyclic stick travel, in
B1S_RANGE = (-4.7, 9.3) # deg, full aft / full forward (+ = forward)
LAT_IN = 8.65
A1S_RANGE = (-5.7, 4.3) # deg, full left / full right (+ = right side down) [deck; the card listing reads -5]
PED_IN = 4.34
TR_PITCH_RANGE = (-6.0, 22.0)   # deg, full right / full left pedal, positive pushing the tail right (anti-torque)
                                # [deck: +6 nose right at full right pedal, range -28]

# ---------------------------------------------------------------- airframe
# horizontal stabiliser with end plates [TR cards 71-79, HF, EA]
S_H = 8.71              # ft^2 (0.809 m^2)
AR_H = 8.09
FS_H, BL_H, WL_H = 277.45, 0.0, 25.84   # centre of pressure
# its zero-lift angle, deg: the DLR S123's in Padfield's Helisim data [Padfield, table 4B.2: alpha_tp0 0.0698 rad]; the
# C-81 deck sets 0 [TR card 71], which gave the model a speed stability Mu above all six identifications at 80 kt and a
# longitudinal cyclic 23-40 % steeper against speed than both flight tests' (4 deg: within the MBB test's from 78 kt)
INC_H = 4.0
# fuselage data reference point (the centre of pressure of the 1/4-scale tests) [TR card 21]
FS_F, BL_F, WL_F = 100.39, 0.0, -1.86

# ---------------------------------------------------------------- mass
# DLR Bo 105-S123 (AR-280 table 6.2.1): 2200 kg; inertias kg m^2 -> slug ft^2
KGM2 = 1 / 1.35582
GW_DESIGN = 2200 * 2.20462
IXX, IYY, IZZ, IXZ = 1433 * KGM2, 4973 * KGM2, 4099 * KGM2, 660 * KGM2
FS_CG, BL_CG, WL_CG = 98.4, 0.0, 6.9
# TR-75-50's CG reference, the rotor reference axis: where the main and tail rotor drive shafts meet [TR p. 28]
FS_RRA = 100.4    # nominal CG [HF fig. III-3: FS 98.4; TR card 21: WL 6.9]
# the C-81 analysis' nominal loading [TR cards 21-22]: 4562 lb at FS 96.8, WL 6.9; Ixx 1268, Iyy 3479, Izz 3203, Ixz 250
C81_GW, C81_CG = 4562.0, (96.80, 0.0, 6.9)
C81_I = (1268.0, 3479.0, 3203.0, 250.0)

# ---------------------------------------------------------------- 3-D model alignment
# The 3-D model's frame (x aft, y right, z up, m) has its rotor hub on the axis x = 0 at z = 1.814. Its fuselage is
# placed by MBB's stations: the nose (745 mm) and the skid cross tubes at the fuselage (2055 and 3755 mm) [FM fig. 6-1]
# fit with the model's x = 0 MODEL_DX aft of the hub (residuals 2 to 6 cm). The model's own hub is too far aft: 2.76 m
# behind its nose against the three-view's 2.50 m [FM fig. 1-1]; its rotor and mast stay where it has them (rotorvis.py:
# at the FDM's hub the mast would stand on the front edge of its fairing). So placed, the model's stabiliser root
# quarter chord falls at FS 279 (C-81's FS 277.45 [TR card
# 71]); vertically the hubs coincide. The JSBSim VRP is the model's origin (the model offsets are zero).
IN_PER_M = 39.3701
MODEL_DX = 0.24
FS_MODEL0, WL_MODEL0 = FS_MR + MODEL_DX * IN_PER_M, WL_MR - 1.814 * IN_PER_M


def model_to_station(x, y, z):
    """3-D model metres (x aft, y right, z up) -> (FS, BL, WL) inches"""
    return FS_MODEL0 + x * IN_PER_M, y * IN_PER_M, WL_MODEL0 + z * IN_PER_M


# the pilot's eye: no Bo 105 design eye point is published. [FM] gives the pilot's loading station (1890 mm) and the
# front seats' 1735-1935 mm; the model's right seat, where its back meets the pan, is at 1805 mm. The eye is the model's
# seated pilot's (behind the lowered visor, above the boom microphone), 29.7 in above the compressed cushion (0.06 m
# less about 0.03 m), against the 30.5 in of the rotorcraft reference eye of FAA AC 29.773-1; that reference's other
# rule, 5 in aft of the cyclic grip held fully aft, would put the eye 0.07 m ahead of the pilot's face in this cockpit.
# Pilot on the right [EA: minimum crew one pilot, right seat]. Model frame, m.
EYE_MODEL = (-1.65, 0.40, 0.785)


def segments(n=NSEG, r0=R_ROOT, r1=R):
    """equal annulus area segments: (r_mid, dr) in ft, and the edges"""
    a = [r0 * r0 + (r1 * r1 - r0 * r0) * k / n for k in range(n + 1)]
    edges = [math.sqrt(x) for x in a]
    return [(math.sqrt((edges[k] ** 2 + edges[k + 1] ** 2) / 2), edges[k + 1] - edges[k]) for k in range(n)], edges

# ---------------------------------------------------------------- inflow, ground effect
# Peters-He states (j, r) as the UH-60 model: three axisymmetric (the hover's radial distribution) and the first harmonic
PH_COS = [(1, 0), (3, 0), (5, 0), (2, 1)]
PH_SIN = [(2, 1)]
# ground effect in hover on the induced velocity, (1 + GE_K (R/Z)^2)^GE_EXP, Z the hub height: the shape of the UH-60
# flight manual's (fitted in the UH-60 model, GE_K 0.3; flight test, 0.80-0.97 of the OGE torque at 5-40 ft), scaled to
# the Bo 105's flight manual [FM figs. 5-8, 5-10]: its hover ceilings in ground effect (3 ft skid height, Z/R 0.76) and
# out of it, at the same power, give 1.093 times the gross mass at every altitude; GE_K 0.174 gives 1.091-1.095 at
# 1000 ft and, for the main rotor, 1.086-1.088 at the charts' 12 000 ft and -37.5 C (the UH-60's 0.3 gave 1.14-1.15)
GE_K, GE_EXP = 0.174, -0.5
GE_ZR_MIN = 0.3

# ---------------------------------------------------------------- tail rotor aerodynamics
SIGMA_TR = NB_TR * CHORD_TR / (math.pi * R_TR)   # 0.12 [AR]
A_TR = 5.73             # lift curve slope, NACA 0012 (C-81's built-in 0012 equations [TR card 32B]: 0.11/deg at M 0)
CD0_TR = 0.008          # [TR card 32B: CD at zero lift, M 0]
# the tail rotor model: 'blades' (blade elements, as the UH-60 model's) or 'bailey' (Bailey's closed form, as GenHel's)
TR_MODEL = 'blades'
# blade elements: NSEG_TR segments of equal annulus area from R_ROOT_TR (C-81's tail rotor hub extent is zero [TR card 53];
# the blade's root fitting taken as 0.2 R, as the UH-60 model's), the 0012 of C-81's equations (data/c81_0012.py); the
# two blades teeter on the hub [TR card 53: teetering, no flapping spring, no precone], with delta-3 [TR card 54]; the
# teeter stops are not published (C-81's at 90 deg: none) and taken as +-12 deg
R_ROOT_TR = 0.2 * R_TR
NSEG_TR = 6
TEETER_STOP_TR = 12.0 * DEG
# tip loss of the blade-element rotor, Prandtl's 1 - sqrt(2 C_T) / Nb at its hover thrust (C_T about 0.0065): 0.94, on
# the lift outboard of B R and on the momentum (the wake of the effective disc B^2 A, as the main rotor's)
TIP_LOSS_TR_BLADES = 0.94

# ---------------------------------------------------------------- loads, fuel, contacts
# Loading stations [FM 6.3]: pilot and copilot 1890 mm, passengers 2690 mm, the cargo compartment's centre 3900 mm, the
# supply tank 2275 mm and the main tank 3220 mm (both under the cabin floor). Seats laterally and the occupants' mass
# centres 0.25 m above the compressed cushions from the 3-D model (front seats at 0.40 m, the rear bench's at 0 and 0.50
# m); the cargo 0.2 m above the floor. (name, default lb, max lb, FS, BL, WL)
_fs = lambda mm: (mm - 745.0) / 25.4
_SEAT_WL = WL_MODEL0 + 0.28 * IN_PER_M
POINT_MASSES = [('Pilot (right seat)', 180.0, 300.0, _fs(1890), 15.7, _SEAT_WL),
                ('Copilot (left seat)', 0.0, 300.0, _fs(1890), -15.7, _SEAT_WL),
                ('Right passenger', 0.0, 300.0, _fs(2690), 19.7, _SEAT_WL),
                ('Middle passenger', 0.0, 300.0, _fs(2690), 0.0, _SEAT_WL),
                ('Left passenger', 0.0, 300.0, _fs(2690), -19.7, _SEAT_WL),
                ('Load', 0.0, 800.0, _fs(3900), 0.0, WL_MODEL0 - 0.05 * IN_PER_M)]
# main 570 l and supply 93 l usable, Jet A (6.68 lb/US gal) [EA: 580 l fuel tank; bo105-set.xml]
TANKS = [(1005.0, _fs(3220), 0.0, -15.0), (163.7, _fs(2275), 0.0, -15.0)]
SUPPLY_LBS = 163.7
PM_TANK0 = len(POINT_MASSES)

# the mast moment indicator [FM 2.17.3, 7.7.5]: no figure is published for its red radial; taken as the hub moment
# of the rotor's flapping stiffness, (Nb/2) K_beta [Padfield, table 5.2: K_beta 113,330 N m/rad], with the disc 5 deg
# off the shaft: the slope limit in all directions without the indicator [FM 2.17.4]; 19.8 kN m
MM_LIMIT_NM = NB / 2 * 113330.0 * math.radians(5.0)

# ---------------------------------------------------------------- empennage tables
# rotor hub, shaft and controls with the blades off [TM-110356 aerodynamic tares: DRAGH 4.2 Q at zero shaft angle]
HUB_DRAG_FT2 = 4.2
# skid landing gear, measured on the 3-D model (skid_L/R): two cross tubes per side, 75 mm, 0.86 m exposed from the
# fuselage's lower edge (BL 20, WL -26) to the skid, in the y-z plane; skid tubes 85 mm, 2.3 m long with a 0.3 m upturned
# nose. Drag areas by the cross-flow principle, circular cylinders at subcritical Reynolds numbers (Cd 1.2, Re ~ 2e5),
# the skids' noses Cd 1.0, friction 0.0045 [Hoerner, Fluid-Dynamic Drag, ch. 3]: per body axis, F = -q_axis f_axis.
# Not in the 1/4-scale static model's data (fuselage and fin, for the tail rotor "kick" tests [TR fig. 14, ref. 10]).
def _gear():
    ct = 4 * 0.86 * 0.075 * 1.2                    # cross tubes, flow normal to them (along x)
    ay, az = 0.7 / 0.86, 0.5 / 0.86                 # their axis' y and z components
    s3 = lambda c: (1 - c * c) ** 1.5               # cross-flow drag along an axis at acos(c) to the tube
    skid_x = 2 * (0.3 * 0.085 * 1.0 + 3.1416 * 0.085 * 2.3 * 0.0045)
    fx = ct + skid_x
    fy = ct * s3(ay) + 2 * 2.6 * 0.085 * 1.2
    fz = ct * s3(az) + 2 * 2.3 * 0.085 * 1.2
    return tuple(f * 10.7639 for f in (fx, fy, fz))
GEAR_F = _gear()                                    # ft^2: about 4.0, 6.4, 6.9
# its centre: the cross tubes' mid-height between them, the skids below (model x -0.56 m)
FS_GEAR, WL_GEAR = FS_MODEL0 - 0.56 * IN_PER_M, WL_MODEL0 - 0.75 * IN_PER_M

# end plates of the horizontal stabiliser, from the 3-D model: about 0.84 m high, 0.5 m chord, 0.22 m^2 each (side
# projection), at the stabiliser's tips (1.05 m out) [EA: stabiliser with end plates]
S_EP = 0.22 * 10.7639
FS_EP, BL_EP, WL_EP = model_to_station(4.45, 1.05, 0.77)


def stab_tables():
    """horizontal stabiliser, 0..90 deg: CL (3-D slope 0.0836/deg to 12 deg, stall at 13), CD (profile 0.0089 [TR card
    77], induced CL^2 / (pi 0.9 AR)); flat plate past the stall (normal force 1.2 sin a)"""
    xs = [0, 4, 8, 12, 13, 15, 18, 22, 30, 45, 60, 75, 90]
    cl, cd = [], []
    for a in xs:
        ar = math.radians(a)
        if a <= 12:
            c = 0.0836 * a; d = 0.0089 + c * c / (math.pi * 0.9 * AR_H)
        elif a <= 13:
            c = 1.0; d = 0.0089 + c * c / (math.pi * 0.9 * AR_H)
        else:
            cn = 1.2 * math.sin(ar)
            fp_cl, fp_cd = cn * math.cos(ar), cn * math.sin(ar)
            w = min(1.0, (a - 13) / 9.0)
            c = (1 - w) * 0.85 + w * fp_cl; d = (1 - w) * 0.12 + w * fp_cd
        cl.append(c); cd.append(d)
    return xs, cl, cd


def endplate_tables():
    """end plate (aspect ratio about 1.4): lift slope 2.4/rad (Helmbold), stall near 25 deg, flat plate beyond"""
    xs = [0, 5, 10, 15, 20, 25, 30, 45, 60, 75, 90]
    cl, cd = [], []
    for a in xs:
        ar = math.radians(a)
        if a <= 20:
            c = 2.4 * ar; d = 0.012 + c * c / (math.pi * 0.85 * 1.4)
        else:
            cn = 1.15 * math.sin(ar)
            w = min(1.0, (a - 20) / 15.0)
            c = (1 - w) * 2.4 * math.radians(20) + w * cn * math.cos(ar)
            d = (1 - w) * (0.012 + (2.4 * math.radians(20)) ** 2 / (math.pi * 0.85 * 1.4)) + w * cn * math.sin(ar)
        cl.append(c); cd.append(d)
    return xs, cl, cd

# the fin in the tail rotor's flow (as the UH-60 model's fin blockage): the 1/4-scale model's data carry the fin without a
# tail rotor [TR fig. 14], so the tail rotor's induced air velocity over the fin (ring-vortex tables, trwake.py) adds the
# difference of the fin's own forces with and without it, strip by strip. Pushing, the tail rotor draws its inflow past
# the fin (with the thrust reversed, it blows its wake onto it). False: none.
TR_FIN = True
# fin: the 3-D model's (trwake.py), 0.38 m^2 (Padfield's 0.805 m^2 [table 4B.2] is about the fin's and the end plates'
# 0.44 together), 0.9 m high, its leading edge swept 38 deg; set to push the tail right in forward flight [Padfield,
# table 4B.2: beta_fn0 -0.0812 rad]
FIN_INC = -4.65


def fin_tables():
    """fin (aspect ratio about 2.2, swept, the boom and the gearbox at its ends): lift slope 2.6/rad (Helmbold, swept),
    stall near 20 deg, flat plate beyond (normal force 1.15)"""
    xs = [0, 5, 10, 15, 20, 25, 30, 45, 60, 75, 90]
    cl, cd = [], []
    a0, k = 2.6, 1 / (math.pi * 0.85 * 2.2)
    for a in xs:
        ar = math.radians(a)
        if a <= 18:
            c = a0 * ar; d = 0.012 + k * c * c
        else:
            cn = 1.15 * math.sin(ar)
            w = min(1.0, (a - 18) / 15.0)
            c = (1 - w) * a0 * math.radians(18) + w * cn * math.cos(ar)
            d = (1 - w) * (0.012 + k * (a0 * math.radians(18)) ** 2) + w * cn * math.sin(ar)
        cl.append(c); cd.append(d)
    return xs, cl, cd

# the airframe's loads (fuselage, hub and skid drag, empennage, fin blockage) are worked out every AIRFRAME_EXECRATE
# frames of the FDM rate: 4 at 960 Hz is 240 Hz (in FlightGear every property read takes a lock: the model's reads are
# its cost there)
AIRFRAME_EXECRATE = 4
# so are the engines (gas generator, governor, temperatures), the fuel, the ambient conditions and the outputs to
# FlightGear (the 3-D rotor, the gauges, the cockpit's vibration), which change slowly or are read once a frame; the
# rotor speed, the drive train and the power turbines' torque, which damps the drive train mode, stay at the FDM rate
SLOW_EXECRATE = 4

# the main rotor's wake on the fuselage, the stabiliser, its end plates, the fin and the tail rotor (ring-vortex tables,
# mrwake.py)
WAKE_ON_AIRFRAME = True
