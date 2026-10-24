#!/usr/bin/env python3
"""The Bo 105's rotors driven blade by blade by the flight model, with a motion blur of the blades' own geometry
(the method and the blur of the UH-60 model's fdm/rotorvis.py).

The 3-D model turned its blades with the YASim rotor's properties, bent them by fixed fractions of one flap angle and
showed translucent discs as the rotors sped up (material animations, which the HDR pipeline leaves out). Here:

- one main rotor blade's objects (blade1 and its segments a-e, the blade fitting, mounting fork, pitch horn, absorber
  and pendulum) are taken out of bo105.ac into Models/Rotor/blade.ac, renamed (mr_...: bo105.xml's animations find
  objects by name in its submodels too, and hide bo105.ac's own), and instanced on the hub four times (blade.xml,
  each instance on its blade's properties); bo105.ac's own blades and discs are hidden;
- the hub turns to the flight model's rotor azimuth (/rotors/main/blade[0]/position-deg, anticlockwise from above,
  from the tail). The head stays on the 3-D model's mast, bo105data.MODEL_DX aft of the FDM's hub: moved there, the
  mast stood on the front edge of its fairing, apart from its control rods;
- each blade has the hub's built-in coning (2.5 deg, drawn from the blade's root: the 3-D model's hub arms are flat),
  its pitch about the feathering bearing (collective and cyclic,
  /rotors/main/blade[k]/incidence-deg) and its elastic flap and lag: the FDM's flap-deg is the coning plus the tip's
  deflection over R, lag-deg the in-plane one (lead +). The blade, a hingeless one, bends with them: each segment
  turns at its inboard joint by the change of the mode shape's mean slope from the segment inboard of it
  (blademodes.py's first flap and lag modes), so that the segments' ends lie on the deflected blade;
- as the rotor speeds up each blade trails a fan of STRIPS faint wedges over SWEEP deg per rpm, which reaches the next
  blade at flight rpm: a translucent disc as dense on average as the blades' time-averaged coverage x GAIN, a smear at
  the blade's own position (fan_ac, as the UH-60's); opaque below SOLID_RPM, the blade fades out to nothing at
  FADE_RPM as the fan fades in (Models/Effects/RotorBlur in the HDR pipeline; elsewhere they switch). The fan cones
  and lags with the blade's tip but doesn't pitch;
- the livery picks the blades' paint (as the UH-60's): sim/model/bo105/material/rotor/blade and blur name the
  textures (Models/Rotor: the variants' black, orange-tipped or white-striped blade, PAINTS, and the fan's blur-<paint>.png,
  the same swatches in each paint's colours along the span), the pbr animation for the HDR pipeline, the material one
  for the others;
- the tail rotor likewise: its forward blade out of bo105.ac (Models/Rotor/tail-blade.ac, about its hub), instanced
  twice on the old hub's spin and toe, each at its pitch (/rotors/tail/blade[k]/incidence-deg), its own fan at the
  tail rotor's rpm; the old blades and disc hidden.

In FlightGear's model frame (x aft, y right, z up) main blade 1 lies along +x with its leading edge toward +y; the tail
rotor's forward blade along -x from its hub, in the x-z plane, turning about +y (forward blade up). A rotate animation
turns by the right-hand rule, and the first animation of an object is the outermost (the hub's side).

    python3 rotorvis.py      writes MBB_Bo_105/Models/Rotor/ (blade.ac, blade-rotor.ac, blade.xml, the paints, blur.png,
                             blur-<paint>.png and the tail-* files) and the rotors' part of Models/bo105.xml
"""
import math, os, re, shutil
from PIL import Image

import bo105data as U
import blademodes
import materials

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'MBB_Bo_105', 'Models')
DIR = os.path.join(MODELS, 'Rotor')

RPM = U.OMEGA * 30 / math.pi            # 100 %: 424 rpm
STRIPS = 24                             # wedges per blade
SWEEP = 90.0 / RPM                      # deg per rpm: the fan reaches the next blade at 100 % rotor speed
GAIN = 2.5                              # x the blades' time-averaged coverage
IMAGE, SMEAR = 0.5, 2.0                 # the share of it in the blades' smears; their decay, in wedges
SOLID_RPM, FADE_RPM = 0.08 * RPM, 0.62 * RPM      # the opaque blade below; from it to FADE_RPM the blade fades out, the fan in
DARK = (0.15, 0.15, 0.15)               # the blades' paint (bo105.ac's material "black")
PAINTS = ['black', 'orange', 'white']   # the variants' blade paints (Models/Textures/Rotor); black, the first, the default
CHORD = 0.27                            # m [AR]
Z_HUB = 1.8143                          # the blades' plane and the feathering axis in the 3-D model (its old pitch centre)
JOINTS = [0.365, 1.007322, 1.807322, 2.607322, 3.307322, 4.007321]   # m: the blade segments' inboard ends
TIP = 4.906
PIECES = 16
SPAN = [(JOINTS[0] * (TIP / JOINTS[0]) ** (i / PIECES), JOINTS[0] * (TIP / JOINTS[0]) ** ((i + 1) / PIECES)) for i in range(PIECES)]
PRECONE = math.degrees(U.PRECONE)

# blade 1's objects in bo105.ac -> their names in blade.ac
SOURCE = {'blade1': 'mr_blade', 'blade1a': 'mr_blade_a', 'blade1b': 'mr_blade_b', 'blade1c': 'mr_blade_c',
          'blade1d': 'mr_blade_d', 'blade1e': 'mr_blade_e', 'absorber1': 'mr_absorber', 'pendulum1': 'mr_pendulum',
          'blade_fitting1': 'mr_fitting', 'blade_mounting_fork1': 'mr_fork', 'pitch_horn1': 'mr_horn'}
SEGMENTS = ['mr_blade', 'mr_blade_a', 'mr_blade_b', 'mr_blade_c', 'mr_blade_d', 'mr_blade_e']
ROOT_PARTS = ['mr_absorber', 'mr_pendulum']               # on the root segment
HUB_PARTS = ['mr_fitting', 'mr_fork', 'mr_horn']
TBLADE = ['t' + s for s in SEGMENTS]                      # the blade's transparent copy, which fades out
PENDULUM = ((0.745474, 0.0, 1.846635), 'sim/model/bo105/absorber-angle-deg')

# bo105.ac's own rotor: its blades and discs (hidden) and the rotating head
OLD = ['%s%d%s' % (o, k, s) for k in range(1, 5) for o, s in
       [('blade', ''), ('blade', 'a'), ('blade', 'b'), ('blade', 'c'), ('blade', 'd'), ('blade', 'e'), ('blade_fitting', ''),
        ('blade_mounting_fork', ''), ('pitch_horn', ''), ('absorber', ''), ('pendulum', ''), ('disc', ''), ('disc', 'a'),
        ('disc', 'b'), ('disc', 'c'), ('disc', 'd'), ('disc', 'e')]]
HEAD = ['star_hub', 'star_cap1', 'star_cap2', 'star_cap3', 'star_cap4', 'rotoraxis', 'swashplate_rotor', 'swashplate_cap',
        'pitch_link1', 'pitch_link2', 'pitch_link3', 'pitch_link4']
BEGIN, END = '\t<!-- main rotor: written by fdm/rotorvis.py -->', '\t<!-- end of fdm/rotorvis.py\'s main rotor -->'

# the tail rotor: hub (the old pitch axes' centre), its blades' span (from the hub) and chord, 100 % rpm
TR_HUB = (5.890, -0.410, 1.910)
TR_SPIN_Y = -0.232                      # the old spin axis's point (any y on it)
TR_TOE = 4.6                            # deg about z: the old model's toe of the tail rotor's shaft
TR_RPM = RPM * U.GEAR_TR
TR_SWEEP = 180.0 / TR_RPM
TR_GAIN = 1.5                           # its solidity is higher than the main rotor's (as the UH-60's)
TR_SOLID, TR_FADE = SOLID_RPM * U.GEAR_TR, FADE_RPM * U.GEAR_TR
TR_ROOT, TR_TIP = 0.12, 0.955           # m (tailrotor_blade1 reaches 0.955 m from the hub)
TR_SPAN = [(TR_ROOT * (TR_TIP / TR_ROOT) ** (i / 12), TR_ROOT * (TR_TIP / TR_ROOT) ** ((i + 1) / 12)) for i in range(12)]
TR_CHORD = U.CHORD_TR * 0.3048
TR_SOURCE = {'tailrotor_blade1': 'tr_blade'}
TR_TBLADE = ['ttr_blade']
TR_OLD = ['tailrotor_blade1', 'tailrotor_blade2', 'rotor_disc_T']
TR_HEAD = ['tailsleeverot', 'tailrotorshaft', 'tailpitchlink_1', 'tailpitchlink_2', 'fake_axis']
TR_FIXED = ['tailrotorgearbox', 'tailsleevefix']
TR_BEGIN, TR_END = '\t<!-- tail rotor: written by fdm/rotorvis.py -->', '\t<!-- end of fdm/rotorvis.py\'s tail rotor -->'


def strip(k, prefix='mr'):
    return '%s_blur%d' % (prefix, k)


def slopes(mode):
    """the segments' mean slopes (rad of the segment per unit of the mode's coordinate, tip deflection / R)"""
    md = blademodes.modes()
    w = [blademodes.phi(md, mode, r / 0.3048)[0] * 0.3048 for r in JOINTS + [TIP]]      # m, for a tip deflection of R
    return [(w[k + 1] - w[k]) / (r1 - r0) for k, (r0, r1) in enumerate(zip(JOINTS, JOINTS[1:] + [TIP]))]


def ac_blocks(path):
    text = open(path, encoding='latin-1').read()
    head, *objs = re.split(r'(?m)^(?=OBJECT poly)', text)
    return head, {re.search(r'^name "([^"]+)"', b, re.M).group(1): b for b in objs}


def extract(source, path, loc=None):
    """the objects of bo105.ac named in source (all at its top level, without loc or rot) renamed into their own ac,
    the rotor texture copied alongside; loc (FlightGear frame, m) shifts them"""
    head, objs = ac_blocks(os.path.join(MODELS, 'bo105.ac'))
    mats = [l for l in head.split('\n') if l.startswith('MATERIAL')]
    blocks = []
    for old, new in source.items():
        b = objs[old].rstrip('\n')
        b = b[:b.index('\nkids 0') + len('\nkids 0')]     # a group's end may follow the object: keep only it
        b = b.replace('texture "Textures/Rotor/black.png"', 'texture "black.png"')
        b = b.replace('texture "Textures/Detail/', 'texture "../Textures/Detail/')     # fdm/materials.py's
        b = re.sub(r'(?m)^name "[^"]+"', 'name "%s"' % new, b)
        if loc:
            b = re.sub(r'(?m)^(name "[^"]+")$', lambda m: '%s\nloc %.5f %.5f %.5f' % (m.group(1), loc[0], loc[2], -loc[1]), b)
        blocks.append(b)
    out = ['AC3Db'] + mats + ['OBJECT world', 'kids %d' % len(blocks)] + blocks
    open(path, 'w', encoding='latin-1').write('\n'.join(out) + '\n')


def paints(span):
    """each of PAINTS' colours over span's pieces: the blade texture's mean along the piece (blade.ac's segments map
    the span linearly onto its v), DARK where the paint is black"""
    head, objs = ac_blocks(os.path.join(DIR, 'blade.ac'))
    pts = []
    for name in SEGMENTS:
        L = objs[name].split('\n')
        loc = next(([float(c) for c in l.split()[1:]] for l in L if l.startswith('loc')), [0.0, 0.0, 0.0])
        i = next(k for k, l in enumerate(L) if l.startswith('numvert')); n = int(L[i].split()[1])
        x = [float(L[i + 1 + k].split()[0]) + loc[0] for k in range(n)]
        pts += [(x[int(m.group(1))], float(m.group(2)), float(m.group(3))) for m in
                (re.match(r'^(\d+) (\S+) (\S+)$', l) for l in L[i + n + 1:]) if m]
    mx, mv = sum(p[0] for p in pts) / len(pts), sum(p[2] for p in pts) / len(pts)
    b = sum((p[0] - mx) * (p[2] - mv) for p in pts) / sum((p[0] - mx) ** 2 for p in pts); a = mv - b * mx
    u = sum(p[1] for p in pts) / len(pts)
    out = {}
    for paint in PAINTS:
        im = Image.open(os.path.join(DIR, paint + '.png')).convert('RGB'); W, H = im.size
        col = []
        for r0, r1 in span:
            y0, y1 = sorted(H * (1 - (a + b * r)) for r in (r0, r1))
            px = [im.getpixel((int(u * W), y)) for y in range(int(y0), max(int(y0) + 1, int(round(y1))))]
            px = [DARK if sum(q) < 3 * 255 * DARK[0] else tuple(c / 255.0 for c in q) for q in px]
            col.append(tuple(round(sum(q[c] for q in px) / len(px), 3) for c in range(3)))
        out[paint] = col
    return out


def fan_ac(src, dst, blur, segments, span, chord, nb, gain, point, prefix, colours=None):
    """dst: src's objects, the fan's wedges (all at the blade: the blade's XML turns them back) and the blade's fading
    copy (segments, renamed t...), in one model so that the XML's effects reach them. The fan spans 360 / nb deg at
    flight rpm; along the span the disc is as opaque on average as the blades' time-averaged coverage x gain. The
    wedges' colour and opacity are a swatch of blur each (the HDR transparent shader takes its alpha from the
    texture). point(r, a): a point of the rotor's plane at radius r, a rad along the rotation, ac frame. colours
    ({paint: its colour on each piece of span}, else DARK): blur-<paint>.png beside blur, the same swatches in each
    paint's colours; blur the first paint's."""
    d = math.radians(360.0 / nb / (STRIPS - 1))             # their spacing at flight rpm; each 2 d wide
    names = list(colours or [])
    texs = [Image.new('RGBA', (256, 256), (0, 0, 0, 0)) for _ in range(1 + len(names))]; cells = []; seen = {}
    g = [math.exp(-k / SMEAR) for k in range(STRIPS)]

    def swatch(rgbs, a):
        key = (rgbs, int(round(255 * a)))
        if key in seen:
            return seen[key]
        n = len(cells); x, y = 8 * (n % 32), 8 * (n // 32)
        assert n < 1024, '%s is full' % blur
        for tex, rgb in zip(texs, rgbs[:1] + rgbs):
            tex.paste(tuple(int(round(255 * c)) for c in rgb) + (int(round(255 * a)),), (x, y, x + 8, y + 8))
        cells.append(((x + 4) / 256.0, 1 - (y + 4) / 256.0)); seen[key] = cells[-1]
        return cells[-1]
    text = open(src, encoding='latin-1').read()
    head, *orig = re.split(r'(?m)^(?=OBJECT poly)', text)
    mats = [l for l in head.split('\n') if l.startswith('MATERIAL')]
    mats.append('MATERIAL "blur" rgb 1 1 1  amb 1 1 1  emis 0 0 0  spec 0 0 0  shi 0  trans 0')
    blur_mat = len(mats) - 1
    objs = [b.rstrip('\n') for b in orig]
    for k in range(STRIPS):
        w = (1 - IMAGE) * (0.5 if k in (0, STRIPS - 1) else 1.0) + IMAGE * STRIPS * g[k] / sum(g)   # mean 1
        faces = []
        for i, (r0, r1) in enumerate(span):
            sigma = nb * chord / (2 * math.pi * math.sqrt(r0 * r1))
            a = min(0.9, gain * sigma / 2 * w)              # two wedges over each point
            rgbs = tuple(colours[p][i] for p in names) if names else (DARK,)
            faces.append((swatch(rgbs, a), [point(r0, 0.0), point(r1, 0.0), point(r1, -2 * d), point(r0, -2 * d)]))
        V = [pt for _, pts in faces for pt in pts]
        lines = ['OBJECT poly', 'name "%s"' % strip(k, prefix), 'loc 0 0 0', 'texture "%s"' % os.path.basename(blur),
                 'texrep 1 1', 'numvert %d' % len(V)] + ['%.5f %.5f %.5f' % p for p in V]
        lines.append('numsurf %d' % len(faces))
        for f, (uv, _) in enumerate(faces):
            lines += ['SURF 0x30', 'mat %d' % blur_mat, 'refs 4'] + ['%d %.5f %.5f' % ((4 * f + q,) + uv) for q in range(4)]
        objs.append('\n'.join(lines + ['kids 0']))
    for b in orig:
        m = re.search(r'^name "([^"]+)"', b, re.M)
        if m and m.group(1) in segments:
            objs.append(re.sub(r'(?m)^name "[^"]+"', 'name "t%s"' % m.group(1), b).rstrip('\n'))
    texs[0].save(blur)
    for p, tex in zip(names, texs[1:]):
        tex.save(os.path.join(os.path.dirname(blur), 'blur-%s.png' % p))
    out = ['AC3Db'] + mats + ['OBJECT world', 'kids %d' % len(objs)] + objs
    open(dst, 'w', encoding='latin-1').write('\n'.join(out) + '\n')


def objects(names, indent='    '):
    return ''.join('%s<object-name>%s</object-name>\n' % (indent, n) for n in names)


def rotate(objs, prop, axis, centre=(0, 0, 0), factor=None, offset=None, alias=False, comment=None):
    out = '\n  <!-- %s -->\n' % comment if comment else '\n'
    out += '  <animation>\n    <type>rotate</type>\n' + objects(objs)
    if prop:
        out += ('    <property alias="%s"/>\n' % prop) if alias else ('    <property>%s</property>\n' % prop)
    if factor is not None:
        out += '    <factor> %.6g </factor>\n' % factor
    if offset is not None:
        out += '    <offset-deg> %.6g </offset-deg>\n' % offset
    out += ('    <center>\n      <x-m> %.5f </x-m>\n      <y-m> %.5f </y-m>\n      <z-m> %.5f </z-m>\n    </center>\n'
            '    <axis>\n      <x> %g </x>\n      <y> %g </y>\n      <z> %g </z>\n    </axis>\n  </animation>\n'
            % (tuple(centre) + tuple(axis)))
    return out


def select(objs, cond, comment=None):
    out = '\n  <!-- %s -->\n' % comment if comment else '\n'
    return out + '  <animation>\n    <type>select</type>\n' + objects(objs) + '    <condition>\n%s    </condition>\n  </animation>\n' % cond


def fade(objs, f, t, rough, shadow, comment, rpm=None):
    """the rotor-blur effect on objs, faded from f to t rpm (of /rotors/main/rpm, or rpm)"""
    use = '      <rotor-rpm type="float"><use>%s</use></rotor-rpm>\n' % rpm if rpm else ''
    return ('\n  <!-- %s -->\n  <effect>\n    <inherits-from>../Effects/RotorBlur/rotor-blur</inherits-from>\n'
            '    <parameters>\n%s      <fade-from type="float"> %g </fade-from>\n      <fade-to type="float"> %g </fade-to>\n'
            '      <pbr-roughness type="float"> %g </pbr-roughness>\n      <shadow-gain type="float"> %g </shadow-gain>\n'
            '    </parameters>\n' % (comment, use, f, t, rough, shadow) + objects(objs) + '  </effect>\n')


def scale_xml(objs, prop, nominal, axis, comment):
    """the wedges narrowed along axis ('y' or 'z') with the rpm, as wide as twice their spacing at any rpm"""
    f = {a: ('    <%s-factor> %.7f </%s-factor>\n    <%s-offset> 0 </%s-offset>\n    <%s-min> 0.02 </%s-min>\n    <%s-max> 1 </%s-max>\n'
             % (a, 1.0 / nominal, a, a, a, a, a, a, a)) if a == axis else
         ('    <%s-factor> 0 </%s-factor>\n    <%s-offset> 1 </%s-offset>\n' % (a, a, a, a)) for a in 'xyz'}
    return ('\n  <!-- %s -->\n  <animation>\n    <type>scale</type>\n' % comment + objects(objs) +
            '    <property>%s</property>\n' % prop + f['x'] + f['y'] + f['z'] + '  </animation>\n')


def wedges_xml(prefix, prop, sweep):
    out = ''
    for k in range(1, STRIPS):
        out += rotate([strip(k, prefix)], prop, (0, 0, 1) if prefix == 'mr' else (0, 1, 0), factor=-sweep * k / (STRIPS - 1),
                      comment='wedge k trails the blade by k / %d of the fan' % (STRIPS - 1) if k == 1 else None)
    return out


def bends(prop, axis, s, offset_q, comment):
    """the blade's segments turned at their inboard joints by the change of the mean slope, s per unit of the
    coordinate (prop in deg, less offset_q)"""
    out = ''
    prev = 0.0
    for k, (r, sk) in enumerate(zip(JOINTS, s)):
        objs = SEGMENTS[k:] + TBLADE[k:] + (ROOT_PARTS if k == 0 else [])
        f = sk - prev; prev = sk
        out += rotate(objs, '/params/' + prop, axis, (r, 0.0, Z_HUB), factor=f, offset=-f * offset_q, alias=True,
                      comment=comment if k == 0 else None)
    return out


def head_xml(title, path, params):
    return ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- %s, written by fdm/rotorvis.py (see there); bo105.xml gives each\n'
            '     instance its blade\'s properties in params -->\n\n<PropertyList>\n\n  <path>%s</path>\n\n  <params>\n%s  </params>\n'
            % (title, path, ''.join('    <%s>%s</%s>\n' % (k, v, k) for k, v in params)))


def visibility(segments, tblade, strips, solid, fade_rpm, prop):
    rpm = lambda r: '<property>%s</property><value> %g </value>' % (prop, r)
    return (select(segments, '      <less-than>%s</less-than>\n' % rpm(solid), 'the opaque blade at low rpm') +
            select(tblade, '      <and>\n        <not><less-than>%s</less-than></not>\n        <less-than>%s</less-than>\n'
                   '      </and>\n' % (rpm(solid), rpm(fade_rpm)), 'then its fading copy') +
            select(strips, '      <not><less-than>%s</less-than></not>\n' % rpm(solid), 'and the fan'))


def livery_textures(objs, prop, default, comment):
    """objs' texture from the livery, sim/model/bo105/material/rotor/prop (else default): the pbr animation for the HDR
    pipeline, the material one for the others; the pbr one has to come first."""
    return ('\n  <!-- %s -->\n  <animation>\n    <type>pbr</type>\n' % comment + objects(objs) +
            '    <property-base>sim/model/bo105/material/rotor</property-base>\n'
            '    <base-color-texture-prop>%s</base-color-texture-prop>\n  </animation>\n' % prop +
            '  <animation>\n    <type>material</type>\n' + objects(objs) +
            '    <property-base>sim/model/bo105/material/rotor</property-base>\n    <texture-prop>%s</texture-prop>\n'
            '    <texture>%s</texture>\n  </animation>\n' % (prop, default))


def part_effects(source):
    """fdm/materials.py's detail texture and PBR on the parts taken out of bo105.ac (source), by their new names"""
    found = [(source[n], k, m, rgb) for n, k, m, rgb in materials.parts() if n in source]
    return '\n  <!-- the parts\' detail texture and PBR (fdm/materials.py) -->\n' + materials.effects_xml(found, materials.means(), '  ')


def blade_xml():
    strips = [strip(k) for k in range(STRIPS)]
    blade = SEGMENTS + TBLADE + ROOT_PARTS
    sf, sl = slopes('f'), slopes('l')
    x = head_xml('One main rotor blade', 'blade-rotor.ac', [('lag', 'rotors/main/blade[0]/lag-deg'),
                 ('flap', 'rotors/main/blade[0]/flap-deg'), ('pitch', 'rotors/main/blade[0]/incidence-deg')])
    x += livery_textures(SEGMENTS + TBLADE, 'blade', PAINTS[0] + '.png',
                         'the livery\'s paint (Models/Variants): the blade, plain, orange-tipped or white-striped')
    x += livery_textures(strips, 'blur', 'blur.png', 'and its blur')
    x += part_effects(SOURCE)
    x += fade(strips, SOLID_RPM, FADE_RPM, 1.0, 2.0, 'the fan fades in with the rpm; its shadow x 2: two wedges over each point')
    x += fade(TBLADE, FADE_RPM, SOLID_RPM, 0.5, 1.0, 'the blade fades out')
    x += visibility(SEGMENTS, TBLADE, strips, SOLID_RPM, FADE_RPM, 'rotors/main/rpm')
    x += wedges_xml('mr', 'rotors/main/rpm', SWEEP)
    x += rotate(blade, None, (0, -1, 0), (JOINTS[0], 0, Z_HUB), offset=PRECONE,
                comment='the built-in coning, from the blade\'s root (the fittings stay on the hub\'s arms, which the 3-D model draws flat)')
    x += rotate(strips, None, (0, -1, 0), (0, 0, Z_HUB), offset=PRECONE)
    x += rotate(strips, '/params/lag', (0, 0, 1), (0, 0, Z_HUB), alias=True, comment='the fan lags with the blade\'s tip, lead +')
    x += bends('lag', (0, 0, 1), sl, 0.0, 'the in-plane bending: the first lag mode, lead +')
    x += rotate(strips, '/params/flap', (0, -1, 0), (0, 0, Z_HUB), offset=-PRECONE, alias=True,
                comment='the fan cones with the blade\'s tip, up +')
    x += bends('flap', (0, -1, 0), sf, PRECONE, 'the flap bending: the first flap mode, up +')
    x += rotate(blade + HUB_PARTS, '/params/pitch', (1, 0, 0), (0, 0, Z_HUB), alias=True,
                comment='pitch about the feathering bearing: collective and cyclic, leading edge up +')
    x += rotate(['mr_pendulum'], PENDULUM[1], (0, -1, 0), PENDULUM[0], comment='the pendulum absorber (Nasal/bo105.nas)')
    x += scale_xml(strips, 'rotors/main/rpm', RPM, 'y', 'the wedges as wide as twice their spacing at any rpm (blade-rotor.ac: at 100 %)')
    open(os.path.join(DIR, 'blade.xml'), 'w', encoding='latin-1').write(x + '\n</PropertyList>\n')
    return sf, sl


def tail_xml():
    strips = [strip(k, 'tr') for k in range(STRIPS)]
    x = head_xml('One tail rotor blade', 'tail-blade-rotor.ac', [('pitch', 'rotors/tail/blade[0]/incidence-deg')])
    x += part_effects(TR_SOURCE)
    x += fade(strips, TR_SOLID, TR_FADE, 1.0, 2.0, 'the fan fades in with the rpm; its shadow x 2: two wedges over each point',
              rpm='/rotors/tail/rpm')
    x += fade(TR_TBLADE, TR_FADE, TR_SOLID, 0.5, 1.0, 'the blade fades out', rpm='/rotors/tail/rpm')
    x += visibility(['tr_blade'], TR_TBLADE, strips, TR_SOLID, TR_FADE, 'rotors/tail/rpm')
    x += wedges_xml('tr', 'rotors/tail/rpm', TR_SWEEP)
    x += rotate(['tr_blade'] + TR_TBLADE, '/params/pitch', (-1, 0, 0), alias=True, comment='pitch (the pedals)')
    x += scale_xml(strips, 'rotors/tail/rpm', TR_RPM, 'z', 'the wedges as wide as twice their spacing at any rpm')
    open(os.path.join(DIR, 'tail-blade.xml'), 'w', encoding='latin-1').write(x + '\n</PropertyList>\n')


def tab(s):
    return '\n'.join(re.sub(r'^((?:  )+)', lambda m: '\t' * (len(m.group(1)) // 2), l) for l in s.strip('\n').split('\n'))


def instance(name, path, params, offsets):
    o = ''.join('\t\t\t<%s>%s</%s>\n' % (k, v, k) for k, v in offsets)
    return ('\t<model>\n\t\t<name>%s</name>\n\t\t<path>Aircraft/MBB_Bo_105/Models/Rotor/%s</path>\n\t\t<overlay>\n\t\t\t<params>\n%s'
            '\t\t\t</params>\n\t\t</overlay>\n%s\t</model>' % (name, path, ''.join('\t\t\t\t<%s>%s</%s>\n' % (k, v, k) for k, v in params),
                                                         '\t\t<offsets>\n%s\t\t</offsets>\n' % o if o else ''))


def model_xml():
    """the main rotor's part of bo105.xml, between BEGIN and END"""
    x = [BEGIN, '\t<!-- the blades: Models/Rotor/blade.xml on blade[k]\'s properties, k anticlockwise from the tail -->']
    for k in range(U.NB):
        x.append(instance('Blade%d' % (k + 1), 'blade.xml', [('lag', 'rotors/main/blade[%d]/lag-deg' % k),
                          ('flap', 'rotors/main/blade[%d]/flap-deg' % k), ('pitch', 'rotors/main/blade[%d]/incidence-deg' % k)],
                          [('heading-deg', 90 * k)] if k else []))
    x.append(tab(select(OLD, '      <property>sim/model/bo105/old-rotor</property>\n', 'bo105.ac\'s own blades and discs: not shown')))
    x.append('\n\t<animation>\n\t\t<name>main_rotor_head</name>\n' + objects(HEAD + ['Blade%d' % (k + 1) for k in range(U.NB)], '\t\t') + '\t</animation>')
    x.append(tab(rotate(['main_rotor_head'], None, (0, 1, 0), (0, 0, 1.8), offset=-math.degrees(U.SHAFT_TILT),
                        comment='the shaft tilted %.0f deg forward' % math.degrees(U.SHAFT_TILT))))
    x.append(tab(rotate(['main_rotor_head'], 'rotors/main/blade[0]/position-deg', (0, 0, 1),
                        comment='the head at blade 0\'s azimuth: anticlockwise from above, from the tail')))
    x.append(END)
    return '\n'.join(x) + '\n'


def tail_model_xml():
    """the tail rotor's part of bo105.xml, between TR_BEGIN and TR_END"""
    x = [TR_BEGIN, '\t<!-- the blades: Models/Rotor/tail-blade.xml about the hub, blade k on blade[k]\'s pitch -->']
    hub = [('x-m', '%.3f' % TR_HUB[0]), ('y-m', '%.3f' % TR_HUB[1]), ('z-m', '%.3f' % TR_HUB[2])]
    for k in range(U.NB_TR):
        x.append(instance('TailBlade%d' % (k + 1), 'tail-blade.xml', [('pitch', 'rotors/tail/blade[%d]/incidence-deg' % k)],
                          hub + ([('pitch-deg', 180)] if k else [])))
    x.append(tab(select(TR_OLD, '      <property>sim/model/bo105/old-rotor</property>\n', 'bo105.ac\'s own blades and disc: not shown')))
    x.append('\n\t<animation>\n\t\t<name>tail_rotor_head</name>\n' + objects(TR_HEAD + ['TailBlade%d' % (k + 1) for k in range(U.NB_TR)], '\t\t') + '\t</animation>')
    x.append(tab(rotate(['tail_rotor_head'] + TR_FIXED, None, (0, 0, 1), (TR_HUB[0], 0, TR_HUB[2]), offset=TR_TOE,
                        comment='the shaft\'s toe')))
    x.append(tab(rotate(['tail_rotor_head'], 'rotors/tail/blade[0]/position-deg', (0, 1, 0), (TR_HUB[0], TR_SPIN_Y, TR_HUB[2]),
                        comment='the hub at blade 0\'s azimuth')))
    x.append(TR_END)
    return '\n'.join(x) + '\n'


def install(block, begin, end, old_from, old_to):
    p = os.path.join(MODELS, 'bo105.xml'); s = open(p).read()
    if begin in s:
        i, j = s.index(begin), s.index(end) + len(end) + 1
    else:
        i, j = s.index(old_from), s.index(old_to)
        block += '\n'
    s = s[:i] + block + s[j:]
    s = s.replace('\t<!-- the model\'s frame is the JSBSim model\'s (its origin, the VRP, on the rotor axis: fdm/bo105data.model_to_station) -->',
                  '\t<!-- the model\'s frame is the JSBSim model\'s: its origin is the VRP (fdm/bo105data.model_to_station) -->')
    open(p, 'w').write(s)


if __name__ == '__main__':
    os.makedirs(DIR, exist_ok=True)
    extract(SOURCE, os.path.join(DIR, 'blade.ac'))
    for p in PAINTS:
        shutil.copy(os.path.join(MODELS, 'Textures', 'Rotor', p + '.png'), DIR)
    fan_ac(os.path.join(DIR, 'blade.ac'), os.path.join(DIR, 'blade-rotor.ac'), os.path.join(DIR, 'blur.png'), SEGMENTS, SPAN, CHORD,
           U.NB, GAIN, lambda r, a: (r * math.cos(a), Z_HUB, -r * math.sin(a)), 'mr', paints(SPAN))      # x-y plane, about +z
    sf, sl = blade_xml()
    extract(TR_SOURCE, os.path.join(DIR, 'tail-blade.ac'), loc=tuple(-c for c in TR_HUB))
    fan_ac(os.path.join(DIR, 'tail-blade.ac'), os.path.join(DIR, 'tail-blade-rotor.ac'), os.path.join(DIR, 'tail-blur.png'),
           ['tr_blade'], TR_SPAN, TR_CHORD, U.NB_TR, TR_GAIN, lambda r, a: (-r * math.cos(a), r * math.sin(a), 0.0), 'tr')   # x-z, about +y
    tail_xml()
    install(model_xml(), BEGIN, END, '\t<!-- main rotor tilt -->', '<!--== rotor material ==-->')
    install(tail_model_xml(), TR_BEGIN, TR_END, '\t<!-- tail rotor disc -->', BEGIN)
    print('segment slopes per unit tip deflection / R: flap %s, lag %s' % (' '.join('%.3f' % v for v in sf), ' '.join('%.3f' % v for v in sl)))
    print('main rotor: %d wedges per blade over 90 deg at %.0f rpm, fan from %.0f rpm, blade gone at %.0f; tail rotor at %.0f rpm, '
          'from %.0f, gone at %.0f' % (STRIPS, RPM, SOLID_RPM, FADE_RPM, TR_RPM, TR_SOLID, TR_FADE))
