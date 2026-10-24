"""The Bo 105's instrument panel after the flight manual's figure 7-3, on the 3-D model's own console and under its
glareshield (they were made for a wider panel; the manual's fits the console's face): the face plate (lettering,
screws), the warning light segments (figure item 16, the legends of section 7.11 fig. 7-19), the engine fire
warning lights and their TEST FIRE pushbuttons, the pitot heat indicator, the instrument light rheostat and the
lamp test pushbutton; the instruments of gauges.py on their places (panel_layout.py).

Writes MBB_Bo_105/Models/Panel/panel.ac, panel.png and panel.xml (the instruments placed in the panel's frame:
y right, z up from the middle of its outline, x toward the crew). Models/bo105.xml places the panel (PLACE): on the
console's face, which leans back 6 deg, centred on the aircraft's centreline [FM fig. 7-2].

    python3 panel.py
"""
import math, os
from PIL import Image, ImageDraw

import acmesh as A
import gauges as G
import panel_layout as L

PANEL = os.path.join(G.INSTR, '..')
# the panel's origin in the model frame (x aft, y right, z up, m) and its pitch: the plate 5 mm proud of the console's
# face, which leans back 6 deg (bo105.ac's object 'cockpit')
PLACE = (-2.235, 0.0, 0.35, -6.0)
GROUND, LETTER = (20, 20, 20), (232, 232, 232)      # the plate black
PROP = '/instrumentation/bo105/'
MATTE = ['face', 'wl-frame', 'fire1-body', 'fire2-body', 'pitot-body', 'test-fire1', 'test-fire2',
         'instr-light', 'lamp-test']

# instruments: figure item -> (instrument folder, xml file)
INSTRUMENTS = {'Airspeed indicator': ('asi', 'asi.xml'), 'Altimeter': ('alt', 'alt.xml'),
               'Vertical speed indicator': ('vsi', 'vsi.xml'), 'Triple tachometer': ('tach', 'tach.xml'),
               'Triple oil temperature indicator': ('oilt', 'oilt.xml'), 'Triple oil pressure indicator': ('oilp', 'oilp.xml'),
               'Dual torque indicator': ('tq', 'tq.xml'), 'Clock': ('clock', 'clock.xml'),
               'TOT indicator, engine 1': ('tot', 'tot1.xml'), 'TOT indicator, engine 2': ('tot', 'tot2.xml'),
               'N1 indicator, engine 1': ('n1', 'n1-1.xml'), 'N1 indicator, engine 2': ('n1', 'n1-2.xml'),
               'Fuel quantity indicator': ('fq', 'fq.xml'), 'Fuel pressure indicator': ('fp', 'fp.xml'),
               'Mast moment indicator': ('mmi', 'mmi.xml'), 'Attitude indicator': ('ai', 'ai.xml'),
               'Horizontal situation indicator': ('hsi', 'hsi.xml'), 'Radio control head': ('radio', 'radio.xml'),
               'GPS': ('gps', 'gps.xml')}
# warning light segments, the figure's: (column, row) -> (upper, lower) legends (None: a blank segment)
SEGMENTS = {(0, 0): ('LOW FUEL', 'HY BLOCK'), (1, 0): ('RPM', 'T OIL'), (0, 1): ('GEN 1', 'GEN 2'),
            (1, 1): ('FILT 1', 'FILT 2'), (0, 2): ('HYD 1', 'HYD 2'), (1, 2): ('MAG PLUG 1', 'MAG PLUG 2'),
            (0, 3): None, (1, 3): ('BAT 60', 'BAT 70'), (0, 4): None, (1, 4): ('OIL COOL', '')}
# The layout: the Flying Bulls' Bo 105 C (D-HSDM), whose panel is the 3-D model's console (a column 0.30 m wide with a
# section 0.22 m wide in front of the pilot on top): the manual's instruments down the column as there (mast moment
# indicator and the triple oil gauges on the left, the warning segments over the engine gauges in the middle, airspeed,
# torque and triple tachometer on the right), the attitude indicator and altimeter, HSI and vertical speed indicator,
# radio control head and GPS in front of the pilot. Centres in the model frame (y right, z up, m), on the console's
# face; sizes are the manual's (panel_layout.py) and avionics.py's.
CONSOLE = [(-0.141, 0.065), (0.155, 0.065), (0.155, 0.270), (0.373, 0.270), (0.373, 0.601), (-0.141, 0.601)]
PLACES = {'TEST FIRE pushbutton, engine 1': (-0.121, 0.578), 'Fire warning light, engine 1': (-0.096, 0.578),
          'Fire warning light, engine 2': (-0.058, 0.578), 'TEST FIRE pushbutton, engine 2': (-0.030, 0.578),
          'Pitot heat indicator': (0.012, 0.578),
          'Clock': (-0.097, 0.515), 'Mast moment indicator': (-0.097, 0.420),
          'Triple oil temperature indicator': (-0.097, 0.290), 'Triple oil pressure indicator': (-0.097, 0.160),
          'Warning light segments': (-0.010, 0.480), 'Instrument light rheostat': (-0.0215, 0.395),
          'Annunciator lamp test pushbutton': (0.0015, 0.395),
          'TOT indicator, engine 1': (-0.010, 0.290), 'N1 indicator, engine 1': (-0.010, 0.200),
          'Fuel quantity indicator': (-0.010, 0.110),
          'Airspeed indicator': (0.083, 0.550), 'Dual torque indicator': (0.083, 0.472), 'Triple tachometer': (0.083, 0.390),
          'TOT indicator, engine 2': (0.083, 0.290), 'N1 indicator, engine 2': (0.083, 0.200),
          'Fuel pressure indicator': (0.083, 0.110),
          'Attitude indicator': (0.205, 0.550), 'Altimeter': (0.318, 0.550),
          'Horizontal situation indicator': (0.205, 0.458), 'Vertical speed indicator': (0.318, 0.458),
          'Radio control head': (0.262, 0.385), 'GPS': (0.262, 0.314)}
# items the manual's figure doesn't have: (name, case, w, h)
EXTRA = [('Attitude indicator', '3ATI', G.CASE3, G.CASE3), ('Horizontal situation indicator', '3ATI', G.CASE3, G.CASE3),
         ('Radio control head', None, 0.159, 0.042), ('GPS', None, 0.159, 0.084)]
# screws on the face plate, model frame
SCREWS = [(-0.134, 0.594), (0.148, 0.072), (-0.134, 0.072), (0.366, 0.594), (0.366, 0.277), (0.140, 0.594),
          (-0.134, 0.230), (0.148, 0.230)]
COS = math.cos(math.radians(PLACE[3]))


def to_face(y, z):
    """model frame y, z on the console's face -> the panel's frame (its plane leans back with the console)"""
    return y, (z - PLACE[2]) / COS


def placed():
    """the manual's items (their sizes) and the extra ones, on the Flying Bulls' places"""
    its = L.items()
    for name, case, w, h in EXTRA:
        its.append(dict(num=0, name=name, case=case, y=0.0, z=0.0, w=w, h=h, round=case is not None))
    for it in its:
        it['y'], it['z'] = to_face(*PLACES[it['name']])
    return its


def prop_name(legend):
    return legend.lower().replace(' ', '-')


class Atlas:
    """panel.png: the face plate's picture on the left, the rest packed on the right"""
    def __init__(s):
        s.im = Image.new('RGB', (2048, 2048), GROUND); s.d = ImageDraw.Draw(s.im)
        s.free = [1040, 8, 0]; s.n = 0

    def alloc(s, w, h):
        x, y, rh = s.free
        if x + w > 2040:
            x, y, rh = 1040, y + rh + 8, 0
        s.free = [x + w + 8, y, max(rh, h)]
        return x, y, x + w, y + h

    def swatch(s, rgb):
        x = 2048 - 16 * (s.n + 1); y = 2048 - 16; s.n += 1
        s.d.rectangle([x, y, x + 15, y + 15], fill=rgb)
        return A.const((x + 8) / 2048, 1 - (y + 8) / 2048)

    def area(s, px0, py0, px1, py1, y0, y1, z0, z1):
        return A.planar(y0, y1, z0, z1, px0 / 2048, px1 / 2048, 1 - py1 / 2048, 1 - py0 / 2048)

    def picture(s, w, h, draw, y0, y1, z0, z1):
        x0, py0, x1, py1 = s.alloc(w, h)
        sub = Image.new('RGB', (w, h), GROUND); draw(ImageDraw.Draw(sub), w, h); s.im.paste(sub, (x0, py0))
        return s.area(x0, py0, x1, py1, y0, y1, z0, z1)


def face_picture(its):
    """face.png: the face plate's paint, lettering and screws over the console's T (2048 px square over its bounds);
    returns its uv"""
    T = [to_face(y, z) for y, z in CONSOLE]
    y0, y1 = min(p[0] for p in T), max(p[0] for p in T); z0, z1 = min(p[1] for p in T), max(p[1] for p in T)
    N = 2048; k = N / max(y1 - y0, z1 - z0)
    P = lambda y, z: ((y - y0) * k, N - (z - z0) * k)
    im = Image.new('RGB', (N, N), GROUND); d = ImageDraw.Draw(im)
    d.polygon([P(*p) for p in T], outline=(70, 70, 70), width=6)
    item = {it['name']: it for it in its}
    lab = lambda txt, y, z, h: d.text(P(y, z), txt, font=G.font(h * k), fill=LETTER, anchor='mm')
    for n in ('TEST FIRE pushbutton, engine 1', 'TEST FIRE pushbutton, engine 2'):
        it = item[n]; lab('TEST', it['y'], it['z'] + 0.0175, 0.0034); lab('FIRE', it['y'], it['z'] + 0.0115, 0.0034)
    it = item['Pitot heat indicator']; lab('PITOT', it['y'], it['z'] + 0.0175, 0.0030); lab('HEATING', it['y'], it['z'] + 0.0115, 0.0030)
    for n, txt in (('TOT indicator, engine 1', 'ENG 1'), ('TOT indicator, engine 2', 'ENG 2')):
        it = item[n]; lab(txt, it['y'], it['z'] + G.CASE2S / 2 + 0.0055, 0.0042)
    for y, z in SCREWS:
        cx, cy = P(*to_face(y, z)); r = 0.0032 * k
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(95, 95, 95), outline=(20, 20, 20), width=2)
        d.line([cx - r * 0.7, cy, cx + r * 0.7, cy], fill=(25, 25, 25), width=3)
        d.line([cx, cy - r * 0.7, cx, cy + r * 0.7], fill=(25, 25, 25), width=3)
    rh, lt = item['Instrument light rheostat'], item['Annunciator lamp test pushbutton']
    for it, top, bot in ((rh, None, ('INSTR.', 'LIGHT')), (lt, 'TEST', ('LAMP',))):
        if top:
            lab(top, it['y'], it['z'] + 0.0115, 0.0040)
        for j, t in enumerate(bot):
            lab(t, it['y'], it['z'] - 0.0085 - 0.0045 * j, 0.0034)
        d.rectangle([P(it['y'] - it['w'] / 2, it['z'] + it['h'] / 2), P(it['y'] + it['w'] / 2, it['z'] - it['h'] / 2)],
                    outline=(80, 80, 80), width=3)
    cx, cy = P(rh['y'], rh['z'] + 0.0020); r = 0.0095 * k                 # the rheostat's brightness arc
    d.arc([cx - r, cy - r, cx + r, cy + r], 215, 325, fill=LETTER, width=6)
    im.save(os.path.join(PANEL, 'face.png'), optimize=True)
    return T, A.planar(y0, y0 + N / k, z0, z0 + N / k, 0.0, 1.0, 0.0, 1.0)


def legend_picture(text, lit):
    """a warning light legend: unlit the dark red lens with its lettering, lit bright"""
    bg = (215, 40, 30) if lit else (58, 16, 14)
    fg = (255, 240, 225) if lit else (128, 70, 62)
    def draw(d, w, h):
        d.rectangle([0, 0, w, h], fill=bg)
        words = text.split(' ')
        lines = [text] if len(text) <= 6 or len(words) == 1 else [' '.join(words[:-1]), words[-1]]
        if text == 'LOW FUEL' or text == 'HY BLOCK' or text == 'OIL COOL':
            lines = words
        f = G.font(h * (0.36 if len(lines) == 1 else 0.33))
        for j, t in enumerate(lines):
            d.text((w / 2, h * (0.5 + (j - (len(lines) - 1) / 2) * 0.38)), t, font=f, fill=fg, anchor='mm')
    return draw


def build():
    its = placed()
    at = Atlas()
    dark, black, frame = at.swatch(GROUND), at.swatch((12, 12, 12)), at.swatch((48, 48, 48))
    grey, white = at.swatch((105, 105, 105)), at.swatch(LETTER)
    objs = []; xml = ''

    face = A.Obj('face', 'face.png')
    outline, uv = face_picture(its)
    A.flat(face, outline, 0.0, uv)
    objs.append(face)

    # warning light segments
    seg = next(it for it in its if it['name'] == 'Warning light segments')
    y0, y1 = seg['y'] - seg['w'] / 2, seg['y'] + seg['w'] / 2
    z0, z1 = seg['z'] - seg['h'] / 2, seg['z'] + seg['h'] / 2
    fr = A.Obj('wl-frame', 'panel.png')
    A.box(fr, y0, y1, z0, z1, 0.0, 0.004, black, black)
    m = 0.0018; cw = (y1 - y0 - 3 * m) / 2; ch = (z1 - z0 - 6 * m) / 5
    lamps = []
    for (c, r), legends in SEGMENTS.items():
        ya = y0 + m + c * (cw + m); za = z1 - m - (r + 1) * ch - r * m
        A.box(fr, ya, ya + cw, za, za + ch, 0.004, 0.008, frame, frame)
        if legends is None:
            A.flat(fr, [(ya + 0.0012, za + 0.0012), (ya + cw - 0.0012, za + 0.0012), (ya + cw - 0.0012, za + ch - 0.0012),
                        (ya + 0.0012, za + ch - 0.0012)], 0.0084, black)
            continue
        for j, text in enumerate(legends):
            zb = za + ch / 2 if j == 0 else za
            box = (ya + 0.0012, ya + cw - 0.0012, zb + 0.0008, zb + ch / 2 - 0.0008)
            quad = [(box[0], box[2]), (box[1], box[2]), (box[1], box[3]), (box[0], box[3])]
            name = 'wl-' + (prop_name(text) if text else 'blank-%d-%d' % (c, r))
            o = A.Obj(name, 'panel.png')
            A.flat(o, quad, 0.0084, at.picture(160, 84, legend_picture(text, False), *box))
            objs.append(o)
            if text:
                on = A.Obj(name + '-on', 'panel.png')
                A.flat(on, quad, 0.0086, at.picture(160, 84, legend_picture(text, True), *box))
                objs.append(on); lamps.append((name + '-on', prop_name(text)))
    objs.append(fr)
    for obj, p in lamps:
        xml += select_xml([obj], '<property>%swarning/%s</property>' % (PROP, p)) + \
               G.lamp_xml([obj], '%swarning/%s' % (PROP, p), (0.65, 0.12, 0.08)) + G.lit_xml([obj], G.LIGHTS_HDR + 'warn')

    # fire warning lights, TEST FIRE pushbuttons
    fires = sorted([it for it in its if it['name'].startswith('Fire warning light')], key=lambda it: it['y'])
    tests = sorted([it for it in its if it['name'].startswith('TEST FIRE')], key=lambda it: it['y'])
    for n, (it, tb) in enumerate(zip(fires, tests), 1):
        r = it['w'] / 2
        def cap(lit):
            def draw(d, w, h):
                d.ellipse([0, 0, w - 1, h - 1], fill=(220, 35, 25) if lit else (70, 14, 12))
                d.text((w / 2, h / 2), 'F', font=G.font(h * 0.62), fill=(255, 235, 220) if lit else (175, 150, 140), anchor='mm')
            return draw
        body = A.Obj('fire%d-body' % n, 'panel.png')
        A.cylinder(body, r + 0.0012, it['y'], it['z'], 0.0, 0.004, black, black, n=36)
        A.walls(body, A.circle(r, it['y'], it['z'], 36), 0.004, 0.009, frame)
        objs.append(body)
        for lit in (False, True):
            o = A.Obj('fire%d%s' % (n, '-on' if lit else ''), 'panel.png')
            A.flat(o, A.circle(r, it['y'], it['z'], 36), 0.009 + 0.0002 * lit,
                   at.picture(128, 128, cap(lit), it['y'] - r, it['y'] + r, it['z'] - r, it['z'] + r))
            objs.append(o)
        xml += select_xml(['fire%d-on' % n], '<property>%swarning/fire%d</property>' % (PROP, n)) + \
               G.lamp_xml(['fire%d-on' % n], '%swarning/fire%d' % (PROP, n), (0.7, 0.1, 0.06)) + G.lit_xml(['fire%d-on' % n], G.LIGHTS_HDR + 'warn')
        b = A.Obj('test-fire%d' % n, 'panel.png')
        A.cylinder(b, 0.0060, tb['y'], tb['z'], 0.0, 0.003, grey, grey, n=6)
        A.cylinder(b, 0.0038, tb['y'], tb['z'], 0.003, 0.008, black, frame, n=24)
        objs.append(b)
        xml += push_xml('test-fire%d' % n, '%stest-fire[%d]' % (PROP, n - 1), 'TEST FIRE, engine %d' % n)

    # pitot heat indicator: a magnetic indicator, black when off, white quarters when heating
    pit = next(it for it in its if it['name'] == 'Pitot heat indicator')
    r = 0.0055
    pb = A.Obj('pitot-body', 'panel.png'); A.cylinder(pb, r + 0.0012, pit['y'], pit['z'], 0.0, 0.004, frame, black, n=24)
    objs.append(pb)
    def eye(on):
        def draw(d, w, h):
            d.ellipse([0, 0, w - 1, h - 1], fill=(10, 10, 10))
            if on:
                for a in (45, 225):
                    d.pieslice([0, 0, w - 1, h - 1], a - 45, a + 45, fill=(235, 235, 235))
        return draw
    for on in (False, True):
        o = A.Obj('pitot%s' % ('-on' if on else ''), 'panel.png')
        A.flat(o, A.circle(r, pit['y'], pit['z'], 24), 0.0042 + 0.0002 * on,
               at.picture(64, 64, eye(on), pit['y'] - r, pit['y'] + r, pit['z'] - r, pit['z'] + r))
        objs.append(o)
    xml += select_xml(['pitot-on'], '<property>/controls/anti-ice/pitot-heat</property>')

    # instrument light rheostat and lamp test pushbutton
    rh = next(it for it in its if it['name'] == 'Instrument light rheostat')
    kn = A.Obj('instr-light', 'panel.png')
    kz = rh['z'] + 0.0020
    A.cylinder(kn, 0.0068, rh['y'], kz, 0.0, 0.010, frame, black, n=8)
    A.box(kn, rh['y'] - 0.0005, rh['y'] + 0.0005, kz + 0.002, kz + 0.0065, 0.010, 0.0103, white, white)
    objs.append(kn)
    xml += rotate_xml('instr-light', G.LIGHTING, 'factor', 270, (rh['y'], kz), offset=-135) + \
           knob_xml('instr-light', G.LIGHTING, 0.1, 0, 1, 'INSTR. LIGHT')
    lt = next(it for it in its if it['name'] == 'Annunciator lamp test pushbutton')
    bt = A.Obj('lamp-test', 'panel.png')
    A.cylinder(bt, 0.0080, lt['y'], lt['z'] + 0.0010, 0.0, 0.006, frame, black, n=28)
    objs.append(bt)
    xml += push_xml('lamp-test', PROP + 'lamp-test', 'Warning lights TEST LAMP')

    A.write(os.path.join(PANEL, 'panel.ac'), objs)
    at.im.save(os.path.join(PANEL, 'panel.png'), optimize=True)

    # panel.xml: the panel, its animations, the instruments on their places
    out = ['<?xml version="1.0" encoding="ISO-8859-1"?>', '',
           '<!-- the Bo 105 instrument panel: the flight manual\'s fig. 7-3 instruments laid out as the Flying Bulls\' Bo 105 C\'s on the '
           'console; written by fdm/cockpit/panel.py -->', '',
           '<PropertyList>', '', '  <path>panel.ac</path>']
    out.append('\n  <animation>\n    <type>noshadow</type>\n%s  </animation>' % G.objects_xml([o.name for o in objs]))
    out.append(xml.rstrip('\n'))
    out.append(matte_xml([o.name for o in objs if o.name in MATTE and o.name != 'face']).rstrip('\n'))
    out.append(G.matte_xml(['face']).rstrip('\n'))         # the plate in the glareshield's shade as the instruments' cases
    for it in its:
        if it['name'] in INSTRUMENTS:
            folder, f = INSTRUMENTS[it['name']]
            out.append('\n  <model>\n    <name>%s</name>\n    <path>Aircraft/MBB_Bo_105/Models/Panel/Instruments/%s/%s</path>\n'
                       '    <offsets>\n      <x-m> 0.0 </x-m>\n      <y-m> %.4f </y-m>\n      <z-m> %.4f </z-m>\n    </offsets>\n'
                       '  </model>' % (it['name'], folder, f, it['y'], it['z']))
    out += ['', '</PropertyList>', '']
    open(os.path.join(PANEL, 'panel.xml'), 'w').write('\n'.join(out))


def matte_xml(objs, roughness=0.85):
    """HDR pipeline: painted, not glossy (model-default's roughness is 0.5)"""
    return ('\n  <effect>\n    <inherits-from>Effects/model-default</inherits-from>\n    <parameters>\n'
            '      <pbr-roughness type="float"> %g </pbr-roughness>\n    </parameters>\n%s  </effect>\n' % (roughness, G.objects_xml(objs)))


def select_xml(objs, cond):
    return ('\n  <animation>\n    <type>select</type>\n%s    <condition>\n      %s\n    </condition>\n  </animation>\n'
            % (G.objects_xml(objs), cond))


def push_xml(obj, prop, tooltip):
    return G.push_pick(obj, prop, tooltip)


def knob_xml(obj, prop, step, lo, hi, tooltip):
    return G.knob_pick(obj, prop, step, lo, hi, tooltip)


def rotate_xml(obj, prop, kind, factor, centre, offset=None):
    return G.rot(obj, prop, factor=factor, centre=centre, offset=offset)


def preview(path):
    """a flat picture of the panel for checking it outside FlightGear: the face plate, the instruments' cases, their
    dials (gauges.py's textures) and needles at sample readings (cruise at 100 kt), the lights off"""
    from PIL import ImageDraw as D
    its = placed()
    face_picture(its)
    im = Image.open(os.path.join(PANEL, 'face.png')).convert('RGB'); d = D.Draw(im)
    T = [to_face(y, z) for y, z in CONSOLE]
    fy0, fz0 = min(p[0] for p in T), min(p[1] for p in T)
    sx = 2048 / max(max(p[0] for p in T) - fy0, max(p[1] for p in T) - fz0)
    P = lambda y, z: ((y - fy0) * sx, 2048 - (z - fz0) * sx)
    R3, R2, R2S = G.DIAL3, G.DIAL2, G.DIAL2S
    ols = lambda v: -55 + 110 * v
    # name -> (folder, dial radius, case w, h, dial centre, needles: (pivot y, z (x R), angle, length (x R), width m))
    S = {'Airspeed indicator': ('asi', R3, G.CASE3, G.CASE3, 0, [(0, 0, G.interp(G.ASI_TABLE, 100), 0.88, 0.0022)]),
         'Altimeter': ('alt', R3, G.CASE3, G.CASE3, 0, [(0, 0, 5.4, 0.95, 0.0009), (0, 0, 54, 0.55, 0.0040), (0, 0, 180, 0.90, 0.0022)]),
         'Vertical speed indicator': ('vsi', R3, G.CASE3, G.CASE3, 0, [(0, 0, -90 + 29, 0.90, 0.0026)]),
         'Triple tachometer': ('tach', R3, G.CASE3, G.CASE3, 0, [(0, 0, G.tach_a(100), 0.45, 0.0042), (0, 0, G.tach_a(101), 0.90, 0.0026),
                                                                  (0, 0, G.tach_a(100.5), 0.90, 0.0026)]),
         'Dual torque indicator': ('tq', R2, G.CASE2, G.CASE2, 0, [(0, 0, G.tq_a(55), 0.88, 0.0020), (0, 0, G.tq_a(57), 0.88, 0.0020)]),
         'TOT indicator, engine 1': ('tot', R2S, G.CASE2S, G.CASE2S, 0, [(0, 0, G.tot_a(640), 0.88, 0.0020)]),
         'TOT indicator, engine 2': ('tot', R2S, G.CASE2S, G.CASE2S, 0, [(0, 0, G.tot_a(650), 0.88, 0.0020)]),
         'N1 indicator, engine 1': ('n1', R2S, G.CASE2S, G.CASE2S, 0, [(-0.40, 0.30, 72, 0.22, 0.0010), (0, 0, G.n1_a(92), 0.88, 0.0020)]),
         'N1 indicator, engine 2': ('n1', R2S, G.CASE2S, G.CASE2S, 0, [(-0.40, 0.30, 108, 0.22, 0.0010), (0, 0, G.n1_a(93), 0.88, 0.0020)]),
         'Fuel quantity indicator': ('fq', R2S, G.CASE2S, G.CASE2S, 0, [(0.28, -0.05, -(140 - 100 * 30 / 46), 0.85, 0.0016),
                                                                     (-0.28, -0.05, 140 - 100 * 7.4 / 8, 0.85, 0.0016)]),
         'Fuel pressure indicator': ('fp', R2S, G.CASE2S, G.CASE2S, 0, [(0.28, -0.05, -(140 - 100 * 1.1 / 2.5), 0.85, 0.0016),
                                                                     (-0.28, -0.05, 140 - 100 * 1.1 / 2.5, 0.85, 0.0016)]),
         'Triple oil temperature indicator': ('oilt', R3, G.CASE3, G.CASE3, 0, [(0, 0.18, ols(60 / 130), 0.40, 0.0018),
                                              (-0.47, -0.32, ols(85 / 130), 0.40, 0.0018), (0.47, -0.32, ols(85 / 130), 0.40, 0.0018)]),
         'Triple oil pressure indicator': ('oilp', R3, G.CASE3, G.CASE3, 0, [(0, 0.18, ols(1.6 / 2.5), 0.40, 0.0018),
                                           (-0.47, -0.32, ols(8.3 / 11), 0.40, 0.0018), (0.47, -0.32, ols(8.3 / 11), 0.40, 0.0018)]),
         'Mast moment indicator': ('mmi', 0.025, 0.081, 0.078, 0.006, [(0, 0, G.mmi_a(15), 0.88, 0.0018)]),
         'Attitude indicator': ('ai', R3, G.CASE3, G.CASE3, 0, []),
         'Horizontal situation indicator': ('hsi', R3, G.CASE3, G.CASE3, 0, [(0, 0, 25, 0.62, 0.0030)]),
         'Clock': ('clock', R2S, G.CASE2S, G.CASE2S, 0, [(0, 0, 305, 0.55, 0.0030), (0, 0, 60, 0.85, 0.0022), (0, -0.42, 0, 0.21, 0.0008)])}
    for it in its:
        if it['name'] not in S:
            continue
        folder, R, cw, ch, dz, needles = S[it['name']]
        y, z = it['y'], it['z']
        d.rounded_rectangle([P(y - cw / 2, z + ch / 2), P(y + cw / 2, z - ch / 2)], radius=0.005 * sx, fill=(24, 24, 24), outline=(55, 55, 55), width=2)
        cy, cz = y, z + dz
        n = int(2 * R * sx)
        dial = Image.open(os.path.join(G.INSTR, folder, folder + '.png')).crop((0, 0, 1024, 1024)).resize((n, n), Image.LANCZOS)
        mask = Image.new('L', (n, n), 0); D.Draw(mask).ellipse([0, 0, n - 1, n - 1], fill=255)
        x0, y0 = P(cy - R, cz + R)
        im.paste(dial, (int(x0), int(y0)), mask)
        if folder == 'ai':
            r = 0.80 * R * sx; qx, qy = P(cy, cz)
            d.pieslice([qx - r, qy - r, qx + r, qy + r], 180, 360, fill=(60, 130, 205))
            d.pieslice([qx - r, qy - r, qx + r, qy + r], 0, 180, fill=(125, 78, 38))
            d.line([qx - r, qy, qx + r, qy], fill=(240, 240, 240), width=3)
            d.line([qx - 0.020 * sx, qy, qx - 0.007 * sx, qy], fill=(240, 140, 30), width=5)
            d.line([qx + 0.007 * sx, qy, qx + 0.020 * sx, qy], fill=(240, 140, 30), width=5)
        if folder == 'hsi':
            rc = 0.86 * R; m = int(2 * rc * sx)
            card = Image.open(os.path.join(G.INSTR, 'hsi', 'hsi.png')).crop((1080, 0, 1980, 900)).resize((m, m), Image.LANCZOS)
            mk = Image.new('L', (m, m), 0); D.Draw(mk).ellipse([0, 0, m - 1, m - 1], fill=255)
            xx, yy = P(cy - rc, cz + rc); im.paste(card, (int(xx), int(yy)), mk)
        for py, pz, a, ln, w in needles:
            ax, az = math.sin(math.radians(a)), math.cos(math.radians(a))
            ox, oz = cy + py * R, cz + pz * R
            pts = [(ox + ax * R * ln - az * 0, oz + az * R * ln), (ox - az * w / 2, oz + ax * w / 2), (ox - ax * 0.006 - az * w / 2, oz - az * 0.006 + ax * w / 2),
                   (ox - ax * 0.006 + az * w / 2, oz - az * 0.006 - ax * w / 2), (ox + az * w / 2, oz - ax * w / 2)]
            d.polygon([P(*p) for p in pts], fill=G.WHITE)
            r = max(w, 0.0015)
            d.ellipse([P(ox - r, oz + r), P(ox + r, oz - r)], fill=(10, 10, 10))
        if folder == 'mmi':
            for yy, t in ((-0.025, 'CAL'), (-0.0085, 'TEST'), (0.0215, 'LIMIT')):
                d.text(P(y + yy, z - 0.0215), t, font=G.font(0.0035 * sx), fill=LETTER, anchor='mm')
            d.text(P(y + 0.0068, z - 0.031), 'MM', font=G.font(0.0050 * sx), fill=LETTER, anchor='mm')
            for yy, rgb in ((-0.025, (90, 90, 90)), (-0.0085, (70, 70, 70)), (0.0215, (95, 12, 10))):
                r = 0.0022 if yy < -0.02 else 0.0040
                d.ellipse([P(y + yy - r, z - 0.031 + r), P(y + yy + r, z - 0.031 - r)], fill=rgb)
    for it in its:
        if it['name'] in ('Radio control head', 'GPS'):
            f = 'radio' if it['name'].startswith('Radio') else 'gps'
            src = Image.open(os.path.join(G.INSTR, f, f + '.png'))
            hpx = int(1024 * it['h'] / it['w'])
            pic = src.crop((0, 0, 1024, hpx)).resize((int(it['w'] * sx), int(it['h'] * sx)), Image.LANCZOS)
            im.paste(pic, tuple(int(c) for c in P(it['y'] - it['w'] / 2, it['z'] + it['h'] / 2)))
    seg = next(it for it in its if it['name'] == 'Warning light segments')
    y0, y1, z0, z1 = seg['y'] - seg['w'] / 2, seg['y'] + seg['w'] / 2, seg['z'] - seg['h'] / 2, seg['z'] + seg['h'] / 2
    d.rectangle([P(y0, z1), P(y1, z0)], fill=(12, 12, 12))
    m = 0.0018; cw = (y1 - y0 - 3 * m) / 2; ch = (z1 - z0 - 6 * m) / 5
    for (c, r), legends in SEGMENTS.items():
        ya = y0 + m + c * (cw + m); za = z1 - m - (r + 1) * ch - r * m
        d.rectangle([P(ya, za + ch), P(ya + cw, za)], fill=(48, 48, 48))
        for j, text in enumerate(legends or ()):
            zb = za + ch / 2 if j == 0 else za
            box = [P(ya + 0.0012, zb + ch / 2 - 0.0008), P(ya + cw - 0.0012, zb + 0.0008)]
            wpx, hpx = int(box[1][0] - box[0][0]), int(box[1][1] - box[0][1])
            sub = Image.new('RGB', (160, 84)); legend_picture(text, False)(D.Draw(sub), 160, 84)
            im.paste(sub.resize((wpx, hpx), Image.LANCZOS), (int(box[0][0]), int(box[0][1])))
    for it in its:
        if it['name'].startswith('Fire warning light'):
            r = it['w'] / 2
            d.ellipse([P(it['y'] - r, it['z'] + r), P(it['y'] + r, it['z'] - r)], fill=(70, 14, 12), outline=(48, 48, 48), width=4)
            d.text(P(it['y'], it['z']), 'F', font=G.font(r * 1.25 * sx), fill=(175, 150, 140), anchor='mm')
        elif it['name'].startswith('TEST FIRE') or it['name'] == 'Pitot heat indicator':
            r = 0.0055
            d.ellipse([P(it['y'] - r, it['z'] + r), P(it['y'] + r, it['z'] - r)], fill=(14, 14, 14), outline=(90, 90, 90), width=3)
        elif it['name'] in ('Instrument light rheostat', 'Annunciator lamp test pushbutton'):
            r = 0.0070; zc = it['z'] + 0.0015
            d.ellipse([P(it['y'] - r, zc + r), P(it['y'] + r, zc - r)], fill=(40, 40, 40), outline=(12, 12, 12), width=3)
    im.save(path, optimize=True)


if __name__ == '__main__':
    build()
    preview(os.path.join(L.HERE, 'panel-preview.png'))
    print('wrote', os.path.normpath(PANEL), 'and panel-preview.png')
