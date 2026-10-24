"""The flight instruments and avionics of the Bo 105's pilot section, after the Flying Bulls' Bo 105 C (D-HSDM): its
panel is the 3-D model's T-shaped console, the manual's engine instruments down its central column and in front of
the pilot an attitude indicator and an HSI beside the altimeter and vertical speed indicator, a radio control head
and a GPS below them (panel.py places them). Built as gauges.py's, HDR-ready:

- attitude indicator (ai): an electric gyro (it needs no vacuum; the aircraft has no electrical system), its sky and
  ground on a disc turned by the roll and its picture moved by the pitch (8 mm per 10 deg), under a fixed roll scale
  (10, 20, 30, 45, 60 deg) and aircraft symbol, its own roll pointer;
- horizontal situation indicator (hsi): a compass card on the magnetic heading, the course pointer and deviation bar
  of NAV 1 (/instrumentation/nav[0]: selected radial, needle deflection), lubber line and aircraft symbol;
- radio control head (radio): COMM and NAV 1, active and standby frequencies in digit cells
  (/instrumentation/comm[0], nav[0]);
- GPS (gps): its bezel and keys, the screen dark (nothing drives a map).

    python3 avionics.py
"""
import math, os
from PIL import Image, ImageDraw

import acmesh as A
import gauges as G
from gauges import Gauge, Dial, font, WHITE, BLACK, rot, CASE3, WIN3, DIAL3

SKY, GROUND, LINE = (60, 130, 205), (125, 78, 38), (240, 240, 240)
ORANGE = (240, 140, 30)


def translate_xml(obj, prop, factor, axis, lo=None, hi=None):
    lim = '    <min-m> %g </min-m>\n    <max-m> %g </max-m>\n' % (lo, hi) if lo is not None else ''
    return ('\n  <animation>\n    <type>translate</type>\n    <object-name>%s</object-name>\n    <property>%s</property>\n'
            '    <factor> %g </factor>\n%s    <axis>\n      <x> %g </x>\n      <y> %g </y>\n      <z> %g </z>\n    </axis>\n'
            '  </animation>\n' % ((obj, prop, factor, lim) + tuple(axis)))


def textranslate_xml(obj, prop, factor, lo, hi):
    return ('\n  <animation>\n    <type>textranslate</type>\n    <object-name>%s</object-name>\n    <property>%s</property>\n'
            '    <factor> %g </factor>\n    <min> %g </min>\n    <max> %g </max>\n    <axis>\n      <x> 0 </x>\n      <y> 1 </y>\n'
            '      <z> 0 </z>\n    </axis>\n  </animation>\n' % (obj, prop, factor, lo, hi))


# ---------------------------------------------------------------- attitude indicator

PITCH_SPAN = 270.0                      # deg of pitch the sky/ground strip covers, horizon in the middle
PITCH_M = 0.0008                        # m per deg of pitch on the dial


def ai():
    R = DIAL3; rin = R * 0.80
    g = Gauge('ai', CASE3, CASE3, WIN3, R, comment='attitude indicator (electric), after the Flying Bulls\' Bo 105 C')
    g.objs.pop('dial', None)
    # the sky and ground: a strip of its own (ai-sphere.png), a disc's window onto it, moved by the pitch
    Wp, Hp = 256, 2048
    strip = Image.new('RGB', (Wp, Hp), SKY); d = ImageDraw.Draw(strip)
    d.rectangle([0, Hp // 2, Wp, Hp], fill=GROUND)
    k = Hp / PITCH_SPAN                                                     # px per deg
    win = 2 * rin / PITCH_M                                                 # deg across the disc
    pxw = Wp / win                                                          # px per deg across (square picture)
    d.line([(0, Hp / 2), (Wp, Hp / 2)], fill=LINE, width=4)
    for p in range(-90, 91, 5):
        if p == 0:
            continue
        y = Hp / 2 - p * k; half = (10 if p % 10 == 0 else 5) * pxw
        d.line([(Wp / 2 - half, y), (Wp / 2 + half, y)], fill=LINE, width=3)
        if p % 10 == 0:
            for sx in (-1, 1):
                d.text((Wp / 2 + sx * (half + 4 * pxw), y), str(abs(p)), font=font(3.2 * pxw), fill=LINE, anchor='mm')
    folder = os.path.join(G.INSTR, 'ai'); os.makedirs(folder, exist_ok=True)
    strip.save(os.path.join(folder, 'ai-sphere.png'))
    card = A.Obj('sphere', 'ai-sphere.png')
    v0 = 0.5 - win / 2 / PITCH_SPAN; v1 = 0.5 + win / 2 / PITCH_SPAN
    A.flat(card, A.circle(rin + 0.0006, n=64), g.dd, A.planar(-rin, rin, -rin, rin, 0.0, 1.0, v0, v1))
    g.objs['sphere'] = card; g.lit.append('sphere')
    # the fixed roll scale round it
    D = Dial(1024, R, bg=(12, 12, 12))
    D.disc(rin, (0, 0, 0))
    for a, big in ((0, True), (10, False), (20, False), (30, True), (45, False), (60, True)):
        for s in ((1, -1) if a else (1,)):
            D.tick(s * a, R * 0.97, R * (0.84 if big else 0.89), 0.0010 if big else 0.0007)
    D.d.polygon([D.p(R * 0.84, -4), D.p(R * 0.84, 4), D.p(R * 0.97, 0)], fill=WHITE)
    img = D.done()
    g.t.im.paste(img, (0, 0))
    ring = g.obj('scale')
    A.bezel(ring, A.circle(R, n=72), A.circle(rin, n=72), g.dd + 0.0012, g.t.area(0, 0, img.width, img.height, -R, R, -R, R))
    g.lit.append('scale')
    # the roll pointer, turning with the sphere, and the aircraft symbol
    rp = g.obj('rollptr')
    A.flat(rp, [(-0.0022, rin - 0.0045), (0.0022, rin - 0.0045), (0.0, rin - 0.0005)], g.dd + 0.0006, g.white)
    g.lit.append('rollptr')
    sym = g.obj('symbol'); o = g.t.swatch(ORANGE)
    for sx in (-1, 1):
        A.box(sym, sx * 0.007, sx * 0.020, -0.0008, 0.0008, g.dd + 0.0020, g.dd + 0.0030, o, o)
        A.box(sym, sx * 0.0070 - 0.0007, sx * 0.0070 + 0.0007, -0.0035, 0.0, g.dd + 0.0020, g.dd + 0.0030, o, o)
    A.cylinder(sym, 0.0012, 0.0, 0.0, g.dd + 0.0020, g.dd + 0.0032, o, o, n=12)
    g.lit.append('symbol')
    g.glass()
    roll = '/orientation/roll-deg'
    g.xml += rot('sphere', roll, factor=-1) + rot('rollptr', roll, factor=-1)
    g.xml += textranslate_xml('sphere', '/orientation/pitch-deg', 1.0 / PITCH_SPAN, -80, 80)
    g.write()


# ---------------------------------------------------------------- HSI

def hsi():
    R = DIAL3; rc = R * 0.86
    g = Gauge('hsi', CASE3, CASE3, WIN3, R, comment='horizontal situation indicator, after the Flying Bulls\' Bo 105 C')
    # the fixed ring: 45 deg marks, the lubber line
    D = Dial(1024, R, bg=(12, 12, 12))
    for a in range(0, 360, 45):
        D.tick(a, R * 0.99, R * 0.92, 0.0010 if a % 90 else 0.0014, ORANGE if a == 0 else WHITE)
    g.dial(D.done())
    # the compass card (its own picture in the texture's right half)
    C = Dial(1024, rc, bg=(18, 18, 18))
    for h in range(0, 360, 5):
        big = h % 10 == 0
        C.tick(h, rc * 0.99, rc * (0.86 if big else 0.92), 0.0008 if big else 0.0005)
    names = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}
    for h in range(0, 360, 30):
        C.text_at(rc * 0.72, h, names.get(h, str(h // 10)), 0.0048)
    cimg = C.done().resize((900, 900), Image.LANCZOS)
    g.t.im.paste(cimg, (1080, 0))
    A.flat(g.obj('card'), A.circle(rc, n=72), g.dd + 0.0006, g.t.area(1080, 0, 1980, 900, -rc, rc, -rc, rc))
    g.lit.append('card')
    # course pointer (with its tail) and the deviation bar, NAV 1; the aircraft symbol
    mag = g.t.swatch((220, 60, 200))
    cp = g.obj('course')
    A.flat(cp, [(-0.0010, rc * 0.30), (0.0010, rc * 0.30), (0.0010, rc * 0.62), (0.0028, rc * 0.62), (0.0, rc * 0.80),
                (-0.0028, rc * 0.62), (-0.0010, rc * 0.62)], g.dd + 0.0012, mag)
    A.flat(cp, [(-0.0010, -rc * 0.80), (0.0010, -rc * 0.80), (0.0010, -rc * 0.30), (-0.0010, -rc * 0.30)], g.dd + 0.0012, mag)
    dev = g.obj('cdi')
    A.flat(dev, [(-0.0010, -rc * 0.28), (0.0010, -rc * 0.28), (0.0010, rc * 0.28), (-0.0010, rc * 0.28)], g.dd + 0.0014, mag)
    for o in ('course', 'cdi'):
        g.lit.append(o)
    sym = g.obj('symbol'); w = g.t.swatch(ORANGE)
    A.box(sym, -0.0007, 0.0007, -0.0075, 0.0075, g.dd + 0.0020, g.dd + 0.0028, w, w)
    A.box(sym, -0.0075, 0.0075, 0.0010, 0.0024, g.dd + 0.0020, g.dd + 0.0028, w, w)
    A.box(sym, -0.0030, 0.0030, -0.0075, -0.0062, g.dd + 0.0020, g.dd + 0.0028, w, w)
    g.lit.append('symbol')
    g.glass()
    hdg = '/orientation/heading-magnetic-deg'
    g.xml += rot('card', hdg, factor=-1) + rot('course', hdg, factor=-1) + rot('cdi', hdg, factor=-1)
    g.xml += rot('course', '/instrumentation/nav[0]/radials/selected-deg', factor=1) + \
             rot('cdi', '/instrumentation/nav[0]/radials/selected-deg', factor=1)
    g.xml += translate_xml('cdi', '/instrumentation/nav[0]/heading-needle-deflection', 0.0011, (0, 1, 0), -0.011, 0.011)
    g.write()


# ---------------------------------------------------------------- radio control head and GPS

class Face:
    """a rectangular unit: a box proud of the panel, its face a picture in its own texture"""
    def __init__(s, name, w, h, depth, draw, px=1024):
        s.name, s.w, s.h, s.depth = name, w, h, depth
        s.t = G.Tex(px, px)
        s.objs = {}; s.lit = []; s.xml = ''
        hpx = int(px * h / w)
        sub = Image.new('RGB', (px, hpx), (24, 24, 24)); draw(ImageDraw.Draw(sub), px, hpx)
        s.t.im.paste(sub, (0, 0))
        dark = s.t.swatch((26, 26, 26))
        box = s.obj('case')
        A.walls(box, [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)], 0.0, depth, dark)
        A.flat(s.obj('face'), [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)], depth,
               s.t.area(0, 0, px, hpx, -w / 2, w / 2, -h / 2, h / 2))
        s.px, s.hpx = px, hpx

    def obj(s, name, glass=False):
        if name not in s.objs:
            s.objs[name] = A.Obj(name, None if glass else s.name + '.png', A.GLASS if glass else 0)
        return s.objs[name]

    def at(s, y, z):
        """face metres -> its picture's pixels"""
        return (s.px * (y / s.w + 0.5), s.hpx * (0.5 - z / s.h))

    def write(s, comment):
        objs = list(s.objs.values())
        x = '<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- %s; written by fdm/cockpit/avionics.py -->\n\n' % comment
        x += '<PropertyList>\n\n  <path>%%s</path>\n\n  <animation>\n    <type>noshadow</type>\n%s  </animation>\n' % \
             G.objects_xml([o.name for o in objs])
        x += G.glass_xml([o.name for o in objs if o.mat == A.GLASS]) + G.matte_xml(['case', 'face']) + s.xml + '\n</PropertyList>\n'
        G.write_instrument(s.name, objs, x, s.t)


RADIO_W, RADIO_H = 0.159, 0.042


def radio():
    w, h = RADIO_W, RADIO_H
    wins = {'com-use': (-0.052, 0.006), 'com-stby': (-0.020, 0.006), 'nav-use': (0.022, 0.006), 'nav-stby': (0.054, 0.006)}
    ww, wh = 0.028, 0.011

    def draw(d, W, H):
        f = font(H * 0.13)
        p = lambda y, z: (W * (y / w + 0.5), H * (0.5 - z / h))
        for k, (y, z) in wins.items():
            d.rectangle([p(y - ww / 2 - 0.001, z + wh / 2 + 0.001), p(y + ww / 2 + 0.001, z - wh / 2 - 0.001)], fill=(4, 4, 4))
            d.text(p(y, z + wh / 2 + 0.004), {'com-use': 'COMM', 'com-stby': 'STBY', 'nav-use': 'NAV', 'nav-stby': 'STBY'}[k],
                   font=f, fill=WHITE, anchor='mm')
        for y in (-0.036, 0.038):
            d.rounded_rectangle([p(y - 0.005, -0.008), p(y + 0.005, -0.014)], radius=4, fill=(70, 70, 70))
            d.text(p(y, -0.0175), '<->', font=f, fill=WHITE, anchor='mm')
        for y in (-0.072, 0.072):
            d.ellipse([p(y - 0.0055, -0.004), p(y + 0.0055, -0.015)], fill=(50, 50, 50))
    u = Face('radio', w, h, 0.012, draw)
    seg = font(60)
    orange = u.t.swatch((255, 150, 60))
    for k, (y, z) in wins.items():
        prop = '/instrumentation/%s/frequencies/%s-mhz' % ('comm[0]' if k.startswith('com') else 'nav[0]',
                                                            'selected' if k.endswith('use') else 'standby')
        strip = G.digit_strip(os.path.join(G._mk('radio'), 'radio-%s.png' % k), 48, 72, (255, 150, 60), seg)
        cells = G.digit_cells(strip, k, 5, y - ww / 2, y + ww / 2, z - wh / 2, z + wh / 2, u.depth + 0.0004)
        for o in cells:
            u.objs[o.name] = o; u.lit.append(o.name)
        u.xml += G.digits_xml(k, 5, prop, unit=0.01)
        dp = y + ww / 2 - 2 * ww / 5                           # the decimal point, ahead of the last two digits
        A.flat(u.obj('dots'), [(dp - 0.0006, z - wh / 2 + 0.0006), (dp + 0.0006, z - wh / 2 + 0.0006),
                               (dp + 0.0006, z - wh / 2 + 0.0018), (dp - 0.0006, z - wh / 2 + 0.0018)], u.depth + 0.0008, orange)
    u.lit.append('dots')
    u.xml += G.lit_xml(u.lit, G.LIGHTS_HDR + 'displays', reflect=0.05)        # its display's own level (photocell, Nasal/panel.nas)
    u.write('radio control head (COMM and NAV 1), after the Flying Bulls\' Bo 105 C')


GPS_W, GPS_H = 0.159, 0.084


def gps():
    w, h = GPS_W, GPS_H
    sw, sh, sy = 0.098, 0.062, -0.012                     # the screen

    def draw(d, W, H):
        p = lambda y, z: (W * (y / w + 0.5), H * (0.5 - z / h))
        d.rectangle([p(sy - sw / 2 - 0.002, sh / 2 + 0.002), p(sy + sw / 2 + 0.002, -sh / 2 - 0.002)], fill=(10, 10, 10))
        f = font(H * 0.06)
        for k, lab in enumerate(('CDI', 'OBS', 'MSG', 'FPL', 'PROC')):
            z = 0.030 - k * 0.013
            d.rounded_rectangle([p(0.055, z + 0.004), p(0.074, z - 0.004)], radius=3, fill=(60, 60, 60))
            d.text(p(0.0645, z), lab, font=f, fill=WHITE, anchor='mm')
        for k in range(6):
            y = sy - sw / 2 + (k + 0.5) * sw / 6
            d.rounded_rectangle([p(y - 0.006, -0.034), p(y + 0.006, -0.039)], radius=3, fill=(60, 60, 60))
        d.ellipse([p(-0.075, 0.034), p(-0.067, 0.026)], fill=(50, 50, 50))
        d.text(p(-0.071, 0.020), 'PWR', font=f, fill=WHITE, anchor='mm')
    u = Face('gps', w, h, 0.014, draw)
    scr = u.obj('screen'); dark = u.t.swatch((3, 4, 5))
    A.flat(scr, [(sy - sw / 2, -sh / 2), (sy + sw / 2, -sh / 2), (sy + sw / 2, sh / 2), (sy - sw / 2, sh / 2)], u.depth + 0.0003, dark)
    A.flat(u.obj('glass', True), [(sy - sw / 2, -sh / 2), (sy + sw / 2, -sh / 2), (sy + sw / 2, sh / 2), (sy - sw / 2, sh / 2)],
           u.depth + 0.0008, A.const(0, 0))
    u.write('GPS (its screen dark), after the Flying Bulls\' Bo 105 C')


if __name__ == '__main__':
    for f in (ai, hsi, radio, gps):
        f()
        print('wrote', f.__name__)
