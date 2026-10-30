"""The Bo 105's round instruments, after the flight manual's figure 7-3 (the faces) and section 2.18 (their markings),
built new so that their faces are sharp: dials drawn at 1024 px across (about 14 px/mm on a 3 in. instrument),
supersampled; the needles flat meshes; counters are digit cells.

Sizes and places are panel_layout.py's; each instrument is built in its own frame (acmesh.py: d toward the crew, y
right, z up, metres), its origin on its bezel's centre and its back on the face plate (d = 0). The colours of the
markings are section 2.18's: green arcs the normal ranges, yellow arcs caution, red radials limits, red dots the
transient limits, the blue radial the power-off never-exceed speed.

- airspeed indicator (asi): 0-150 kt;
- altimeter (alt): three pointers, 100, 1000 and 10,000 ft, a hPa window and its knob;
- vertical speed indicator (vsi): 0-6000 ft/min up and down;
- triple tachometer (tach): both engines' N2 (long pointers 1, 2) and the rotor's speed (short pointer R) in %, the
  engines' markings outside and the rotor's (CBS) inside;
- dual torque indicator (tq): % of each engine;
- turbine outlet temperature (tot1, tot2), gas producer speed with its vernier (n1-1, n1-2);
- dual fuel quantity (fq: main tank M, supply tank S, kg x 10) and dual fuel pressure (fp: kp/cm2), each pointer
  sweeping its own side;
- triple oil temperature (oilt) and pressure (oilp): engines 1 and 2 below, main transmission (R) above;
- mast moment indicator (mmi): % of the hub moment limit (bo105data.MM_LIMIT_NM), its TEST button and LIMIT light;
- clock: hours, minutes, sweep seconds, a minute counter at 6 o'clock;
- standby compass (compass): its card behind a round window, read against a lubber line.

    python3 gauges.py
"""
import math, os, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import acmesh as A

HERE = os.path.dirname(os.path.abspath(__file__))
INSTR = os.path.join(HERE, '..', '..', 'MBB_Bo_105', 'Models', 'Panel', 'Instruments')
SS = 2                                          # supersampling of the dials
WHITE, BLACK = (236, 236, 236), (14, 14, 14)
GREEN, YELLOW, RED, BLUE = (40, 190, 70), (240, 200, 30), (225, 35, 30), (60, 120, 235)
# Instrument lighting: NVG compatible (flight manual supplement 10-52, NVG Compatible Instrument Lighting), NVIS Green A
# as the UH-60 model's, on the INSTR LIGHT switch and rheostat [FM 7.12.3]; Nasal/panel.nas sets the circuit's level
# (0-1) in LIGHTS + 'instruments'. Each instrument has a lighted bezel: in the HDR pipeline spot lamps round its dial
# (bezel_lamps, after the UH-60 model's), its markings paint; elsewhere the material animation's emission. The standby
# compass is lit from within (its card emits). The HDR pipeline leaves out the material animation's emission: there
# lamps emit through Models/Effects/Lit (as the UH-60 model's), at a level in cd/m2 that Nasal/panel.nas sets in
# LIGHTS_HDR: a warning lamp LAMP_BRT (1030 cd/m2, bright enough by day) while on, or dimmed. The pipeline's unit of
# radiance is 635 cd/m2 (HDR_CD, Shaders/HDR/lit.frag). Glass: Models/Effects/Glass/instrument-glass.
LIGHTS = '/instrumentation/bo105/lights/'
LIGHTING = '/controls/lighting/instruments-norm'        # the INSTR LIGHT rheostat on the panel
NVG_GREEN = (0.10, 1.00, 0.45)                   # NVIS Green A (MIL-L-85762A: u' 0.088, v' 0.543), sRGB about (0, 255, 110)
LAMP = NVG_GREEN                                 # the instrument lights' colour
LIGHT_LUX = 6.0                                  # HDR: the mean illuminance of an instrument's face at full INSTR LIGHT, lux
LIGHTS_HDR = LIGHTS + 'hdr/'
LAMP_BRT = 1030.0
HDR_CD = 635.0
EFFECTS = 'Aircraft/MBB_Bo_105/Models/Effects/'
CASE3, WIN3, DIAL3 = 0.0825, 0.0372, 0.0365     # 3ATI: case, window and dial radii
CASE2, WIN2, DIAL2 = 0.0600, 0.0262, 0.0256     # 2ATI (2 1/4 in)
CASE2S, WIN2S, DIAL2S = 0.0540, 0.0240, 0.0235  # 2 in.: the figure's clock, TOT, N1 and fuel gauges (45-51 mm bezels)


def font(size, narrow=False):
    """Arial Bold (or Arial Narrow Bold), found by its file name among the system's fonts."""
    for name in (('Arial Narrow Bold.ttf', 'ARIALNB.TTF') if narrow else ('Arial Bold.ttf', 'arialbd.ttf')):
        try:
            return ImageFont.truetype(name, max(1, int(size)))
        except OSError:
            pass
    sys.exit('Arial%s Bold is not among the system fonts' % (' Narrow' if narrow else ''))


def interp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


class Tex:
    """A texture: drawing areas mapped onto the instrument, and plain colours (swatches) along its bottom row."""
    def __init__(s, w, h, bg=(0, 0, 0)):
        s.im = Image.new('RGB', (w, h), bg); s.d = ImageDraw.Draw(s.im); s.n = 0

    def swatch(s, rgb):
        x = s.im.width - 16 * (s.n + 1); y = s.im.height - 16; s.n += 1
        s.d.rectangle([x, y, x + 15, y + 15], fill=rgb)
        return A.const((x + 8) / s.im.width, 1 - (y + 8) / s.im.height)

    def area(s, px0, py0, px1, py1, y0, y1, z0, z1):
        """uv for the rectangle y0..y1, z0..z1 (m) drawn in the pixels px0..px1, py0..py1 (y down)"""
        W, H = s.im.size
        return A.planar(y0, y1, z0, z1, px0 / W, px1 / W, 1 - py1 / H, 1 - py0 / H)


class Dial:
    """A dial drawn supersampled: positions in metres from its centre (or from a sub-dial's pivot c), angles clockwise
    from 12 o'clock."""
    def __init__(s, px, radius, bg=(12, 12, 12)):
        s.px, s.R = px, radius
        s.N = px * SS
        s.im = Image.new('RGB', (s.N, s.N), (0, 0, 0))
        s.d = ImageDraw.Draw(s.im)
        s.k = s.N / (2 * radius)
        s.d.ellipse([0, 0, s.N - 1, s.N - 1], fill=bg)

    def p(s, r, a, c=(0.0, 0.0)):
        a = math.radians(a)
        return (s.N / 2 + (c[0] + r * math.sin(a)) * s.k, s.N / 2 - (c[1] + r * math.cos(a)) * s.k)

    def xy(s, y, z):
        return (s.N / 2 + y * s.k, s.N / 2 - z * s.k)

    def w(s, m):
        return max(1, int(round(m * s.k)))

    def tick(s, a, r0, r1, w, fill=WHITE, c=(0.0, 0.0)):
        s.d.line([s.p(r0, a, c), s.p(r1, a, c)], fill=fill, width=s.w(w))

    def arc(s, a0, a1, r0, r1, fill, c=(0.0, 0.0)):
        """a coloured band from angle a0 to a1 between radii r0 and r1"""
        n = max(4, int(abs(a1 - a0) / 1.5))
        pts = [s.p(r1, a0 + (a1 - a0) * k / n, c) for k in range(n + 1)] + \
              [s.p(r0, a1 - (a1 - a0) * k / n, c) for k in range(n + 1)]
        s.d.polygon(pts, fill=fill)

    def dot(s, a, r, rad, fill=RED, c=(0.0, 0.0)):
        x, y = s.p(r, a, c); q = rad * s.k
        s.d.ellipse([x - q, y - q, x + q, y + q], fill=fill)

    def circle(s, r, w, fill=WHITE, c=(0.0, 0.0)):
        x, y = s.xy(*c); q = r * s.k
        s.d.ellipse([x - q, y - q, x + q, y + q], outline=fill, width=s.w(w))

    def disc(s, r, fill, c=(0.0, 0.0)):
        x, y = s.xy(*c); q = r * s.k
        s.d.ellipse([x - q, y - q, x + q, y + q], fill=fill)

    def text(s, y, z, txt, size, fill=WHITE, narrow=False, anchor='mm', angle=0):
        if not angle:
            s.d.text(s.xy(y, z), txt, font=font(size * s.k, narrow), fill=fill, anchor=anchor)
            return
        f = font(size * s.k, narrow)                     # turned text: drawn upright on its own, rotated, pasted
        x0, y0, x1, y1 = s.d.textbbox((0, 0), txt, font=f)
        im = Image.new('L', (x1 - x0 + 8, y1 - y0 + 8), 0)
        ImageDraw.Draw(im).text((4 - x0, 4 - y0), txt, font=f, fill=255)
        im = im.rotate(angle, expand=True, resample=Image.BICUBIC)
        cx, cy = s.xy(y, z)
        s.im.paste(Image.new('RGB', im.size, fill), (int(cx - im.width / 2), int(cy - im.height / 2)), im)

    def text_at(s, r, a, txt, size, fill=WHITE, c=(0.0, 0.0), narrow=False):
        x, y = s.p(r, a, c)
        s.d.text((x, y), txt, font=font(size * s.k, narrow), fill=fill, anchor='mm')

    def done(s):
        return s.im.resize((s.px, s.px), Image.LANCZOS)


def rot(obj, prop, table=None, factor=None, centre=(0.0, 0.0), offset=None):
    """rotate `obj` clockwise (as the crew sees it) about the pivot centre (y, z): by a table (value, deg) or factor"""
    out = '\n  <animation>\n    <type>rotate</type>\n    <object-name>%s</object-name>\n    <property>%s</property>\n' % (obj, prop)
    if table:
        out += '    <interpolation>\n' + ''.join('      <entry><ind> %g </ind><dep> %g </dep></entry>\n' % e for e in table) + \
               '    </interpolation>\n'
    else:
        out += '    <factor> %g </factor>\n' % factor
        if offset is not None:
            out += '    <offset-deg> %g </offset-deg>\n' % offset
    return out + ('    <center>\n      <x-m> 0 </x-m>\n      <y-m> %.5f </y-m>\n      <z-m> %.5f </z-m>\n    </center>\n'
                  '    <axis>\n      <x> -1 </x>\n      <y> 0 </y>\n      <z> 0 </z>\n    </axis>\n  </animation>\n' % centre)


def objects_xml(objs):
    return ''.join('    <object-name>%s</object-name>\n' % o for o in objs)


def lighting_xml(objs):
    """the instrument lights (INSTR LIGHT) on `objs`, outside the HDR pipeline: the material's emission (in it the
    bezel's lamps light them, bezel_light_xml)"""
    return ('\n  <animation>\n    <type>material</type>\n%s    <emission>\n      <factor-prop>%sinstruments</factor-prop>\n'
            '      <red> %.2f </red>\n      <green> %.2f </green>\n      <blue> %.2f </blue>\n    </emission>\n'
            '  </animation>\n' % ((objects_xml(objs), LIGHTS) + LAMP))


def integral_xml(objs):
    """an integrally lit part's instrument lighting (its markings light themselves): the material's emission, and in the
    HDR pipeline the emission that matches a face lit by LIGHT_LUX"""
    return lighting_xml(objs) + lit_xml(objs, LIGHTS + 'instruments', NVG_GREEN, reflect=1.0, gain=LIGHT_LUX / math.pi)


def lit_xml(objs, level, rgb=(1, 1, 1), reflect=0.05, gain=1.0):
    """HDR pipeline: `objs` emit their texture x rgb x level x gain (cd/m2: level a property, or a number), through
    Effects/Lit, and reflect the light on them as their texture x `reflect` (a lamp's dead front by default)"""
    lv = '<use>%s</use>' % level if isinstance(level, str) else ' %g ' % level
    return ('\n  <effect>\n    <inherits-from>%sLit/lit</inherits-from>\n    <parameters>\n'
            '      <lit-color type="vec3d"> %.2f %.2f %.2f </lit-color>\n      <lit-level type="float">%s</lit-level>\n'
            '      <lit-gain type="float"> %.4g </lit-gain>\n      <lit-reflectance type="float"> %g </lit-reflectance>\n'
            '    </parameters>\n%s  </effect>\n' % ((EFFECTS,) + tuple(rgb) + (lv, gain, reflect, objects_xml(objs))))


def bezel_lamps(face, R, glass):
    """The lamps of a round lighted bezel (after the UH-60 model's instruments.bezel_lamps): eight spot lights round the
    dial (radius R, centre face = (d, y, z)), 0.15 R out from its edge and a quarter of R above it but behind the glass
    (at depth `glass`, else the glass mirrors them), each aimed across the dial at its centre 0.1 R above it; their
    exponent of 3 holds back the light on the dial near each lamp, so that it is lit about evenly. Returns
    [(position, aim point)], the exponent, cutoff (deg), range (m) and the intensity (cd) that gives the dial a mean
    illuminance of 1 lux."""
    d, y, z = face
    h, off, up, e = min(0.25 * R, glass - 0.0015 - d), 0.15 * R, 0.10 * R, 3.0
    lamps = [((d + h, y + (R + off) * math.cos(t), z + (R + off) * math.sin(t)), (d + up, y, z))
             for t in (2 * math.pi * (k + 0.5) / 8 for k in range(8))]
    g = np.linspace(-0.97, 0.97, 41); GY, GZ = np.meshgrid(g * R, g * R)
    inside = GY ** 2 + GZ ** 2 <= (0.97 * R) ** 2
    F = np.stack([np.full(inside.sum(), d), y + GY[inside], z + GZ[inside]], 1)
    rng = 1.16 * max(np.linalg.norm(F - np.array(p), axis=1).max() for p, a in lamps)
    cut, E = 0.0, np.zeros(len(F))
    for p, a in lamps:
        v = F - np.array(p); dist = np.linalg.norm(v, axis=1); u = v / dist[:, None]
        axis = np.array(a) - np.array(p); axis /= np.linalg.norm(axis)
        c = np.clip(u @ axis, 1e-6, 1)
        cut = max(cut, math.degrees(math.acos(c.min())))
        E += c ** e * (-u[:, 0]) * np.clip(1 - (dist / rng) ** 4, 0, None) ** 2 / dist ** 2
    return lamps, e, min(89.0, cut + 2), rng, 1 / E.mean()


def bezel_light_xml(face, R, glass, light='instruments'):
    """HDR pipeline: the instrument's lighted bezel, NVIS Green A, on circuit `light` (in LIGHTS): its dial's mean
    illuminance LIGHT_LUX at full INSTR LIGHT"""
    lamps, e, cut, rng, cd = bezel_lamps(face, R, glass)
    # off, the lamps leave the scene: the HDR renderer sorts every light in view into its light grid each frame, lit or not
    sel = ('\n  <animation>\n    <type>select</type>\n%s    <condition>\n      <greater-than>\n        <property>%s%s</property>\n'
           '        <value>0</value>\n      </greater-than>\n    </condition>\n  </animation>\n'
           % (''.join('    <object-name>bezel-lamp-%d</object-name>\n' % k for k in range(len(lamps))), LIGHTS, light))
    return sel + ''.join('\n  <light>\n    <name>bezel-lamp-%d</name>\n    <type>spot</type>\n    <position>\n      <x-m> %.4f </x-m>\n'
                   '      <y-m> %.4f </y-m>\n      <z-m> %.4f </z-m>\n    </position>\n    <direction>\n'
                   '      <lookat-x-m> %.4f </lookat-x-m>\n      <lookat-y-m> %.4f </lookat-y-m>\n      <lookat-z-m> %.4f </lookat-z-m>\n'
                   '    </direction>\n    <color>\n      <r> %.2f </r>\n      <g> %.2f </g>\n      <b> %.2f </b>\n    </color>\n'
                   '    <intensity>\n      <property>%s%s</property>\n      <scale> %.6g </scale>\n    </intensity>\n'
                   '    <spot-exponent> %g </spot-exponent>\n    <spot-cutoff> %.1f </spot-cutoff>\n    <range-m> %.3f </range-m>\n'
                   '  </light>\n' % ((k,) + tuple(p) + tuple(a) + NVG_GREEN + (LIGHTS, light, LIGHT_LUX * cd / HDR_CD, e, cut, rng))
                   for k, (p, a) in enumerate(lamps))


SHADE = 0.3            # the share of the sky's light that reaches the panel under the glareshield
ORM = 'Models/Textures/case-orm-%02d.png'           # in the aircraft: an ORM texture (occlusion, roughness 1, metal 0)


def orm(occlusion):
    """ORM's texture for occlusion (written once), its path in FlightGear"""
    path = ORM % round(100 * occlusion)
    f = os.path.join(INSTR, '..', '..', '..', path)
    if not os.path.exists(f):
        Image.new('RGB', (4, 4), (round(255 * occlusion), 255, 0)).save(f)
    return 'Aircraft/MBB_Bo_105/' + path


def matte_xml(objs, roughness=1.0, occlusion=SHADE):
    """HDR pipeline: the cases painted matte and in the glareshield's shade: the PBR shader at roughness 1 and an
    ambient occlusion (the ORM texture's), else their walls mirror the sky at grazing angles (model-default: roughness
    0.5, the sky's light unoccluded) and show as bright strips"""
    return ('\n  <effect>\n    <inherits-from>Effects/model-pbr</inherits-from>\n    <parameters>\n'
            '      <texture n="2">\n        <image>%s</image>\n        <type>2d</type>\n        <filter>nearest</filter>\n'
            '        <mag-filter>nearest</mag-filter>\n        <wrap-s>repeat</wrap-s>\n        <wrap-t>repeat</wrap-t>\n      </texture>\n'
            '      <metallic-factor type="float"> 0 </metallic-factor>\n      <roughness-factor type="float"> %g </roughness-factor>\n'
            '    </parameters>\n%s  </effect>\n' % (orm(occlusion), roughness, objects_xml(objs)))


def glass_xml(objs):
    return ('\n  <effect>\n    <inherits-from>%sGlass/instrument-glass</inherits-from>\n%s  </effect>\n' % (EFFECTS, objects_xml(objs)))


def write_instrument(name, objs, xml, tex):
    folder = os.path.join(INSTR, name); os.makedirs(folder, exist_ok=True)
    A.write(os.path.join(folder, name + '.ac'), objs)
    open(os.path.join(folder, name + '.xml'), 'w').write(xml.replace('%s', name + '.ac', 1))
    tex.im.save(os.path.join(folder, name + '.png'), optimize=True)


def variant(name, fname, subs):
    """a copy of the instrument's XML with properties swapped (the same mesh and texture)"""
    v = open(os.path.join(INSTR, name, name + '.xml')).read()
    for a, b in subs.items():
        v = v.replace(a, b)
    open(os.path.join(INSTR, name, fname), 'w').write(v)


# ---------------------------------------------------------------- digit cells (the altimeter's hPa window)
# FlightGear's text nodes neither update nor glow in the HDR pipeline, so a counter is digit cells: each a quad onto a
# strip of the digits 0-9 that spans its own texture, which repeats; a textranslate animation steps the cell's texture
# along by a tenth per unit of its digit (as the UH-60 model's instruments.py)

def digit_strip(path, cw, ch, fill, face):
    im = Image.new('RGB', (10 * cw, ch), (0, 0, 0)); d = ImageDraw.Draw(im)
    for k in range(10):
        d.text(((k + 0.5) * cw, ch / 2), str(k), font=face, fill=fill, anchor='mm')
    im.save(path, optimize=True)
    return os.path.basename(path)


def digit_cells(texture, name, n, y0, y1, z0, z1, d):
    w = (y1 - y0) / n; objs = []
    for k in range(n):
        ya = y1 - (k + 1) * w
        o = A.Obj('%s%d' % (name, k), texture)
        A.flat(o, [(ya, z0), (ya + w, z0), (ya + w, z1), (ya, z1)], d, A.planar(ya, ya + w, z0, z1, 0.0, 0.1, 0.0, 1.0))
        objs.append(o)
    return objs


def digits_xml(name, n, prop, unit=1.0):
    out = ''
    for k in range(n):
        place = unit * 10 ** k
        out += ('\n  <animation>\n    <type>textranslate</type>\n    <object-name>%s%d</object-name>\n'
                '    <property>%s</property>\n    <bias> %.9g </bias>\n    <step> %.9g </step>\n    <factor> %.9g </factor>\n'
                '    <min> 0 </min>\n    <axis>\n      <x> 1 </x>\n      <y> 0 </y>\n      <z> 0 </z>\n    </axis>\n'
                '  </animation>\n' % (name, k, prop, 0.5 * unit, place, 0.1 / place))
    return out


# ---------------------------------------------------------------- the instrument

class Gauge:
    """A square-cased round instrument: the dial drawn by the caller; needles, knobs, glass added; the parts' XML."""
    def __init__(s, name, case_w, case_h, win_r, dial_r, win_c=(0.0, 0.0), dial_d=0.006, front=0.016, comment=''):
        s.name, s.R, s.dd, s.front, s.win_c, s.comment = name, dial_r, dial_d, front, win_c, comment
        s.t = Tex(2048, 1024)
        s.black, s.white, s.case_uv = s.t.swatch(BLACK), s.t.swatch(WHITE), s.t.swatch((22, 22, 22))
        s.grey = s.t.swatch((70, 70, 70))
        s.objs = {}; s.xml = ''; s.lit = []; s.free = [1040, 8, 0]; s.occlusion = SHADE; s.integral = False
        body = s.obj('case')
        outline = A.rrect(case_w, case_h, 0.005)
        window = A.circle(win_r, *win_c, n=64)
        A.walls(body, outline, 0.0, front, s.case_uv)
        A.bezel(body, outline, window, front, s.case_uv, *win_c)
        A.walls(body, window, dial_d, front, s.black, outward=False)

    def obj(s, name, glass=False):
        if name not in s.objs:
            s.objs[name] = A.Obj(name, None if glass else s.name + '.png', A.GLASS if glass else 0)
        return s.objs[name]

    def alloc(s, w, h):
        x, y, rh = s.free
        if x + w > 2040:
            x, y, rh = 1040, y + rh + 8, 0
        s.free = [x + w + 8, y, max(rh, h)]
        return x, y, x + w, y + h

    def dial(s, img):
        s.t.im.paste(img, (0, 0))
        cy, cz = s.win_c
        A.flat(s.obj('dial'), A.circle(s.R, cy, cz, n=72), s.dd,
               s.t.area(0, 0, img.width, img.height, cy - s.R, cy + s.R, cz - s.R, cz + s.R))
        s.lit.append('dial')

    def sprite(s, name, draw, w_px, h_px, poly, y0, y1, z0, z1, d, lit=True):
        """the polygon `poly` (inside y0..y1, z0..z1) at depth d, textured with `draw(ImageDraw, w, h)`'s picture of
        that rectangle"""
        x0, py0, x1, py1 = s.alloc(w_px, h_px)
        sub = Image.new('RGB', (w_px, h_px), (0, 0, 0))
        draw(ImageDraw.Draw(sub), w_px, h_px)
        s.t.im.paste(sub, (x0, py0))
        A.flat(s.obj(name), poly, d, s.t.area(x0, py0, x1, py1, y0, y1, z0, z1))
        if lit and name not in s.lit:
            s.lit.append(name)

    def needle(s, name, length, width, tail, d, pivot=(0.0, 0.0), label=None, colour=WHITE, hub=None, tip=0.85):
        """a needle pointing up at zero from its pivot, a label (pointer number) along it, and its hub"""
        py, pz = pivot
        poly = [(py - width / 2, pz - tail), (py + width / 2, pz - tail), (py + width / 2, pz + length * tip),
                (py, pz + length), (py - width / 2, pz + length * tip)]
        y0, y1, z0, z1 = py - width / 2, py + width / 2, pz - tail, pz + length
        hpx = 256; wpx = max(8, int(hpx * width / (length + tail)))

        def draw(dr, W, H):
            dr.rectangle([0, 0, W, H], fill=colour)
            if label:
                f = font(W * 0.95)
                dr.text((W / 2, H * (1 - (tail + 0.58 * length) / (length + tail))), label, font=f, fill=BLACK, anchor='mm')
        s.sprite(name, draw, wpx, hpx, poly, y0, y1, z0, z1, d)
        A.cylinder(s.obj(name), hub or width * 0.9, py, pz, d, d + 0.0006, s.black, s.black, n=20)

    def knob(s, name, y, z, dia, h=0.008):
        o = s.obj(name)
        A.cylinder(o, dia / 2, y, z, 0.0, h, s.grey, s.black, n=28)
        A.box(o, y - 0.0005, y + 0.0005, z + dia * 0.10, z + dia * 0.44, h, h + 0.0003, s.white, s.white)

    def glass(s):
        A.flat(s.obj('glass', True), A.circle(s.R + 0.0006, *s.win_c, n=64), s.front - 0.0012, A.const(0, 0))

    def write(s, variants=()):
        objs = list(s.objs.values())
        x = '<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- %s; written by fdm/cockpit/gauges.py -->\n\n' % s.comment
        x += '<PropertyList>\n\n  <path>%%s</path>\n\n  <animation>\n    <type>noshadow</type>\n%s  </animation>\n' % \
             objects_xml([o.name for o in objs])
        light = (integral_xml(s.lit) if s.integral else
                 lighting_xml(s.lit) + bezel_light_xml((s.dd, s.win_c[0], s.win_c[1]), s.R, s.front - 0.0012))
        x += glass_xml([o.name for o in objs if o.mat == A.GLASS]) + light + matte_xml(['case'], occlusion=s.occlusion) + s.xml + '\n</PropertyList>\n'
        write_instrument(s.name, objs, x, s.t)
        for fname, subs in variants:
            variant(s.name, fname, subs)


def ticks(D, values, angle, r0, r_long, r_short, big, w_long, w_short, c=(0.0, 0.0)):
    for v in values:
        b = big(v)
        D.tick(angle(v), r0, r_long if b else r_short, w_long if b else w_short, c=c)


# ---------------------------------------------------------------- airspeed

# the figure's dial: 10 kt at 15 deg, 20 at 36, then 2.36 deg/kt to 150 at 343
ASI_TABLE = [(0, 0), (10, 15), (20, 36), (150, 343), (160, 352)]


def asi():
    R = DIAL3; g = Gauge('asi', CASE3, CASE3, WIN3, R, comment='airspeed indicator, flight manual fig. 7-3 and 2.18.1')
    D = Dial(1024, R); a = lambda v: interp(ASI_TABLE, v)
    D.arc(a(10), a(45), R * 0.90, R * 0.97, YELLOW)
    D.arc(a(45), a(145), R * 0.90, R * 0.97, GREEN)
    D.tick(a(145), R * 0.80, R * 0.99, 0.0012, RED)
    D.tick(a(100), R * 0.80, R * 0.99, 0.0010, BLUE)
    ticks(D, range(20, 151, 5), a, R * 0.89, R * 0.74, R * 0.81, lambda v: v % 10 == 0, 0.0010, 0.0006)
    D.tick(a(10), R * 0.89, R * 0.78, 0.0007)
    for v in range(20, 141, 20):
        D.text_at(R * 0.58, a(v), str(v), 0.0072)
    D.text_at(R * 0.62, a(150), '150', 0.0042); D.text_at(R * 0.66, a(10), '10', 0.0042)
    D.text(0, R * 0.14, 'AIRSPEED', 0.0040); D.text(0, -R * 0.24, 'KNOTS', 0.0040)
    g.dial(D.done())
    g.needle('needle', R * 0.88, 0.0022, 0.008, g.dd + 0.0010)
    g.glass()
    g.xml += rot('needle', 'instrumentation/airspeed-indicator/indicated-speed-kt', [(-999, 0)] + ASI_TABLE)
    g.write()


# ---------------------------------------------------------------- altimeter

def alt():
    R = DIAL3; g = Gauge('alt', CASE3, CASE3, WIN3, R, comment='altimeter, flight manual fig. 7-3')
    D = Dial(1024, R)
    ticks(D, range(50), lambda k: k * 7.2, R * 0.97, R * 0.82, R * 0.89, lambda k: k % 5 == 0, 0.0011, 0.0006)
    for n in range(10):
        D.text_at(R * 0.67, n * 36, str(n), 0.0085)
    D.text(0.0035, R * 0.50, '100 FEET', 0.0026)
    D.text(0.0, R * 0.33, '1000 FEET', 0.0026); D.text(0.0, R * 0.23, '10 000 FEET', 0.0026)
    # the hPa window below the centre
    wy0, wy1, wz0, wz1 = -0.0095, 0.0095, -R * 0.52, -R * 0.36
    D.d.rectangle([D.xy(wy0 - 0.0006, wz1 + 0.0006), D.xy(wy1 + 0.0006, wz0 - 0.0006)], fill=(0, 0, 0), outline=WHITE,
                  width=D.w(0.0004))
    D.text(0, -R * 0.26, 'hPa', 0.0030)
    g.dial(D.done())
    tex = digit_strip(os.path.join(_mk('alt'), 'alt-hpa.png'), 64, 96, WHITE, font(70))
    # on the dial's window, in front of it (the dial is solid: behind it the cells were hidden) and under the needles
    for o in digit_cells(tex, 'hpa', 4, wy0, wy1, wz0, wz1, g.dd + 0.0003):
        g.objs[o.name] = o; g.lit.append(o.name)
    g.needle('ptr10k', R * 0.95, 0.0009, 0.0, g.dd + 0.0006, hub=0.0018)
    o = g.obj('ptr10k')                                              # its triangle tip
    A.flat(o, [(-0.0026, R * 0.80), (0.0026, R * 0.80), (0.0, R * 0.95)], g.dd + 0.0006, g.white)
    g.needle('ptr1000', R * 0.55, 0.0040, 0.006, g.dd + 0.0012, tip=0.75)
    g.needle('ptr100', R * 0.90, 0.0022, 0.010, g.dd + 0.0018)
    g.knob('knob', -0.0335, -0.0335, 0.012)
    g.glass()
    p = 'instrumentation/altimeter/indicated-altitude-ft'
    g.xml += rot('ptr100', p, factor=0.36) + rot('ptr1000', p, factor=0.036) + rot('ptr10k', p, factor=0.0036)
    g.xml += digits_xml('hpa', 4, 'instrumentation/altimeter/setting-hpa')
    g.xml += knob_pick('knob', 'instrumentation/altimeter/setting-hpa', 1, 900, 1100, 'Altimeter setting (hPa)')
    g.write()


def _mk(name):
    folder = os.path.join(INSTR, name); os.makedirs(folder, exist_ok=True); return folder


def knob_pick(obj, prop, step, lo, hi, tooltip):
    """click (left down, right up) or scroll to set `prop`"""
    def act(d, b):
        return ('    <action>\n      <button>%d</button>\n      <repeatable>true</repeatable>\n      <binding>\n'
                '        <command>property-adjust</command>\n        <property>%s</property>\n        <step>%g</step>\n'
                '        <min>%g</min>\n        <max>%g</max>\n        <wrap>false</wrap>\n      </binding>\n    </action>\n'
                % (b, prop, d, lo, hi))
    return ('\n  <animation>\n    <type>pick</type>\n    <object-name>%s</object-name>\n%s%s%s%s'
            '    <hovered>\n      <binding>\n        <command>set-tooltip</command>\n        <tooltip-id>%s</tooltip-id>\n'
            '        <label>%s: %%s</label>\n        <property>%s</property>\n      </binding>\n    </hovered>\n  </animation>\n'
            % (obj, act(-step, 0), act(step, 2), act(step, 3), act(-step, 4), obj, tooltip, prop))


# ---------------------------------------------------------------- vertical speed

# the figure's dial: 0 at 9 o'clock, up clockwise (down anticlockwise): 0.5, 1, 2, 4, 6 thousand ft/min at
# 29, 47, 83, 129 and 180 deg
VSI_SCALE = [(0, 0), (500, 29), (1000, 47), (2000, 83), (4000, 129), (6000, 180)]


def vsi():
    R = DIAL3; g = Gauge('vsi', CASE3, CASE3, WIN3, R, comment='vertical speed indicator, flight manual fig. 7-3')
    D = Dial(1024, R)
    for up in (1, -1):
        a = lambda v: -90 + up * interp(VSI_SCALE, v)
        vals = list(range(0, 1001, 100)) + list(range(1200, 2001, 200)) + list(range(2500, 6001, 500))
        ticks(D, [v for v in vals if not (up < 0 and v == 6000)], a, R * 0.97, R * 0.80, R * 0.88,
              lambda v: v % 1000 == 0, 0.0011, 0.0006)
        for v, lab in ((1000, '1'), (2000, '2'), (4000, '4')) + (((6000, '6'),) if up > 0 else ()):
            D.text_at(R * 0.64, a(v), lab, 0.0085)
        D.text_at(R * 0.68, a(500), '.5', 0.0042)
    D.text(0, R * 0.40, 'CLIMB', 0.0040)
    D.text(0, -R * 0.40, '1000 FT/MIN', 0.0030)
    g.dial(D.done())
    g.needle('needle', R * 0.90, 0.0026, 0.010, g.dd + 0.0010)
    g.glass()
    table = [(-99999, -270)] + [(-v, -90 - d) for v, d in VSI_SCALE[:0:-1]] + [(v, -90 + d) for v, d in VSI_SCALE] + [(99999, 90)]
    g.xml += rot('needle', 'instrumentation/vertical-speed-indicator/indicated-speed-fpm', table)
    g.write()


# ---------------------------------------------------------------- triple tachometer

def tach_a(v):
    """the figure's scale: 0 % at 30 deg, 2.43 deg per %"""
    return 30 + 2.43 * v


def tach():
    R = DIAL3; g = Gauge('tach', CASE3, CASE3, WIN3, R, comment='triple tachometer, flight manual fig. 7-3 and 2.18.3 '
                         '(the CBS: model S variants)')
    D = Dial(1024, R); a = tach_a
    # engines (N2), outside: green 98-102 %, red radials 98 and 102, red dots 95 and 105
    D.arc(a(98), a(102), R * 0.90, R * 0.98, GREEN)
    for v in (98, 102):
        D.tick(a(v), R * 0.84, R * 0.99, 0.0011, RED)
    for v in (95, 105):
        D.dot(a(v), R * 0.94, 0.0011)
    ticks(D, range(0, 141, 2), a, R * 0.89, R * 0.75, R * 0.82, lambda v: v % 10 == 0, 0.0010, 0.0005)
    for v in range(10, 141, 10):
        D.text_at(R * 0.66, a(v), str(v), 0.0046, narrow=True)
    # rotor, inside: green 85-104 %, red radials 85 and 104, red dot 110
    Ri = R * 0.47
    D.circle(Ri, 0.0004)
    D.arc(a(85), a(104), Ri - 0.0028, Ri - 0.0006, GREEN)
    for v in (85, 104):
        D.tick(a(v), Ri - 0.0040, Ri + 0.0006, 0.0010, RED)
    D.dot(a(110), Ri - 0.0017, 0.0010)
    ticks(D, range(0, 141, 10), a, Ri, Ri - 0.0030, Ri - 0.0030, lambda v: True, 0.0005, 0.0005)
    D.text(0, R * 0.24, 'ROT', 0.0034)
    D.text_at(R * 0.80, a(140) + 9, 'ENG', 0.0026)
    D.text(0, -R * 0.20, 'PERCENT', 0.0028); D.text(0, -R * 0.30, 'RPM', 0.0028)
    g.dial(D.done())
    g.needle('rotor', Ri - 0.0008, 0.0042, 0.006, g.dd + 0.0008, label='R', tip=0.80)
    g.needle('eng1', R * 0.90, 0.0026, 0.008, g.dd + 0.0014, label='1')
    g.needle('eng2', R * 0.90, 0.0026, 0.008, g.dd + 0.0020, label='2')
    g.glass()
    t = lambda: [(-10, a(0)), (0, a(0)), (140, a(140)), (150, a(140) + 5)]
    g.xml += rot('eng1', '/engines/engine[0]/n2-pct', t()) + rot('eng2', '/engines/engine[1]/n2-pct', t()) + \
             rot('rotor', '/rotors/main/rpm-pct', t())
    g.write()


# ---------------------------------------------------------------- dual torque

def tq_a(v):
    """the figure's scale: 0 % at 190 deg, 1.7 deg per % (100 % at 12 o'clock)"""
    return 190 + 1.7 * v


def tq():
    R = DIAL2; g = Gauge('tq', CASE2, CASE2, WIN2, R, comment='dual torque indicator, flight manual fig. 7-3 and 2.18.7')
    D = Dial(1024, R); a = tq_a
    D.arc(a(0), a(86), R * 0.88, R * 0.97, GREEN)
    for v in (86, 95):
        D.tick(a(v), R * 0.80, R * 0.99, 0.0009, RED)
    ticks(D, range(0, 121, 2), a, R * 0.87, R * 0.72, R * 0.80, lambda v: v % 10 == 0, 0.0008, 0.0004)
    for v in (0, 20, 40, 60, 80, 100, 110):
        D.text_at(R * (0.57 if v < 100 else 0.55), a(v) + {100: -3, 110: 3}.get(v, 0), str(v),
                  0.0042 if v < 100 else 0.0030, narrow=True)
    D.text(R * 0.42, R * 0.10, '%', 0.0032); D.text(R * 0.42, -R * 0.22, 'TORQUE', 0.0026, angle=90)
    g.dial(D.done())
    g.needle('eng1', R * 0.88, 0.0020, 0.006, g.dd + 0.0010, label='1')
    g.needle('eng2', R * 0.88, 0.0020, 0.006, g.dd + 0.0016, label='2')
    g.glass()
    t = [(-10, a(0) - 4), (0, a(0)), (120, a(120)), (130, a(120) + 4)]
    g.xml += rot('eng1', '/engines/engine[0]/torque-pct', t) + rot('eng2', '/engines/engine[1]/torque-pct', t)
    g.write()


# ---------------------------------------------------------------- TOT

def tot_a(v):
    """0 degC at 225 deg (7:30), 10 x 100 at 135 deg (4:30): 0.27 deg per degC"""
    return 225 + 0.27 * v


def tot():
    R = DIAL2S; g = Gauge('tot', CASE2S, CASE2S, WIN2S, R, comment='TOT indicator, flight manual fig. 7-3 and 2.18.4')
    D = Dial(1024, R); a = tot_a
    D.arc(a(0), a(779), R * 0.88, R * 0.97, GREEN)
    D.arc(a(779), a(810), R * 0.88, R * 0.97, YELLOW)
    D.tick(a(810), R * 0.80, R * 0.99, 0.0009, RED)
    for v in (843, 927):
        D.dot(a(v), R * 0.925, 0.0009)
    ticks(D, range(0, 1001, 20), a, R * 0.87, R * 0.72, R * 0.80, lambda v: v % 100 == 0, 0.0008, 0.0004)
    for v in (2, 4, 6, 8, 10):
        D.text_at(R * 0.57, a(100 * v), str(v), 0.0050)
    D.text(0, R * 0.30, '°C', 0.0034); D.text(0, -R * 0.30, 'x100', 0.0026); D.text(0, -R * 0.48, 'TOT', 0.0034)
    g.dial(D.done())
    g.needle('needle', R * 0.88, 0.0020, 0.006, g.dd + 0.0010)
    g.glass()
    g.xml += rot('needle', '/engines/engine[0]/tot-degc', [(-100, a(0) - 3), (0, a(0)), (1000, a(1000)), (1200, a(1000) + 5)])
    g.write(variants=[('tot1.xml', {}), ('tot2.xml', {'/engines/engine[0]/': '/engines/engine[1]/'})])


# ---------------------------------------------------------------- N1

def n1_a(v):
    return 2.7 * v                              # 0 at 12 o'clock, 100 % at 9 o'clock


def n1():
    R = DIAL2S; g = Gauge('n1', CASE2S, CASE2S, WIN2S, R, comment='gas producer (N1) indicator, flight manual fig. 7-3 and 2.18.2')
    D = Dial(1024, R); a = n1_a
    D.arc(a(59), a(105), R * 0.89, R * 0.97, GREEN)
    D.tick(a(105), R * 0.80, R * 0.99, 0.0009, RED)
    ticks(D, range(0, 106, 2), a, R * 0.87, R * 0.73, R * 0.80, lambda v: v % 10 == 0, 0.0008, 0.0004)
    for v in range(10, 101, 10):
        D.text_at(R * 0.60, a(v), str(v), 0.0040 if v < 100 else 0.0034, narrow=True)
    # the vernier: one turn per 10 %, upper left
    vc = (-R * 0.40, R * 0.30); vr = R * 0.24
    D.circle(vr, 0.0003, c=vc)
    for k in range(10):
        D.tick(k * 36, vr, vr * 0.78, 0.0004, c=vc)
        D.text_at(vr * 0.52, k * 36, str(k), 0.0020, c=vc)
    D.text(R * 0.30, -R * 0.02, '%', 0.0030); D.text(R * 0.30, -R * 0.22, 'N1', 0.0030)
    g.dial(D.done())
    g.needle('vernier', vr * 0.90, 0.0010, 0.002, g.dd + 0.0006, pivot=vc, hub=0.0012)
    g.needle('needle', R * 0.88, 0.0020, 0.006, g.dd + 0.0012)
    g.glass()
    g.xml += rot('needle', '/engines/engine[0]/n1-pct', [(-10, -3), (0, 0), (110, a(110)), (120, a(110) + 4)])
    g.xml += rot('vernier', '/engines/engine[0]/n1-pct', factor=36, centre=vc)
    g.write(variants=[('n1-1.xml', {}), ('n1-2.xml', {'/engines/engine[0]/': '/engines/engine[1]/'})])


# ---------------------------------------------------------------- dual fuel gauges: two pointers pivoting low

def lens(name, comment, left, right, labels, props):
    """left/right: (top value, numbers, unit factor, markings [(kind, v0, v1)]); each pointer pivots on the far side
    of the centre and sweeps 100 deg from 0 (low) up its own side to its top value: a lens between the scales"""
    R = DIAL2S; g = Gauge(name, CASE2S, CASE2S, WIN2S, R, comment=comment)
    D = Dial(1024, R)
    L = R * 0.88
    sides = []
    for sgn, (top, nums, _, marks) in ((-1, left), (1, right)):
        piv = (-sgn * R * 0.28, -R * 0.05)
        a = (lambda top, sgn: lambda v: sgn * (140 - 100 * v / top))(top, sgn)
        for kind, v0, v1 in marks:
            if kind in ('green', 'yellow'):
                D.arc(a(v0), a(v1), L * 0.92, L * 0.99, GREEN if kind == 'green' else YELLOW, c=piv)
            elif kind == 'red':
                D.tick(a(v0), L * 0.86, L * 1.01, 0.0009, RED, c=piv)
        n = 20
        for k in range(n + 1):
            v = top * k / n
            big = k % 5 == 0
            D.tick(a(v), L * 0.90, L * (0.80 if big else 0.85), 0.0007 if big else 0.0004, c=piv)
        for v, txt in nums:
            D.text_at(L * 1.12, a(v), txt, 0.0032, c=piv, narrow=True)
        sides.append((piv, a))
    for y, z, txt, size in labels:
        D.text(y * R, z * R, txt, size)
    g.dial(D.done())
    for k, ((piv, a), prop, f) in enumerate(zip(sides, props, (left[2], right[2]))):
        g.needle('ptr%d' % k, L * 0.97, 0.0016, 0.004, g.dd + 0.0010 + 0.0006 * k, pivot=piv, hub=0.0018)
        top = (left, right)[k][0]
        g.xml += rot('ptr%d' % k, prop, [(-1e6, a(-0.04 * top)), (0, a(0)), (top / f, a(top)), (1.1 * top / f, a(1.05 * top))],
                     centre=piv)
    g.glass()
    g.write()


def fq():
    KG10 = 0.45359 / 10                         # lb -> kg x 10
    lens('fq', 'dual fuel quantity indicator, flight manual fig. 7-3 and 7.6.3: main tank M, supply tank S, kg x 10',
         (46.0, [(0, '0'), (10, '10'), (20, '20'), (30, '30'), (40, '40')], KG10, []),
         (8.0, [(0, '0'), (2, '2'), (4, '4'), (6, '6'), (8, '8')], KG10, []),
         [(-0.80, 0.46, 'M', 0.0040), (0.80, 0.46, 'S', 0.0040), (0, -0.40, 'kg', 0.0024), (0, -0.54, 'x10', 0.0024),
          (0, 0.44, 'QUANT', 0.0024), (0, 0.30, 'FUEL', 0.0024)],
         ['/consumables/fuel/tank[0]/level-lbs', '/consumables/fuel/tank[1]/level-lbs'])


def fp():
    marks = [('yellow', 0.0, 0.6), ('green', 0.6, 1.7), ('red', 1.7, 1.7)]
    lens('fp', 'dual fuel pressure indicator, flight manual fig. 7-3 and 2.18.8: engines 1 and 2, kp/cm2',
         (2.5, [(0, '0'), (1, '1'), (2, '2')], 1.0, marks), (2.5, [(0, '0'), (1, '1'), (2, '2')], 1.0, marks),
         [(-0.80, 0.46, '1', 0.0040), (0.80, 0.46, '2', 0.0040), (0, -0.46, 'kp/cm²', 0.0024),
          (0, 0.44, 'PRESS', 0.0024), (0, 0.30, 'FUEL', 0.0024)],
         ['/engines/engine[0]/fuel-press-kpcm2', '/engines/engine[1]/fuel-press-kpcm2'])


# ---------------------------------------------------------------- triple oil gauges

def triple(name, comment, scales, labels, props):
    """three sub-gauges, each a 110 deg scale above its pivot: (pivot y, z (x R), top value, numbers, markings)"""
    R = DIAL3; g = Gauge(name, CASE3, CASE3, WIN3, R, comment=comment)
    D = Dial(1024, R)
    subs = []
    for (py, pz, lo, top, nums, marks) in scales:
        c = (py * R, pz * R); l = R * 0.42
        a = (lambda lo, top: lambda v: -55 + 110 * (v - lo) / (top - lo))(lo, top)
        for kind, v0, v1 in marks:
            if kind in ('green', 'yellow'):
                D.arc(a(v0), a(v1), l * 0.90, l * 1.00, GREEN if kind == 'green' else YELLOW, c=c)
            elif kind == 'red':
                D.tick(a(v0), l * 0.80, l * 1.03, 0.0011, RED, c=c)
        span = top - lo; step = span / 20
        for k in range(21):
            v = lo + k * step
            D.tick(a(v), l * 0.88, l * (0.74 if k % 5 == 0 else 0.80), 0.0008 if k % 5 == 0 else 0.0005, c=c)
        for v, txt in nums:
            D.text_at(l * 0.56, a(v), txt, 0.0029, c=c, narrow=True)
        D.disc(0.0042, (40, 40, 40), c=c)
        subs.append((c, a, lo, top))
    for y, z, txt, size in labels:
        D.text(y * R, z * R, txt, size)
    g.dial(D.done())
    for k, ((c, a, lo, top), prop) in enumerate(zip(subs, props)):
        g.needle('ptr%d' % k, R * 0.40, 0.0018, 0.003, g.dd + 0.0010, pivot=c, hub=0.0030)
        g.xml += rot('ptr%d' % k, prop, [(lo - 1000, a(lo - 0.04 * (top - lo))), (lo, a(lo)), (top, a(top)),
                                         (top + 1000, a(top + 0.04 * (top - lo)))], centre=c)
    g.glass()
    g.write()


def oilt():
    eng = [('green', 30, 107), ('red', 107, 107)]
    xmsn = [('green', 30, 105), ('red', 105, 105)]
    nums = [(30, '30'), (60, '60'), (90, '90'), (120, '120')]
    triple('oilt', 'triple oil temperature indicator, flight manual fig. 7-3 and 2.18.6: engines 1 and 2, main '
           'transmission (top), degC',
           [(0.0, 0.18, 0, 130, nums, xmsn), (-0.47, -0.32, 0, 130, nums, eng), (0.47, -0.32, 0, 130, nums, eng)],
           [(0, -0.30, '°C', 0.0040), (0, -0.48, 'TEMP', 0.0034), (0, -0.64, 'OIL', 0.0034),
            (-0.47, -0.62, '1', 0.0034), (0.47, -0.62, '2', 0.0034)],
           ['/rotors/gear/oil-temp-degc', '/engines/engine[0]/oil-temp-degc', '/engines/engine[1]/oil-temp-degc'])


def oilp():
    eng = [('red', 3.5, 3.5), ('yellow', 3.5, 6.3), ('green', 6.3, 9.1), ('red', 9.1, 9.1)]
    xmsn = [('red', 0.5, 0.5), ('yellow', 0.5, 0.8), ('green', 0.8, 2.0)]
    triple('oilp', 'triple oil pressure indicator, flight manual fig. 7-3 and 2.18.5: engines 1 and 2, main '
           'transmission R (top), kp/cm2',
           [(0.0, 0.18, 0, 2.5, [(0, '0'), (1, '1'), (2, '2')], xmsn),
            (-0.47, -0.32, 0, 11, [(0, '0'), (5, '5'), (10, '10')], eng),
            (0.47, -0.32, 0, 11, [(0, '0'), (5, '5'), (10, '10')], eng)],
           [(0, 0.06, 'R', 0.0032), (0, -0.30, 'kp/cm²', 0.0028), (0, -0.46, 'PRESS', 0.0034),
            (0, -0.66, 'OIL', 0.0034), (-0.47, -0.62, '1', 0.0034), (0.47, -0.62, '2', 0.0034)],
           ['/rotors/gear/oil-pressure-kpcm2', '/engines/engine[0]/oil-pressure-kpcm2',
            '/engines/engine[1]/oil-pressure-kpcm2'])


# ---------------------------------------------------------------- mast moment indicator

def mmi_a(v):
    """0 at 7:30, 100 % (the red radial) at 2 o'clock"""
    return -135 + 1.95 * v


def mmi():
    R = 0.0250; c = (0.0, 0.0060)
    g = Gauge('mmi', 0.081, 0.078, R + 0.0007, R, win_c=c, comment='mast moment indicator, flight manual fig. 7-3, '
              '2.17.3 and 7.7.5: % of the hub moment limit')
    D = Dial(1024, R); a = mmi_a
    D.arc(a(0), a(75), R * 0.80, R * 0.95, GREEN)
    D.arc(a(75), a(100), R * 0.80, R * 0.95, YELLOW)
    D.tick(a(100), R * 0.70, R * 0.99, 0.0011, RED)
    D.tick(a(69.2), R * 0.70, R * 0.99, 0.0009, WHITE)                    # the calibration radial, 12 o'clock
    ticks(D, range(0, 131, 5), a, R * 0.78, R * 0.66, R * 0.72, lambda v: v % 25 == 0, 0.0007, 0.0004)
    g.dial(D.done())
    g.needle('needle', R * 0.88, 0.0018, 0.005, g.dd + 0.0010, pivot=c, hub=0.0026)
    g.glass()
    # the face below the dial: CAL screw, TEST pushbutton, LIMIT light, their lettering
    t = g.t; x0, y0, x1, y1 = g.alloc(512, 192)
    sub = Image.new('RGB', (512, 192), (22, 22, 22)); dr = ImageDraw.Draw(sub)
    for yy, txt in ((-0.025, 'CAL'), (-0.0085, 'TEST'), (0.0215, 'LIMIT')):
        dr.text((256 + yy / 0.081 * 512, 40), txt, font=font(30), fill=WHITE, anchor='mm')
    dr.text((256 + 0.0068 / 0.081 * 512, 128), 'MM', font=font(46), fill=WHITE, anchor='mm')
    t.im.paste(sub, (x0, y0))
    fz0, fz1 = -0.0385, -0.0185
    A.flat(g.obj('face'), [(-0.0405, fz0), (0.0405, fz0), (0.0405, fz1), (-0.0405, fz1)], g.front + 0.0002,
           t.area(x0, y0, x1, y1, -0.0405, 0.0405, fz0, fz1))
    g.lit.append('face')
    A.cylinder(g.obj('cal'), 0.0022, -0.025, -0.031, g.front, g.front + 0.0015, g.grey, g.black, n=16)
    A.cylinder(g.obj('test'), 0.0040, -0.0085, -0.031, g.front, g.front + 0.0060, g.grey, g.black, n=6)
    lim = g.obj('limit')
    A.cylinder(lim, 0.0040, 0.0215, -0.031, g.front, g.front + 0.0040, g.t.swatch((230, 45, 35)), g.black, n=24)
    tbody = [(-1e6, a(-3)), (0, a(0)), (130, a(130)), (1e6, a(133))]
    g.xml += rot('needle', '/instrumentation/mmi/indicated-pct', tbody, centre=c)
    g.xml += push_pick('test', '/instrumentation/mmi/test', 'Mast moment indicator TEST')
    g.xml += lamp_xml(['limit'], '/instrumentation/mmi/limit-light', (1.0, 0.10, 0.06))
    g.xml += lit_xml(['limit'], LIGHTS_HDR + 'mmi-limit', reflect=0.3)
    g.write()


def push_pick(obj, prop, tooltip):
    """a momentary pushbutton: prop 1 while held; the button moves in"""
    return ('\n  <animation>\n    <type>pick</type>\n    <object-name>%s</object-name>\n    <action>\n      <button>0</button>\n'
            '      <repeatable>false</repeatable>\n      <binding>\n        <command>property-assign</command>\n'
            '        <property>%s</property>\n        <value type="bool">true</value>\n      </binding>\n'
            '      <mod-up>\n        <binding>\n          <command>property-assign</command>\n          <property>%s</property>\n'
            '          <value type="bool">false</value>\n        </binding>\n      </mod-up>\n    </action>\n'
            '    <hovered>\n      <binding>\n        <command>set-tooltip</command>\n        <tooltip-id>%s</tooltip-id>\n'
            '        <label>%s</label>\n      </binding>\n    </hovered>\n  </animation>\n'
            '\n  <animation>\n    <type>translate</type>\n    <object-name>%s</object-name>\n    <property>%s</property>\n'
            '    <factor> -0.002 </factor>\n    <axis>\n      <x> 1 </x>\n      <y> 0 </y>\n      <z> 0 </z>\n    </axis>\n'
            '  </animation>\n' % (obj, prop, prop, obj, tooltip, obj, prop))


def lamp_xml(objs, prop, rgb):
    """a lamp: emits rgb while prop is true"""
    return ('\n  <animation>\n    <type>material</type>\n%s    <condition>\n      <property>%s</property>\n    </condition>\n'
            '    <emission>\n      <red> %.2f </red>\n      <green> %.2f </green>\n      <blue> %.2f </blue>\n    </emission>\n'
            '  </animation>\n' % ((objects_xml(objs), prop) + tuple(rgb)))


# ---------------------------------------------------------------- standby compass

def compass():
    """the standby magnetic compass above the panel (flight manual fig. 7-3, item 1): a round case, its axis toward the
    crew, the card a band round a vertical axis behind a round window, read against the lubber line; the card fixed to
    the earth (it turns with /instrumentation/magnetic-compass/indicated-heading-deg the other way from the aircraft),
    so that the headings to the left of the line are the higher ones"""
    R, rw, rc, hc, dc = 0.030, 0.024, 0.020, 0.016, -0.024     # case, window, card radius and height, card's axis
    g = Gauge('compass', 0.001, 0.001, rw, 0.001, comment='standby magnetic compass, flight manual fig. 7-3')
    g.objs = {}; g.lit = []; g.occlusion = 1.0           # on the windshield's frame, in the open
    g.integral = True                                     # lit from within: its card emits
    case = g.obj('case')
    # the case: its wall and back, open at the front (a closed cylinder's front cap would cover the window)
    A.walls(case, A.circle(R, n=48), -0.060, 0.0, g.case_uv)
    A.flat(case, A.circle(R, n=48), -0.060, g.case_uv, normal=-1)
    A.bezel(case, A.circle(R, n=48), A.circle(rw, n=48), 0.0005, g.case_uv)
    A.walls(case, A.circle(rw, n=48), -0.050, 0.0005, g.black, outward=False)    # the bore round the card
    A.flat(case, A.circle(rw, n=48), -0.050, g.black)                         # inside, behind the card
    A.box(case, -0.004, 0.004, R - 0.002, R + 0.012, -0.040, -0.020, g.case_uv, g.case_uv)    # the strap to the bracket
    # the card's strip: headings every 30 deg, ticks every 5, laid out with u to the right as the crew sees it
    W_, H_ = 2048, 128
    x0, y0, x1, y1 = 0, 1024 - 8 - H_, W_, 1024 - 8
    sub = Image.new('RGB', (W_, H_), (14, 14, 14)); d = ImageDraw.Draw(sub)
    labels = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}
    for h in range(0, 360, 5):
        u = ((360 - h) % 360) / 360 * W_
        big = h % 30 == 0
        d.line([(u, 0), (u, H_ * (0.30 if big else 0.18))], fill=WHITE, width=5 if big else 3)
        if big:
            d.text((u, H_ * 0.65), labels.get(h, str(h // 10)), font=font(H_ * 0.48), fill=WHITE, anchor='mm')
    g.t.im.paste(sub, (x0, y0))
    W, H = g.t.im.size
    card = g.obj('card')
    n = 72
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        u0, u1 = k / n, (k + 1) / n
        p = lambda a, zz: (dc + rc * math.cos(a), rc * math.sin(a), zz)
        uv = lambda u, top: (u, 1 - (y0 if top else y1) / H)
        pts = [p(a0, -hc / 2), p(a1, -hc / 2), p(a1, hc / 2), p(a0, hc / 2)]
        mid = (a0 + a1) / 2
        card.face(pts, [uv(u0, False), uv(u1, False), uv(u1, True), uv(u0, True)], (math.cos(mid), math.sin(mid), 0))
    g.lit.append('card')
    lub = g.obj('lubber')
    A.box(lub, -0.0006, 0.0006, -0.011, 0.011, dc + rc + 0.0015, dc + rc + 0.0025, g.t.swatch((235, 120, 40)), g.t.swatch((235, 120, 40)))
    A.flat(g.obj('glass', True), A.circle(rw, n=48), -0.002, A.const(0, 0))
    g.xml += ('\n  <animation>\n    <type>rotate</type>\n    <object-name>card</object-name>\n'
              '    <property>/instrumentation/magnetic-compass/indicated-heading-deg</property>\n    <factor> 1 </factor>\n'
              '    <center>\n      <x-m> %.4f </x-m>\n      <y-m> 0 </y-m>\n      <z-m> 0 </z-m>\n    </center>\n'
              '    <axis>\n      <x> 0 </x>\n      <y> 0 </y>\n      <z> 1 </z>\n    </axis>\n  </animation>\n' % dc)
    g.write()


# ---------------------------------------------------------------- clock

def clock():
    R = DIAL2S; g = Gauge('clock', CASE2S, CASE2S, WIN2S, R, comment='clock, flight manual fig. 7-3')
    D = Dial(1024, R)
    ticks(D, range(60), lambda m: 6 * m, R * 0.97, R * 0.80, R * 0.89, lambda m: m % 5 == 0, 0.0010, 0.0004)
    for h, txt in ((12, '12'), (3, '3'), (9, '9')):
        D.text_at(R * 0.63, 30 * h, txt, 0.0060)
    sc = (0.0, -R * 0.42); sr = R * 0.25                                  # the minute counter
    D.circle(sr, 0.0003, c=sc)
    for m in range(0, 60, 5):
        D.tick(6 * m, sr, sr * (0.70 if m % 15 == 0 else 0.82), 0.0004, c=sc)
    g.dial(D.done())
    g.needle('elapsed', sr * 0.85, 0.0008, 0.001, g.dd + 0.0006, pivot=sc, hub=0.0010)
    g.needle('hour', R * 0.55, 0.0030, 0.004, g.dd + 0.0010, tip=0.80)
    g.needle('minute', R * 0.85, 0.0022, 0.005, g.dd + 0.0016)
    g.needle('second', R * 0.92, 0.0008, 0.008, g.dd + 0.0022, colour=(230, 60, 40))
    g.knob('knob', 0.0, -0.0275, 0.007)
    g.glass()
    p = 'instrumentation/clock/indicated-sec'
    g.xml += rot('hour', p, factor=360 / 43200.0) + rot('minute', p, factor=0.1) + rot('second', p, factor=6.0)
    g.xml += rot('elapsed', '/sim/time/elapsed-sec', factor=0.1, centre=sc)
    g.write()


if __name__ == '__main__':
    for f in (asi, alt, vsi, tach, tq, tot, n1, fq, fp, oilt, oilp, mmi, clock, compass):
        f()
        print('wrote', f.__name__)
