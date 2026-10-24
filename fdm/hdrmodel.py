#!/usr/bin/env python3
"""The 3-D model's glass and exterior lights for FlightGear's HDR pipeline, and the others (as the UH-60 model's).

- Windows: bo105.ac's glass (windshields, the fixed chin and roof glazing, the door windows) had no texture, a legacy
  chrome shader and the material animation's transparency, all of which the HDR pipeline leaves out: opaque grey there.
  They get a tinted glass texture whose alpha is the glass's opacity (Models/Textures/glass.png: the HDR transparent
  shader takes its alpha from the texture) and Models/Effects/Glass/glass (model-combined-transparent: reflections in
  ALS, model-transparent in HDR).
- Exterior lights: the old halos (4 cm quads lit by material animations and scaled up to 30 x at night) are hidden, their
  lenses always shown. Each light gets the UH-60 model's halo, a billboard through model-transparent with its emission
  for the other pipelines, scaled with distance, and in the HDR pipeline (/sim/rendering/hdr/hdr-enabled,
  Nasal/hdr.nas) a spot light aimed out through its lens, so that it lights the night around it and not its own skin;
  intensities in candela / 635 (the pipeline's unit), switched by the old states (Nasal/bo105.nas): position lights
  40 cd, red left and green right; anticollision beacons top and bottom, 50 cd on axis rising to 290 cd sideways;
  strobes 400 cd, the tail's aft and the belly's down.
- Interior lights (INTERIOR): the whip light (NVIS green), the utility light and the cabin's ceiling light, HDR spot
  lights in Models/Lights/interior.xml at the levels Nasal/panel.nas sets from the Cockpit lights dialog.
- Liveries: a variant (Models/Variants, the livery selector's) tints the white livery.rgb through the material
  animation's diffuse colour, which the HDR pipeline leaves out (there every livery showed the same colour). The
  painted objects get the PBR shader (Effects/model-pbr) and a pbr animation that sets its base colour and roughness
  from the variant's material/fuselage/pbr, which each variant gets here: its diffuse colour (sRGB, as the old
  lighting showed it) made linear, the paint gloss (civil) or matte (military); the pilots' helmet and flight suit
  likewise (material/helmet/pbr, flightsuit/pbr: fdm/materials.py). Each variant also names the blades'
  paint for Models/Rotor/blade.xml (fdm/rotorvis.py): material/rotor/blade and blur, its rotor texture's.
- Grey textures: the HDR pipeline (a core OpenGL profile) samples a one-channel texture as red and a two-channel one
  as red and green: the white livery.rgb made every livery red, the grey-and-alpha blade paints and dust particles
  (dirt.rgb) likewise. Every such texture under Models is rewritten as RGB or RGBA, its name and format kept.

    python3 hdrmodel.py      edits Models/bo105.ac (the glass's texture), Models/bo105.xml and Models/Variants; writes
                             Models/Lights (with interior.xml), Models/Textures/glass.png
"""
import os, re, shutil
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'MBB_Bo_105', 'Models')
UH60_LIGHTS = os.path.expanduser('~/git/UH-60/UH-60A/Models/lights/Classic')
HDR_CD = 635.0

GLASS = ['glass_fixed', 'windshield_L', 'windshield_R', 'glass_front_L', 'glass_front_R', 'glass_back_L', 'glass_back_R']
GLASS_RGBA = (176, 190, 196, 56)        # a faint grey-green tint, 22 % opaque
OLD_HALOS = ['strobe_halo_T', 'strobe_halo_B', 'strobe_halo_Bf', 'beacon_halo_T', 'beacon_halo_B', 'beacon_halo_Bf',
             'nav_halo_L', 'nav_halo_Lf', 'nav_halo_R', 'nav_halo_Rf']
L = 'sim/model/bo105/lighting/'
RED, GREEN, WHITE = (1.0, 0.06, 0.03), (0.15, 1.0, 0.45), (1.0, 1.0, 1.0)
# name, halo, position (model frame, m: the old halos'), direction, colour, emission (other pipelines), cd on axis,
# spot exponent, cutoff (deg), range (m), the state that lights it
LIGHTS = [('nav-red', 'RedLight.ac', (0.680, -0.814, 0.150), (0, -1, 0), RED, (2, 0, 0), 40, 3, 70, 20, L + 'nav-lights'),
          ('nav-green', 'GreenLight.ac', (0.680, 0.814, 0.150), (0, 1, 0), GREEN, (0, 2, 0), 40, 3, 70, 20, L + 'nav-lights'),
          ('beacon-top', 'RedLight.ac', (5.883, 0.0, 2.008), (0, 0, 1), RED, (2, 0, 0), 50, -1, 80, 30, L + 'beacon-top/state'),
          ('beacon-bottom', 'RedLight.ac', (-1.313, 0.0, -0.734), (0, 0, -1), RED, (2, 0, 0), 50, -1, 80, 30, L + 'beacon-bottom/state'),
          ('strobe-top', 'WhiteLight.ac', (5.992, 0.0, 1.784), (1, 0, 0), WHITE, (2, 2, 2), 400, 1, 80, 40, L + 'strobe-top/state'),
          ('strobe-bottom', 'WhiteLight.ac', (-1.787, 0.0, -0.683), (0, 0, -1), WHITE, (2, 2, 2), 400, 1, 80, 40, L + 'strobe-bottom/state')]
HALO_SCALE = [(0, 0.15), (20, 0.35), (1500, 4.0)]      # m -> the halo's size (m), as the UH-60's a little smaller
# Interior lights [FM 7.12.3-7.12.5], HDR spot lights (Models/Lights/interior.xml) at their levels (0-1) in
# /instrumentation/bo105/lights (Nasal/panel.nas): the whip light at the left of the centre console's switch panel,
# NVIS Green A as the NVG compatible lighting's (FM supplement 10-52), on its own switch, aimed at the panel; the utility
# light at the left of the overhead console, white, on its OFF-BRT rheostat, aimed at the panel and the pilot's lap; the
# cabin's ceiling light (INTERNAL LIGHT). Lights cast no shadows: their ranges end short of the airframe's skin.
# name, position, aim point (model frame, m), colour, cd at full, exponent, cutoff (deg), range (m)
LT_INT = '/instrumentation/bo105/lights/'
NVG_GREEN = (0.10, 1.00, 0.45)
INTERIOR = [('whip', (-1.85, -0.12, 0.42), (-2.21, 0.10, 0.33), NVG_GREEN, 0.8, 2, 35, 0.9),
            ('utility', (-1.65, -0.12, 0.95), (-2.21, 0.12, 0.38), (1.0, 0.95, 0.85), 3.0, 2, 32, 1.3),
            ('cabin', (-0.80, 0.0, 1.00), (-0.80, 0.0, 0.0), (1.0, 0.95, 0.85), 12.0, 1, 75, 1.5)]
VARIANTS = os.path.join(MODELS, 'Variants')
GLOSS, MATTE = 0.35, 0.7                # the paint's roughness: civil liveries, military (Mil_*.xml) ones (as the UH-60's)
LIVERY = 'sim/model/bo105/material/fuselage'
LIVERY_BEGIN = '\t<!-- the livery in the HDR pipeline: written by fdm/hdrmodel.py -->'
LIVERY_END = '\t<!-- end of fdm/hdrmodel.py\'s livery -->'


def glass_ac():
    p = os.path.join(MODELS, 'bo105.ac'); s = open(p, encoding='latin-1').read()
    for n in GLASS:
        i = s.index('name "%s"\n' % n)
        j = s.index('numvert', i)
        if 'texture' not in s[i:j]:
            s = s[:j] + 'texture "Textures/glass.png"\ntexrep 1 1\n' + s[j:]
    open(p, 'w', encoding='latin-1').write(s)
    Image.new('RGBA', (8, 8), GLASS_RGBA).save(os.path.join(MODELS, 'Textures', 'glass.png'))


def light_xml(name, halo, d, rgb, emis, cd, exp, cut, rng, state):
    sel = lambda obj, extra='': ('  <animation>\n    <type>select</type>\n    <object-name>%s</object-name>\n    <condition>\n'
                                 '%s      <property>%s</property>\n    </condition>\n  </animation>\n' % (obj, extra, state))
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- %s, written by fdm/hdrmodel.py (see there): the UH-60 model\'s halo and, in\n'
         '     the HDR pipeline, a spot light out through the lens -->\n\n<PropertyList>\n\n  <path>%s</path>\n  <nopreview/>\n\n' % (name, halo))
    x += '  <animation>\n    <type>noshadow</type>\n    <object-name>Light</object-name>\n  </animation>\n\n'
    x += '  <effect>\n    <inherits-from>Effects/model-transparent</inherits-from>\n    <object-name>Light</object-name>\n  </effect>\n\n'
    x += '  <animation>\n    <type>alpha-test</type>\n    <alpha-factor>0.01</alpha-factor>\n  </animation>\n\n'
    x += '  <animation>\n    <type>billboard</type>\n    <object-name>Light</object-name>\n    <spherical type="bool">true</spherical>\n  </animation>\n\n'
    x += ('  <animation>\n    <type>material</type>\n    <object-name>Light</object-name>\n    <emission>\n      <red> %g </red>\n'
          '      <green> %g </green>\n      <blue> %g </blue>\n    </emission>\n  </animation>\n\n' % emis)
    x += ('  <animation>\n    <type>dist-scale</type>\n    <object-name>Light</object-name>\n    <interpolation>\n' +
          ''.join('      <entry><ind> %g </ind><dep> %g </dep></entry>\n' % e for e in HALO_SCALE) + '    </interpolation>\n  </animation>\n\n')
    x += sel('Light') + '\n'
    x += ('  <light>\n    <name>%s-light</name>\n    <type>spot</type>\n    <position>\n      <x-m> %g </x-m>\n      <y-m> %g </y-m>\n'
          '      <z-m> %g </z-m>\n    </position>\n    <direction>\n      <lookat-x-m> %g </lookat-x-m>\n      <lookat-y-m> %g </lookat-y-m>\n'
          '      <lookat-z-m> %g </lookat-z-m>\n    </direction>\n    <color>\n      <r> %g </r>\n      <g> %g </g>\n      <b> %g </b>\n'
          '    </color>\n    <intensity> %.5f </intensity>      <!-- %g cd -->\n    <spot-exponent> %g </spot-exponent>\n'
          '    <spot-cutoff> %g </spot-cutoff>\n    <range-m> %g </range-m>\n  </light>\n\n'
          % ((name,) + tuple(0.04 * c for c in d) + tuple(0.04 * c + c for c in d) + tuple(rgb) + (cd / HDR_CD, cd, exp, cut, rng)))
    x += '  <animation>\n    <type>noshadow</type>\n    <object-name>%s-light</object-name>\n  </animation>\n\n' % name
    x += sel('%s-light' % name, '      <property>/sim/rendering/hdr/hdr-enabled</property>\n')
    return x + '\n</PropertyList>\n'


def interior_xml():
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- the interior lights, written by fdm/hdrmodel.py (see there): HDR spot\n'
         '     lights at their levels in %s -->\n\n<PropertyList>\n' % LT_INT)
    for name, p, a, rgb, cd, e, cut, rng in INTERIOR:
        x += ('\n  <light>\n    <name>%s-light</name>\n    <type>spot</type>\n    <position>\n      <x-m> %g </x-m>\n      <y-m> %g </y-m>\n'
              '      <z-m> %g </z-m>\n    </position>\n    <direction>\n      <lookat-x-m> %g </lookat-x-m>\n      <lookat-y-m> %g </lookat-y-m>\n'
              '      <lookat-z-m> %g </lookat-z-m>\n    </direction>\n    <color>\n      <r> %g </r>\n      <g> %g </g>\n      <b> %g </b>\n'
              '    </color>\n    <intensity>\n      <property>%s%s</property>\n      <scale> %.6g </scale>\n    </intensity>      <!-- %g cd at full -->\n'
              '    <spot-exponent> %g </spot-exponent>\n    <spot-cutoff> %g </spot-cutoff>\n    <range-m> %g </range-m>\n  </light>\n'
              % ((name,) + tuple(p) + tuple(a) + tuple(rgb) + (LT_INT, name, cd / HDR_CD, cd, e, cut, rng)))
    return x + '\n</PropertyList>\n'


def lights():
    folder = os.path.join(MODELS, 'Lights'); os.makedirs(folder, exist_ok=True)
    open(os.path.join(folder, 'interior.xml'), 'w').write(interior_xml())
    for f in ('RedLight.ac', 'GreenLight.ac', 'WhiteLight.ac', 'lights.png'):
        shutil.copy(os.path.join(UH60_LIGHTS, f), folder)
    for name, halo, pos, d, rgb, emis, cd, exp, cut, rng, state in LIGHTS:
        open(os.path.join(folder, name + '.xml'), 'w').write(light_xml(name, halo, d, rgb, emis, cd, exp, cut, rng, state))


def model_xml():
    x = ['<!--== lights: written by fdm/hdrmodel.py ==-->', '']
    for name, halo, pos, *_ in LIGHTS:
        x.append('\t<model>\n\t\t<name>light-%s</name>\n\t\t<path>Aircraft/MBB_Bo_105/Models/Lights/%s.xml</path>\n\t\t<offsets>\n'
                 '\t\t\t<x-m>%g</x-m>\n\t\t\t<y-m>%g</y-m>\n\t\t\t<z-m>%g</z-m>\n\t\t</offsets>\n\t</model>' % ((name, name) + pos))
    x.append('\t<model>\n\t\t<name>interior-lights</name>\n\t\t<path>Aircraft/MBB_Bo_105/Models/Lights/interior.xml</path>\n\t</model>')
    x.append('\n\t<!-- the old halos: not shown (their lenses always are) -->\n\t<animation>\n\t\t<type>select</type>\n' +
             ''.join('\t\t<object-name>%s</object-name>\n' % o for o in OLD_HALOS) +
             '\t\t<condition>\n\t\t\t<property>sim/model/bo105/old-lights</property>\n\t\t</condition>\n\t</animation>')
    return '\n'.join(x) + '\n\n\n'


def install():
    p = os.path.join(MODELS, 'bo105.xml'); s = open(p).read()
    for begin in ('<!--== lights: written by fdm/hdrmodel.py ==-->', '<!--== lights ==-->'):
        if begin in s:
            i = s.index(begin); j = s.index('<!--== LOD ==-->', i)
            s = s[:i] + model_xml() + s[j:]
            break
    chrome = re.search(r'\t<animation>\n\t\t<type>shader</type>\n\t\t<shader>chrome</shader>.*?</animation>\n', s, re.S)
    glass = ('\t<!-- the glass (fdm/hdrmodel.py): tinted, its alpha its opacity, through the UH-60 model\'s glass effect -->\n'
             '\t<effect>\n\t\t<inherits-from>Aircraft/MBB_Bo_105/Models/Effects/Glass/glass</inherits-from>\n' +
             ''.join('\t\t<object-name>%s</object-name>\n' % o for o in GLASS) + '\t</effect>\n')
    if chrome:
        s = s[:chrome.start()] + glass + s[chrome.end():]
    open(p, 'w').write(s)


def sgi(path):
    """an SGI image of 1 byte per channel (PIL reads no two-channel one), as RGBA"""
    import struct
    b = open(path, 'rb').read()
    rle, bpc, _, w, h, zs = struct.unpack('>xxBBHHHH', b[:12])
    assert bpc == 1
    def row(y, z):
        if not rle:
            i = 512 + (z * h + y) * w
            return list(b[i:i + w])
        i = struct.unpack('>I', b[512 + 4 * (z * h + y):516 + 4 * (z * h + y)])[0]; out = []
        while True:
            c = b[i]; i += 1; n = c & 0x7f
            if not n:
                return out
            if c & 0x80:
                out += b[i:i + n]; i += n
            else:
                out += [b[i]] * n; i += 1
    ch = [[row(y, z) for y in range(h)] for z in range(zs)]
    grey, alpha = ch[0], ch[min(zs, 4) - 1] if zs in (2, 4) else None
    im = Image.new('RGBA', (w, h))
    im.putdata([(grey[y][x], ch[1][y][x] if zs >= 3 else grey[y][x], ch[2][y][x] if zs >= 3 else grey[y][x],
                 alpha[y][x] if alpha else 255) for y in range(h - 1, -1, -1) for x in range(w)])
    return im, zs


def rgb_textures():
    """every one- or two-channel texture under Models rewritten as RGB or RGBA"""
    done = []
    for root, _, files in os.walk(MODELS):
        for f in files:
            p = os.path.join(root, f); ext = f.lower().rsplit('.', 1)[-1]
            if ext == 'png':
                im = Image.open(p)
                if im.mode in ('L', 'LA'):
                    im.convert('RGB' if im.mode == 'L' else 'RGBA').save(p); done.append(f)
            elif ext in ('rgb', 'rgba', 'sgi'):
                im, zs = sgi(p)
                if zs <= 2:
                    (im.convert('RGB') if zs == 1 else im).save(p, format='SGI'); done.append(f)
    return done


def linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def variants():
    """each variant's HDR paint (material/fuselage/pbr) and blade paint (material/rotor/blade, blur); [(file, rgb, roughness)]"""
    out = []
    for f in sorted(os.listdir(VARIANTS)):
        if not f.endswith('.xml'):
            continue
        p = os.path.join(VARIANTS, f); s = open(p).read()
        s = re.sub(r'\n\t*<!-- the HDR pipeline\'s paint.*?</pbr>\n', '', s, flags=re.S)
        s = re.sub(r'\t*<!-- the blades\' paint.*?</blur>\n', '', s, flags=re.S)
        rough = MATTE if f.startswith('Mil') else GLOSS
        for part in ('fuselage', 'helmet', 'flightsuit'):          # the pilots' too: fdm/materials.py
            blk = re.search(r'(?s)(\t*)<%s>.*?</%s>' % (part, part), s)
            diffuse = re.search(r'(?s)<diffuse>(.*?)</diffuse>', blk.group(0)).group(1)
            c = [linear(float(re.search(r'<%s>\s*([\d.]+)\s*</%s>' % (k, k), diffuse).group(1))) for k in ('red', 'green', 'blue')]
            t = blk.group(1) + '\t'
            pbr = ('\n%s<!-- the HDR pipeline\'s paint (fdm/hdrmodel.py): the diffuse colour, linear%s -->\n%s<pbr>\n'
                   % (t, '; its roughness' if part == 'fuselage' else '', t) +
                   ''.join('%s\t<%s type="float">%.4f</%s>\n' % (t, k, v, k) for k, v in zip(('red', 'green', 'blue'), c)) +
                   ('%s\t<roughness type="float">%.2f</roughness>\n' % (t, rough) if part == 'fuselage' else '') + '%s</pbr>\n' % t)
            i = s.index('</%s>' % part, blk.start()); i = s.rindex('\n', 0, i) + 1
            s = s[:i] + pbr + s[i:]
            if part == 'fuselage':
                rgb = c
        rotor = re.search(r'(?s)<rotor>\s*<texture>Textures/Rotor/(\w+)\.png</texture>\n', s)
        paint = rotor.group(1); t = re.search(r'\n(\t*)<rotor>', s).group(1) + '\t'
        s = s[:rotor.end()] + ('%s<!-- the blades\' paint (Models/Rotor, fdm/rotorvis.py) -->\n%s<blade>%s.png</blade>\n%s<blur>blur-%s.png</blur>\n'
                               % (t, t, paint, t, paint)) + s[rotor.end():]
        open(p, 'w').write(s)
        out.append((f, rgb, rough))
    return out


def livery_xml(objs):
    obj = ''.join('\t\t<object-name>%s</object-name>\n' % o for o in objs)
    return (LIVERY_BEGIN + '\n\t<!-- the material animation below doesn\'t reach it: the PBR shader, its base colour and roughness the\n'
            '\t     livery\'s (Models/Variants: material/fuselage/pbr) through the pbr animation (first) -->\n'
            '\t<animation>\n\t\t<type>pbr</type>\n' + obj + '\t\t<property-base>%s/pbr</property-base>\n' % LIVERY +
            '\t\t<base-color-factor>\n\t\t\t<red>red</red>\n\t\t\t<green>green</green>\n\t\t\t<blue>blue</blue>\n\t\t\t<alpha>1</alpha>\n'
            '\t\t</base-color-factor>\n\t\t<roughness-factor>roughness</roughness-factor>\n\t</animation>\n'
            '\t<effect>\n\t\t<inherits-from>Effects/model-pbr</inherits-from>\n\t\t<parameters>\n'
            '\t\t\t<metallic-factor type="float">0.0</metallic-factor>\n\t\t\t<roughness-factor type="float">%g</roughness-factor>\n'
            '\t\t</parameters>\n' % GLOSS + obj + '\t</effect>\n' + LIVERY_END + '\n\n')


def install_livery():
    """livery_xml on the objects of the fuselage's material animation, before it"""
    p = os.path.join(MODELS, 'bo105.xml'); s = open(p).read()
    if LIVERY_BEGIN in s:
        s = s[:s.index(LIVERY_BEGIN)] + s[s.index(LIVERY_END) + len(LIVERY_END) + 2:]
    m = re.search(r'\t<animation>\n\t\t<type>material</type>\n((?:\t\t<object-name>[^<]+</object-name>\n)+)'
                  r'\t\t<property-base>%s</property-base>' % LIVERY, s)
    objs = re.findall(r'<object-name>([^<]+)</object-name>', m.group(1))
    s = s[:m.start()] + livery_xml(objs) + s[m.start():]
    open(p, 'w').write(s)
    return objs


if __name__ == '__main__':
    glass_ac(); lights(); install()
    print('glass textured on %d objects; %d lights' % (len(GLASS), len(LIGHTS)))
    print('grey textures made RGB(A): %s' % (', '.join(rgb_textures()) or 'none'))
    print('livery on %d objects' % len(install_livery()))
    for f, rgb, rough in variants():
        print('  %-22s base colour %.3f %.3f %.3f, roughness %.2f' % ((f,) + tuple(rgb) + (rough,)))
