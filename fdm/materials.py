#!/usr/bin/env python3
"""The 3-D model's untextured parts in FlightGear's HDR pipeline: the cabin, the flight controls, the pilots, the rotor
heads and the tail rotor (textured and PBR, as the UH-60 model's parts are).

bo105.ac paints them in flat material colours, and the HDR pipeline takes a material's colour as linear: the "black"
console (0.15) showed mid grey, the whole cabin washed out and flat. Here:

- each part gets a fine tileable detail texture of its kind (Models/Textures/Detail, KINDS: crinkle paint, paint,
  vinyl trim, ribbed floor mat, seat fabric, Nomex twill, leather, skin, rubber, brushed and cast metal), white with
  darker detail so that the other pipelines' look (texture x material) barely changes, mapped by a planar projection
  along each face's main axis, a repeat every TILE m;
- and the PBR shader (Effects/model-pbr): its base colour the part's material's (its faces' most), made linear (sRGB,
  as the old lighting showed it) over the texture's mean, its kind's roughness and metalness. The flight suits and
  helmets take the livery's colour (Models/Variants: material/flightsuit/pbr, helmet/pbr, fdm/hdrmodel.py) through
  the pbr animation.

The rotor parts that fdm/rotorvis.py takes out of bo105.ac (the blade fittings, forks, horns, absorbers and pendulums,
the tail blade) carry their texture with them; its blade XMLs take their effects from effects_xml here.

    python3 materials.py     edits Models/bo105.ac (textures, uv) and Models/bo105.xml; writes Models/Textures/Detail
                             (then rotorvis.py, hdrmodel.py)
"""
import os, re
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'MBB_Bo_105', 'Models')
DETAIL = os.path.join(MODELS, 'Textures', 'Detail')
N = 512                                 # texture size (px)

# kind -> repeat (m), roughness, metalness
KINDS = {'crinkle': (0.12, 0.80, 0.0), 'paint': (0.30, 0.45, 0.0), 'trim': (0.15, 0.70, 0.0), 'floor': (0.25, 0.85, 0.0),
         'fabric': (0.06, 0.90, 0.0), 'twill': (0.05, 0.95, 0.0), 'leather': (0.05, 0.60, 0.0), 'skin': (0.08, 0.55, 0.0),
         'rubber': (0.05, 0.85, 0.0), 'brushed': (0.25, 0.35, 1.0), 'cast': (0.15, 0.45, 0.8)}
# bo105.ac's materials -> the kind of their parts (the lights' lenses, the glass, the visor and the livery's own
# yellow parts keep theirs); OBJECT_KIND by part
MATERIAL_KIND = {'black': 'paint', 'aluminum': 'brushed', 'grey': 'trim', 'grey_seat': 'fabric', 'flightsuit': 'twill',
                 'helmet': 'paint', 'human_skin': 'skin', 'shoes': 'leather', 'glossy_grey': 'paint', 'mil_metal_fade': 'cast',
                 'mil_metal_nofade': 'cast', 'rubber.001': 'rubber', 'earphone.001': 'paint', 'exhaust': 'cast',
                 'dull_black': 'rubber', 'tail_rotor': 'paint'}
OBJECT_KIND = dict([(o, 'crinkle') for o in ('cockpit', 'shield', 'Hspider_i', 'compass_mounting', 'sock_back_L',
                                             'sock_back_M', 'sock_back_R', 'sock_front_L', 'sock_front_R')] +
                   [(o, 'leather') for o in ('stick_grip_L', 'stick_grip_R', 'pilot_hand_L', 'pilot_hand_R',
                                             'copilot_hand_L', 'copilot_hand_R')] +
                   [('floor', 'floor'), ('swashplate_cap', 'cast')])
# drawn, and casting shadows, from both sides: the console and glareshield are open shells, which the sun ahead sees
# from behind (the shadow pass leaves out back faces), so that they shaded neither the glareshield's underside nor the panel
TWO_SIDED = {'crinkle'}
# the livery's: material -> its property base (Models/Variants), the objects its material animation lists
LIVERY = {'flightsuit': 'sim/model/bo105/material/flightsuit', 'helmet': 'sim/model/bo105/material/helmet'}
BEGIN, END = '\t<!-- the untextured parts in the HDR pipeline: written by fdm/materials.py -->', '\t<!-- end of fdm/materials.py\'s parts -->'
OLD_CONSOLE = re.compile(r'\t<!-- the console and glareshield matte black.*?</effect>\n\n', re.S)


def linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def noise(rng, sx, sy=None):
    """periodic gaussian noise, smoothed sx (along x) by sy px, zero mean, unit deviation"""
    sy = sx if sy is None else sy
    k = np.fft.fftfreq(N)
    g = np.exp(-2 * np.pi ** 2 * ((sx * k[None, :]) ** 2 + (sy * k[:, None]) ** 2))
    n = np.real(np.fft.ifft2(np.fft.fft2(rng.standard_normal((N, N))) * g))
    return (n - n.mean()) / n.std()


def pattern(kind):
    """the kind's detail: 0 (the base) and below, about unit depth"""
    rng = np.random.default_rng(sum(map(ord, kind)))
    y, x = np.mgrid[0:N, 0:N].astype(float)
    if kind == 'paint':                                     # orange peel
        p = 0.7 * noise(rng, 2) + 0.3 * noise(rng, 12)
    elif kind == 'crinkle':                                 # wrinkle finish
        p = -np.abs(noise(rng, 1.5) - 0.8 * noise(rng, 4)) + 0.2 * noise(rng, 20)
    elif kind in ('trim', 'leather'):                       # grain
        s = 1.0 if kind == 'trim' else 1.6
        p = -np.abs(noise(rng, s) - 0.7 * noise(rng, 2.5 * s)) + 0.25 * noise(rng, 15)
    elif kind == 'floor':                                   # ribbed rubber mat
        p = -np.abs(np.sin(np.pi * x / 16)) ** 6 * 2 + 0.3 * noise(rng, 1.5)
    elif kind == 'fabric':                                  # plain weave
        p = np.sin(2 * np.pi * x / 8) * np.sin(2 * np.pi * y / 8) + 0.35 * noise(rng, 0.8) + 0.3 * noise(rng, 25)
    elif kind == 'twill':                                   # diagonal twill
        p = np.sin(2 * np.pi * (x + y) / 6) + 0.4 * noise(rng, 0.8) + 0.4 * noise(rng, 30)
    elif kind == 'skin':
        p = 0.6 * noise(rng, 3) + 0.4 * noise(rng, 15)
    elif kind == 'rubber':
        p = 0.5 * noise(rng, 1) + 0.5 * noise(rng, 6)
    elif kind == 'brushed':                                 # streaks along x, scratches
        p = noise(rng, 40, 0.7) + 0.3 * noise(rng, 8)
    elif kind == 'cast':
        p = 0.5 * noise(rng, 1.5) + 0.3 * noise(rng, 6) + 0.4 * noise(rng, 25)
    return (p - p.max()) / p.std()


DEPTH = {'paint': 5, 'crinkle': 14, 'trim': 9, 'leather': 14, 'floor': 18, 'fabric': 16, 'twill': 12, 'skin': 6,
         'rubber': 7, 'brushed': 12, 'cast': 10}       # the detail's deviation, of 255


def textures():
    """Models/Textures/Detail/<kind>.png"""
    os.makedirs(DETAIL, exist_ok=True)
    for kind in KINDS:
        v = np.clip(255 + DEPTH[kind] * pattern(kind), 0, 255)
        v = np.clip(v + (252 - np.percentile(v, 90)), 0, 255)       # the brightest tenth white
        Image.fromarray(np.repeat(v.round().astype(np.uint8)[:, :, None], 3, axis=2), 'RGB').save(os.path.join(DETAIL, kind + '.png'))


def means():
    """{kind: its texture's mean, linear}"""
    out = {}
    for kind in KINDS:
        v = np.asarray(Image.open(os.path.join(DETAIL, kind + '.png')).convert('L'), float) / 255
        out[kind] = float(np.where(v <= 0.04045, v / 12.92, ((v + 0.055) / 1.055) ** 2.4).mean())
    return out


def materials(head):
    return [(m.group(1), [float(c) for c in m.group(2).split()]) for m in
            re.finditer(r'^MATERIAL "([^"]*)" rgb ([-\d.e]+ [-\d.e]+ [-\d.e]+)', head, re.M)]


def uv(chunk, tile):
    """chunk's surfaces mapped by a planar projection along each face's main axis, a repeat every tile m"""
    L = chunk.split('\n')
    i = next(k for k, l in enumerate(L) if l.startswith('numvert')); n = int(L[i].split()[1])
    V = np.array([[float(c) for c in L[i + 1 + k].split()[:3]] for k in range(n)])
    k = i + n + 1
    while k < len(L):
        if L[k].startswith('refs'):
            m = int(L[k].split()[1]); rows = range(k + 1, k + 1 + m)
            idx = [int(L[r].split()[0]) for r in rows]
            P = V[idx]
            nrm = np.zeros(3)
            for a in range(1, len(P) - 1):
                nrm += np.cross(P[a] - P[0], P[a + 1] - P[0])
            ax = int(np.argmax(np.abs(nrm))); u, w = [c for c in range(3) if c != ax]
            for r, j in zip(rows, idx):
                L[r] = '%d %.5f %.5f' % (j, V[j][u] / tile, V[j][w] / tile)
            k += m + 1
        else:
            k += 1
    return '\n'.join(L)


def parts():
    """bo105.ac's untextured parts of a kind given a texture and uv (once); [(name, kind, material rgb)] of all"""
    p = os.path.join(MODELS, 'bo105.ac'); s = open(p, encoding='latin-1').read()
    head, *chunks = re.split(r'(?m)^(?=OBJECT )', s)
    mats = materials(head)
    out, found = [head], []
    for c in chunks:
        name = re.search(r'^name "([^"]+)"', c, re.M)
        faces = [int(m) for m in re.findall(r'^mat (\d+)', c, re.M)]
        if name and faces:
            name = name.group(1)
            mat = max(set(faces), key=faces.count)
            tex = re.search(r'^texture "Textures/Detail/(\w+)\.png"', c, re.M)
            kind = tex.group(1) if tex else (None if re.search(r'^texture ', c, re.M) else
                                             OBJECT_KIND.get(name, MATERIAL_KIND.get(mats[mat][0])))
            if kind:
                if not tex:
                    j = c.index('\nnumvert') + 1
                    c = uv(c[:j] + 'texture "Textures/Detail/%s.png"\ntexrep 1 1\n' % kind + c[j:], KINDS[kind][0])
                found.append((name, kind, mats[mat][0], mats[mat][1]))
        out.append(c)
    out = ''.join(out)
    if out != s:
        open(p, 'w', encoding='latin-1').write(out)
    return found


def effects_xml(found, mean, indent='\t'):
    """the PBR effects on found's parts, one per kind and colour; the livery's parts' colour through the pbr animation
    (indent: the file's step)"""
    groups = {}
    for name, kind, mat, rgb in found:
        groups.setdefault((kind, mat, tuple(rgb)), []).append(name)
    a, b, c = indent, indent * 2, indent * 3; x = ''
    for (kind, mat, rgb), names in sorted(groups.items()):
        tile, rough, metal = KINDS[kind]
        obj = ''.join('%s<object-name>%s</object-name>\n' % (b, o) for o in names)
        if mat in LIVERY:
            x += ('%s<animation>\n%s<type>pbr</type>\n' % (a, b) + obj + '%s<property-base>%s/pbr</property-base>\n%s<base-color-factor>\n' % (b, LIVERY[mat], b) +
                  ''.join('%s<%s><property>%s</property><scale>%.4f</scale></%s>\n' % (c, k, k, 1 / mean[kind], k) for k in ('red', 'green', 'blue')) +
                  '%s<alpha>1</alpha>\n%s</base-color-factor>\n%s</animation>\n' % (c, b, a))
        f = [linear(v) / mean[kind] for v in rgb]
        x += ('%s<effect>\n%s<inherits-from>Effects/model-pbr</inherits-from>\n%s<parameters>\n' % (a, b, b) +
              '%s<base-color-factor type="vec4d">%.4f %.4f %.4f 1.0</base-color-factor>      <!-- %s, %s -->\n' % (c, f[0], f[1], f[2], mat, kind) +
              '%s<metallic-factor type="float">%g</metallic-factor>\n%s<roughness-factor type="float">%g</roughness-factor>\n'
              % (c, metal, c, rough) + ('%s<cull-face>none</cull-face>\n' % c if kind in TWO_SIDED else '') +
              '%s</parameters>\n' % b + obj + '%s</effect>\n' % a)
    return x


def install(found, mean):
    p = os.path.join(MODELS, 'bo105.xml'); s = open(p).read()
    s = OLD_CONSOLE.sub('', s)
    block = (BEGIN + '\n\t<!-- each its kind\'s detail texture (Textures/Detail) and the PBR shader, its material\'s colour made linear; the\n'
             '\t     pilots\' suits and helmets the livery\'s (the pbr animation first) -->\n' + effects_xml(found, mean) + END + '\n')
    if BEGIN in s:
        s = s[:s.index(BEGIN)] + block + s[s.index(END) + len(END) + 1:]
    else:
        i = s.index('<!--== material ==-->')
        s = s[:i] + block + '\n\n' + s[i:]
    open(p, 'w').write(s)


if __name__ == '__main__':
    textures(); mean = means()
    found = parts()
    install(found, mean)
    kinds = {}
    for name, kind, mat, rgb in found:
        kinds.setdefault(kind, []).append(name)
    for kind, names in sorted(kinds.items()):
        print('%-8s mean %.2f  %2d parts: %s' % (kind, mean[kind], len(names), ' '.join(names)[:110]))
