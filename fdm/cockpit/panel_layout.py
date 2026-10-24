"""The Bo 105 CB-5/CBS-5 instrument panel measured from its flight manual's figure 7-3 ("Typical Instrument Panel"),
for the cockpit tools (gauges.py, panel.py).

The figure is a scanned drawing, 1-bit, 1501 x 1615 px on page 206 of the ManualsLib copy of the manual (refs/):
    pdfimages -png -f 206 -l 206 refs/BO105-FM/BO105-FM.pdf refs/BO105-FM/hi/fig7-3
Round items are placed by circles fitted to their bezel rings (the circle through the most drawn pixels near a first
guess); the panel's outline by its border lines (their pixel columns and rows); the rest are boxes read off the
drawing.

Scale: 3.25 in (82.5 mm) per 3ATI bezel ring, the airspeed indicator's, the altimeter's and the triple oil gauges'
(225.5 px mean): the panel is 251 mm wide and 503 mm tall. The other round instruments are drawn without their
bezels (vertical speed indicator, triple tachometer) or are 2 1/4 in ones (2ATI).

Frame (the panel's, panel.xml's): y right, z up, metres, from the middle of the panel's outline, in the plane of the
figure, which is the face.

    python3 panel_layout.py        writes panel-layout.md and panel-layout.png
"""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, '..', '..', 'refs', 'BO105-FM', 'hi', 'fig7-3-000.png')

# the outline: the border lines' middles, px
X0, X1, Y0, Y1 = 503.5, 1190.0, 233.5, 1607.5
BEZEL_3ATI = 0.0825
PX = 225.5 / BEZEL_3ATI                 # px per m
W, H = (X1 - X0) / PX, (Y1 - Y0) / PX

# round items: (legend no., name, figure's guess: centre x, y and radius range, px; case: 3ATI, 2ATI or None)
ROUND = [(17, 'Airspeed indicator', (660, 478, 95, 125), '3ATI'),
         (3, 'Altimeter', (1045, 485, 105, 122), '3ATI'),
         (4, 'Vertical speed indicator', (1035, 715, 88, 118), '3ATI'),
         (5, 'Triple tachometer', (1045, 950, 98, 120), '3ATI'),
         (12, 'Triple oil temperature indicator', (657, 1225, 95, 125), '3ATI'),
         (11, 'Triple oil pressure indicator', (657, 1475, 95, 125), '3ATI'),
         (6, 'Dual torque indicator', (843, 985, 62, 88), '2ATI'),
         (14, 'Clock', (845, 770, 55, 80), '2ATI'),
         (7, 'TOT indicator, engine 1', (873, 1190, 58, 82), '2ATI'),
         (7, 'TOT indicator, engine 2', (1063, 1190, 58, 82), '2ATI'),
         (18, 'Fire warning light, engine 1', (700, 300, 32, 50), None),
         (18, 'Fire warning light, engine 2', (992, 300, 32, 50), None)]
# the N1, fuel quantity and fuel pressure indicators' rings are broken by their markings: centres and radii read off
ROUND_READ = [(8, 'N1 indicator, engine 1', (876, 1354, 66), '2ATI'),
              (8, 'N1 indicator, engine 2', (1063, 1353, 66), '2ATI'),
              (10, 'Fuel quantity indicator', (877, 1508, 62), '2ATI'),
              (9, 'Fuel pressure indicator', (1068, 1508, 62), '2ATI')]
# boxes: (legend no., name, x0, x1, y0, y1 px)
BOXES = [(16, 'Warning light segments', 783, 912, 270, 602),
         (15, 'Instrument light rheostat', 788, 848, 604, 690),
         (15, 'Annunciator lamp test pushbutton', 850, 910, 604, 690),
         (13, 'Mast moment indicator', 540, 762, 858, 1072),
         (18, 'TEST FIRE pushbutton, engine 1', 590, 620, 285, 315),
         (18, 'TEST FIRE pushbutton, engine 2', 1061, 1091, 285, 315),
         (2, 'Pitot heat indicator', 1122, 1158, 294, 330)]
# lettering on the face plate: (text, centre x, y px, letter height px)
LABELS = [('TEST', 610, 266, 13), ('FIRE', 610, 281, 13), ('TEST', 1082, 266, 13), ('FIRE', 1082, 281, 13),
          ('PITOT', 1143, 266, 13), ('HEATING', 1143, 281, 13),
          # the figure's ENG 1 and ENG 2 sit beside the TOT indicators' round bodies, under their square cases here: below them
          ('ENG 1', 877, 1274, 12), ('ENG 2', 1068, 1274, 12)]


def to_panel(x, y):
    """figure px -> panel y, z (m)"""
    return (x - (X0 + X1) / 2) / PX, ((Y0 + Y1) / 2 - y) / PX


def image():
    from PIL import Image
    return np.array(Image.open(FIG).convert('L')) > 128


def fit_circle(im, cx0, cy0, r0, r1, d=12):
    """the circle (cx, cy, r) through the most drawn pixels, centre within d px of the guess, radius r0..r1"""
    from scipy import ndimage
    im = ndimage.binary_dilation(im, iterations=1)
    th = np.linspace(0, 2 * np.pi, 720, endpoint=False)
    c, s = np.cos(th), np.sin(th)
    best = (-1, 0, 0, 0)
    for r in np.arange(r0, r1, 0.5):
        for cx in range(cx0 - d, cx0 + d + 1):
            ys = np.clip(np.round(cy0 - d + r * s).astype(int), 0, im.shape[0] - 1)
            xs = np.clip(np.round(cx + r * c).astype(int), 0, im.shape[1] - 1)
            for k in range(2 * d + 1):
                f = im[np.clip(ys + k, 0, im.shape[0] - 1), xs].mean()
                if f > best[0]:
                    best = (f, cx, cy0 - d + k, r)
    return best[1:]


def items(im=None):
    """every item: num, name, case, y, z (centre, m), w, h (m; a round item's bezel diameter)"""
    im = image() if im is None else im
    out = []
    for num, name, (cx, cy, r0, r1), case in ROUND:
        x, y, r = fit_circle(im, cx, cy, r0, r1)
        py, pz = to_panel(x, y)
        out.append(dict(num=num, name=name, case=case, y=py, z=pz, w=2 * r / PX, h=2 * r / PX, round=True))
    for num, name, (x, y, r), case in ROUND_READ:
        py, pz = to_panel(x, y)
        out.append(dict(num=num, name=name, case=case, y=py, z=pz, w=2 * r / PX, h=2 * r / PX, round=True))
    for num, name, x0, x1, y0, y1 in BOXES:
        (ya, za), (yb, zb) = to_panel(x0, y1), to_panel(x1, y0)
        out.append(dict(num=num, name=name, case=None, y=(ya + yb) / 2, z=(za + zb) / 2, w=yb - ya, h=zb - za, round=False))
    return out


def table(its):
    rows = ['| No. | Item | Case | Size, mm | Centre y, m | Centre z, m |', '|---|---|---|---|---|---|']
    for it in sorted(its, key=lambda it: (-round(it['z'], 2), it['y'])):
        size = ('%.0f dia.' % (it['w'] * 1e3)) if it['round'] else ('%.0f x %.0f' % (it['w'] * 1e3, it['h'] * 1e3))
        rows.append('| %d | %s | %s | %s | %+.3f | %+.3f |' % (it['num'], it['name'], it['case'] or '-', size, it['y'], it['z']))
    return '\n'.join(rows)


def figure(its, im, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    (ya, za), (yb, zb) = to_panel(0, im.shape[0]), to_panel(im.shape[1], 0)
    fig, ax = plt.subplots(figsize=(6.5, 11))
    ax.imshow(~im, cmap='gray', extent=(ya, yb, za, zb), alpha=0.5)
    ax.add_patch(Rectangle((-W / 2, -H / 2), W, H, fill=False, ec='k', lw=1.2))
    for it in its:
        if it['round']:
            ax.add_patch(Circle((it['y'], it['z']), it['w'] / 2, fill=False, ec='tab:red', lw=1.2))
        else:
            ax.add_patch(Rectangle((it['y'] - it['w'] / 2, it['z'] - it['h'] / 2), it['w'], it['h'], fill=False, ec='tab:blue', lw=1.2))
        ax.text(it['y'], it['z'], str(it['num']), color='tab:red', ha='center', va='center', fontsize=9, fontweight='bold')
    ax.set_xlim(-W / 2 - 0.01, W / 2 + 0.01); ax.set_ylim(-H / 2 - 0.01, H / 2 + 0.01); ax.set_aspect('equal')
    ax.set_xlabel('y, m'); ax.set_ylabel('z, m')
    ax.set_title('Bo 105 instrument panel, flight manual fig. 7-3:\nfitted bezels (red), boxes (blue), legend numbers', fontsize=9)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


if __name__ == '__main__':
    im = image()
    its = items(im)
    md = table(its)
    open(os.path.join(HERE, 'panel-layout.md'), 'w').write(
        '# Bo 105 instrument panel layout\n\nMeasured from the flight manual\'s figure 7-3 by `panel_layout.py`: the panel '
        'is %.0f x %.0f mm; y, z in its frame (m, from the middle of its outline), round items\' sizes their bezels\' '
        'diameters.\n\n%s\n' % (W * 1e3, H * 1e3, md))
    figure(its, im, os.path.join(HERE, 'panel-layout.png'))
    print('panel %.1f x %.1f mm' % (W * 1e3, H * 1e3))
    print(md)
