#!/usr/bin/env python3
"""Run the validation programme of the report into report/results/*.json (the figures, tables and numbers are made
from these by mktables.py and figures.py). Regenerate the harness's model first:

    cd fdm && python3 genbo105.py root/aircraft/bo105 && python3 genbo105.py root-bailey/aircraft/bo105 --tr bailey
    python3 genbo105.py root-nofin/aircraft/bo105 --no-tr-fin
    python3 report/mkresults.py [stage ...]      all stages in parallel, or the ones named

Stages: trims_dlr, trims_dlr_bailey (Bailey's tail rotor), trims_dlr_nofin (no fin blockage), trims_mbb (sweep.py),
derivs (derivatives.py), freqresp (freqresp.py), doublet0-3 (doublets.py),
engine (engine_response.py), lagdamp (lag_damping.py), hover (hover_rotor.py), modes (blademodes.py)."""
import os, sys, json, math, time
from multiprocessing import Pool
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
OUT = os.path.join(HERE, 'results')


def plain(x):
    if isinstance(x, dict):
        return {str(k): plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return plain(x.tolist())
    if isinstance(x, complex) or isinstance(x, np.complexfloating):
        return [float(x.real), float(x.imag)]
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return x


def dump(name, obj):
    json.dump(plain(obj), open(os.path.join(OUT, name + '.json'), 'w'), indent=1)


def trims_dlr():
    import sweep
    dump('trims_dlr', sweep.run('bo105', [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120], 0.0))


def trims_dlr_bailey():
    """the same with Bailey's closed-form tail rotor (genbo105.py root-bailey/aircraft/bo105 --tr bailey)"""
    os.environ['BO105_ROOT'] = os.path.join(FDM, 'root-bailey')
    import sweep
    dump('trims_dlr_bailey', sweep.run('bo105', [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120], 0.0))


def trims_dlr_nofin():
    """the same without the fin blockage (genbo105.py root-nofin/aircraft/bo105 --no-tr-fin)"""
    os.environ['BO105_ROOT'] = os.path.join(FDM, 'root-nofin')
    import sweep
    dump('trims_dlr_nofin', sweep.run('bo105', [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120], 0.0))


def trims_mbb():
    import sweep
    from data import flight_trims as FT
    dump('trims_mbb', sweep.run('bo105-mbb', [0, 20, 33, 44, 55, 66, 78, 88, 98, 108, 118], FT.MBB_DENSITY_ALT))


def derivs():
    import derivatives as DV
    D, trim = DV.model_derivatives()
    ev = DV.modes(D, trim)
    trim = {k: (v if k != 'x' else list(v)) for k, v in trim.items()}
    dump('derivs', dict(D=D, trim=trim, eig=[complex(e) for e in ev], modes=DV.describe(ev)))


def freqresp():
    import freqresp as FR
    t, d, p, res = FR.sweep()
    w = np.logspace(0, math.log10(25.0), 40)
    dump('freqresp', dict(w=w, H=[complex(h) for h in FR.response(t, d, p, w)], res=res))


def doublet(k):
    import doublets as DB
    k, rec, res = DB.simulate(k)
    dump('doublet%d' % k, dict(k=k, rec=rec, res=res))


def engine():
    import engine_response as ER
    m, res, x = ER.run()
    dump('engine', dict(m=m, res=res, x=list(x), table=ER.table(m)))


def lagdamp():
    import harness as H, lag_damping as LD
    f = H.load()
    f['ctl/init'] = 1
    H.freeze(f)
    rows = []
    for coll in (0.30, 0.40, 0.50, 0.55, 0.62, 0.70):
        cts, s, nu, rms = LD.decay(f, coll)
        rows.append(dict(coll=coll, ct_sigma=cts, decay=s, nu=nu, rms=rms))
    dump('lagdamp', dict(rows=rows, test=LD.TM85892_SIGMA))


def hover():
    import harness as H, hover_rotor as HR
    from data import nasa_hover as NH
    f = H.load()
    H.freeze(f)
    f['drive/hold-rpm'] = 1
    rows = []
    for th, ct_t, cp_t in NH.HOVER:
        cts, cps, mt, a1, b1 = HR.point(f, th)
        rows.append(dict(th07=th, ct=cts, cp=cps, torsion=mt, a1=a1, b1=b1, ct_test=ct_t, cp_test=cp_t))
    dump('hover', dict(rows=rows, sigma=NH.SIGMA, torsion_test=NH.TORSION_034))


def modes():
    import blademodes as BM
    import bo105data as U
    om = 425 * math.pi / 30
    (wf, _), (wl, _), (wt, _) = BM.frequencies(om, 1.0)
    md = BM.modes()
    rr = np.linspace(0, U.R, 60)
    shapes = {k: [BM.phi(md, k, r)[0] for r in rr] for k in ('f', 'l')}
    dump('modes', dict(table4=dict(flap=wf / om, lag=wl / om, torsion=wt / om), model=md['freq'], lag_root_scale=md['lag_root_scale'],
                       y71=U.FREQ_Y71, pt=U.FREQ_PT, r=rr, shapes=shapes, R=U.R))


STAGES = dict(trims_dlr=trims_dlr, trims_dlr_bailey=trims_dlr_bailey, trims_dlr_nofin=trims_dlr_nofin, trims_mbb=trims_mbb, derivs=derivs, freqresp=freqresp, engine=engine, lagdamp=lagdamp,
              hover=hover, modes=modes, **{'doublet%d' % k: (lambda k=k: doublet(k)) for k in range(4)})


def run(name):
    t0 = time.time()
    try:
        STAGES[name]()
        return name, time.time() - t0, None
    except Exception as e:                       # one stage's failure leaves the others' results
        import traceback
        return name, time.time() - t0, traceback.format_exc()


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    names = sys.argv[1:] or ['trims_dlr', 'trims_dlr_bailey', 'trims_dlr_nofin', 'trims_mbb', 'derivs', 'doublet0', 'doublet1', 'doublet2', 'doublet3', 'engine',
                             'freqresp', 'hover', 'lagdamp', 'modes']
    with Pool(min(len(names), os.cpu_count() or 4)) as pool:
        for name, dt, err in pool.imap_unordered(run, names):
            print('%-10s %6.0f s %s' % (name, dt, 'FAILED\n' + err if err else 'ok'), flush=True)
