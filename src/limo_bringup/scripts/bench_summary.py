#!/usr/bin/env python3
"""Bir bench_many.sh kökündeki (r1..rN) sonuçları özetler; JSON da yazar.

Kullanım: bench_summary.py <kok> [--json]
Skor (büyük = iyi), sıralama için:
  1000 * başarı_oranı - 5 * ort_kurtarma - 1 * ort_planlayıcı_hatası
  - 200 * (min_açıklık < 0.05 olan koşu oranı) - 0.2 * ort_toplam_süre
"""

import glob
import json
import os
import statistics as st
import sys


def load(root):
    runs = []
    for d in sorted(glob.glob(os.path.join(root, 'r[0-9]*'))):
        if '.' in os.path.basename(d):
            continue
        p = os.path.join(d, 'result.json')
        status = open(os.path.join(d, 'status')).read().strip() if os.path.exists(
            os.path.join(d, 'status')) else 'NO_STATUS'
        if os.path.exists(p):
            r = json.load(open(p))
            r['run'] = os.path.basename(d)
            r['status'] = status
            runs.append(r)
        else:
            runs.append({'run': os.path.basename(d), 'status': status, 'summary': None, 'goals': []})
    return runs


def summarize(root):
    runs = load(root)
    ok = [r for r in runs if r.get('summary')]
    n_goals = sum(r['summary']['total'] for r in ok)
    n_succ = sum(r['summary']['succeeded'] for r in ok)
    per_goal = {}
    for r in ok:
        for g in r['goals']:
            per_goal.setdefault(g['goal'], []).append(g)
    goals = {k: {'success': f"{sum(x['status'] == 'SUCCEEDED' for x in v)}/{len(v)}",
                 'mean_err': round(st.mean(x['pos_err_m'] for x in v), 3),
                 'mean_rec': round(st.mean(x['recoveries'] for x in v), 1)}
             for k, v in per_goal.items()}

    def m(key):
        vals = [r['summary'][key] for r in ok if r['summary'].get(key) is not None]
        return round(st.mean(vals), 3) if vals else None

    close = [r for r in ok if r['summary'].get('min_clear_m') is not None and r['summary']['min_clear_m'] < 0.05]
    out = {
        'root': root,
        'runs': len(runs), 'runs_ok': len(ok),
        'bringup_failed': sum(r['status'].startswith('BRINGUP') for r in runs),
        'success_rate': round(n_succ / n_goals, 3) if n_goals else 0.0,
        'succeeded': n_succ, 'goals_total': n_goals,
        'mean_recoveries': m('total_recoveries'),
        'mean_planner_fail': m('total_planner_fail'),
        'mean_total_time_s': m('total_sim_time_s'),
        'mean_pos_err_m': m('mean_pos_err_m'),
        'worst_min_clear_m': min((r['summary']['min_clear_m'] for r in ok
                                  if r['summary'].get('min_clear_m') is not None), default=None),
        'close_call_runs': len(close),
        'rtf': m('real_time_factor'),
        'per_goal': goals,
    }
    if ok:
        out['score'] = round(1000 * out['success_rate'] - 5 * (out['mean_recoveries'] or 0)
                             - (out['mean_planner_fail'] or 0) - 200 * len(close) / len(ok)
                             - 0.2 * (out['mean_total_time_s'] or 0), 1)
    else:
        out['score'] = None
    return out


def main():
    root = sys.argv[1]
    s = summarize(root)
    with open(os.path.join(root, 'summary.json'), 'w') as f:
        json.dump(s, f, indent=2, ensure_ascii=False)
    if '--json' in sys.argv:
        print(json.dumps(s, ensure_ascii=False))
        return
    print(f"{root}: basari {s['succeeded']}/{s['goals_total']} ({s['success_rate']:.0%}), "
          f"kosu {s['runs_ok']}/{s['runs']} (bringup hata {s['bringup_failed']}), "
          f"ort. kurtarma {s['mean_recoveries']}, ort. plan hatasi {s['mean_planner_fail']}, "
          f"ort. sure {s['mean_total_time_s']} s, ort. hata {s['mean_pos_err_m']} m, "
          f"en kotu aciklik {s['worst_min_clear_m']} m ({s['close_call_runs']} kosuda <5cm), "
          f"RTF {s['rtf']}, SKOR {s['score']}")
    for g, v in s['per_goal'].items():
        print(f"   {g:22s} {v['success']:>5s}  ort.hata {v['mean_err']:.3f}  ort.kurtarma {v['mean_rec']}")


if __name__ == '__main__':
    main()
