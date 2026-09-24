#!/usr/bin/env python3
"""Bir Nav2 parametre dosyasından, noktalı yol değişiklikleriyle varyant üretir.

Kullanım:
  make_variant.py <temel.yaml> <cikti.yaml> '<JSON: {"a.b.c": deger, ...}>'

Noktalı yol YAML'daki anahtar zinciridir, ör.
  "controller_server.ros__parameters.FollowPath.vx_max": 0.4
  "planner_server.ros__parameters.GridBased.motion_model_for_search": "DUBIN"
Ara anahtar yoksa hata verir (yazım hatası sessizce yeni anahtar yaratmasın);
son anahtar yoksa ancak --allow-new ile eklenir.
Çıktının sonuna uygulanan değişiklikler yorum olarak yazılır.
"""

import argparse
import json
import sys

import yaml


def apply(doc, dotted, value, allow_new):
    keys = dotted.split('.')
    cur = doc
    for k in keys[:-1]:
        if not isinstance(cur, dict) or k not in cur:
            raise KeyError(f'{dotted}: ara anahtar yok -> {k!r}')
        cur = cur[k]
    last = keys[-1]
    if last not in cur and not allow_new:
        raise KeyError(f'{dotted}: son anahtar yok -> {last!r} (eklemek için --allow-new)')
    old = cur.get(last, '<yok>')
    cur[last] = value
    return old


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('base')
    ap.add_argument('out')
    ap.add_argument('changes')
    ap.add_argument('--allow-new', action='store_true')
    a = ap.parse_args(argv)
    doc = yaml.safe_load(open(a.base))
    changes = json.loads(a.changes)
    log = []
    for k, v in changes.items():
        old = apply(doc, k, v, a.allow_new)
        log.append(f'# {k}: {old!r} -> {v!r}')
    with open(a.out, 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True, default_flow_style=None)
        f.write('\n# --- make_variant değişiklikleri ---\n' + '\n'.join(log) + '\n')
    print('\n'.join(log))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (KeyError, json.JSONDecodeError) as e:
        print(f'error: {e}', file=sys.stderr)
        sys.exit(2)
