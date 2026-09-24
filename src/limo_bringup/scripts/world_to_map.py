#!/usr/bin/env python3
"""Gazebo SDF dünyasındaki statik kutu/silindir engellerden Nav2 haritası üretir.

Neden: slam_toolbox ilk haritayı ancak robot minimum_travel_distance kadar hareket
edince yayınlıyor; Nav2 ise hareket için haritaya ihtiyaç duyuyor. Ölçüm (benchmark)
koşularının her biri aynı, kesin bir haritayla başlamalı — dünya dosyası bunun
tek doğru kaynağı.

Kullanım:
    world_to_map.py <world.sdf> <cikti_on_eki> [--resolution 0.05] [--margin 0.5]
Çıktı: <on_ek>.pgm ve <on_ek>.yaml (map_server biçimi, trinary).
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET


def _pose(el):
    p = el.find('pose')
    vals = [float(v) for v in p.text.split()] if p is not None and p.text else []
    vals += [0.0] * (6 - len(vals))
    return vals  # x y z roll pitch yaw


def collect_shapes(world_path):
    """(tip, x, y, yaw, boyut) listesi döndürür. Yalnızca <static>true modeller."""
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError(f'{world_path}: <world> bulunamadı')
    shapes = []
    for model in world.findall('model'):
        static = model.find('static')
        if static is None or static.text.strip().lower() not in ('1', 'true'):
            continue
        mx, my, _, _, _, myaw = _pose(model)
        for link in model.findall('link'):
            lx, ly, _, _, _, lyaw = _pose(link)
            for col in link.findall('collision'):
                cx, cy, _, _, _, cyaw = _pose(col)
                geo = col.find('geometry')
                # link/collision pozlarını model pozuna göre döndür
                c, s = math.cos(myaw), math.sin(myaw)
                ox, oy = lx + cx, ly + cy
                wx, wy = mx + c * ox - s * oy, my + s * ox + c * oy
                yaw = myaw + lyaw + cyaw
                box = geo.find('box')
                cyl = geo.find('cylinder')
                if box is not None:
                    sx, sy, _ = [float(v) for v in box.find('size').text.split()]
                    shapes.append(('box', wx, wy, yaw, (sx, sy)))
                elif cyl is not None:
                    r = float(cyl.find('radius').text)
                    shapes.append(('cyl', wx, wy, yaw, (r,)))
    return shapes


def rasterize(shapes, resolution, margin):
    xs, ys = [], []
    for kind, x, y, yaw, dims in shapes:
        ext = math.hypot(*dims) / 2 if kind == 'box' else dims[0]
        xs += [x - ext, x + ext]
        ys += [y - ext, y + ext]
    if not xs:
        raise ValueError('dünyada statik engel yok')
    x0, x1 = min(xs) - margin, max(xs) + margin
    y0, y1 = min(ys) - margin, max(ys) + margin
    w = int(math.ceil((x1 - x0) / resolution))
    h = int(math.ceil((y1 - y0) / resolution))
    grid = [[0] * w for _ in range(h)]  # 0 = boş, 1 = dolu
    for j in range(h):
        py = y0 + (j + 0.5) * resolution
        for i in range(w):
            px = x0 + (i + 0.5) * resolution
            for kind, x, y, yaw, dims in shapes:
                dx, dy = px - x, py - y
                if kind == 'cyl':
                    if dx * dx + dy * dy <= dims[0] ** 2:
                        grid[j][i] = 1
                        break
                else:
                    c, s = math.cos(-yaw), math.sin(-yaw)
                    lx, ly = c * dx - s * dy, s * dx + c * dy
                    if abs(lx) <= dims[0] / 2 and abs(ly) <= dims[1] / 2:
                        grid[j][i] = 1
                        break
    return grid, (x0, y0), w, h


def write_map(grid, origin, w, h, resolution, prefix):
    pgm = prefix + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        # PGM'nin ilk satırı haritanın ÜST kenarıdır (y en büyük)
        for j in reversed(range(h)):
            f.write(bytes(0 if v else 254 for v in grid[j]))
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(pgm)}\n'
                'mode: trinary\n'
                f'resolution: {resolution}\n'
                f'origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.25\n')
    return pgm, prefix + '.yaml'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('world')
    ap.add_argument('prefix')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--margin', type=float, default=0.5)
    a = ap.parse_args(argv)
    shapes = collect_shapes(a.world)
    grid, origin, w, h = rasterize(shapes, a.resolution, a.margin)
    pgm, yml = write_map(grid, origin, w, h, a.resolution, a.prefix)
    occ = sum(map(sum, grid))
    print(f'{len(shapes)} engel -> {w}x{h} hücre @ {a.resolution} m, '
          f'origin ({origin[0]:.2f}, {origin[1]:.2f}), dolu hücre {occ}')
    print(pgm)
    print(yml)
    return 0


if __name__ == '__main__':
    sys.exit(main())
