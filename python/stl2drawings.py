#!/usr/bin/env python3
"""
stl2drawings.py — 2D drawings from an STL mesh (e.g. OpenSCAD / AdamCAD / Thingiverse
exports). Companion to step2drawings.py (uses its sheet/PDF/DXF code).

An STL has no part names and no exact circles, so this tool:
  * splits the mesh into separate bodies (pieces that don't touch),
  * merges identical bodies (qty), flags tiny hex prisms as likely bolt heads/nuts,
  * draws each unique body: front/top/side (hidden lines dashed) + isometric,
    overall dimensions, title block, plus a 1:1 cut-profile DXF for plates,
  * draws a general-arrangement (GA) sheet of the whole model with overall size.
Edges are taken from sharp creases (> --crease degrees) and silhouettes, and
hidden lines are found by ray casting (fast with `pip install embreex`).

Usage:
  pip install cadquery ezdxf matplotlib trimesh rtree scipy embreex
  python stl2drawings.py model.stl -o drawings
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import itertools
import math
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from step2drawings import (SHEETS, Sheet, bom_page, fmt, pick_scale, prims_bbox,  # noqa: E402
                           render_dxf, render_pdf, scale_label, view_axes)


# ----------------------------------------------------------------------------
# mesh projection with hidden-line removal
# ----------------------------------------------------------------------------

def mesh_edges(mesh, n, crease_deg):
    """Edges worth drawing for view direction n: sharp creases, silhouettes,
    open / non-manifold edges."""
    adj = mesh.face_adjacency
    ang = mesh.face_adjacency_angles
    ae = mesh.face_adjacency_edges
    fn = mesh.face_normals
    d0, d1 = fn[adj[:, 0]] @ n, fn[adj[:, 1]] @ n
    sharp = ang > math.radians(crease_deg)
    sil = (d0 > 1e-9) != (d1 > 1e-9)
    sel = ae[sharp | sil]
    # edges not shared by exactly two faces (open boundaries / touching parts)
    es = mesh.edges_sorted
    uniq, counts = np.unique(es, axis=0, return_counts=True)
    odd = uniq[counts != 2]
    E = np.vstack([np.sort(sel, axis=1), odd]) if len(odd) else np.sort(sel, axis=1)
    return np.unique(E, axis=0)


def mesh_project(mesh, view, hidden=True, crease_deg=25.0, samples_per_diag=250):
    n, x, y = view_axes(view)
    E = mesh_edges(mesh, n, crease_deg)
    if len(E) == 0:
        return [], []
    V = mesh.vertices
    a, b = V[E[:, 0]], V[E[:, 1]]
    diag = float(np.linalg.norm(mesh.extents))
    step = diag / samples_per_diag
    L = np.linalg.norm(b - a, axis=1)
    k = np.clip(np.ceil(L / step).astype(int), 1, 60)
    # sub-pieces: edge i split into k[i] pieces; test each piece's midpoint
    idx = np.repeat(np.arange(len(E)), k)
    j = np.concatenate([np.arange(kk) for kk in k])
    kk = k[idx]
    t0, t1 = j / kk, (j + 1) / kk
    tm = (t0 + t1) / 2
    P = a[idx] + (b[idx] - a[idx]) * tm[:, None]
    eps = 1e-4 * diag + 1e-3
    origins = P + n * eps
    dirs = np.tile(n, (len(P), 1))
    hit = mesh.ray.intersects_any(origins, dirs)
    # merge consecutive pieces of the same edge with the same visibility
    pa = a[idx] + (b[idx] - a[idx]) * t0[:, None]
    pb = a[idx] + (b[idx] - a[idx]) * t1[:, None]
    vis, hid = [], []
    start = 0
    N = len(P)
    for i in range(1, N + 1):
        if i == N or idx[i] != idx[start] or hit[i] != hit[start]:
            p, q = pa[start], pb[i - 1]
            prim = ("L", float(p @ x), float(p @ y), float(q @ x), float(q @ y))
            if not hit[start]:
                vis.append(prim)
            elif hidden:
                hid.append(prim)
            start = i
    return vis, hid


# ----------------------------------------------------------------------------
# bodies: orientation, dedupe
# ----------------------------------------------------------------------------

def orient(mesh):
    m = mesh.copy()
    aabb = np.prod(np.maximum(m.extents, 1e-9))
    try:
        T, ext = trimesh.bounds.oriented_bounds(m)
        obb = np.prod(np.maximum(ext, 1e-9))
    except Exception:
        T, obb = None, aabb
    if T is not None and aabb / obb > 1.03:
        m.apply_transform(T)
        order = np.argsort(m.extents)[::-1]  # longest -> X, thinnest -> Z
        R = np.zeros((4, 4)); R[3, 3] = 1
        for row, col in enumerate(order):
            R[row, col] = 1
        if np.linalg.det(R[:3, :3]) < 0:
            R[2] *= -1
        m.apply_transform(R)
    m.apply_translation(-m.bounds[0])
    return m


_PERMS = []
for perm in itertools.permutations(range(3)):
    for signs in itertools.product([1, -1], repeat=3):
        M = np.zeros((3, 3))
        for i, (jj, s) in enumerate(zip(perm, signs)):
            M[i, jj] = s
        _PERMS.append(M)


def compare(m1, m2, tol=0.05):
    """'same' / 'mirror' / None for two oriented meshes."""
    v1 = np.unique(np.round(m1.vertices, 3), axis=0)
    v2 = np.unique(np.round(m2.vertices, 3), axis=0)
    if len(v1) != len(v2) or len(v1) > 4000:
        return None
    c1, c2 = v1 - m1.bounds.mean(axis=0), v2 - m2.bounds.mean(axis=0)
    from scipy.spatial import cKDTree
    tree = cKDTree(c1)
    mirror = False
    for M in _PERMS:
        d, _ = tree.query(c2 @ M.T)
        if np.all(d < tol):
            if np.linalg.det(M) > 0:
                return "same"
            mirror = True
    return "mirror" if mirror else None


class Body:
    def __init__(self, mesh, name=""):
        self.mesh = mesh
        self.qty = 1
        self.name = name
        self.mirror_of = None
        self.hardware = False


def classify_name(m):
    d = sorted(m.extents, reverse=True)
    if len(m.faces) == 20 and d[0] < 15:
        return "hex_head_or_nut", True
    if d[2] <= 12 and d[2] <= 0.2 * d[1]:
        return "plate", False
    if abs(d[0] - d[1]) < 0.02 * d[0] and d[2] < 0.5 * d[0]:
        return "disc", False
    if d[1] < 0.2 * d[0] and abs(d[1] - d[2]) < 0.1 * d[1]:
        return "rod", False
    return "part", False


# ----------------------------------------------------------------------------
# sheet layout (3 orthographic views + iso + title block)
# ----------------------------------------------------------------------------

def compose(views, iso, title_rows, W, H, first=True, notes=""):
    side = "left" if first else "right"
    names = ["front", "top", side]
    bb = {v: prims_bbox(views[v][0] + views[v][1]) for v in names}
    bb["iso"] = prims_bbox(iso)
    size = {v: (bb[v][2] - bb[v][0], bb[v][3] - bb[v][1]) for v in bb}
    m, right_col = 10, (120 if W >= 400 else 95)
    rx0 = W - m - right_col
    vx0, vy0, vx1, vy1 = m + 18, m + 14, rx0 - 14, H - m - 14
    gap = 26
    fw, fh = size["front"]; tw, th = size["top"]; sw, sh = size[side]
    s = pick_scale(fw + sw, fh + th, vx1 - vx0, vy1 - vy0, gap, gap)
    tot_w, tot_h = s * (fw + sw) + gap, s * (fh + th) + gap
    bx = vx0 + (vx1 - vx0 - tot_w) / 2
    by = vy0 + (vy1 - vy0 - tot_h) / 2
    if first:
        pos = {"front": (bx, by + s * th + gap), "top": (bx, by), side: (bx + s * fw + gap, by + s * th + gap)}
        iso_box = (bx + s * fw + gap, vy0, vx1, by + s * th + gap * 0.6)
    else:
        pos = {"front": (bx, by), "top": (bx, by + s * fh + gap), side: (bx + s * fw + gap, by)}
        iso_box = (bx + s * fw + gap, by + s * fh + gap * 0.4, vx1, vy1)
    sh_ = Sheet(W, H)
    sh_.rect(m, m, W - 2 * m, H - 2 * m, "border")
    for v in names:
        ox, oy = pos[v]
        X0, Y0 = ox - s * bb[v][0], oy - s * bb[v][1]
        sh_.place(views[v][1], X0, Y0, s, "hidden")
        sh_.place(views[v][0], X0, Y0, s, "visible")
        lab = {"front": "FRONT VIEW", "top": "TOP VIEW", "left": "LEFT VIEW", "right": "RIGHT VIEW"}[v]
        sh_.text(ox + s * size[v][0] / 2, oy - 13, lab, 2.5, "center", "top")

    def corners(v):
        ox, oy = pos[v]
        return ox, oy, ox + s * size[v][0], oy + s * size[v][1]
    fx0, fy0, fx1, fy1 = corners("front")
    sh_.hdim(fx0, fx1, fy1, fy1 + 9, fmt(size["front"][0]))
    sh_.vdim(fy0, fy1, fx0, fx0 - 9, fmt(size["front"][1]))
    tx0, ty0, tx1, ty1 = corners("top")
    sh_.vdim(ty0, ty1, tx0, tx0 - 9, fmt(size["top"][1]))
    sx0, sy0, sx1, sy1 = corners(side)
    sh_.hdim(sx0, sx1, sy1, sy1 + 9, fmt(size[side][0]))
    ix0, iy0, ix1, iy1 = iso_box
    iw, ih = size["iso"]
    if ix1 - ix0 > 25 and iy1 - iy0 > 25:
        si = pick_scale(iw, ih, ix1 - ix0 - 6, iy1 - iy0 - 10, 0, 0)
        cx = (ix0 + ix1) / 2 - si * iw / 2
        cy = (iy0 + iy1) / 2 - si * ih / 2 + 3
        sh_.place(iso, cx - si * bb["iso"][0], cy - si * bb["iso"][1], si, "visible")
        sh_.text((ix0 + ix1) / 2, cy - 4, f"ISOMETRIC VIEW (scale {scale_label(si)})", 2.5, "center", "top")
    # title block
    rows = title_rows + [("SCALE", f"{scale_label(s)}   UNITS: mm"),
                         ("PROJECTION", "FIRST ANGLE" if first else "THIRD ANGLE"),
                         ("DATE", _dt.date.today().isoformat())]
    rh = 5.2
    tbh = rh * len(rows)
    sh_.rect(rx0, m, right_col, tbh, "border")
    for i, (k, val) in enumerate(rows):
        yy = m + tbh - (i + 1) * rh
        if i:
            sh_.line(rx0, yy + rh, rx0 + right_col, yy + rh, "thin")
        sh_.text(rx0 + 1.5, yy + rh / 2, k, 2.0, "left", "center")
        sh_.text(rx0 + 30, yy + rh / 2, str(val), 3.2 if k == "PART" else 2.6, "left", "center", bold=(k == "PART"))
    sh_.line(rx0 + 28, m, rx0 + 28, m + tbh, "thin")
    y = H - m - 4
    for line in notes.split("\n"):
        if line:
            sh_.text(rx0, y, line, 2.3, "left", "top")
        y -= 4.2
    sh_.text(m + 2, m + 2, "Auto-generated by stl2drawings.py from an STL mesh (faceted geometry, round features are "
             "approximated) - check all dimensions before manufacturing.", 2.0, "left", "bottom")
    return sh_, s


def body_sheet(mesh, title_rows, W, H, first, hidden, crease, notes=""):
    side = "left" if first else "right"
    views = {v: mesh_project(mesh, v, hidden, crease) for v in ("front", "top", side)}
    iso, _ = mesh_project(mesh, "iso", False, crease)
    return compose(views, iso, title_rows, W, H, first, notes)


def write_cut(mesh, path, crease):
    import ezdxf
    vis, _ = mesh_project(mesh, "top", False, crease)
    x0, y0, _, _ = prims_bbox(vis)
    doc = ezdxf.new("R2010", units=4)
    doc.layers.add("CUT", color=7)
    msp = doc.modelspace()
    for p in vis:
        msp.add_line((p[1] - x0, p[2] - y0), (p[3] - x0, p[4] - y0), dxfattribs={"layer": "CUT"})
    doc.saveas(path)


# ----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="2D drawings from an STL mesh")
    ap.add_argument("stl")
    ap.add_argument("-o", "--out", default="drawings")
    ap.add_argument("--projection", choices=["first", "third"], default="first")
    ap.add_argument("--sheet", choices=list(SHEETS), default="A3")
    ap.add_argument("--crease", type=float, default=25.0,
                    help="min angle (deg) between faces to draw an edge (raise for very faceted models)")
    ap.add_argument("--no-hidden", action="store_true")
    ap.add_argument("--include-hardware", action="store_true")
    args = ap.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    os.makedirs(args.out, exist_ok=True)
    W, H = SHEETS[args.sheet]
    first = args.projection == "first"
    src = os.path.basename(args.stl)
    print(f"Reading {src} ...")
    full = trimesh.load(args.stl, force="mesh")
    full.merge_vertices()
    print(f"  {len(full.faces)} triangles, size {fmt(full.extents[0])} x {fmt(full.extents[1])} x {fmt(full.extents[2])} mm")

    pages = []
    # --- general arrangement --------------------------------------------------
    print("  general arrangement sheet ...")
    ga = full.copy()
    ga.apply_translation(-ga.bounds[0])
    ga_sheet, ga_s = body_sheet(ga, [("PART", "GENERAL ARRANGEMENT"), ("SOURCE", src[:34]),
                                     ("SIZE (X×Y×Z)", " × ".join(fmt(v) for v in full.extents) + " mm")],
                                W, H, first, hidden=False, crease=args.crease)
    pages.append(("00_GENERAL_ARRANGEMENT", [ga_sheet], ga_s))

    # --- bodies ------------------------------------------------------------------
    raw = full.split(only_watertight=False)
    print(f"  {len(raw)} separate bodies")
    bodies: list[Body] = []
    for r in sorted(raw, key=lambda b: -len(b.faces)):
        m = orient(r)
        kind, hw = classify_name(m)
        sig = (len(m.faces), tuple(np.round(sorted(m.extents), 1)))
        for b in bodies:
            if (len(b.mesh.faces), tuple(np.round(sorted(b.mesh.extents), 1))) != sig:
                continue
            res = compare(b.mesh, m)
            if res == "same":
                b.qty += 1
                break
            if res == "mirror" and b.mirror_of is None:
                pass
        else:
            nb = Body(m)
            nb.hardware = hw
            d = sorted(m.extents, reverse=True)
            nb.name = f"{kind}_{fmt(d[0])}x{fmt(d[1])}x{fmt(d[2])}"
            if len(m.faces) > 0.5 * len(full.faces) and len(raw) > 1:
                nb.name = "main_body_" + nb.name.split("_", 1)[1]
            bodies.append(nb)
    print(f"  {len(bodies)} unique bodies")

    bom_rows, hw_rows, csv_rows = [], [], []
    item = 0
    for b in bodies:
        d = b.mesh.extents
        size = f"{fmt(d[0])} × {fmt(d[1])} × {fmt(d[2])}"
        vol = abs(b.mesh.volume) if b.mesh.is_watertight else float("nan")
        if b.hardware and not args.include_hardware:
            hw_rows.append(["HW", b.name, b.qty, size, "-", "Likely bolt head / nut (no drawing)", "-"])
            csv_rows.append(["HW", b.name, b.qty, *[fmt(v) for v in d], "", "Likely bolt head / nut", ""])
            continue
        item += 1
        it = f"{item:02d}"
        base = f"{it}_{b.name}"
        print(f"  [{it}] {b.name} x{b.qty} ...", end=" ", flush=True)
        srt = sorted(d)
        plate = srt[0] <= 12 and srt[0] <= 0.2 * srt[1]
        typ = f"Plate t={fmt(srt[0])} (laser/CNC cut)" if plate else "Machined / printed part"
        note = ""
        if b.name.startswith("main_body"):
            typ = "Several parts merged in the STL"
            note = ("NOTE: in this STL several parts are fused into\none body (chassis, wheels, ...). "
                    "Export each part\nseparately (or the .scad / STEP) for\nindividual part drawings.")
        rows = [("PART", b.name), ("ITEM / QTY", f"{it}   /   {b.qty} off"),
                ("SIZE (X×Y×Z)", f"{size} mm"), ("MATERIAL", "________________"),
                ("TOLERANCE", "________________"),
                ("VOLUME", f"{fmt(vol / 1000)} cm³" if vol == vol else "n/a (open mesh)"),
                ("SOURCE", src[:34])]
        sheet, s = body_sheet(b.mesh, rows, W, H, first, not args.no_hidden, args.crease, note)
        files = [base + ".pdf", base + ".dxf", base + ".stl"]
        b.mesh.export(os.path.join(args.out, base + ".stl"))
        if plate:
            write_cut(b.mesh, os.path.join(args.out, base + "_CUT.dxf"), args.crease)
            files.append(base + "_CUT.dxf")
        pages.append((base, [sheet], s))
        bom_rows.append([it, b.name, b.qty, size, "-", typ, ", ".join(f.split(".")[-1] for f in files)])
        csv_rows.append([it, b.name, b.qty, *[fmt(v) for v in d], fmt(vol / 1000) if vol == vol else "", typ, " ".join(files)])
        print(f"scale {scale_label(s)}")

    for base, sheets, s in pages:
        with PdfPages(os.path.join(args.out, base + ".pdf")) as pdf:
            for sh in sheets:
                fig = render_pdf(sh); pdf.savefig(fig); plt.close(fig)
        render_dxf(sheets, os.path.join(args.out, base + ".dxf"), s)
    with open(os.path.join(args.out, "BOM.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["item", "part", "qty", "size_x_mm", "size_y_mm", "size_z_mm", "volume_cm3", "type", "files"])
        w.writerows(csv_rows)
    with PdfPages(os.path.join(args.out, "ALL_DRAWINGS.pdf")) as pdf:
        fig = render_pdf(bom_page(bom_rows, hw_rows, src, W, H)); pdf.savefig(fig); plt.close(fig)
        for _, sheets, _ in pages:
            for sh in sheets:
                fig = render_pdf(sh); pdf.savefig(fig); plt.close(fig)
    print(f"Done -> {os.path.abspath(args.out)}  (open ALL_DRAWINGS.pdf)")


if __name__ == "__main__":
    main()
