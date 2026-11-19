#!/usr/bin/env python3
"""
step2drawings.py — Turn a full robot/assembly STEP file into 2D manufacturing
drawings for every unique part. 100% free (CadQuery/OCP + ezdxf + matplotlib).

For each unique part it produces:
  * <item>_<name>.pdf        A3 drawing sheet: front/top/side views (hidden lines),
                             isometric view, overall dimensions, hole table,
                             title block (part name, qty, scale, projection)
  * <item>_<name>.dxf        Same sheet as an editable DXF (true size, mm)
  * <item>_<name>_CUT.dxf    1:1 flat cut profile (only for plate-like parts,
                             ready for laser / waterjet / CNC router)
  * <item>_<name>.step/.stl  The single part (for CAM / 3D printing)
And for the whole assembly:
  * ALL_DRAWINGS.pdf         Cover page with BOM + every drawing
  * BOM.csv                  Item, name, qty, size, holes, files

Usage:
  pip install cadquery ezdxf matplotlib
  python step2drawings.py robot.step -o drawings
  python step2drawings.py robot.step -o drawings --projection third --no-hidden

Duplicate parts (e.g. 4 identical wheels) are drawn once with the right
quantity. Purchased hardware (screws, nuts, bearings ...) is detected by name and
listed in the BOM without a drawing (use --include-hardware to draw it anyway).

Limits: dimensions are overall sizes + a full hole table (position, diameter,
depth / THRU, counterbore). Tolerances, surface finish and thread callouts must
be added by you. Always check a drawing before sending it to a shop.
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import math
import os
import re
import sys
from dataclasses import dataclass, field

import numpy as np

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepGProp import BRepGProp, BRepGProp_Face
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.Bnd import Bnd_Box, Bnd_OBB
from OCP.GCPnts import GCPnts_TangentialDeflection
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Cylinder, GeomAbs_Line
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence, TDF_Tool
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_OUT, TopAbs_SOLID, TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

class quiet:
    """Silence OpenCASCADE's C-level console chatter."""
    def __enter__(self):
        try:
            sys.stdout.flush()
            self.fd = os.dup(1)
            self.null = os.open(os.devnull, os.O_WRONLY)
            os.dup2(self.null, 1)
        except Exception:
            self.fd = None
        return self

    def __exit__(self, *a):
        if self.fd is not None:
            sys.stdout.flush()
            os.dup2(self.fd, 1)
            os.close(self.fd)
            os.close(self.null)


GENERIC_NAME = re.compile(r"^(open cascade.*|solid\d*|body\d*|part\d*|shape\d*|compound\d*|unnamed.*|none|nauo\d*|[-_ .\d]*)$", re.I)

HARDWARE_WORDS = re.compile(
    r"(screw|bolt|nut\b|nut_|washer|bearing|rivet|insert|circlip|spring|pin\b|"
    r"\bdin[ _-]?\d|\biso[ _-]?\d|\bm[2-9]x|\bm1[0-6]x|grub|setscrew|standard[_ ]?part|"
    r"socket[_ ]?head|hex[_ ]?head|countersunk|\bshcs\b|\bbhcs\b|\bfhcs\b)",
    re.I,
)

# ----------------------------------------------------------------------------
# 1. Reading the STEP file
# ----------------------------------------------------------------------------

@dataclass
class Part:
    name: str
    shape: TopoDS_Shape
    qty: int = 1
    key: str = ""
    hardware: bool = False
    instance_names: list = field(default_factory=list)
    mirror_of: str | None = None


def _label_name(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return None


def _label_entry(label):
    s = TCollection_AsciiString()
    TDF_Tool.Entry_s(label, s)
    return s.ToCString()


def _solids(shape):
    out = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(TopoDS.Solid_s(exp.Current()))
        exp.Next()
    return out


def read_step_parts(path):
    """Return list of Part (one per unique part definition, with instance count)."""
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(False)
    reader.SetLayerMode(False)
    parts: dict[str, Part] = {}
    order: list[str] = []
    if reader.ReadFile(path) == IFSelect_RetDone and reader.Transfer(doc):
        st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

        def walk(label, inst_name):
            if st.IsAssembly_s(label):
                comps = TDF_LabelSequence()
                st.GetComponents_s(label, comps, False)
                for i in range(1, comps.Length() + 1):
                    comp = comps.Value(i)
                    ref = TDF_Label()
                    if st.GetReferredShape_s(comp, ref):
                        walk(ref, _label_name(comp) or inst_name)
                return
            key = _label_entry(label)
            if key in parts:
                parts[key].qty += 1
                parts[key].instance_names.append(inst_name or "")
                return
            shape = st.GetShape_s(label)
            name = _label_name(label) or inst_name or f"part_{len(order)+1}"
            parts[key] = Part(name=name, shape=shape, key=key, instance_names=[inst_name or name])
            order.append(key)

        free = TDF_LabelSequence()
        st.GetFreeShapes(free)
        for i in range(1, free.Length() + 1):
            walk(free.Value(i), None)
    else:
        print("  (XCAF read failed, falling back to plain STEP reader)")
        r = STEPControl_Reader()
        if r.ReadFile(path) != IFSelect_RetDone:
            sys.exit(f"Could not read STEP file: {path}")
        r.TransferRoots()
        shp = r.OneShape()
        parts["root"] = Part(name=os.path.splitext(os.path.basename(path))[0], shape=shp, key="root")
        order.append("root")

    # Explode multi-body parts into separate solids
    result: list[Part] = []
    for k in order:
        p = parts[k]
        sols = _solids(p.shape)
        if not sols:
            continue
        if len(sols) == 1:
            p.shape = sols[0]
            result.append(p)
        else:
            for j, s in enumerate(sols, 1):
                result.append(Part(name=f"{p.name}_body{j}", shape=s, qty=p.qty,
                                   key=f"{k}#{j}", instance_names=p.instance_names))
    for p in result:
        p.name = _clean_name(p.name, p.qty)
        p.hardware = bool(HARDWARE_WORDS.search(p.name))
    return result


def _clean_name(name, qty):
    name = name.strip() or "part"
    if qty > 1:  # "wheel_1" used 4 times -> "wheel"
        name = re.sub(r"([ _\-.]\d+|\s*\(\d+\)|<\d+>)$", "", name) or name
    return name


# ----------------------------------------------------------------------------
# 2. Geometry helpers
# ----------------------------------------------------------------------------

def tight_bbox(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    return box.Get()  # xmin, ymin, zmin, xmax, ymax, zmax


def transformed(shape, trsf):
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def obb_frame(shape):
    obb = Bnd_OBB()
    BRepBndLib.AddOBB_s(shape, obb, False, True, False)
    c = obb.Center()
    x = obb.XDirection()
    y = obb.YDirection()
    z = x.Crossed(y)
    return obb, gp_Pnt(c.X(), c.Y(), c.Z()), gp_Dir(x), gp_Dir(z)


def canonical_orient(shape):
    """Keep the designer's orientation if the part is axis-aligned; otherwise
    rotate it onto its tight bounding box axes. Then move bbox min to origin."""
    xmin, ymin, zmin, xmax, ymax, zmax = tight_bbox(shape)
    aabb_vol = max((xmax - xmin) * (ymax - ymin) * (zmax - zmin), 1e-9)
    try:
        obb, c, xd, zd = obb_frame(shape)
        obb_vol = 8 * obb.XHSize() * obb.YHSize() * obb.ZHSize()
    except Exception:
        obb_vol = aabb_vol
    if obb_vol > 0 and aabb_vol / obb_vol > 1.03:
        t = gp_Trsf()
        t.SetTransformation(gp_Ax3(c, zd, xd))
        shape = transformed(shape, t)
        # lay it flat: thinnest dimension along Z, longest along X
        xmin, ymin, zmin, xmax, ymax, zmax = tight_bbox(shape)
        dims = [xmax - xmin, ymax - ymin, zmax - zmin]
        order = np.argsort(dims)[::-1]  # longest first
        axes = [gp_Dir(1, 0, 0), gp_Dir(0, 1, 0), gp_Dir(0, 0, 1)]
        newx, newy = axes[order[0]], axes[order[1]]
        newz = newx.Crossed(newy)
        t2 = gp_Trsf()
        t2.SetTransformation(gp_Ax3(gp_Pnt(0, 0, 0), newz, newx))
        shape = transformed(shape, t2)
    xmin, ymin, zmin, *_ = tight_bbox(shape)
    t3 = gp_Trsf()
    t3.SetTranslation(gp_Vec(-xmin, -ymin, -zmin))
    return transformed(shape, t3)


def props(shape):
    vp = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, vp)
    sp = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, sp)
    return vp.Mass(), sp.Mass()


def vertices_np(shape):
    pts = []
    exp = TopExp_Explorer(shape, TopAbs_VERTEX)
    while exp.More():
        p = BRep_Tool.Pnt_s(TopoDS.Vertex_s(exp.Current()))
        pts.append((p.X(), p.Y(), p.Z()))
        exp.Next()
    return np.unique(np.round(np.array(pts), 3), axis=0) if pts else np.zeros((0, 3))


def _signed_perms():
    mats = []
    import itertools
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([1, -1], repeat=3):
            m = np.zeros((3, 3))
            for i, (j, s) in enumerate(zip(perm, signs)):
                m[i, j] = s
            mats.append(m)
    return mats


_PERMS = _signed_perms()


def _match(a, b, tol):
    if len(a) != len(b) or len(a) == 0:
        return False
    from math import inf
    # brute force nearest neighbour (vertex counts are small)
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    return bool(np.all(d.min(axis=1) < tol) and np.all(d.min(axis=0) < tol))


def compare_geometry(sa, sb, tol=0.05):
    """Return 'same', 'mirror' or None for two solids (any position/rotation)."""
    va, vb = vertices_np(sa), vertices_np(sb)
    if len(va) != len(vb):
        return None
    _, ca, xa, za = obb_frame(sa)
    _, cb, xb, zb = obb_frame(sb)

    def local(v, c, x, z):
        z = np.array([z.X(), z.Y(), z.Z()])
        x = np.array([x.X(), x.Y(), x.Z()])
        y = np.cross(z, x)
        return (v - np.array([c.X(), c.Y(), c.Z()])) @ np.vstack([x, y, z]).T

    la, lb = local(va, ca, xa, za), local(vb, cb, xb, zb)
    found_mirror = False
    for m in _PERMS:
        if _match(la, lb @ m.T, tol):
            if np.linalg.det(m) > 0:
                return "same"
            found_mirror = True
    return "mirror" if found_mirror else None


def merge_identical(parts):
    """Second pass: merge separate part definitions that are geometrically
    identical (flattened STEP exports). Mirrored (left/right) parts stay separate."""
    sigs = []
    for p in parts:
        v, a = props(p.shape)
        x0, y0, z0, x1, y1, z1 = tight_bbox(p.shape)
        sigs.append((round(v, 0), round(a, 0)))
    keep = []
    for i, p in enumerate(parts):
        merged = False
        for k in keep:
            q = parts[k]
            if sigs[k] != sigs[i] or q.hardware != p.hardware:
                continue
            res = compare_geometry(q.shape, p.shape)
            if res == "same":
                q.qty += p.qty
                q.instance_names += p.instance_names
                merged = True
                break
            if res == "mirror":
                p.mirror_of = q.name
        if not merged:
            keep.append(i)
    return [parts[k] for k in keep]


# ----------------------------------------------------------------------------
# 3. Hidden-line projection
# ----------------------------------------------------------------------------

# view name -> (direction towards viewer, screen-x direction)
VIEWS = {
    "front": ((0, -1, 0), (1, 0, 0)),
    "top": ((0, 0, 1), (1, 0, 0)),
    "left": ((-1, 0, 0), (0, -1, 0)),   # seen from the left (first angle: drawn on the right)
    "right": ((1, 0, 0), (0, 1, 0)),    # seen from the right (third angle)
    "bottom": ((0, 0, -1), (1, 0, 0)),
    "iso": ((1, -1, 1), (1, 1, 0)),
}


def view_axes(view):
    n, x = VIEWS[view]
    n = np.array(n, float) / np.linalg.norm(n)
    x = np.array(x, float) / np.linalg.norm(x)
    y = np.cross(n, x)
    return n, x, y


def _edges(shape):
    out = []
    if shape is None or shape.IsNull():
        return out
    exp = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp.More():
        out.append(TopoDS.Edge_s(exp.Current()))
        exp.Next()
    return out


def _ang(cx, cy, p):
    return math.degrees(math.atan2(p[1] - cy, p[0] - cx)) % 360


def edge_prims(edge):
    try:
        c = BRepAdaptor_Curve(edge)
    except Exception:
        return []
    u0, u1 = c.FirstParameter(), c.LastParameter()
    t = c.GetType()
    P = lambda u: (c.Value(u).X(), c.Value(u).Y())
    if t == GeomAbs_Line:
        return [("L", *P(u0), *P(u1))]
    if t == GeomAbs_Circle:
        circ = c.Circle()
        cx, cy, r = circ.Location().X(), circ.Location().Y(), circ.Radius()
        if abs(u1 - u0) >= 2 * math.pi - 1e-6:
            return [("C", cx, cy, r)]
        a0, a1, am = _ang(cx, cy, P(u0)), _ang(cx, cy, P(u1)), _ang(cx, cy, P((u0 + u1) / 2))
        sweep = (a1 - a0) % 360
        if (am - a0) % 360 < sweep:
            return [("A", cx, cy, r, a0, a1)]
        return [("A", cx, cy, r, a1, a0)]
    try:
        d = GCPnts_TangentialDeflection(c, 0.08, 0.01)
        pts = [(d.Value(i).X(), d.Value(i).Y()) for i in range(1, d.NbPoints() + 1)]
    except Exception:
        pts = [P(u0 + (u1 - u0) * k / 24) for k in range(25)]
    return [("P", pts)] if len(pts) > 1 else []


def project(shape, view, hidden=True):
    n, x, _ = view_axes(view)
    ax = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*n), gp_Dir(*x))
    algo = HLRBRep_Algo()
    algo.Add(shape)
    algo.Projector(HLRAlgo_Projector(ax))
    algo.Update()
    algo.Hide()
    h = HLRBRep_HLRToShape(algo)
    vis, hid = [], []
    for s in (h.VCompound(), h.OutLineVCompound()):
        for e in _edges(s):
            vis += edge_prims(e)
    if hidden:
        for s in (h.HCompound(), h.OutLineHCompound()):
            for e in _edges(s):
                hid += edge_prims(e)
    return vis, hid


def prims_bbox(prims):
    xs, ys = [], []
    for p in prims:
        if p[0] == "L":
            xs += [p[1], p[3]]; ys += [p[2], p[4]]
        elif p[0] == "C":
            xs += [p[1] - p[3], p[1] + p[3]]; ys += [p[2] - p[3], p[2] + p[3]]
        elif p[0] == "A":
            _, cx, cy, r, a0, a1 = p
            sweep = (a1 - a0) % 360 or 360
            for k in range(0, 33):
                a = math.radians(a0 + sweep * k / 32)
                xs.append(cx + r * math.cos(a)); ys.append(cy + r * math.sin(a))
        elif p[0] == "P":
            xs += [q[0] for q in p[1]]; ys += [q[1] for q in p[1]]
    if not xs:
        return 0, 0, 1, 1
    return min(xs), min(ys), max(xs), max(ys)


# ----------------------------------------------------------------------------
# 4. Hole detection
# ----------------------------------------------------------------------------

@dataclass
class Hole:
    axis_dir: np.ndarray
    center: np.ndarray       # point on axis at the entry end
    dia: float
    depth: float | None      # None = THRU
    cbore: list = field(default_factory=list)  # [(dia, depth)]
    hid: str = ""
    view: str = ""
    xy: tuple = (0, 0)       # position in view (relative to view datum)


def find_holes(solid):
    groups = []  # each: dict(dir, origin, faces=[(r, tmin, tmax, coverage)])
    exp = TopExp_Explorer(solid, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        exp.Next()
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() != GeomAbs_Cylinder:
            continue
        cyl = surf.Cylinder()
        r = cyl.Radius()
        a = cyl.Axis()
        o = np.array([a.Location().X(), a.Location().Y(), a.Location().Z()])
        d = np.array([a.Direction().X(), a.Direction().Y(), a.Direction().Z()])
        # canonical direction sign
        i = int(np.argmax(np.abs(d)))
        if d[i] < 0:
            d = -d
        umin, umax, vmin, vmax = BRepTools.UVBounds_s(face)
        um, vm = (umin + umax) / 2, (vmin + vmax) / 2
        pnt, vec = gp_Pnt(), gp_Vec()
        BRepGProp_Face(face).Normal(um, vm, pnt, vec)
        p = np.array([pnt.X(), pnt.Y(), pnt.Z()])
        radial = (p - o) - np.dot(p - o, d) * d
        nrm = np.array([vec.X(), vec.Y(), vec.Z()])
        if np.dot(nrm, radial) >= 0:
            continue  # convex (boss / outer surface), not a hole
        # axial extent of this face along d
        ends = []
        for u in (umin, um, umax):
            for v in (vmin, vmax):
                q = surf.Value(u, v)
                ends.append(np.dot(np.array([q.X(), q.Y(), q.Z()]) - o, d))
        # normalise axis origin: closest point to the global origin
        o0 = o - np.dot(o, d) * d
        t_off = np.dot(o - o0, d)
        entry = (r, min(ends) + t_off, max(ends) + t_off, umax - umin)
        for g in groups:
            if np.allclose(g["dir"], d, atol=1e-4) and np.linalg.norm(g["origin"] - o0) < 1e-3:
                g["faces"].append(entry)
                break
        else:
            groups.append({"dir": d, "origin": o0, "faces": [entry]})

    holes = []
    for g in groups:
        # combine faces of same radius (split cylinders)
        by_r: dict[float, list] = {}
        for r, t0, t1, cov in g["faces"]:
            by_r.setdefault(round(r, 3), []).append((t0, t1, cov))
        steps = []
        for r, lst in by_r.items():
            cov = sum(c for _, _, c in lst)
            if cov < 1.5 * math.pi:     # fillets / slot ends, not a round hole
                continue
            steps.append((r, min(t0 for t0, _, _ in lst), max(t1 for _, t1, _ in lst)))
        if not steps:
            continue
        steps.sort()
        r_main, a0, a1 = steps[0]
        tmin = min(s[1] for s in steps)
        tmax = max(s[2] for s in steps)
        d, o0 = g["dir"], g["origin"]
        clf = BRepClass3d_SolidClassifier()
        def outside(t):
            p = o0 + d * t
            clf.Load(solid)
            clf.Perform(gp_Pnt(*p), 1e-6)
            return clf.State() == TopAbs_OUT
        out_lo, out_hi = outside(tmin - 0.05), outside(tmax + 0.05)
        thru = out_lo and out_hi
        # entry side: the open end (prefer end where counterbore sits)
        if len(steps) > 1:
            cb_mid = np.mean([(s[1] + s[2]) / 2 for s in steps[1:]])
            entry_t = tmax if cb_mid > (tmin + tmax) / 2 else tmin
        else:
            entry_t = tmax if out_hi else tmin
        cbores = [(2 * s[0], s[2] - s[1]) for s in steps[1:]]
        holes.append(Hole(axis_dir=d, center=o0 + d * entry_t, dia=2 * r_main,
                          depth=None if thru else (tmax - tmin), cbore=cbores))
    return holes


def hole_hint(dia):
    hints = [(2.5, "M3 tap drill?"), (3.3, "M4 tap drill?"), (4.2, "M5 tap drill?"),
             (3.4, "M3 clearance?"), (3.2, "M3 clearance?"), (4.5, "M4 clearance?"),
             (5.5, "M5 clearance?"), (6.6, "M6 clearance?"), (6.8, "M8 tap drill?"),
             (9.0, "M8 clearance?")]
    for d, h in hints:
        if abs(dia - d) < 0.051:
            return h
    return ""


def fmt(v):
    s = f"{v:.2f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


# ----------------------------------------------------------------------------
# 5. Sheet composition (primitive list in paper mm, rendered to PDF and DXF)
# ----------------------------------------------------------------------------

SHEETS = {"A3": (420, 297), "A4": (297, 210)}
SCALES = [20, 10, 5, 4, 2, 1, 1 / 2, 1 / 2.5, 1 / 5, 1 / 10, 1 / 20, 1 / 50, 1 / 100]


def scale_label(s):
    return f"{int(round(s))}:1" if s >= 1 else f"1:{fmt(1 / s)}"


class Sheet:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.items = []  # (kind, layer, data)

    def line(self, x1, y1, x2, y2, layer="visible"):
        self.items.append(("L", layer, (x1, y1, x2, y2)))

    def circle(self, cx, cy, r, layer="visible"):
        self.items.append(("C", layer, (cx, cy, r)))

    def arc(self, cx, cy, r, a0, a1, layer="visible"):
        self.items.append(("A", layer, (cx, cy, r, a0, a1)))

    def poly(self, pts, layer="visible"):
        self.items.append(("P", layer, pts))

    def tri(self, pts, layer="dim"):
        self.items.append(("T", layer, pts))

    def text(self, x, y, s, h=3.5, ha="center", va="center", rot=0, layer="text", bold=False):
        self.items.append(("X", layer, (x, y, s, h, ha, va, rot, bold)))

    def rect(self, x, y, w, h, layer="border"):
        self.poly([(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)], layer)

    # --- placing projected geometry --------------------------------------
    def place(self, prims, ox, oy, s, layer):
        for p in prims:
            if p[0] == "L":
                self.line(ox + s * p[1], oy + s * p[2], ox + s * p[3], oy + s * p[4], layer)
            elif p[0] == "C":
                self.circle(ox + s * p[1], oy + s * p[2], s * p[3], layer)
            elif p[0] == "A":
                self.arc(ox + s * p[1], oy + s * p[2], s * p[3], p[4], p[5], layer)
            elif p[0] == "P":
                self.poly([(ox + s * x, oy + s * y) for x, y in p[1]], layer)

    def arrow(self, x, y, dx, dy, L=3.0, W=0.6):
        n = math.hypot(dx, dy) or 1
        ux, uy = dx / n, dy / n
        bx, by = x - ux * L, y - uy * L
        self.tri([(x, y), (bx - uy * W, by + ux * W), (bx + uy * W, by - ux * W)])

    def hdim(self, x1, x2, y_obj, y_dim, value):
        sgn = 1 if y_dim > y_obj else -1
        self.line(x1, y_obj + sgn * 1, x1, y_dim + sgn * 2, "dim")
        self.line(x2, y_obj + sgn * 1, x2, y_dim + sgn * 2, "dim")
        self.line(x1, y_dim, x2, y_dim, "dim")
        if x2 - x1 > 7:
            self.arrow(x1, y_dim, -1, 0); self.arrow(x2, y_dim, 1, 0)
        else:
            self.arrow(x1, y_dim, 1, 0); self.arrow(x2, y_dim, -1, 0)
        self.text((x1 + x2) / 2, y_dim + 1.2, value, 3.5, "center", "bottom", layer="dim")

    def vdim(self, y1, y2, x_obj, x_dim, value):
        sgn = 1 if x_dim > x_obj else -1
        self.line(x_obj + sgn * 1, y1, x_dim + sgn * 2, y1, "dim")
        self.line(x_obj + sgn * 1, y2, x_dim + sgn * 2, y2, "dim")
        self.line(x_dim, y1, x_dim, y2, "dim")
        if y2 - y1 > 7:
            self.arrow(x_dim, y1, 0, -1); self.arrow(x_dim, y2, 0, 1)
        else:
            self.arrow(x_dim, y1, 0, 1); self.arrow(x_dim, y2, 0, -1)
        self.text(x_dim - 1.2, (y1 + y2) / 2, value, 3.5, "center", "bottom", rot=90, layer="dim")


def pick_scale(need_w, need_h, avail_w, avail_h, gaps_w, gaps_h):
    for s in SCALES:
        if need_w * s + gaps_w <= avail_w and need_h * s + gaps_h <= avail_h:
            return s
    return SCALES[-1]


def make_sheets(part, item, src_name, args):
    """Return (list of Sheet, info dict)."""
    W, H = SHEETS[args.sheet]
    shape = part.shape
    first = args.projection == "first"
    side_view = "left" if first else "right"
    names = ["front", "top", side_view]
    proj = {v: project(shape, v, hidden=not args.no_hidden) for v in names}
    iso = project(shape, "iso", hidden=False)
    bb = {v: prims_bbox(proj[v][0] + proj[v][1]) for v in names}
    bb["iso"] = prims_bbox(iso[0])
    size = {v: (bb[v][2] - bb[v][0], bb[v][3] - bb[v][1]) for v in bb}

    # holes -> table rows
    holes = find_holes(shape)
    axis_view = {0: side_view, 1: "front", 2: "top"}
    letters = iter("ABCDEFGHJKLMNPQRSTUVWXYZ")
    groups: dict[tuple, list] = {}
    for hobj in holes:
        key = (round(hobj.dia, 2), None if hobj.depth is None else round(hobj.depth, 2),
               tuple((round(a, 2), round(b, 2)) for a, b in hobj.cbore))
        groups.setdefault(key, []).append(hobj)
    rows = []
    pcd_note = {}
    for key, lst in groups.items():
        if len(lst) >= 3:
            c = np.array([h.center for h in lst])
            ctr = c.mean(axis=0)
            rr = np.linalg.norm(c - ctr, axis=1)
            if rr.mean() > 1e-3 and np.ptp(rr) < 0.02:
                for h in lst:
                    pcd_note[id(h)] = f"{len(lst)}x on PCD Ø{fmt(2 * rr.mean())}"
    for key, lst in sorted(groups.items(), key=lambda kv: -kv[0][0]):
        letter = next(letters, "Z")
        for i, hobj in enumerate(lst, 1):
            hobj.hid = f"{letter}{i}"
            ai = int(np.argmax(np.abs(hobj.axis_dir)))
            if abs(abs(hobj.axis_dir[ai]) - 1) < 1e-4:
                hobj.view = axis_view[ai]
                _, vx, vy = view_axes(hobj.view)
                px, py = float(np.dot(hobj.center, vx)), float(np.dot(hobj.center, vy))
                hobj.xy = (px - bb[hobj.view][0], py - bb[hobj.view][1])
            else:
                hobj.view = "oblique"
                hobj.xy = tuple(hobj.center)
            depth = "THRU" if hobj.depth is None else fmt(hobj.depth)
            cb = "; ".join(f"CBORE Ø{fmt(a)} x {fmt(b)}" for a, b in hobj.cbore)
            note = "; ".join(x for x in (cb, pcd_note.get(id(hobj), ""), hole_hint(hobj.dia)) if x)
            pos = (f"{fmt(hobj.xy[0])}", f"{fmt(hobj.xy[1])}") if hobj.view != "oblique" else \
                (f"({fmt(hobj.xy[0])},{fmt(hobj.xy[1])},", f"{fmt(hobj.xy[2])})")
            rows.append([hobj.hid, hobj.view.upper(), pos[0], pos[1], f"Ø{fmt(hobj.dia)}", depth, note])

    # layout -----------------------------------------------------------------
    m = 10
    right_col = 120 if W >= 400 else 95
    rx0 = W - m - right_col
    vx0, vy0, vx1, vy1 = m + 18, m + 14, rx0 - 14, H - m - 14
    gap = 26
    fw, fh = size["front"]
    tw, th = size["top"]
    sw, sh = size[side_view]
    s = pick_scale(fw + sw, fh + th, vx1 - vx0, vy1 - vy0, gap, gap)
    tot_w, tot_h = s * (fw + sw) + gap, s * (fh + th) + gap
    bx = vx0 + (vx1 - vx0 - tot_w) / 2
    by = vy0 + (vy1 - vy0 - tot_h) / 2
    if first:   # top view below front, left view to the right of front
        pos = {"front": (bx, by + s * th + gap), "top": (bx, by), side_view: (bx + s * fw + gap, by + s * th + gap)}
        iso_box = (bx + s * fw + gap, vy0, vx1, by + s * th + gap * 0.6)
    else:       # third angle: top view above front, right view to the right of front
        pos = {"front": (bx, by), "top": (bx, by + s * fh + gap), side_view: (bx + s * fw + gap, by)}
        iso_box = (bx + s * fw + gap, by + s * fh + gap * 0.4, vx1, vy1)

    sh_ = Sheet(W, H)
    sh_.rect(m, m, W - 2 * m, H - 2 * m, "border")
    for v in names:
        ox, oy = pos[v]
        X0, Y0 = ox - s * bb[v][0], oy - s * bb[v][1]
        sh_.place(proj[v][1], X0, Y0, s, "hidden")
        sh_.place(proj[v][0], X0, Y0, s, "visible")
        label = {"front": "FRONT VIEW", "top": "TOP VIEW", "left": "LEFT VIEW", "right": "RIGHT VIEW"}[v]
        sh_.text(ox + s * size[v][0] / 2, oy - 4 - (8 if v == "top" and not first else 0) - (9 if v != "front" or first else 0),
                 label, 2.5, "center", "top")
    # overall dimensions
    def corners(v):
        ox, oy = pos[v]
        return ox, oy, ox + s * size[v][0], oy + s * size[v][1]
    fx0, fy0, fx1, fy1 = corners("front")
    sh_.hdim(fx0, fx1, fy1, fy1 + 9, fmt(size["front"][0]))
    sh_.vdim(fy0, fy1, fx0, fx0 - 9, fmt(size["front"][1]))
    tx0, ty0, tx1, ty1 = corners("top")
    sh_.vdim(ty0, ty1, tx0, tx0 - 9, fmt(size["top"][1]))
    sx0, sy0, sx1, sy1 = corners(side_view)
    sh_.hdim(sx0, sx1, sy1, sy1 + 9, fmt(size[side_view][0]))
    # datum markers
    for v in names:
        x0_, y0_, _, _ = corners(v)
        sh_.circle(x0_, y0_, 0.8, "dim")
    # hole centre marks + labels
    for hobj in holes:
        if hobj.view not in pos:
            continue
        ox, oy = pos[hobj.view]
        cx, cy = ox + s * hobj.xy[0], oy + s * hobj.xy[1]
        r = s * hobj.dia / 2
        e = r + 1.5
        sh_.line(cx - e, cy, cx + e, cy, "center")
        sh_.line(cx, cy - e, cx, cy + e, "center")
        sh_.text(cx + r * 0.75 + 0.8, cy + r * 0.75 + 0.8, hobj.hid, 2.2, "left", "bottom", layer="dim")
    # isometric view
    ix0, iy0, ix1, iy1 = iso_box
    iw, ih = size["iso"]
    if ix1 - ix0 > 25 and iy1 - iy0 > 25:
        si = pick_scale(iw, ih, ix1 - ix0 - 6, iy1 - iy0 - 10, 0, 0)
        si = min(si, s * 1.0) if si >= s else si
        cx = (ix0 + ix1) / 2 - si * iw / 2
        cy = (iy0 + iy1) / 2 - si * ih / 2 + 3
        sh_.place(iso[0], cx - si * bb["iso"][0], cy - si * bb["iso"][1], si, "visible")
        sh_.text((ix0 + ix1) / 2, cy - 4, f"ISOMETRIC VIEW (scale {scale_label(si)})", 2.5, "center", "top")

    # title block ------------------------------------------------------------
    tbx, tby, tbw = rx0, m, right_col
    v_, a_ = props(shape)
    x0_, y0_, z0_, x1_, y1_, z1_ = tight_bbox(shape)
    L, Wd, Ht = x1_ - x0_, y1_ - y0_, z1_ - z0_
    tb_rows = [
        ("PART", part.name),
        ("ITEM / QTY", f"{item}   /   {part.qty} off"),
        ("SIZE (X×Y×Z)", f"{fmt(L)} × {fmt(Wd)} × {fmt(Ht)} mm"),
        ("MATERIAL", "________________"),
        ("TOLERANCE", "________________"),
        ("SCALE", f"{scale_label(s)}   UNITS: mm"),
        ("PROJECTION", "FIRST ANGLE" if first else "THIRD ANGLE"),
        ("VOLUME", f"{fmt(v_ / 1000)} cm³  (Al ≈ {fmt(v_ * 2.70e-3)} g)"),
        ("SOURCE", src_name[:34]),
        ("DATE", _dt.date.today().isoformat()),
    ]
    if part.mirror_of:
        tb_rows.insert(2, ("NOTE", f"MIRROR of '{part.mirror_of}'"))
    rh = 5.2
    tbh = rh * len(tb_rows)
    sh_.rect(tbx, tby, tbw, tbh, "border")
    for i, (k, val) in enumerate(tb_rows):
        y = tby + tbh - (i + 1) * rh
        if i:
            sh_.line(tbx, y + rh, tbx + tbw, y + rh, "thin")
        sh_.text(tbx + 1.5, y + rh / 2, k, 2.0, "left", "center")
        sh_.text(tbx + 30, y + rh / 2, val, 2.6 if k != "PART" else 3.2, "left", "center", bold=(k == "PART"))
    sh_.line(tbx + 28, tby, tbx + 28, tby + tbh, "thin")
    sh_.text(m + 2, m + 2, "Auto-generated by step2drawings.py from 3D model - check all dimensions before manufacturing. "
             "Hole positions are from the view datum (o = bottom-left of each view).", 2.0, "left", "bottom")

    # hole table ---------------------------------------------------------------
    sheets = [sh_]
    cols = [("HOLE", 11), ("VIEW", 13), ("X", 14), ("Y", 14), ("SIZE", 14), ("DEPTH", 14), ("NOTE", right_col - 80)]
    table_top = H - m - 2
    table_bottom = tby + tbh + 6
    rh2 = 4.4
    max_rows = int((table_top - table_bottom) / rh2) - 2
    chunks = [rows[i:i + max_rows] for i in range(0, len(rows), max_rows)] or [[]]

    def draw_table(sheet, chunk, x, top, title):
        sheet.text(x, top - 1, title, 2.8, "left", "top", bold=True)
        y = top - 7
        cx = x
        for name, w in cols:
            sheet.text(cx + 1, y, name, 2.0, "left", "center", bold=True)
            cx += w
        sheet.line(x, y - rh2 / 2, x + sum(w for _, w in cols), y - rh2 / 2, "thin")
        for row in chunk:
            y -= rh2
            cx = x
            for (name, w), val in zip(cols, row):
                sheet.text(cx + 1, y, str(val), 1.9 if name == "NOTE" else 2.1, "left", "center")
                cx += w
        return y

    if rows:
        draw_table(sh_, chunks[0], rx0, table_top, f"HOLE TABLE ({len(rows)} holes)")
        for k, chunk in enumerate(chunks[1:], 2):
            extra = Sheet(W, H)
            extra.rect(m, m, W - 2 * m, H - 2 * m, "border")
            draw_table(extra, chunk, m + 10, H - m - 4, f"{part.name} - HOLE TABLE (cont. {k}/{len(chunks)})")
            sheets.append(extra)
    else:
        sh_.text(rx0, table_top - 1, "No round holes detected", 2.6, "left", "top")

    # cut-profile view: look along the thinnest axis
    dims = [L, Wd, Ht]
    thin = int(np.argmin(dims))
    srt = sorted(dims)
    plate = srt[0] <= 12 and srt[0] <= 0.2 * srt[1]
    info = dict(scale=s, size=(L, Wd, Ht), holes=len(rows), volume=v_, plate=plate,
                thickness=srt[0], cut_view={0: side_view, 1: "front", 2: "top"}[thin],
                proj=proj)
    return sheets, info


# ----------------------------------------------------------------------------
# 6. Rendering: PDF (matplotlib) and DXF (ezdxf)
# ----------------------------------------------------------------------------

STYLE = {  # layer: (linewidth pt, linestyle, color)
    "visible": (0.75, "-", "black"),
    "hidden": (0.35, (0, (5, 3)), "#555555"),
    "center": (0.25, (0, (12, 3, 2, 3)), "#1f5fbf"),
    "dim": (0.3, "-", "#1f5fbf"),
    "thin": (0.3, "-", "black"),
    "border": (0.8, "-", "black"),
    "text": (0.3, "-", "black"),
    "cut": (0.5, "-", "black"),
}


def render_pdf(sheet, fig=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Arc, Circle, Polygon

    fig = plt.figure(figsize=(sheet.w / 25.4, sheet.h / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, sheet.w)
    ax.set_ylim(0, sheet.h)
    ax.set_aspect("equal")
    ax.axis("off")
    segs: dict[str, list] = {}
    for kind, layer, d in sheet.items:
        lw, ls, col = STYLE.get(layer, STYLE["thin"])
        if kind == "L":
            segs.setdefault(layer, []).append([(d[0], d[1]), (d[2], d[3])])
        elif kind == "P":
            segs.setdefault(layer, []).append(d)
        elif kind == "C":
            ax.add_patch(Circle((d[0], d[1]), d[2], fill=False, lw=lw, ls=ls, ec=col))
        elif kind == "A":
            ax.add_patch(Arc((d[0], d[1]), 2 * d[2], 2 * d[2], theta1=d[3], theta2=d[4], lw=lw, ls=ls, ec=col))
        elif kind == "T":
            ax.add_patch(Polygon(d, closed=True, fc=col, ec=col, lw=0.2))
        elif kind == "X":
            x, y, s, h, ha, va, rot, bold = d
            ax.text(x, y, s, fontsize=h * 72 / 25.4 / 0.72, ha=ha, va=va, rotation=rot,
                    color=col, fontweight="bold" if bold else "normal", family="DejaVu Sans")
    for layer, lst in segs.items():
        lw, ls, col = STYLE.get(layer, STYLE["thin"])
        ax.add_collection(LineCollection(lst, linewidths=lw, linestyles=ls, colors=col))
    return fig


def render_dxf(sheets, path, scale):
    import ezdxf
    from ezdxf.enums import TextEntityAlignment
    doc = ezdxf.new("R2010", setup=True, units=4)  # 4 = mm
    k = 1.0 / scale  # paper mm -> true model mm
    doc.header["$LTSCALE"] = max(k, 0.01)
    layers = {"visible": (7, "CONTINUOUS", 50), "hidden": (8, "DASHED", 25), "center": (5, "CENTER", 18),
              "dim": (5, "CONTINUOUS", 18), "thin": (7, "CONTINUOUS", 18), "border": (7, "CONTINUOUS", 50),
              "text": (7, "CONTINUOUS", 18)}
    for name, (color, lt, lw) in layers.items():
        doc.layers.add(name.upper(), color=color, linetype=lt, lineweight=lw)
    msp = doc.modelspace()
    align = {("left", "bottom"): TextEntityAlignment.BOTTOM_LEFT, ("center", "bottom"): TextEntityAlignment.BOTTOM_CENTER,
             ("left", "center"): TextEntityAlignment.MIDDLE_LEFT, ("center", "center"): TextEntityAlignment.MIDDLE_CENTER,
             ("left", "top"): TextEntityAlignment.TOP_LEFT, ("center", "top"): TextEntityAlignment.TOP_CENTER}
    y_shift = 0.0
    for sheet in sheets:
        for kind, layer, d in sheet.items:
            a = {"layer": layer.upper()}
            if kind == "L":
                msp.add_line((d[0] * k, (d[1] + y_shift) * k), (d[2] * k, (d[3] + y_shift) * k), dxfattribs=a)
            elif kind == "P":
                msp.add_lwpolyline([(x * k, (y + y_shift) * k) for x, y in d], dxfattribs=a)
            elif kind == "C":
                msp.add_circle((d[0] * k, (d[1] + y_shift) * k), d[2] * k, dxfattribs=a)
            elif kind == "A":
                msp.add_arc((d[0] * k, (d[1] + y_shift) * k), d[2] * k, d[3], d[4], dxfattribs=a)
            elif kind == "T":
                pts = [(x * k, (y + y_shift) * k) for x, y in d]
                msp.add_solid([pts[0], pts[1], pts[2], pts[2]], dxfattribs=a)
            elif kind == "X":
                x, y, s, h, ha, va, rot, _ = d
                t = msp.add_text(s, height=h * k * 0.75, rotation=rot, dxfattribs=a)
                t.set_placement((x * k, (y + y_shift) * k), align=align.get((ha, va), TextEntityAlignment.BOTTOM_LEFT))
        y_shift -= sheet.h + 20  # extra sheets placed below
    doc.saveas(path)


def write_cut_dxf(info, path):
    import ezdxf
    vis, _ = info["proj"][info["cut_view"]]
    x0, y0, _, _ = prims_bbox(vis)
    doc = ezdxf.new("R2010", units=4)
    doc.layers.add("CUT", color=7)
    msp = doc.modelspace()
    a = {"layer": "CUT"}
    for p in vis:
        if p[0] == "L":
            msp.add_line((p[1] - x0, p[2] - y0), (p[3] - x0, p[4] - y0), dxfattribs=a)
        elif p[0] == "C":
            msp.add_circle((p[1] - x0, p[2] - y0), p[3], dxfattribs=a)
        elif p[0] == "A":
            msp.add_arc((p[1] - x0, p[2] - y0), p[3], p[4], p[5], dxfattribs=a)
        elif p[0] == "P":
            msp.add_lwpolyline([(x - x0, y - y0) for x, y in p[1]], dxfattribs=a)
    doc.saveas(path)


def write_step(shape, path):
    with quiet():
        w = STEPControl_Writer()
        w.Transfer(shape, STEPControl_AsIs)
        w.Write(path)


def write_stl(shape, path):
    BRepMesh_IncrementalMesh(shape, 0.05, False, 0.3, True)
    StlAPI_Writer().Write(shape, path)


def bom_page(rows, hw_rows, src, W, H):
    sh = Sheet(W, H)
    m = 10
    sh.rect(m, m, W - 2 * m, H - 2 * m, "border")
    sh.text(m + 8, H - m - 8, f"BILL OF MATERIALS - {src}", 5, "left", "top", bold=True)
    sh.text(m + 8, H - m - 16, f"{len(rows)} manufactured part(s) with drawings, {len(hw_rows)} hardware item(s). "
            f"Generated {_dt.date.today().isoformat()} by step2drawings.py", 2.6, "left", "top")
    cols = [("ITEM", 14), ("PART", 95), ("QTY", 14), ("SIZE X×Y×Z (mm)", 50), ("HOLES", 16), ("TYPE", 60), ("FILES", 120)]
    y = H - m - 30
    x = m + 8
    cx = x
    for n, w in cols:
        sh.text(cx, y, n, 2.6, "left", "center", bold=True); cx += w
    sh.line(x, y - 3, W - m - 8, y - 3, "thin")
    for r in rows + hw_rows:
        y -= 5.5
        if y < m + 8:
            break
        cx = x
        for (n, w), v in zip(cols, r):
            sh.text(cx, y, str(v)[:int(w / 1.9)], 2.4, "left", "center"); cx += w
    return sh


# ----------------------------------------------------------------------------
# 7. Main
# ----------------------------------------------------------------------------

def safe(name):
    return re.sub(r"[^A-Za-z0-9_\-]+", "_", name).strip("_")[:60] or "part"


def main():
    ap = argparse.ArgumentParser(description="Make 2D manufacturing drawings for every part in a STEP assembly.")
    ap.add_argument("step", help="assembly .step/.stp file")
    ap.add_argument("-o", "--out", default="drawings", help="output folder")
    ap.add_argument("--projection", choices=["first", "third"], default="first",
                    help="first angle (India/Europe, default) or third angle (USA)")
    ap.add_argument("--sheet", choices=list(SHEETS), default="A3")
    ap.add_argument("--no-hidden", action="store_true", help="do not draw hidden (dashed) lines")
    ap.add_argument("--include-hardware", action="store_true", help="also draw screws/nuts/bearings")
    ap.add_argument("--no-3d", action="store_true", help="skip per-part STEP/STL export")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    os.makedirs(args.out, exist_ok=True)
    src = os.path.basename(args.step)
    print(f"Reading {src} ...")
    with quiet():
        parts = read_step_parts(args.step)
    print(f"  {len(parts)} part definition(s), {sum(p.qty for p in parts)} instance(s)")
    used = set()
    for p in parts:
        p.shape = canonical_orient(p.shape)
        base = re.sub(r"_body\d+$", "", p.name)
        if GENERIC_NAME.match(base.strip()):
            x0, y0, z0, x1, y1, z1 = tight_bbox(p.shape)
            d = sorted([x1 - x0, y1 - y0, z1 - z0], reverse=True)
            kind = "plate" if d[2] <= 12 and d[2] <= 0.2 * d[1] else "part"
            nm = f"{kind}_{fmt(d[0])}x{fmt(d[1])}x{fmt(d[2])}"
            p.name = nm
        n, k = p.name, 2
        while p.name in used:
            p.name = f"{n}_{k}"; k += 1
        used.add(p.name)
    parts = merge_identical(parts)
    print(f"  {len(parts)} unique part(s) after merging identical geometry")

    to_draw = [p for p in parts if args.include_hardware or not p.hardware]
    hardware = [p for p in parts if p.hardware and not args.include_hardware]
    to_draw.sort(key=lambda p: -props(p.shape)[0])  # biggest parts first

    W, H = SHEETS[args.sheet]
    bom_rows, hw_rows, csv_rows = [], [], []
    all_sheets = []
    for i, p in enumerate(to_draw, 1):
        item = f"{i:02d}"
        base = f"{item}_{safe(p.name)}"
        print(f"  [{item}] {p.name} x{p.qty} ...", end=" ", flush=True)
        try:
            sheets, info = make_sheets(p, item, src, args)
        except Exception as e:  # keep going on odd geometry
            print(f"FAILED ({e})")
            continue
        files = [base + ".pdf", base + ".dxf"]
        with PdfPages(os.path.join(args.out, base + ".pdf")) as pdf:
            for sh in sheets:
                fig = render_pdf(sh)
                pdf.savefig(fig)
                plt.close(fig)
        render_dxf(sheets, os.path.join(args.out, base + ".dxf"), info["scale"])
        typ = "Machined / printed part"
        if info["plate"]:
            write_cut_dxf(info, os.path.join(args.out, base + "_CUT.dxf"))
            files.append(base + "_CUT.dxf")
            typ = f"Plate t={fmt(info['thickness'])} (laser/CNC cut)"
        if not args.no_3d:
            write_step(p.shape, os.path.join(args.out, base + ".step"))
            write_stl(p.shape, os.path.join(args.out, base + ".stl"))
            files += [base + ".step", base + ".stl"]
        if p.mirror_of:
            typ += f"; mirror of {p.mirror_of}"
        all_sheets.append(sheets)
        L, Wd, Ht = info["size"]
        size = f"{fmt(L)} × {fmt(Wd)} × {fmt(Ht)}"
        bom_rows.append([item, p.name, p.qty, size, info["holes"], typ, ", ".join(f.split(".")[-1] for f in files)])
        csv_rows.append([item, p.name, p.qty, fmt(L), fmt(Wd), fmt(Ht), info["holes"],
                         fmt(info["volume"] / 1000), typ, " ".join(files)])
        print(f"scale {scale_label(info['scale'])}, {info['holes']} holes")
    for p in hardware:
        x0, y0, z0, x1, y1, z1 = tight_bbox(p.shape)
        size = f"{fmt(x1-x0)} × {fmt(y1-y0)} × {fmt(z1-z0)}"
        hw_rows.append(["HW", p.name, p.qty, size, "-", "Purchased hardware (no drawing)", "-"])
        csv_rows.append(["HW", p.name, p.qty, fmt(x1 - x0), fmt(y1 - y0), fmt(z1 - z0), "", "", "Purchased hardware", ""])

    with open(os.path.join(args.out, "BOM.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["item", "part", "qty", "size_x_mm", "size_y_mm", "size_z_mm", "holes", "volume_cm3", "type", "files"])
        w.writerows(csv_rows)
    with PdfPages(os.path.join(args.out, "ALL_DRAWINGS.pdf")) as pdf:
        fig = render_pdf(bom_page(bom_rows, hw_rows, src, W, H))
        pdf.savefig(fig); plt.close(fig)
        for sheets in all_sheets:
            for sh in sheets:
                fig = render_pdf(sh)
                pdf.savefig(fig); plt.close(fig)
    print(f"Done -> {os.path.abspath(args.out)}  (open ALL_DRAWINGS.pdf)")


if __name__ == "__main__":
    main()
