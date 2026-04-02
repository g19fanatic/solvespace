# Finding 10: NURBS Surface Generation for Extrusion in SolveSpace

## Summary

This document traces the complete pipeline by which SolveSpace generates NURBS surfaces
from a 2D profile (sketch) during extrusion, documenting the exact data structures,
method signatures, and control-point math. Understanding this pipeline is essential for
designing the `SShell::MakeFromChamferOf()` and `SShell::MakeFromFilletOf()` methods.

---

## 1. Entry Point: `Group::GenerateShellAndMesh()` (`src/groupmesh.cpp:212`)

For an `EXTRUDE` group, the code path is:

```
Group::GenerateShellAndMesh()
  └─ if(type == Type::EXTRUDE && haveSrc)
       ├─ get src = SK.GetGroup(opA)      // the drawing group
       ├─ translate = Vector{h.param(0..2)} // extrude direction/length
       ├─ compute tbot, ttop based on subtype (ONE_SIDED vs TWO_SIDED)
       ├─ loop over src->bezierLoops (SBezierLoopSetSet)
       │    └─ for each SBezierLoopSet *sbls:
       │         thisShell.MakeFromExtrusionOf(sbls, tbot, ttop, color)
       └─ annotate face entities (REMAP_TOP, REMAP_BOTTOM, REMAP_LINE_TO_FACE)
```

Key source locations:
- `src/groupmesh.cpp:212` — `GenerateShellAndMesh()` definition
- `src/groupmesh.cpp:235` — EXTRUDE branch begins
- `src/groupmesh.cpp:244` — `thisShell.MakeFromExtrusionOf(sbls, tbot, ttop, color)`

### Subtype Variants

| Subtype | tbot | ttop |
|---------|------|------|
| `ONE_SIDED` | `{0,0,0}` | `translate * 2` |
| `ONE_SKEWED` | `{0,0,0}` | `translate * 2` |
| `TWO_SIDED` | `translate * -1` | `translate * 1` |

The translate vector comes from solver params `h.param(0,1,2)` — 3 scalar params per EXTRUDE group.

---

## 2. `SShell::MakeFromExtrusionOf()` (`src/srf/shell.cpp`)

**Signature:**
```cpp
void SShell::MakeFromExtrusionOf(SBezierLoopSet *sbls, Vector t0, Vector t1, RgbaColor color)
```

**What it does** — builds a complete closed B-rep shell:

### Step A: Compute bounding box coordinate system

```cpp
Vector n = sbls->normal.ScaledBy(-1);  // inward normal of sketch plane
Vector u = n.Normal(0), v = n.Normal(1); // orthogonal basis
Vector orig = sbls->point;              // a point on sketch plane
// ...get umin,umax,vmin,vmax via GetBoundingProjd...
// Normalize so u,v span [0,1]
orig = orig.Plus(u.ScaledBy(umin)).Plus(v.ScaledBy(vmin));
u = u.ScaledBy(umax - umin);
v = v.ScaledBy(vmax - vmin);
```

### Step B: Generate top and bottom cap surfaces

```cpp
SSurface s0 = SSurface::FromPlane(orig.Plus(t0), u, v);    // bottom cap
SSurface s1 = SSurface::FromPlane(orig.Plus(t1).Plus(u),   // top cap (mirrored)
                                   u.ScaledBy(-1), v);
s0.color = s1.color = color;
hSSurface hs0 = surface.AddAndAssignId(&s0);
hSSurface hs1 = surface.AddAndAssignId(&s1);
```

**Critical note**: s1 is generated with `u.ScaledBy(-1)` to reverse the normal direction —
this ensures the top face normal points upward (outward) while the bottom face also points
outward. This is the fundamental B-rep outward-normal convention.

### Step C: For each Bezier curve in each loop, generate a side surface

```cpp
for(sbl in sbls->l) {
    for(sb in sbl->l) {
        // 1. Generate side surface (extrusion of sb from t0 to t1)
        SSurface ss = SSurface::FromExtrusionOf(sb, t0, t1);
        hSSurface hsext = surface.AddAndAssignId(&ss);

        // 2. Generate bottom trim curve (sb translated by t0)
        SCurve sc0; sc0.exact = sb->TransformedBy(t0, ...); ...
        sc0.surfA = hs0; sc0.surfB = hsext;
        hSCurve hc0 = curve.AddAndAssignId(&sc0);

        // 3. Generate top trim curve (sb translated by t1)
        SCurve sc1; sc1.exact = sb->TransformedBy(t1, ...); ...
        sc1.surfA = hs1; sc1.surfB = hsext;
        hSCurve hc1 = curve.AddAndAssignId(&sc1);

        // 4. Apply trims to top/bottom caps
        STrimBy stb0 = STrimBy::EntireCurve(this, hc0, false);
        STrimBy stb1 = STrimBy::EntireCurve(this, hc1, true);   // backwards
        (surface.FindById(hs0))->trim.Add(&stb0);
        (surface.FindById(hs1))->trim.Add(&stb1);

        // 5. Apply trims to side surface (top and bottom edges)
        stb0 = STrimBy::EntireCurve(this, hc0, true);   // backwards
        stb1 = STrimBy::EntireCurve(this, hc1, false);
        (surface.FindById(hsext))->trim.Add(&stb0);
        (surface.FindById(hsext))->trim.Add(&stb1);

        // 6. Generate vertical (side-to-side) trim line at Finish() of sb
        Vector pt = sb->Finish();
        SCurve scl; scl.exact = SBezier::From(pt.Plus(t0), pt.Plus(t1));
        hSCurve hl = curve.AddAndAssignId(&scl);
        trimLines.Add({hl, hsext});
    }

    // Step D: Connect adjacent side surfaces via vertical trim lines
    for(i in 0..trimLines.n) {
        TrimLine *tl = &trimLines[i];         // end of curve i
        TrimLine *tlp = &trimLines[WRAP(i-1, n)]; // end of curve i-1

        // Vertical trim line trims both adjacent side surfaces
        (surface.FindById(tl->hs))->trim.Add(STrimBy::EntireCurve(this, tl->hc, true));
        (surface.FindById(tl->hs))->trim.Add(STrimBy::EntireCurve(this, tlp->hc, false));

        // Each vertical line separates two side surfaces
        (curve.FindById(tl->hc))->surfA = ss->h;
        (curve.FindById(tlp->hc))->surfB = ss->h;
    }
}
```

---

## 3. `SSurface::FromExtrusionOf()` (`src/srf/surface.cpp:11`)

**Signature:**
```cpp
static SSurface SSurface::FromExtrusionOf(SBezier *sb, Vector t0, Vector t1)
```

**Math:**

```cpp
ret.degm = sb->deg;    // m-degree = input curve degree (1, 2, or 3)
ret.degn = 1;          // n-degree = 1 (linear in extrusion direction)

for(i = 0; i <= ret.degm; i++) {
    ret.ctrl[i][0] = sb->ctrl[i].Plus(t0);   // bottom row
    ret.ctrl[i][1] = sb->ctrl[i].Plus(t1);   // top row
    ret.weight[i][0] = sb->weight[i];         // same weight both rows
    ret.weight[i][1] = sb->weight[i];
}
```

**Key insight**: The extrusion surface is a **bilinear tensor product** of the input curve
with a line segment from t0 to t1. For a linear input (degm=1), this is a bilinear patch.
For a degree-2 arc (degm=2), this is a cylindrical NURBS patch (exact representation of
a cylinder if the input is a rational quadratic arc representing a circle).

**Weight preservation**: The NURBS weight from the input curve is preserved unchanged for
both rows. This is correct because the extrusion direction is linear (weight=1.0 for linear
direction), and the rational representation of the profile curve is faithfully propagated.

---

## 4. `SSurface::FromPlane()` (`src/srf/surface.cpp:80`)

**Signature:**
```cpp
static SSurface SSurface::FromPlane(Vector pt, Vector u, Vector v)
```

**Math:**
```cpp
ret.degm = 1; ret.degn = 1;
ret.weight[0][0] = ret.weight[0][1] = 1;
ret.weight[1][0] = ret.weight[1][1] = 1;
ret.ctrl[0][0] = pt;
ret.ctrl[0][1] = pt.Plus(u);
ret.ctrl[1][0] = pt.Plus(v);
ret.ctrl[1][1] = pt.Plus(v).Plus(u);
```

This is a degree (1,1) bilinear patch (a flat plane, exact). Note that the parameterization
spans the range [0,1] in both u and v, and the four control points are the four corners of
the plane.

**Used by:** Cap surfaces (top/bottom of extrusion), planar faces in revolutions,
**and would be used by:** Chamfer face surfaces (degree (1,1) planar patch).

---

## 5. `SSurface::FromRevolutionOf()` (`src/srf/surface.cpp:52`)

**Signature:**
```cpp
static SSurface SSurface::FromRevolutionOf(SBezier *sb, Vector pt, Vector axis,
                                            double thetas, double thetaf,
                                            double dists, double distf)
```

**Math:**
- `degm = sb->deg` (degree of profile curve)
- `degn = 2` (always degree-2 in the revolve direction — exact circle representation)
- Uses the standard quadratic NURBS arc formula for each row:

```cpp
double w = cos(dtheta / 2);  // weight for middle control point

for(i = 0; i <= degm; i++) {
    Vector p = sb->ctrl[i];
    Vector ps = p.RotatedAbout(pt, axis, thetas);  // start
    Vector pf = p.RotatedAbout(pt, axis, thetaf);  // finish
    Vector mid = ps.Plus(pf).ScaledBy(0.5);
    Vector c   = ps.ClosestPointOnLine(pt, axis);   // foot on axis
    Vector ct  = mid.Minus(c).ScaledBy(1/(w*w)).Plus(c); // middle ctrl pt

    ret.ctrl[i][0] = ps.Plus(axis.ScaledBy(dists));
    ret.ctrl[i][1] = ct.Plus(axis.ScaledBy((dists+distf)/2));
    ret.ctrl[i][2] = pf.Plus(axis.ScaledBy(distf));

    ret.weight[i][0] = sb->weight[i];
    ret.weight[i][1] = sb->weight[i] * w;  // NURBS arc weight
    ret.weight[i][2] = sb->weight[i];
}
```

**Key NURBS formula**: `w = cos(dtheta/2)` is the standard formula for the rational weight
that makes the quadratic NURBS exactly represent a circular arc. For 90°, `w = cos(45°) = √2/2 ≈ 0.707`.

**For chamfer/fillet**: This formula also applies to fillet surface generation! A fillet
surface for two planar faces at dihedral angle θ is a cylindrical NURBS surface with
extrusion direction (the edge), and the cross-section is a circular arc of angle (π - θ).

---

## 6. `MakeFirstOrderRevolvedSurfaces()` Post-Processing (`src/srf/shell.cpp`)

After building revolution surfaces, `MakeFirstOrderRevolvedSurfaces()` is called to
canonicalize degree-(1,2) surfaces:

- If a line is revolved and the line is parallel to the axis → result is a **plane** → converts to degree (1,1)
- If the line is parallel and perpendicular distance is constant → result is a **cylinder** → **transposes** to become degree (2,1) (extrusion form!)

```cpp
// Cylinder case: transpose so degm=2, degn=1
sn.degm = 2; sn.degn = 1;
for(dm = 0; dm <= 1; dm++)
    for(dn = 0; dn <= 2; dn++) {
        sn.ctrl[dn][dm]   = srf->ctrl[1-dm][dn];
        sn.weight[dn][dm] = srf->weight[1-dm][dn];
    }
```

**Why**: After transposition, `IsCylinder()` can call `IsExtrusion()` which requires `degn==1`.
This canonical form is critical for STEP export, where cylinders get special treatment.

---

## 7. `STrimBy::EntireCurve()` — Creating Trim References

**Signature (surface.h:248):**
```cpp
static STrimBy EntireCurve(SShell *shell, hSCurve hsc, bool backwards)
```

**What it does:**
```cpp
STrimBy ret;
SCurve *sc = shell->curve.FindById(hsc);
ret.curve = hsc;
ret.backwards = backwards;
if(!backwards) {
    ret.start  = sc->pts[0].p;           // first PWL point
    ret.finish = sc->pts[sc->pts.n-1].p; // last PWL point
} else {
    ret.start  = sc->pts[sc->pts.n-1].p;
    ret.finish = sc->pts[0].p;
}
return ret;
```

**Contract:** A trim must have `start == finish` of adjacent trim on the same surface.
The `backwards` flag controls direction: two adjacent surfaces sharing a curve must trim
that curve in opposite directions (one forward, one backward) for consistent orientation.

---

## 8. The SCurve Data Structure

Every SCurve encodes exactly which two surfaces it separates:

```cpp
class SCurve {
    hSCurve         h;        // its own handle
    hSCurve         newH;     // new handle after Boolean copying
    bool            isExact;  // true = has a rational polynomial exact form
    SBezier         exact;    // the exact NURBS curve
    List<SCurvePt>  pts;      // PWL approximation (used for trimming)
    hSSurface       surfA;    // first surface it separates
    hSSurface       surfB;    // second surface it separates
};
```

**Invariant:** Every SCurve must be referenced by exactly the two surfaces `surfA` and `surfB`
via STrimBy entries. Violation produces an open shell (non-watertight).

---

## 9. Mapping to Chamfer/Fillet Surface Generation

### Chamfer Surface

A chamfer is a flat cut at angle 45° (or arbitrary angle) along an edge. For the simple case
of two adjacent planar faces meeting at a right-angle edge:

**Input:**
- Edge = line segment from vertex A to vertex B
- Face1: plane with normal n1
- Face2: plane with normal n2
- Chamfer distance d (equal-leg)

**Geometry:**
- On Face1: move distance d from the edge along the surface → point set P1(t)
- On Face2: move distance d from the edge along the surface → point set P2(t)
- The chamfer surface = ruled surface from P1(t) to P2(t)

**NURBS representation:**
```cpp
// At each parameter t along the edge:
// p1(t) = edge_point(t) + d * (n1 x edge_dir).normalized()
// p2(t) = edge_point(t) + d * (n2 x edge_dir).normalized()
// chamfer_surface = SSurface::FromExtrusionOf(line_p1_to_p2, edge_t0, ...)
// ... but this needs a sweep, not a simple extrusion
```

More concretely, for a straight edge (degree-1 curve):
- The intersection lines on each face are line segments → degree-1
- The chamfer surface is a **bilinear patch** (degree (1,1)) → `SSurface::FromPlane()` works
- But only when the edge is straight AND both faces are flat

For a curved edge, the chamfer surface requires a more general approach:
- Build a `SBezier` for P1(t) and P2(t) as degree-1 curves (if edge is straight)
- Use `SSurface::FromExtrusionOf(P1_to_P2_at_t0, edge_delta_t0_to_t1, ...)` — no, this
  doesn't generalize well
- Better: use `SSurface::FromPlane()` for each planar face pair with straight edge

**For MVP (straight edges, flat faces):**
```cpp
// Four corner points of chamfer surface:
// Corner A1 = edgeA + d * perp1   (on face1 side of edge start)
// Corner A2 = edgeA + d * perp2   (on face2 side of edge start)
// Corner B1 = edgeB + d * perp1   (on face1 side of edge end)
// Corner B2 = edgeB + d * perp2   (on face2 side of edge end)
// chamferSurface = SSurface::FromPlane(A1, A2-A1, B1-A1)
// -- but this is only correct if it's truly a planar quadrilateral
```

Actually for straight edge with flat faces: the four corners ARE coplanar → exactly
representable as `SSurface::FromPlane()`.

### Fillet Surface

A fillet at 90° dihedral (two perpendicular flat faces) is a **quarter cylinder**:

**Input:**
- Edge from vertex A to vertex B, direction `edgeDir`
- Faces with normals n1, n2 (90° between them)
- Fillet radius r

**Geometry:**
- Cylinder axis = the edge itself (direction `edgeDir`)
- Cylinder center at each point along edge: `center(t) = edge(t) + r*n1 + r*n2`
  (for 90° dihedral)
- Fillet surface = part of this cylinder from (n1 side) to (n2 side)

**NURBS representation:**
```cpp
// The fillet arc at any cross-section (perpendicular to edge):
// P0 = center + r * (-n1) = point on face1
// P1 = corner (intersection of face1 and face2 extended)
// P2 = center + r * (-n2) = point on face2
// weight[1] = cos(dtheta/2) where dtheta = angle of arc = pi - dihedral_angle

// For 90° dihedral: dtheta = 90°, weight = cos(45°) = sqrt(2)/2
SBezier arc = SBezier::From(P0, P1, P2);
arc.weight[1] = cos(PI/4); // = sqrt(2)/2

// Fillet surface = extrude this arc along the edge
SSurface filletSurf = SSurface::FromExtrusionOf(&arc, edgeVec_start, edgeVec_end);
// Result: degm=2, degn=1 — a cylindrical NURBS surface
```

**This is exactly what SolveSpace uses for cylinders from revolving lines!**
After `MakeFirstOrderRevolvedSurfaces()`, cylinders become `degm=2, degn=1` extrusions of arcs.
The fillet surface would be constructed directly using `SSurface::FromExtrusionOf()` with a
quadratic Bezier arc as input.

---

## 10. Complete Topology for a Simple Box Chamfer (One Edge)

Suppose we have a unit cube and we chamfer one top edge with distance d.

**Before chamfer:** 6 faces, 12 edges, 8 vertices
**After chamfer:** 7 faces (6 original modified + 1 new chamfer face), 14 edges, 10 vertices

**Shell construction via `MakeFromChamferOf(SShell *src, hSSurface srcFace1, hSSurface srcFace2, double d)`:**

1. Copy all surfaces from src: `MakeFromCopyOf(src)` (start point)
2. Find the edge (SCurve) where srcFace1 and srcFace2 meet
3. Compute P1(t) = setback points on face1 (line parallel to edge, distance d)
4. Compute P2(t) = setback points on face2 (line parallel to edge, distance d)
5. Create chamfer surface: `SSurface::FromPlane(P1A, P2A-P1A, P1B-P1A)`
6. Modify face1's trim: remove the original edge trim, add the setback line trim
7. Modify face2's trim: same
8. Add the two new boundary curves (setback lines as SCurves on face1/chamfer and face2/chamfer)
9. Create 4 vertex triangles (if edge was bounded by vertices) using degenerate SCurves

**Watertightness requirement:** After modification, every SCurve must be referenced
by exactly 2 surfaces.

---

## 11. Key Insights for Implementation

### Surface Generation is Simple
`SSurface::FromPlane()` for chamfer, `SSurface::FromExtrusionOf(arc, ...)` for fillet —
both are single function calls using existing code.

### Trim Polygon Modification is Hard
Modifying the trim polygons of the two adjacent faces is the hard part:
- Must find the shared edge SCurve
- Must create new SCurves for the setback lines
- Must remove the old edge trim and add new trims to both original faces
- Must add trims to the new chamfer/fillet surface
- Must handle the vertex "cap" triangles at each end of the edge

### No Solver Needed for Geometry
Unlike other groups, chamfer/fillet geometry is **computed directly** from the source
shell geometry + the parameter (d or r). No solver variables are needed for the geometry
itself. The parameter `valA` is simply stored and read directly.

### Edge Identification Challenge
The primary challenge is: given two `hSSurface` handles (or some other identification),
find the `SCurve` that separates them. This is a traversal:

```cpp
for(SCurve &sc : shell->curve) {
    if((sc.surfA == hA && sc.surfB == hB) ||
       (sc.surfA == hB && sc.surfB == hA)) {
        // found the shared edge
    }
}
```

For chamfer MVP: user selects an edge directly (by clicking in the viewport, which gives
an `SEdge` or `SCurve`), so the identification is straightforward.

---

## 12. File Locations Summary

| File | Location | Purpose |
|------|----------|---------|
| `src/srf/surface.h` | Lines 192-445 | SSurface, SCurve, STrimBy, SShell declarations |
| `src/srf/surface.cpp` | Lines 11-106 | FromExtrusionOf, FromRevolutionOf, FromPlane |
| `src/srf/shell.cpp` | ~Lines 1-350 | SShell::MakeFromExtrusionOf, MakeFromRevolutionOf |
| `src/groupmesh.cpp` | Lines 212-340 | Group::GenerateShellAndMesh, EXTRUDE branch |
| `src/srf/boolean.cpp` | Lines 1-1000 | MakeFromBoolean (complex reference) |

---

## 13. Summary of Key Numbers

| Surface Type | degm | degn | Method | Notes |
|-------------|------|------|--------|-------|
| Extrusion side | profile.deg | 1 | `FromExtrusionOf` | Exact |
| Top/bottom cap | 1 | 1 | `FromPlane` | Exact plane |
| Revolution side | profile.deg | 2 | `FromRevolutionOf` | Exact rational |
| Cylinder (post-process) | 2 | 1 | (transposed) | Exact cylinder = extrusion of arc |
| **Chamfer** | 1 | 1 | `FromPlane` | Exact plane (flat faces) |
| **Fillet** | 2 | 1 | `FromExtrusionOf(arc,...)` | Exact cylinder |

---

## 14. Pseudo-Code for `SShell::MakeFromChamferOf()`

```cpp
void SShell::MakeFromChamferOf(SShell *src, SCurve *edge, double d) {
    // 1. Start from a copy of the source shell
    this->MakeFromCopyOf(src);

    // 2. Find the two surfaces sharing this edge
    hSSurface hA = edge->surfA, hB = edge->surfB;
    SSurface *faceA = this->surface.FindById(hA);
    SSurface *faceB = this->surface.FindById(hB);

    // 3. Compute setback geometry (for straight edge, flat faces)
    Vector edgeA = edge->pts[0].p, edgeB = edge->pts[edge->pts.n-1].p;
    Vector edgeDir = edgeB.Minus(edgeA).WithMagnitude(1);

    // Compute perpendicular setback direction on each face
    Vector nA = faceA->NormalAt(0.5, 0.5).WithMagnitude(1);
    Vector nB = faceB->NormalAt(0.5, 0.5).WithMagnitude(1);
    Vector setbackA = (edgeDir.Cross(nA)).WithMagnitude(d);  // into face A
    Vector setbackB = (edgeDir.Cross(nB)).ScaledBy(-1).WithMagnitude(d); // into face B

    // 4. Compute 4 corners of chamfer surface
    Vector P_A_start = edgeA.Plus(setbackA);
    Vector P_A_end   = edgeB.Plus(setbackA);
    Vector P_B_start = edgeA.Plus(setbackB);
    Vector P_B_end   = edgeB.Plus(setbackB);

    // 5. Create chamfer surface (planar quad)
    SSurface chamfer = SSurface::FromPlane(P_A_start,
                                           P_B_start.Minus(P_A_start),
                                           P_A_end.Minus(P_A_start));
    chamfer.color = faceA->color;
    chamfer.face = Remap(Entity::NO_ENTITY, REMAP_CHAMFER_FACE).v;
    hSSurface hChamfer = this->surface.AddAndAssignId(&chamfer);

    // 6. Create 3 new SCurves:
    //    - setback line on face A
    //    - setback line on face B
    //    - (edge SCurve already exists but now separates chamfer from... nothing →
    //       it will be deleted/replaced)

    // 7. Modify trim polygons of faceA and faceB:
    //    Remove the original edge trim, add the setback line trim
    // ... (complex, see boolean.cpp for reference)

    // 8. Add vertex cap triangles at edgeA and edgeB
    // ...
}
```

---

## 15. Conclusion

The SolveSpace NURBS surface generation pipeline is:

1. **Profile curves** (SBezier, degree 1-3) from sketch entities
2. **FromExtrusionOf()**: sweeps profile along a linear direction → side surfaces
3. **FromPlane()**: creates cap surfaces for the top and bottom
4. **FromRevolutionOf()**: for revolve/lathe groups
5. **All connected** via SCurve trim curves that must form a watertight closed shell

For chamfer/fillet:
- A **chamfer surface** = `FromPlane()` (4 corners of the chamfer cut) — exact
- A **fillet surface** = `FromExtrusionOf(arc, ...)` where arc is a degree-2 NURBS arc — exact
- The **hard work** is modifying the trim polygons of the two adjacent original faces to
  reflect the setback lines where the chamfer/fillet starts

The existing `MakeFromExtrusionOf()` in `shell.cpp` serves as the **best template** for
writing `MakeFromChamferOf()` — it shows exactly how to create surfaces, add curves,
and wire up the STrimBy references.
