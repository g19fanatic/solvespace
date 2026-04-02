# Finding 12: Chamfer Mathematics and Algorithms

## Overview

A **chamfer** is a flat, angled transitional edge cut between two faces of a solid (Wikipedia: Chamfer).
Unlike a fillet (which creates a curved surface), a chamfer creates a **planar** surface that replaces
the sharp edge. This is mathematically the simplest possible form of edge modification in B-rep.

Key sources used:
- Wikipedia: Chamfer (https://en.wikipedia.org/wiki/Chamfer)
- SolveSpace source: `src/srf/surface.cpp` — `SSurface::FromPlane()` at line 114
- SolveSpace source: `src/srf/surface.h` — `SSurface` struct at line 263
- Finding 11 (fillet math) for comparative context

---

## 1. Engineering Definition (from Wikipedia)

A chamfer is:
> "A transitional edge between two faces of an object. Sometimes defined as a form of bevel,
> it is often created at a 45° angle between two adjoining right-angled faces."

Applications:
- **Assembly**: eases insertion of bolts into holes, interference fits
- **Safety**: removes sharp edges to prevent injury
- **Manufacturing**: prevents bruising of edges during assembly/disassembly
- **Aesthetics**: decorative effect in furniture, architecture

The **45° equal-leg chamfer** is the most common type. However, other types exist:
- **Equal distance (equal leg)**: `d` along each face from the edge — 45° for right-angle faces
- **Unequal distance (unequal leg)**: `d1` along face 1, `d2` along face 2
- **Distance and angle**: distance `d` from edge along one face, plus angle θ of chamfer plane

For SolveSpace MVP: **equal-leg (single parameter d)** is sufficient.

---

## 2. Chamfer Geometry: The Fundamental Difference from Fillet

The key insight is that **chamfer = planar surface**, while **fillet = cylindrical/curved surface**.

| Property | Chamfer | Fillet |
|----------|---------|--------|
| New surface type | Flat plane | Cylinder (or canal surface) |
| NURBS degree | (1,1) bilinear | (2,1) quadratic-linear |
| Parameters | offset distance d (or d1,d2) | radius r |
| Geometry exact | Yes (exactly planar) | Yes for cylinders (via rational NURBS) |
| G1 continuity | No (sharp edges at chamfer boundary) | Yes (tangent to both faces) |
| Topology | Adds 1 face, 2 new edges per edge-chamfer | Adds 1 face, 2 new edges per edge-fillet |

The chamfer is simpler to implement than a fillet because:
1. The new face is a flat plane — no rational weights needed
2. `SSurface::FromPlane()` already exists in SolveSpace
3. No arc construction required
4. All trim curves are straight lines (linear SCurves)

---

## 3. Mathematical Definition of the Chamfer Surface

### Input Parameters
- Two planar faces F₁ and F₂ meeting at edge E
- Normals n₁ and n₂ (pointing outward from the solid)
- Edge direction unit vector **t**
- Offset distance d (equal-leg chamfer): how far back on each face the chamfer starts

### Step 1: Find Setback Lines

On face F₁, the setback line L₁ is:
```
L₁ = { P + t·s : s ∈ ℝ }  where P = (any point on E) + d · (n₁ × t).Normalized() × d
```

More precisely: L₁ is the line on F₁ at distance `d` from E, measured within the face plane.

The **inward tangent direction** on F₁ toward the edge is:
```
inward₁ = t × n₁   (points from face interior toward edge E, normalized)
```
So: L₁ = E + d · inward₁ = E translated by `d` along `t × n₁`.

Similarly: L₂ = E + d · inward₂ = E + d · (-t × n₂) [opposite side]

Wait, let's be precise with sign convention:

For face F₁ with outward normal n₁:
- `t × n₁` = vector pointing INTO the solid surface along F₁, perpendicular to the edge
- This is the direction from the edge INWARD along face F₁
- Setback line L₁ = E_start + (t × n₁) * d  (start point)
                    E_end + (t × n₁) * d    (end point)

For face F₂ with outward normal n₂:
- `(-t) × n₂` = vector pointing INTO the solid surface along F₂
- Or equivalently: `n₂ × t` = inward direction along F₂
- Setback line L₂ = E_start + (n₂ × t) * d  (start point)
                    E_end + (n₂ × t) * d    (end point)

The sign depends on orientation convention; in practice, test numerically.

### Step 2: Define the Four Chamfer Surface Corners

The chamfer surface is a quadrilateral (parallelogram for equal-leg chamfer):
```
Corner A = E_start + d * inward₁   = start of L₁
Corner B = E_end   + d * inward₁   = end of L₁ 
Corner C = E_end   + d * inward₂   = end of L₂
Corner D = E_start + d * inward₂   = start of L₂
```

The chamfer surface = planar quad with corners A, B, C, D.

### Step 3: The Chamfer Normal

The chamfer plane's normal is:
```
n_chamfer = (B - A) × (D - A).Normalized()
           = t × (inward₂ - inward₁).Normalized()
```

Equivalently, the chamfer plane bisects the dihedral angle between F₁ and F₂:
```
n_chamfer = (n₁ + n₂).Normalized()   [for equal-leg chamfer]
```

This is **not** tangent to F₁ or F₂ at the contact lines — chamfers are G0, not G1.

---

## 4. NURBS Representation of the Chamfer Surface

### The SSurface::FromPlane() Function

From `src/srf/surface.cpp:114`:
```cpp
SSurface SSurface::FromPlane(Vector pt, Vector u, Vector v) {
    SSurface ret = {};

    ret.degm = 1;  // degree 1 in m direction (linear)
    ret.degn = 1;  // degree 1 in n direction (linear)

    ret.weight[0][0] = ret.weight[0][1] = 1;
    ret.weight[1][0] = ret.weight[1][1] = 1;

    ret.ctrl[0][0] = pt;           // corner 0,0
    ret.ctrl[0][1] = pt.Plus(u);   // corner 0,1
    ret.ctrl[1][0] = pt.Plus(v);   // corner 1,0
    ret.ctrl[1][1] = pt.Plus(v).Plus(u);  // corner 1,1

    return ret;
}
```

This creates a bilinear (degree 1×1) planar patch with:
- **degm = 1, degn = 1** — all weights = 1, so this is pure polynomial, not rational
- Corner at `pt`, edges along vectors `u` and `v`
- Total 2×2 = 4 control points

This is exactly what we need for a chamfer surface!

### How to Call FromPlane for a Chamfer

```cpp
// Given: corners A, B, C, D of chamfer quad
// A = E_start + d*inward1
// D = E_start + d*inward2
// B = A + edge_vec
// C = D + edge_vec

Vector pt = A;           // origin corner
Vector u = D.Minus(A);   // direction from A to D (along setback perpendicular)
Vector v = B.Minus(A);   // direction from A to B (along edge)

SSurface chamfer_surface = SSurface::FromPlane(pt, u, v);
```

Result: `degm=1, degn=1`, 4 corners, all weights=1. A perfectly planar patch.

---

## 5. Comparison of Chamfer vs Fillet NURBS Properties

| Property | Chamfer `FromPlane()` | Fillet `FromExtrusionOf(arc)` |
|----------|----------------------|-------------------------------|
| `degm` | 1 | 2 |
| `degn` | 1 | 1 |
| Control points | 2×2 = 4 | 3×2 = 6 |
| Weights | All 1 (no rational) | weight[i][1] = cos(angle/2) |
| Surface type | Exact planar | Exact cylindrical (rational) |
| Normal variation | Constant | Varies (cylindrical) |
| G1 at boundaries | No | Yes |

---

## 6. The Chamfer Algorithm for SolveSpace

### Algorithm for Each Chamfered Edge

**Input**: SCurve (edge E), two surface handles (hA for F₁, hB for F₂), distance d

**Step 1: Compute setback vectors**
```cpp
// Get edge endpoints
Vector edgeStart = E.endpoints[0];
Vector edgeEnd   = E.endpoints[1];
Vector edgeVec   = edgeEnd.Minus(edgeStart);
Vector t = edgeVec.WithMagnitude(1.0);  // unit edge direction

// Get face normals at midpoint of edge
Vector mid = edgeStart.Plus(edgeEnd).ScaledBy(0.5);
Point2d uv1, uv2;
srf1->ClosestPointTo(mid, &uv1);
srf2->ClosestPointTo(mid, &uv2);
Vector n1 = srf1->NormalAt(uv1);  // outward normal of F1
Vector n2 = srf2->NormalAt(uv2);  // outward normal of F2

// Inward directions on each face
Vector inward1 = t.Cross(n1).WithMagnitude(1.0);   // points inward on F1
Vector inward2 = n2.Cross(t).WithMagnitude(1.0);   // points inward on F2
```

**Step 2: Compute chamfer corners**
```cpp
Vector A = edgeStart.Plus(inward1.ScaledBy(d));  // start on L1
Vector B = edgeEnd.Plus(inward1.ScaledBy(d));    // end on L1
Vector C = edgeEnd.Plus(inward2.ScaledBy(d));    // end on L2
Vector D = edgeStart.Plus(inward2.ScaledBy(d));  // start on L2
```

**Step 3: Build chamfer surface**
```cpp
SSurface chamfer = SSurface::FromPlane(A, D.Minus(A), B.Minus(A));
// ctrl[0][0] = A, ctrl[0][1] = D, ctrl[1][0] = B, ctrl[1][1] = C
```

**Step 4: Build trim curves (SCurves)**
- SCurve_L1: from A to B (shared between chamfer surface and F₁)
- SCurve_L2: from D to C (shared between chamfer surface and F₂)
- SCurve_capStart: from A to D (cap at edge start — may be shared with vertex face)
- SCurve_capEnd: from B to C (cap at edge end)

**Step 5: Modify adjacent faces**
- F₁ trim polygon: remove the region from E to L₁ (clip along L₁)
- F₂ trim polygon: remove the region from E to L₂ (clip along L₂)
- The original edge E (SCurve between F₁ and F₂) is REMOVED from the shell

---

## 7. Unequal-Leg Chamfer

For unequal-leg chamfer (d1 along F₁, d2 along F₂):
```cpp
Vector A = edgeStart.Plus(inward1.ScaledBy(d1));  // d1 on F1
Vector B = edgeEnd.Plus(inward1.ScaledBy(d1));
Vector C = edgeEnd.Plus(inward2.ScaledBy(d2));    // d2 on F2
Vector D = edgeStart.Plus(inward2.ScaledBy(d2));
```

The chamfer surface is still planar (bilinear quad), just not symmetric.
The normal angle relative to each face is different.

For `d1 = d2`, this is the equal-leg chamfer with 45° angle at 90° dihedral.

---

## 8. Angle-Based Chamfer

For an "angle+distance" chamfer (distance d from edge along F₁, angle α):
- d2 = d * tan(α) — distance along F₂ from the edge
- Where α is measured from the chamfer plane to F₁

This is a straightforward algebraic relation; still `FromPlane()` for the chamfer surface.

---

## 9. Chamfer at Vertices (Corner Cases)

When multiple edges of a solid meet at a vertex V, and all are chamfered:
- Each chamfer surface terminates at a small triangular or polygonal cap face at V
- For a box corner (3 edges meeting), you get 3 chamfer rectangles + 1 small triangular face
- The triangular face = planar surface with 3 corners: the setback points from each edge

This **vertex cap face** is also created with `SSurface::FromPlane()` — another simple planar patch.

For MVP: handle this case (it's much simpler than the fillet vertex case which requires spherical patches).

---

## 10. Comparison to FreeCAD Chamfer

FreeCAD's `BRepFilletAPI_MakeChamfer` (via OpenCASCADE) uses the `Add(d1, d2, edge, face)` API where:
- `face` tells OCC which side gets `d1`
- `edge` is the edge to chamfer

In OCC's internal representation:
- Chamfer creates exactly a **planar ruled surface** connecting the two setback lines
- Type: `Geom_Plane` in OCCT geometry terms = flat, bounded

SolveSpace's equivalent: `SSurface::FromPlane(A, D-A, B-A)` — same concept, different API.

---

## 11. Key Differences from Fillet That Make Chamfer Simpler

1. **No arc math needed**: No NURBS weight computation, no dihedral angle → weight conversion
2. **No rational surface**: All weights = 1, simplifies intersection math
3. **No tangency requirement**: G0 is sufficient (chamfer has sharp edges at its boundaries)
4. **Simpler vertex case**: Triangular cap face instead of spherical patch
5. **Simpler adjacent face trimming**: Straight trim lines vs. arcs
6. **Exact with integers**: Chamfer can even be represented with exact rational arithmetic

---

## 12. Chamfer Surface Normal Direction

For the B-rep watertightness requirement, the chamfer surface normal must point **outward** from the solid.

Using `SSurface::FromPlane(A, u, v)`:
- Surface normal = (u × v).Normalized()
- This must point away from the solid interior

Verification: `n_chamfer · (n1 + n2)` should be > 0 (chamfer normal points outward, same half-space as sum of face normals).

If negative, reverse the surface (swap u and v, or call `SSurface::Reverse()`).

---

## 13. SolveSpace-Specific Implementation Notes

### SSurface::FromPlane() signature (`surface.h:299`)
```cpp
static SSurface FromPlane(Vector pt, Vector u, Vector v);
```
- `pt`: one corner of the planar patch
- `u`: edge vector for one side
- `v`: edge vector for perpendicular side
- Corners: pt, pt+u, pt+v, pt+u+v

### Already present in SolveSpace:
- `SSurface::FromPlane()` at `surface.cpp:114` — **exactly what we need**
- `IsPlane()` detection (line ~369 in surface.h) — can identify chamfer faces
- `NormalAt()` for orientation verification
- `ClosestPointTo()` for computing setback vectors

### Not yet present (needs new code):
- `SShell::MakeFromChamferOf(SShell *src, hSCurve edgeToChf, double d)` — main new function
- Trim curve clipping logic for adjacent faces

---

## 14. Chamfer Parameterization Options (for valA, valB)

In SolveSpace's Group struct:
- `Group::valA` = d1 (primary chamfer distance) — main parameter
- `Group::valB` = d2 (secondary distance for unequal-leg) — 0 means equal-leg
- OR: `Group::valA` = d (equal-leg distance), `Group::valB` = 0 (reserved)

For MVP: equal-leg only. `valA = d`, `valB = 0` (ignored).
For V2: `valA = d1`, `valB = d2`.

---

## 15. Sources

1. **Wikipedia — Chamfer**: https://en.wikipedia.org/wiki/Chamfer
   - Engineering definition: "transitional edge between two faces"
   - Types: equal distance, two distances, decorative uses
   - Manufacturing: assembly, safety, machining applications

2. **SolveSpace source — `src/srf/surface.cpp:114`**:
   - `SSurface::FromPlane(Vector pt, Vector u, Vector v)` — creates bilinear planar patch
   - `degm=1, degn=1`, all weights=1, corners at pt, pt+u, pt+v, pt+u+v

3. **SolveSpace source — `src/srf/surface.h:299`**:
   - `static SSurface FromPlane(Vector pt, Vector u, Vector v);` declaration

4. **Finding 11 (Fillet Math)** for comparative context:
   - Chamfer = degree (1,1) flat; Fillet = degree (2,1) cylindrical
   - Chamfer far simpler: no rational weights, no arc geometry, no tangency requirements

5. **FreeCAD source analysis (Finding 08)**:
   - `BRepFilletAPI_MakeChamfer::Add(d1, d2, edge, face)` — OCC chamfer API
   - Creates ruled/planar surface between two setback lines = same as `FromPlane()`

---

## Key Takeaways

1. **Chamfer surface = planar bilinear patch** — `SSurface::FromPlane()` handles this exactly
2. **No rational NURBS needed**: all weights = 1, degree (1,1)
3. **Algorithm**: compute two setback lines (L₁, L₂) at distance d from edge on each face → `FromPlane(A, D-A, B-A)` where A,B on L₁ and D,C on L₂
4. **Inward direction on F₁**: `t × n₁` where t = edge unit direction, n₁ = face outward normal
5. **Chamfer is G0** (no tangency at boundaries) — this is by design, not a deficiency
6. **Vertex caps** for chamfer are also flat triangles — still `FromPlane()`, simpler than fillet
7. **Chamfer is easier to implement than fillet**: no arc construction, no rational weights, no tangency
8. **SolveSpace already has `FromPlane()`** — the hardest part is trim polygon modification
9. **Equal-leg chamfer MVP**: single parameter d, valA=d in Group struct
10. **The "hard part"** is the same as for fillet: modifying trim polygons of adjacent faces
