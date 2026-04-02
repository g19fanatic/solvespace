# Finding 11: Fillet Mathematics and Algorithms

## Overview

A **fillet** in mechanical engineering is a rounding of an interior or exterior corner of a part
(Wikipedia: Fillet mechanics). It contrasts with a chamfer, which is a flat bevel. This document
covers:
1. The geometric definition of a fillet surface (rolling ball model)
2. The canal/pipe surface mathematical formulation
3. NURBS exact representation of cylindrical fillet surfaces
4. The specific case of fillets between two planar faces (most common in CAD)
5. How SolveSpace's existing surface machinery maps to fillet math

---

## 1. Engineering Definition

From Wikipedia (Fillet mechanics):
- **Interior fillet** (concave): rounds an inside corner — adds material
- **Exterior fillet** (round/convex): rounds an outside corner — removes material
- Applications: stress concentration reduction, aerodynamics, manufacturability (end mill access)
- Terminology varies: FreeCAD/SolidWorks call it "fillet"; Creo calls it "round"; Unigraphics calls it "blend"

For SolveSpace context: we target the **exterior fillet** (convex edge of a solid — removes material
and adds a curved face). This is the most common CAD operation (e.g., rounding a box corner).

---

## 2. Rolling Ball Algorithm

### Conceptual Definition
The rolling ball algorithm is the standard geometric definition for a constant-radius fillet:

> Imagine a sphere of radius `r` that rolls along an edge while remaining tangent to both
> adjacent faces. The surface swept by the sphere's exterior envelope defines the fillet surface.

### Mathematical Formulation
For two planar faces meeting at a dihedral angle θ (interior angle):
- The sphere center traces a **spine curve** (a straight line for two flat faces)
- The spine lies at distance `r` from both face planes
- The fillet surface = locus of all tangent points of the sphere on both faces + the cylindrical/toroidal patch between them

For two **planar** faces:
- Spine = line parallel to the edge, offset into the solid by `r / sin(θ/2)` (approximately)
- More precisely: spine offset = `r / tan(θ/2)` from each face (along each face's normal)
- The fillet surface between two planar faces is a **portion of a cylinder** (not general canal surface)

For curved faces:
- The spine becomes a curve in 3D
- The fillet surface becomes a true **canal surface** (envelope of moving sphere)

---

## 3. Canal Surface Theory (Wikipedia: Channel Surface)

A **canal surface** is the envelope of a family of spheres whose centers lie on a space curve (the directrix), with radius function r(u).

### Mathematical Definition
Given directrix **c**(u) and radius r(u), the canal surface satisfies:
- f(**x**; u) = ||**x** - **c**(u)||² - r²(u) = 0  (point on sphere)
- f_u(**x**, u) = 0  (envelope condition)

The envelope condition is:
> -(**x** - **c**(u))ᵀ **ċ**(u) - r(u)ṙ(u) = 0

This is the equation of a **plane** orthogonal to the tangent **ċ**(u) of the directrix.

### Parametric Form
x(u,v) = **c**(u) - [r(u)ṙ(u) / ||**ċ**(u)||²] **ċ**(u)
         + r(u)√(1 - ṙ(u)²/||**ċ**(u)||²) × (**e₁**(u)cos(v) + **e₂**(u)sin(v))

where **e₁**, **e₂**, **ċ**/||**ċ**|| form an orthonormal frame.

### Special Case: Pipe Surface (Constant Radius)
When ṙ = 0 (constant radius, as in a standard fillet):
x(u,v) = **c**(u) + r(**e₁**(u)cos(v) + **e₂**(u)sin(v))

This is a **pipe surface** — the rolling ball fillet with constant radius is a special case.

### Simplest Case: Cylinder (Two Planar Faces)
When the two faces are **flat planes** and the edge is a **straight line**:
- Directrix **c**(u) = P₀ + u·**t** (line parallel to edge)
- **e₁**, **e₂** = fixed vectors (no twist)
- Result: a **right circular cylinder** — the simplest case of a pipe surface

A cylinder is also a canal surface with constant radius and linear directrix.

---

## 4. NURBS Exact Representation of Fillet Surfaces

### Key NURBS Property: Exact Circles
From Wikipedia (NURBS) — a circle can be represented **exactly** as a rational B-spline:
- Quarter-circle: 3 control points, degree 2, weight of middle control point = √2/2 ≈ 0.7071
- Each quarter uses knots: {0,0,0, π/2,π/2, π,...} with double interior knots
- The middle control point lies at the intersection of the tangents at each endpoint

### General Formula for Circular Arc
For an arc spanning angle dθ:
- P₀ = start point on circle
- P₂ = end point on circle
- P₁ = intersection of tangent lines at P₀ and P₂ (the "corner" point)
- Weight of P₀ = weight of P₂ = 1
- Weight of P₁ = **cos(dθ/2)**

This is the formula SolveSpace already uses in its arc Bezier generation
(`entity.cpp:~730`: `weight[1] = cos(dtheta/2)`) and in `FromRevolutionOf` in `surface.cpp`:

```cpp
double w = cos(dtheta / 2);
// ...
ret.weight[i][0] = sb->weight[i];
ret.weight[i][1] = sb->weight[i] * w;  // middle weight
ret.weight[i][2] = sb->weight[i];
```

### Cylindrical Fillet Surface as NURBS
A cylinder of radius `r` with axis along direction **t**, spanning angle dθ:
- 3 control "columns" in the angular direction (degree 2 in v)
- N control "rows" in the axial direction (degree 1 in u for simple extrusion)
- ctrl[i][0] = point on face 1 at parameter i
- ctrl[i][1] = "corner" point = tangent intersection
- ctrl[i][2] = point on face 2 at parameter i
- weight[i][1] = cos(dθ/2) where dθ = π - (dihedral angle) (the supplement)

For a 90° exterior edge (dihedral = 90°):
- The fillet spans 90° (quarter cylinder)
- dθ = π/2, so weight[i][1] = cos(π/4) = √2/2 ≈ 0.7071

For a general dihedral angle α between face normals:
- The fillet arc spans angle = π - α (supplement of the dihedral measured inside the solid)
- weight[i][1] = cos((π - α)/2) = sin(α/2)

### SolveSpace's `FromExtrusionOf` for Fillet
The ideal NURBS representation for a **constant-radius fillet between two planar faces**
is a surface created by `SSurface::FromExtrusionOf(arc, t0, t1)` where:
- `arc` is the quadratic NURBS arc (3 control points, degree 2)
- `t0 = (0,0,0)` — no offset at start
- `t1 = edge_direction * edge_length` — extrude along the edge

This produces `degm=2, degn=1` — exactly the format SolveSpace uses for cylinders
(as seen in `IsCylinder()` which calls `IsExtrusion()` then checks if the profile is circular).

---

## 5. Detailed Fillet Construction for Two Planar Faces

### Input
- Two planar faces F₁ and F₂ meeting at edge E
- Normals n₁ and n₂ (pointing outward from the solid)
- Fillet radius r

### Step 1: Find the Offset Lines on Each Face
On face F₁: the "setback line" L₁ is parallel to E, at distance r from E within the face.
- L₁ = E translated by `r` along **n₁ × t** (where **t** is the edge direction unit vector)
- Points on L₁ are where the fillet arc touches F₁

On face F₂: similarly, L₂ at distance r from E within F₂.

### Step 2: Find the Fillet Arc
The arc center for each cross-section lies:
- At distance r from both faces
- In the plane perpendicular to **t** at each point along E

For each position along E, the arc center C satisfies:
- |C - P₁|² + (distance from E)² = [geometry] → C = E_point + r·**n_bisector**

More precisely:
- **n_bisector** = normalize(n₁ + n₂) × (-1) [points into the solid, away from the edge]
- C = P_on_edge + r/sin(φ/2) × **n_bisector**

where φ is the angle between the two face normals (= exterior dihedral angle).

The arc goes from:
- P0 = C - r·(-n₁) = C + r·n₁_perp [where arc touches F₁]
- P2 = C - r·(-n₂) = C + r·n₂_perp [where arc touches F₂]
- P1 = intersection of tangent lines at P0 and P2 = the original edge point
- Arc spans angle = π - φ (complement of the interior dihedral angle)

### Step 3: Build the NURBS Arc
```
SBezier arc;
arc.deg = 2;  // quadratic
arc.ctrl[0] = P0;  // tangent point on face 1
arc.ctrl[1] = P1;  // original edge point (corner)
arc.ctrl[2] = P2;  // tangent point on face 2
arc.weight[0] = 1.0;
arc.weight[1] = cos(arc_angle / 2);  // = sin(dihedral/2)
arc.weight[2] = 1.0;
```

### Step 4: Extrude Along the Edge
```
SSurface fillet_surface = SSurface::FromExtrusionOf(&arc, t0, t1);
// where t0 = (0,0,0), t1 = edge_direction * edge_length
```

This gives `degm=2, degn=1` — a cylinder segment.

---

## 6. Verification: SolveSpace's FromRevolutionOf vs FromExtrusionOf

From `surface.cpp`, `FromRevolutionOf` creates a `degm=sb->deg, degn=2` surface with:
```cpp
double w = cos(dtheta / 2);
ret.weight[i][1] = sb->weight[i] * w;  // middle angular weight
```

From `surface.cpp`, `FromExtrusionOf` creates a `degm=sb->deg, degn=1` surface with:
```cpp
ret.ctrl[i][0] = (sb->ctrl[i]).Plus(t0);
ret.ctrl[i][1] = (sb->ctrl[i]).Plus(t1);
```

For a fillet, we want **FromExtrusionOf** because:
- The arc is quadratic (3 control points): `sb->deg = 2`, so `degm = 2`
- The extrusion direction is along the edge: `degn = 1`
- Result: `degm=2, degn=1` which is a cylinder = exactly what `IsCylinder()` recognizes

**For a fillet surface: use `FromExtrusionOf(&arc_sb, zero_vector, edge_vector)`.**

The `FromRevolutionOf` path would give `degm=arc.deg, degn=2` which is a **torus** (revolving an arc around an axis) — that's useful for vertex roundovers and corner fillets, not edge fillets.

---

## 7. Corner Fillets (Vertex Cases)

At corners where three or more edges meet, the rolling ball algorithm requires a **spherical triangle** or similar patch. This is the hardest case:

### Options
1. **Spherical patch**: portion of a sphere of radius r; rational NURBS of degree (2,2); exact
2. **Torus patch**: `FromRevolutionOf` can create partial tori for specific corner configurations
3. **Trimmed cylinder junction**: where two cylindrical fillet surfaces are joined at a corner

For the **MVP**, corner fillets should be **deferred**. Only edge fillets (between two planar faces) need to be handled initially. A simple approach is to **not** handle vertex corners at all — just cut the fillet surfaces at the vertex plane.

---

## 8. Variable Radius Fillets

The rolling ball generalizes to variable radius: r = r(t) along the edge.
- The spine becomes a canal surface (not a cylinder)
- The fillet surface is a true canal surface, not representable as simple extrusion
- NURBS approximation is needed (not exact like constant-radius)

**For MVP: constant radius only.**

---

## 9. Tangency Conditions (G1 Continuity)

The fillet surface must be **tangent** to both adjacent faces at the contact lines:

For face F₁:
- The fillet surface normal at any point on the contact line L₁ must equal F₁'s normal
- This is automatically satisfied if the arc is constructed geometrically correctly (tangent to F₁ at P0)
- The `FromExtrusionOf` surface with the correctly-constructed arc guarantees this

For the trim curves:
- L₁ and L₂ are straight lines (for planar face pairs)
- They are SCurves shared between the fillet surface and each adjacent face
- Both adjacent faces must be trimmed (their trim polygons clipped along L₁ and L₂)

---

## 10. SolveSpace-Specific Implementation Details

### Available Surface Factory Methods (from `surface.h`)
```
static SSurface FromExtrusionOf(SBezier *spc, Vector t0, Vector t1);  // line:296
static SSurface FromRevolutionOf(SBezier *sb, ...);                    // line:297
static SSurface FromPlane(Vector pt, Vector u, Vector v);              // line:299
static SSurface FromTransformationOf(SSurface *a, ...);                // line:300
```

### SSurface Data Structure (from `surface.h`)
```cpp
int      degm, degn;          // line:282 — degrees (0-3) in m and n directions
Vector   ctrl[4][4];          // line:283 — control points (max 4×4)
double   weight[4][4];        // line:284 — NURBS weights
List<STrimBy> trim;           // trim curves that bound this surface
```

### IsCylinder Detection (from `surface.cpp`)
```cpp
bool SSurface::IsCylinder(Vector *axis, Vector *center, double *r,
                            Vector *start, Vector *finish) const
{
    SBezier sb;
    if(!IsExtrusion(&sb, axis)) return false;
    if(!sb.IsCircle(*axis, center, r)) return false;
    // ...
}
```
Fillet surfaces created with `FromExtrusionOf(arc)` where arc is circular will be recognized
as cylinders by this function.

### Key Functions for Fillet Geometry (from `surface.cpp`)

**`ExactSurfaceTangentAt`** (line ~175): computes tangent to intersection curve between two surfaces
— useful for creating trim curves where fillet meets adjacent faces.

**`TriangulateInto`** (line ~210): already handles degree-(2,1) surfaces correctly:
```cpp
if(degm == 1 && degn == 1) {
    poly.UvTriangulateInto(sm, this);   // flat surface
} else {
    poly.UvGridTriangulateInto(sm, this);  // curved surface (our case!)
}
```

---

## 11. The Fillet Algorithm Summary for SolveSpace

For each filleted edge (defined by: edge curve + two adjacent face handles + radius r):

1. **Compute setback distance**: `d = r / tan(dihedral_half_angle)` where half_angle = acos(-n1·n2)/2
   - More precisely: `d = r * |t × n1| / |n1 + n2|` 
   
2. **Find contact lines** L₁, L₂: lines on each face at distance `d` from the original edge

3. **Construct arc**: 3-point quadratic NURBS arc
   - P0 = point on L₁ at start of edge
   - P2 = point on L₂ at start of edge  
   - P1 = original edge start point
   - weight[1] = sin(interior_dihedral / 2) [using identity: cos((π-α)/2) = sin(α/2)]

4. **Extrude arc** along edge direction × edge length via `FromExtrusionOf`

5. **Build trim curves**:
   - SCurve on L₁: shared between fillet surface and F₁
   - SCurve on L₂: shared between fillet surface and F₂
   - SCurve at start cap (if edge has a vertex junction)
   - SCurve at end cap

6. **Modify adjacent faces**: clip their trim polygons to remove the filleted region
   (this is the hard part — see finding 16/boolean pipeline)

7. **Add fillet surface** to SShell

---

## 12. Dihedral Angle Calculation

To compute the angle between two faces meeting at an edge in SolveSpace:

```cpp
// Get normals at a point on the edge:
Point2d puv1, puv2;
srfA->ClosestPointTo(edge_midpoint, &puv1);  // line:338 in surface.h
srfB->ClosestPointTo(edge_midpoint, &puv2);
Vector n1 = srfA->NormalAt(puv1);  // line:349 in surface.h
Vector n2 = srfB->NormalAt(puv2);  // line:350 in surface.h

// Interior dihedral angle:
double cos_alpha = n1.Dot(n2);  // faces' normals
// For exterior edge (convex): n1·n2 < 0 (normals point away from each other)
// dihedral interior angle = π + acos(n1·n2) for convex edge
// fillet arc half-angle = acos(-n1·n2)/2
double half_angle = acos(-n1.Dot(n2)) / 2.0;
double arc_weight = sin(half_angle);  // = cos((π-dihedral)/2)
```

SolveSpace surface methods relevant:
- `SSurface::NormalAt(Point2d)` at `surface.h:349`
- `SSurface::PointAt(double u, double v)` at `surface.h:346`
- `SSurface::ClosestPointTo(Vector, Point2d*)` at `surface.h:338`

---

## 13. Sources

1. **Wikipedia — Fillet (mechanics)**: https://en.wikipedia.org/wiki/Fillet_(mechanics)
   - Engineering definition, applications, terminology
   
2. **Wikipedia — Channel surface (Canal surface)**: https://en.wikipedia.org/wiki/Canal_surface
   - Mathematical formulation of canal/pipe surfaces
   - Parametric representation: x(u,v) = c(u) + r(e₁cos(v) + e₂sin(v)) for constant r
   
3. **Wikipedia — NURBS**: https://en.wikipedia.org/wiki/NURBS
   - Exact circle representation with rational B-splines
   - Quarter-circle: weight = √2/2 for middle control point
   
4. **SolveSpace source — src/srf/surface.cpp**:
   - `FromExtrusionOf()`: creates degm=sb->deg, degn=1 surface
   - `FromRevolutionOf()`: weight formula: `w = cos(dtheta/2)`, `weight[i][1] = sb->weight[i] * w`
   - `IsCylinder()`: detects fillet surfaces by checking IsExtrusion + IsCircle
   - `NormalAt()`, `ClosestPointTo()`: for dihedral angle computation
   
5. **SolveSpace source — src/srf/surface.h**:
   - `SSurface` struct: `degm, degn`, `ctrl[4][4]`, `weight[4][4]` (line:282-284)
   - Factory methods: `FromExtrusionOf` (line:296), `FromPlane` (line:299)
   - Detection: `IsExtrusion` (line:356), `IsCylinder` (line:357)
   - Access methods: `NormalAt` (line:349-350), `ClosestPointTo` (line:338-340)

---

## Key Takeaways

1. **The common fillet (two planar faces, constant radius) is a cylinder** — exact, not approximate
2. **NURBS representation**: degree (2,1), 3×2 control points, weight[i][1] = sin(interior_dihedral/2)
3. **Use `SSurface::FromExtrusionOf(&arc, zero, edge_vec)`** — SolveSpace already has this method
4. **The arc weight formula** is already in use in `surface.cpp::FromRevolutionOf`
5. **IsCylinder() will recognize the result** — it calls `IsExtrusion` + `IsCircle`
6. **Dihedral angle** computed via cross-product of face normals using `NormalAt()`
7. **The mathematically hard part is NOT the fillet surface** — it's modifying the trim curves of adjacent faces (clipping them to the setback lines)
8. **Variable radius and corner fillets** are out of scope for MVP
9. **Canal surfaces** are the general case; cylindrical pipe surfaces are the fillet special case
10. **G1 tangency** between fillet and adjacent faces is guaranteed by geometric construction
