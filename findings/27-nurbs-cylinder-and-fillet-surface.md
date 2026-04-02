# NURBS Cylinder Representation and Fillet Surfaces in B-rep

## Task 27: How NURBS Represent a Cylinder (Fillet Result) and Use in B-rep

---

## 1. Fundamental NURBS Background

NURBS (Non-Uniform Rational B-Splines) are a mathematical framework that can exactly
represent both free-form curves/surfaces AND standard analytic shapes like circles,
cylinders, cones, and spheres. This is what makes them ideal for CAD: one representation
handles both.

### The Key Property: Rational Weights Enable Exact Conics

A non-rational B-spline (all weights = 1) can only *approximate* a circle. But a
**rational** B-spline (with weights ≠ 1 for middle control points) can represent a
circle **exactly**. This is the mathematical foundation for exact fillet geometry.

From Wikipedia's NURBS article, the exact circle representation uses:
- 9 control points, 3rd order (degree 2), divided into 4 quarter-arcs
- Weights at "off-axis" control points = `cos(π/4) = √2/2 ≈ 0.7071`
- Knot vector with double knots at each quarter boundary

For a **single quadrant arc** (0 to π/2):
- 3 control points: P0, P1, P2
- P0 = arc start (on circle)
- P2 = arc finish (on circle)  
- P1 = intersection of tangent lines at P0 and P2 (outside the circle)
- Weights: w0=1, **w1=cos(θ/2)**, w2=1 where θ is the arc angle
- For a quarter circle: w1 = cos(45°) = √2/2

This is the **exact rational Bezier arc formula** that SolveSpace uses throughout.

---

## 2. How SolveSpace Represents Cylindrical Surfaces

### 2.1 SSurface Data Structure

From `src/srf/surface.h`:
```
SSurface {
    int    degm, degn;        // degree in each parameter direction (0-3)
    Vector ctrl[4][4];        // control points (max 4×4)
    double weight[4][4];      // rational weights
    List<STrimBy> trim;       // trim curves bounding this surface
    uint32_t face;            // face entity handle for selection
    ...
}
```

A **cylinder** in SolveSpace is an **extrusion of a circular arc**:
- `degm = 2` (quadratic in the circular direction)
- `degn = 1` (linear/degree-1 along the axis)
- 3×2 control points = 6 effective control points
- `weight[m][n] = w_circle[m] * 1` (arc weight × extrusion weight of 1)

### 2.2 The IsCylinder() Method (surface.cpp:56)

```cpp
bool SSurface::IsCylinder(Vector *axis, Vector *center, double *r,
                            Vector *start, Vector *finish) const
{
    SBezier sb;
    if(!IsExtrusion(&sb, axis)) return false;        // must be extrusion (degn=1)
    if(!sb.IsCircle(*axis, center, r)) return false; // profile must be a circle arc
    *start = sb.ctrl[0];                              // start point on circle
    *finish = sb.ctrl[2];                             // end point on circle
    return true;
}
```

This detection method is used in:
- `raycast.cpp:278` — for closed-form ray-cylinder intersection
- STEP export — for cylindrical surface handling

### 2.3 The IsCircle() Method (curve.cpp:127)

```cpp
bool SBezier::IsCircle(Vector axis, Vector *center, double *r) const {
    if(deg != 2) return false;    // must be quadratic
    
    // Find center by intersecting tangent lines at P0 and P2
    Vector t0 = ctrl[0].Minus(ctrl[1]),
           t2 = ctrl[2].Minus(ctrl[1]),
           r0 = axis.Cross(t0),
           r2 = axis.Cross(t2);
    *center = Vector::AtIntersectionOfLines(ctrl[0], ctrl[0].Plus(r0),
                                            ctrl[2], ctrl[2].Plus(r2), ...);
    
    // Verify equal radii
    double rd0 = center->Minus(ctrl[0]).Magnitude(),
           rd2 = center->Minus(ctrl[2]).Magnitude();
    if(fabs(rd0 - rd2) > LENGTH_EPS) return false;
    *r = rd0;
    
    // THE KEY CHECK: verify weight[1] == cos(dtheta/2)
    double dtheta = ...; // arc sweep angle
    if(fabs(weight[1] - cos(dtheta/2)) > LENGTH_EPS) return false;
    
    return true;
}
```

**Critical formula**: `weight[1] = cos(dtheta/2)` where `dtheta` is the arc sweep angle.
- For 90° arc: w1 = cos(45°) = √2/2 ≈ 0.7071
- For 60° arc: w1 = cos(30°) ≈ 0.8660
- For 180° arc: w1 = cos(90°) = 0 (degenerate — not allowed)

### 2.4 FromRevolutionOf() Creates Cylindrical Surfaces (surface.cpp:70)

```cpp
SSurface SSurface::FromRevolutionOf(SBezier *sb, Vector pt, Vector axis, 
                                     double thetas, double thetaf, ...) {
    SSurface ret = {};
    ret.degm = sb->deg;   // profile degree
    ret.degn = 2;          // ALWAYS degree-2 in revolution direction (for exact circle)
    
    double dtheta = fabs(WRAP_SYMMETRIC(thetaf - thetas, 2*PI));
    double w = cos(dtheta / 2);   // THE WEIGHT FORMULA
    
    for(i = 0; i <= ret.degm; i++) {
        Vector ps = rotate(sb->ctrl[i], thetas);  // start position
        Vector pf = rotate(sb->ctrl[i], thetaf);  // finish position
        Vector ct = tangent_intersection(ps, pf, axis);  // middle control point
        
        ret.ctrl[i][0] = ps;  ret.weight[i][0] = sb->weight[i];      // w1
        ret.ctrl[i][1] = ct;  ret.weight[i][1] = sb->weight[i] * w;  // w1*cos(θ/2)
        ret.ctrl[i][2] = pf;  ret.weight[i][2] = sb->weight[i];      // w1
    }
}
```

This creates a tensor-product surface: profile curve × circular arc in revolution direction.

### 2.5 Cylinder Transposition (shell.cpp:527)

After `FromRevolutionOf`, a cylindrical surface has `degm=1, degn=2`. But `IsCylinder()`
requires `degm=2, degn=1` (quadratic in m-direction, linear in n). So:

```cpp
// shell.cpp:527 — MakeFirstOrderRevolvedSurfaces
if(fabs(d0 - d1) < LENGTH_EPS) {
    // This is a cylinder; so transpose it so that we'll recognize it as extrusion
    SSurface sn = *srf;
    sn.degm = 2;
    sn.degn = 1;
    // Transpose: sn.ctrl[dn][dm] = srf->ctrl[1-dm][dn]
    // (note the 1-dm reversal to maintain normal direction)
    for(dm = 0; dm <= 1; dm++) {
        for(dn = 0; dn <= 2; dn++) {
            sn.ctrl[dn][dm] = srf->ctrl[1-dm][dn];
            sn.weight[dn][dm] = srf->weight[1-dm][dn];
        }
    }
    *srf = sn;
}
```

After transposition, `IsExtrusion()` works: same translation vector for all rows.

---

## 3. The Fillet Surface as a NURBS Cylinder

### 3.1 What a Fillet Between Two Flat Faces Actually Is

For the simplest case — a fillet at the edge between two **planar** faces:

**Geometry**: A cylindrical surface tangent to both planes.
- If the dihedral angle between the faces is φ, the fillet radius is r
- The fillet cylinder has axis direction = edge direction (along the edge)
- The arc cross-section spans the angle (π - φ), the supplement of the dihedral
- For a 90° corner: arc spans 90°, a quarter cylinder
- For a 135° corner: arc spans 45°
- For a 60° corner: arc spans 120°

**Control points of the fillet arc** (in the plane perpendicular to edge):
```
P0 = tangent point on face1  = foot point r away from edge on face1
P2 = tangent point on face2  = foot point r away from edge on face2
P1 = intersection of normal lines from P0 and P2 = the "corner"
w1 = cos((π-φ)/2) = cos(π/2 - φ/2) = sin(φ/2)
```

For a 90° dihedral (typical manufacturing corner):
- Arc sweep = 90°
- P0 = edge_start + r * n1  (n1 = face1 inward normal in cross-section)
- P2 = edge_start + r * n2  (n2 = face2 inward normal in cross-section)
- P1 = edge_point (the original sharp corner)
- w1 = cos(45°) = √2/2

### 3.2 Constructing the Fillet Arc for SolveSpace

From AGENT.md Known Learnings:
```
Fillet arc control points: P0=face1, P1=corner, P2=face2; weight[1]=cos(dihedral/2)
```

More precisely:
- Let `e` = edge start point
- Let `n1` = inward unit vector on face1 (perpendicular to edge, pointing into face1 surface)
- Let `n2` = inward unit vector on face2 (perpendicular to edge, pointing into face2 surface)
- Let φ = dihedral angle between the faces (the interior angle of the solid)
- Let r = fillet radius

Then:
```
P0 = e + r * n1          (tangent point on face 1)
P2 = e + r * n2          (tangent point on face 2)
P1 = e                   (corner = intersection of face tangent lines from P0, P2)
w1 = cos(π - φ)/2) = sin(φ/2)
```

Wait — more carefully: dtheta = the arc span angle = π - φ.
So w1 = cos(dtheta/2) = cos((π-φ)/2).

For φ = 90°: dtheta = 90°, w1 = cos(45°) = √2/2 ✓
For φ = 120°: dtheta = 60°, w1 = cos(30°) = √3/2 ≈ 0.866
For φ = 60°: dtheta = 120°, w1 = cos(60°) = 0.5
For φ near 180° (nearly flat): dtheta → 0°, w1 → 1 (nearly a straight segment)

**Constraint**: dtheta < 180° (i.e., φ > 0°). For dtheta ≥ 180°, need to split into 2 arcs. But dtheta = 180° means φ = 0° (zero-thickness solid), so practically never applies.

### 3.3 Fillet Surface = Extrusion of the Arc Along the Edge

The full fillet surface is the fillet arc **extruded along the edge direction**:

```cpp
// Pseudo-code for creating a fillet surface
SBezier arc;
arc.deg = 2;
arc.ctrl[0] = P0;          // tangent point on face1
arc.ctrl[1] = P1;          // corner point
arc.ctrl[2] = P2;          // tangent point on face2
arc.weight[0] = 1.0;
arc.weight[1] = cos(dtheta / 2.0);   // exact rational weight
arc.weight[2] = 1.0;

// Extrude arc from edge start to edge end
Vector t0 = {0,0,0};       // no offset at start
Vector t1 = edge_end - edge_start;  // edge direction vector
SSurface fillet = SSurface::FromExtrusionOf(&arc, t0, t1);
// Result: degm=2, degn=1 — exactly the canonical cylinder form
```

The `FromExtrusionOf()` function (surface.cpp:10) handles this directly:
```cpp
SSurface SSurface::FromExtrusionOf(SBezier *sb, Vector t0, Vector t1) {
    SSurface ret = {};
    ret.degm = sb->deg;   // = 2 for circular arc
    ret.degn = 1;         // extrusion direction
    for(i = 0; i <= ret.degm; i++) {
        ret.ctrl[i][0] = sb->ctrl[i].Plus(t0);  // start row
        ret.weight[i][0] = sb->weight[i];
        ret.ctrl[i][1] = sb->ctrl[i].Plus(t1);  // end row
        ret.weight[i][1] = sb->weight[i];
    }
    return ret;
}
```

Result: **exactly** the canonical cylinder form that `IsCylinder()` recognizes!

---

## 4. B-rep Integration: Trim Curves and Watertightness

### 4.1 How the Fillet Cylinder Fits into SShell

A fillet surface in a SShell requires:

1. **The cylinder SSurface** (degm=2, degn=1):
   - ctrl[0][*] = P0 (on face1) extruded along edge
   - ctrl[1][*] = P1 (corner) extruded along edge  
   - ctrl[2][*] = P2 (on face2) extruded along edge
   - weight[0][*]=1, weight[1][*]=w1, weight[2][*]=1

2. **Two longitudinal SCurves** (along the fillet-face interfaces):
   - SCurve A: separates fillet surface from face1
     - Is a line segment (the trim line at tangent point on face1)
     - `surfA` = fillet SSurface handle
     - `surfB` = face1 SSurface handle
   - SCurve B: separates fillet surface from face2
     - `surfA` = fillet SSurface handle
     - `surfB` = face2 SSurface handle

3. **Two "cap" SCurves** (at the ends of the edge):
   - End cap at edge_start and edge_end
   - These separate the fillet from adjacent cap faces (or the solid boundary)

4. **Updated trim polygons** on face1 and face2:
   - Old trim curves on the edge between face1 and face2 are **removed**
   - New trim curves (SCurve A, SCurve B) are added to the trim lists of face1 and face2

### 4.2 The Watertightness Invariant

From AGENT.md Known Learnings:
> "Watertightness invariant: every SCurve MUST be referenced by exactly 2 surfaces"

For the fillet:
- SCurve A: referenced by fillet surface AND face1  ✓
- SCurve B: referenced by fillet surface AND face2  ✓
- Old edge SCurve (between face1 and face2): REMOVED from both face1 and face2 trim lists ✓
- End cap SCurves at edge endpoints: referenced by fillet surface AND adjacent cap faces ✓

If any SCurve is not referenced by exactly 2 surfaces, `AssemblePolygon()` fails, creating
invisible holes in the mesh (silent failure in SolveSpace).

### 4.3 Trim Curve Types for Fillet

On the fillet cylinder surface itself, the trim polygon is a rectangle in (u,v) space:
- v=0 edge: edge_start boundary (cap)
- v=1 edge: edge_end boundary (cap) 
- u=0 edge: tangent line on face1 side → SCurve A
- u=1 edge: tangent line on face2 side → SCurve B

The parametric form is particularly clean because the fillet cylinder has natural alignment
with the feature geometry.

---

## 5. STEP Export of Cylindrical Fillet Surfaces

### 5.1 Current SolveSpace STEP Export

From `src/exportstep.cpp:493-520`:
```cpp
fprintf(f, "BOUNDED_SURFACE()\n");
fprintf(f, "B_SPLINE_SURFACE(%d,%d,(...)", ss->degm, ss->degn);
fprintf(f, "B_SPLINE_SURFACE_WITH_KNOTS((%d,%d),(%d,%d),",
    ss->degm+1, ss->degm+1, ss->degn+1, ss->degn+1);
fprintf(f, "(0.000,1.000),(0.000,1.000),.UNSPECIFIED.)\n");
fprintf(f, "RATIONAL_B_SPLINE_SURFACE((...weights...))");
```

SolveSpace exports ALL surfaces as generic `RATIONAL_B_SPLINE_SURFACE` in STEP format. This means:
- Planar faces: exported as degree-1 B-spline surfaces
- Cylindrical faces: exported as degree-2 B-spline surfaces (with rational weights)
- No special `CYLINDRICAL_SURFACE` entity used

The recipient CAD system will recognize the rational B-spline surface as a cylinder
automatically (through their own `IsCylinder`-equivalent detection).

**For fillet surfaces**: The existing STEP export code handles them with ZERO modifications.
A fillet cylinder (degm=2, degn=1) exports as `RATIONAL_B_SPLINE_SURFACE` with the correct
rational weights. Import into another CAD tool will reconstruct the exact geometry.

### 5.2 STEP Validation

The key for valid STEP export:
1. Rational weights must be correct (especially w1 = cos(dtheta/2) for fillet arcs)
2. Trim curves must match surface boundaries exactly
3. The STEP `ADVANCED_FACE` references both the surface and the trim loop

---

## 6. Triangulation of Cylindrical Fillet Surfaces

### 6.1 Path in TriangulateInto (surface.cpp:414)

```cpp
void SSurface::TriangulateInto(SShell *shell, SMesh *sm) {
    // ...
    if(degm == 1 && degn == 1) {
        poly.UvTriangulateInto(sm, this);    // Chamfer path: ear-clip, flat quads
    } else {
        poly.UvGridTriangulateInto(sm, this); // Fillet path: grid, curved surface
    }
```

A fillet surface (degm=2, degn=1) takes the `UvGridTriangulateInto` path.

### 6.2 Grid Triangulation Quality

- Adaptive grid: more segments where curvature is higher
- `MakeTriangulationGridInto()` computes segment counts based on `ChordTolMm()`
- Per-vertex normals (`NormalAt(u,v)`) ensure smooth G1 shading
- Minimum 4 segments along the circular direction for the fillet arc
- Zero extra code needed: fillet surfaces are triangulated automatically

---

## 7. Exact Cylinder Weight Formula — Summary Table

| Arc Angle (θ) | Dihedral (φ) | Weight w1 = cos(θ/2) | Use Case |
|---|---|---|---|
| 30° | 150° | cos(15°) ≈ 0.9659 | Slight fillet on obtuse edge |
| 45° | 135° | cos(22.5°) ≈ 0.9239 | 135° edge fillet |
| 60° | 120° | cos(30°) ≈ 0.8660 | 120° edge fillet |
| 90° | 90° | cos(45°) = √2/2 ≈ 0.7071 | **Right-angle fillet (most common)** |
| 120° | 60° | cos(60°) = 0.5 | Sharp 60° edge fillet |
| 135° | 45° | cos(67.5°) ≈ 0.3827 | Very sharp edge fillet |
| 179° | 1° | cos(89.5°) ≈ 0.0087 | Near-degenerate (concave) |

**Key insight**: SolveSpace's `SBezier::IsCircle()` already validates this formula by checking
`weight[1] == cos(dtheta/2)` with `LENGTH_EPS` tolerance. New fillet surfaces created with
the exact formula will pass this check.

---

## 8. Fillet vs. Canal Surfaces (General Case)

For the MVP (two flat faces), the fillet is a cylinder (constant radius).

For **general curved faces**, the fillet would be a canal surface:
- Defined by a spine curve (center of rolling ball) and a radius function
- Cannot be represented exactly as a single NURBS patch in general
- The spine curve (B-rep intersection of offset surfaces) is itself complex
- SolveSpace's `degm/degn` max of 3 could theoretically handle degree-3 approximations
- **For MVP: skip general case, implement flat-face cylinder only**

This is consistent with phkahler's advice (from AGENT.md): "limit myself to flat surfaces
only just to get the basics down."

---

## 9. SolveSpace-Specific Implementation Notes

### 9.1 Creating the Arc SBezier for Fillet

```cpp
SBezier MakeFilletArc(Vector P0, Vector cornerPt, Vector P2, double dihedralAngle) {
    SBezier arc = {};
    arc.deg = 2;
    arc.ctrl[0] = P0;
    arc.ctrl[1] = cornerPt;
    arc.ctrl[2] = P2;
    arc.weight[0] = 1.0;
    arc.weight[1] = cos((M_PI - dihedralAngle) / 2.0);  // = sin(dihedralAngle/2)
    arc.weight[2] = 1.0;
    return arc;
}
```

### 9.2 Creating the Fillet SSurface

```cpp
SSurface MakeFilletSurface(SBezier& arc, Vector edgeStart, Vector edgeEnd) {
    Vector t0 = Vector::From(0,0,0);
    Vector t1 = edgeEnd.Minus(edgeStart);
    SSurface fillet = SSurface::FromExtrusionOf(&arc, t0, t1);
    // Translate to edge position
    fillet = SSurface::FromTransformationOf(&fillet, edgeStart, Quaternion::IDENTITY, 1.0, false);
    return fillet;
}
```

Actually, `FromExtrusionOf` adds t0 and t1 to each control point, so we'd pass
`edgeStart` as the offset:

```cpp
SSurface MakeFilletSurface(SBezier& localArc, Vector edgeStart, Vector edgeEnd) {
    Vector t0 = edgeStart;
    Vector t1 = edgeEnd;   // absolute positions (localArc already at origin)
    return SSurface::FromExtrusionOf(&localArc, t0, t1);
}
```

### 9.3 IsCylinder Verification After Creation

The created fillet surface will pass `IsCylinder()` because:
- `IsExtrusion()` checks: same translation vector (edgeEnd-edgeStart) for all ctrl rows ✓
- `IsCircle()` checks: deg==2, weight[1]==cos(dtheta/2) ✓

This means `raycast.cpp` will correctly compute intersections with the fillet surface using
the closed-form cylinder formula (fast, numerically stable).

### 9.4 Face ID Assignment

From AGENT.md Known Learnings:
> "REMAP_CHAMFER_FACE=1011, REMAP_FILLET_FACE=1012"

The fillet surface's `face` field must be set to a Remap'd value for selection to work:
```cpp
fillet.face = group->Remap(someSCurveHandle, REMAP_FILLET_FACE).v;
```

---

## 10. Key Findings Summary

1. **Cylindrical NURBS is degree-2 rational**: The exact formula is `weight[1] = cos(dtheta/2)`
   where `dtheta` is the arc sweep angle. This is hardcoded into `SBezier::IsCircle()`.

2. **Fillet surface = arc extrusion**: `SSurface::FromExtrusionOf(circularArc, edgeStart, edgeEnd)`
   creates the fillet cylinder directly. No new code needed for geometry creation.

3. **SolveSpace already handles cylinders**: `IsCylinder()`, `IsCircle()`, `raycast.cpp`, and
   `TriangulateInto()` all handle degree-2 cylindrical surfaces. A new fillet surface is
   automatically triangulated, ray-tested, and STEP-exported correctly.

4. **Watertightness is the hard part**: The surrounding faces (face1, face2) must have their
   trim polygons updated to remove the old shared SCurve and add the two new fillet boundary
   SCurves. This "direct topology injection" approach (vs. full boolean) is the primary
   implementation challenge.

5. **For MVP (flat faces only)**: The fillet arc lies entirely in a plane (edge cross-section
   plane). Computing P0, P1, P2, and the dihedral angle is straightforward linear algebra
   on two planar faces.

6. **Vertex caps at edge endpoints**: Where the fillet cylinder starts and ends, small spherical
   patches would be needed for the general case. For MVP, the fillet extends to the face
   boundaries (no vertex blend). This is a known limitation.

7. **STEP export**: No changes needed. `RATIONAL_B_SPLINE_SURFACE` handles all cases.

---

## Sources

- SolveSpace source: `src/srf/surface.cpp` (FromExtrusionOf, IsCylinder, FromRevolutionOf)
- SolveSpace source: `src/srf/curve.cpp:127` (SBezier::IsCircle, weight formula)  
- SolveSpace source: `src/srf/shell.cpp:527` (MakeFirstOrderRevolvedSurfaces, cylinder transpose)
- SolveSpace source: `src/srf/raycast.cpp:278` (IsCylinder closed-form ray test)
- SolveSpace source: `src/exportstep.cpp:493` (RATIONAL_B_SPLINE_SURFACE export)
- Wikipedia: Non-uniform rational B-spline (circle example, w1=√2/2 for quarter circle)
- AGENT.md Known Learnings (fillet arc formula confirmed: weight[1]=cos(dihedral/2))
