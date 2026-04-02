# Task 4: SolveSpace NURBS Surface System Deep Dive

## Sources Examined
- `src/srf/surface.h` — complete header with all class definitions
- `src/srf/surface.cpp` — surface creation, trimming, triangulation
- `src/srf/boolean.cpp` — boolean operations (MakeFromBoolean, MakeCopyTrimAgainst, etc.)
- `src/srf/shell.cpp` — shell creation (extrusion, revolution, helix, copy, transform)
- `src/srf/ratpoly.cpp` — rational polynomial math (Bezier eval, Newton iteration, etc.)

---

## 1. Core Data Structures

### `SSurface` (surface.h)
The fundamental surface type. A **rational polynomial (NURBS/Bezier) surface patch** in Bezier form:

```cpp
class SSurface {
    int     tag;
    hSSurface h;        // handle (unique ID)
    hSSurface newH;     // used during boolean: maps old handle to new one
    RgbaColor color;
    uint32_t face;      // face ID for selection (links to Entity handle)
    
    int     degm, degn;               // degree in u and v (0-3 each)
    Vector  ctrl[4][4];               // control points (up to 4x4)
    double  weight[4][4];             // weights for NURBS (=1.0 for non-rational)
    
    List<STrimBy> trim;               // trim curves bounding this patch
    SBspUv *bsp;                      // 2D BSP for fast inside/outside tests
    SEdgeList edges;                  // cached XYZ edges for boolean ops
    Point2d cached;                   // last (u,v) guess for Newton iteration
};
```

**Key insight**: Maximum degree is 3 in each direction (4x4 control points). The `ctrl[4][4]` array is statically allocated — no dynamic sizing.

**Degrees used in practice:**
- `(1,1)` — planar surface (e.g., top/bottom of extrusion)
- `(degm, 1)` — extrusion surface (profile curve extruded linearly)  
- `(degm, 2)` — revolution surface (rational degree-2 in angular direction)
- A **chamfer surface** (planar) would be `(1,1)`
- A **fillet surface** (cylindrical) would be `(1,2)` or `(2,2)` (after transposition for cylinder detection)

### `SCurve` (surface.h)
Trim curves that bound surface patches:

```cpp
class SCurve {
    hSCurve h, newH;
    enum class Source { A, B, INTERSECTION };
    Source source;       // which operand this curve came from in a boolean
    bool isExact;        // true if exact Bezier, false if piecewise-linear approx
    SBezier exact;       // exact rational polynomial form (if isExact)
    List<SCurvePt> pts;  // piecewise linear approximation (always populated)
    hSSurface surfA, surfB;  // the two surfaces this curve separates
};
```

**Key insight**: Every `SCurve` separates exactly two surfaces (`surfA` and `surfB`). This is the B-rep "half-edge" concept — each edge of a surface patch is a curve shared between two faces.

### `STrimBy` (surface.h)
Links a surface to one of its trimming curves:

```cpp
class STrimBy {
    hSCurve curve;    // which curve
    bool backwards;   // traverse direction
    Vector start;     // XYZ start of this trim segment
    Vector finish;    // XYZ finish of this trim segment
};
```

### `SShell` (surface.h)
A watertight B-rep solid — collection of surfaces + curves:

```cpp
class SShell {
    IdList<SCurve,hSCurve>     curve;    // all trim curves
    IdList<SSurface,hSSurface> surface;  // all surface patches
    bool booleanFailed;                  // error flag
    
    // Factory methods:
    void MakeFromExtrusionOf(SBezierLoopSet*, Vector t0, Vector t1, RgbaColor);
    void MakeFromRevolutionOf(SBezierLoopSet*, Vector pt, Vector axis, ...);
    void MakeFromHelicalRevolutionOf(...);
    
    // Boolean operations:
    void MakeFromUnionOf(SShell *a, SShell *b);
    void MakeFromDifferenceOf(SShell *a, SShell *b);
    void MakeFromIntersectionOf(SShell *a, SShell *b);
    void MakeFromBoolean(SShell *a, SShell *b, SSurface::CombineAs type);
    
    // Utilities:
    void MakeFromCopyOf(SShell *a);
    void MakeFromTransformationOf(SShell *a, Vector t, Quaternion q, double scale);
    void MakeFromAssemblyOf(SShell *a, SShell *b);   // no boolean, just merge
    
    void RemapFaces(Group *g, int remap);  // assigns face IDs for selection
    void TriangulateInto(SMesh *sm);       // convert to mesh for rendering
    void MakeEdgesInto(SEdgeList *sel);    // extract boundary edges
};
```

### `SBezier` (surface.h / ratpoly.cpp)
Rational Bezier curve, degree 1-3:

```cpp
class SBezier {
    int deg;            // 1=line, 2=conic/arc, 3=cubic
    Vector ctrl[4];     // control points
    double weight[4];   // weights (1.0 for non-rational)
    uint32_t entity;    // linked entity handle
    
    // Key methods:
    static SBezier From(Vector p0, Vector p1);           // line
    static SBezier From(Vector p0, Vector p1, Vector p2); // conic
    static SBezier From(Vector4 p0, Vector4 p1, Vector4 p2); // rational conic
    
    Vector PointAt(double t);
    Vector TangentAt(double t);
    bool IsCircle(Vector axis, Vector *center, double *r);
    SBezier TransformedBy(Vector t, Quaternion q, double scale);
    void MakePwlInto(SEdgeList *sel, ...);  // piecewise-linearize
};
```

---

## 2. Surface Construction Methods

### `SSurface::FromExtrusionOf()` (surface.cpp:8)
Creates a bilinear/bicubic surface patch from a Bezier curve + translation vector:

```cpp
SSurface SSurface::FromExtrusionOf(SBezier *sb, Vector t0, Vector t1) {
    ret.degm = sb->deg;   // degree of profile curve
    ret.degn = 1;         // degree 1 in extrusion direction (linear)
    for(i = 0; i <= ret.degm; i++) {
        ret.ctrl[i][0] = sb->ctrl[i].Plus(t0);   // bottom row
        ret.ctrl[i][1] = sb->ctrl[i].Plus(t1);   // top row
        ret.weight[i][0] = ret.weight[i][1] = sb->weight[i];
    }
}
```

**For chamfer implementation**: A chamfer surface between two planar faces is also a linear extrusion — but it's swept along an edge direction rather than a straight translation. Alternatively, it can be created as `SSurface::FromPlane()` if the chamfer is flat.

### `SSurface::FromRevolutionOf()` (surface.cpp:56)
Creates a rational degree-2 surface from revolving a Bezier curve:

```cpp
SSurface SSurface::FromRevolutionOf(SBezier *sb, Vector pt, Vector axis, 
                                     double thetas, double thetaf, ...) {
    ret.degm = sb->deg;   // degree of profile
    ret.degn = 2;         // degree 2 in angular direction (exact circle)
    double w = cos(dtheta / 2);  // rational weight for exact conic
    for(i = 0; i <= ret.degm; i++) {
        // Three control points per row: start, middle (tangent intersection), end
        ret.ctrl[i][0] = ps;   // rotated start
        ret.ctrl[i][1] = ct;   // tangent intersection (middle weight = w)
        ret.ctrl[i][2] = pf;   // rotated finish
        ret.weight[i][0] = ret.weight[i][2] = sb->weight[i];
        ret.weight[i][1] = sb->weight[i] * w;  // rational weight
    }
}
```

**For fillet implementation**: A fillet between two planar faces is a partial cylinder. The fillet surface IS a `FromRevolutionOf` result where:
- Profile is a point (or the edge length becomes the profile)
- Actually: profile is a line segment perpendicular to the edge direction
- Revolved 90° (for 90° dihedral angle) around the fillet axis
- This yields a quarter-cylinder with degree `(1,2)` = exact in rational form

**For a fillet between two planar faces at 90°:**
```
fillet_surface = SSurface::FromRevolutionOf(
    &edge_line,     // the edge profile (1D line = profile degree 1)
    center_point,   // point on fillet axis
    edge_direction, // axis = direction of the shared edge
    0,              // start angle = 0
    PI/2,           // end angle = 90°
    0, 0            // no axial translation
);
// Result: degm=1, degn=2 — a quarter-cylinder surface
```

### `SSurface::FromPlane()` (surface.cpp:102)
Creates a degree (1,1) planar surface from an origin point + two direction vectors:

```cpp
SSurface SSurface::FromPlane(Vector pt, Vector u, Vector v) {
    ret.degm = 1; ret.degn = 1;
    ret.ctrl[0][0] = pt;
    ret.ctrl[0][1] = pt.Plus(u);
    ret.ctrl[1][0] = pt.Plus(v);
    ret.ctrl[1][1] = pt.Plus(v).Plus(u);
    // all weights = 1.0
}
```

**For chamfer implementation**: A chamfer surface IS a `FromPlane()` result. Given the shared edge and the offset distances d1, d2 on each face:
```
chamfer_surface = SSurface::FromPlane(
    corner_point,         // one endpoint of chamfer edge
    face1_offset_vector,  // vector from edge along face1 by distance d1
    edge_vector           // direction along the edge (length = edge length)
);
// Result: (1,1) planar surface = exact chamfer
```

---

## 3. Shell Assembly (shell.cpp)

### `SShell::MakeFromExtrusionOf()` (shell.cpp:16)
The canonical example of how to build a complete B-rep shell.

**Algorithm:**
1. Create top and bottom planar surfaces with `SSurface::FromPlane()`
2. For each edge curve in the profile:
   - Call `SSurface::FromExtrusionOf()` to get the side wall surface
   - Create trim curves (translated copies of the profile curve) for top/bottom
   - Create a vertical trim line at each endpoint
3. Assign `surfA`/`surfB` to each curve (which two surfaces does it separate?)
4. Add `STrimBy` references to each surface

**Key observation for chamfer/fillet**: The shell construction is explicit — you manually create each surface, each trim curve, and each `STrimBy`. There's no automatic topology generation. A chamfer/fillet shell must manually build all this topology.

### `SShell::MakeFirstOrderRevolvedSurfaces()` (shell.cpp:338)
Post-processes revolved surfaces that are actually planar (cone with apex) or cylindrical. After `FromRevolutionOf`, it recognizes:
- **Plane**: rewrites `(1,2)` surface to `(1,1)` for efficiency
- **Cylinder**: transposes u/v so `IsCylinder()` detects it as extrusion

This is important: **SolveSpace canonicalizes cylinders as extrusions** (`degm=2, degn=1`) for detection purposes. A fillet surface should also be built as an extrusion of an arc, not a revolution of a line.

### `SSurface::IsExtrusion()` (surface.cpp:20)
Detects if a surface is a linear extrusion:
```cpp
bool SSurface::IsExtrusion(SBezier *of, Vector *alongp) {
    if(degn != 1) return false;  // must be degree 1 in n direction
    // checks all column pairs are equal translation
}
```

### `SSurface::IsCylinder()` (surface.cpp:44)
Detects cylindrical surfaces:
```cpp
bool SSurface::IsCylinder(...) {
    SBezier sb;
    if(!IsExtrusion(&sb, axis)) return false;
    if(!sb.IsCircle(*axis, center, r)) return false;
    // returns axis, center, radius, start, finish
}
```

**Key insight**: A fillet surface is a **cylinder**, detected by `IsCylinder()`. This is used in STEP export (to write it as a `CYLINDRICAL_SURFACE`) and may be used for other optimizations.

---

## 4. Boolean Operations (boolean.cpp)

### `SShell::MakeFromBoolean()` — The Core Algorithm
```
1. Build classifying BSPs for both shells
2. Copy all curves from A and B into result, splitting at intersections
3. Generate intersection curves (A surfaces vs B surfaces)
4. Remove short degenerate segments
5. Rebuild BSPs with split curves
6. Copy surfaces from A (trimmed against B) into result
7. Copy surfaces from B (trimmed against A) into result
8. Rewrite surface handles in all curves
```

### `SSurface::MakeCopyTrimAgainst()` — Per-Surface Boolean Trimming
This is where each surface patch gets its new trimming:
1. Take existing trim curves, update to split versions
2. For DIFFERENCE, reverse normal of B-operand surfaces
3. Build original trim polygon (in UV space)
4. Find intersection curves projected into this surface's UV space
5. For each edge chain: classify as inside/outside the other shell
6. Keep/discard edges based on the boolean operation type
7. Reconstruct the trim polygon from kept edges

### `KeepRegion()` / `KeepEdge()` — The Decision Logic
```cpp
// For DIFFERENCE (A - B):
if(opA) return outSide || coincOpp;  // A's surfaces: keep if outside B
else    return inShell;               // B's surfaces: keep if inside A
```

**For chamfer/fillet**: The "chamfer tool body" approach would:
- Build a chamfer/fillet shell `C` (the new surfaces)
- Use `MakeFromDifferenceOf(original, cutter)` to remove material
- Use `MakeFromUnionOf(result, chamfer_shell)` to add chamfer faces

But this requires building `cutter` as a complete closed solid and `chamfer_shell` as a closed solid. Alternative: **directly inject** new surfaces into the SShell without boolean, which is simpler for MVP.

---

## 5. Surface Math (ratpoly.cpp)

### Bernstein Polynomials
SolveSpace uses precomputed Bernstein coefficient tables for efficiency:
```cpp
static inline double Bernstein(int k, int deg, double t);
static inline double BernsteinDerivative(int k, int deg, double t);
```
These are evaluated up to degree 3 only (the `[4][4][4]` table).

### Newton Iteration for Projection
`SSurface::ClosestPointNewton()` projects a 3D point onto a surface by Newton iteration. This is used extensively in boolean operations and trimming.

### Surface Evaluation
```cpp
Vector SSurface::PointAt(double u, double v)  // homogeneous evaluation
void   SSurface::TangentsAt(double u, double v, Vector *tu, Vector *tv)  // partial derivatives
Vector SSurface::NormalAt(double u, double v)  // = tu × tv
```

---

## 6. Trimming System

### How Trimming Works
Each `SSurface` has a `List<STrimBy> trim` — an ordered list of trim curves that form a closed polygon in UV space. The trim polygon must be a closed loop; if not, `AssemblePolygon()` will fail.

The `SBspUv` (2D BSP in UV space) accelerates point-in-polygon tests. Each trim curve is a `SCurve` in the parent `SShell::curve` list.

### For Chamfer/Fillet: What Trimming Is Needed
Each new surface needs:
1. **Trim curves along the original edges** (where chamfer/fillet meets the original faces)
2. **Trim curves at the ends** (where chamfer/fillet terminates at vertices)
3. Existing faces need their trim curves **modified** to cut off the corner

**This is the hardest part of implementing chamfer/fillet** — not the geometry of the new surface, but correctly updating the trim curves of the adjacent faces.

---

## 7. Key APIs for Chamfer/Fillet Implementation

### New Surface Creation
```cpp
// For chamfer (planar surface):
SSurface chamfer = SSurface::FromPlane(edge_start, offset_vec_face1, edge_direction);

// For fillet (cylindrical surface, built as arc extrusion):
SBezier arc = SBezier::From(p_start, p_mid_w, p_end);  // rational degree-2 arc
arc.weight[1] = cos(PI/4);  // w = cos(45°) for quarter-circle
SSurface fillet = SSurface::FromExtrusionOf(&arc, edge_start_vec, edge_end_vec);
```

### Adding Surfaces to Shell
```cpp
hSSurface hs = shell.surface.AddAndAssignId(&new_surface);
```

### Adding Trim Curves
```cpp
SCurve sc;
sc.isExact = true;
sc.exact = SBezier::From(p0, p1);  // line curve
sc.exact.MakePwlInto(&sc.pts);
sc.surfA = hs_new_face;
sc.surfB = hs_original_face;
hSCurve hc = shell.curve.AddAndAssignId(&sc);

STrimBy stb;
stb = STrimBy::EntireCurve(&shell, hc, /*backwards=*/false);
shell.surface.FindById(hs_new_face)->trim.Add(&stb);
stb = STrimBy::EntireCurve(&shell, hc, /*backwards=*/true);
shell.surface.FindById(hs_original_face)->trim.Add(&stb);
```

### Face ID Assignment (for selection)
```cpp
hEntity hface = group->Remap(some_entity, Group::REMAP_CHAMFER_FACE);
new_surface.face = hface.v;
```

---

## 8. Directly Building a Chamfer Shell (No Boolean Required)

The most feasible approach for an MVP is to **not use the boolean pipeline**. Instead:

1. Take the source group's `SShell`
2. Find the selected edge (identified by its two adjacent `SSurface` handles)
3. For the chamfer:
   a. Create the new `SSurface` (planar patch)
   b. Create new `SCurve` boundary curves (4 lines: two along adjacent faces, two end caps)
   c. **Modify the existing surfaces' trim curves** to cut off the chamfered region
   d. Add new `STrimBy` references to all affected surfaces
4. Result: a modified `SShell` with the chamfer faces inserted

This requires implementing `SShell::MakeFromChamferOf(SShell *src, hSSurface sfA, hSSurface sfB, double dist)`.

The challenge: **modifying existing trim polygons** without breaking watertightness.

### Alternative: Boolean-Based Chamfer
Build a "cutter" solid (the triangle prism being removed), boolean-subtract it from the source, then union in the chamfer face. This reuses existing infrastructure but requires building a closed cutter solid.

---

## 9. Relevant SShell Methods for a New `MakeFromChamferOf()`

```cpp
// These exist and are useful:
SShell::MakeFromCopyOf(SShell *a)           // deep copy
SShell::MakeFromAssemblyOf(SShell *a, SShell *b)  // combine without boolean
SSurface::Reverse()                          // flip normal
SShell::RemapFaces(Group *g, int remap)      // assign face IDs for selection
SShell::MakeEdgesInto(SEdgeList *sel)        // extract boundary edges for display

// These need to be written for chamfer:
// SShell::FindEdge(hSSurface surfA, hSSurface surfB) -> SCurve*
// SShell::ModifyTrimForChamfer(SCurve *edge, double dist) -> (new trim curves)
// SShell::MakeFromChamferOf(SShell *src, SCurve *edge, double dist, double dist2=0)
```

---

## 10. Degree Requirements Summary

| Feature | Surface Type | `degm` | `degn` | Notes |
|---------|-------------|--------|--------|-------|
| Chamfer | Planar | 1 | 1 | `FromPlane()` — exact |
| Fillet (90°) | Quarter-cylinder | 2 | 1 | `FromExtrusionOf(arc)` — exact rational |
| Fillet (arbitrary angle) | Partial cylinder | 2 | 1 | `FromRevolutionOf(line)` + transposition |
| End cap (vertex) | Planar or cone | 1 | 1 | Triangular trim region |

All required surface types are within SolveSpace's `[4][4]` control point grid limit.

---

## 11. `SSurface::CombineAs` Enum

```cpp
enum class CombineAs : uint32_t {
    UNION       = 10,
    DIFFERENCE  = 11,
    INTERSECTION = 12
};
```

The `face` field on each `SSurface` (type `uint32_t`) maps to an `hEntity::v` value via `Group::Remap()`. When a face is rendered, the `face` value is used to look up the color and highlight state.

---

## 12. NURBS Representation of a Fillet Arc

For a **quarter-circle arc** (90° fillet), the exact rational Bezier form is:
```
P0 = (r, 0, 0)     w0 = 1
P1 = (r, r, 0)     w1 = cos(45°) = 1/√2 ≈ 0.7071
P2 = (0, r, 0)     w2 = 1
```

In SolveSpace's degree-2 `SBezier`:
```cpp
SBezier arc;
arc.deg = 2;
arc.ctrl[0] = face1_point;           // start on face 1
arc.ctrl[1] = corner_point;          // tangent intersection
arc.ctrl[2] = face2_point;           // end on face 2
arc.weight[0] = arc.weight[2] = 1.0;
arc.weight[1] = cos(dihedral_angle / 2);  // rational weight
```

Then the fillet surface is an extrusion of this arc along the edge direction:
```cpp
SSurface fillet = SSurface::FromExtrusionOf(&arc, t0, t1);
// degm=2, degn=1 — exact quarter-cylinder
```

`SBezier::IsCircle()` will return `true` for this arc, so `IsCylinder()` will detect the resulting surface as a cylinder — which is correct for STEP export as `CYLINDRICAL_SURFACE`.

---

## Summary of Key Findings

1. **SolveSpace uses explicit B-rep**: Each surface patch, trim curve, and topology connection must be manually constructed. There is no automatic topology inference.

2. **Surface degrees are bounded at 3**: All surfaces fit in `ctrl[4][4]`. Chamfer (planar, degree 1,1) and fillet (cylindrical, degree 2,1) both fit perfectly within this limit.

3. **Chamfer = FromPlane()**: A chamfer surface is a `(1,1)` planar patch created with `SSurface::FromPlane()`. This is algebraically exact.

4. **Fillet = FromExtrusionOf(arc)**: A fillet surface is a `(2,1)` cylindrical patch created by extruding a rational degree-2 arc along the edge direction. This is algebraically exact via rational NURBS.

5. **The hard part is trim curve modification**: The adjacent original faces need their trim polygons modified to "clip off" the filleted/chamfered region. This is the core algorithmic challenge.

6. **Boolean pipeline exists but is complex**: `MakeFromDifferenceOf()` works but requires building a complete closed cutter solid. Direct topology manipulation is simpler for an MVP.

7. **`SShell::RemapFaces()`** assigns face IDs for selection and rendering — chamfer/fillet faces need new `REMAP_CHAMFER_FACE` and `REMAP_FILLET_FACE` constants.

8. **`MakeFirstOrderRevolvedSurfaces()`** post-processes revolved surfaces to canonical form — a fillet built by revolution would be transposed to extrusion form by this routine.

9. **Thread safety**: `SShell::TriangulateInto()` and `SShell::MakeBoolean()` use OpenMP parallel loops. New methods must be thread-safe or exclude the new surfaces from parallel regions.

10. **Watertightness requirement**: `SCurve::surfA` and `surfB` must always be set correctly. Every curve must be referenced by exactly the two surfaces it separates. Breaking this causes boolean failures and rendering artifacts.
