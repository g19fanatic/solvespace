# Finding 17: SolveSpace NURBS Surface Triangulation — How It Works and Implications for Fillet Surfaces

## Source Files Analyzed
- `src/srf/triangulate.cpp` — Complete triangulation algorithms (ear-clipping, grid, bridging)
- `src/srf/surface.cpp` lines 414–455 — `SSurface::TriangulateInto` dispatcher
- `src/srf/shell.cpp` lines 590–600 — `SShell::TriangulateInto` OpenMP loop
- `src/solvespace.cpp` lines 496–507 — `ChordTolMm()`, `ExportChordTolMm()`, `GetMaxSegments()`
- `src/solvespace.h` lines 546–548 — declarations
- `src/groupmesh.cpp` lines 244, 425–483 — callers

---

## 1. Top-Level Triangulation Call Chain

### `SShell::TriangulateInto(SMesh *sm)` — `src/srf/shell.cpp:590`
```cpp
void SShell::TriangulateInto(SMesh *sm) {
#pragma omp parallel for
    for(int i=0; i<surface.n; i++) {
        SSurface *s = &surface[i];
        SMesh m;
        s->TriangulateInto(this, &m);
        #pragma omp critical
        sm->MakeFromCopyOf(&m);
        m.Clear();
    }
}
```
- **OpenMP parallelized** over surfaces. Each surface is triangulated independently.
- Results are merged under a critical section via `SMesh::MakeFromCopyOf`.
- **Implication**: Any new chamfer/fillet surfaces in the shell get triangulated automatically — no special-casing needed. Just add the surface to the SShell and it will be triangulated.

### `SSurface::TriangulateInto(SShell *shell, SMesh *sm)` — `src/srf/surface.cpp:414`
```cpp
void SSurface::TriangulateInto(SShell *shell, SMesh *sm) {
    SEdgeList el = {};
    MakeEdgesInto(shell, &el, MakeAs::UV);   // Collect all trim curves in UV space

    SPolygon poly = {};
    if(el.AssemblePolygon(&poly, NULL, /*keepDir=*/true)) {
        int i, start = sm->l.n;
        if(degm == 1 && degn == 1) {
            // Planar or ruled surface — ear-clip path
            poly.UvTriangulateInto(sm, this);
        } else {
            // Curved surface (degree > 1 in either direction) — grid path
            poly.UvGridTriangulateInto(sm, this);
        }
        
        STriMeta meta = { face, color };
        for(i = start; i < sm->l.n; i++) {
            STriangle *st = &(sm->l[i]);
            st->meta = meta;
            st->an = NormalAt(st->a.x, st->a.y);
            st->bn = NormalAt(st->b.x, st->b.y);
            st->cn = NormalAt(st->c.x, st->c.y);
            st->a = PointAt(st->a.x, st->a.y);
            st->b = PointAt(st->b.x, st->b.y);
            st->c = PointAt(st->c.x, st->c.y);
            st->FlipNormal();
        }
    }
    // ... (else: assemblePolygon failed = degenerate, skip)
    el.Clear();
    poly.Clear();
}
```

**KEY DECISION BRANCH**: `degm == 1 && degn == 1`

| Surface Type | degm | degn | Path Used |
|---|---|---|---|
| Plane (FromPlane) | 1 | 1 | `UvTriangulateInto` (ear-clip) |
| Chamfer surface | 1 | 1 | `UvTriangulateInto` (ear-clip) ← **EASY** |
| Extrusion of line | 1 | 1 | `UvTriangulateInto` (ear-clip) |
| Cylinder (fillet) | 2 | 1 | `UvGridTriangulateInto` (grid) ← **HANDLED** |
| Extrusion of arc | 2 | 1 | `UvGridTriangulateInto` (grid) |
| Revolution | 2 | 2 | `UvGridTriangulateInto` (grid) |

**Chamfer surfaces (degree 1,1) use the simple ear-clip path — no extra work needed.**
**Fillet surfaces (degree 2,1 cylinders) use the grid path — also already handled.**

---

## 2. Chord Tolerance System

### `SS.ChordTolMm()` — `src/solvespace.cpp:496–498`
```cpp
double SolveSpaceUI::ChordTolMm() {
    if(exportMode) return ExportChordTolMm();
    return chordTolCalculated;
}
double SolveSpaceUI::ExportChordTolMm() {
    return exportChordTol / exportScale;
}
int SolveSpaceUI::GetMaxSegments() {
    if(exportMode) return exportMaxSegments;
    return maxSegments;
}
```
- `chordTolCalculated` is updated before regeneration (based on bounding box of model)
- `exportChordTol` is the user-set export tolerance (from preferences)
- `maxSegments` is stored in preferences, used to limit recursion in `MakeTriangulationGridInto`
- Two distinct tolerances: **display** (adaptive, based on viewport) and **export** (fixed user setting)

---

## 3. Path A: Ear-Clipping Triangulation (`UvTriangulateInto`)

### Entry Point: `SPolygon::UvTriangulateInto(SMesh *m, SSurface *srf)` — `src/srf/triangulate.cpp`

**Algorithm Overview** (applied to UV polygon):
1. **Fix contour directions** — Outer contour CCW, holes CW
2. **Merge holes** using `BridgeToContour()` — finds bridge edges to merge holes into a single contour
3. **Call** `SContour::UvTriangulateInto(SMesh *m, SSurface *srf)` on merged contour

### `SContour::UvTriangulateInto` — the actual ear-clip:
1. **Scale epsilon** by surface tangent magnitude at (0.5, 0.5) for numerics
2. **Remove zero-length edges** from contour
3. **Triangle fan optimization** (optional): for degree (1,1) surfaces, detect runs of equal-length consecutive ear-ears and convert to fans for performance
4. **Main ear-clip loop** (while > 3 vertices):
   - For each candidate vertex, test if it's an "ear" (`IsEar()`)
   - For curved surfaces: score ears by `ChordToleranceForEdge(prev, next)` — prefer ears that don't deviate much from the surface
   - For planes: any ear is equally good, take the first
   - Stop early if `bestChordTol < 0.1 * SS.ChordTolMm()` (sufficient quality)
   - Clip the best ear (`ClipEarInto`)
5. **Final triangle**: clip last 3 vertices

### `SSurface::ChordToleranceForEdge(Vector a, Vector b)` — `src/srf/triangulate.cpp`
```cpp
double SSurface::ChordToleranceForEdge(Vector a, Vector b) const {
    Vector as = PointAt(a.x, a.y), bs = PointAt(b.x, b.y);
    double worst = VERY_NEGATIVE;
    for(int i = 1; i <= 3; i++) {
        Vector p  = a. Plus((b. Minus(a )).ScaledBy(i/4.0)),
               ps = as.Plus((bs.Minus(as)).ScaledBy(i/4.0));
        Vector pps = PointAt(p.x, p.y);
        worst = max(worst, (pps.Minus(ps)).MagSquared());
    }
    return sqrt(worst);
}
```
- Measures how far the straight chord between two UV points deviates from the actual surface
- For planes (degm=1, degn=1): deviation is exactly 0 — optimal ear selection doesn't need chord test
- **For chamfer surfaces (degree 1,1 bilinear planes): this returns 0 always** — triangulation is exact and trivially fast

---

## 4. Path B: Grid Triangulation (`UvGridTriangulateInto`)

### `SPolygon::UvGridTriangulateInto(SMesh *mesh, SSurface *srf)` — `src/srf/triangulate.cpp`

**Algorithm Overview**:
1. Build a **rectangular grid** in UV space with adaptive spacing
2. Grid spacing computed by `SSurface::MakeTriangulationGridInto()` — recursive subdivision
3. For each interior grid cell (quad):
   - Skip if edges cross any trim curve
   - Skip if center not inside polygon
   - If inside: emit two triangles, track boundary edges
4. Remaining boundary region triangulated by `UvTriangulateInto` (ear-clip fallback)

### `SSurface::MakeTriangulationGridInto()` — adaptive grid spacing
```cpp
void SSurface::MakeTriangulationGridInto(List<double> *l, double vs, double vf,
                                         bool swapped, int depth) const {
    // Measure chord deviation for 4 isoparametric curves
    // Also measure normal twist between two points
    double step = 1.0/SS.GetMaxSegments();
    if( ((vf - vs) < step || worst < SS.ChordTolMm())
        && ((worst_twist > 0.999) || (depth > 3)) ) {
        l->Add(&vf);  // this interval is fine enough
    } else {
        MakeTriangulationGridInto(l, vs, (vs+vf)/2, swapped, depth+1);
        MakeTriangulationGridInto(l, (vs+vf)/2, vf, swapped, depth+1);
    }
}
```
- Adaptive subdivision stops when worst chord deviation < `SS.ChordTolMm()`
- OR when interval < `1.0/GetMaxSegments()` (minimum segment size)
- For cylinders (degree 2,1): also checks twist of normals — if normals deviate by more than ~2.5°, subdivide
- **Minimum 4 grid divisions for degree-2 surfaces** (hardcoded in `UvGridTriangulateInto`)

### Grid → Triangle Mapping
```cpp
// For each interior quad:
srf->TangentsAt(us, vs, &tu, &tv);
if(tu.Dot(tv) < LENGTH_EPS) {
    // Orthogonal tangents: split a-b-c / a-c-d
    mesh->AddTriangle(a, b, c);
    mesh->AddTriangle(a, c, d);
} else {
    // Non-orthogonal: split a-b-d / b-c-d  
    mesh->AddTriangle(a, b, d);
    mesh->AddTriangle(b, c, d);
}
```
- Tangent dot product determines diagonal — avoids poorly-shaped triangles
- **For fillet surfaces (cylinders)**: tu is along the cylinder axis (perpendicular to arc), tv is along the arc — they are perpendicular, so uses the a-b-c / a-c-d split

---

## 5. Normal Computation for Triangles

After triangulation in UV, all triangles are transformed to XYZ:
```cpp
st->an = NormalAt(st->a.x, st->a.y);  // Normal at vertex a (UV)
st->bn = NormalAt(st->b.x, st->b.y);  // Normal at vertex b (UV)
st->cn = NormalAt(st->c.x, st->c.y);  // Normal at vertex c (UV)
st->a = PointAt(st->a.x, st->a.y);    // Transform UV → XYZ
st->b = PointAt(st->b.x, st->b.y);
st->c = PointAt(st->c.x, st->c.y);
st->FlipNormal();                       // Fix winding (UV poly directions vs surface normal)
```

**Implications for fillet rendering**:
- Fillet cylinders automatically get **per-vertex normals** — smooth shading
- The normals at the longitudinal vertices (along the arc) will be perpendicular to the cylinder axis and match the tangent point on the adjacent face — perfect G1 shading
- This means fillets will visually appear smoothly blended (no sharp edge between fillet and face) in the render

**Implications for chamfer rendering**:
- Chamfer planes get the same treatment, but since the chamfer surface and adjacent faces form sharp dihedral angles (G0 only), the normals at shared edges will be discontinuous — you see the sharp edge, which is correct behavior for a chamfer

---

## 6. The `face` Field and Selection Highlighting

```cpp
STriMeta meta = { face, color };
for(i = start; i < sm->l.n; i++) {
    STriangle *st = &(sm->l[i]);
    st->meta = meta;
    ...
}
```

- `face` is `SSurface::face` (uint32_t) — this is what the selection system uses to identify which surface was clicked
- For chamfer/fillet surfaces, `face` must be set to a valid `REMAP_CHAMFER_FACE` or `REMAP_FILLET_FACE` value from the Group's remap table
- This is how `Group::GetRedundantConstraints()` and face-selection in `graphicswin.cpp` work
- **IMPORTANT**: Setting `face = 0` means the surface is unselectable. Chamfer/fillet surfaces should be selectable for deletion.

---

## 7. Handling of Degenerate Cases

### `el.AssemblePolygon(&poly, NULL, /*keepDir=*/true)` may fail if:
- Trim curves don't form a closed loop (topology error in B-rep)
- Trim curve endpoints don't exactly match

If polygon assembly fails, the surface produces **no triangles** — it's silently skipped with `dbp("...failed...")` in debug builds. This is a silent failure mode that could produce visible holes in the mesh.

**Critical implication**: Chamfer/fillet B-rep construction MUST ensure watertight trim curves. Any gap in SCurve endpoint matching will cause a hole in the triangulation. This is one of the key risks in the implementation.

---

## 8. Specific Triangulation Behavior for Chamfer and Fillet Surfaces

### Chamfer Surface (degree 1,1 bilinear plane)
- **Algorithm**: `UvTriangulateInto` (ear-clip)
- **Grid**: Not used — trivial 2D polygon
- **Shape**: Typically a quadrilateral (4 trim curves = 4 edges). May be triangular at edge endpoints.
- **Triangulation**: A quad with 4 vertices → 2 triangles. For a quadrilateral with corners A,B,C,D in UV, the polygon assembly produces 4 vertices → 1 ear-clip iteration → 2 triangles.
- **Performance**: Fastest possible — 4 vertices, 2 triangles
- **Chord tolerance**: Not consulted (degm=degn=1 plane)
- **Result quality**: Exact (no approximation needed for flat surface)

### Fillet Surface (degree 2,1 cylindrical extrusion)
- **Algorithm**: `UvGridTriangulateInto` (grid)
- **Grid**: Minimum 4 divisions in the degree-2 (arc) direction, 1+ divisions in degree-1 (linear) direction
- **Shape**: `u` ∈ [0,1] along edge length, `v` ∈ [0,1] along arc angle
- **Adaptive refinement**: Grid is refined until chord deviation < `SS.ChordTolMm()`
- **Normal vectors**: Properly computed per-vertex — smooth shading across the fillet
- **Corner vertices**: Where the fillet arc meets the face at u=0 and u=1 (edge endpoints), the grid produces proper vertex normals
- **Result quality**: Bounded by `SS.ChordTolMm()` — looks smooth in viewport

### Vertex Cap Surfaces (at edge endpoints — 3 or more surfaces meeting)
- Small triangular planar patches (degree 1,1) or small spherical patches
- **MVP scope**: Skip vertex caps — just let adjacent surfaces slightly overlap, or use simpler planar caps
- Ear-clip algorithm handles triangular caps trivially (1 triangle)

---

## 9. Tolerance Values and Performance

From `src/solvespace.cpp`:
- `chordTolCalculated` is updated by `SolveSpaceUI::ReloadAllLinked()` or similar
- `chordTol` (stored as `ChordTolerancePct`) is a percentage of the model bounding box diagonal
- Typical display value: 0.1% of bounding box diagonal → very smooth curves
- Export value: user-set in export preferences (fixed mm value)

For a 1mm radius fillet on a 10mm part:
- Bounding box diagonal ≈ ~14mm
- ChordTolMm ≈ 0.1% × 14mm ≈ 0.014mm
- For r=1mm fillet: arc length ≈ π/2 ≈ 1.57mm for 90°
- `MakeTriangulationGridInto` with tol=0.014mm: subdivision until chord < 0.014mm
  - Max chord for single segment on r=1 circle: chord = 2r sin(θ/2) ≈ r·θ for small θ
  - Required: r(1 - cos(θ/2)) < 0.014 → θ < arccos(1 - 0.014) ≈ 9.6°
  - Total arc = 90° → 90/9.6 ≈ 10 segments minimum
  - Minimum 4 is already enforced → uses ~10 grid divisions for 1mm fillet at typical tolerance

---

## 10. UV Coordinate Conventions for New Surfaces

For `SSurface::FromPlane(pt, u, v)` (chamfer surface):
- UV space is simply [0,1]×[0,1]
- `PointAt(s,t) = pt + u*s + v*t`
- Trim polygon vertices are expressed in this normalized UV space

For `SSurface::FromExtrusionOf(sb, t0, t1)` (fillet cylindrical surface):
- `u` ∈ [0,1] maps to the profile curve parameter
- `v` ∈ [0,1] maps from t0 to t1 (along the extrusion axis = along the edge)
- For a fillet cylinder: `u` is the arc parameter (controls how much of the quarter-circle we've traversed)
- `PointAt(u,v) = sb.PointAt(u) + (t1-t0)*v`

When creating trim curves for a new surface, points must be expressed in that surface's UV space. This is why the `MakeEdgesInto(..., MakeAs::UV)` call in `TriangulateInto` projects all trim curve points into UV before passing to the polygon assembler.

---

## 11. Key Functions in src/srf/triangulate.cpp

| Function | Role |
|---|---|
| `SPolygon::UvTriangulateInto(SMesh*, SSurface*)` | Entry: merge holes, then ear-clip |
| `SContour::BridgeToContour(...)` | Merge a hole into outer contour via bridge edge |
| `SContour::UvTriangulateInto(SMesh*, SSurface*)` | Ear-clip algorithm on merged contour |
| `SContour::IsEar(int bp, double eps)` | Test if vertex bp is a valid ear |
| `SContour::IsEmptyTriangle(ap,bp,cp, eps)` | Test if triangle has no interior points |
| `SContour::ClipEarInto(SMesh*, int bp, double eps)` | Emit triangle for ear, remove vertex |
| `SPolygon::UvGridTriangulateInto(SMesh*, SSurface*)` | Grid + ear-clip for curved surfaces |
| `SSurface::MakeTriangulationGridInto(List<double>*, ...)` | Adaptive grid spacing computation |
| `SSurface::ChordToleranceForEdge(Vector a, Vector b)` | UV chord deviation from surface |
| `SPolygon::TriangulateInto(SMesh*)` | XYZ triangulation (not UV) — for 2D polygons |

---

## 12. Critical Implications for Chamfer/Fillet Implementation

### Good News
1. **No changes needed to triangulation code** — The existing system handles both degree (1,1) and degree (2,1) surfaces perfectly
2. **Chamfer** (degree 1,1) is the simplest possible case — ear-clip, 2 triangles per face
3. **Fillet** (degree 2,1) is handled by the grid path with adaptive refinement — already works for extrusion surfaces
4. **OpenMP parallel triangulation** already works — new surfaces are automatically included
5. **Per-vertex normals** are automatically computed correctly for both surface types
6. **Smooth shading for fillets** is automatic — no extra code needed

### Watch-Outs
1. **`face` field**: Must be set to a valid remap-derived value (not 0) for selectability
2. **`color` field**: Should be set to the group's color for consistent appearance
3. **Watertightness first**: If the B-rep is not watertight (SCurve mismatches), `AssemblePolygon` will fail silently — surfaces will be missing from the mesh
4. **UV trim polygon orientation**: Must be consistent — wrong orientation causes `FixContourDirections()` errors
5. **Grid minimum**: Degree-2 surfaces always get at least 4 grid segments (even if chord tol says 1 would suffice) — minor performance overhead, correct behavior

### No Action Items for Triangulation
The triangulation system is complete and correct for our needs. The only requirement is:
- Create `SSurface` objects with correct `degm`, `degn`, `ctrl[][]`, `weight[][]`
- Add proper `STrimBy` trim curves that form a closed polygon in UV space
- Set `face` and `color` fields appropriately
- Ensure SCurve `pts[]` are within `SS.ChordTolMm()` of the exact curve

---

## Summary

SolveSpace's triangulation system uses a **two-path architecture**:
- **Degree (1,1) surfaces** → ear-clipping (fast, exact for planes)
- **Degree ≥ 2 in any direction** → adaptive grid + ear-clip (curved surface tessellation)

**Chamfer surfaces** (degree 1,1 bilinear planes) are trivially handled by the ear-clip path — they are the simplest case. A chamfer face is just 4 UV corner points → 2 triangles.

**Fillet surfaces** (degree 2,1 cylindrical extrusions) are handled by the grid path — same as any other extruded arc surface (like a cylinder from a sketch). The adaptive grid produces smooth, chord-tolerance-bounded tessellation with correct per-vertex normals.

**No modifications to `src/srf/triangulate.cpp` or `src/srf/surface.cpp` are needed** for chamfer/fillet support. The triangulation system will work correctly with properly constructed SSurface objects added to the result SShell.
