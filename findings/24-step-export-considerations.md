# Finding 24: STEP Export Considerations for Chamfer/Fillet Geometry

## Overview

This document analyzes `src/exportstep.cpp` to understand how SolveSpace's STEP exporter
works, what NURBS surface types are currently exported, and what changes (if any) are
needed to support chamfer/fillet geometry in STEP output.

## How STEP Export Works in SolveSpace

### Entry Point: `ExportSurfacesTo()`

```cpp
void StepFileWriter::ExportSurfacesTo(const Platform::Path &filename) {
    Group *g = SK.GetGroup(SS.GW.activeGroup);
    SShell *shell = &(g->runningShell);
    ...
    for(SSurface &ss : shell->surface) {
        if(ss.trim.IsEmpty()) continue;
        SBezierList sbl = {};
        ss.MakeSectionEdgesInto(shell, NULL, &sbl);
        ss.ScaleSelfBy(1.0/SS.exportScale);
        sbl.ScaleSelfBy(1.0/SS.exportScale);
        ExportSurface(&ss, &sbl);
        sbl.Clear();
    }
```

**Key insight**: The exporter iterates over `shell->surface` — ALL surfaces in the
`runningShell` of the active group. This means:

1. **Any surface added to the SShell is automatically exported**, including chamfer and fillet surfaces
2. No special-casing is needed for different surface types
3. The exporter treats all surfaces uniformly as rational B-spline surfaces (NURBS)

### Surface Export: `ExportSurface()`

The function exports any `SSurface` as a `B_SPLINE_SURFACE` with these STEP entities:
- `BOUNDED_SURFACE`
- `B_SPLINE_SURFACE(degm, degn, control_points, ...)`
- `B_SPLINE_SURFACE_WITH_KNOTS`
- `GEOMETRIC_REPRESENTATION_ITEM`
- `RATIONAL_B_SPLINE_SURFACE(weights)`
- `ADVANCED_FACE` with trim loops

The control point grid uses `ss->ctrl[i][j]` and weights use `ss->weight[i][j]`.

### STEP NURBS Surface Format

```
#srfid=(
BOUNDED_SURFACE()
B_SPLINE_SURFACE(degm, degn, ((#cp1, #cp2,...), (...)),
    .UNSPECIFIED.,.F.,.F.,.F.)
B_SPLINE_SURFACE_WITH_KNOTS((degm+1,degm+1),(degn+1,degn+1),
    (0.000,1.000),(0.000,1.000),.UNSPECIFIED.)
GEOMETRIC_REPRESENTATION_ITEM()
RATIONAL_B_SPLINE_SURFACE(((w00,w01,...),(w10,w11,...)))
REPRESENTATION_ITEM('')
SURFACE()
);
```

Note: The knots are always `(0.0, 1.0)` — a single Bezier patch per surface.

### Trim Curve Export: `ExportCurveLoop()`

Each surface's trim curves (as `SBezierLoop`) are exported as:
- `CARTESIAN_POINT` for control points
- `B_SPLINE_CURVE` (rational) for each Bezier trim segment
- `VERTEX_POINT` for start/end
- `EDGE_CURVE` connecting vertex pairs
- `ORIENTED_EDGE` for directional traversal
- `EDGE_LOOP` containing all oriented edges
- `FACE_OUTER_BOUND` or `FACE_BOUND` for the loop

The duplicate detection (alias tracking via `pointAliases`, `curveAliases`, etc.) ensures
shared vertices and edges are not written twice.

## Implications for Chamfer/Fillet Surfaces

### Chamfer Surface (Degree 1,1 Bilinear Patch)

A chamfer surface created via `SSurface::FromPlane()` has:
- `degm = 1, degn = 1` → 4 control points
- All weights = 1.0 (non-rational, pure bilinear)
- Two trim curves: one per adjacent face boundary (line segments)

**STEP representation**:
```
#N=(
BOUNDED_SURFACE()
B_SPLINE_SURFACE(1, 1, ((#p0, #p1), (#p2, #p3)),
    .UNSPECIFIED.,.F.,.F.,.F.)
B_SPLINE_SURFACE_WITH_KNOTS((2,2),(2,2),
    (0.000,1.000),(0.000,1.000),.UNSPECIFIED.)
GEOMETRIC_REPRESENTATION_ITEM()
RATIONAL_B_SPLINE_SURFACE(((1.0,1.0),(1.0,1.0)))
REPRESENTATION_ITEM('')
SURFACE()
);
```

This is a **planar surface** in STEP — the most basic possible case.
STEP viewers/importers will recognize it as a simple flat quadrilateral.

✅ **No changes needed** to export a chamfer surface — it's handled automatically.

### Fillet Surface (Degree 2,1 Cylindrical Rational NURBS Patch)

A fillet surface created via `SSurface::FromExtrusionOf(arc)` has:
- `degm = 2, degn = 1` → 6 control points (3×2 grid)
- Weights: `weight[1][0] = weight[1][1] = cos(dihedral/2)` (rational for circular arc)
- All other weights = 1.0

**STEP representation**:
```
#N=(
BOUNDED_SURFACE()
B_SPLINE_SURFACE(2, 1, ((#p00, #p01), (#p10, #p11), (#p20, #p21)),
    .UNSPECIFIED.,.F.,.F.,.F.)
B_SPLINE_SURFACE_WITH_KNOTS((3,3),(2,2),
    (0.000,1.000),(0.000,1.000),.UNSPECIFIED.)
GEOMETRIC_REPRESENTATION_ITEM()
RATIONAL_B_SPLINE_SURFACE(((1.0,1.0),(w,w),(1.0,1.0)))
REPRESENTATION_ITEM('')
SURFACE()
);
```

Where `w = cos(dihedral_angle/2)`.

This is an **exact cylindrical surface** in STEP. Most CAD importers handle degree-2
rational B-spline surfaces (quadratic NURBS) correctly and will recognize the cylinder.

✅ **No changes needed** to export a fillet surface — it's also handled automatically.

## STEP Schema Validation

### Current STEP Schema Used

SolveSpace uses `CONFIG_CONTROL_DESIGN` schema (AP203), as seen in the header:
```cpp
"FILE_SCHEMA (('CONFIG_CONTROL_DESIGN'));\\n"
```

This schema supports:
- `ADVANCED_BREP_SHAPE_REPRESENTATION` → chamfer/fillet surfaces qualify
- `B_SPLINE_SURFACE_WITH_KNOTS` → supports degrees 1-3
- `RATIONAL_B_SPLINE_SURFACE` → supports non-unit weights for fillets

### Degree Compatibility

The exporter writes `B_SPLINE_SURFACE(degm, degn, ...)` using whatever `ss->degm` and
`ss->degn` are stored in the surface. For chamfer (1,1) and fillet (2,1), these are
valid B-spline degrees.

**Important**: The STEP spec allows degree 1 in B_SPLINE_SURFACE, so planar chamfer
surfaces export correctly. Some older STEP importers may not handle rational degree-1
surfaces (they might expect PLANE entity instead), but non-rational degree-1 (all weights=1)
is universally supported.

## Trim Curve Requirements

The `ExportSurface()` function calls `ss.MakeSectionEdgesInto(shell, NULL, &sbl)` to get
the trim curves as Bezier loops. This call uses the surface's `trim` polygon to build
the boundary curves.

For chamfer/fillet surfaces to export correctly, their `trim` polygons must be properly
populated. This is already a requirement for watertightness (SCurve entries), so if the
3D B-rep is correct, the STEP export will work.

### Edge Case: Shared Edges (Alias Detection)

The alias detection system (`HasCartesianPointAnAlias`, `HasEdgeCurveAnAlias`, etc.)
handles shared edges between surfaces. Chamfer surfaces share 2 edges with adjacent
original faces, so the alias system will:
1. Export the shared edge curve ONCE (first occurrence)
2. Reference it from both surfaces by alias

This is exactly how STEP B-rep should work — shared edges referenced by both adjacent faces.

**Potential issue**: The alias detection uses `PRECISION = 2*LENGTH_EPS` for point
matching. If chamfer/fillet control points are computed with floating-point error larger
than this tolerance, shared points might be exported as duplicates (causing STEP validity
issues). This is a general watertightness concern, not STEP-specific.

## Color Handling

Each `ADVANCED_FACE` gets color from `ss->color`:
```cpp
fprintf(f, "#%d=COLOUR_RGB('',%.2f,%.2f,%.2f);\n", ++id, 
        ss->color.redF(), ss->color.greenF(), ss->color.blueF());
```

Chamfer/fillet surfaces created in `MakeFromChamferOf()` should inherit color from the
source shell or use a default. The face field (`ss->face`) is used for selection, not color.
Setting `ss->color = source_surface->color` in the geometry generator is the correct approach.

## What Would Fail Without Changes

### Scenario A: Chamfer surface has no trim curves
If `ss->trim.IsEmpty()` returns true (no trim polygon populated), the surface is **silently
skipped** in STEP export:
```cpp
if(ss.trim.IsEmpty()) continue;
```
This would result in missing faces in the STEP output. The watertightness work in
`MakeFromChamferOf()` must ensure trim curves are properly set up.

### Scenario B: Incorrect surface orientation
If chamfer/fillet surface normals point inward instead of outward, the STEP `ADVANCED_FACE`
will have wrong orientation, causing import failures. The `SSurface::Reverse()` method
can flip orientation if needed.

### Scenario C: Discontinuous trim loops
The `FindOuterFacesFrom()` call in `ExportSurface()` expects closed contours. If trim curves
don't close properly (watertightness issue), the contour detection fails.

## Summary of Required Changes for STEP Export

| Change | Required? | Reason |
|--------|-----------|--------|
| New STEP entity types | ❌ No | Existing NURBS surface format covers chamfer and fillet |
| New degree handling | ❌ No | Degrees 1 and 2 already supported |
| Rational weight export | ❌ No | Already outputs weights for all surfaces |
| Color inheritance | ✅ Yes | Set `ss->color` in geometry generator |
| Trim curve population | ✅ Yes | Required for any surface to export at all |
| Watertight SCurve setup | ✅ Yes | Alias detection requires matching endpoints |
| Schema version upgrade | ❌ No | AP203 CONFIG_CONTROL_DESIGN handles B-rep fully |

## Complete STEP Export Pipeline (No Changes Needed)

```
SShell (after MakeFromChamferOf or MakeFromFilletOf)
  ↓
ExportSurfacesTo():
  Group.runningShell.surface (iterates ALL surfaces including chamfer/fillet)
  ↓
  For each SSurface:
    MakeSectionEdgesInto() → SBezierList (trim curves as Bezier loops)
    ExportSurface() → writes ADVANCED_FACE with correct NURBS + trims
  ↓
CLOSED_SHELL → MANIFOLD_SOLID_BREP → ADVANCED_BREP_SHAPE_REPRESENTATION
```

**Conclusion**: The STEP exporter is surface-agnostic. Chamfer and fillet surfaces
(as `SSurface` objects in `runningShell`) will be exported automatically with ZERO
changes to `exportstep.cpp`, provided:
1. The surfaces have non-empty `trim` polygons
2. The surface normals point outward
3. The trim curve endpoints match within `PRECISION = 2*LENGTH_EPS`

These are all requirements of the geometry generator, not the exporter itself.

## STEP Interoperability Notes

### Chamfer (Degree 1,1, Non-Rational)
- Exports as a planar bilinear patch in STEP
- Some STEP importers may convert this to a PLANE entity internally
- All major CAD tools (Fusion 360, SolidWorks, FreeCAD, etc.) handle this correctly

### Fillet (Degree 2,1, Rational, w = cos(dihedral/2))
- Exports as an exact cylindrical NURBS surface
- STEP AP203 and AP214 both support rational B-spline surfaces
- The exact circle representation (via rational NURBS) is preferred over approximation
- Most CAD importers recognize degree-2 rational NURBS as cylinders and may show them
  with appropriate feature recognition (cylindrical face, radius info)

### Vertex Caps (Chamfer only, small triangular/polygonal patches)
- Degree (1,1) planar patches with triangular trim loops
- Standard STEP handling — no issues expected

## References

- `src/exportstep.cpp` — Full implementation reviewed above
- `ExportSurfacesTo()` at exportstep.cpp:437 — Entry point iterates all surfaces
- `ExportSurface()` at exportstep.cpp:295 — Writes B_SPLINE_SURFACE entity
- `ExportCurveLoop()` at exportstep.cpp:226 — Writes trim loop as EDGE_LOOP
- `PRECISION = 2*LENGTH_EPS` at exportstep.cpp:12 — Point matching tolerance
- ISO 10303-42 (STEP geometry) — B_SPLINE_SURFACE entities
- ISO 10303-203 (AP203) — CONFIG_CONTROL_DESIGN schema used by SolveSpace
