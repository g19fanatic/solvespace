# Research Synthesis: Eliminating the Corner Triangle in SolveSpace CF Endcaps

## 1. Comparative Analysis

### 1.1 OCCT / FreeCAD (B-rep, DIRECTLY comparable to SolveSpace)

**Approach**: Surface-surface intersection at multi-surface junctions.

OCCT handles corners (vertices where fillet/chamfer "stripes" meet) with a hierarchy:

| Stripes at Vertex | OCCT Function | Strategy |
|---|---|---|
| 1 | `PerformOneCorner` | Extend single stripe to vertex |
| 2 | `PerformTwoCornerSameExt` | **Surface-surface intersection** → shared trim curve |
| N≥3 | `PerformMoreThreeCorner` | **GeomPlate** smooth interpolating surface (NOT flat triangle) |

**Key function**: `ChFi3d_ComputeCurves(HS1, HS2, Pardeb, Parfin, cint, C2dint1, C2dint2, tol)`
- Computes surface-surface intersection between two stripe surfaces
- Produces: 3D curve (`cint`) + 2D parametric curve on each surface (`C2dint1`, `C2dint2`)
- The intersection curve is stored as a **shared trim boundary** on both surfaces
- **NO separate corner surface is created for 2-stripe corners**

**Result**: At a chamfer→fillet junction, the fillet cylinder and chamfer plane intersect along
an elliptical arc. This arc IS the trim boundary. Both surfaces are trimmed to it. No gap, no
separate surface, no corner triangle.

### 1.2 OpenSCAD / CGAL (CSG + Minkowski, NOT directly comparable)

**Approach**: Volumetric global operations, not per-edge B-rep modifications.

- OpenSCAD has **NO native edge fillet/chamfer** operations
- Uses `minkowski()` with sphere for global 3D rounding (rounds ALL edges)
- Uses `offset()` for 2D only (then linear_extrude)
- CGAL provides Nef polyhedra for exact Booleans; `minkowski_sum_3()` for rounding
- CGAL has NO "fillet an edge" API — only Minkowski sum with sphere (O(n³m³))
- Corners are handled naturally by the Minkowski sweep — the sphere traces smoothly

**Result**: Corner triangle problem doesn't arise because these systems never create
per-edge B-rep modifications. Operations are global and volumetric.

### 1.3 libfive / ImplicitCAD (SDF / f-rep)

**Approach**: Mathematical blending via signed distance functions.

- Objects = `f(x,y,z) → distance`; CSG = `min`/`max`; fillets = mathematical blending
- `(blend a b m) = min(a, b, (sqrt(abs a) + sqrt(abs b) - m))`
- Corners blend naturally — no topology management needed
- No explicit B-rep, no trim curves, no surface patches

**Result**: Fundamentally different paradigm. Corner triangle artifact is impossible.

## 2. Summary Comparison

| System | Representation | Corner Handling | Corner Triangle? |
|--------|---------------|-----------------|-----------------|
| **SolveSpace** | B-rep NURBS + trim curves | Flat `FromPlane` triangle | **YES (WRONG)** |
| **OCCT/FreeCAD** | B-rep + intersection | Surface-surface intersection curve | **NO** |
| **OpenSCAD/CGAL** | CSG + Minkowski Nef | Global volumetric operation | **NO** (N/A) |
| **libfive** | SDF/f-rep | Mathematical blending | **NO** (N/A) |

**Conclusion**: Only OCCT is directly comparable. Every system surveyed confirms the corner
triangle is a SolveSpace-specific artifact. The OCCT approach (intersection curve) is the
correct B-rep solution.

## 3. Fundamental Approach Difference from SolveSpace

### What SolveSpace does (WRONG)

At the corner vertex V1 where a fillet meets a chamfer, the current code (chamfer.cpp:2956-3115):

1. Finds bridge pairs sharing vertex V1
2. Creates a **separate flat surface** (`SSurface::FromPlane(V1, A0-V1, B0-V1)`) — the "corner triangle"
3. Creates NC1 (A0→V1) shared between corner triangle and cap-face-A
4. Creates NC2 (V1→B0) shared between corner triangle and cap-face-B
5. Creates NC3 (A0→B0) shared between corner triangle and fillet — **replaces** the fillet's arc
6. The fillet surface is **CUT SHORT** at NC3 (a straight line), losing its curved endcap

**The fillet arc `hArcV1` is destroyed** — replaced by NC3, a straight line. The corner triangle
fills the gap between the fillet (now shortened) and the adjacent surfaces.

### What OCCT does (CORRECT)

For a 2-stripe corner (chamfer + fillet at same vertex):
1. Computes surface-surface intersection of the fillet cylinder and chamfer plane
2. The intersection curve (elliptical arc) becomes the **shared trim boundary**
3. Both surfaces are trimmed to this intersection curve
4. **NO separate surface needed** — the two surfaces share the intersection as their boundary

### The Fundamental Difference

**SolveSpace**: Creates a separate surface to fill a gap caused by cutting the fillet short.
**OCCT**: Doesn't cut the fillet short — lets it continue to the natural intersection with the chamfer.

The root cause in SolveSpace is that the code **removes** the fillet's arc (`hArcV1`) and
**replaces** it with a straight line (NC3), which shortens the fillet. Then a flat triangle
fills the resulting gap. If the fillet kept its arc, the gap would not exist.

## 4. Mathematical Analysis: CF Corner Geometry

### The 4 surfaces meeting at corner vertex V1:

1. **hFillet** — Cylindrical NURBS surface (fillet). Bounded by arc `hArcV1` at V1 end.
2. **hCapSurfV1** — Flat planar surface (chamfer cap). Currently, `hArcV1.surfB = hCapSurfV1`.
3. **hSurfA0** — Body face (cap-face-A). Has bridge edge to V1.
4. **hSurfB0** — Body face (cap-face-B). Has bridge edge to V1.

### Key geometric fact: V1, A0, B0 are coplanar with hCapSurfV1

The chamfer cap (hCapSurfV1) is a flat plane. The corner vertex V1 and the bridge endpoints
A0 and B0 all lie on (or very near) this plane. This means:
- The triangular area V1→A0→B0 is geometrically part of the chamfer cap plane
- No separate surface is needed to represent this area
- The chamfer cap can be **extended** to cover the corner area

### Key topological fact: hArcV1 already connects fillet to chamfer cap

The fillet arc `hArcV1` has:
- `surfA = hFillet` (the fillet surface)
- `surfB = hCapSurfV1` (the chamfer cap)

This means the arc **already defines the boundary** between fillet and chamfer cap.
If we don't replace this arc with NC3, the boundary relationship is preserved.

### Desired topology WITHOUT corner triangle:

**Edges at the corner:**
- `hArcV1` (A0→B0, curved): shared between **hFillet** and **hCapSurfV1** ✓
- `NC1` (A0→V1, linear): shared between **hCapSurfV1** and **hSurfA0** ✓
- `NC2` (V1→B0, linear): shared between **hCapSurfV1** and **hSurfB0** ✓

**Vertex connectivity:**
- A0: endpoint of hArcV1 + endpoint of NC1 → connects hFillet, hCapSurfV1, hSurfA0 ✓
- B0: endpoint of hArcV1 + endpoint of NC2 → connects hFillet, hCapSurfV1, hSurfB0 ✓
- V1: endpoint of NC1 + endpoint of NC2 → connects hCapSurfV1, hSurfA0, hSurfB0 ✓

**This topology is watertight.** Every edge is shared by exactly 2 surfaces. Every vertex
connects at least 3 surfaces. No gap, no separate corner surface.

The triangular area V1→A0→B0 is now part of hCapSurfV1's domain (the chamfer cap), which
is geometrically correct since V1, A0, B0 all lie on the chamfer plane.

## 5. Concrete Implementation Plan

### Recommended Approach: "Absorb Corner into Chamfer Cap" (Approach A+C Hybrid)

This approach is the simplest and most analogous to OCCT's method:

1. **DON'T** create `SSurface::FromPlane` corner surface
2. **DON'T** create NC3 (the straight line that replaces the fillet arc)
3. **DON'T** replace the fillet's arc `hArcV1` with NC3
4. **DO** create NC1 (A0→V1) with `surfA = hCapSurfV1` (not hCorner)
5. **DO** create NC2 (V1→B0) with `surfA = hCapSurfV1` (not hCorner)
6. **DO** replace bridges in hSurfA0 and hSurfB0 with NC1 and NC2 (same as current code)
7. **DO** add NC1 and NC2 to hCapSurfV1's trim chain (this extends the chamfer cap to cover the corner area)
8. **DO** set `cornerCreated = true` to skip the CF fallback

### Detection: When to apply this approach

Apply when the prior operation is a chamfer (the CF case):
- `hCapSurfV1` is a flat planar surface: `DepartureFromCoplanar(hCapSurfV1) <= LENGTH_EPS`
- The corner area V1→A0→B0 lies on the chamfer cap plane

This detection already exists in the code (for the color-matching fix), so the guard is straightforward.

### Specific Code Changes (chamfer.cpp:2956-3115)

Within the bridge pair loop (after finding `cornerV1`, `A0`, `B0`, `hSurfA0`, `hSurfB0`):

```
// NEW: Detect CF case — hCapSurfV1 is a flat planar surface
SSurface *capSurf = surface.FindByIdNoOops(hCapSurfV1);
bool isCFcase = false;
if(capSurf && capSurf->IsExactly(SSurface::Type::PLANE) /* or coplanar check */) {
    isCFcase = true;
}

if(isCFcase) {
    // --- CF APPROACH: Absorb corner into chamfer cap ---
    // NO corner triangle surface created
    
    // NC1 (A0→V1): shared between hCapSurfV1 and hSurfA0
    hSCurve hNC1 = AddLinearCurve(this, A0, cornerV1, hCapSurfV1, hSurfA0);
    
    // NC2 (V1→B0): shared between hCapSurfV1 and hSurfB0
    hSCurve hNC2 = AddLinearCurve(this, cornerV1, B0, hCapSurfV1, hSurfB0);
    
    // Replace bridges in hSurfA0 and hSurfB0 (same as before)
    SSurface *ssA0 = surface.FindById(hSurfA0);
    for(int ti = 0; ti < ssA0->trim.n; ti++) {
        if(ssA0->trim[ti].curve == bridges[bB].h) {
            ssA0->trim[ti] = STrimBy::EntireCurve(this, hNC1, forkPattern);
            break;
        }
    }
    SSurface *ssB0 = surface.FindById(hSurfB0);
    for(int ti = 0; ti < ssB0->trim.n; ti++) {
        if(ssB0->trim[ti].curve == bridges[bA].h) {
            ssB0->trim[ti] = STrimBy::EntireCurve(this, hNC2, false);
            break;
        }
    }
    
    // Add NC1 and NC2 to hCapSurfV1's trim chain
    // (extends the chamfer cap to cover the corner area V1→A0→B0)
    SSurface *capS = surface.FindById(hCapSurfV1);
    STrimBy stb;
    stb = STrimBy::EntireCurve(this, hNC1, false); // A0→V1
    capS->trim.Add(&stb);
    capS = surface.FindById(hCapSurfV1);
    stb = STrimBy::EntireCurve(this, hNC2, false); // V1→B0
    capS->trim.Add(&stb);
    
    // NO NC3 created — fillet keeps its arc hArcV1
    // The arc already has surfB = hCapSurfV1, so the boundary is correct
    
    cornerCreated = true;
} else {
    // --- Original code: create corner triangle (for non-CF cases) ---
    // ... existing FromPlane + NC1 + NC2 + NC3 code ...
}
```

### Fallback: Approach B (if topology doesn't close)

If Approach A+C leaves topological gaps (naked edges, mesh holes):

**Approach B**: Compute the actual fillet-cylinder ∩ chamfer-plane intersection.
- The fillet surface is an extrusion of a rational quadratic Bezier arc (the fillet profile)
- The chamfer plane is `hCapSurfV1`'s control point plane
- Their intersection is a conic section (elliptical arc for non-perpendicular cuts)
- This intersection curve replaces the fillet arc at V1 with the **actual** intersection
- Both surfaces share this curve as their trim boundary
- This is the closest analog to OCCT's `ChFi3d_ComputeCurves` approach

### Fallback: Approach C (alternative)

If intersection computation is too complex:

**Approach C**: Extend hCapSurfV1's control point mesh to explicitly include V1→A0→B0.
- The chamfer cap surface needs its domain enlarged in UV space
- NC1 and NC2 become standard trims on the enlarged surface
- The fillet arc remains unchanged
- More invasive but avoids intersection math

## 6. Key Risk: Trim Chain Ordering

The main risk with Approach A+C is **trim chain ordering** on hCapSurfV1. SolveSpace
expects trims to form a closed, correctly-ordered loop. When we add NC1 and NC2 to
hCapSurfV1's trim chain, they must be inserted at the correct position in the chain
(after the arc endpoint, before the next existing trim) to maintain a valid loop.

If the trims are just appended at the end, `RepairOpenTrimLoops` (line 3323) may or may
not fix the ordering. This needs testing.

## 7. Test Strategy

### Red Test (verify current code fails):
- Create a CF (chamfer→fillet) configuration
- Check that NO surface has exactly 3 linear trim curves meeting at V1 (corner triangle signature)
- Check that the fillet surface retains its arc trim (not replaced by NC3)

### Green Test (verify fix works):
- Same CF configuration
- Verify no corner triangle surface exists
- Verify the fillet's arc is intact (surfB = hCapSurfV1)
- Verify hCapSurfV1 now has NC1 and NC2 in its trim chain
- Verify no naked edges (topology is watertight)
- Verify all 451 existing OK tests still pass

## 8. Summary

**The corner triangle is NOT topologically necessary.** OCCT proves this for B-rep systems.
OpenSCAD/CGAL/libfive confirm it from other paradigms.

The recommended approach for SolveSpace is to **absorb the corner area into the chamfer
cap surface** by:
1. Skipping the corner triangle creation
2. Keeping the fillet arc intact
3. Adding NC1/NC2 to the chamfer cap's trim chain with `surfA = hCapSurfV1`
4. The chamfer cap, being coplanar with V1/A0/B0, naturally covers the area

This is the simplest B-rep analog to OCCT's intersection approach: rather than computing
a new intersection curve, we observe that the fillet arc `hArcV1` (with `surfB = hCapSurfV1`)
**already IS** the intersection boundary between fillet and chamfer cap. We just need to stop
destroying it and let the chamfer cap extend to cover the now-unnecessary corner area.
