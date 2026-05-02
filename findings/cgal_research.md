# OpenSCAD/CGAL/libfive Research: How Alternative CAD Systems Handle Multi-Surface Junctions

## Executive Summary

**OpenSCAD does NOT have native B-rep edge fillet/chamfer operations.** It uses fundamentally different paradigms (CSG + Minkowski sums) that inherently avoid the corner triangle problem. CGAL's Nef polyhedra use exact Boolean operations. libfive uses functional representations (SDFs). None of these systems face the "corner triangle" problem because they operate at a different level of abstraction from B-rep edge operations.

**Key insight for SolveSpace**: The only open-source system that does B-rep edge filleting like SolveSpace is OCCT (researched in task 1). OpenSCAD/CGAL/libfive confirm that the corner triangle is an artifact of SolveSpace's specific B-rep approach — professional B-rep systems (OCCT) solve it via surface-surface intersection.

## 1. OpenSCAD — CSG + Minkowski Sum Approach

### Does OpenSCAD Support Edge Chamfer/Fillet?

**No.** OpenSCAD has NO native edge fillet or chamfer operation. It cannot select an edge and apply a fillet/chamfer to it. Instead, OpenSCAD provides:

1. **2D `offset()` with linear_extrude** — For 2D profiles only:
   - `offset(r=-3) offset(delta=+3)` → rounds inside (concave) corners (acts as 2D fillet)
   - `offset(r=+3) offset(delta=-3)` → rounds outside (convex) corners (acts as 2D round)
   - This only works in 2D and must be extruded to 3D
   
2. **3D `minkowski()` sum** — For 3D rounding:
   - `minkowski() { cube(...); sphere(r); }` → rounds ALL edges with radius r
   - Cannot selectively fillet individual edges
   - Rounds corners naturally as part of the Minkowski sweep
   - Very computationally expensive (O(n²) or worse)
   
3. **Manual CSG operations** — `difference()` and `intersection()`:
   - Users must manually construct fillet geometry
   - e.g., subtract a cylinder to create a concave fillet

### How OpenSCAD Handles Corners (in Minkowski approach)

When using `minkowski()` with a sphere to round edges, **corners are handled automatically**:
- The Minkowski sum of a polyhedron with a sphere produces rounded edges AND rounded corners
- The corner rounding is a natural consequence of the mathematical operation
- No separate "corner surface" is ever created — the sphere traces smoothly around vertices
- The vertex of the original polyhedron becomes a spherical patch on the result
- **This is fundamentally different from B-rep edge operations** where you must handle edge-edge meetings explicitly

### OpenSCAD's Geometry Backend

OpenSCAD uses CGAL's Nef polyhedra for all Boolean/CSG operations internally:
- Source: `src/geometry/cgal/` contains CGAL integration code
- `CGALHybridPolyhedron.cc` — Hybrid mesh representation
- `boolean_utils.cc` — Boolean operations using CGAL
- Also uses the "Manifold" library as an alternative geometry backend
- All operations produce manifold meshes — no naked edges, no corner issues

### Why OpenSCAD Doesn't Have the Corner Triangle Problem

1. **No edge-level operations**: OpenSCAD never selects individual edges for modification
2. **Volumetric approach**: All operations work on solid volumes (CSG tree)
3. **Minkowski is global**: Rounding is applied to the entire solid, not edge-by-edge
4. **Boolean operations are exact**: CGAL ensures watertight manifold results
5. **No B-rep trim curve management**: There are no trim curves to manage at multi-surface junctions

## 2. CGAL — Nef Polyhedra and Boolean Operations

### CGAL's Approach to Geometry

CGAL provides exact geometric computation via Nef polyhedra:

- **Nef polyhedra** = point sets generated from halfspaces by complement and intersection
- **Closed under ALL Boolean operations**: union, intersection, difference, complement
- **Representation**: Selective Nef Complex (SNC) — boundary with selection marks
- **Exact arithmetic**: Uses exact number types (no floating point errors)

### Does CGAL Have Edge Fillet/Chamfer?

**No.** CGAL does not provide a "fillet this edge" operation. CGAL handles:
- Exact Boolean operations on polyhedra
- Minkowski sum of 3D Nef polyhedra (via Gaussian map decomposition)
- Point location queries
- Mesh conversion (Nef → Surface_mesh → STL)

For rounding/filleting, CGAL would use:
1. **Minkowski sum with sphere** — `minkowski_sum_3(polyhedron, sphere_nef)` rounds all edges
2. **Boolean operations** — Can approximate fillets by intersecting with appropriate shapes
3. **Mesh smoothing** — Post-process mesh vertices (not exact fillet)

### CGAL Minkowski Sum (3D)

From the CGAL documentation:
- Decomposes both polyhedra into convex pieces
- Computes all pairwise Minkowski sums of convex pieces
- Merges the pairwise sums
- Complexity: O(n³m³) — very expensive!
- Produces exact results with no topological issues

The Minkowski sum with a sphere naturally handles multi-surface junctions:
- At a vertex where 3+ edges meet, the sphere traces a smooth spherical patch
- No separate corner surface needed — it's a continuous deformation

### Why CGAL Doesn't Have Corner Triangles

1. **Volumetric operations**: Works on point sets, not B-rep with trim curves
2. **Exact computation**: No approximations that require "filling gaps"
3. **Global operations**: Cannot produce isolated corner artifacts because all operations are global
4. **Nef polyhedra closure**: Boolean operations always produce valid results

## 3. libfive — Functional Representation (SDF) Approach

### How libfive Works

libfive uses **functional representations** (f-reps / signed distance functions):
- Objects defined as mathematical functions f(x,y,z) → distance from surface
- Inside: f < 0, Outside: f > 0, Surface: f = 0
- CSG operations are simple: union = min(a,b), intersection = max(a,b), complement = -f

### Fillet/Chamfer in libfive

libfive achieves fillets through mathematical blending operations:
```scheme
(define (blend a b m)
  (min a b (+ (sqrt (abs a))
              (sqrt (abs b))
              (- m))))
```

This produces smooth blends at intersections of ANY surfaces — including multi-surface corners.

### Why SDF Approaches Don't Have Corner Triangles

1. **Implicit surfaces**: No explicit B-rep with trim curves to manage
2. **Mathematical blending**: Smooth transitions are a property of the math, not constructed geometry
3. **No topology management**: Surface topology emerges from the function evaluation
4. **Corners blend naturally**: The blending function applies uniformly regardless of how many surfaces meet

## 4. ImplicitCAD (Haskell SDF)

ImplicitCAD (now appears to be archived/unmaintained) used a similar approach to libfive:
- Functional representations for 3D solids
- Mathematical operations for CSG
- Chamfers and fillets via offset operations
- No corner triangle issues for the same reasons as libfive

## 5. Synthesis: Relevance to SolveSpace

### The Fundamental Difference

| System | Approach | Corner Handling |
|--------|----------|-----------------|
| **SolveSpace** | B-rep with NURBS trim curves, edge-by-edge operations | Creates corner triangle (WRONG) |
| **OCCT/FreeCAD** | B-rep with surface-surface intersection | Surface intersection curve as shared trim (CORRECT) |
| **OpenSCAD/CGAL** | CSG + Minkowski on Nef polyhedra | N/A — no edge-level operations |
| **libfive** | SDF/f-rep with mathematical blending | N/A — implicit surfaces, no trim curves |

### What SolveSpace Can Learn

1. **From OCCT** (most relevant): Use surface-surface intersection to produce the trim boundary between fillet and chamfer. This is the direct analog — both are B-rep systems with trim curves.

2. **From CGAL/OpenSCAD**: The corner triangle is an artifact of incomplete topology handling. In a mathematically correct system, the junction of multiple surfaces either:
   - Has a natural smooth transition (Minkowski/SDF)
   - Has a computed intersection curve serving as the shared boundary (OCCT)
   - NEVER requires a separate flat "filler" surface

3. **From libfive**: The concept of "blending" surfaces at corners shows that the flat triangle is geometrically WRONG. A correct corner would be either:
   - The intersection curve of the two existing surfaces (OCCT approach)
   - A smooth transition (but SolveSpace doesn't need this — the fillet IS the smooth surface)

### Conclusion for Implementation

The research confirms what OCCT research already established:
- **The corner triangle is NOT topologically necessary** — no other system creates one
- **The correct approach for B-rep is surface-surface intersection** (OCCT approach)
- **For SolveSpace's specific case**: The fillet cylinder and chamfer plane should intersect to produce the trim boundary. The fillet's existing arc (`hArcV1.surfB = hCapSurfV1`) already references the chamfer surface — the simplest fix may be to skip the corner triangle entirely (Approach A) since the topology may already close correctly.

## 6. OpenSCAD Fillet Tips & Tricks (Community Approaches)

The OpenSCAD community documentation shows 3D filleting via `offset_3d()` using Minkowski:

```openscad
module offset_3d(r=1) {
    if (r > 0)
        minkowski() {
            children();
            sphere(r);
        }
    else {
        difference() {
            cube(size, center=true);
            minkowski() {
                difference() {
                    cube(size*2, center=true);
                    children();
                }
                sphere(-r);
            }
        }
    }
}
```

This confirms: in CSG approaches, filleting is a GLOBAL operation and corners are handled naturally by the mathematical properties of Minkowski sums.
