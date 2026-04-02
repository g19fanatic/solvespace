# Task 30: CGAL and Geometric Algorithms for Fillet/Chamfer

## Overview

CGAL (Computational Geometry Algorithms Library) is an open-source C++ library providing
computational geometry algorithms. This research examines CGAL's algorithms relevant to
chamfer/fillet operations and how they conceptually relate to SolveSpace's implementation needs.

**Key finding: SolveSpace has no CGAL dependency and cannot use CGAL directly.** However,
CGAL's algorithms illuminate the mathematical foundations and provide algorithmic insight
that can inform SolveSpace's custom implementation.

---

## 1. Minkowski Sum — The Mathematical Foundation of Fillets

### Definition
The **Minkowski sum** of two sets A and B is: `A ⊕ B = {a + b | a ∈ A, b ∈ B}`.

Geometrically, the Minkowski sum of a solid with a sphere of radius r produces an **offset solid**
where every surface point has been moved outward by distance r. This is the theoretical
foundation of rounding/fillet operations.

**Fillet as Minkowski Sum:**
1. Compute offset solid `S ⊕ Ball(r)` — expands solid outward by r
2. Compute deflation (Minkowski difference): `(S ⊕ Ball(r)) ⊖ Ball(r)`
3. The "rolling ball" fillet is the surface of the offset solid minus the interior regions

The **Minkowski difference** (erosion) is: `A ⊖ B = {x | x+B ⊆ A}`.

In 2D image processing, the Minkowski sum/difference are called "dilation" and "erosion".

### CGAL's Minkowski Sum Packages

#### 2D Minkowski Sum (`PkgMinkowskiSum2`)
- Introduced CGAL 3.3
- Computes exact Minkowski sum of two simple polygons in the plane
- Also computes "polygon offset" = Minkowski sum of polygon with a disc (circle)
- The offset polygon is the mathematical equivalent of a 2D fillet
- Implementation depends on `2D Arrangements`, `2D Fast Intersection`, `2D Triangulations`
- License: GPL

**Relevance for SolveSpace 2D fillet:**
The 2D polygon offset operation IS a 2D fillet. For two lines meeting at a corner, offsetting
with a disc of radius r produces the tangent arc between the lines. This is the exact
algorithm underlying 2D CAD fillet.

#### 3D Minkowski Sum of Polyhedra (`PkgMinkowskiSum3`)
- Introduced CGAL 3.5
- Computes Minkowski sum of two sets in R³
- Uses: 3D Boolean Operations on Nef Polyhedra + Convex Decomposition of Polyhedra
- Can compute the configuration space of translational robots
- The "glide operation" sweeps a polyhedron along a polygonal line — relevant to extrusion

**Conceptual relevance for 3D chamfer/fillet:**
- Fillet = Minkowski sum of original solid with ball of radius r, then erode by ball
- Chamfer = Minkowski sum with a cube or truncated octahedron (fixed distance, planar result)
- However: SolveSpace uses EXACT NURBS B-rep, not polyhedral meshes
- CGAL's Minkowski sum operates on polyhedral meshes — NOT directly applicable

---

## 2. CGAL 2D Straight Skeleton and Polygon Offsetting

### What Is a Straight Skeleton?
A **straight skeleton** of a polygon is a topological skeleton (tree) computed by shrinking
the polygon inward at uniform speed along each edge. The bisectors of adjacent edges trace
out the skeleton.

- First defined by Aichholzer et al. (1995) for simple polygons
- Generalizes the medial axis but uses only straight line segments
- Interpreted as "roof surface projection" — rooftop ridges over a floor plan

### CGAL Package: `2D Straight Skeleton and Polygon Offsetting` (`CGAL::SS_2`)
- Introduced CGAL 3.2 (Cacciola, Loriot, Rouxel-Labbé)
- Implements weighted straight skeleton in interior of 2D polygons with holes
- Constructs inward offset polygons at any distance given a straight skeleton
- License: GPL

**Algorithm:**
1. Compute straight skeleton (wavefront propagation, O(n log n) or O(n²) in worst case)
2. At each offset distance t, intersect the wavefront to get offset polygon
3. Handles "collapse events" where polygon corners meet

**Why straight skeleton produces CHAMFER, not fillet:**
The offset polygon produced by straight skeleton offsetting has straight edges and
*chamfered* corners (the bisector lines create mitered/chamfered corners, not arcs).
For smooth (fillet) offsets, one needs CIRCULAR arc corners instead.

**Direct relevance to chamfer:** The 2D chamfer algorithm for a polygon is equivalent to
inward/outward offsetting the polygon using a straight skeleton approach with
distance d at each edge. The corner treatments are chamfered (planar cut) not rounded.

---

## 3. CGAL Polygon Mesh Processing — Relevant Algorithms

### Feature Detection
`CGAL::Polygon_mesh_processing::detect_sharp_edges()` / `sharp_edges_segmentation()`:
- Detects edges where the dihedral angle between adjacent faces exceeds a threshold
- Computes surface patches separated by sharp edges
- Directly analogous to what SolveSpace needs: "find all edges that are sharp enough
  to merit a chamfer/fillet"

This CGAL function shows the algorithm:
1. For each edge: compute dihedral angle between adjacent face normals
2. If angle > threshold (e.g., 45°): mark as sharp
3. Group faces into patches separated by sharp edges

**SolveSpace equivalent:** iterate SShell::curve, check angle between surfA and surfB normals.

### Boolean Operations (Corefinement)
CGAL's `corefine_and_compute_difference()` is the "right" way to implement chamfer as
a boolean subtraction. The algorithm:
1. **Corefine** two meshes: find all intersections, insert new edges at intersection polylines
2. **Tag** faces as inside/outside the other mesh
3. **Extract** the desired result (union/intersection/difference)

This is EXACTLY what SolveSpace's `SShell::MakeFromBoolean` does, but for B-rep NURBS
surfaces instead of triangle meshes.

**Critical insight:** CGAL uses corefinement (triangle mesh splitting at intersections),
but SolveSpace uses a "keeping" approach (keep faces from each shell based on inside/outside
classification). Both achieve boolean operations, but different implementations.

### Isotropic Remeshing and Curvature
CGAL's `isotropic_remeshing()` has a curvature-adaptive sizing field (`Adaptive_sizing_field`)
that puts shorter edges in high-curvature regions. This is relevant for fillet triangulation:
fillet surfaces have non-zero curvature along the arc direction, so adaptive meshing would
naturally refine the fillet region.

SolveSpace's triangulation uses a chord tolerance (`ChordTolMm()`) which achieves similar
adaptive refinement for curved surfaces.

---

## 4. CGAL 3D Nef Polyhedra — The Exact Boolean Framework

### What Are Nef Polyhedra?
**Nef polyhedra** (3D Boolean Operations on Nef Polyhedra package) support:
- EXACT boolean operations (union, intersection, difference, complement)
- Non-manifold geometry (isolated vertices, edges, etc.)
- Open and closed sets
- General topology (unlike simple convex or manifold meshes)

**Chamfer/fillet via Nef polyhedra:**
1. Build the source solid as Nef polyhedron
2. Build the "cutter" prism (for chamfer) or rolling-ball swept volume (for fillet)
3. Boolean difference: `solid ⊖ cutter = chamfered solid`

CGAL's Nef polyhedra use exact arithmetic (CGAL Exact kernel) to guarantee correctness.
This is expensive but robust.

**SolveSpace vs Nef polyhedra:**
- SolveSpace uses exact NURBS B-rep (analytically exact for planes and cylinders)
- No floating-point approximation needed for chamfer (planes) or simple fillet (cylinders)
- SolveSpace IS already doing exact geometry, just with different data structures

---

## 5. Key Algorithms from CGAL Relevant to Chamfer/Fillet Implementation

### 5.1 Offset Algorithm (2D)
For 2D chamfer: `Minkowski_sum_2` or straight skeleton inward offset
- Input: polygon with corners
- Output: offset polygon where corners are replaced by chamfer cuts (straight skeleton)
  or arc transitions (Minkowski sum with circle)
- **SolveSpace equivalent:** `MakeTangentArc()` for arcs (2D fillet), or the new
  parametric chamfer/fillet Group for 2D line-line junctions

### 5.2 Rolling Ball Fillet via Offset Surface
For 3D fillet: conceptually `P ⊕ Ball(r)` then `⊖ Ball(r)`
- Operationally: trace the center of a ball of radius r rolling along each edge of the solid
- The traced locus is the "spine" of a canal surface
- **SolveSpace MVP:** skip canal surface spine, instead directly compute cylindrical arc
  tangent to both face planes (works for planar faces only — confirmed MVP scope)

### 5.3 Feature Detection
For auto-chamfer: CGAL's `detect_sharp_edges()` algorithm:
```
for each SCurve c in SShell:
    n1 = surfA.NormalAt(midpoint_u, midpoint_v)
    n2 = surfB.NormalAt(midpoint_u, midpoint_v)
    angle = acos(n1.Dot(n2))
    if angle < threshold: c.is_sharp = true
```
**SolveSpace implementation:** iterate `shell.curve`, for each SCurve compute face normals,
classify as sharp/smooth. Used for future "auto-chamfer all sharp edges" feature.

### 5.4 Polyhedral Envelope (CGAL::Polyhedral_envelope)
CGAL has a class for checking if geometry stays within a tolerance envelope of the input.
For chamfer/fillet validation:
- Generate chamfer/fillet geometry
- Verify it lies within ε of the theoretical perfect geometry
- **SolveSpace equivalent:** not needed — SolveSpace generates exact geometry (degree-1
  planes for chamfer, degree-2 exact cylinders for fillet)

### 5.5 Sharp Edge Detection Algorithm
```python
# CGAL sharp_edges_segmentation equivalent in pseudo-SolveSpace:
for curve in shell.curve:
    # Get the two adjacent surfaces
    surfA = shell.GetSurface(curve.h.surfA)
    surfB = shell.GetSurface(curve.h.surfB)
    
    # Sample midpoint of the curve for normal computation
    mid = curve.bezier[0].PointAt(0.5)
    
    # Compute face normals at adjacent points
    n1 = surfA.NormalAt(mid, epsilon_offset_into_surface)
    n2 = surfB.NormalAt(mid, epsilon_offset_into_surface)
    
    # Dihedral angle
    dihedral = acos(n1.Dot(n2))
    
    if dihedral > sharp_threshold:  # e.g., 30 degrees
        # This edge is a candidate for chamfer/fillet
        mark_as_sharp(curve)
```

---

## 6. The Straight Skeleton Algorithm — Deep Dive

The straight skeleton is particularly relevant for **2D chamfer** of polygons:

### Algorithm for 2D Chamfer via Straight Skeleton

**Input:** Polygon with vertices V1, V2, ..., Vn
**Parameter:** Chamfer distance d

**For each corner Vi:**
1. Compute inward bisector `bi = normalize(ei-1 + ei)` where `ei-1, ei` are inward edge normals
2. Compute chamfer distance along each edge: `di = d / sin(θi/2)` where `θi` is angle at Vi
3. Tangent points: `T1 = Vi - di * ei-1_direction`, `T2 = Vi + di * ei_direction`
4. Replace corner Vi with edge `T1--T2`

This is exactly the 2D chamfer algorithm, producing a flat cut at each sharp corner.

### CGAL `CGAL::Straight_skeleton_2` gives exactly this for polygon offsetting:
- Inward offsetting at distance d = chamfer
- Straight skeleton tracks which corners collapse first (for large d values)
- Handles intersections between adjacent chamfer regions

**For SolveSpace 2D chamfer sketch group:**
The parametric Group::CHAMFER at sketch level (future feature) would use this algorithm.
But for MVP, the 3D chamfer between flat faces is the priority.

---

## 7. Offset Surfaces — 3D Mathematical Foundation

### What Are Offset Surfaces?
From the Wikipedia article on parallel curves/offset surfaces:
- An **offset surface** of surface S at distance r is the locus of points at distance r along
  the surface normal: `S_r(u,v) = S(u,v) + r * N(u,v)`
- Offset surfaces are important in NC machining (ball-nose end mill paths)
- For a PLANE: offset surface = parallel plane (exact, trivial)
- For a CYLINDER of radius R: offset surface = concentric cylinder of radius R±r (exact)
- For general surfaces: offset can be irrational (not expressible in closed form)

### Relevance for SolveSpace Fillet:
- Fillet between two flat faces = NOT an offset surface operation
  - Fillet surface = cylinder tangent to both planes
  - This is SIMPLER than a general offset surface
- The "rolling ball" fillet IS the intersection of the two offset planes plus an arc between them
- For flat faces: fillet radius r produces offset planes at distance r, their intersection
  defines the edge endpoints, and the fillet surface is a cylinder of radius r

**Mathematical derivation:**
```
Face1: plane P1 with normal n1
Face2: plane P2 with normal n2
Fillet radius: r
Offset plane 1: P1' = P1 + r*n1 (inward offset)
Offset plane 2: P2' = P2 + r*n2 (inward offset)
Fillet center spine: intersection of P1' and P2' (a line)
Fillet surface: cylinder of radius r with axis = spine line
```

This is EXACTLY the algorithm implemented in SolveSpace for fillet between flat faces.

---

## 8. CGAL Summary: What SolveSpace Can Learn

| CGAL Algorithm | SolveSpace Equivalent | Notes |
|---|---|---|
| `Minkowski_sum_2` (disc offset) | `MakeTangentArc()` | 2D fillet at corners |
| `Straight_skeleton_2` (polygon offset) | New 2D sketch chamfer group | Chamfer corners |
| `BooleanOps::corefine_and_compute_difference` | `SShell::MakeFromBoolean` | Already exists |
| `detect_sharp_edges()` | Custom SCurve classifier | For auto-chamfer |
| `Nef_polyhedra` boolean ops | SolveSpace B-rep boolean | SolveSpace is more precise |
| Offset surface formula | Direct `SSurface` geometry | Exact NURBS, no approximation |
| Polyhedral_envelope | N/A | SolveSpace generates exact geometry |

**Key insight: SolveSpace's exact NURBS geometry is MORE mathematically precise than
CGAL's triangular mesh approximations.** For:
- Chamfer (planar face): `SSurface::FromPlane()` = exact bilinear patch
- Fillet (cylindrical face): `SSurface::FromExtrusionOf(arc)` = exact rational NURBS degree-2
- Both are exact representations, not approximations

CGAL's triangle meshes approximate these with potentially thousands of triangles.
SolveSpace represents them with 4 control points (chamfer) or 6 control points (fillet).

---

## 9. Algorithms NOT Applicable to SolveSpace

### CGAL's Nef Polyhedra (too heavy)
- Requires floating-point exact arithmetic kernel
- Operates on polyhedral meshes, not NURBS
- Performance: much slower than SolveSpace's direct topology injection

### CGAL's 3D Minkowski Sum (wrong representation)
- Operates on polyhedral meshes
- Would destroy the exact NURBS structure
- Output would need to be converted back to NURBS — lossy

### CGAL's Remeshing algorithms
- SolveSpace doesn't use triangulation for modeling (only for display)
- Remeshing only relevant for final export/display, not for the B-rep geometry

---

## 10. Relevant Algorithms for SolveSpace Implementation

### For MVP Chamfer (planar faces):
1. Compute edge tangent `t = V2-V1 / |V2-V1|`
2. Compute inward normals: `d1 = t.Cross(n1)`, `d2 = n2.Cross(t)` (from AGENT.md learnings)
3. Offset edge endpoints by distance `d` along each face: `A = V1+d*d1, B = V2+d*d1, C = V2+d*d2, D = V1+d*d2`
4. Create chamfer surface: `SSurface::FromPlane(A, D-A, B-A)` 
5. Trim adjacent face polygons to remove the chamfered region

### For Auto-Sharp-Edge Detection (future feature, CGAL-inspired):
```cpp
for (auto& c : shell.curve) {
    SSurface* sA = shell.SurfaceById(c.h.surfA);
    SSurface* sB = shell.SurfaceById(c.h.surfB);
    // Sample curve midpoint
    Vector mid = c.bezier[0].PointAt(0.5);
    // Get face normals facing INTO the solid
    Vector n1 = sA->NormalAt(0.5, 0.5).Negated();
    Vector n2 = sB->NormalAt(0.5, 0.5).Negated();
    double dihedral = acos(n1.Dot(n2));
    if (dihedral < thresholdAngle) {
        sharpEdges.push_back(c.h);
    }
}
```

---

## 11. Summary of Key Takeaways for SolveSpace

1. **Minkowski sum** is the theoretical foundation of both chamfer and fillet, but SolveSpace
   bypasses it entirely by directly constructing the resulting NURBS geometry.

2. **Straight skeleton** is the 2D chamfer algorithm — useful if SolveSpace ever adds
   a "2D sketch chamfer" group feature.

3. **CGAL's detect_sharp_edges()** provides the algorithm for future "auto-chamfer all
   sharp edges" capability: iterate edges, classify by dihedral angle threshold.

4. **SolveSpace's exact NURBS is superior to CGAL's mesh approximation** for chamfer/fillet:
   - Chamfer → exact degree-1 planar NURBS (2 triangles when displayed)
   - Fillet → exact degree-2 rational NURBS cylinder (mathematically exact circle)
   - No approximation error, unlike CGAL triangle meshes

5. **The hardest part is NOT the geometry generation** (which is a few lines of math) but
   the **trim polygon modification** — removing the chamfered/filleted region from adjacent
   face boundaries. CGAL handles this via corefinement; SolveSpace must do it manually.

6. **CGAL's Polygon_mesh_processing::corefine_and_compute_difference()** is conceptually
   equivalent to SolveSpace's `MakeFromBoolean`, but the key insight from phkahler's PR #1501
   is that the standard boolean pipeline causes COINC_OPP degeneracy for chamfer. **Direct
   topology injection** (bypassing the boolean pipeline) is the correct approach for SolveSpace.

---

## Sources
- CGAL 6.1.1 Package Overview: https://doc.cgal.org/latest/Manual/packages.html
- CGAL Polygon Mesh Processing Manual: https://doc.cgal.org/latest/Polygon_mesh_processing/index.html
- Wikipedia: Minkowski sum
- Wikipedia: Straight skeleton
- Previous findings: 11-fillet-math-and-algorithms.md, 12-chamfer-math-and-algorithms.md,
  16-boolean-pipeline.md, 28-solvespace-chamfer-attempts.md
