# Boolean Pipeline in SolveSpace — Task 16 Findings

## Overview

SolveSpace maintains **two parallel boolean pipelines** — one for exact rational-polynomial
B-rep surfaces (SShell) and one for triangle meshes (SMesh). Both are used at different
stages of the system. Understanding both is critical for implementing chamfer/fillet.

---

## 1. Dual Pipeline Architecture

### SShell Pipeline (B-rep / NURBS surfaces)
**Files**: `src/srf/boolean.cpp`, `src/srf/surface.h`

Used for the primary geometric representation. Generates exact, watertight B-rep shells.

### SMesh Pipeline (Triangle Mesh / BSP)
**Files**: `src/bsp.cpp`, `src/mesh.cpp`

Used as fallback, for groups that "force mesh", and for export. The SMesh is derived
from SShell via `SShell::TriangulateInto()`. Also used by step-and-repeat groups.

---

## 2. SShell Boolean Pipeline (src/srf/boolean.cpp)

### Top-Level Entry Points

```cpp
// src/srf/boolean.cpp:12-23
void SShell::MakeFromUnionOf(SShell *a, SShell *b);
void SShell::MakeFromDifferenceOf(SShell *a, SShell *b);
void SShell::MakeFromIntersectionOf(SShell *a, SShell *b);
```

All three delegate to the private:
```cpp
void SShell::MakeFromBoolean(SShell *a, SShell *b, SSurface::CombineAs type);
```

### MakeFromBoolean Algorithm (boolean.cpp:934-970)

The algorithm has 7 distinct phases:

**Phase 1: Build Classifying BSPs**
```cpp
a->MakeClassifyingBsps(NULL);
b->MakeClassifyingBsps(NULL);
```
Each `SSurface` gets a `SBspUv *bsp` for its trim polygon in UV space, and `SEdgeList edges` in XYZ. Used for inside/outside classification later.

**Phase 2: Copy Curves (Split Against Other Shell)**
```cpp
a->CopyCurvesSplitAgainst(/*opA=*/true,  b, this);
b->CopyCurvesSplitAgainst(/*opA=*/false, a, this);
```
The existing trim curves from each shell are copied into `this` (the result shell), but first split at any point where they intersect a surface from the _other_ shell. This ensures no piecewise-linear segment of a trim curve ever straddles a surface boundary.

Critical: Each `SCurve` stores `.newH` — its new handle in the result shell. This is used to rewrite surface trims.

**Phase 3: Generate Intersection Curves**
```cpp
a->MakeIntersectionCurvesAgainst(b, this);
```
Calls `SSurface::IntersectAgainst()` for every pair of surfaces (one from A, one from B). New `SCurve` objects with `source = INTERSECTION` are added to the result shell.

**Phase 4: Remove Short Segments**
```cpp
sc.RemoveShortSegments(srfA, srfB);
```
Numerical cleanup — removes intersection curve segments shorter than chord tolerance.

**Phase 5: Rebuild Classifying BSPs (with intersection curves)**
```cpp
a->MakeClassifyingBsps(this);
b->MakeClassifyingBsps(this);
```
Now uses the split+intersection curves to build accurate BSPs for each surface.

**Phase 6: Copy & Trim Surfaces**
```cpp
a->CopySurfacesTrimAgainst(a, b, this, type);
b->CopySurfacesTrimAgainst(a, b, this, type);
```
Each surface is copied into the result shell, but its trim polygon is modified:
- Original trim curves: kept or discarded based on `KeepEdge()` classification
- Intersection curves: added as new trim segments
- `CullExtraneousEdges(both=true)` removes duplicate/antiparallel edges
- `TrimFromEdgeList()` reassembles the final trim polygon

**Phase 7: Rewrite Surface Handles**
```cpp
RewriteSurfaceHandlesForCurves(a, b);
```
Each `SCurve` in the result stores `surfA` and `surfB` handles; these are rewritten to point to the new surface IDs in the result shell.

### KeepRegion and KeepEdge Logic

The core boolean logic is in:
```cpp
// boolean.cpp:280-315
static bool KeepRegion(SSurface::CombineAs type, bool opA, SShell::Class shell, SShell::Class orig);
static bool KeepEdge(SSurface::CombineAs type, bool opA, 
                     SShell::Class indir_shell, SShell::Class outdir_shell,
                     SShell::Class indir_orig, SShell::Class outdir_orig);
```

For **DIFFERENCE** (the operation needed for chamfer):
- `opA=true` (the solid being chamfered): keep regions **OUTSIDE** the cutting tool, or **COINC_OPP** (coincident with opposite normal = interface surfaces kept)
- `opA=false` (the chamfer cut volume): keep regions **INSIDE** the original solid

This is exactly how chamfer works: `solid - chamfer_prism = chamfered_solid`.

### SSurface::MakeCopyTrimAgainst (boolean.cpp:395-545)

The most complex function. Per-surface trimming pipeline:

1. Copy surface geometry (exact, no change)
2. Update existing trim curve handles to new IDs
3. If DIFFERENCE and opB: `ret.Reverse()` (flip normal of the "B" shell faces)
4. Build original trim polygon as SEdgeList (in UV space)
5. Extract intersection curves that affect this surface
6. Determine orientation of intersection curve on this surface (`bkwds` flag)
7. Add intersection curve edges to `inter` edge list
8. Find "choosing points" (vertices where >2 edges meet — T-junctions in trim polygon)
9. `FindChainAvoiding()` segments the edges into chains that don't cross choosing points
10. Each chain is classified: keep or discard via `EdgeNormalsWithinSurface()` + `ClassifyEdge()`
11. Remaining edges assembled via `TrimFromEdgeList()`

### Thread Safety Note

`CopyCurvesSplitAgainst()` and `CopySurfacesTrimAgainst()` use `#pragma omp parallel for`. The critical sections use `#pragma omp critical` for adding to the result. Any new `SShell::MakeFromChamferOf()` method must follow this pattern.

---

## 3. SMesh Boolean Pipeline (src/mesh.cpp + src/bsp.cpp)

### Data Structures

**SMesh** (`src/mesh.h`):
```
SMesh {
    List<STriangle> l;        // triangles, each with STriMeta (face handle + color)
    bool atLeastOneDiscarded;
    bool flipNormal;          // used during boolean ops
    bool keepInsideOtherShell;
    bool keepCoplanar;
}
```

**SBsp3** (`src/bsp.h`): BSP tree of triangles for 3D space partition  
**SBsp2** (`src/bsp.h`): BSP tree of edges for plane/face partition

### SMesh Boolean Operations (mesh.cpp:248-310)

```cpp
void SMesh::MakeFromUnionOf(SMesh *a, SMesh *b);
void SMesh::MakeFromDifferenceOf(SMesh *a, SMesh *b);
void SMesh::MakeFromIntersectionOf(SMesh *a, SMesh *b);
```

All use **BSP tree method**:
1. Build BSP from one mesh: `SBsp3 *bspa = SBsp3::FromMesh(a)`
2. Insert triangles from the other mesh against that BSP: `AddAgainstBsp(b, bspa)`
3. Each triangle is classified: inside/outside/coplanar the BSP
4. Triangles are kept/discarded/flipped based on boolean operation flags

For **DIFFERENCE**:
```cpp
// mesh.cpp:273-283 (MakeFromDifferenceOf)
flipNormal = true;            // flip normals of b-mesh triangles (they become "walls")
keepCoplanar = true;
keepInsideOtherShell = true;  // keep b-triangles that are inside a
AddAgainstBsp(b, bspa);

flipNormal = false;
keepCoplanar = false;
keepInsideOtherShell = false; // keep a-triangles that are OUTSIDE b
AddAgainstBsp(a, bspb);
```

### SBsp3::Insert (bsp.cpp:470-530) — Triangle Classification

Each triangle is classified relative to the BSP plane:
- All positive (above): route to `pos` child
- All negative (below): route to `neg` child
- All on-plane: mark COPLANAR
- Split required: Triangle split into 2-3 pieces, each piece classified separately

Split cases:
- 1 on-plane vertex (posc=1, negc=1, onc=1): split into 2 triangles
- 2 on-plane vertices (posc=2 or negc=2): split into triangle + quad (triangulated)

### SBsp3::InsertInPlane — Coplanar Face Handling

For triangles in the same plane as a BSP node:
```cpp
// bsp.cpp:46-82
// Uses STriangle::ContainsPoint() to check which BSP-node face the triangle center falls on
// sameNormal = dot product of normals > 0
// If keepCoplanar && flipNormal && !sameNormal → keep (for difference, opposite-normal coplanar faces are kept)
// If keepCoplanar && !flipNormal && sameNormal → keep (for union, same-normal coplanar faces are kept)
```

---

## 4. GenerateForBoolean (groupmesh.cpp:182-211)

The bridge between group generation and the boolean pipeline:

```cpp
template<class T>
void Group::GenerateForBoolean(T *prevs, T *thiss, T *outs, Group::CombineAs how) {
    // If no new geometry or suppressed, just copy previous
    if(thiss->IsEmpty() || suppress) {
        outs->MakeFromCopyOf(prevs);
        return;
    }
    
    switch(how) {
        case CombineAs::UNION:        outs->MakeFromUnionOf(prevs, thiss); break;
        case CombineAs::DIFFERENCE:   outs->MakeFromDifferenceOf(prevs, thiss); break;
        case CombineAs::INTERSECTION: outs->MakeFromIntersectionOf(prevs, thiss); break;
        case CombineAs::ASSEMBLE:     outs->MakeFromAssemblyOf(prevs, thiss); break;
    }
}
```

This is a **template function** instantiated for both `SShell` and `SMesh`. A CHAMFER group
using `CombineAs::DIFFERENCE` would call `MakeFromDifferenceOf(prevShell, chamferShell)`.

---

## 5. MakeFromAssemblyOf — Simpler Alternative

```cpp
// boolean.cpp:860-905
void SShell::MakeFromAssemblyOf(SShell *a, SShell *b) {
    // Copy all curves from a and b, assigning new IDs
    // Copy all surfaces, rewriting trim curve handles
    // Rewrite surface handles in curves
}
```

This is much simpler — no boolean computation, just concatenation. Used for ASSEMBLE groups.

**Relevance for chamfer**: If we implement chamfer as direct topology injection (not boolean),
`MakeFromAssemblyOf` is the closest model. We'd:
1. Copy all surfaces from `prevShell` to result (minus trimmed portions of adjacent faces)
2. Add new chamfer surfaces
3. Add new boundary SCurves

---

## 6. Why Direct Topology Injection is Better for Chamfer/Fillet

The `MakeFromBoolean` pipeline was designed for general solid boolean operations where two complete shells intersect in arbitrary ways. For chamfer/fillet:

**Problems with using standard boolean pipeline:**
1. The chamfer volume must be exactly constructed as a watertight SShell — already requires knowing the exact edge geometry
2. Known bug (#1291): When arc profile corner point is on face edge AND tangent, boolean fails
3. Numerical precision issues at shared edges (the chamfer volume shares edges exactly with the original solid)
4. The "DIFFERENCE" approach would create perfectly coincident faces (COINC_OPP), which is a degenerate boolean case
5. phkahler's PR #1501 concluded this approach tends to crash

**Advantages of direct topology injection:**
1. We know exactly which surfaces to modify (the two faces adjacent to the selected edge)
2. We know exactly where new surfaces go (the chamfer/fillet surface between them)
3. We know exactly which SCurves to add (the two boundary curves)
4. No intersection computation needed — all curves are analytically defined
5. No BSP classification needed — we know which trim curve goes on which surface
6. Avoids the boolean tangency bugs entirely

---

## 7. How Chamfer Would Fit — Proposed SShell::MakeFromChamferOf

```cpp
void SShell::MakeFromChamferOf(SShell *src, hSCurve edgeCurve, double dist) {
    // 1. Copy entire source shell (all surfaces and curves)
    //    - Use MakeFromCopyOf pattern
    
    // 2. Find the target SCurve (the edge to chamfer)
    SCurve *ec = src->curve.FindById(edgeCurve);
    SSurface *facA = src->surface.FindById(ec->surfA);
    SSurface *facB = src->surface.FindById(ec->surfB);
    
    // 3. Compute edge tangent vector t at each endpoint
    //    (from ec->exact or ec->pts if pwl)
    
    // 4. For face A: compute inward direction (perpendicular to edge, in face plane)
    //    iA = t × normalA
    //    Offset the edge endpoints by dist*iA → creates A0, A1
    
    // 5. For face B: compute inward direction
    //    iB = normalB × t  
    //    Offset edge endpoints by dist*iB → creates B0, B1
    
    // 6. Create chamfer surface (planar quad: A0-A1-B1-B0)
    //    SSurface chamferSurf = SSurface::FromPlane(A0, A1-A0, B0-A0);
    
    // 7. Create 2 new SCurves:
    //    - scA: line A0-A1 (boundary between faceA and chamferSurf)  
    //    - scB: line B0-B1 (boundary between faceB and chamferSurf)
    
    // 8. For the copied faceA: replace the trim using old edgeCurve
    //    with new trim using scA (trimmed at A0/A1 instead of original endpoints)
    
    // 9. For the copied faceB: replace trim using old edgeCurve
    //    with new trim using scB
    
    // 10. For chamferSurf: add trims for scA, scB, and end-cap edges
    
    // 11. Handle vertices: add triangular or polygonal cap faces at
    //     each endpoint of the original edge (where 3+ faces meet)
}
```

**Critical watertightness requirement**: Every SCurve must be referenced by exactly the 2 surfaces it separates. The 3 new SCurves (scA, scB, and possibly end-cap curves) must each be in the trim lists of exactly 2 surfaces.

---

## 8. SMesh Boolean and Chamfer

For the mesh pipeline, chamfer would need similar direct injection, but operating on triangles:

```cpp
void SMesh::MakeFromChamferOf(SMesh *src, ...) {
    // 1. Copy all triangles from src except those adjacent to the chamfer edge
    // 2. Retriangulate the modified faces (clipped triangles)
    // 3. Add triangles for the chamfer face
    // 4. Add triangles for any vertex caps
}
```

However, the SShell pipeline handles mesh generation implicitly via `SShell::TriangulateInto()`,
so for a chamfer group that uses SShell, the SMesh follows automatically.

---

## 9. SBsp2 — 2D Edge BSP for Coplanar Face Classification

Used inside `SBsp3` for handling coplanar triangles. Each `SBsp3` node stores a `SBsp2 *edges` 
for the in-plane 2D BSP.

**InsertEdge** splits and routes edges to pos/neg children of the BSP.
**InsertTriangle** classifies coplanar triangles using the 2D BSP.

This is only needed for the triangle mesh pipeline. For SShell-based chamfer, we bypass this.

---

## 10. Key Data Structures Summary

### SCurve (half-edge concept)
```cpp
struct SCurve {
    hSCurve h;
    hSSurface surfA, surfB;  // two surfaces this curve separates
    bool isExact;
    SBezier exact;           // exact curve geometry (if isExact)
    List<SCurvePt> pts;      // piecewise-linear approximation
    enum class Source { ORIGINAL_A, ORIGINAL_B, INTERSECTION } source;
    hSCurve newH;            // handle in result shell (used during boolean)
};
```

### STrimBy (entry in a surface's trim list)
```cpp
struct STrimBy {
    hSCurve curve;
    bool backwards;     // curve direction relative to surface normal convention
    Vector start, finish;  // endpoints in xyz space
};
```

### SSurface::booleanFailed
```cpp
bool SShell::booleanFailed;  // set to true if any surface's trim polygon assembly fails
```
Checked after GenerateShellAndMesh to decide if error message shown to user.

---

## 11. Implications for Chamfer/Fillet Implementation

### For Chamfer (flat cut):
- **Avoid standard MakeFromBoolean** — use direct topology injection
- New method: `SShell::MakeFromChamferOf(SShell *src, hSCurve edge, double d1, double d2)`
- Algorithm: copy SShell, then surgically modify the two adjacent face trims and add new chamfer surface + curves
- New surfaces: 1 planar SSurface (the chamfer face) + 2 triangular caps (at edge endpoints)
- New SCurves: 2 lines (chamfer boundary curves) + 4 endpoint edges (for caps)

### For Fillet (curved blend):
- Same approach but replace planar chamfer surface with cylindrical `SSurface::FromExtrusionOf(arc)`
- The arc must be properly positioned using rolling-ball geometry
- Arc NURBS: `weight[1] = cos(dihedral/2)` for exact rational representation
- New surfaces: 1 cylindrical SSurface + 2 spherical caps (much harder than chamfer caps)
- New SCurves: 2 lines or curves

### Thread Safety:
- New MakeFromChamferOf doesn't need OpenMP — it's not iterating large parallel arrays
- The existing `MakeFromBoolean` uses OpenMP; new method should be sequential for simplicity

### Fallback to SMesh:
- If `IsForcedToMesh()` returns true for the group, need a `SMesh::MakeFromChamferOf()` too
- For MVP, chamfer/fillet groups could always use SShell (never forced to mesh)

### Error Handling:
- If `booleanFailed = true` after generation, show red error in text window
- For chamfer: validate that selected SCurve exists in source group's shell
- For fillet: validate that `dist < min(face_width)` — too large a fillet destroys the topology

---

## 12. Source File Line References

| Code | File:Line |
|------|-----------|
| `SShell::MakeFromBoolean` | `src/srf/boolean.cpp:934` |
| `SShell::MakeFromUnionOf/DifferenceOf/IntersectionOf` | `src/srf/boolean.cpp:12-23` |
| `SShell::MakeFromAssemblyOf` | `src/srf/boolean.cpp:860` |
| `CopyCurvesSplitAgainst` | `src/srf/boolean.cpp:197` |
| `MakeIntersectionCurvesAgainst` | `src/srf/boolean.cpp:905` |
| `CopySurfacesTrimAgainst` | `src/srf/boolean.cpp:846` |
| `SSurface::MakeCopyTrimAgainst` | `src/srf/boolean.cpp:395` |
| `KeepRegion` / `KeepEdge` | `src/srf/boolean.cpp:280-315` |
| `RewriteSurfaceHandlesForCurves` | `src/srf/boolean.cpp:852` |
| `SMesh::MakeFromDifferenceOf` | `src/mesh.cpp:273` |
| `SMesh::MakeFromUnionOf` | `src/mesh.cpp:258` |
| `SMesh::AddAgainstBsp` | `src/mesh.cpp:238` |
| `SBsp3::FromMesh` | `src/bsp.cpp:16` |
| `SBsp3::Insert` | `src/bsp.cpp:470` |
| `SBsp3::InsertInPlane` | `src/bsp.cpp:46` |
| `Group::GenerateForBoolean` | `src/groupmesh.cpp:182` |
| `Group::GenerateShellAndMesh` | `src/groupmesh.cpp:212` |
