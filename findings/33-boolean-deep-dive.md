# Task 33: Boolean Pipeline Deep Dive — src/srf/boolean.cpp

## Overview

`src/srf/boolean.cpp` is 1049 lines and contains the full SShell B-rep boolean pipeline. 
It implements union, difference, and intersection operations on shells of rational 
polynomial (NURBS) surfaces. Understanding it deeply is essential for knowing why the 
standard boolean pipeline **must NOT** be used for chamfer/fillet, and what patterns 
we can safely borrow.

---

## 1. Top-Level Entry Points

```cpp
// boolean.cpp:12-20
void SShell::MakeFromUnionOf(SShell *a, SShell *b)       → MakeFromBoolean(a, b, UNION)
void SShell::MakeFromDifferenceOf(SShell *a, SShell *b)  → MakeFromBoolean(a, b, DIFFERENCE)
void SShell::MakeFromIntersectionOf(SShell *a, SShell *b) → MakeFromBoolean(a, b, INTERSECTION)
```

All three are thin wrappers over `MakeFromBoolean()`.

---

## 2. MakeFromBoolean — The Core Pipeline (7 Phases)

**Location**: `boolean.cpp:983-1029`

```cpp
void SShell::MakeFromBoolean(SShell *a, SShell *b, SSurface::CombineAs type) {
    booleanFailed = false;
    
    // Phase 1: Build classifying BSPs
    a->MakeClassifyingBsps(NULL);
    b->MakeClassifyingBsps(NULL);
    
    // Phase 2: Copy curves, splitting where they cross the other shell
    a->CopyCurvesSplitAgainst(true,  b, this);
    b->CopyCurvesSplitAgainst(false, a, this);
    
    // Phase 3: Generate intersection curves
    a->MakeIntersectionCurvesAgainst(b, this);
    
    // Phase 4: Remove short segments
    for(SCurve &sc : curve) {
        sc.RemoveShortSegments(srfA, srfB);
    }
    
    // Phase 5: Cleanup temporary PWL data
    a->CleanupAfterBoolean();
    b->CleanupAfterBoolean();
    
    // Phase 6: Rebuild BSPs with split curves
    a->MakeClassifyingBsps(this);
    b->MakeClassifyingBsps(this);
    
    // Phase 7: Trim and copy surfaces
    a->CopySurfacesTrimAgainst(a, b, this, type);
    b->CopySurfacesTrimAgainst(a, b, this, type);
    
    // Phase 8: Rewrite surface handles for curves
    RewriteSurfaceHandlesForCurves(a, b);
    
    // Phase 9: Final cleanup
    a->CleanupAfterBoolean();
    b->CleanupAfterBoolean();
}
```

### Why Standard MakeFromBoolean CANNOT Be Used for Chamfer

The pipeline assumes **two geometrically separate shells** that intersect or overlap.
For chamfer, the "chamfer solid" (a triangular prism cutting the edge) shares its 
boundary faces exactly with the source solid's original faces — this creates **COINC_OPP** 
(coincident, opposite normal) situations. This causes:

1. `KeepRegion()` to return incorrect keep/discard decisions
2. `ClassifyEdge()` to fail on coincident-face edges
3. `AssemblePolygon()` to fail → `booleanFailed = true`

This was confirmed empirically by phkahler's PR #1501 which reported crashes when 
attempting to use the boolean pipeline.

---

## 3. KeepRegion and KeepEdge — The Decision Logic

**Location**: `boolean.cpp:280-327`

```cpp
static bool KeepRegion(CombineAs type, bool opA, SShell::Class shell, SShell::Class orig)
```

The `SShell::Class` enum has 4 values:
- `SURF_INSIDE` — surface is inside the other shell
- `SURF_OUTSIDE` — surface is outside the other shell  
- `SURF_COINC_SAME` — coincident, same normal direction
- `SURF_COINC_OPP` — coincident, opposite normal direction

For **DIFFERENCE** operation:
```
opA=true  (the solid being modified):  keep OUTSIDE || COINC_OPP
opA=false (the cutting tool):          keep INSIDE
```

For **UNION** operation:
```
opA=true:  keep OUTSIDE
opA=false: keep OUTSIDE || COINC_SAME
```

`KeepEdge()` returns true only if the region to one side is kept and the other is not
(i.e., the edge is a genuine boundary between kept and discarded regions).

**Key insight for chamfer**: If we tried to use DIFFERENCE with the chamfer prism as 
opB, the chamfer prism's face that is coincident with the source solid would be 
classified as `COINC_OPP` (facing opposite direction). The `KeepRegion` logic says
opA (solid) keeps `COINC_OPP` — this would leave the old face in place instead of 
replacing it with the chamfer surface. This is fundamentally the wrong behavior.

---

## 4. Surface Trimming — MakeCopyTrimAgainst

**Location**: `boolean.cpp:430-580`

This is the heart of the boolean — it computes the new trim polygon for each surface:

```cpp
SSurface SSurface::MakeCopyTrimAgainst(SShell *parent, SShell *sha, SShell *shb,
                                        SShell *into, SSurface::CombineAs type, int dbg_index)
```

**Algorithm**:
1. Start with original trim curves (updated to split versions via `newH`)
2. For DIFFERENCE on opB: flip surface normal (`ret.Reverse()`)
3. Build original trim polygon as `SEdgeList orig`
4. Find intersection curves in `into->curve` that touch this surface → `SEdgeList inter`
5. Classify each chain in `orig` using `KeepEdge()` → keep or discard
6. Classify each chain in `inter` using `KeepEdge()` → keep or discard
7. Cull extraneous edges (`CullExtraneousEdges`)
8. Build new trim from final edge list (`TrimFromEdgeList`)
9. Verify polygon assembles (`AssemblePolygon`) → on failure, set `booleanFailed = true`

**Key detail**: The `orig` edges come from the split trim curves (after 
`CopyCurvesSplitAgainst`). The `inter` edges come from intersection curves
generated by `MakeIntersectionCurvesAgainst`. This is why the boolean pipeline
needs two separate shells — it needs both original AND new intersection curves.

For chamfer's direct topology injection, we bypass this entirely: we manually compute
the chamfer curves and manually update the trim polygons.

---

## 5. CopyCurvesSplitAgainst — Curve Splitting

**Location**: `boolean.cpp:203-216`

```cpp
void SShell::CopyCurvesSplitAgainst(bool opA, SShell *agnst, SShell *into) {
    for(int i=0; i<curve.n; i++) {
        SCurve *sc = &curve[i];
        SCurve scn = sc->MakeCopySplitAgainst(agnst, NULL, ...);
        scn.source = opA ? SCurve::Source::A : SCurve::Source::B;
        hSCurve hsc = into->curve.AddAndAssignId(&scn);
        sc->newH = hsc;   // record new handle for trim rewriting
    }
}
```

**Purpose**: Before the boolean, each curve is split at every point where the other 
shell's surfaces intersect the curve. This ensures the PWL representation of curves
can be classified accurately.

**Why chamfer avoids this**: The chamfer SCurves are exactly known (line segments on 
flat faces). No intersection splitting needed — we construct them directly.

---

## 6. MakeIntersectionCurvesAgainst — Intersection Generation

**Location**: `boolean.cpp:714-727`

```cpp
void SShell::MakeIntersectionCurvesAgainst(SShell *agnst, SShell *into) {
    for(int i=0; i < surface.n; i++) {
        SSurface *sa = &surface[i];
        for(SSurface &sb : agnst->surface) {
            sa->IntersectAgainst(&sb, this, agnst, into);
        }
    }
}
```

This O(n²) surface-vs-surface intersection is the most expensive part of the boolean.
For chamfer, this step is skipped — we know analytically exactly where the chamfer 
surface meets the adjacent faces.

---

## 7. MakeFromAssemblyOf — The Simpler Alternative

**Location**: `boolean.cpp:860-930`

```cpp
void SShell::MakeFromAssemblyOf(SShell *a, SShell *b) {
```

**Algorithm**:
1. Copy all curves from `a` and `b`, assigning new IDs; record `c.newH`
2. Copy all surfaces from `a` and `b`, assigning new IDs; rewrite trim curve handles
3. Call `RewriteSurfaceHandlesForCurves(a, b)` to fix surfA/surfB in curves

**No boolean math**: This is just concatenation. If solids overlap, the result is 
self-intersecting. Used for LINKED/ASSEMBLY groups.

**Why chamfer borrows this pattern**: The `RewriteSurfaceHandlesForCurves` step shows
the correct way to maintain surfA/surfB consistency. Our direct topology injection 
must do equivalent bookkeeping.

---

## 8. TrimFromEdgeList — Building Trim Data from Edge List

**Location**: `boolean.cpp:221-265`

```cpp
void SSurface::TrimFromEdgeList(SEdgeList *el, bool asUv)
```

Takes a flat list of edges (with curve handle in auxA, backwards flag in auxB) and 
assembles them into STrimBy records by finding connected chains of same-curve edges.

**For chamfer direct injection**: This is the function we would call to rebuild the 
trim polygon of an adjacent face after chamfering. We'd build the new edge list:
- old edges (minus the ones adjacent to the chamfered region)
- new edges referencing the new chamfer boundary SCurves
Then call `TrimFromEdgeList()` to rebuild `trim`.

---

## 9. EdgeNormalsWithinSurface — Edge Classification Helper

**Location**: `boolean.cpp:360-418`

```cpp
void SSurface::EdgeNormalsWithinSurface(Point2d auv, Point2d buv, Vector *pt,
    Vector *enin, Vector *enout, Vector *surfn, uint32_t auxA,
    SShell *shell, SShell *sha, SShell *shb)
```

Given an edge UV range, computes:
- `pt` = midpoint in XYZ (refined to lie on any exact curve)
- `surfn` = surface normal at midpoint
- `enin` = inward normal of edge within surface (pointing into surface)
- `enout` = outward normal of edge within surface

**Used by**: `MakeCopyTrimAgainst` to classify each edge chain.

For chamfer development: we don't call this during geometry construction, but 
understanding inward/outward normals is key to computing chamfer offset directions.

---

## 10. FindChainAvoiding — Edge Chain Assembly

**Location**: `boolean.cpp:325-365`

```cpp
void SSurface::FindChainAvoiding(SEdgeList *src, SEdgeList *dest, SPointList *avoid)
```

Greedily assembles a connected chain of edges from `src` into `dest`, 
stopping at any point in the `avoid` list. Used to extract chains that should
all be kept or all discarded together.

**Reason for "avoid" points**: At vertices where 3+ edges meet (T-junctions,
corner points), chains must be allowed to split so each branch is classified 
independently. These are the "choosing points."

---

## 11. BSP Structures — SBspUv

**Location**: `boolean.cpp:938-980` and various

`SBspUv` is a 2D BSP tree in UV space used to classify points/edges as 
INSIDE, OUTSIDE, EDGE_PARALLEL, EDGE_ANTIPARALLEL, or EDGE_OTHER relative 
to a surface's trim boundary.

```cpp
SBspUv::Class ClassifyPoint(Point2d p, Point2d eb, SSurface *srf)
SBspUv::Class ClassifyEdge(Point2d ea, Point2d eb, SSurface *srf)
```

**For chamfer**: After we compute the new trim polygon for the chamfered adjacent faces,
we could call `MakeClassifyingBsp()` on the modified surfaces to rebuild the BSP.
This is done automatically in the next call to `MakeClassifyingBsps()` during 
subsequent group regeneration.

---

## 12. CopySurfacesTrimAgainst — Parallel Surface Processing

**Location**: `boolean.cpp:698-712`

```cpp
void SShell::CopySurfacesTrimAgainst(SShell *sha, SShell *shb, SShell *into, ...) {
    std::vector<SSurface> ssn(surface.n);
    #pragma omp parallel for
    for(int i = 0; i < surface.n; i++) {
        ssn[i] = surface[i].MakeCopyTrimAgainst(this, sha, shb, into, type, i);
    }
    for(int i = 0; i < surface.n; i++) {
        surface[i].newH = into->surface.AddAndAssignId(&ssn[i]);
    }
}
```

Uses OpenMP for parallel surface processing. Note the two-phase design:
1. Parallel phase: compute all new surfaces (no writes to shared data)
2. Serial phase: commit to `into->surface` with assigned IDs

**For chamfer thread safety**: If we ever want to parallelize chamfer generation for 
multi-edge chamfers, follow this same two-phase pattern.

---

## 13. AssemblePolygon Failure Detection

**Location**: boolean.cpp:562-572 (inside MakeCopyTrimAgainst)

```cpp
if(!final.AssemblePolygon(&poly, NULL, /*keepDir=*/true))
#pragma omp critical
{
    into->booleanFailed = true;
    dbp("failed: I=%d, avoid=%d", I+dbg_index, choosing.l.n);
    DEBUGEDGELIST(&final, &ret);
}
```

After building the final edge list, tries to assemble it into a closed polygon.
Failure means the trim polygon has gaps (open contour) — which manifests as 
the "booleanFailed" flag and black surface display.

**For chamfer watertightness**: Our direct topology injection must produce SCurve 
endpoints that exactly match (within LENGTH_EPS) the trimmed face boundaries,
or `AssemblePolygon` will fail for the modified faces.

---

## 14. Critical Finding: Why Direct Topology Injection is Superior for Chamfer

Summarizing the analysis of the boolean pipeline:

### What the boolean pipeline does (7 steps, expensive):
1. O(n²) surface-surface intersection curves
2. PWL curve splitting at intersection points
3. Multiple BSP tree constructions  
4. Per-surface trim polygon reclassification using BSP
5. Edge chain extraction and classification
6. Polygon reassembly validation

### What chamfer needs (3 steps, cheap):
1. Find the two adjacent surfaces and their shared SCurve
2. Compute 4 offset points (equal distance d along each face)
3. Directly add: 1 new SSurface (chamfer patch) + 2 new SCurves (boundaries) + update 2 existing surface trim polygons

### Why boolean would fail:
- Chamfer prism shares exact face planes with source solid → COINC_OPP classification
- COINC_OPP in DIFFERENCE → the original face gets kept (KeepRegion returns true for opA with COINC_OPP) 
- The chamfer face surface would NOT replace the trimmed original face
- Result: CRASH or incorrect geometry (phkahler PR #1501 confirmed this)

---

## 15. What MakeFromChamferOf Should Look Like

Based on the full boolean.cpp analysis, the new function `SShell::MakeFromChamferOf()` 
should follow the `MakeFromAssemblyOf` pattern but with targeted modifications:

```cpp
void SShell::MakeFromChamferOf(SShell *src, Group *g, double dist) {
    // Step 1: Full copy of source shell (like beginning of MakeFromAssemblyOf)
    // ... copy curves with newH tracking
    // ... copy surfaces with trim rewrite

    // Step 2: Find the two target faces from g->predef.entityB and entityC
    // ... lookup SSurface pointers from face handles
    // ... find shared SCurve in the copied shell

    // Step 3: Compute chamfer geometry
    // ... edge direction t = (V2-V1).WithMagnitude(1)
    // ... face normals n1, n2 from adjacent surfaces at edge midpoint
    // ... inward offset dirs w1 = t.Cross(n1), w2 = n2.Cross(t)
    // ... 4 corner points: A=V1+d*w1, B=V2+d*w1, C=V2+d*w2, D=V1+d*w2

    // Step 4: Create chamfer surface (degree 1,1)
    // ... SSurface::FromPlane(A, D-A, B-A) → new SSurface sC
    // ... assign sC.face = Remap(g->h, REMAP_CHAMFER_FACE).v

    // Step 5: Create 2 new boundary SCurves
    // ... curveA: line from A to B (on face1's trimmed boundary)
    // ... curveB: line from D to C (on face2's trimmed boundary)
    // ... each curve: surfA=newFace1Handle, surfB=newChamferHandle

    // Step 6: Update trim polygons of the two adjacent surfaces
    // ... for face1: remove STrimBy referencing old shared curve
    //                add STrimBy referencing curveA
    //                add STrimBy referencing cap curve at V1 and V2
    // ... for face2: same pattern with curveB

    // Step 7: Add chamfer surface trim polygon
    // ... curveA (forward), cap1 (line A→D), curveB (backward), cap2 (line C→B)

    // Step 8: RewriteSurfaceHandlesForCurves equivalent for new curves
}
```

---

## 16. RewriteSurfaceHandlesForCurves Pattern

**Location**: `boolean.cpp:837-845`

```cpp
void SShell::RewriteSurfaceHandlesForCurves(SShell *a, SShell *b) {
    for(SCurve &sc : curve) {
        sc.surfA = sc.GetSurfaceA(a, b)->newH,
        sc.surfB = sc.GetSurfaceB(a, b)->newH;
    }
}
```

After copying surfaces (which assigns new IDs via `AddAndAssignId`), all SCurve 
surfA/surfB handles must be updated to the new handle values. This uses the `newH`
field which was set during `CopySurfacesTrimAgainst`.

**For chamfer**: The chamfer's new SCurves must have surfA/surfB set to the NEW 
handles (post-copy), not the old handles from the source shell.

---

## 17. FindVertsOnCurve — Vertex Insertion

**Location**: `boolean.cpp:35-57`

```cpp
static void FindVertsOnCurve(List<SInter> *l, const SCurve *curve, SShell *sh)
```

For each curve in `sh`, checks if the curve's endpoints lie on `curve` (within LENGTH_EPS).
If so, adds those points to `l` for curve splitting.

**Purpose**: Handles cases where surfaces are tangent along a trim — the usual 
surface-surface intersection test doesn't generate split points there.

**For chamfer**: Irrelevant — our chamfer SCurves are newly created and don't have 
existing vertex insertion issues.

---

## 18. Summary Table: Boolean Functions and Chamfer Relevance

| Function | What it does | Chamfer relevance |
|----------|-------------|-------------------|
| `MakeFromBoolean` | Full boolean pipeline | DO NOT USE — COINC_OPP crash |
| `MakeFromDifferenceOf` | Calls MakeFromBoolean(DIFFERENCE) | DO NOT USE |
| `MakeFromAssemblyOf` | Simple concatenation, no intersection | COPY PATTERN for ID tracking |
| `MakeCopyTrimAgainst` | Trim surface against other shell | UNDERSTAND — manual equiv needed |
| `TrimFromEdgeList` | Build STrimBy from edge list | CALL DIRECTLY for modified faces |
| `CopyCurvesSplitAgainst` | Split curves at intersections | NOT NEEDED — exact curves |
| `MakeIntersectionCurvesAgainst` | O(n²) surface-surface intersection | NOT NEEDED — analytical curves |
| `KeepRegion/KeepEdge` | Boolean classification logic | UNDERSTAND ONLY — explains why direct injection needed |
| `RewriteSurfaceHandlesForCurves` | Update curve surfA/surfB after copy | MUST DO EQUIVALENT |
| `MakeClassifyingBsps` | Build UV BSPs per surface | CALLED AUTOMATICALLY on next regen |
| `AssemblePolygon` | Validate trim polygon | MUST SUCCEED — verify watertightness |
| `CleanupAfterBoolean` | Clear temporary edge lists | MUST DO EQUIVALENT |
| `FindChainAvoiding` | Assemble edge chains | NOT NEEDED for direct injection |

---

## 19. Error Handling and booleanFailed Flag

The `booleanFailed` field of `SShell` is set when `AssemblePolygon` fails.
In `groupmesh.cpp`, this is checked after boolean operations:

```cpp
// groupmesh.cpp (approximate):
if(runningShell.booleanFailed || runningMesh.isError) {
    // Show error message in text window
    g->booleanFailed = true;
}
```

**For chamfer**: If our direct topology injection produces malformed trim polygons,
`booleanFailed` will be set during the NEXT boolean operation (which calls 
`MakeClassifyingBsps` → `MakeClassifyingBsp` → `bsp = SBspUv::From(...)` which 
in turn calls `MakeEdgesInto` → `AssemblePolygon`).

To validate chamfer output, we can explicitly call:
```cpp
// After constructing chamfered shell:
SEdgeList el = {};
for(SSurface &ss : shell.surface) {
    ss.MakeEdgesInto(&shell, &el, MakeAs::XYZ);
}
SPolygon poly = {};
if(!el.AssemblePolygon(&poly, NULL, true)) {
    // chamfer failed — set booleanFailed
}
```

---

## Key Takeaways for Implementation

1. **Standard boolean pipeline CANNOT be used for chamfer** — COINC_OPP degeneracy causes crashes (confirmed by phkahler PR #1501 and code analysis)

2. **Direct topology injection IS the right approach** — we construct the exact chamfer geometry analytically and surgically update the B-rep

3. **TrimFromEdgeList is the key utility function** — call it to rebuild trim polygons of the two adjacent faces after removing the chamfered portion

4. **Watertightness invariant is critical** — every SCurve must be referenced by exactly 2 surfaces; enforce this or AssemblePolygon fails

5. **RewriteSurfaceHandlesForCurves pattern must be followed** — after copying the shell with new IDs, all new SCurves' surfA/surfB must reference the new handles

6. **AssemblePolygon is the validation oracle** — if it succeeds, the trim polygon is valid; use it for testing

7. **OpenMP parallelism not needed for MVP** — single-edge chamfer is fast enough; add parallelism only if multi-edge is implemented later
