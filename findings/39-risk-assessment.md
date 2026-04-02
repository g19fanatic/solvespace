# Risk Assessment: Chamfer/Fillet Implementation in SolveSpace

## Summary

This document provides a detailed risk assessment for implementing chamfer/fillet features
in SolveSpace. Based on 38 prior research findings and accumulated learnings from code 
analysis, community discussions (phkahler PR #1501), and analogous implementations in
FreeCAD and Blender, this assessment identifies the hardest algorithmic challenges, likely
failure modes, MVP vs. full-feature tradeoffs, and mitigation strategies.

---

## 1. Risk Matrix Overview

| Risk | Likelihood | Impact | Severity | Mitigation |
|------|-----------|--------|----------|------------|
| Trim polygon surgery fails (watertightness) | HIGH | HIGH | **CRITICAL** | Test with simple rectangular faces first; validate with AssemblePolygon |
| Vertex cap topology breaks assembly | HIGH | MEDIUM | HIGH | Skip vertex caps in MVP; document open edges |
| Face handle instability (toponaming) | HIGH | MEDIUM | HIGH | MVP limitation; use remap; document explicitly |
| Fillet arc weight computation error | MEDIUM | HIGH | HIGH | Validate with IsCylinder()/SBezier::IsCircle() |
| Shared SCurve lookup fails (faces not adjacent) | MEDIUM | MEDIUM | MEDIUM | Add user-facing error with clear message |
| Non-flat faces cause incorrect chamfer | LOW-MED | HIGH | MEDIUM | IsFlat() guard + error message |
| Chamfer overshoots edge (d > edge_length/2) | MEDIUM | LOW | LOW | Clamp d; warn user |
| STEP export breaks for new surfaces | LOW | HIGH | MEDIUM | Test immediately; RATIONAL_B_SPLINE handles it |
| Regression in existing boolean pipeline | LOW | HIGH | MEDIUM | No changes to boolean.cpp — zero regression risk |
| Fillet for non-convex edges fails | MEDIUM | MEDIUM | MEDIUM | Detect concave edge; error message |

---

## 2. Critical Risk: Trim Polygon Surgery

### Why This Is the Hardest Part

The trim polygon surgery is the central algorithmic challenge in the entire implementation.
Every finding, every source code analysis, and phkahler's own PR all converge on this
conclusion. Synthesis Part 3 (finding 38) explains the exact requirements:

**Before chamfer:**
```
surf1.trim: [..., {sharedSC, A→B direction}, ...]  
surf2.trim: [..., {sharedSC, B→A direction}, ...]
```

**After chamfer:**
```
surf1.trim: [..., {hCurve1, A→B direction}, ...]  (setback line on face 1)
surf2.trim: [..., {hCurve2, D→C direction}, ...]  (setback line on face 2)
chamfer.trim: {hCurve1, A→B} + {capEnd, B→C} + {hCurve2, C→D} + {capStart, D→A}
```

**The hidden complexity**: Adjacent trim curves on surf1 and surf2 that previously 
terminated at V1 and V2 (the original edge endpoints) must now terminate at A,B (for surf1)
or D,C (for surf2). If these neighboring trim curves are NOT updated, `AssemblePolygon()`
fails silently → the surface produces no triangles → invisible hole in the mesh.

### What Exactly Can Go Wrong

1. **Neighboring trim curve endpoint mismatch**: The trim loop must close exactly. If the
   neighboring trim curve ends at V1 but the chamfer side now starts at A (≠ V1), the loop
   is broken. `AssemblePolygon()` fails → `booleanFailed = true` silently.

2. **Backwards flag incorrectness**: If `STrimBy.backwards` is set incorrectly for the
   new curves, the surface normal points inward → the face renders inside-out → visible as
   a black patch or missing surface.

3. **SCurve surfA/surfB inconsistency**: After surgery, if `sc.surfA` refers to a surface
   that no longer has this curve in its trim list, `RewriteSurfaceHandlesForCurves()` may
   corrupt the topology.

4. **Multiple trim curves referencing the same edge**: In complex faces with curved edges,
   a single STrimBy might not represent the full shared edge. There may be multiple
   `STrimBy` entries pointing to different portions of the original edge, requiring more
   complex loop surgery.

### Mitigation Strategy

**Phase 1 (MVP)**: Only support chamfering edges where the two adjacent faces are simple
planar quadrilaterals (rectangles). In this case:
- The shared edge is one full side of each face
- The neighboring trim curves are the other sides of each face
- Updating their endpoints is a simple coordinate substitution, no geometric recomputation

**Phase 2**: Use `SBezier::PointAt(t)` to find the parameterized split point on neighboring
trim curves at the setback distance, creating new shorter SCurves for the trimmed portions.

**Validation**: After every chamfer/fillet construction, call:
```cpp
SPolygon poly;
bool ok = surf1->AssemblePolygon(&poly, NULL, /*keepEdges=*/false);
if(!ok) { g->booleanFailed = true; }
```

### Estimated Implementation Effort for Trim Surgery
- MVP (rectangular faces only): 2-3 days
- General case (arbitrary planar faces): 1-2 weeks
- Non-planar faces: 3-4 weeks (future)

---

## 3. High Risk: Vertex Cap Topology

### The Problem

At each end of the chamfered/filleted edge, there is a vertex where 3+ faces meet. The
chamfer surgery creates open boundaries at V1 and V2:
- On surf1: the trim loop ends at A (not V1 anymore)  
- On surf2: the trim loop ends at D (not V1 anymore)
- The chamfer surface trim loop has cap curves A→D and B→C, but these are boundaries —
  they need two surfaces on each side

Without vertex caps:
- Open edges at A→D and B→C
- `AssemblePolygon()` may still succeed if both A→D ends are open (no loop closure required
  at vertices in some implementations), but the mesh will have visible seams

### MVP Decision: Accept Open Edges at Vertices

For the MVP, vertex caps are explicitly **not implemented**. The justification:
- phkahler confirmed in PR #1501 that vertex caps are extremely difficult
- FreeCAD's vertex cap (spherical corner for fillet) required months of work
- The remaining geometry (chamfer/fillet face + modified adjacent faces) is still useful
- Users can see the chamfer/fillet shape even without perfect vertex topology

### Risk: AssemblePolygon Failure Cascade

If open edges at vertices cause `AssemblePolygon()` to fail for the adjacent faces, the
entire display mesh generation fails. To mitigate:
- In `MakeFromChamferOf()`, set `surf1->closedPolygon = false` if vertex cap is missing
- The triangulation should proceed anyway for each complete sub-loop in the polygon

### Vertex Cap for Chamfer (Future)

The chamfer vertex cap is a triangular planar face:
- Vertices: V1 (original edge start) → A (face1 setback) → D (face2 setback) → back to V1
- But wait: V1 is on the ORIGINAL EDGE which no longer exists in the trimmed faces
- Correction: the cap is a triangular face with vertices A, D, and the intersection of
  the edges of surf1 and surf2 that converge at the original vertex V1

This requires computing the intersection of adjacent face edges, which is non-trivial.

### Vertex Cap for Fillet (Future, Very Hard)

The fillet vertex cap is a spherical patch at the corner where 3+ fillets meet. This
requires computing a **Dupin cyclide** or a spherical patch approximation. FreeCAD's
OCC-backed implementation handles this; SolveSpace would need:
1. Find all fillet surfaces meeting at vertex
2. Compute their intersection curves (where one cylinder hits another)
3. Create a spherical/cyclide patch bounded by those intersections
4. Wire trim polygons for all surfaces meeting at the vertex

**This is beyond MVP scope.** Estimated effort: 4-8 weeks of specialized geometry work.

---

## 4. High Risk: Topological Naming (Face Handle Instability)

### The Core Problem

When the user selects two faces and creates a chamfer group, the face handles 
(`predef.entityB`, `predef.entityC`) are stored. These handles come from:
```
ss.face = Remap(e.h, REMAP_LINE_TO_FACE).v
```
where `e.h` is the handle of the entity (REQUEST) that generated the surface.

**The stability guarantee**: Face handles are stable AS LONG AS the source group's
generate sequence produces the same entities in the same order. For simple extrudes, this
is true. But if the user:
- Deletes and recreates the source extrude
- Changes the sketch that drives the extrude
- Reorders groups

...then the face handles may change, breaking the chamfer group's selection.

### Evidence From Community

From GitHub issue #1236 (Topological Naming Problem) and phkahler's comments:
> "The toponaming problem will require breaking changes to fix properly."

FreeCAD invested 2+ years on this problem (2022-2024 Toponaming project), developing a
`MappedName → IndexedName` indirection system. SolveSpace will face identical challenges.

### MVP Limitation Strategy

For MVP:
1. Store face handles as-is in `predef.entityB/C`
2. In `MakeFromChamferOf()`, validate that both faces still exist with:
   ```cpp
   SSurface *surf1 = findByFaceHandle(shell, g->predef.entityB.v);
   if(!surf1) {
       g->booleanFailed = true;
       // Error: "Chamfer face reference is no longer valid. Please recreate the chamfer."
       return;
   }
   ```
3. Document in UI: "Note: Modifying the source solid may invalidate this chamfer."
4. Use `Group::remap` mechanism for any newly created face entities

### Face Handle Lookup Implementation

```cpp
SSurface *findByFaceHandle(SShell *sh, uint32_t faceHandleV) {
    for(SSurface &ss : sh->surface) {
        if(ss.face == faceHandleV) return &ss;
    }
    return nullptr;
}
```

This lookup works within a single generation because face handles are assigned during
`GenerateShellAndMesh()` via `Remap()`, which uses the same group/request hierarchy.

---

## 5. Medium Risk: Fillet Arc Weight Computation Error

### The Math

For exact NURBS representation of the fillet circle arc:
```
arc_weight = sin(half_angle)
```
where `half_angle = acos(-n1.Dot(n2)) / 2` and `n1, n2` are the outward face normals.

### Failure Modes

1. **Normal vector direction error**: If `n1` or `n2` points INWARD (wrong sign), the
   computed `half_angle` will be `π - correct_angle`. The weight becomes negative or > 1.
   - **Detect**: `if(arc_weight <= 0 || arc_weight >= 1) → invalid geometry`
   - **Fix**: Ensure normals are computed via `SSurface::NormalAt(uv)` with a test point
     well inside the face (not at the edge where degenerate)

2. **Concave edge (reflex angle)**: If the dihedral angle is > 180° (concave/reflex edge),
   `n1.Dot(n2) > 0`, and `half_angle > π/2`. The fillet would go "outward" adding material.
   - For MVP: detect and reject: `if(n1.Dot(n2) > -0.01) → "Fillet only works on convex edges"`

3. **Nearly-parallel faces**: If `|n1.Dot(n2)| ≈ 1`, the two faces are nearly parallel
   (nearly flat angle). `setback = r/tan(half_angle)` → setback → ∞.
   - **Detect**: `if(setback > edgeLength) → "Fillet radius too large for this edge"`

4. **Degenerate arc (zero span)**: If `half_angle ≈ 0` (near-180° dihedral, barely any
   curve), the fillet degenerates to a knife edge. Not an error per se, but visually poor.
   - **Warn**: if setback is very small, note that a chamfer may be more appropriate.

### Validation Strategy

Use `SBezier::IsCircle()` at `src/srf/curve.cpp:127`:
```cpp
Vector center, axis;
double r;
if(!filletArc.IsCircle(&center, &axis, &r)) {
    // Bezier is not a valid arc → computation error
    g->booleanFailed = true;
    return;
}
```

Also validate `IsCylinder()` on the created fillet surface — if it fails, the extrusion
did not produce a proper cylinder.

---

## 6. Medium Risk: Non-Flat Face Handling

### The Problem

The MVP design requires `surf1->IsFlat()` and `surf2->IsFlat()`. SolveSpace's `SSurface`
can represent:
- Planar faces (degree 1,1 — from extrusion sides and end caps)
- Cylindrical faces (degree 2,1 — from lathe operations)
- General NURBS (from revolution/complex extrusion)

If the user selects a cylindrical face (e.g., the side of a lathe-generated cylinder),
the chamfer/fillet algorithm fails:
- The "inward" direction isn't constant along the face
- The shared SCurve between a plane and a cylinder is a general curve, not a straight line
- The setback offset along a curved face is not a straight line

### MVP Decision: Flat Faces Only

```cpp
bool SSurface::IsFlat() const {
    return (degm == 1 && degn == 1);
}
```

This correctly identifies planar faces only. Cylindrical faces (from lathe) are degree (2,1)
and will be rejected. This is the right MVP boundary.

### User Communication

```
Error: "Chamfer/Fillet MVP only supports flat (planar) faces.
        The selected face is not a flat plane.
        Please select a flat face adjacent to the edge you want to chamfer."
```

### Future: Cylindrical-to-Cylindrical Edge

When the edge between two cylindrical faces (e.g., two cylindrical surfaces from a lathe)
needs to be chamfered, the algorithm requires:
1. Computing the intersection curve of the two offset surfaces
2. This is a general surface-surface intersection problem
3. Requires `SShell::MakeFromBoolean`-level intersection computation
4. Estimated effort: 3-6 additional months

---

## 7. Lower Risks

### 7.1 Chamfer Overshoot (d > edge_length/2)

**Risk**: If the user sets `d` larger than half the edge length, the setback points A,B,C,D
overlap or go past the edge endpoints V1,V2 → invalid geometry.

**Detection**:
```cpp
double edgeLen = V2.Minus(V1).Magnitude();
if(dist >= edgeLen / 2.0) {
    // Clamp and warn:
    dist = edgeLen / 2.0 * 0.99;  // clamp to 99%
    // Or: g->booleanFailed = true; return;
}
```

**Mitigation**: Clamp `d` and/or show a warning in TextWindow. Blender calls this "Clamp Overlap."

### 7.2 STEP Export Regression

**Risk**: New chamfer surfaces (degree 1,1 bilinear) and fillet surfaces (degree 2,1 rational)
need to export correctly.

**Analysis**: `src/exportstep.cpp` exports all surfaces via:
```cpp
// For each SSurface in the SShell:
// → RATIONAL_B_SPLINE_SURFACE (handles rational and non-rational by weight list)
```

Degree (1,1) non-rational surfaces export as `B_SPLINE_SURFACE_WITH_KNOTS` or
`RATIONAL_B_SPLINE_SURFACE` with all-unity weights. This is standard and correct.

Degree (2,1) rational surfaces (fillet) export as `RATIONAL_B_SPLINE_SURFACE` with
non-unity weights. This is also standard.

**Testing**: Import the exported STEP file into FreeCAD/OpenSCAD after first implementation.

**Conclusion**: Zero regression risk if no changes are made to `exportstep.cpp`.

### 7.3 OpenMP Thread Safety in MakeFromChamferOf

**Risk**: `MakeFromBoolean` uses OpenMP parallel loops. If `MakeFromChamferOf` is called
from a threaded context, access to `SShell::surface` and `SShell::curve` (which use
`IdList`) must be thread-safe.

**Analysis**: 
- `MakeFromChamferOf()` is called from `GenerateShellAndMesh()`, which is called per-group
- Groups are processed sequentially in `GenerateAll()`
- `MakeFromBoolean` uses OpenMP within a SINGLE call, not across calls
- `MakeFromChamferOf` is called once per group, sequentially

**Conclusion**: No thread-safety risk in practice. The function is called sequentially.

### 7.4 UI Regression in ShowGroupInfo

**Risk**: Adding CHAMFER/FILLET branches to the `if-chain` in `textscreens.cpp:394` could
accidentally break existing branches if the chain logic is modified incorrectly.

**Mitigation**: Add new branches as `else if(g->type == Type::CHAMFER)` and
`else if(g->type == Type::FILLET)` — purely additive, cannot break existing branches.
Run existing test cases with each existing group type to confirm no regression.

---

## 8. What External Tools Could Help

### For Geometry Validation
- **GeoGebra / Desmos**: Visualize 2D cross-sections of the chamfer/fillet geometry to
  validate the setback calculations and arc weights before coding.

### For B-rep Debugging
- **SolveSpace's own `--debug-export` mode**: Export the chamfered solid as STEP and
  inspect in FreeCAD/OpenSCAD to validate B-rep topology.
- **FreeCAD's "Check Geometry" tool**: Can detect open edges, non-manifold surfaces,
  and other B-rep invalidity after STEP import.

### For Reference Implementation Comparison
- **phkahler's `fillet` branch** (PR #1501): Contains 112 lines of `src/srf/chamfer.cpp`
  with `GetEdgeModifier()` + `ModifyEdges()`. Even though it crashes, the data structure
  choices and the `fillet_info` struct are valuable reference.
  URL: https://github.com/phkahler/solvespace/tree/fillet

### For Testing
- **Simple test models**: Create hand-crafted `.slvs` files with a simple extruded
  rectangle, then manually invoke chamfer generation to validate the B-rep before
  the UI is wired up.

---

## 9. MVP vs. Full-Feature Tradeoffs

### MVP Scope (Recommended Starting Point)

**What MVP implements:**
- 3D chamfer only (no fillet in v1)
- Flat (planar) faces only
- Single edge (one face pair selection per group)
- Equal-leg chamfer (valA=d, valB=0 → both setbacks equal)
- No vertex caps (open edges at V1, V2 endpoints)
- Fixed edge orientation (edge must be fully shared between the two faces)
- Menu-driven only (no toolbar icon)

**What MVP explicitly excludes:**
- Fillet (rounded blend) — implement after chamfer works
- Non-flat (cylindrical, arbitrary NURBS) faces
- Multiple simultaneous edges
- Unequal-leg chamfer (d1 ≠ d2)
- Vertex caps (plugging the holes at edge endpoints)
- Variable radius fillet
- 2D sketch fillet (parametric) — separate feature
- Chamfer angle-and-distance specification (not equal-leg)

**Why this MVP scope is correct:**
- phkahler's own advice: "limit myself to flat surfaces only just to get the basics down"
- The trim polygon surgery is hard enough for rectangles without adding curved face complexity
- The architecture is completely in place — extending is incremental once MVP works
- Chamfer before fillet: chamfer surface is planar (simpler math, all-unity weights)
- Single edge before multiple: avoids all vertex junction complexity (90% of Blender's
  7000-line bevel code deals with vertex junctions and multi-edge interactions)

**Estimated MVP effort:**
- `src/sketch.h` additions: 30 mins
- `src/ui.h` additions: 30 mins
- `src/graphicswin.cpp` menu additions: 1 hour
- `src/textscreens.cpp` UI additions: 2 hours
- `src/group.cpp` Generate() + MenuGroup(): 3 hours
- `src/groupmesh.cpp` IsMeshGroup() + GenerateShellAndMesh(): 3 hours
- `src/srf/chamfer.cpp` MakeFromChamferOf() (flat faces only, no vertex caps): 1-2 weeks
- Testing and debugging: 1-2 weeks
- **Total: approximately 3-5 weeks for a working MVP chamfer**

### Full Feature Scope (Long-Term Roadmap)

**Phase 2 (After MVP chamfer works):**
- Fillet with cylindrical surface generation
- Vertex cap for chamfer (triangular planar face)
- Unequal-leg chamfer (valA=d1, valB=d2)
- Multiple edge selection (select 3+ faces for multi-edge chamfer)

**Phase 3 (Months of work):**
- Non-flat faces (cylindrical-to-cylindrical, cylinder-to-plane edges)
- Vertex cap for fillet (spherical/cyclide patch — very hard)
- Variable radius fillet
- Auto-chamfer all sharp edges (angle threshold mode)
- 2D sketch fillet (parametric Group)

**Phase 4 (Stable edge references):**
- Robust toponaming (stable edge references across sketch modifications)
- Multi-edge selection with edge deduplication
- Chamfer/fillet of imported STEP solids

---

## 10. Architecture Decision Risks

### Decision: Direct Topology Injection vs. Boolean Pipeline

**Risk if wrong**: If the boolean pipeline COULD be made to work, using direct topology
injection creates a maintenance burden (two code paths). If direct injection misses edge
cases the boolean handles, it could produce subtly wrong geometry.

**Evidence supporting the decision**:
1. phkahler's PR #1501 crash was due to COINC_OPP degeneracy from standard boolean
2. Finding 16 (boolean deep dive) shows DIFFERENCE pipeline fundamentally doesn't model
   the chamfer operation (keeps coincident faces wrong)
3. Finding 33 (boolean deep dive) confirms: `MakeFromAssemblyOf()` (simple concatenation)
   is the closest model, but even that doesn't handle the trim surgery correctly
4. Blender's bevel completely bypasses any boolean — directly manipulates mesh topology
5. FreeCAD bypasses this via OCC which handles it internally, but their raw CGAL
   chamfer confirmation shows direct manipulation is standard

**Conclusion**: Decision is well-supported. Risk of reverting to boolean is LOW.

### Decision: Face-Pair Selection vs. Edge Selection

**Risk if wrong**: If face-pair selection proves unusable (e.g., selecting coplanar faces
that share no common edge), users may find it confusing.

**Evidence supporting the decision**:
- Finding 23 confirmed: no first-class edge entities exist in SolveSpace
- Adding edge entities requires changes to 6+ files and adds significant complexity
- Face selection infrastructure already works; no changes needed
- User guidance ("select 2 adjacent faces") is clear; validation in MenuGroup() catches
  non-adjacent face selection

**Conclusion**: Decision is correct. Risk is LOW.

### Decision: Single Solver Param, No Equations

**Risk if wrong**: If the DOF calculation is wrong, the constraint solver might report
infeasibility or incorrect DOF counts to users.

**Evidence supporting the decision**:
- Finding 35 (solver vs. stored params) showed REVOLVE angle uses identical pattern
- CHAMFER gets 1 DOF — semantically correct (the distance IS the free DOF)
- No changes to system.cpp needed
- `suppressDofCalculation` is available if DOF reporting is distracting, but 1 DOF is fine

**Conclusion**: Decision is well-founded. Risk is LOW.

---

## 11. Risk Prioritization for Implementation Order

### Step 1 (Highest Priority — Validate Core Concept)
Write `MakeFromChamferOf()` as a standalone test function that takes a hard-coded
simple SShell (a box/cube), applies a chamfer, and prints the resulting B-rep for
verification. Do this BEFORE wiring up any UI. This validates:
- Face lookup by handle
- Shared SCurve identification
- Offset direction computation
- Chamfer surface creation (FromPlane)
- SCurve creation
- Trim polygon surgery on a known simple case

**Risk**: This test reveals if the trim polygon surgery is correct. If not, fix it before
proceeding to UI wiring.

### Step 2 (High Priority — Infrastructure)
Wire up the UI stubs (menu, GroupType enum, GenerateShellAndMesh branching, IsMeshGroup)
WITHOUT the geometry. The group can be created but GenerateShellAndMesh returns early with
`booleanFailed = true`. This validates:
- The group creation flow
- File save/load round-trip (valA, opA, predef.entityB/C survive)
- TextWindow UI display
- Face selection and handle storage

### Step 3 (Core — Full Chamfer)
Connect the geometry from Step 1 to the infrastructure from Step 2. This should "just
work" if Steps 1 and 2 are both correct.

### Step 4 (Lower Priority — After Chamfer Works)
Implement fillet using the same infrastructure. The fillet adds:
- Rational arc cross-section computation
- FromExtrusionOf instead of FromPlane
- Setback calculation using arc weight

### Step 5 (Future — After Fillet Works)
Address vertex caps, multi-edge, and non-flat faces incrementally.

---

## 12. Known Unknowns

These are risks that cannot be fully assessed without actually writing the code:

1. **Exact `AssemblePolygon()` failure modes**: The exact conditions under which the trim
   polygon assembly fails for partial trims (open edges at vertex caps) need empirical testing.

2. **Normals at setback points for non-trivial faces**: Even for flat faces, the normal
   direction returned by `SSurface::NormalAt(uv)` must be tested to confirm it points
   outward (not inward) for all face orientations that arise in practice.

3. **`MakeFromCopyOf()` vs. working on the live shell**: The plan is to `MakeFromCopyOf()`
   and then modify the copy. It's unclear whether `FindById()` on the copied shell's
   surfaces/curves behaves correctly after modification (the IdList preserves original handles).

4. **The trim curve `start`/`finish` fields**: Are these cosmetic (just for display) or are
   they actually used by `AssemblePolygon()`? If they're cosmetic, the surgery only needs
   to update `STrimBy.curve`. If they're used for ordering, precise endpoint coordinates
   are critical.

5. **Handling of edges where the adjacent face is a "cap" from extrusion**: When an extrusion
   creates top and bottom cap faces, these are internally `SSurface::FromPlane(...)` 
   surfaces. The trim polygons on these caps may have 4 sides (for a rectangular extrusion)
   or N sides (for an N-sided profile). The surgery for a non-rectangular face (e.g.,
   chamfering an edge of a triangular prism) needs more complex neighboring trim update.

---

## 13. Summary

| Priority | Risk Area | Mitigation |
|----------|-----------|------------|
| 1 | Trim polygon surgery correctness | Test with hard-coded simple box first; validate watertightness |
| 2 | Vertex cap open edges | Accept as MVP limitation; document clearly |
| 3 | Topological naming instability | Store face handles; validate on regen; document limitation |
| 4 | Fillet arc weight errors | Validate with IsCircle()/IsCylinder(); guard division by tan(0) |
| 5 | Non-flat face rejection | IsFlat() guard + clear user error message |
| 6 | Chamfer overshoot | Clamp dist < edgeLength/2 |
| 7 | STEP export regression | Zero risk — no changes to exportstep.cpp |
| 8 | Boolean pipeline regression | Zero risk — no changes to boolean.cpp |

**Bottom line**: The chamfer/fillet implementation is feasible and well-architected.
The primary risk is the trim polygon surgery, which requires careful testing with simple
cases before scaling to complex geometry. The MVP scope (flat faces, single edge, equal-leg,
chamfer only) is the right starting point. Vertex caps and fillet can be added incrementally
once the core trim surgery is proven.

**Estimated total effort for MVP chamfer**: 3-5 weeks
**Estimated total effort for MVP chamfer + fillet**: 6-10 weeks
**Estimated total effort for full feature (multi-edge, vertex caps, non-flat)**: 4-6 months
