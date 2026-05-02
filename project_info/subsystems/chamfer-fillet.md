# Chamfer & Fillet Subsystem

> **Purpose**: Deep-dive reference for the CHAMFER (5400) and FILLET (5401) group types —
> solid-modeling operations that bevel or round an edge between two selected flat faces
> via direct topology injection. **Status**: Feature complete with extensive test coverage.
>
> **Cross-references**: [architecture.md](../architecture.md) | [sketch.md](sketch.md) |
> [../code-patterns.md](../code-patterns.md) | [../context-strategy.md](../context-strategy.md)
>
> **End-cap deep-dive**: [chamfer-fillet-endcap-geometry.md](chamfer-fillet-endcap-geometry.md) — exhaustive documentation of end-cap math: cap scoring, fillet arc construction (rational quadratic Bézier, `arc_weight = sin(half_angle)`), cap trim updates, RECON path, corner post-processing, CF case, arc copy approach, and 10 design decisions with rationale.

---

## Overview

Chamfer and fillet are implemented as two new `Group::Type` values that produce a modified
copy of the source solid shell without going through SolveSpace's traditional boolean
union/difference pipeline. Instead they use **direct topology injection**: the source
shell is deep-copied and then surgically modified so that the shared edge between two
user-selected faces is replaced by a new flat (chamfer) or arc (fillet) surface.

**Source file**: `src/srf/chamfer.cpp` (1636 lines — expanded with new repair helpers, DIFF/ASSEMBLE fixes)  
**Core methods**: `SShell::MakeFromChamferOf()`, `SShell::MakeFromFilletOf()`

---

## Architecture: Direct Topology Injection

The key design decision is that chamfer/fillet groups do NOT call
`SShell::MakeFromBooleanOf()`. Instead:

1. `MakeFromCopyOf(src)` — deep-copy the entire source shell (all surfaces + curves)
2. Find two surfaces by face-entity handle (entityB, entityC)
3. Find the shared SCurve between them
4. Build a new chamfer/fillet surface and its boundary curves
5. Splice those curves into the trim loops of the two adjacent faces
6. Update cap surfaces (top/bottom faces at the endpoints of the shared edge)
7. Remove the old shared curve

The result is `thisShell` — a fully self-contained modified shell that becomes
`runningShell` directly (see ASSEMBLE-skip fix below).

**Advantage**: Avoids the complexity of full boolean intersection; works reliably for
flat (planar) faces.  
**Constraint**: Both selected faces must be planar and non-degenerate; the chamfer
distance must be less than half the edge length.

---

## ASSEMBLE-Skip Fix (`src/groupmesh.cpp:419-435`)

After `MakeFromChamferOf()` produces a complete modified shell in `thisShell`, the
generic combination code in `GenerateShellAndMesh()` would normally call
`MakeFromAssemblyOf(prevShell, thisShell)` — concatenating the source shell with
`thisShell`. Since `thisShell` is already a copy of the source shell, this would
**double all geometry**, causing chained chamfers to fail (duplicate face handles
corrupt the second chamfer's face lookup).

The fix at `src/groupmesh.cpp:419`:

```cpp
if(type == Type::CHAMFER || type == Type::FILLET) {
    if(!IsForcedToMesh()) {
        runningShell.MakeFromCopyOf(&thisShell);
        booleanFailed = thisShell.booleanFailed;
    } else {
        runningMesh.MakeFromCopyOf(&thisMesh);
        thisShell.TriangulateInto(&runningMesh);
    }
    if(booleanFailed != prevBooleanFailed) SS.ScheduleShowTW();
    displayDirty = true;
    return;
}
```

This early-return skips `GenerateForBoolean(ASSEMBLE)` entirely and uses `thisShell`
directly as `runningShell`. This is what enables **chaining**: multiple chamfers/fillets
on the same face work correctly because each subsequent operation finds exactly one copy
of each face handle.

---

## MakeFromChamferOf — 15-Step Algorithm

**Signature** (`src/srf/surface.h:435`):
```cpp
void SShell::MakeFromChamferOf(SShell *src, Group *g, double dist);
```

**Steps** (in `src/srf/chamfer.cpp`, starting around line 83):

| Step | What | Key Code Location |
|------|------|-------------------|
| 1 | `MakeFromCopyOf(src)` — deep-copy entire source shell | `chamfer.cpp:~100` |
| 2 | Find hSurf1 (entityB face) and hSurf2 (entityC face) by `ss.face == entityB.v` | `chamfer.cpp:~110` |
| 3 | Check both surfaces are planar (`DepartureFromCoplanar`) | `chamfer.cpp:~130` |
| 4 | Find shared SCurve: `sc.surfA == hSurf1 && sc.surfB == hSurf2` (or swapped) | `chamfer.cpp:~150` |
| 5 | Extract endpoints V1, V2 from shared curve; check `dist < edgeLen / 2.0` | `chamfer.cpp:~170` |
| 6 | Compute face normals n1, n2 by cross-product of ctrl points | `chamfer.cpp:~190` |
| 7 | Compute inward offset directions d1, d2; validate using centroid check | `chamfer.cpp:~210` |
| 8 | Compute chamfer corners: A=V1+d1·dist, B=V2+d1·dist, C=V2+d2·dist, D=V1+d2·dist | `chamfer.cpp:~240` |
| 9 | Create chamfer SSurface via `SSurface::FromPlane(A, B-A, D-A)` with remap face entity | `chamfer.cpp:~260` |
| 10 | Cap surface detection: scan neighbor curves at V1/V2 to find hCapSurf1 (at V1) and hCapSurf2 (at V2); create new cap curves hCapV1 (A→D) and hCapV2 (B→C) | `chamfer.cpp:286-350` |
| 11 | Build chamfer surface trim polygon (CW winding): A→D (hCapV1 fwd), D→C (hCurve2 fwd), C→B (hCapV2 bkw), B→A (hCurve1 bkw) | `chamfer.cpp:~352` |
| 12 | Update surf1 trims: replace hShared with hCurve1; update neighbor endpoints V1→A, V2→B using `InsertPointIntoCurvePts` | `chamfer.cpp:~380` |
| 13 | Update surf2 trims: replace hShared with hCurve2; update neighbor endpoints V1→D, V2→C | `chamfer.cpp:~410` |
| 14 | Remove old shared SCurve via tag+`RemoveTagged()` | `chamfer.cpp:~440` |
| 15 | Update cap surface trims at V1 (hCapSurf1) and V2 (hCapSurf2): patch neighbor endpoints and add new cap curve to trim polygon | `chamfer.cpp:449-510` |
| Post-15 | `BridgeTrimGapIfOpen` sweep: for all modified/neighbor surfaces, close any residual open trim loops via graph traversal | `chamfer.cpp:795-818` |
| 16 | Handle surfaces with intermediate vertices on the shared edge V1V2: detect trims whose endpoints fall on V1V2 but were missed by standard endpoint updates; insert new curve segments so every affected surface has a complete trim loop | `chamfer.cpp:1433` |

### Step 15 Detail: Cap Surface Trim Update

The cap surfaces (e.g., top and bottom faces of a box) are adjacent to the shared edge
at its endpoints V1 and V2. Their trim loops must be updated to:
- Replace neighbor curve endpoints that touched V1 with A/D (surf1/surf2 side)
- Replace neighbor curve endpoints that touched V2 with B/C
- Add the new cap curve (A→D or B→C) as an extra trim entry

The `bordersSurf1 / bordersSurf2` check determines which setback point to use:

```cpp
for(STrimBy &stb_c : capSurf1->trim) {
    SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
    bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
    bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
    if(stb_c.start.Equals(V1)) {
        if(bordersSurf1) { InsertPointIntoCurvePts(nc, A); stb_c.start = A; }
        else if(bordersSurf2) { InsertPointIntoCurvePts(nc, D); stb_c.start = D; }
    }
    if(stb_c.finish.Equals(V1)) {
        if(bordersSurf1) { InsertPointIntoCurvePts(nc, A); stb_c.finish = A; }
        else if(bordersSurf2) { InsertPointIntoCurvePts(nc, D); stb_c.finish = D; }
    }
}
```

The `backwards` flag for the new cap curve (`EntireCurve(this, hCapV1, backwards)`)
is detected dynamically by checking which cap trim entries have their `finish` at V1
(if finish == V1 → backwards=true, meaning the new curve runs from A to D backwards).

**Key locations** (updated for 1636-line file):
- Chamfer `InsertTrimAt` for capSurf1: `chamfer.cpp:739`
- Chamfer `InsertTrimAt` for capSurf2: `chamfer.cpp:790`
- Fillet `InsertTrimAt` for capSurf1: `chamfer.cpp:1270`
- Fillet `InsertTrimAt` for capSurf2: `chamfer.cpp:~1320`
- Post-loop `BridgeTrimGapIfOpen` sweep: `chamfer.cpp:795-818` (chamfer), `chamfer.cpp:~1340-1365` (fillet)

---

## MakeFromFilletOf — Algorithm Summary

**Signature** (`src/srf/surface.h:436`):
```cpp
void SShell::MakeFromFilletOf(SShell *src, Group *g, double r);
```

Same 15-step flow as chamfer but:

- **Step 8**: Computes tangent points on each face at distance `r` from the edge, plus
  the arc center — corners become A0/B0 (on surf1) and A1/B1 (on surf2)
- **Step 9**: Creates the fillet surface as an `SSurface` with degU=2 (quadratic NURBS
  arc in U direction); uses `SSurface::FromRevolutionOf()` or similar arc construction
- **Step 11**: Trim polygon uses arc boundary curves instead of straight lines
- **Step 15**: Cap updates use A0/B0 for capSurf1 and A1/B1 for capSurf2

**Key locations** (fillet section starts at line 852 in chamfer.cpp):
- `SShell::MakeFromFilletOf` entry: `chamfer.cpp:852`
- Fillet corner computation: `chamfer.cpp:~900-1000`
- Fillet surface creation: `chamfer.cpp:~1010`
- **Cap candidate scoring** (Step 10 fillet): cap surface candidates now selected by
  `|normal.Dot(t)|` — the candidate whose face normal has the largest projection onto
  the edge tangent is chosen, replacing the old first-match heuristic — `chamfer.cpp:~1062`
- Fillet Step 15 (cap updates): `chamfer.cpp:~1240-1330`
- **RECON path** (trim.n==0 for DIFF internal surface endcaps): when a surface has no
  existing trim entries at this step (`trim.n==0`), reconstruct the endcap boundary via
  graph traversal of adjacent curves to build a closed trim loop — `chamfer.cpp:~1388`
- **ASSEMBLE stitch**: for third-surface trim edges ending at V1, insert the new fillet
  curve segment to close the loop — part of the DIFF/ASSEMBLE fix
- **Step 16**: handle surfaces with intermediate vertices on V1V2 — `chamfer.cpp:1433`
- **WIP note**: 2 debug `fprintf` statements remain (`RECON:` at ~line 1388,
  `RECON_DONE:` at ~line 1393) — **stripped in cleanup commit** (see Debug Instrumentation section)

---

## Key Data Structures

### Group fields used by CHAMFER/FILLET (`src/sketch.h:186-191, 313-319`)

```cpp
// Group::Type enum:
CHAMFER = 5400,
FILLET  = 5401

// Remap constants:
REMAP_CHAMFER_FACE = 1011,  // generates chamfer face entity
REMAP_FILLET_FACE  = 1012,  // generates fillet face entity
```

**`predef.entityB`** — face handle for surf1 (first selected face)  
**`predef.entityC`** — face handle for surf2 (second selected face)  
**`h.param(0)`** — chamfer distance / fillet radius (set via text window edit)

### SShell / SSurface / SCurve (relevant fields)

- `SSurface::face` — entity handle identifying which group/face this surface belongs to
- `SSurface::trim` — `List<STrimBy>` — the ordered boundary curve loop
- `STrimBy::curve` — `hSCurve` handle of the boundary curve
- `STrimBy::start` / `STrimBy::finish` — 3D endpoints of the boundary segment
- `STrimBy::backwards` — if true, curve traversed in reverse
- `SCurve::surfA`, `SCurve::surfB` — the two surfaces this edge borders
- `SCurve::pts` — `List<SCurvePt>` piecewise-linear approximation
- `SCurve::tag` — used for tag-based deletion (Step 14)

---

## Helper Functions

### `InsertPointIntoCurvePts(SCurve *sc, Vector P)` — `chamfer.cpp:18`
**Location**: `src/srf/chamfer.cpp:19-63`

Inserts a new vertex point P into a curve's `pts` list at the segment that geometrically
contains P. Used to split a curve when a chamfer setback point lies on it (Steps 12, 13, 15).

Returns `true` if P was inserted or already present. No-op if P doesn't lie on any segment.
**Important**: Callers must check the return value — if `false`, the subsequent
`TruncateCurveAtVertex` and `UpdateAllSurfaceTrimEndpoints` calls must be skipped.

### `TruncateCurveAtVertex(SCurve *sc, Vector oldEnd, Vector newEnd)` — `chamfer.cpp:71`
**Location**: `src/srf/chamfer.cpp:66-79`

Removes all pts from sc that are beyond `oldEnd` (i.e., past the new setback point `newEnd`).
Called immediately after `InsertPointIntoCurvePts` to trim the stale original corner from
the curve's point list. Precondition: `newEnd` is already in `sc->pts`.

### `AddLinearCurve(SShell *shell, Vector from, Vector to, hSSurface surfA, hSSurface surfB)` — `chamfer.cpp:99`
**Location**: `src/srf/chamfer.cpp:99-112`

Creates a new straight-line `SCurve` between two points, assigns `surfA`/`surfB`, populates
`pts` via `SBezier::MakePwlInto()`, and inserts it into the shell with a new handle.
Used to create hCurve1, hCurve2, hCapV1, hCapV2 in Steps 10+.

---

## File References

| Location | What |
|----------|------|
| `src/srf/chamfer.cpp:1-79` | File header + `InsertPointIntoCurvePts` + `AddLinearCurve` |
| `src/srf/chamfer.cpp:83-560` | `SShell::MakeFromChamferOf()` — full 15-step algorithm |
| `src/srf/chamfer.cpp:286-350` | Chamfer cap surface detection (Step 10) |
| `src/srf/chamfer.cpp:449-510` | Chamfer Step 15: cap surface trim update |
| `src/srf/chamfer.cpp:546,574` | Dynamic `backwards` flag for chamfer cap curves |
| `src/srf/chamfer.cpp:852-1636` | `SShell::MakeFromFilletOf()` — full algorithm (now includes Step 16 at 1433) |
| `src/srf/chamfer.cpp:~1062` | Fillet cap surface detection (Step 10) + cap candidate scoring by `\|normal.Dot(t)\|` |
| `src/srf/chamfer.cpp:~1270` | Fillet Step 15: cap surface trim update |
| `src/srf/chamfer.cpp:1433` | Step 16: handle surfaces with intermediate vertices on shared edge V1V2 |
| `src/srf/surface.h:432-436` | `MakeFromChamferOf` / `MakeFromFilletOf` declarations; `int tag` on `SCurve` |
| `src/sketch.h:186-191` | `Group::Type::CHAMFER=5400`, `FILLET=5401` |
| `src/sketch.h:313-319` | `REMAP_CHAMFER_FACE=1011`, `REMAP_FILLET_FACE=1012` |
| `src/groupmesh.cpp:392-435` | `GenerateShellAndMesh` CHAMFER/FILLET branches + ASSEMBLE-skip |
| `src/groupmesh.cpp:583-597` | `RunningMeshGroup()` + `IsMeshGroup()` for CHAMFER/FILLET |
| `src/group.cpp:324-362` | `MenuGroup()` — command handlers for GROUP_CHAMFER, GROUP_FILLET |
| `src/ui.h:157-161` | `Command::GROUP_CHAMFER`, `Command::GROUP_FILLET` |
| `src/ui.h:372-378` | `Edit::CHAMFER_OFFSET=803`, `Edit::FILLET_RADIUS=804` |
| `src/ui.h:512-513` | `Group::Type` forward-declared for UI |
| `src/textscreens.cpp:488-517` | `ShowGroupInfo()` for CHAMFER: shows source group + offset edit link |
| `src/textscreens.cpp:948-985` | `EditControl` handler: `CHAMFER_OFFSET` → updates `h.param(0)` |
| `src/srf/boolean.cpp:750-752` | Defensive `surfA.v != 0 && surfB.v != 0` guard |
| `src/draw.cpp:28,45,55` | `FindByIdNoOops` guards in `Selection::Draw` / `HasEndpoints` |
| `test/group/chamfer/test.cpp` | **77 programmatic tests** covering basic, mesh, chaining, stale-vertex, backface, origin-line regression, DIFF/ASSEMBLE geometry (14 tests added in 96df040d) |
| `test/group/chamfer/chaining_test.slvs` | `.slvs` file with fillet + 2 chamfers on same face |
| `test/CMakeLists.txt:68` | Test source entry |
| `src/CMakeLists.txt` | `srf/chamfer.cpp` added to source list |

---

## Chaining Support

Multiple chamfers/fillets can be applied to the same face in sequence. For example:

```
Group 3: extrude (box)
Group 4: fillet  entityB=right-face  entityC=top-face  radius=1.0
Group 5: chamfer entityB=left-face   entityC=back-face  dist=1.0
Group 6: chamfer entityB=left-face   entityC=front-face dist=1.0
```

Group 5 and 6 both reference `left-face` (same entityB). This works because:

1. Each group's `runningShell` is set directly from its own `thisShell` (ASSEMBLE-skip)
2. `thisShell` is always a copy of `src->runningShell` (the previous group's output)
3. Each copy has exactly ONE instance of each face handle — no duplicates
4. The second chamfer (Group 6) finds the left face cleanly in Group 5's output

Without the ASSEMBLE-skip fix, Group 5's `runningShell` would be an assembly of
`extrude.runningShell` + `chamfer.thisShell` = **2× surfaces**, causing Group 6 to
match face handles from different sub-shells with no shared curves between them.

---

## Test Coverage Summary

`test/group/chamfer/test.cpp` — **77 TEST_CASEs**, 0 failures as of 96df040d.

### Helper functions (top of test.cpp)

| Function | Line | Purpose |
|----------|------|---------|
| `CreateBoxExtrude()` | `28` | Creates a standard 20×20×20 box via extrude, returns `hGroup` |
| `FindTwoAdjacentFaces(hGroup, ...)` | `109` | Finds two adjacent planar faces sharing an edge |
| `FindCapFace(hGroup, bool wantTop)` | `128` | Finds top or bottom cap face of extrude group |
| `AddChamferGroup(hGroup, ...)` | `146` | Adds a CHAMFER group to the sketch, runs GenerateAll |
| `AddFilletGroup(hGroup, ...)` | `172` | Adds a FILLET group to the sketch, runs GenerateAll |
| `CountLineSegmentsInGroup(hGroup, bool visibleOnly)` | `1695` | Counts LINE_SEGMENT entities in a group |
| `CountLineSegmentsWithOriginEndpoint(bool skipHidden)` | `1707` | Counts lines with an endpoint at (0,0,0) — origin-line bug detector |
| `FindPointNear(Vector, List<Vector>)` | `1588` | Returns true if any point in a list is within tolerance of the query point; used by DIFF/ASSEMBLE tests |
| `CreateBoxWithCutout(double offset)` | `1600` | Creates a box with a rectangular cutout using DIFF operation; used by fillet_diff_* tests |
| `CreateBoxWithCutoutFromBottom(double offset)` | `1684` | Creates a box with a cutout from the bottom face; alternate DIFF geometry for no-crash tests |

### Test categories

| Category | Test names (prefix) | Count |
|----------|---------------------|-------|
| Basic crash / boolean / mesh | `chamfer_basic_*`, `fillet_basic_*` | 6 |
| Geometry quality | `chamfer_no_self_intersection`, `chamfer_shell_has_surfaces`, etc. | 6 |
| Edge cases | `chamfer_edge_case_small_dist`, `chamfer_edge_case_large_dist` | 2 |
| Chaining | `chamfer_chaining_no_boolean_fail` | 1 |
| Face order invariance | `chamfer_face_order_forward/invariant`, `fillet_face_order_*` | 4 |
| Stale vertex / setback | `chamfer_no_stale_vertices`, `chamfer_setback_*`, `fillet_setback_*` | 8 |
| Curve/surface wiring | `chamfer_new_curves_have_correct_surface_associations` | 1 |
| Count checks | `chamfer_curve_count_correct`, `fillet_curve_count_correct` | 2 |
| Face entity / hide | `chamfer_group_has_face_entity`, `chamfer_original_vertices_hidden`, etc. | 10 |
| Edge/entity existence | `chamfer_edges_should_exist_as_entities`, `chamfer_has_line_segment_entities`, etc. | 6 |
| Backface regression | `chamfer_side_top_cap_no_backface`, `fillet_side_top_cap_no_backface`, `chamfer_face1_face2_cap_no_backface`, `fillet_face1_face2_cap_no_backface`, `chamfer_adjacent_cap_no_backface`, `chamfer_adjacent_vertical_no_backface` | 6 |
| Cap edge hide | `chamfer_cap_edges_hidden`, `fillet_cap_edges_hidden`, `fillet_original_edge_hidden` | 3 |
| Chained origin-line regressions | `chamfer_chained_no_null_point_handles`, `chamfer_chained_no_origin_line`, `chamfer_chained_no_null_endpoint_handles`, `chamfer_second_visible_line_count`, `chamfer_line_endpoints_valid`, `chamfer_chained_total_visible_lines`, `chamfer_chained_offset_from_origin`, `chamfer_three_chained` | 8 |
| Fillet chaining | `fillet_chained_no_origin_line`, `fillet_chained_no_null_endpoints` | 2 |
| Offset / size edge cases | `chamfer_minimal_offset_no_extra_line`, `chamfer_large_offset_no_extra_line` | 2 |
| Diagnostic / mixed | `chamfer_chained_diagnostic_no_visible_origin_line`, `chamfer_two_side_faces_no_extra_line`, `chamfer_then_fillet_no_extra_line`, `fillet_then_chamfer_no_extra_line` | 4 |
| DIFF geometry no-crash | `fillet_diff_inside_faces_no_crash`, `fillet_diff_bottom_cutout_no_crash`, and 10 more `fillet_diff_*` via `CreateBoxWithCutout`/`CreateBoxWithCutoutFromBottom` | 12 |
| DIFF endcap geometry | `fillet_diff_endcap_has_triangles` — asserts that DIFF fillet endcaps produce triangles (non-empty mesh) | 1 |
| ASSEMBLE backface | `fillet_assemble_cap_no_backface` — asserts no backfacing cap triangles on ASSEMBLE fillet geometry | 1 |

### Origin-line regression (chained chamfer bug)

A key bug class discovered during development: chained chamfers could produce a spurious
`LINE_SEGMENT` entity with one endpoint at `(0,0,0)` (the origin). This was caused by
null/uninitialized `hEntity` handles on LINE_SEGMENT point entities in the chamfer group.

**Detection**: `CountLineSegmentsWithOriginEndpoint(skipHidden=true)` > 0  
**Regression tests**: `chamfer_chained_no_origin_line` (line 1775), `chamfer_chained_diagnostic_no_visible_origin_line` (line 2391), and 6+ more.

### Backface regression

Cap surfaces (top/bottom faces at edge endpoints) could produce triangles facing inward
(backfacing) when the trim polygon had an open gap. Fixed by `BridgeTrimGapIfOpen` + `InsertTrimAt`.

**Detection**: `normal.Dot(centroid - boxCenter) < -0.01` (camera-independent backface check)  
**Regression tests**: `chamfer_side_top_cap_no_backface` (line 1498), `chamfer_face1_face2_cap_no_backface` (line 2601), `chamfer_adjacent_cap_no_backface` (line 2728), `chamfer_adjacent_vertical_no_backface` (line 2794).

---

## Debug Instrumentation

During development, 190+ `CHAMFER_DEBUG` logging lines were added across 7 files:
`chamfer.cpp`, `boolean.cpp`, `surface.cpp`, `groupmesh.cpp`, `group.cpp`,
`textscreens.cpp`, `draw.cpp`.

**Current status**: All `CHAMFER_DEBUG` macro lines have been stripped (0 remaining in chamfer.cpp,
0 in boolean.cpp). The strip was completed as part of the feature stabilization.

**Cleanup complete**: 0 remaining — all debug instrumentation stripped as of cleanup commit.
The `RECON:` and `RECON_DONE:` `fprintf(stderr,...)` calls have been removed from chamfer.cpp.

---

## Build Notes

- `src/CMakeLists.txt`: `srf/chamfer.cpp` in `solvespace_core` source list
- Build: `cmake --build build/ -j$(nproc)`
- Tests: `ctest --test-dir build/ -R chamfer --output-on-failure` (**77 tests, 0 failures** — test count as of 96df040d)
- The chamfer/fillet feature requires `ENABLE_TESTS=ON` for the test suite

See `project_info/build-notes.md` for GUI startup issues on Ubuntu 22.04.
