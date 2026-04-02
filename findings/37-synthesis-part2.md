# Synthesis Part 2: SolveSpace Internals Relevant to Chamfer/Fillet
## (Synthesizing Findings 11–20)

---

## 1. Overview

This document synthesizes findings 11 through 20, covering:
- The mathematical representations of fillet and chamfer surfaces (findings 11, 12)
- How SolveSpace's constraint solver handles parameters for new group types (finding 13)
- Community history and prior implementation attempts (finding 14)
- The SolveSpace file format and serialization requirements (finding 15)
- The boolean pipeline and why it must be bypassed for chamfer/fillet (finding 16)
- How NURBS surfaces are triangulated and rendered (finding 17)
- Edge selection UX patterns in parametric CAD (finding 18)
- The 2D sketch fillet algorithm and existing MakeTangentArc infrastructure (finding 19)
- ARC_OF_CIRCLE entity lifecycle and tangent constraints (finding 20)

Together, these findings establish the SolveSpace-specific internal mechanics that chamfer/fillet implementation must work within.

---

## 2. Fillet Surface Math: Exact Cylindrical NURBS (Finding 11)

### The Rolling Ball Algorithm
For two planar faces meeting at a convex edge with exterior dihedral angle α:
- Roll a sphere of radius `r` along the edge — the envelope swept by the sphere defines the fillet surface
- For two flat faces: the result is a **cylinder** (not a general canal surface)
- The fillet surface is thus a **degree (2,1) rational NURBS** — exact, not approximate

### The Fillet Arc Cross-Section
At any point along the edge, the fillet arc is a quadratic NURBS arc:
```
P0 = edge_point + inward1 * (r/tan(α/2))   // tangent on face 1
P1 = edge_point                              // corner (original edge point)
P2 = edge_point + inward2 * (r/tan(α/2))   // tangent on face 2
weight[1] = cos(arc_span/2) = sin(α/2)      // exact rational weight
```
For a 90° edge: `weight[1] = sin(45°) = √2/2 ≈ 0.7071`

### SolveSpace Factory Method
```cpp
SSurface fillet = SSurface::FromExtrusionOf(&arc, zero_vec, edge_vec);
// Produces: degm=2, degn=1 — exact cylindrical surface
// Recognized by: IsCylinder() → IsExtrusion() + IsCircle()
```

### Key Properties
- **`degm=2, degn=1`**: SolveSpace's `IsCylinder()` detects it → used for exact raycast intersection, constraint detection, STEP export
- **G1 continuity**: The arc is tangent to both adjacent faces at the contact lines — smooth shading automatically
- **No vertex cap needed** for MVP: skip spherical corner patches (very complex); handle only the edge region

---

## 3. Chamfer Surface Math: Exact Bilinear Plane (Finding 12)

### The Chamfer Algorithm
For two planar faces meeting at edge E, with outward normals n1, n2, offset distance d:
```
t       = edge unit direction
inward1 = t.Cross(n1).WithMagnitude(1)   // inward along face 1
inward2 = n2.Cross(t).WithMagnitude(1)   // inward along face 2
A = edgeStart + inward1 * d              // start setback on face 1
B = edgeEnd   + inward1 * d              // end setback on face 1
C = edgeEnd   + inward2 * d              // end setback on face 2
D = edgeStart + inward2 * d              // start setback on face 2
```

### SolveSpace Factory Method
```cpp
SSurface chamfer = SSurface::FromPlane(A, B-A, D-A);
// Produces: degm=1, degn=1, all weights=1
// 4 control points: A, B, C, D
```

### Key Properties
- **`degm=1, degn=1`**: Non-rational (all weights=1), exact planar surface
- **G0 only**: Sharp edges where chamfer meets adjacent faces — this is by design
- **Simpler than fillet**: No arc weights, no setback-vs-radius calculation, no rational arithmetic
- **Vertex caps**: Triangular planar faces (`FromPlane()`) at edge endpoints — simpler than fillet's spherical patches

### Chamfer vs. Fillet Summary
| Property | Chamfer | Fillet |
|----------|---------|--------|
| Surface factory | `FromPlane()` | `FromExtrusionOf(&arc)` |
| NURBS degrees | (1,1) | (2,1) |
| Weights | All 1 | middle row < 1 |
| G-continuity | G0 | G1 |
| Setback distance | `d` (= offset = face truncation) | `r/tan(α/2)` ≠ r |
| Vertex caps | Planar triangle | Spherical (skip in MVP) |
| Difficulty | Simpler | Harder |

---

## 4. Parametric Integration: How the Solver Handles Chamfer/Fillet Parameters (Finding 13)

### The Critical Decision
The chamfer offset and fillet radius are **stored scalars** in `Group::valA` — they are NOT actively solved by the Newton-Raphson solver. However, they are registered as solver params via `AddParam()` to allow future constraint-driven control.

### The Pattern (REVOLVE Analogy)
REVOLVE adds `h.param(3)` for its rotation angle with no constraining equation → angle stays at `valA`-initialized value. CHAMFER/FILLET uses the same pattern:

```cpp
// In Group::Generate():
case Type::CHAMFER:
    AddParam(param, h.param(0), valA);  // d1: free param initialized from valA
    // NO equations in GenerateEquations() — param stays at valA
    break;
case Type::FILLET:
    AddParam(param, h.param(0), valA);  // r: free param initialized from valA
    break;
```

```cpp
// In GenerateShellAndMesh():
double dist = SK.GetParam(h.param(0))->val;  // reads back valA
```

```cpp
// TextWindow callback to change value:
g->valA = newDist;                    // update stored value
SS.MarkGroupDirty(g->h);              // triggers regen
// Next regen: AddParam re-initializes h.param(0) from valA
```

### DOF Reporting
- CHAMFER gets 1 DOF (the distance is the free DOF — semantically correct)
- No `suppressDofCalculation` needed — 1 DOF for chamfer/fillet is semantically meaningful
- No new Constraint types needed for MVP

### Solver Flow for CHAMFER Group
```
GenerateAll():
  genForBBox phase:
    g->Generate() → AddParam(h.param(0), valA)
    SolveGroupAndReport() → solver sees 1 free param, 0 equations → OK, param stays at valA
    g->GenerateLoops() → empty (no sketch entities)
  
  Main phase:
    g->GenerateShellAndMesh() → reads SK.GetParam(h.param(0))->val = valA
    g->clean = true
```

---

## 5. Community History and the phkahler PR (Finding 14)

### Key GitHub Issues
| Issue | Topic | Date |
|-------|-------|------|
| #149 | "make chamfers as easy as fillets" (2D) | Jan 2017 |
| #577 | "Add chamfer and fillet tools" (3D) | Mar 2020 |
| #1024 | Official "commonly requested features" index | Apr 2021 |
| #1236 | Topological Naming Problem | May 2022 |
| #1291 | Rolling ball fillet via boolean fails | Sep 2022 |
| #1501 | Draft PR: WIP chamfer/fillet implementation | Dec 2024 |

### PR #1501: The Only Implementation Attempt
**Author**: phkahler (core contributor)  
**Status**: Draft, crashes  
**Architecture confirmed by PR**:
- Hook-in point: `Group::Generate()` → `groupmesh.cpp` post-generate
- Requires 2 NEW SCurves per edge + update BOTH TrimBy entries
- Direct topology injection (not boolean)
- **Why it crashes**: srfC has zero ctrl points, no TrimBy entries, and the curves are identity copies at the same position

### Critical Advice from phkahler
> "Next time I try I might limit myself to flat surfaces only just to get the basics down."

**This confirms the MVP scope**: flat faces only, single edge, no vertex caps.

### Topological Naming Problem (#1236)
The stable identification of "which edge" across regenerations is an unsolved hard problem. For MVP: store face pair handles (predef.entityB/C); faces are stable within a regeneration session. Document as known limitation.

### Boolean Pipeline Bug (#1291)
When using revolve+difference to manually create a fillet, if the arc profile corner point is on the face edge AND tangent, the boolean fails. Direct topology injection avoids this entirely.

---

## 6. File Format: No New SAVED[] Entries Needed (Finding 15)

### The Key Finding
All necessary data for CHAMFER/FILLET groups maps to **existing** SAVED[] fields. No changes to `src/file.cpp` are required.

| Data | SAVED[] field | Notes |
|------|---------------|-------|
| Group type | `Group.type` | CHAMFER=5400, FILLET=5401 |
| Source group | `Group.opA.v` | Handle of solid group being modified |
| Distance/radius | `Group.valA` | Primary parameter |
| Second distance | `Group.valB` | For unequal chamfer; 0=equal-leg |
| Face 1 handle | `Group.predef.entityB.v` | First selected face |
| Face 2 handle | `Group.predef.entityC.v` | Second selected face |
| Stable entity IDs | `Group.remap` | 'M' format EntityMap |
| Visibility/suppression | Various | Already serialized |

### The valC Bug
`Group.valC` in `SAVED[]` maps to `&(SS.sv.g.valB)` (copy-paste bug) — valC is broken. **Use only valA and valB** for parameters.

### Param Serialization
When `Generate()` calls `AddParam(h.param(0), valA)`, the param is stored as:
```
Param.h.v=<handle>
Param.val=<dist_value>
AddParam
```
This seeds the solver's initial guess on file load. No new format entries needed — `Param.h.v` and `Param.val` are already in SAVED[].

### Example .slvs Chamfer Block
```
Group.h.v=00000005
Group.type=5400
Group.order=2
Group.name=chamfer
Group.opA.v=00000004
Group.valA=2.00000000000000000000
Group.meshCombine=4
Group.visible=1
Group.remap={
    1 000002a4 0
}
AddGroup

Param.h.v=00050001
Param.val=2.00000000000000000000
AddParam
```

---

## 7. The Boolean Pipeline Must Be Bypassed (Finding 16)

### SolveSpace's Two Boolean Pipelines
1. **SShell pipeline** (`src/srf/boolean.cpp`): B-rep operations on rational polynomial surfaces
2. **SMesh pipeline** (`src/bsp.cpp`, `src/mesh.cpp`): Triangle mesh BSP operations

Both must be bypassed for chamfer/fillet.

### Why MakeFromBoolean Fails for Chamfer
The 7-phase boolean pipeline:
1. Build classifying BSPs
2. Copy curves, splitting at intersections
3. Generate O(n²) surface-surface intersection curves
4. Remove short segments
5. Rebuild BSPs with split curves
6. Trim surfaces: classify each region as INSIDE/OUTSIDE/COINC_SAME/COINC_OPP
7. Rewrite surface handles

**The COINC_OPP degeneracy**: The chamfer plane shares its edges exactly with the source solid's faces. The `DIFFERENCE` classification keeps `COINC_OPP` for opA (the solid), which means the coincident faces from the source solid are KEPT — exactly wrong. The chamfer face would not replace the trimmed adjacent faces.

**Confirmed**: phkahler's PR #1501 crashed using boolean approaches.

### The Direct Topology Injection Approach
Instead of boolean operations, chamfer/fillet uses:
1. `MakeFromCopyOf(prevShell)` — copy entire source shell (preserves IDs)
2. Find shared SCurve via face handle lookup
3. Compute chamfer/fillet geometry analytically (exact, no intersection computation)
4. Add new SSurface + 2 new SCurves directly to the copied shell
5. Surgically update trim polygons of the two adjacent faces
6. Assign result to `runningShell` — no `GenerateForBoolean` call

### MakeFromAssemblyOf as a Reference
`MakeFromAssemblyOf()` (`boolean.cpp:860`) is the simplest existing pipeline — concatenation without boolean math. It shows the pattern for:
- ID rewriting (`c.newH` tracking)
- `RewriteSurfaceHandlesForCurves()` call
- Post-merge bookkeeping

The chamfer's direct topology injection follows a similar but more targeted pattern.

### GenerateForBoolean Template
```cpp
// groupmesh.cpp:182
template<class T>
void Group::GenerateForBoolean(T *prevs, T *thiss, T *outs, Group::CombineAs how) {
    if(thiss->IsEmpty() || suppress) { outs->MakeFromCopyOf(prevs); return; }
    // ...
}
```
For CHAMFER/FILLET: skip this entirely. Build `runningShell` directly:
```cpp
// In GenerateShellAndMesh() for CHAMFER:
thisShell.MakeFromChamferOf(&prevg->runningShell, this, dist);
runningShell.MakeFromCopyOf(&thisShell);
return;  // Skip GenerateForBoolean
```

---

## 8. Triangulation: Both Surface Types Are Handled Automatically (Finding 17)

### The Two Triangulation Paths in SSurface::TriangulateInto()
```cpp
// src/srf/surface.cpp:414
if(degm == 1 && degn == 1) {
    poly.UvTriangulateInto(sm, this);     // ear-clip (for planes)
} else {
    poly.UvGridTriangulateInto(sm, this); // adaptive grid (for curved surfaces)
}
```

**Chamfer surfaces (degree 1,1)**: Ear-clip path → 4 vertices, 2 triangles, trivially fast, exact

**Fillet surfaces (degree 2,1 cylinders)**: Grid path → adaptive refinement until chord deviation < `SS.ChordTolMm()`, minimum 4 grid segments, automatic smooth per-vertex normals

### The Good News
**NO changes to `src/srf/triangulate.cpp` or `src/srf/surface.cpp` are needed.** The existing triangulation system:
- Handles both degree (1,1) and degree (2,1) surfaces correctly
- Runs via OpenMP parallel loop in `SShell::TriangulateInto()` — new surfaces included automatically
- Computes per-vertex normals for smooth shading of fillet cylinders
- Applies chord tolerance adaptively

### The Critical Watch-Out: `face` Field
```cpp
// In SSurface::TriangulateInto():
STriMeta meta = { face, color };
for(i = start; i < sm->l.n; i++) {
    STriangle *st = &(sm->l[i]);
    st->meta = meta;  // face field propagated to all triangles
```
If `ss->face == 0`, the surface triangles are unselectable. **New chamfer/fillet surfaces must have `face` set to a non-zero REMAP value** (REMAP_CHAMFER_FACE=1011 or REMAP_FILLET_FACE=1012).

### Watertightness Warning
`AssemblePolygon()` fails silently if trim curve endpoints don't match within LENGTH_EPS → surface produces no triangles → invisible hole in mesh. The chamfer B-rep must have exactly matching SCurve endpoints.

---

## 9. Edge Selection: Face-Pair Approach Is the Right Architecture (Finding 18)

### The Key Finding
SolveSpace has **no first-class selectable edge entities**. Edges are `SCurve` objects in `SShell`, not in `SK.entity`. However, faces ARE selectable (via `SSurface::face` → `STriangle::meta.face` → `SMesh::FirstIntersectionWith()`).

### Face-Pair Selection Flow
```
1. User hovers/clicks on face A → HitTestMakeSelection → SMesh::FirstIntersectionWith
                                 → returns tr.meta.face = hEntity.v
2. User hovers/clicks on face B → second face entity in selection
3. User invokes "New Chamfer" menu
4. GroupSelection() → gs.faces==2, gs.face[0]=faceA, gs.face[1]=faceB
5. MenuGroup(GROUP_CHAMFER) creates group with predef.entityB=faceA, entityC=faceB
```

### How to Find the Shared SCurve from Face Handles
```cpp
// In SShell::MakeFromChamferOf():
SSurface *surf1 = findSurfaceByFaceHandle(shell, face1h);
SSurface *surf2 = findSurfaceByFaceHandle(shell, face2h);

SCurve *shared = nullptr;
for(SCurve &sc : shell->curve) {
    if((sc.surfA == surf1->h && sc.surfB == surf2->h) ||
       (sc.surfA == surf2->h && sc.surfB == surf1->h)) {
        shared = &sc;
        break;
    }
}
```

### No New Selection Infrastructure Needed
- `HitTestMakeSelection` already handles face selection via ray-cast
- `GroupSelection()` already populates `gs.face[]`
- `MAX_SELECTABLE_FACES=3` is sufficient for 1-edge MVP (needs 2 faces)
- `DrawMesh(HOVERED)` and `DrawMesh(SELECTED)` already highlight faces correctly
- Zero changes needed to `draw.cpp`, `drawentity.cpp`, or `ui.h` for MVP selection

### MenuGroup() Validation
```cpp
case Command::GROUP_CHAMFER:
    SS.GW.GroupSelection();
    if(SS.GW.gs.faces != 2) {
        Error("Select exactly 2 adjacent faces first.");
        return;
    }
    // Both faces must be from the same source group:
    Entity *f0 = SK.GetEntity(SS.GW.gs.face[0]);
    Entity *f1 = SK.GetEntity(SS.GW.gs.face[1]);
    if(f0->group != f1->group) {
        Error("Both faces must belong to the same solid group.");
        return;
    }
    // Create group...
    break;
```

---

## 10. 2D Sketch Fillet: Existing Infrastructure and Parametric Design (Finding 19)

### What Already Exists
`MakeTangentArc()` in `src/modify.cpp:252` is a **non-parametric** 2D fillet:
- Uses `ParametricCurve` helper struct to parameterize lines and arcs
- Newton iteration (1000 steps) for general line-arc and arc-arc cases
- Closed-form solution for line-line case (using `el = r/tan(θ/2)`, `C = V + bisector*r/sin(θ/2)`)
- Creates `ARC_OF_CIRCLE` with `ARC_LINE_TANGENT` constraints
- Uses session global `tangentArcRadius` (NOT saved)

### The 2D Fillet Algorithm (Closed-Form, Line-Line)
```
θ = acos(t1·t2)            // corner angle
el = r / tan(θ/2)           // leg length = setback on each line
P1 = V + normalize(-t1)*el  // tangent point on line 1
P2 = V + normalize(-t2)*el  // tangent point on line 2
bis = normalize(normalize(-t1) + normalize(-t2))  // bisector
C = V + bis * r/sin(θ/2)    // arc center
```

### For Parametric 2D Fillet Group
A `Group::Type::FILLET_2D` would:
- Store `r` in `valA` (saved to file, unlike `tangentArcRadius`)
- `Generate()` computes the arc geometry using the closed-form formula above
- Creates `ARC_OF_CIRCLE` entity with center C, start P1, finish P2
- Uses `AddParam(h.param(0), valA)` for radius
- Existing `ARC_LINE_TANGENT` constraint type handles tangency enforcement

### ParametricCurve Reusability
`ParametricCurve` struct (`src/graphicswin.cpp:109-228`, declared `src/ui.h:721`):
- `MakeFromEntity(hEntity, reverse)` — initialize from line or arc
- `PointAt(t)` — position at parameter t
- `TangentAt(t)` — tangent direction at parameter t
- `CreateRequestTrimmedTo(t, reuseOrig, hent, harc, arcFinish, pointf)` — trim+constrain

**Fully reusable for parametric 2D fillet group** — avoids reimplementing the parameterization logic.

---

## 11. ARC_OF_CIRCLE Entity: Full Lifecycle (Finding 20)

### Data Model
```
ARC_OF_CIRCLE request generates:
  point[0] = center     (NOT an endpoint!)
  point[1] = start endpoint
  point[2] = finish endpoint
  normal   = workplane orientation

Radius = |point[1] - point[0]|   (implicit, no explicit radius param)
Equation generated: Distance(center, start) == Distance(center, finish)
```

### Bezier Generation (Exact NURBS)
```cpp
// drawentity.cpp:412 — from 1 to 4 segments based on sweep angle
sb.weight[1] = cos(dtheta/2);  // EXACT rational representation of circle arc
```
This same formula is used in `surface.cpp::FromRevolutionOf` and is the basis for fillet surface weights.

### ARC_LINE_TANGENT Constraint
```cpp
// constrainteq.cpp:938
AddEq(l, ld.Dot(ac.Minus(ap)), 0);
// Meaning: line_direction · (center - endpoint) = 0
// Enforces: line is perpendicular to radius at tangent point = tangency
```
This is the constraint that makes the 2D fillet arc tangent to the adjacent lines. No new constraint types needed.

### CURVE_CURVE_TANGENT Constraint
```cpp
// constrainteq.cpp:977
// For arc-arc: uses radius vectors (not tangents); flips to antiparallel → cross product = 0
// For arc-cubic: dot product of radius and cubic tangent = 0
```

### Mapping to 3D Fillet Construction
The 2D arc machinery maps directly:
| 2D Concept | 3D Equivalent |
|-----------|---------------|
| `ARC_OF_CIRCLE` entity | Fillet surface's cross-section arc (SBezier with `weight[1]`) |
| `weight[1] = cos(dtheta/2)` | `arc_weight = sin(α/2)` in fillet NURBS surface |
| `ARC_LINE_TANGENT: ld·radius=0` | G1 continuity: fillet normal matches adjacent face normal |
| `ParametricCurve::PointAt(t)` | `SSurface::PointAt(u, v)` |
| `tangentArcRadius` (session global) | `Group::valA` (persisted) |

---

## 12. Key File-Level Action Map: Findings 11-20

### Files That Need Changes (Per These Findings)

#### `src/group.cpp`
- `Group::Generate()` (~line 340): Add CHAMFER/FILLET cases with `AddParam(h.param(0), valA)`
- `Group::MenuGroup()` (~line 70): Add `GROUP_CHAMFER` and `GROUP_FILLET` cases
- `Group::GenerateEquations()`: No changes needed (CHAMFER/FILLET add no equations)

#### `src/groupmesh.cpp`
- `IsMeshGroup()` (~line 540): Add `CHAMFER` and `FILLET` to returning-true cases
- `GenerateShellAndMesh()` (~line 212): Add `else if(type == CHAMFER/FILLET)` injection point
- Skip `GenerateForBoolean` for CHAMFER/FILLET — use direct `runningShell.MakeFromCopyOf(&thisShell)`

#### `src/srf/shell.cpp` (or new `src/srf/dressup.cpp`)
- New: `SShell::MakeFromChamferOf(SShell *src, Group *g, double dist)` 
- New: `SShell::MakeFromFilletOf(SShell *src, Group *g, double radius)`

#### `src/srf/surface.h`
- Declare new `SShell::MakeFromChamferOf()` and `MakeFromFilletOf()` methods

#### `src/sketch.h`
- Add `CHAMFER=5400` and `FILLET=5401` to `Group::Type` enum (~line 181)
- Add `REMAP_CHAMFER_FACE=1011` and `REMAP_FILLET_FACE=1012` to REMAP constants (~line 305)

#### `src/ui.h`
- Add `GROUP_CHAMFER` and `GROUP_FILLET` to `Command` enum
- Add `CHAMFER_OFFSET=803` and `FILLET_RADIUS=804` to `TextWindow::Meaning` enum

#### `src/graphicswin.cpp`
- Add menu entries for GROUP_CHAMFER and GROUP_FILLET in the Group submenu

#### `src/textscreens.cpp`
- Add CHAMFER and FILLET branches to `ShowGroupInfo()` if-chain (~line 394)
- Add `ScreenChangeChamferOffset` and `ScreenChangeFilletRadius` callbacks
- Add `case Edit::CHAMFER_OFFSET` and `case Edit::FILLET_RADIUS` to `EditControlDone()`

#### Files That Do NOT Need Changes (Auto-Handles)
- `src/srf/triangulate.cpp` — both surface types handled by existing code
- `src/srf/surface.cpp` — `FromPlane()` and `FromExtrusionOf()` already exist
- `src/file.cpp` — no new SAVED[] entries needed
- `src/exportstep.cpp` — RATIONAL_B_SPLINE_SURFACE handles both surface types
- `src/mesh.cpp` — SMesh triangulation inherits from SShell automatically
- `src/draw.cpp` — face selection already works
- `src/system.cpp` — no new constraint equations needed

---

## 13. Integration Architecture Summary

The core insight from findings 11-20 is that SolveSpace's existing infrastructure is remarkably well-suited for chamfer/fillet — the new feature fits naturally into the Group pipeline:

```
SolveSpace Group Pipeline (existing):
  [Sketch Group] → EXTRUDE → LATHE → [running solid mesh]

With Chamfer/Fillet:
  [Sketch Group] → EXTRUDE → CHAMFER → [running solid mesh with chamfered edge]
  
Where CHAMFER:
  - Group::Generate(): registers 1 free solver param (valA = distance)
  - Group::GenerateShellAndMesh(): 
      calls thisShell.MakeFromChamferOf(prevShell, face1, face2, dist)
      → direct topology injection (no boolean pipeline)
      → runningShell = chamfered copy
  - IsMeshGroup() = true
  - No sketch entities generated (unlike EXTRUDE)
```

### The Five Core Algorithms Needed
1. **Find shared SCurve** from two face handles (iterate `SShell::curve`)
2. **Compute offset directions** from face normals at edge midpoint
3. **Create chamfer/fillet surface** via existing `FromPlane()` / `FromExtrusionOf()`
4. **Update trim polygons** of two adjacent faces (the hard part — trim polygon surgery)
5. **Wire watertight SCurves** (every new SCurve exactly in 2 surface trim lists)

### Critical Risk: Trim Polygon Surgery
The hardest algorithmic challenge identified across findings 11-20 is step 4: modifying the trim polygons of the two adjacent faces. The current trim polygon structure:
- `STrimBy.curve` → which SCurve trims this surface
- `STrimBy.start/finish` → XYZ endpoints

After chamfering:
- Remove the old shared SCurve from both adjacent surface trim lists
- Add the new contact curves (scContact1 to face1, scContact2 to face2)
- Adjust the neighboring trim curves' endpoint coordinates to match

If endpoints don't exactly match, `AssemblePolygon()` fails silently → invisible holes in the mesh. This watertightness requirement is the primary implementation risk.

---

## 14. Conclusions

1. **Chamfer surface = `SSurface::FromPlane()`** — degree (1,1), all weights=1, exact bilinear patch. Already in SolveSpace.

2. **Fillet surface = `SSurface::FromExtrusionOf(&arc)`** — degree (2,1), rational cylinder, weight[1]=sin(α/2). Already supported by the factory method.

3. **No solver equations needed** — both use `AddParam(h.param(0), valA)` with no constraining equations. The param stays free at `valA`.

4. **No file format changes needed** — `valA`, `opA`, `predef.entityB/C`, `remap` are all already in SAVED[].

5. **No triangulation changes needed** — both surface types are fully handled by existing `TriangulateInto()` paths.

6. **No selection infrastructure changes needed** — face-pair selection uses existing `HitTestMakeSelection()` + `GroupSelection()` + `DrawFaces()`.

7. **Standard boolean pipeline must NOT be used** — COINC_OPP degeneracy causes crashes. Direct topology injection is the correct approach.

8. **The hard part is trim polygon surgery** — removing the original shared SCurve from two adjacent face trim polygons and replacing it with two new contact curves, ensuring exact endpoint matching.

9. **MVP is well-bounded**: flat faces only, single edge, equal-leg chamfer (valB=0), constant radius fillet, no vertex caps. phkahler has confirmed this is the right starting point.

10. **2D sketch fillet** uses `MakeTangentArc()` + `ARC_LINE_TANGENT` constraints — non-parametric but reuses `ParametricCurve` infrastructure that a future parametric 2D fillet group could leverage.
