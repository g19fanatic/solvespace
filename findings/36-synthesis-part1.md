# Synthesis Part 1: What Are Chamfers/Fillets and How Do CAD Tools Implement Them?
## (Synthesizing Findings 01–10)

---

## 1. Definitions and Core Concepts

### Chamfer
A **chamfer** is a flat, angled transitional cut at the edge where two faces of a solid meet. The most common form is the equal-leg 45° chamfer: setback distance `d` measured from the edge along each adjacent face, producing a flat planar strip. The chamfer surface is **G0** (position continuous, but NOT tangent-continuous) at its boundaries — there is a sharp edge where the chamfer meets each adjacent face.

**Parametrizations:**
- **Equal-leg (equal distance):** one parameter `d` (the setback on both faces). For a 90° edge, this gives a 45° chamfer. MVP target.
- **Unequal-leg (two distances):** parameters `d1`, `d2` (different setbacks on each face, producing an asymmetric angle). Phase-2 target.
- **Distance and angle:** distance `d` on one face, angle `α` of the chamfer plane. Useful for specific machining specs.

**Engineering uses:** assembly ease (bolt holes, shafts), safety (eliminate sharp edges), manufacturing (prevent edge bruising), aesthetics.

### Fillet
A **fillet** is a rounded, curved blend at the edge where two faces meet. The surface is **G1** (tangent-continuous) — the fillet surface meets each adjacent face tangentially, with no sharp edge. This produces smooth shading and is the preferred form for stress reduction in load-bearing parts.

**Parametrizations:**
- **Constant radius:** one parameter `r` (the radius of the rolling ball). MVP target.
- **Variable radius:** `r(t)` varying along the edge (linear interpolation between `r_start` and `r_end`). Phase-2 target.
- **Full round fillet:** when a face is entirely replaced by the fillet (special case, rare).

**Engineering uses:** stress concentration reduction, aerodynamics (wing-body junctions), manufacturability (round-tipped end mills), safety.

### 2D vs. 3D Operations
The implementations are fundamentally different:

| Dimension | Chamfer Input | Output | SolveSpace Entity |
|-----------|---------------|--------|------------------|
| 2D sketch | Two lines at a corner | New `LINE_SEGMENT` between trimmed lines | `LINE_SEGMENT` |
| 2D sketch | Two lines at a corner (fillet) | Arc tangent to both lines | `ARC_OF_CIRCLE` |
| 3D solid | Two adjacent faces, shared SCurve | New planar face + trimmed adjacent faces | New `SSurface` in SShell |
| 3D solid | Two adjacent faces, shared SCurve (fillet) | New cylindrical face + trimmed adjacent faces | New `SSurface` in SShell |

---

## 2. Mathematical Foundations

### Chamfer Geometry (from Finding 12)
Given two faces meeting at edge E with outward normals `n1`, `n2`, and edge direction unit vector `t`:

**Setback directions (inward along each face, perpendicular to edge):**
```
inward1 = t × n1    (points into face 1 from the edge)
inward2 = n2 × t    (points into face 2 from the edge)
```

**Four chamfer corners (equal-leg chamfer, distance d):**
```
A = edgeStart + d * inward1   (start of L1 on face 1)
B = edgeEnd   + d * inward1   (end of L1 on face 1)
C = edgeEnd   + d * inward2   (end of L2 on face 2)
D = edgeStart + d * inward2   (start of L2 on face 2)
```

**Chamfer surface:** `SSurface::FromPlane(A, B-A, D-A)` — a degree (1,1) bilinear patch with all weights = 1. This is exact — no approximation, no rational NURBS needed.

### Fillet Geometry (from Finding 11)
The **rolling ball algorithm** conceptually rolls a sphere of radius `r` along the edge, remaining tangent to both adjacent faces simultaneously. For the common case of two flat faces:

**Setback distance (how far along each face to the tangent line):**
```
half_angle = exterior_dihedral_angle / 2
setback = r / tan(half_angle)
```
For a 90° edge: `half_angle = 45°`, `setback = r`.

**Fillet arc (cross-section at any point along the edge):**
```
P0 = edge_point + inward1 * setback   (tangent on face 1)
P1 = edge_point                        (original edge — the "corner" control point)
P2 = edge_point + inward2 * setback   (tangent on face 2)
arc_weight = sin(half_angle) = cos((π - ext_dihedral)/2)
```
For 90° edge: `arc_weight = sin(45°) = √2/2 ≈ 0.7071`.

**Fillet surface:** `SSurface::FromExtrusionOf(&arc, zero_vec, edge_vec)` — a degree (2,1) cylindrical patch with rational NURBS weights. This is also exact (not an approximation) — the `cos(θ/2)` weight formula produces an exact circle.

### Surface Degrees Summary
| Surface | deg_m | deg_n | Method | Exact? |
|---------|-------|-------|--------|--------|
| Chamfer | 1 | 1 | `FromPlane()` | Yes (bilinear) |
| Fillet (90° edge) | 2 | 1 | `FromExtrusionOf(arc)` | Yes (rational cylinder) |
| Fillet (arbitrary angle) | 2 | 1 | `FromExtrusionOf(arc)` | Yes |
| Vertex cap (chamfer) | 1 | 1 | `FromPlane()` | Yes (triangle) |
| Vertex cap (fillet) | 2 | 2 | `FromRevolutionOf(arc)` | Spherical (complex) |

Both chamfer and fillet surfaces fit within SolveSpace's `ctrl[4][4]` maximum control-point grid.

---

## 3. B-rep Topology Changes

Both operations are **topological changes** to the B-rep solid. Before/after for a single-edge operation:

**Before:** `[face1] ── edge ── [face2]` (1 edge, 2 adjacent faces)
**After chamfer:** `[face1'] ── curveA ── [chamfer_face] ── curveB ── [face2']`  + caps at each vertex

The number of faces increases by 1 (the new chamfer/fillet face), the number of edges increases by 2 (the two new contact curves replacing the original edge), and the two adjacent faces are trimmed back by the setback distance.

**Watertightness invariant (critical):** Every SCurve must be referenced by exactly two surfaces in its trim lists. Creating chamfer/fillet geometry requires:
1. Removing the original shared SCurve from both adjacent face trim lists
2. Adding two new SCurves (the contact lines on each face)
3. Adding the new chamfer/fillet surface with its own closed trim polygon
4. Handling vertex caps at edge endpoints

---

## 4. Open-Source CAD Implementations

### FreeCAD (Findings 02, 08)
**Architecture:** "DressUp" pattern — a Group/feature that takes a previous solid as input and modifies it. Two modules:
- **Part::Fillet/Chamfer** (per-edge, uses `FilletBase`): stores per-edge `{radius1, radius2}` list plus `EdgeLinks` for stable edge references.
- **PartDesign::Fillet/Chamfer** (history-based, uses `DressUp`): single radius parameter, optional "UseAllEdges" toggle.

**Key insight:** Both FreeCAD implementations delegate ALL geometry computation to OpenCASCADE via `BRepFilletAPI_MakeFillet` / `BRepFilletAPI_MakeChamfer`. FreeCAD is essentially a wrapper — it does not implement the geometry math itself.

**Chamfer API note:** `BRepFilletAPI_MakeChamfer::Add(d1, d2, edge, face)` requires a face reference because chamfer is directional (face tells OCC which side gets d1). Fillet's `::Add(r, edge)` does not need a face reference.

**Toponaming problem (2024):** FreeCAD invested multi-year effort to stabilize edge references across regeneration. The old "name edges by position" approach was unreliable. SolveSpace faces the same challenge — the MVP must acknowledge this as a known limitation.

**Chamfer types (FreeCAD):** Equal distance, Two distances, Distance+Angle. MVP: equal distance only.

### Other Open-Source Tools (Finding 02)
| Tool | Approach | Native chamfer/fillet? |
|------|----------|----------------------|
| OpenSCAD | CSG, no B-rep → use Minkowski sum workarounds | ❌ No native |
| LibreCAD | 2D sketch only → `Modify > Fillet/Chamfer` | ✅ 2D only |
| BRL-CAD | CSG tree → subtract prism/cylinder manually | ❌ No native |

**Key lesson from OpenSCAD/BRL-CAD:** Without a B-rep kernel, real edge-based fillet is impossible. SolveSpace has a B-rep kernel (SShell/SSurface), so it CAN implement real fillets.

---

## 5. SolveSpace Group Pipeline Overview (Findings 03, 04)

SolveSpace models solids through a **linear chain of Groups**. Each Group:
1. Takes a source (previous running shell or a 2D sketch in `opA`)
2. Produces `thisShell` (the group's own geometric contribution)
3. Combines `thisShell` with the previous `runningShell` via a boolean operation
4. Outputs the combined result as `runningShell`

The resulting `runningShell` is triangulated into `displayMesh` for rendering.

### Group Struct Key Fields (for Chamfer/Fillet)
```
g.type           = CHAMFER (5400) or FILLET (5401)
g.opA            = handle of the source solid group
g.valA           = chamfer distance d (or fillet radius r)
g.valB           = second distance d2 (unequal chamfer, else 0)
g.predef.entityB = hEntity of first selected face
g.predef.entityC = hEntity of second selected face
g.meshCombine    = ASSEMBLE (chamfer/fillet bypasses standard boolean)
g.remap          = EntityMap for stable face IDs
```

### Pipeline for CHAMFER/FILLET (bypasses standard boolean)
Standard EXTRUDE: `thisShell = new primitive → GenerateForBoolean → runningShell`
CHAMFER/FILLET: `thisShell = modified copy of prevShell → runningShell = thisShell` (no boolean)

The reason: the standard `MakeFromBoolean/MakeFromDifferenceOf` pipeline fails for chamfer because the chamfer surface is coincident with the adjacent original faces, causing `COINC_OPP` degeneracy in the boolean classifier (confirmed by phkahler's crashed PR #1501).

### Why Direct Topology Injection
Instead of boolean, chamfer/fillet uses **direct topology injection** into a copy of the source shell:
1. `MakeFromCopyOf(srcShell)` — copy all surfaces + curves with preserved IDs
2. Find the two target surfaces by face handle lookup (`ss.face == faceH.v`)
3. Find the shared SCurve between them
4. Compute chamfer/fillet geometry (corners, arc weight)
5. Create new SSurface + 2 new SCurves
6. Surgically update the trim polygons of the two adjacent surfaces
7. This is the complete modified solid — assign to `runningShell`

---

## 6. SolveSpace NURBS Surface System (Finding 04)

SolveSpace uses **explicit B-rep** with rational polynomial (NURBS/Bezier) surfaces. Key structures:

### SSurface
- `degm`, `degn`: degree in m and n directions (max 3 each)
- `ctrl[4][4]`: control points (statically allocated 4×4)
- `weight[4][4]`: NURBS weights (1.0 for non-rational)
- `List<STrimBy> trim`: ordered list of trim curves forming a closed boundary
- `face`: uint32_t = hEntity.v of the face entity (for selection)
- `color`: RgbaColor

### SCurve
- `surfA`, `surfB`: the two surfaces this curve separates (half-edge concept)
- `isExact`: whether the exact Bezier form is available
- `exact`: the SBezier (rational polynomial curve)
- `pts`: piecewise-linear approximation

### STrimBy
- `curve`: hSCurve — which curve trims this surface
- `backwards`: traversal direction
- `start`, `finish`: XYZ endpoints

### SShell
- `IdList<SCurve,hSCurve> curve`: all B-rep edges
- `IdList<SSurface,hSSurface> surface`: all B-rep faces
- `booleanFailed`: error flag

### Factory Methods Available (no new ones needed)
- `SSurface::FromPlane(pt, u, v)` → degree (1,1), all weights=1 — **chamfer surface**
- `SSurface::FromExtrusionOf(&sb, t0, t1)` → degree (degm,1) — **fillet surface when sb is arc**
- `SSurface::FromRevolutionOf(...)` → degree (degm,2) — for revolution/lathe, also vertex caps

---

## 7. Edge Detection and Selection (Finding 09)

### Current State of Edge Selection
SolveSpace has **NO first-class selectable edge entities**. Edges (SCurves) are internal B-rep topology, invisible to the entity system.

**What IS selectable:**
- **Face entities** (`FACE_NORMAL_PT` etc.) — via `SMesh::FirstIntersectionWith()` ray-cast
- **Sketch entities** (LINE_SEGMENT, ARC_OF_CIRCLE etc.) — via ObjectPicker/Canvas

### Face-Pair Approach (MVP)
For chamfer/fillet MVP, the user selects **two adjacent faces** and the system derives the shared SCurve:

```cpp
// Find shared SCurve from face handles:
for(SCurve &sc : shell->curve) {
    if((sc.surfA == surfA.h && sc.surfB == surfB.h) ||
       (sc.surfA == surfB.h && sc.surfB == surfA.h)) {
        sharedCurve = &sc;
        break;
    }
}
```

**GroupSelection()** already populates `gs.face[]` and `gs.faces` count. MAX_SELECTABLE_FACES=3 is sufficient for 1-edge MVP (needs 2 faces).

**Storage:** Face handles stored in `predef.entityB` and `predef.entityC` — both already serialized in the SAVED[] table. No new file format entries needed.

---

## 8. Existing 2D Fillet Infrastructure (Finding 05)

SolveSpace already has a non-parametric 2D fillet: `MakeTangentArc()` in `src/modify.cpp:252`. This creates an `ARC_OF_CIRCLE` with `ARC_LINE_TANGENT` constraints tangent to both adjacent lines.

**Key differences from parametric fillet group:**
- Non-parametric: radius is fixed at creation time (from `SS.tangentArcRadius` global, NOT saved)
- Not a Group: the arc is just a regular sketch entity
- Parametric 2D fillet would need `Group::valA` for the radius (saved) + `Group::Generate()` to create the arcs

**Existing constraints already handle 2D tangency:**
- `ARC_LINE_TANGENT` (constrainteq.cpp:938): `ld · (center - endpoint) = 0` — one equation
- `CURVE_CURVE_TANGENT` (constrainteq.cpp:977): for arc-arc tangency
- No new constraint types needed for 2D fillet

**Exact NURBS arc weight:** `weight[1] = cos(dtheta/2)` for arc spanning angle `dtheta` — already used in entity.cpp for arc Bezier generation and in surface.cpp for FromRevolutionOf. Same formula used for fillet cylindrical surfaces.

---

## 9. NURBS Surface Generation for Extrusion (Finding 10)

The `SShell::MakeFromExtrusionOf()` in `src/srf/shell.cpp` is the best model for implementing `MakeFromChamferOf()`. It shows the complete pattern:

1. **Create top/bottom surfaces:** `SSurface::FromPlane(...)`
2. **For each profile curve:** `SSurface::FromExtrusionOf(&curve, t0, t1)`
3. **Create SCurves** at each profile position (both top and bottom translations)
4. **Wire trim references:** `STrimBy::EntireCurve(this, hc, backwards)` for both surfaces sharing each curve
5. **Create vertical seam curves** connecting adjacent side surfaces

**Key pattern for trim wiring:**
```cpp
// A curve separating surface A and surface B:
sc.surfA = hA;
sc.surfB = hB;
hSCurve hc = curve.AddAndAssignId(&sc);

// Surface A traverses curve forwards:
stb = STrimBy::EntireCurve(this, hc, /*backwards=*/false);
surface.FindById(hA)->trim.Add(&stb);

// Surface B traverses the SAME curve backwards:
stb = STrimBy::EntireCurve(this, hc, /*backwards=*/true);
surface.FindById(hB)->trim.Add(&stb);
```

The `backwards` flag ensures consistent orientation convention — the interior of each surface is on the "left" of each traversed trim curve.

---

## 10. Key Synthesis Findings

1. **Chamfer = planar surface, fillet = cylindrical surface** — both exact in NURBS, both within SolveSpace's ctrl[4][4] limit.

2. **SolveSpace's Group model is a perfect DressUp pattern** — `opA` = source group, `valA` = parameter, `GenerateShellAndMesh()` produces modified shell. No new Group struct fields needed.

3. **Standard boolean pipeline CANNOT be used for chamfer/fillet** — COINC_OPP degeneracy. Must use direct topology injection.

4. **Direct topology injection** = copy shell + find shared edge + compute geometry + surgically update two adjacent trim polygons + add new surface + add 2 new SCurves.

5. **The hardest part is trim polygon surgery** — not the surface creation itself. The adjacent faces must be clipped at the setback/contact lines with correct endpoint matching.

6. **No new file format entries needed** — `valA`, `valB`, `predef.entityB/C`, `opA`, `meshCombine`, `remap` are all already in SAVED[].

7. **No changes to triangulation or STEP export** — both are degree-agnostic and handle the new surfaces automatically.

8. **Face-pair selection using existing infrastructure** — `HitTestMakeSelection()` + `GroupSelection()` already handle face entities; two face handles uniquely identify a shared edge.

9. **FreeCAD invested years in toponaming** — SolveSpace should acknowledge this as a known limitation for MVP and use the remap mechanism for stability.

10. **phkahler's PR #1501 confirms the hardest parts**: partial shell modification is very difficult; starting with flat surfaces only is the right MVP scope.
