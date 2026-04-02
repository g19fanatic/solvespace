# Finding 22: Group::Type::FILLET — Design

## Overview

This document provides the complete design for `Group::Type::FILLET` in SolveSpace,
parallel to finding 21 (chamfer design) but covering the key differences that make
FILLET more complex. Sources include:
- `src/sketch.h` (Group struct, enums, REMAP constants)
- `src/group.cpp` (MenuGroup, Generate patterns)
- `src/groupmesh.cpp` (GenerateShellAndMesh, IsMeshGroup)
- `src/srf/surface.h` (SSurface, SCurve, STrimBy)
- `src/srf/surface.cpp` (FromExtrusionOf, IsCylinder, FromPlane, FromRevolutionOf)
- AGENT.md Known Learnings (accumulated from findings 01-21)
- Finding 11 (fillet mathematics)
- Finding 12 (chamfer mathematics for comparison)
- Finding 21 (chamfer group design — FILLET mirrors this structure)

---

## 1. Type Enum Addition

**File:** `src/sketch.h:181`

FILLET shares the same enum block as CHAMFER (see finding 21):

```cpp
enum class Type : uint32_t {
    // ... existing ...
    CHAMFER  = 5400,   // flat angled cut at edge
    FILLET   = 5401,   // rounded blend at edge
};
```

Value 5401 is immediately after CHAMFER (5400) — semantically grouped as the two
"edge dressing" operations. Both fit the SolveSpace numbering convention (5100s
are geometry ops; 5400 onward is a natural extension).

---

## 2. FILLET vs CHAMFER: Core Differences

| Property | CHAMFER | FILLET |
|----------|---------|--------|
| Surface type | Planar (degree 1,1) | Cylindrical (degree 2,1) |
| Parameter | Offset distance d | Radius r |
| Arc weight | N/A (planar) | cos(arc_angle/2) = sin(dihedral/2) |
| G-continuity | G0 (position only) | G1 (tangent to both faces) |
| Setback distance | d (equal to offset) | r / tan(θ/2) from edge |
| Exact NURBS? | Yes (all weights=1) | Yes (rational, weights < 1) |
| Trim polygon work | Clip faces at offset lines | Clip faces at setback lines |
| Vertex cap | Planar triangle (simpler) | Spherical (complex, skip in MVP) |
| Implementation difficulty | Lower | Higher (but same structure) |
| SolveSpace factory | `SSurface::FromPlane()` | `SSurface::FromExtrusionOf(&arc)` |

---

## 3. Group Field Usage for FILLET

The existing Group struct (src/sketch.h:165-300) provides exactly the fields needed —
NO new fields required, just like CHAMFER:

| Field | FILLET usage |
|-------|--------------|
| `opA` | `hGroup` of the source solid group |
| `valA` | Fillet radius r in model units |
| `valB` | Unused for MVP (reserved for variable-start-radius) |
| `predef.entityB` | `hEntity` of face 0 (first selected face) |
| `predef.entityC` | `hEntity` of face 1 (second selected face) |
| `meshCombine` | ASSEMBLE (direct injection, bypass boolean) |
| `remap` | EntityMap for stable generated entity IDs |
| `name` | Default: `"fillet"` |
| `color` | Inherited from source group |

### Parameter Details

Fillet has a **single parameter** (radius r) stored in `valA` and mirrored as solver
param(0). Unlike chamfer's equal/unequal leg duality, fillets are rotationally symmetric
— there's no "d1 vs d2" concept. This is confirmed by FreeCAD's API:
- `BRepFilletAPI_MakeChamfer::Add(d1, d2, edge, face)` — two distances
- `BRepFilletAPI_MakeFillet::Add(r, edge)` — single radius

For variable-radius fillets (future): r1=valA at edge start, r2=valB at edge end.
For MVP: only constant radius — valA is the single parameter.

---

## 4. REMAP Constants

**File:** `src/sketch.h:305`

From finding 21 (and confirmed by AGENT.md):

```cpp
enum {
    // ... existing (1000-1010) ...
    REMAP_CHAMFER_FACE = 1011,   // from task 21
    REMAP_FILLET_FACE  = 1012,   // for fillet surface entity
};
```

The fillet surface entity gets `face = g->Remap(face1h, REMAP_FILLET_FACE).v`.

---

## 5. The Core Geometry: SShell::MakeFromFilletOf()

**New function to add in:** `src/srf/shell.cpp` (or new `src/srf/fillet.cpp`)

```cpp
void SShell::MakeFromFilletOf(SShell *src,
                               hEntity face1h, hEntity face2h,
                               double r,
                               Group *g);
```

### Fillet Algorithm (for two planar faces)

The complete algorithm based on findings 11 and 12 (math) and finding 21 (structure):

```
INPUTS:
  src       = source SShell (copy from prevg->runningShell)
  face1h    = hEntity of first face (stored in predef.entityB)
  face2h    = hEntity of second face (stored in predef.entityC)
  r         = fillet radius

STEP 1: Copy source shell
  *this = copy of src (MakeFromCopyOf)

STEP 2: Find the two faces
  surf1 = find SSurface* where ss.face == face1h.v
  surf2 = find SSurface* where ss.face == face2h.v
  if either not found → booleanFailed = true; return

STEP 3: Validate flat faces (MVP constraint)
  if !surf1->IsFlat() || !surf2->IsFlat() → error "fillet MVP requires flat faces"

STEP 4: Find shared SCurve
  sharedSC = iterate this->curve; find sc where
    (sc.surfA == surf1->h && sc.surfB == surf2->h) ||
    (sc.surfA == surf2->h && sc.surfB == surf1->h)
  if not found → booleanFailed = true; error "faces not adjacent"

STEP 5: Get edge geometry
  edgeStart = sharedSC.pts.l.First()->p
  edgeEnd   = sharedSC.pts.l.Last()->p
  edgeVec   = edgeEnd - edgeStart       // edge direction vector (not normalized)
  edgeDir   = edgeVec.WithMagnitude(1)  // unit edge direction

STEP 6: Get face normals at edge midpoint
  edgeMid = (edgeStart + edgeEnd) * 0.5
  Point2d uv1, uv2
  surf1->ClosestPointTo(edgeMid, &uv1)
  surf2->ClosestPointTo(edgeMid, &uv2)
  n1 = surf1->NormalAt(uv1)  // outward normal of face 1
  n2 = surf2->NormalAt(uv2)  // outward normal of face 2

STEP 7: Validate convex edge (outward normals diverge)
  // For convex edge: n1·n2 < 0 (normals point away from each other)
  if n1.Dot(n2) >= 0:
    // Concave edge — could add material fillet but deferred for MVP
    booleanFailed = true; return

STEP 8: Compute fillet arc geometry
  // Setback directions (inward along each face, perpendicular to edge)
  inward1 = edgeDir.Cross(n1).WithMagnitude(1)   // points INTO face 1 from edge
  inward2 = n2.Cross(edgeDir).WithMagnitude(1)   // points INTO face 2 from edge

  // Setback distance (how far along each face to the tangent line)
  // half_angle = acos(-n1·n2) / 2   [= half of arc sweep angle]
  half_angle = acos(CLAMP(-n1.Dot(n2), -1.0, 1.0)) / 2.0
  setback    = r / tan(half_angle)   // = r * cos(half_angle) / sin(half_angle)

  // Arc weight for quadratic rational NURBS (exact circle)
  arc_weight = sin(half_angle)   // = cos(arc_angle / 2) where arc_angle = π - 2*half_angle

STEP 9: Compute the four key points
  // At edgeStart:
  A0 = edgeStart + inward1.ScaledBy(setback)  // tangent point on face 1, start
  B0 = edgeStart + inward2.ScaledBy(setback)  // tangent point on face 2, start
  // At edgeEnd:
  A1 = edgeEnd   + inward1.ScaledBy(setback)  // tangent point on face 1, end
  B1 = edgeEnd   + inward2.ScaledBy(setback)  // tangent point on face 2, end

STEP 10: Build the fillet arc (cross-section at edgeStart)
  SBezier arc
  arc.deg       = 2       // quadratic NURBS
  arc.ctrl[0]   = A0      // tangent on face 1 at start
  arc.ctrl[1]   = edgeStart  // original edge point (acts as "corner" P1)
  arc.ctrl[2]   = B0      // tangent on face 2 at start
  arc.weight[0] = 1.0
  arc.weight[1] = arc_weight  // < 1, = sin(half_angle) for exact circle
  arc.weight[2] = 1.0

STEP 11: Extrude arc along edge to create cylinder surface
  Vector t0 = Vector::From(0, 0, 0)  // start offset = zero (arc already at edgeStart)
  Vector t1 = edgeVec                 // = edgeEnd - edgeStart
  SSurface filletSurf = SSurface::FromExtrusionOf(&arc, t0, t1)
  filletSurf.face = g->Remap(face1h, REMAP_FILLET_FACE).v
  filletSurf.color = src->surface.First()->color  // inherit color

STEP 12: Create 2 new contact SCurves
  // scContact1: line from A0 to A1, shared between surf1 and filletSurf
  SCurve scContact1 = CreateLineSCurve(A0, A1, surf1, &filletSurf)
  
  // scContact2: line from B0 to B1, shared between surf2 and filletSurf  
  SCurve scContact2 = CreateLineSCurve(B0, B1, &filletSurf, surf2)

STEP 13: Create fillet surface boundary SCurves
  // The fillet surface needs trim curves on all 4 sides:
  // - scContact1 (shared with surf1)
  // - scContact2 (shared with surf2)
  // - scEnd0: curve at edgeStart end (arc from A0 to B0)
  // - scEnd1: curve at edgeEnd end (arc from A1 to B1)
  SCurve scEnd0 = CreateArcSCurve(arc, &filletSurf, /*endcap surface*/)
  // For MVP: end caps can be simple planar faces or left open

STEP 14: Update trim polygons on surf1
  // Remove old sharedSC from surf1's trim list
  // Add scContact1 as new trim boundary (the setback line)
  // surf1 is now truncated at A0..A1 instead of reaching the original edge

STEP 15: Update trim polygons on surf2
  // Remove old sharedSC from surf2's trim list
  // Add scContact2 as new trim boundary

STEP 16: Add fillet surface and new SCurves to this shell
  hSCurve h1 = this->curve.AddAndAssignId(&scContact1)
  hSCurve h2 = this->curve.AddAndAssignId(&scContact2)
  this->surface.AddAndAssignId(&filletSurf)

STEP 17: Add trim polygon to filletSurf
  // filletSurf needs trim curves describing its rectangular-ish boundary:
  filletSurf.trim.Add(STrimBy for scContact1, forwards)
  filletSurf.trim.Add(STrimBy for scContact2, backwards)
  // Plus endcap curves if created

STEP 18: Validate watertightness
  // Verify every SCurve referenced by exactly 2 surfaces
  // Log error and set booleanFailed if violated
```

### Key Mathematical Detail: The Arc Weight Formula

From finding 11, section 4 and the SolveSpace source (`surface.cpp::FromRevolutionOf`):

For a fillet arc between two faces with outward normals n1, n2:
- If the edge is convex: n1·n2 < 0 (normals diverge away from each other)
- The arc spans angle: `arc_angle = π - 2*half_angle` where `half_angle = acos(-n1·n2)/2`
- NURBS weight: `arc_weight = cos(arc_angle/2) = cos((π/2) - half_angle) = sin(half_angle)`

For a 90° edge (interior dihedral = 90°, n1·n2 = 0):
- `half_angle = acos(0)/2 = π/4`
- `setback = r / tan(π/4) = r`
- `arc_weight = sin(π/4) = √2/2 ≈ 0.7071`
- Arc spans 90° (quarter cylinder) — correct!

For a 120° edge (e.g., chamfered hexagonal prism edge):
- Interior dihedral = 120°, n1·n2 = cos(60°) = 0.5... wait
- Actually for OUTWARD normals: n1·n2 = -cos(exterior_dihedral) where exterior_dihedral is measured outside the solid
- For a right angle box: exterior angle = 90°, outward normals are perpendicular → n1·n2 = 0

The correct formula (clarified):
- Let α = exterior dihedral angle (angle OUTSIDE the solid, between the two face planes)
- For a right-angle box edge: α = 90°
- `n1·n2 = cos(180° - α) = -cos(α)` (normals point outward)
- `half_angle = α/2`
- `arc_weight = sin(α/2)`
- `setback = r / tan(α/2)`

This matches the formula in `surface.cpp::FromRevolutionOf`: `w = cos(dtheta/2)`.

---

## 6. Parameter Setup in Generate()

**File:** `src/group.cpp` — Group::Generate() function

FILLET needs one parameter (radius r):

```cpp
case Type::FILLET:
    // param(0) = fillet radius r
    AddParam(param, h.param(0), valA);
    // valB reserved for future variable-radius (start radius)
    break;
```

Same pattern as CHAMFER. The radius is stored in `valA` (human-readable, in mm).
It becomes solver param(0) to allow future constraint-driven radius.

---

## 7. MenuGroup() Handler

**File:** `src/group.cpp:70`

```cpp
case Command::GROUP_FILLET: {
    // Require active group to be a mesh group
    Group *srcg = SK.GetGroup(SS.GW.activeGroup);
    if(!srcg->IsMeshGroup()) {
        Error(_(\"Fillet can only be applied to solid groups. \"\
                \"The active group must produce a solid.\\n\"));
        return;
    }
    // Require selection of exactly 2 adjacent faces
    SS.GW.GroupSelection();
    auto const &gs = SS.GW.gs;
    if(gs.faces != 2 || gs.n != 2) {
        Error(_(\"Fillet requires exactly 2 adjacent faces to be selected. \"\
                \"Click one face, Ctrl+click the adjacent face.\\n\"));
        return;
    }

    g.type = Type::FILLET;
    g.opA  = SS.GW.activeGroup;
    g.predef.entityB = gs.face[0];
    g.predef.entityC = gs.face[1];
    g.valA = 1.0;   // default 1mm fillet radius
    g.valB = 0.0;   // unused in MVP
    g.meshCombine = CombineAs::ASSEMBLE;
    g.name = C_("group-name", "fillet");
    break;
}
```

Identical structure to GROUP_CHAMFER but with different defaults:
- Default radius = 1mm (matching FreeCAD default of 1mm)
- Name = "fillet" instead of "chamfer"

---

## 8. GenerateShellAndMesh() Addition

**File:** `src/groupmesh.cpp` — in Group::GenerateShellAndMesh()

```cpp
} else if(type == Type::FILLET) {
    double r = SK.GetParam(h.param(0))->val;

    Group *src = SK.GetGroup(opA);
    SShell *prevShell = &(src->runningShell);

    if(prevShell->IsEmpty()) {
        return;  // source has no solid
    }

    if(r <= 0) {
        booleanFailed = true;
        return;  // invalid radius
    }

    // Build the filleted shell via direct topology injection
    thisShell.MakeFromFilletOf(prevShell,
                                predef.entityB,
                                predef.entityC,
                                r, this);
    
    // The fillet IS the complete modified solid
    runningShell.MakeFromCopyOf(&thisShell);
    
    displayDirty = true;
    return;  // Skip GenerateForBoolean
}
```

**Same paradigm as CHAMFER** — bypass the standard boolean pipeline.
The fillet group produces the COMPLETE modified solid directly.

---

## 9. IsMeshGroup() Addition

**File:** `src/groupmesh.cpp:545`

```cpp
bool Group::IsMeshGroup() {
    switch(type) {
        case Type::EXTRUDE:
        case Type::LATHE:
        case Type::REVOLVE:
        case Type::HELIX:
        case Type::LINKED:
        case Type::CHAMFER:
        case Type::FILLET:   // NEW
            return true;
        default:
            return false;
    }
}
```

---

## 10. SSurface::FromExtrusionOf for Fillet Surface

The fillet surface is created using the EXISTING factory method `SSurface::FromExtrusionOf`.

From `surface.cpp:10-28`:
```cpp
SSurface SSurface::FromExtrusionOf(SBezier *sb, Vector t0, Vector t1) {
    SSurface ret = {};
    ret.degm = sb->deg;  // = 2 for fillet arc
    ret.degn = 1;        // = 1 for linear extrusion along edge

    int i;
    for(i = 0; i <= ret.degm; i++) {
        ret.ctrl[i][0] = (sb->ctrl[i]).Plus(t0);   // start column
        ret.weight[i][0] = sb->weight[i];
        ret.ctrl[i][1] = (sb->ctrl[i]).Plus(t1);   // end column
        ret.weight[i][1] = sb->weight[i];
    }
    return ret;
}
```

For fillet:
- `sb` = the quadratic arc (deg=2, 3 ctrl points, weight[1] < 1)
- `t0` = zero vector (arc already positioned at edgeStart)
- `t1` = `edgeEnd - edgeStart` (translate to form the cylindrical surface)
- Result: `degm=2, degn=1` → **exactly what `IsCylinder()` recognizes!**

This means:
1. Fillet surfaces are automatically detected as cylinders (for constraints, export, etc.)
2. `TriangulateInto` handles them via the `UvGridTriangulateInto` path (smooth shading)
3. STEP export can export them as exact cylindrical faces

---

## 11. The Fillet Surface NURBS Structure

For a fillet of radius `r` between two faces with outward normals `n1`, `n2`,
meeting at an edge from `E0` to `E1`:

```
ctrl[0][0] = A0 (tangent on face 1 at edge start)   weight[0][0] = 1.0
ctrl[0][1] = A1 (tangent on face 1 at edge end)     weight[0][1] = 1.0
ctrl[1][0] = E0 (original edge start point)         weight[1][0] = arc_weight
ctrl[1][1] = E1 (original edge end point)           weight[1][1] = arc_weight
ctrl[2][0] = B0 (tangent on face 2 at edge start)   weight[2][0] = 1.0
ctrl[2][1] = B1 (tangent on face 2 at edge end)     weight[2][1] = 1.0
```

Where:
- `A0 = E0 + inward1 * setback`  (setback = r/tan(α/2))
- `B0 = E0 + inward2 * setback`
- `A1 = E1 + inward1 * setback`
- `B1 = E1 + inward2 * setback`
- `arc_weight = sin(α/2)` where α = exterior dihedral angle

Degrees:
- `degm = 2` (quadratic in the arc/angular direction)
- `degn = 1` (linear in the edge/axial direction)

This is a **rational bilinear-like surface in n, quadratic in m** — exactly a
cylindrical surface segment. SolveSpace's `IsCylinder()` will recognize it.

---

## 12. G1 Continuity Verification

The fillet surface achieves **G1 continuity** (tangent continuity) with both adjacent faces.

For face 1 (F1):
- The fillet surface's normal at any point on the contact line L₁ = outward normal of F1
- Why: The fillet arc is constructed tangent to F1 at P0. The extrusion direction (along edge)
  is parallel to F1. Therefore `FromExtrusionOf(arc, ...)` at column 0 produces a surface
  whose normal matches F1 along the entire contact line L₁.

For face 2 (F2): identical reasoning by symmetry.

This is guaranteed by the construction of the arc control point:
- `arc.ctrl[0] = A0` is ON face F1
- `arc.ctrl[1] = E0` is at the original edge (on both face planes extended)
- The tangent to the arc at A0 is `E0 - A0` (direction from A0 toward edge)
- This tangent is IN the plane of F1 (since A0 and E0 are both on F1's plane)
- Therefore the fillet surface at A0 is tangent to F1 ✓

---

## 13. Trim Polygon Modification (The Hard Part)

The most algorithmically complex part of fillet implementation is modifying the trim
polygons of the two adjacent faces.

### Current state (before fillet):
- `surf1` has a trim polygon that includes the original edge (sharedSC as a trim curve)
- `surf2` has a trim polygon that includes the original edge (same sharedSC)
- Both trim polygons close around the original edge vertices

### Required state (after fillet):
- `surf1`'s trim polygon ends at the setback line (scContact1: A0→A1)
- `surf2`'s trim polygon ends at the setback line (scContact2: B0→B1)
- `sharedSC` is removed from both trim polygons
- `scContact1` is added to surf1's trim polygon AND filletSurf's trim polygon
- `scContact2` is added to surf2's trim polygon AND filletSurf's trim polygon

### Algorithm for trim polygon clipping:

```
For surf1:
  1. Find STrimBy entry in surf1.trim[] that references sharedSC
  2. Replace that STrimBy with a new one referencing scContact1
     (same direction, but the curve is now the setback line)
  3. The endpoints (start/finish in STrimBy) change:
     - Old: edge endpoints E0, E1
     - New: setback endpoints A0, A1

For surf2: analogous replacement of sharedSC → scContact2 (B0→B1)
```

### The tricky part: endpoint connectivity

The trim polygon of surf1 currently has edges meeting at E0 and E1.
After the fillet:
- The trim polygon of surf1 needs to end at A0 and A1 (not E0, E1)
- Other trim curves in surf1's polygon that connected TO E0/E1 now need to connect to A0/A1

**This requires finding and updating the neighboring trim curves** that share the
vertices E0 and E1 with the shared edge. For the MVP case (flat rectangular face),
the adjacent trim curves (the face's other edges) will need their endpoints adjusted.

**Simpler approach for MVP**: Only support fillets where the two adjacent faces meet
at a single straight edge, and the edge is a complete side of each face (not a partial
edge). In this case:
- Replace `sharedSC` with `scContact1` in surf1's trim (updating start/finish vectors)
- The OTHER trim curves of surf1 (the other sides) still end at E0 and E1
- We need to extend/clip those other trim curves to end at A0 and A1 instead

**Even simpler approach (true MVP)**: Accept that the trim polygon may be slightly
incorrect at the fillet ends, producing a small gap. Use this as "works for the
middle of long edges; vertex endpoint handling deferred."

phkahler's advice (finding 14): "limit to flat surfaces only" — this is consistent with
the trim polygon approach being simpler for rectangular face panels.

---

## 14. End Cap Handling (Vertex Cases)

At the two ends of the filleted edge (at vertex E0 and vertex E1), there's a gap
where the fillet surface ends. Closing this gap requires additional geometry.

### Options:
1. **Skip (leave open)**: Simple hole in the B-rep. Not watertight, but visually
   acceptable if fillets are applied to complete edges only.
2. **Planar end cap**: Add a triangular planar `SSurface::FromPlane()` that closes
   the gap at each vertex. Works for convex vertex cases.
3. **Spherical corner patch**: For vertex where 3 fillets meet (a box corner),
   needs a spherical triangle patch. `FromRevolutionOf` could create this. Complex.

**For MVP**: Use option 1 (skip end caps) if the edge spans the full face boundary
(i.e., the face has this edge as a complete side). Leave vertex caps as future work.

If end caps are needed, note that:
- End cap at E0: small polygon bounded by arc (A0→B0) + contact lines → planar if F1,F2 flat
- This arc is the cross-section arc from step 10 above
- Create as an `SSurface::FromPlane(A0, E0-A0, B0-A0)` (planar cap)
  → Wait, this is actually a curved region. Use exact arc → need `FromRevolutionOf` for corners.

**Recommendation**: For MVP, only support fillets where the two faces are bounded
rectangles and the shared edge spans their full height. Skip vertex geometry. The
fillet surface has open ends, but since the adjacent faces are also truncated at the
same points, the boundary is consistent.

---

## 15. Textscreen UI

**File:** `src/textscreens.cpp:394` — ShowGroupInfo()

Add FILLET branch in the if-chain:

```cpp
} else if(g->type == Group::Type::FILLET) {
    Printf(true, "%Ft fillet group");
    Printf(false, "%Ba   source: %s", SK.GetGroup(g->opA)->DescriptionString());
    Printf(false, "%Bd   radius: %s",
        SS.MmToString(SK.GetParam(g->h.param(0))->val).c_str());
    Printf(false, "%Ba   [%f%D change radius%E]",
        &TextWindow::ScreenEditGroupName, g->h.v);
    // Could also show dihedral angle here for user reference
}
```

**Edit meaning for FILLET_RADIUS:**

```cpp
// In src/ui.h, struct TextWindow, enum class Meaning:
CHAMFER_OFFSET = 803,
FILLET_RADIUS  = 804,
```

When user clicks "change radius":
```cpp
// In TextWindow::EditControlDone():
case Meaning::FILLET_RADIUS: {
    SS.UndoRemember();
    Group *g = SK.GetGroup(editControl.group);
    g->valA = SS.StringToMm(s);
    // Also update param(0) initial guess
    SK.GetParam(g->h.param(0))->val = g->valA;
    break;
}
```

---

## 16. Command Enum Addition

**File:** `src/ui.h` — Command enum

```cpp
// Group
GROUP_3D,
GROUP_WRKPL,
GROUP_EXTRUDE,
GROUP_LATHE,
GROUP_REVOLVE,
GROUP_HELIX,
GROUP_ROT,
GROUP_TRANS,
GROUP_LINK,
GROUP_RECENT,
GROUP_CHAMFER,  // from task 21
GROUP_FILLET,   // NEW
```

---

## 17. Menu Entry

**File:** `src/graphicswin.cpp` — static MenuEntry[] array

```cpp
{ 3, "New Chamfer Group",  Command::GROUP_CHAMFER, 0, &Group::MenuGroup },
{ 3, "New Fillet Group",   Command::GROUP_FILLET,  0, &Group::MenuGroup },
```

Both go in the same location near the other solid operation group commands.

---

## 18. FILLET vs CHAMFER: Implementation Complexity Analysis

### Why FILLET is harder than CHAMFER:

1. **Arc weight calculation**: Must compute dihedral angle → `sin(α/2)` weight
   - Requires `NormalAt()` on both faces
   - Requires `ClosestPointTo()` for UV coordinate at edge midpoint
   - CHAMFER has no such computation — all weights = 1 (planar surface)

2. **Setback distance differs from surface extent**:
   - CHAMFER: the face is clipped at distance d from edge (d = offset = chamfer extent)
   - FILLET: the face is clipped at setback `r/tan(α/2)` from edge, but the fillet
     surface extends over a different arc length `r * α` (arc length of fillet)
   - Must compute `setback` separately from radius `r`

3. **Rational NURBS**: The fillet surface has weights ≠ 1 for the middle row
   - All surface evaluation (PointAt, NormalAt) must use homogeneous coordinates
   - CHAMFER control points all have weight 1 → simpler bilinear interpolation

4. **Triangulation**: FILLET uses `UvGridTriangulateInto` (adaptive grid, smooth normals)
   vs CHAMFER's `UvTriangulateInto` (ear-clip, only 2 triangles)
   - More complex rendering path, but already implemented in SolveSpace

5. **STEP export**: Cylinders export as exact cylindrical faces in STEP
   - `exportstep.cpp` likely has special handling for `IsCylinder()` surfaces
   - CHAMFER (planar) exports as simple planar B-rep faces

6. **G1 continuity enforcement**: Fillet MUST be geometrically tangent to both faces
   - If the setback/arc computations are slightly off, you get shading artifacts
   - CHAMFER's G0 (no tangency required) is forgiving of minor inaccuracies

### Why they share so much structure:
- Both bypass the boolean pipeline
- Both use `MakeFromCopyOf` + direct topology injection
- Both use identical face-pair selection model
- Both use identical Group field layout
- Both add one face surface + two contact SCurves per edge
- Both need the same trim polygon modification (the hard part)

---

## 19. MVP Scope for FILLET

**Confirmed minimal feature set** (consistent with phkahler's recommendation and FreeCAD patterns):

- ✅ Constant radius only (valA = r)
- ✅ Single edge (two adjacent planar faces at a time)
- ✅ Flat faces only (no curved surface fillets)
- ✅ Convex edges only (exterior fillet, removes material)
- ❌ Vertex cap geometry (skip — leave ends open or planar)
- ❌ Variable radius (valB unused in MVP)
- ❌ Concave edge fillets (interior fillet = adds material, different topology)
- ❌ Multi-edge fillets in one group (add multiple FILLET groups)

### Why flat faces only for MVP?
For curved faces (e.g., filleting the edge between a cylinder and a flat face):
- The setback line L₁ on the flat face is still a straight line ✓
- But the setback line on the cylinder is a helix, not a straight line ✗
- The fillet surface is no longer a simple cylinder but a canal surface
- The `FromExtrusionOf` approach breaks down (not a simple extrusion)
- Would need `MakePwlInto` approximation and curved SCurves

---

## 20. FILLET Group Type: Complete Struct Layout

For a FILLET group in memory:
```
Group {
    type    = Type::FILLET         // 5401
    h       = {group_handle}
    opA     = {source_group_h}
    name    = "fillet"
    visible = true
    color   = {inherited}
    meshCombine = CombineAs::ASSEMBLE
    
    valA    = 1.0              // fillet radius in model units (e.g., mm)
    valB    = 0.0              // unused in MVP
    
    predef.entityB = face1h   // first selected face entity
    predef.entityC = face2h   // second selected face entity
    
    // params[0] = solver param for radius (h.param(0))
    
    remap = EntityMap {
        {face1h, REMAP_FILLET_FACE} → fillet_surface_entity_h
    }
    
    // thisShell = complete filleted B-rep (copy of prevShell with fillet applied)
    // runningShell = copy of thisShell
    // displayMesh = triangulated version for rendering
}
```

---

## 21. File-Level Change Summary for FILLET

This is in addition to (or shared with) the CHAMFER changes from finding 21:

```
src/sketch.h
  :181 — Add FILLET=5401 to Group::Type enum (alongside CHAMFER=5400)
  :305 — Add REMAP_FILLET_FACE=1012 (alongside REMAP_CHAMFER_FACE=1011)

src/group.cpp
  :70  — Add case Command::GROUP_FILLET in MenuGroup()
  :XXX — Add case Type::FILLET in Generate() for AddParam

src/groupmesh.cpp
  :212 — Add else-if branch for Type::FILLET in GenerateShellAndMesh()
  :545 — Add FILLET to IsMeshGroup()

src/srf/shell.cpp (or new file src/srf/fillet.cpp)
  :NEW — Implement SShell::MakeFromFilletOf()
  :NEW — Helper: ComputeFilletArc(face1, face2, sharedEdge, r)
         → returns SBezier arc with correct weight

src/ui.h
  :XXX — Add GROUP_FILLET to Command enum
  :XXX — Add FILLET_RADIUS=804 to TextWindow::Meaning enum

src/graphicswin.cpp
  :XXX — Add menu entry for GROUP_FILLET

src/textscreens.cpp
  :394 — Add Type::FILLET branch in ShowGroupInfo()
  :XXX — Add FILLET_RADIUS handling in EditControlDone()
```

---

## 22. Differences Between CHAMFER and FILLET Implementation: Decision Points

### Decision 1: Same helper function or two separate functions?

```cpp
// Option A: Two separate functions
void SShell::MakeFromChamferOf(SShell *src, hEntity f1, hEntity f2, double d1, double d2, Group *g);
void SShell::MakeFromFilletOf(SShell *src, hEntity f1, hEntity f2, double r, Group *g);

// Option B: Combined function with type flag
void SShell::MakeFromDressUpOf(SShell *src, hEntity f1, hEntity f2, 
                                Group::Type type, double paramA, double paramB, Group *g);
```

**Recommendation**: Two separate functions. They share the edge-finding and trim-polygon-
modification code, which can be factored into a private helper. But the geometry computation
(arc weight, setback, surface creation) is different enough to warrant separate code paths.

### Decision 2: New file or extend existing?

Options:
1. Add `MakeFromChamferOf` and `MakeFromFilletOf` to `src/srf/shell.cpp`
2. Create new `src/srf/chamfer.cpp` and `src/srf/fillet.cpp`
3. Create new `src/srf/dressup.cpp` for both

**Recommendation**: New file `src/srf/dressup.cpp` containing both functions. This
mirrors how `boolean.cpp` contains all boolean operations. The CMakeLists.txt needs
one new entry.

### Decision 3: CHAMFER before FILLET?

**Yes** — implement CHAMFER first, then FILLET.
Reasoning:
1. CHAMFER has simpler geometry (planar surface, weights=1)
2. CHAMFER validates the trim polygon modification approach
3. FILLET reuses the same infrastructure

---

## 23. Sources

- `src/srf/surface.cpp:10-28` — `FromExtrusionOf` implementation (confirmed)
- `src/srf/surface.cpp:114-124` — `FromPlane` implementation (confirmed)
- `src/srf/surface.h:241-320` — SSurface struct with degm/degn, ctrl, weight, trim
- `src/srf/surface.h:161-240` — SCurve, STrimBy definitions
- AGENT.md Known Learnings — accumulated from tasks 1-21
- Finding 11 — Fillet mathematics (rolling ball, canal surface, NURBS arc weight)
- Finding 12 — Chamfer mathematics (FromPlane, bilinear patch)
- Finding 14 — GitHub community requests (phkahler PR #1501 analysis)
- Finding 16 — Boolean pipeline (MakeFromBoolean crash with coincident faces)
- Finding 17 — Triangulation (UvGridTriangulateInto for degree-2 surfaces)
- Finding 21 — Chamfer group design (complete reference for parallel structure)

---

## Key Takeaways

1. **FILLET group mirrors CHAMFER group exactly** in terms of Group struct usage, MenuGroup handler, GenerateShellAndMesh hook, and IsMeshGroup
2. **Core difference**: Fillet surface = `FromExtrusionOf(&arc, t0, t1)` (degm=2, degn=1 cylinder) vs Chamfer = `FromPlane()` (degm=1, degn=1)
3. **The arc weight formula**: `arc_weight = sin(exterior_dihedral_angle / 2)` — exact rational NURBS
4. **Setback distance**: `r / tan(exterior_dihedral_angle / 2)` — how far to set back each face
5. **Both bypass the boolean pipeline** — use direct topology injection (MakeFromCopyOf + surgical trim update)
6. **Implement CHAMFER first** — validates the infrastructure; FILLET adds the arc weight math
7. **MVP constraint**: flat faces only, constant radius, convex edges only, no vertex caps
8. **Triangulation and STEP export are FREE** — `UvGridTriangulateInto` and `IsCylinder` already handle fillet surfaces
9. **FILLET is harder** due to rational weights, setback ≠ radius, G1 continuity requirements
10. **The hard part for both**: Trim polygon surgery on the two adjacent faces
