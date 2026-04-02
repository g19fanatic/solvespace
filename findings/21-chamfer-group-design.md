# Finding 21: Group::Type::CHAMFER — Design

## Overview

This document synthesizes the complete design for `Group::Type::CHAMFER` in SolveSpace,
drawing on all prior findings (01–20) and direct source code inspection of:
- `src/sketch.h` (Group struct, enums, REMAP constants)
- `src/group.cpp` (MenuGroup, Generate, existing group patterns)
- `src/groupmesh.cpp` (GenerateShellAndMesh, GenerateForBoolean, IsMeshGroup)

---

## 1. Type Enum Addition

**File:** `src/sketch.h:181`

```cpp
enum class Type : uint32_t {
    DRAWING_3D                    = 5000,
    DRAWING_WORKPLANE             = 5001,
    EXTRUDE                       = 5100,
    LATHE                         = 5101,
    REVOLVE                       = 5102,
    HELIX                         = 5103,
    ROTATE                        = 5200,
    TRANSLATE                     = 5201,
    LINKED                        = 5300,
    CHAMFER                       = 5400,   // NEW
    FILLET                        = 5401,   // NEW (for task 22)
};
```

Values 5400/5401 are safely unused and fit the existing numbering scheme (5100s = geometry ops).

---

## 2. Group Field Usage for CHAMFER

The existing Group struct (src/sketch.h:165-300) has exactly the fields needed — NO new fields are required.

| Field | CHAMFER usage |
|-------|--------------|
| `opA` | `hGroup` of the source solid group (the group to chamfer) |
| `valA` | Chamfer offset distance d1 (equal-leg) in model units |
| `valB` | Chamfer offset distance d2 (unequal-leg; 0 = equal-leg) |
| `predef.entityB` | `hEntity` of face 0 (first selected face, the REMAP face entity) |
| `predef.entityC` | `hEntity` of face 1 (second selected face, adjacent to face 0) |
| `meshCombine` | Set to `CombineAs::ASSEMBLE` (not DIFFERENCE — see section 9 below) |
| `remap` | EntityMap for stable entity IDs of generated chamfer faces/edges |
| `name` | Default: `"chamfer"` |
| `visible` | `true` |
| `color` | Inherited from source group |

### Why NOT DIFFERENCE for meshCombine?

**Critical decision:** phkahler's failed PR (#1501) revealed that using the standard
`MakeFromBoolean/MakeFromDifferenceOf` pipeline crashes for chamfer because the chamfer
plane is coincident with the two adjacent faces. The COINC_OPP branch in KeepEdge()
(boolean.cpp:280–315) degenerates.

**Correct approach: Direct topology injection.** The CHAMFER group's
`GenerateShellAndMesh()` should NOT call GenerateForBoolean with DIFFERENCE. Instead it
should call a new method `SShell::MakeFromChamferOf(prevShell, face1, face2, d1, d2)` that
directly modifies the B-rep by:
1. Starting from `prevg->runningShell` (copy)
2. Finding the shared SCurve between face1 and face2
3. Surgically replacing the shared edge with 2 new trim curves
4. Adding the new chamfer planar face
5. Outputting result to `thisShell`

Then `GenerateForBoolean<SShell>` is called with `CombineAs::ASSEMBLE` (simple concatenation,
no boolean logic), which just merges the shells.

**Wait — that's not right either.** The correct approach is:

The chamfer group builds `thisShell` as the COMPLETE modified solid (a copy of the
previous running shell with chamfer applied directly). Then `runningShell = thisShell`
(no boolean needed, since chamfer is a DressUp/in-place operation, not adding a new
separate solid). This is different from EXTRUDE which adds a new primitive.

**CHAMFER Paradigm:**
- CHAMFER group takes prevg->runningShell
- Produces a modified version of it in thisShell
- runningShell = thisShell (same shell, chamfered)
- meshCombine = ASSEMBLE (but the assembly just copies thisShell, since prevShell is empty)

More precisely, the CHAMFER group should NOT contribute to `runningShell` via boolean
pipeline at all. Instead `runningShell` should be set directly to the chamfered result.

Looking at `groupmesh.cpp:213-400`, the GenerateShellAndMesh() function follows this pattern:
```
if (type == EXTRUDE) { thisShell = extrusion of opA sketch }
// ...
prevg = srcg->RunningMeshGroup()
GenerateForBoolean(prevg->runningShell, thisShell, runningShell, meshCombine)
```

For CHAMFER, the cleanest approach is:
- `thisShell` = empty (no new primitive)
- Build chamfered result directly into `runningShell` by copying prevg->runningShell and
  surgically modifying it inline

Since GenerateForBoolean copies prevs to outs when thiss is empty (`if thiss->IsEmpty()`),
a chamfer group can intercept AFTER the copy:
```
// After GenerateForBoolean copies prevs to runningShell when thisShell is empty...
// ...chamfer modifies runningShell in-place
```

OR more cleanly: Skip GenerateForBoolean entirely for CHAMFER/FILLET and do:
```cpp
} else if(type == Type::CHAMFER) {
    Group *prevg = PreviousGroup()->RunningMeshGroup();
    runningShell.MakeFromChamferOf(&(prevg->runningShell),
        predef.entityB, predef.entityC,
        SK.GetParam(h.param(0))->val,   // d1
        SK.GetParam(h.param(1))->val,   // d2 (0 = equal-leg)
        this);
    return; // skip GenerateForBoolean
}
```

This is analogous to how TRANSLATE/ROTATE handle their own `srcg` directly.

---

## 3. REMAP Constants

**File:** `src/sketch.h:305`

```cpp
enum {
    REMAP_LAST             = 1000,
    REMAP_TOP              = 1001,
    REMAP_BOTTOM           = 1002,
    REMAP_PT_TO_LINE       = 1003,
    REMAP_LINE_TO_FACE     = 1004,
    REMAP_LATHE_START      = 1006,
    REMAP_LATHE_END        = 1007,
    REMAP_PT_TO_ARC        = 1008,
    REMAP_PT_TO_NORMAL     = 1009,
    REMAP_LATHE_ARC_CENTER = 1010,
    REMAP_CHAMFER_FACE     = 1011,   // NEW: face entity for chamfer surface
    REMAP_FILLET_FACE      = 1012,   // NEW: face entity for fillet surface (task 22)
};
```

The chamfer face surface gets `face = Remap(predef.entityB, REMAP_CHAMFER_FACE).v`
so that it has a stable, selectable face entity.

---

## 4. Parameter Setup in Generate()

**File:** `src/group.cpp` — `Group::Generate()` function

For CHAMFER, the Generate() function only needs ONE parameter: the chamfer distance(s).
These are stored as simple `valA`, `valB` (not solver params with equations), because:
1. Chamfer distance is NOT constrained by the solver — it's a direct user value
2. Only EXTRUDE uses a vector param because the vector direction can be constrained
3. HELIX uses params because pitch/angle can be constrained

```cpp
case Type::CHAMFER:
    AddParam(param, h.param(0), valA);  // d1 (distance on face 1)
    AddParam(param, h.param(1), valB);  // d2 (distance on face 2; 0 = equal)
    break;
```

Wait — should chamfer distance be a solver param at all? Looking at how EXTRUDE works:
- EXTRUDE adds 3 params (x,y,z of extrusion vector) which CAN be constrained
- The depth is encoded in the magnitude of the vector
- These are actual solver variables subject to Newton-Raphson

For chamfer: The offset distance doesn't need to be a solver variable. It's similar to
`Group::scale` (a stored value, not a param). However, having it as a param ALLOWS
future dimensioned constraints like "this chamfer is 3mm".

**Decision for MVP:** Make d1 a solver param (param 0) so it CAN be constrained later.
d2 for unequal-leg can be param 1. For equal-leg MVP, valB=0 and param 1 is unused.

```cpp
// In Group::Generate(), add case for CHAMFER:
case Type::CHAMFER:
    // param(0) = offset distance d1
    AddParam(param, h.param(0), valA);
    // param(1) = offset distance d2 (0 if equal-leg)
    if(valB > 0) {
        AddParam(param, h.param(1), valB);
    }
    // No entity copies needed (unlike EXTRUDE which copies the sketch entities)
    break;
```

---

## 5. MenuGroup() Handler

**File:** `src/group.cpp:70`

```cpp
case Command::GROUP_CHAMFER: {
    // Require the active group to be a mesh group (solid)
    Group *srcg = SK.GetGroup(SS.GW.activeGroup);
    if(!srcg->IsMeshGroup()) {
        Error(_("Chamfer can only be applied to solid groups. "
                "The active group must produce a solid (extrude, lathe, etc.).\n"));
        return;
    }
    // Require selection of exactly 2 adjacent faces
    SS.GW.GroupSelection();
    auto const &gs = SS.GW.gs;
    if(gs.faces != 2 || gs.n != 2) {
        Error(_("Chamfer requires exactly 2 adjacent faces to be selected. "
                "Click to select one face, then Ctrl+click the adjacent face.\n"));
        return;
    }

    g.type = Type::CHAMFER;
    g.opA  = SS.GW.activeGroup;
    g.predef.entityB = gs.face[0];   // face handle 1
    g.predef.entityC = gs.face[1];   // face handle 2
    g.valA = SS.GW.SS.tangentArcRadius; // reuse as default, or a sensible default
    // Better: use 1mm as default, shown in textscreen
    g.valA = 1.0;  // 1mm default chamfer distance
    g.valB = 0.0;  // equal-leg (d1 == d2)
    g.meshCombine = CombineAs::ASSEMBLE;
    g.name = C_("group-name", "chamfer");
    break;
}
```

**Face selection validation in MenuGroup:**
- `gs.faces` is the count of selected face entities
- `gs.face[0]` and `gs.face[1]` are the hEntity handles of the two faces
- We should verify the two faces are from the same source group (`opA`) and share an edge
- Edge validation can be deferred to Generate() time (report booleanFailed if invalid)

---

## 6. IsMeshGroup() Addition

**File:** `src/groupmesh.cpp:545` (approx)

```cpp
bool Group::IsMeshGroup() {
    switch(type) {
        case Type::EXTRUDE:
        case Type::LATHE:
        case Type::REVOLVE:
        case Type::HELIX:
        case Type::LINKED:
        case Type::CHAMFER:    // NEW
        case Type::FILLET:     // NEW (task 22)
            return true;
        default:
            return false;
    }
}
```

---

## 7. The Core Geometry: SShell::MakeFromChamferOf()

**New function to add in:** `src/srf/shell.cpp` (or `src/srf/boolean.cpp`)

```cpp
void SShell::MakeFromChamferOf(SShell *src,
                                hEntity face1h, hEntity face2h,
                                double d1, double d2,
                                Group *g);
```

**Algorithm:**

```
1. Start: copy src into *this (MakeFromCopyOf)
2. Find SSurface* surf1 = find surface in this->surface where ss.face == face1h.v
3. Find SSurface* surf2 = find surface in this->surface where ss.face == face2h.v
4. Verify surf1 and surf2 are planar (degm==1 && degn==1 or degree-1 polynomial)
   - GetNormal() at center; check IsSurface(PLANE) or similar
5. Find shared SCurve: iterate this->curve, find sc where
   (sc.surfA==surf1->h && sc.surfB==surf2->h) ||
   (sc.surfA==surf2->h && sc.surfB==surf1->h)
   - This is the edge to be chamfered
6. Get edge geometry:
   - edge start point: sc.pts.l.First()->p
   - edge end point: sc.pts.l.Last()->p
   - edge tangent direction: t = (end - start).WithMagnitude(1)
7. Get face normals:
   - n1 = surf1->NormalAt(midUV of surf1)  [normal pointing outward]
   - n2 = surf2->NormalAt(midUV of surf2)
8. Compute chamfer geometry:
   - inward1 = t.Cross(n1).WithMagnitude(1)  // inward direction on face 1
   - inward2 = n2.Cross(t).WithMagnitude(1)  // inward direction on face 2
   - For each endpoint P of the shared edge:
     A = P + inward1.ScaledBy(d1)   // offset point on face 1
     B = P + inward2.ScaledBy(d2 > 0 ? d2 : d1)  // offset point on face 2
   - A0, A1 = offset points on face 1 (start and end of edge)
   - B0, B1 = offset points on face 2
9. Create chamfer surface:
   SSurface chamferSurf = SSurface::FromPlane(A0, A1-A0, B0-A0);
   chamferSurf.face = g->Remap(face1h, REMAP_CHAMFER_FACE).v;
   this->surface.Add(&chamferSurf);
10. Create 2 new SCurves (new trim edges):
    - sc_new1: straight line A0 → A1 on face1 / chamferSurf
    - sc_new2: straight line B0 → B1 on face2 / chamferSurf
11. Update trim curves on surf1:
    - Remove the old shared SCurve from surf1's trim polygon
    - Add sc_new1 to surf1's trim polygon (clipping the face at the chamfer line)
12. Update trim curves on surf2:
    - Remove the old shared SCurve from surf2's trim polygon
    - Add sc_new2 to surf2's trim polygon
13. Remove the old shared SCurve from this->curve (it no longer separates surf1/surf2)
14. Add the 2 new SCurves and the chamfer surface's own trim curves
15. Validate watertightness: each SCurve referenced by exactly 2 surfaces
```

**NURBS surface details:**
- Chamfer surface = `SSurface::FromPlane(A0, A1-A0, B0-A0)` — degree (1,1), all weights=1
- This uses the existing factory method at `src/srf/surface.cpp:114`
- The "u" direction goes along the edge (A0→A1), "v" direction goes across (A0→B0)

---

## 8. GenerateShellAndMesh() Addition

**File:** `src/groupmesh.cpp`

Add to the if-else chain in `Group::GenerateShellAndMesh()` (around line 212):

```cpp
} else if(type == Type::CHAMFER) {
    double d1 = SK.GetParam(h.param(0))->val;
    double d2 = (valB > 0) ? SK.GetParam(h.param(1))->val : d1;

    // Get source solid shell from the referenced group
    Group *src = SK.GetGroup(opA);
    SShell *prevShell = &(src->runningShell);

    if(prevShell->IsEmpty()) {
        // Source has no solid — no chamfer possible
        return;
    }

    // Build the chamfered shell
    thisShell.MakeFromChamferOf(prevShell,
                                 predef.entityB,
                                 predef.entityC,
                                 d1, d2, this);
    // Note: DO NOT go through GenerateForBoolean for CHAMFER
    // The chamfer IS the runningShell (complete modified solid)
    runningShell.MakeFromCopyOf(&thisShell);

    if(meshCombine != CombineAs::ASSEMBLE) {
        runningShell.MergeCoincidentSurfaces();
    }
    displayDirty = true;
    return;  // Skip GenerateForBoolean at the bottom of the function
}
```

This needs to be placed BEFORE the `GenerateForBoolean` call at the bottom
of `GenerateShellAndMesh()`. The early return ensures the normal boolean pipeline
is bypassed.

---

## 9. Key Invariant: Chamfer vs. Boolean Pipeline

The CHAMFER group is fundamentally different from EXTRUDE/LATHE/REVOLVE:

| Feature | EXTRUDE | CHAMFER |
|---------|---------|---------|
| Creates a new primitive? | Yes (extrusion solid) | No — modifies existing solid |
| Uses boolean pipeline? | Yes (union/diff) | No — direct topology injection |
| thisShell contains... | The new primitive | The complete modified solid |
| runningShell = ... | Boolean of prev+this | Copy of this (complete) |
| meshCombine field | UNION (default) | ASSEMBLE (but bypassed) |

---

## 10. Face Handle Lookup Logic

**Problem:** `predef.entityB` stores the hEntity of a face. Face entities are virtual
entities — they're not in `SK.entity` directly. They are generated during
`GenerateShellAndMesh` and stored as `ss->face` in SSurface objects.

**How to find the SSurface from a face hEntity:**

```cpp
SSurface *Group::FindFaceByHandle(SShell *shell, hEntity faceH) {
    for(SSurface &ss : shell->surface) {
        if(ss.face == faceH.v) return &ss;
    }
    return nullptr;
}
```

This search requires that the face handle stored at group creation time (in MenuGroup)
matches the face handle that will be regenerated. This is ensured by the REMAP mechanism:
the face entity ID is computed from the source SCurve handle via `Remap(curveH, REMAP_LINE_TO_FACE)`.
As long as the source group regenerates the same SCurve with the same handle, the face
will have the same ID.

---

## 11. Edge Validation and Error Handling

```cpp
// In MakeFromChamferOf or in GenerateShellAndMesh:
if(surf1 == nullptr || surf2 == nullptr) {
    // Selected face entity not found — report error
    booleanFailed = true;
    return;
}
if(sharedCurve == nullptr) {
    // Faces are not adjacent — no shared edge
    booleanFailed = true;
    Error(_("Chamfer requires two adjacent (edge-sharing) faces."));
    return;
}
if(!surf1->IsFlat() || !surf2->IsFlat()) {
    // MVP: only flat faces supported
    booleanFailed = true;
    Error(_("Chamfer MVP only supports flat (planar) faces."));
    return;
}
if(d1 <= 0 || (valB > 0 && d2 <= 0)) {
    booleanFailed = true;
    return;
}
```

---

## 12. Textscreen UI (src/textscreens.cpp)

**File:** `src/textscreens.cpp:394`

The `ShowGroupInfo()` if-chain needs a CHAMFER branch:

```cpp
} else if(g->type == Group::Type::CHAMFER) {
    Printf(true, "%Ft chamfer group");
    Printf(false, "%Ba   source: %s", SK.GetGroup(g->opA)->DescriptionString());
    Printf(false, "%Bd   offset: %s",
        SS.MmToString(SK.GetParam(g->h.param(0))->val).c_str());
    // Show edit button for the offset
    Printf(false, "%Ba   [%f%D change offset%E]",
        &TextWindow::ScreenEditGroupName, g->h.v);
}
```

Also need to add CHAMFER to the Edit::Meaning enum in `src/ui.h`:
```cpp
// In struct TextWindow:
enum class Meaning : uint32_t {
    // ... existing ...
    HELIX_PITCH    = 802,
    CHAMFER_OFFSET = 803,  // NEW
    FILLET_RADIUS  = 804,  // NEW (task 22)
};
```

---

## 13. Command Enum Addition

**File:** `src/ui.h` — Command enum under `// Group` comment

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
GROUP_CHAMFER,  // NEW
GROUP_FILLET,   // NEW (task 22)
```

---

## 14. Menu Entry

**File:** `src/graphicswin.cpp` — the static `MenuEntry` array

Add near the existing group operations:
```cpp
{ 3, "New Chamfer Group",   Command::GROUP_CHAMFER, 0,          &Group::MenuGroup },
{ 3, "New Fillet Group",    Command::GROUP_FILLET,  0,          &Group::MenuGroup },
```

---

## 15. CHAMFER vs. Prior Group Types: What's Different

| Property | EXTRUDE | LATHE | CHAMFER |
|----------|---------|-------|---------|
| Needs active workplane? | YES | YES | NO |
| Input selection | activeGroup's sketch | axis vector + point | 2 faces from mesh group |
| Creates new sketch entities? | Yes (copy of sketch) | Yes (circles) | NO |
| Generates entities? | Yes (extrusion lines) | Yes (lathe circles) | NO |
| Boolean pipeline? | UNION/DIFF | UNION/DIFF | None (in-place) |
| Param count | 3 (vector x,y,z) | 0 | 1 or 2 (d1, d2) |
| Source group type | DRAWING_2D | DRAWING_2D | Any IsMeshGroup |

---

## 16. Differences from FILLET (Task 22 Preview)

| Property | CHAMFER | FILLET |
|----------|---------|--------|
| Surface type | Planar (degree 1,1) | Cylindrical (degree 2,1) |
| Parameter | offset distance d | radius r |
| G-continuity | G0 (sharp joins) | G1 (tangent joins) |
| Vertex caps | Triangular planar faces | Spherical patches (complex) |
| Exact NURBS? | Yes (all weights=1) | Yes (weights=cos(θ/2)) |
| Implementation difficulty | Lower | Higher |
| MVP priority | First | Second |

---

## 17. File-Level Change Summary for CHAMFER

```
src/sketch.h
  :181 — Add CHAMFER=5400, FILLET=5401 to Group::Type enum
  :305 — Add REMAP_CHAMFER_FACE=1011, REMAP_FILLET_FACE=1012

src/group.cpp
  :70  — Add case Command::GROUP_CHAMFER in MenuGroup()
  :XXX — Add case Type::CHAMFER in Generate() for AddParam

src/groupmesh.cpp
  :212 — Add else-if branch for Type::CHAMFER in GenerateShellAndMesh()
  :545 — Add CHAMFER/FILLET to IsMeshGroup()

src/srf/shell.cpp (or new file src/srf/chamfer.cpp)
  :NEW — Implement SShell::MakeFromChamferOf()

src/ui.h
  :XXX — Add GROUP_CHAMFER, GROUP_FILLET to Command enum
  :XXX — Add CHAMFER_OFFSET=803, FILLET_RADIUS=804 to TextWindow::Meaning enum

src/graphicswin.cpp
  :XXX — Add menu entries for GROUP_CHAMFER, GROUP_FILLET

src/textscreens.cpp
  :394 — Add Type::CHAMFER branch in ShowGroupInfo()
```

---

## 18. Key Insights from Community Research

From phkahler's PR #1501 (the only prior attempt):

1. **Hook point is Group::Generate()** — confirmed by the PR code
2. **Crashes were from standard boolean pipeline** — confirmed, hence direct injection approach
3. **"Limit to flat surfaces only"** — phkahler's own recommendation for MVP
4. **Two NEW SCurves per edge** — each new trim line is an SCurve shared between the
   original face (now truncated) and the new chamfer face
5. **Watertightness is the main challenge** — endpoints of new SCurves must exactly match

---

## 19. MVP Scope for CHAMFER (Confirmed Minimal Feature Set)

For a working MVP:
- Equal-leg chamfer only (d1=d2)
- Single edge (two adjacent faces at a time)
- Flat faces only (no curved surface chamfers)
- No vertex cap for 3-edge meetings (just leave open or skip)
- No constraint-driven chamfer (valA is a number, not a constrained param initially)

This is exactly what phkahler recommended and what FreeCAD's Part::Chamfer calls
"Equal distance" mode.

---

## Sources

- `src/sketch.h` pages 2-5 (direct inspection)
- `src/group.cpp` pages 1-4 (direct inspection)
- `src/groupmesh.cpp` pages 1-5 (direct inspection)
- AGENT.md Known Learnings (accumulated from findings 01-20)
- GitHub PR #1501 by phkahler (findings/14-github-community-requests.md)
- Finding 12: chamfer math (SSurface::FromPlane, inward vectors)
- Finding 16: boolean pipeline analysis
- Finding 18: edge selection via face pair
