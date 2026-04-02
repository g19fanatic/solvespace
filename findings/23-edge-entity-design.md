# Task 23: Edge/Entity Selection Design for Chamfer/Fillet in SolveSpace

## Overview

This document investigates how edge selection should work in SolveSpace's UI model for chamfer/fillet groups. The core question: **should edges be new Entity types, or referenced via existing hEntity/hGroup handles?**

The answer, backed by source evidence, is: **no new Entity types are needed — use the existing face-based entity system to specify the pair of adjacent faces, which implicitly defines the shared edge (SCurve).**

---

## 1. SolveSpace's Current Entity/Selection Architecture

### `GraphicsWindow::Selection` struct (`src/ui.h:750`)

```cpp
class Selection {
public:
    int         tag;
    hEntity     entity;        // handle to selected entity
    hConstraint constraint;    // handle to selected constraint
    bool        emphasized;
    ...
};
```

A `Selection` holds only an `hEntity` or `hConstraint` handle. There is no concept of a "selected SCurve" or "selected edge" at the UI level.

### `GroupSelection` accumulator (`src/ui.h:786-813`)

```cpp
struct {
    std::vector<hEntity>     point;
    std::vector<hEntity>     entity;
    std::vector<hEntity>     anyNormal;
    std::vector<hEntity>     vector;
    std::vector<hEntity>     face;    // <-- face handles
    std::vector<hConstraint> constraint;
    int         points;
    int         entities;
    int         workplanes;
    int         faces;           // count
    int         lineSegments;
    int         circlesOrArcs;
    int         arcs;
    int         cubics;
    int         periodicCubics;
    int         anyNormals;
    int         vectors;
    int         constraints;
    int         stylables;
    int         constraintLabels;
    int         withEndpoints;
    int         n;
} gs;
```

Key: `gs.face[]` is a `std::vector<hEntity>` that collects face entity handles. Face entities have type `FACE_NORMAL_PT`, `FACE_N_TRANS`, `FACE_N_ROT_AA`, etc. — see below.

### `MAX_SELECTABLE_FACES = 3` (`src/ui.h:780`)
```cpp
const unsigned MAX_SELECTABLE_FACES = 3u;
```
At most 3 faces can be selected simultaneously. For 1-edge chamfer/fillet (needing 2 adjacent faces), this is sufficient. For multi-edge chamfer/fillet, this might need increasing.

---

## 2. How Faces Are Hit-Tested and Selected

### `HitTestMakeSelection` (`src/draw.cpp:420`)

Face selection happens through `SMesh::FirstIntersectionWith()`. When no entity or constraint is under the cursor:

```cpp
// Faces, from the triangle mesh; these are lowest priority
if(sel.constraint.v == 0 && sel.entity.v == 0 && showShaded && showFaces) {
    Group *g = SK.GetGroup(activeGroup);
    SMesh *m = &(g->displayMesh);

    uint32_t v = m->FirstIntersectionWith(mp);
    if(v) {
        sel.entity.v = v;  // face handle stored as hEntity.v!
    }
}
```

`FirstIntersectionWith()` returns the `tr.meta.face` value — a `uint32_t` that is the `hEntity.v` of the face entity. So face selection works by ray-casting into the triangle mesh and returning the face entity handle encoded in each triangle's metadata.

### `SMesh::FirstIntersectionWith` (`src/mesh.cpp:342`)
```cpp
uint32_t SMesh::FirstIntersectionWith(Point2d mp) const {
    ...
    if(tr.meta.face == 0) continue;
    ...
    face = tr.meta.face;  // returns hEntity.v for the face
}
```

### `MakeSelected` face limit (`src/draw.cpp:184`)
```cpp
void GraphicsWindow::MakeSelected(Selection *stog) {
    if(stog->entity.v != 0 && SK.GetEntity(stog->entity)->IsFace()) {
        // only MAX_SELECTABLE_FACES faces may be selected at a time
        unsigned int c = 0;
        ...
        if(c >= MAX_SELECTABLE_FACES) s->tag = 1;
    }
    selection.Add(stog);
}
```

---

## 3. Entity Types for Faces

### `EntityBase::IsFace()` (`src/entity.cpp:708`)
```cpp
bool EntityBase::IsFace() const {
    switch(type) {
        case Type::FACE_NORMAL_PT:
        case Type::FACE_XPROD:         // = 5001
        case Type::FACE_N_ROT_TRANS:   // = 5002
        case Type::FACE_N_TRANS:       // = 5003
        case Type::FACE_N_ROT_AA:      // = 5004
        case Type::FACE_ROT_NORMAL_PT: // = 5005
        case Type::FACE_N_ROT_AXIS_TRANS: // = 5006
            return true;
        default:
            return false;
    }
}
```

These entity types represent the face normal for each generated solid face. They are created by `GenerateShellAndMesh()` via `Remap()` during group generation.

### Face entities created by EXTRUDE/REVOLVE/HELIX (`src/groupmesh.cpp:283-310`)
```cpp
// For top cap face (EXTRUDE):
face = Remap(Entity::NO_ENTITY, REMAP_TOP);      // top face
face = Remap(Entity::NO_ENTITY, REMAP_BOTTOM);   // bottom face
face = Remap(e.h, REMAP_LINE_TO_FACE);           // side face per sketch edge
```

Each face entity's `hEntity.v` is what gets stored in `STriangle::meta.face` — it's the "face ID" embedded in every triangle of that face.

---

## 4. How Face Handles Map to SCurves (The Edge)

### The challenge: faces don't directly encode which SCurve is their boundary

In SolveSpace's B-rep:
- Each `SSurface` has a list of `STrimBy` entries that reference `hSCurve` handles
- Each `SCurve` has `surfA` and `surfB` — the two surfaces it separates

**The connection between face entity handles and SCurves:**
- Face entity `h` → `h.v` is stored in each triangle in the SMesh for that face
- Face entity maps to an SSurface via the `SSurface::face` field (set to the same `Remap(...)` value during generation)
- To find the shared SCurve between two adjacent faces: iterate `shell.curve` and find the SCurve where `sc.surfA == faceA_surface && sc.surfB == faceB_surface` (or vice versa)

### Code for finding shared edge from two face handles (`src/groupmesh.cpp:92-103` region)

```cpp
// In Group::Generate() for CHAMFER:
hEntity face0 = g->predef.entityB; // first selected face
hEntity face1 = g->predef.entityC; // second selected face

// Get the source group's shell
Group *srcGrp = SK.GetGroup(g->opA);
SShell *sh = &srcGrp->thisShell;

// Find the SSurface for each face by matching SSurface::face field
SSurface *surfA = NULL, *surfB = NULL;
for(SSurface &ss : sh->surface) {
    if(ss.face == face0.v) surfA = &ss;
    if(ss.face == face1.v) surfB = &ss;
}

// Find the shared SCurve
SCurve *sharedCurve = NULL;
for(SCurve &sc : sh->curve) {
    if((sc.surfA == surfA->h && sc.surfB == surfB->h) ||
       (sc.surfA == surfB->h && sc.surfB == surfA->h)) {
        sharedCurve = &sc;
        break;
    }
}
```

This algorithm confirms the face-pair approach — once you have the two face handles, you can find the shared SCurve in Generate().

---

## 5. Why No New "Edge Entity" Type is Needed

### Arguments against adding an `EDGE_ENTITY` type:

1. **SCurves are B-rep topology, not sketch geometry** — they don't exist in `SK.entity`; they're in `SShell::curve`. Adding them to `SK.entity` would require major refactoring.

2. **Faces are already selectable** — the full infrastructure for face hit-testing, highlighting, selection counting, and handle storage is already implemented.

3. **Two faces uniquely determine one edge** — for the MVP (chamfer/fillet one edge between two flat adjacent faces), two face handles are sufficient.

4. **Face handles are stable across regeneration** — they're generated via `Remap()` which produces deterministic handles based on the source entity and remap constant. This is more stable than raw SCurve indices.

5. **`GroupSelection()` already tracks `gs.face[]`** — no new counting logic needed.

6. **phkahler's existing fillet branch (PR #1501)** — used the same face-pair approach.

---

## 6. Proposed Edge Identification Strategy

### For MVP (single edge, two flat faces):

**User interaction:**
1. User activates an `IsMeshGroup()` group (e.g., an EXTRUDE) as the target
2. User selects exactly 2 adjacent face entities (by clicking on solid faces in the 3D view)
3. User invokes `Group > Chamfer/Fillet` menu command

**Storage in Group:**
```cpp
// In Group struct (src/sketch.h):
// predef.entityB = hEntity of first selected face (already saved in SAVED[])
// predef.entityC = hEntity of second selected face (already saved in SAVED[])
// opA            = hGroup of source group (the one being chamfered)
// valA           = chamfer distance d (or fillet radius r)
// valB           = second distance d2 (for unequal-leg chamfer, else 0)
```

**In `Group::MenuGroup()` (`src/group.cpp:70`):**
```cpp
case Command::GROUP_CHAMFER: {
    // Verify exactly 2 faces are selected
    GroupSelection();
    if(gs.faces != 2) {
        Error("Select exactly two adjacent faces");
        return;
    }
    Group g = {};
    g.type = Group::Type::CHAMFER;
    g.opA  = SS.GW.activeGroup;  // source group
    g.predef.entityB = gs.face[0];
    g.predef.entityC = gs.face[1];
    g.valA = 1.0;  // default distance = 1mm
    g.valB = 0.0;  // equal-leg
    g.meshCombine = CombineAs::ASSEMBLE;  // or DIFFERENCE
    SK.group.AddAndAssignId(&g);
    SS.GW.AnimateOntoWorkplane();
    // ...
    break;
}
```

---

## 7. Face Selection UI Flow (Existing Infrastructure)

The complete flow for selecting faces and invoking chamfer:

```
1. Mouse click on solid surface
   → GraphicsWindow::MouseLeftDown()
   → HitTestMakeSelection()
   → SMesh::FirstIntersectionWith() returns face hEntity.v
   → sel.entity.v = face_handle
   → MakeSelected(&sel)

2. Selection count check (in MakeSelected):
   → if entity.IsFace() && count >= MAX_SELECTABLE_FACES: evict oldest

3. Highlighting of selected faces:
   → Group::DrawMesh() (DrawMeshAs::SELECTED path)
   → Renders selected triangles in highlighted color (groupmesh.cpp:560+)

4. User invokes menu: Group > Add Chamfer
   → Group::MenuGroup(Command::GROUP_CHAMFER)
   → GroupSelection()  // categorizes selection into gs.*
   → gs.faces == 2  // check passes
   → gs.face[0], gs.face[1]  // the two selected faces
```

---

## 8. Multi-Edge Strategy (Future Extension)

For multi-edge chamfer/fillet (chamfer all selected edges at once):

**Option A: Multiple face pairs stored in a list**
- Requires new storage: `List<std::pair<hEntity,hEntity>> facePairs` in Group
- Problem: Group struct has no dynamic lists currently

**Option B: Enumerate face pairs from a set of faces**
- Store up to N faces (increase `MAX_SELECTABLE_FACES`)
- During Generate(), find all shared SCurves between any pair in the face set
- More flexible but requires O(n²) edge lookup

**Option C: Add an `EDGE_ENTITY` type to expose SCurves as SK.entity items**
- Most powerful, but requires significant infrastructure work
- SCurves would need to be registered in `SK.entity` during GenerateShellAndMesh
- Requires new Entity types (e.g., `EDGE_3D = 6000`)
- Would allow "select edge" like "select face" — hit test against `SEdgeList`
- This is what parametric CAD tools (CATIA, SolidWorks, NX) do internally

**Recommendation for MVP:** Use Option A with face pairs (2 faces = 1 edge). For post-MVP, implement Option C for proper edge entities.

---

## 9. Edge Entity Type Design (Post-MVP)

If proper edge entities are implemented later, here is the proposed design:

### New entity types (`src/sketch.h`):
```cpp
// In Entity::Type enum (after existing entries):
EDGE_3D = 6000,  // A 3D edge from a solid (references an SCurve)
```

### New entity fields:
```cpp
// In EntityBase struct:
uint32_t  edgeCurve;  // hSCurve.v of the referenced SCurve
```

### Generation in `GenerateShellAndMesh`:
```cpp
// For each SCurve in the generated shell:
for(SCurve &sc : thisShell.curve) {
    Entity e = {};
    e.type = Entity::Type::EDGE_3D;
    e.group = h;
    e.h = Remap(Entity::NO_ENTITY, REMAP_EDGE_BASE + i);  // stable handle
    e.edgeCurve = sc.h.v;
    el->Add(&e);
}
```

### Hit-testing in `HitTestMakeSelection`:
```cpp
// After entity loop, before face loop:
for(Entity &e : SK.entity) {
    if(e.type != Entity::Type::EDGE_3D) continue;
    // Project SCurve polyline to screen, check proximity to mp
    ...
}
```

This would require changes to: `sketch.h`, `entity.cpp`, `groupmesh.cpp`, `draw.cpp`, `ui.h` (GroupSelection struct), and `file.cpp` (SAVED[]).

---

## 10. Summary and Recommendation

### MVP approach: **Face-pair selection using existing infrastructure**

| Component | Change Required | Notes |
|-----------|-----------------|-------|
| `ui.h` | None — `gs.face[]` already exists | MAX_SELECTABLE_FACES=3 sufficient |
| `sketch.h` | None — `predef.entityB/C` already exist for face storage | |
| `group.cpp` | Add `case Command::GROUP_CHAMFER:` in `MenuGroup()` | ~20 lines |
| `draw.cpp` | None — face hit-testing already works | |
| `mesh.cpp` | None — `FirstIntersectionWith()` already returns face handles | |
| New entity types | NOT NEEDED for MVP | Face types already exist |

### Key findings:

1. **SolveSpace has NO edge entities** — edges are `SCurve` objects in `SShell`, invisible to `SK.entity`
2. **Face selection is fully implemented** — `HitTestMakeSelection` → `FirstIntersectionWith` → `gs.face[]`
3. **Two face handles uniquely identify one edge** — find shared `SCurve` by iterating `SShell::curve`
4. **`Group::predef.entityB/C` already serialized** — no new SAVED[] entries needed
5. **`MAX_SELECTABLE_FACES = 3`** — sufficient for 1-edge chamfer (2 faces)
6. **Future post-MVP**: add `EDGE_3D` entity type for direct edge selection

### Confirmed code locations:

- `src/ui.h:750` — `Selection` class definition
- `src/ui.h:780` — `MAX_SELECTABLE_FACES = 3`
- `src/ui.h:786-813` — `GroupSelection` accumulator struct `gs`
- `src/draw.cpp:184` — `MakeSelected()` face limit enforcement
- `src/draw.cpp:250` — `GroupSelection()` face counting (`gs.faces++`, `gs.face.push_back`)
- `src/draw.cpp:420` — `HitTestMakeSelection()` — entities first, then faces via `FirstIntersectionWith`
- `src/mesh.cpp:342` — `SMesh::FirstIntersectionWith()` returns face handle
- `src/entity.cpp:708` — `IsFace()` — 7 face entity types
- `src/sketch.h:440-446` — Face entity type enum values (5000-5006)
- `src/sketch.h:310` — `REMAP_LINE_TO_FACE = 1004`
- `src/groupmesh.cpp:283-310` — Face entity creation during EXTRUDE generation
- `src/srf/surface.h:199-220` — `SCurve` struct with `surfA`, `surfB`
