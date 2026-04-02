# Task 18: Edge Selection UX in Parametric CAD Systems

## Overview

This research investigates how parametric CAD systems handle "edge selection" for
chamfer/fillet operations — specifically how a user selects which edges of a solid
get modified, and how SolveSpace's existing selection infrastructure can support this.

---

## 1. FreeCAD Edge Selection UX (Reference Implementation)

### Part::Chamfer / Part::Fillet Flow
From the FreeCAD wiki (fetched directly: wiki.freecad.org/Part_Chamfer):

1. **Pre-select** edges in the 3D View before invoking the command (optional).
2. Invoke the command via menu: **Part → Chamfer**.
3. A **Task Panel** (modal-like side panel) opens with:
   - A "Selected Shape" dropdown (choose the base solid)
   - A list of edges with checkboxes for each edge to chamfer
   - A size/distance input field (equal-distance) or two fields (two-distance mode)
4. User can **select by edge** or **select by face** (which selects all bordering edges).
5. Press **OK** to apply.

### Key UX Insight from FreeCAD
- Edge selection is presented as a **list-based checkbox dialog**, not real-time 3D picking.
- This avoids the "edge identity" instability problem by presenting edges as an indexed
  list from the current shape topology.
- After applying, the stored references are edge *indices*, which are fragile if topology
  changes (known Toponaming problem solved in FreeCAD only in 2024).

### FreeCAD Fillet (wiki.freecad.org/Part_Fillet)
- Same workflow: optional pre-select edges → invoke → task panel with edge list → radius input.
- No face reference needed (fillet is symmetric).

### FreeCAD PartDesign::Chamfer / PartDesign::Fillet
- The PartDesign variants operate on the **current body** and apply as a "DressUp" operation.
- User selects edges directly in the 3D view via click/hover, then confirms.
- Parameters: radius (fillet) or distance(s) (chamfer) — stored as feature parameters.
- Multiple edges can be added/removed from the edge list interactively.

---

## 2. SolveSpace Existing Selection Mechanism (Source Code Analysis)

### 2.1 Selection Infrastructure (`src/draw.cpp`, `src/ui.h`)

SolveSpace's selection system has two tiers:
- **`Selection` struct** (ui.h:750): holds `hEntity entity` + `hConstraint constraint` + `bool emphasized`
- **`Hover` struct** (ui.h:767): holds `zIndex`, `distance`, `depth`, and a `Selection`

Key fields in `GraphicsWindow` (ui.h:775-780):
```cpp
List<Hover>     hoverList;       // candidates under cursor
Selection       hover;           // current hover (single best candidate)
List<Selection> selection;       // all selected items
```

### 2.2 The Hit-Test Pipeline (`draw.cpp:420`)

`GraphicsWindow::HitTestMakeSelection(Point2d mp)`:
1. Creates an `ObjectPicker` canvas with `selRadius=10.0` pixels
2. Iterates ALL `SK.entity` — calls `e.Draw(DrawAs::DEFAULT, &canvas)` 
3. `canvas.Pick()` returns true if the entity was drawn within 10px of cursor
4. Adds `Hover{distance, zIndex, depth, entity.h}` to `hoverList`
5. Also iterates `SK.constraint` for constraint picking
6. **FACE PICKING**: as the last step, if no entity was found, calls
   `m->FirstIntersectionWith(mp)` on the display mesh (triangle intersection)
7. Calls `ChooseFromHoverToSelect()` to pick the best candidate (by zIndex/depth)
8. Sets `hover = sel` if changed

### 2.3 Face Picking via Triangle Mesh (`mesh.cpp:342`)

```cpp
uint32_t SMesh::FirstIntersectionWith(Point2d mp) const {
    uint32_t face = 0;
    double faceT = VERY_NEGATIVE;
    for(const STriangle &tr : l) {
        if(tr.meta.face == 0) continue;  // face==0 means unidentified
        // ray-triangle intersection with screen ray
        if(t > faceT) {
            face = tr.meta.face;
            faceT = t;
        }
    }
    return face;  // returns hEntity.v value encoded in SSurface::face
}
```

The returned `uint32_t` is the `.v` field of an `hEntity` — the face entity.
This is placed directly into `sel.entity.v`:
```cpp
sel.entity.v = v;   // the face hEntity value
```

### 2.4 Face Entities (`entity.cpp:708`, `sketch.h:440-446`)

Face entity types:
```cpp
FACE_NORMAL_PT         = 5000,  // flat extruded top/bottom faces
FACE_XPROD             = 5001,  // lathe faces
FACE_N_ROT_TRANS       = 5002,  // translate group faces
FACE_N_TRANS           = 5003,  // translate group side faces
FACE_N_ROT_AA          = 5004,  // rotate group faces
FACE_ROT_NORMAL_PT     = 5005,  // revolve group
FACE_N_ROT_AXIS_TRANS  = 5006,  // helix group
```

`Entity::IsFace()` returns true for all of these.

These entities carry:
- A normal vector (for plane constraint targets)
- A point on the face
- A reference to the group that created them

The face entities are **NOT drawn directly** in `drawentity.cpp` (fall-through case — "Do nothing; drawn with the triangle mesh"). They appear only via the mesh hit-test path.

### 2.5 How Face Entities Are Created (`groupmesh.cpp:262-315`)

During `GenerateShellAndMesh()` for EXTRUDE groups:
```cpp
// Assign face entity to each surface so it can be selected with mouse
for(i = is; i < thisShell.surface.n; i++) {
    SSurface *ss = &(thisShell.surface[i]);
    hEntity face = Entity::NO_ENTITY;
    
    if(top_face_condition) {
        face = Remap(Entity::NO_ENTITY, REMAP_TOP);
        ss->face = face.v;
    } else if(bottom_face_condition) {
        face = Remap(Entity::NO_ENTITY, REMAP_BOTTOM);
        ss->face = face.v;
    } else {
        // Match line_segment to extruded face
        face = Remap(e.h, REMAP_LINE_TO_FACE);
        ss->face = face.v;
    }
}
```

The `face` field in `SSurface` stores the entity handle as `uint32_t`.
During triangulation, each `STriangle` gets `meta.face = ss->face`, propagating the
face identity to every triangle of that surface.

### 2.6 REMAP Constants (`sketch.h:305-315`)

```cpp
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
// Next available: 1011
```

For chamfer/fillet faces:
- `REMAP_CHAMFER_FACE = 1011`
- `REMAP_FILLET_FACE  = 1012`

### 2.7 How GroupSelection Works (`draw.cpp:244-315`)

`GraphicsWindow::GroupSelection()` classifies selected items:
- `gs.faces` / `gs.face[]` — the face entities
- `gs.entities` / `gs.entity[]` — non-point, non-face entities
- `gs.points` / `gs.point[]` — point entities
- `gs.n` — total count

Face selection is limited to `MAX_SELECTABLE_FACES = 3` simultaneously.

### 2.8 DrawMesh Hover/Selection Highlighting (`groupmesh.cpp:560+`)

```cpp
case DrawMeshAs::HOVERED: {
    Canvas::Fill fill = {};
    fill.color = Style::Color(Style::HOVERED);
    fill.pattern = Canvas::FillPattern::CHECKERED_A;
    fill.zIndex = 2;
    Canvas::hFill hcf = canvas->GetFill(fill);

    std::vector<uint32_t> faces;
    hEntity he = SS.GW.hover.entity;
    if(he.v != 0 && SK.GetEntity(he)->IsFace()) {
        faces.push_back(he.v);
    }
    canvas->DrawFaces(displayMesh, faces, hcf);
    break;
}
```

This calls `DrawFaces(mesh, faceIds, fillStyle)` which renders only the triangles
whose `meta.face` matches one of the provided IDs — giving the hover highlight
to the exact face under the mouse.

---

## 3. The Edge Selection Problem for Chamfer/Fillet

### 3.1 What Is an "Edge" in SolveSpace?

In SolveSpace, edges are NOT first-class entities in the B-rep sense. They are:
- **`SCurve`** objects in `SShell` (src/srf/surface.h:191-230) — boundary curves between surfaces
- **`SEdge`** objects in `SEdgeList` (polygon.h) — for display/analysis purposes
- Both have `surfA` and `surfB` fields identifying the two adjacent surfaces

Edges do NOT have an `hEntity` handle — they are not in `SK.entity` — so they **cannot
be selected using the existing selection mechanism** as-is.

### 3.2 How Currently-Used Face Selection Would Extend to Edge Selection

The current system can identify FACES via `SSurface::face` → `meta.face` → hit-test.
For chamfer/fillet, we need to identify an EDGE (the boundary between two faces).

**Option A: Use Face-Pair Selection** (recommended for MVP)
- User selects **two adjacent faces** (already possible with `MAX_SELECTABLE_FACES=3`)
- The shared SCurve between them IS the edge
- The chamfer/fillet group takes `gs.face[0]` and `gs.face[1]` as its input
- The shared edge is computed by `FindSharedEdge(face1, face2)` in geometry code
- Stored in group as `predef.entityB` and `predef.entityC` (face handles)

**Option B: Add Edge Entities (similar to face entities)**
- Define a new entity type `EDGE_BETWEEN_FACES` (similar to FACE_NORMAL_PT)
- Store edge identity in `SCurve::edgeEntity` (new uint32_t field)
- Extend `HitTestMakeSelection` to test proximity to edge lines
- Requires drawing edge lines as selectable entities
- More complex; needed for multi-edge chamfer

**Option C: Edge as SCurve Index (fragile, phkahler's known approach)**
- Store the SCurve index directly as an integer
- Known to be topologically unstable (changes when model regenerates)
- NOT recommended

### 3.3 Recommended Approach for MVP: Face-Pair → Edge Derivation

**Workflow:**
1. User hovers over face → face entity highlights (existing)
2. User clicks face A → goes into `selection` as face entity
3. User hovers over adjacent face B → highlights
4. User clicks face B → goes into `selection` as second face entity
5. Status bar shows: "2 faces selected — Sketch > Chamfer Edge (or Fillet Edge)"
6. User invokes menu: Group → Chamfer (or Fillet)
7. `Group::MenuGroup(Command::GROUP_CHAMFER)` reads `gs.face[0]`, `gs.face[1]`
8. Stores them in `g->predef.entityB = gs.face[0]`, `g->predef.entityC = gs.face[1]`
9. During `Generate()`, resolves the shared SCurve from the two face handles

**Finding the Shared Edge:**
```cpp
// In SShell::MakeFromChamferOf():
// Given two face entity handles (face1_v, face2_v), find the SCurve they share
SCurve *sharedCurve = nullptr;
for(SCurve &sc : runningShell.curve) {
    SSurface *sA = runningShell.surface.FindById(sc.surfA);
    SSurface *sB = runningShell.surface.FindById(sc.surfB);
    if(sA && sB &&
       ((sA->face == face1_v && sB->face == face2_v) ||
        (sA->face == face2_v && sB->face == face1_v))) {
        sharedCurve = &sc;
        break;
    }
}
```

### 3.4 Multi-Edge Chamfer (Post-MVP)

For chamfering multiple edges simultaneously:
- User selects 3+ face entities OR a special "face loop"
- Each adjacent face pair defines one edge
- Group stores a `List<hEntity>` of face pairs
- This requires extending the Group struct (beyond MVP)
- FreeCAD stores an indexed edge list; SolveSpace would store face-pair list

---

## 4. SolveSpace-Specific Constraints on Edge Selection

### 4.1 MAX_SELECTABLE_FACES Limit
Currently hard-coded to `3u` in `ui.h`:
```cpp
const unsigned MAX_SELECTABLE_FACES = 3u;
```
For chamfer MVP (one edge = two faces), this is sufficient.
For multi-edge chamfer, this limit would need to be increased.

### 4.2 Face Entity Scope
Face entities only exist in `SK.entity` while the group they belong to is active
and has been generated. The face entity for a chamfer INPUT group would be from
the source group (e.g., EXTRUDE group). This is fine — the face entities persist
in the entity list as long as the source group exists.

### 4.3 Workplane Context
Face selection in 3D mode (no active workplane) is allowed. The CHAMFER/FILLET
group would work in any context as long as the active group has a displayMesh.

### 4.4 MenuGroup() Context Check
In `Group::MenuGroup()` (group.cpp:70), before creating a new CHAMFER group:
```cpp
case Command::GROUP_CHAMFER: {
    // Require exactly 2 face entities selected
    SS.GW.GroupSelection();
    auto const &gs = SS.GW.gs;
    if(gs.faces != 2) {
        Error(_("Select two adjacent faces to chamfer their shared edge."));
        return;
    }
    // Also verify faces are from a mesh group
    // ... proceed to create CHAMFER group
}
```

---

## 5. Comparison: How Other Tools Handle This

### SolidWorks / Fusion 360 (proprietary, reference)
- Full 3D edge picking: hover over edge → edge highlights as a distinct selectable entity
- Edges are first-class "Edge" objects in the BREP, separate from faces
- This requires the BREP to maintain an edge list with stable IDs

### OpenSCAD
- No real-time edge selection — chamfer/fillet is specified parametrically as
  `minkowski()` or via third-party modules (e.g., `BOSL2`)
- No interactive geometry selection

### Blender (bevel reference)
- Edit Mode: user selects mesh edges directly (they're first-class mesh elements)
- "Bevel Weight" can be assigned to edges as a float
- Tab into Object Mode → modifier applied to weighted edges
- Blender's approach works because it has a half-edge mesh structure, not B-rep

### LibreCAD
- 2D only; chamfer/fillet applies between two 2D line segments
- Selection: click line A, click line B → operation applied at intersection

---

## 6. Summary of Key Findings

| Aspect | Finding |
|---|---|
| SolveSpace selection unit | `hEntity` — only entities in `SK.entity` can be selected |
| Faces are selectable | Yes, via `SSurface::face` → `meta.face` → ray-cast hit test |
| Edges are NOT directly selectable | No edge entities exist; edges are `SCurve` objects not in `SK.entity` |
| Best approach for chamfer edge selection | Select 2 adjacent face entities, derive shared SCurve in geometry code |
| Face hover/highlight mechanism | `DrawMesh(DrawMeshAs::HOVERED)` → `DrawFaces(mesh, faces)` with hovered face IDs |
| Face selection limit | 3 faces max (hard-coded `MAX_SELECTABLE_FACES = 3`) |
| REMAP for new face types | Next available: 1011 (CHAMFER), 1012 (FILLET) |
| GroupSelection access | `gs.face[0]`, `gs.face[1]` — face hEntity handles |
| Store in Group | `predef.entityB = gs.face[0]`, `predef.entityC = gs.face[1]` |
| How to resolve shared edge | Iterate `SShell.curve`, find SCurve where `sA->face==f1 && sB->face==f2` |
| FreeCAD approach | Modal task panel with edge checkbox list; fragile topology indices |
| MVP UX workflow | Select 2 faces → Sketch menu → Group > Chamfer |
| Multi-edge future | Extend to List of face-pairs, increase MAX_SELECTABLE_FACES |

---

## 7. Concrete Implementation Notes for `Group::MenuGroup()`

```cpp
// src/group.cpp — in Group::MenuGroup() switch statement
case Command::GROUP_CHAMFER: {
    SS.GW.GroupSelection();
    const auto &gs = SS.GW.gs;
    if(gs.faces != 2) {
        Error(_("Chamfer requires exactly 2 adjacent faces selected.\n"
                "Click two adjacent planar faces of a solid."));
        return;
    }
    // Both faces must be from the same source group
    Entity *f0 = SK.GetEntity(gs.face[0]);
    Entity *f1 = SK.GetEntity(gs.face[1]);
    if(f0->group != f1->group) {
        Error(_("Both faces must be from the same solid."));
        return;
    }
    Group g = {};
    g.type = Type::CHAMFER;
    g.opA = f0->group;           // source group
    g.predef.entityB = gs.face[0]; // first face entity
    g.predef.entityC = gs.face[1]; // second face entity
    g.valA = SS.GW.tangentArcRadius;  // reuse existing radius, or 1.0mm default
    g.meshCombine = CombineAs::DIFFERENCE;
    // ... (name, color, etc.)
    AddGroup(&g);
    break;
}
```

---

## 8. Conclusions

1. **SolveSpace cannot select edges directly** — only entities and faces via the triangle mesh.

2. **The face-pair approach is the natural fit**: Select two adjacent planar faces (already
   fully supported), derive the shared SCurve during `Generate()`. This reuses ALL existing
   selection infrastructure with zero changes to `draw.cpp` or `ui.h`.

3. **REMAP mechanism provides stable face entity IDs**: `Remap(Entity::NO_ENTITY, REMAP_LINE_TO_FACE)`
   for side faces, `REMAP_TOP`/`REMAP_BOTTOM` for end caps. These are stable across regeneration
   as long as the source group's sketch doesn't change topology.

4. **MenuGroup() validation**: Check `gs.faces == 2` and that both faces share a group.

5. **The selection UI design is: pick faces, not edges** — this is both simpler and more
   consistent with SolveSpace's existing patterns (face selection already exists for constraints).

6. **Post-MVP**: Edge entities could be added for direct edge picking, similar to how face
   entities are currently implemented. But for MVP, face-pair selection is sufficient and
   avoids all new selection infrastructure.
