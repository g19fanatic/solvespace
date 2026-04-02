# Task 34: 3D Entity/Face/Edge Selection Mechanism in SolveSpace

## Source Files Read
- `src/drawentity.cpp` — Entity rendering and Bezier curve generation
- `src/draw.cpp` — Selection management, `HitTestMakeSelection`, `GroupSelection`
- `src/graphicswin.cpp` — Menu system, `PopulateMainMenu`, menu entry table
- `src/groupmesh.cpp` — `DrawMesh`, `IsMeshGroup`, `GenerateDisplayItems`

---

## 1. Selection Data Structures

### `GraphicsWindow::Selection` (draw.cpp:1–35)
```cpp
struct Selection {
    hEntity     entity;      // selected entity handle
    hConstraint constraint;  // selected constraint handle
    bool        emphasized;  // for highlighting
    bool Equals(Selection *b);
    bool IsEmpty();
    bool HasEndpoints();
    void Clear();
    void Draw(bool isHovered, Canvas *canvas);
};
```
- `selection` is a `List<Selection>` in GraphicsWindow
- `hover` is a single `Selection` representing the current hover
- `hoverList` is a `List<Hover>` built each mouse-move

### `GraphicsWindow::Hover` struct
```cpp
struct Hover {
    double   distance;
    int      zIndex;
    double   depth;
    Selection selection;
};
```

### `GroupSelection::gs` (draw.cpp:248–320)
Populated by `GroupSelection()`, categorizes selected entities:
```cpp
struct GroupSelection {
    int n, points, entities, faces, workplanes;
    std::vector<hEntity> point, entity, face;
    std::vector<hEntity> anyNormal, vector;
    int anyNormals, vectors, faces, lineSegments, circlesOrArcs;
    int constraints, constraintLabels, stylables;
    // ... etc
};
```
Key for chamfer/fillet: `gs.faces` count and `gs.face` vector.

---

## 2. Entity Hit-Testing: `HitTestMakeSelection` (draw.cpp:418–516)

This function runs every mouse-move to determine what the mouse is hovering over:

```cpp
void GraphicsWindow::HitTestMakeSelection(Point2d mp) {
    hoverList = {};
    
    // Phase 1: Check all entities (points, lines, arcs, etc.)
    ObjectPicker canvas = {}; // special canvas that records picks
    canvas.selRadius = 10.0;  // 10px selection radius
    canvas.point = mp;
    
    for(Entity &e : SK.entity) {
        if(!e.IsVisible()) continue;
        // Skip IMAGE entities if !showFaces
        
        // Pick by re-drawing entity into ObjectPicker
        if(canvas.Pick([&]{ e.Draw(Entity::DrawAs::DEFAULT, &canvas); })) {
            Hover hov = {};
            hov.distance = canvas.minDistance;
            hov.zIndex   = canvas.maxZIndex;
            hov.depth    = canvas.minDepth;
            hov.selection.entity = e.h;
            hoverList.Add(&hov);
        }
    }
    
    // Phase 2: Constraints (only when not in pending operation)
    for(Constraint &c : SK.constraint) { ... }
    
    // Sort by zIndex (highest first), then by distance (closest first)
    std::sort(hoverList.begin(), hoverList.end(), [](const Hover &a, const Hover &b) {
        if(a.zIndex == b.zIndex) return a.distance < b.distance;
        return a.zIndex > b.zIndex;
    });
    sel = ChooseFromHoverToSelect();
    
    // Phase 3: Face selection from triangle mesh (LOWEST PRIORITY)
    if(sel.constraint.v == 0 && sel.entity.v == 0 && showShaded && showFaces) {
        Group *g = SK.GetGroup(activeGroup);
        SMesh *m = &(g->displayMesh);
        
        uint32_t v = m->FirstIntersectionWith(mp); // ray-cast into SMesh
        if(v) {
            sel.entity.v = v;  // Face handle stored as entity handle
        }
    }
}
```

**Critical Insight**: Face selection uses ray-casting into the `displayMesh` (SMesh), NOT into the B-rep (SShell). The return value `v` is a `uint32_t` that is the face entity handle (`tr.meta.face` from the STriangle's metadata).

---

## 3. Face Selection: How Faces are Identified

### How faces get their `uint32_t` ID
During `GenerateShellAndMesh()`, each generated surface gets a face entity:
```cpp
// In groupmesh.cpp, when generating EXTRUDE:
ss->face = Remap(e.h, REMAP_LINE_TO_FACE).v;
// REMAP_LINE_TO_FACE = 1000 (from sketch.h)
```
The `ss->face` is set on each `SSurface` object. When triangulated into `displayMesh`, each `STriangle::meta.face` gets this value.

### `SMesh::FirstIntersectionWith` (mesh.cpp:342)
Casts a ray through the screen point and returns the face entity handle from the first triangle hit:
```cpp
uint32_t SMesh::FirstIntersectionWith(Point2d mp) {
    // Casts ray from camera
    // Returns tr.meta.face for closest intersecting triangle
    return tr.meta.face;  // This is the hEntity.v for the face
}
```

### `Entity::IsFace()` (entity.cpp or sketch.h)
Returns true for these entity types:
- `FACE_NORMAL_PT`
- `FACE_XPROD`
- `FACE_N_ROT_TRANS`
- `FACE_N_TRANS`
- `FACE_N_ROT_AA`
- `FACE_ROT_NORMAL_PT`
- `FACE_N_ROT_AXIS_TRANS`

---

## 4. Face Rendering: `Group::DrawMesh` (groupmesh.cpp:560–640)

```cpp
void Group::DrawMesh(DrawMeshAs how, Canvas *canvas) {
    switch(how) {
        case DrawMeshAs::DEFAULT:
            canvas->DrawMesh(displayMesh, hcfFront, hcfBack);
            break;
            
        case DrawMeshAs::HOVERED: {
            std::vector<uint32_t> faces;
            hEntity he = SS.GW.hover.entity;
            if(he.v != 0 && SK.GetEntity(he)->IsFace()) {
                faces.push_back(he.v);  // Push face handle
            }
            canvas->DrawFaces(displayMesh, faces, hcf);  // Highlight triangles with this face handle
            break;
        }
        
        case DrawMeshAs::SELECTED: {
            std::vector<uint32_t> faces;
            SS.GW.GroupSelection();
            auto const &gs = SS.GW.gs;
            // Push all selected face handles
            for(hEntity he : gs.face) faces.push_back(he.v);
            canvas->DrawFaces(displayMesh, faces, hcf);
            break;
        }
    }
}
```

`canvas->DrawFaces(mesh, faces, fill)` renders only the triangles whose `meta.face` value is in the `faces` vector, with the highlighted fill color.

---

## 5. `MakeSelected` with Face Limit (draw.cpp:183–213)

```cpp
void GraphicsWindow::MakeSelected(Selection *stog) {
    if(stog->entity.v != 0 && SK.GetEntity(stog->entity)->IsFace()) {
        // Enforce MAX_SELECTABLE_FACES limit (= 3 from ui.h)
        unsigned int c = 0;
        for(s = selection.First(); s; s = selection.NextAfter(s)) {
            hEntity he = s->entity;
            if(he.v != 0 && SK.GetEntity(he)->IsFace()) {
                c++;
                if(c >= MAX_SELECTABLE_FACES) s->tag = 1; // Deselect oldest
            }
        }
        selection.RemoveTagged();
    }
    selection.Add(stog);
}
```

**MAX_SELECTABLE_FACES = 3** (from ui.h). This is the hard limit on simultaneously selectable faces.

---

## 6. `GroupSelection()` for Face Data (draw.cpp:248–320)

After user clicks and entities are in `selection`, `GroupSelection()` categorizes them:

```cpp
void GraphicsWindow::GroupSelection() {
    gs = {};
    for(i = 0; i < selection.n; i++) {
        Entity *e = SK.entity.FindById(s->entity);
        
        // Faces (which are special, associated/drawn with triangles)
        if(e->IsFace()) {
            gs.faces++;
            gs.face.push_back(s->entity);
        }
        // ... also categorizes as points/entities/etc.
    }
}
```

**For chamfer/fillet MenuGroup() handler**: 
- Check `gs.faces == 2` (exactly 2 faces selected)
- Access face handles as `gs.face[0]` and `gs.face[1]`
- These handles identify the 2 adjacent faces to chamfer/fillet

---

## 7. Drawing in `drawentity.cpp` — Face Entities Draw Nothing

From `drawentity.cpp` line ~840:
```cpp
case Type::FACE_NORMAL_PT:
case Type::FACE_XPROD:
case Type::FACE_N_ROT_TRANS:
case Type::FACE_N_TRANS:
case Type::FACE_N_ROT_AA:
case Type::FACE_ROT_NORMAL_PT:
case Type::FACE_N_ROT_AXIS_TRANS:
    // Do nothing; these are drawn with the triangle mesh
    return;
```

Face entities themselves don't draw anything in `Entity::Draw`. They are rendered indirectly through `DrawFaces()` on the SMesh. This confirms: **face entities are purely meta-data handles**.

---

## 8. ObjectPicker Canvas

`HitTestMakeSelection` uses `ObjectPicker` (from render/render.h presumably):
- `canvas.Pick(lambda)` executes the lambda with the picker active
- Records `minDistance`, `maxZIndex`, `minDepth` for whatever was drawn
- Returns true if anything in the lambda was within `selRadius`

This means: to make a chamfer surface "hoverable" as an entity (not a face), 
it would need to go through the `Entity::Draw` path. But for face selection 
(the approach used for chamfer target), it goes through the `SMesh::FirstIntersectionWith` path.

---

## 9. Menu System in `graphicswin.cpp`

The menu table (graphicswin.cpp:42–...) is a `const MenuEntry Menu[]`:
```cpp
struct MenuEntry {
    int          level;    // 0=menubar, 1=submenu item
    const char  *label;
    Command      cmd;
    int          accel;
    MenuKind     kind;
    MenuHandler *fn;
};
```

For new group commands like CHAMFER/FILLET:
```cpp
{ 1, N_("New C&hamfer"), Command::GROUP_CHAMFER, 0, KN, mGrp },
{ 1, N_("New &Fillet"),  Command::GROUP_FILLET,  0, KN, mGrp },
```
These would go in the `New Group` submenu (where EXTRUDE, LATHE, etc. are).

The `mGrp` macro expands to `(&Group::MenuGroup)`, which handles all group creation commands.

`PopulateMainMenu()` iterates `Menu[]` and builds the platform menu — no changes needed there beyond adding entries to `Menu[]`.

---

## 10. Key Facts for Chamfer/Fillet Implementation

### Face Selection Pipeline for Chamfer Group Creation:
```
1. User selects 2 faces by clicking on solid surfaces
   → HitTestMakeSelection calls SMesh::FirstIntersectionWith(mp)
   → Returns tr.meta.face (= uint32_t face entity handle)
   → Face entity added to selection[]

2. User invokes "New Chamfer" menu
   → Group::MenuGroup(Command::GROUP_CHAMFER) called

3. In MenuGroup():
   SS.GW.GroupSelection();   // Categorize selection
   if(SS.GW.gs.faces == 2) {
       Group g = {};
       g.type = Group::Type::CHAMFER;
       g.opA = SS.GW.activeGroup;  // source group
       g.predef.entityB = SS.GW.gs.face[0];  // face 1 handle
       g.predef.entityC = SS.GW.gs.face[1];  // face 2 handle
       g.valA = 1.0 * SS.MmToLength(1);      // default 1mm offset
       SK.group.AddAndAssignId(&g);
       SS.GenerateAll();
   }
```

### No "Edge" Entity Needed:
- SolveSpace has NO hSCurve in Selection struct
- Face-pair approach is the correct architecture
- The shared SCurve between 2 faces is computed inside `Generate()` from the face handles

### Display and Selection of Chamfer Surfaces:
Chamfer surfaces added to `runningShell` → triangulated into `displayMesh` → automatically selectable as faces (if their `SSurface::face` field is set to non-zero REMAP value).

### For Chamfer Surface Selection:
```cpp
// In SShell::MakeFromChamferOf():
sNew.face = Remap(h_from_Group_predef, REMAP_CHAMFER_FACE).v;
// This gives the chamfer surface a face entity handle
// → user can hover/select it → can use it as input to another group
```

---

## 11. `IsMeshGroup()` in groupmesh.cpp:540-555

```cpp
bool Group::IsMeshGroup() {
    switch(type) {
        case Group::Type::EXTRUDE:
        case Group::Type::LATHE:
        case Group::Type::REVOLVE:
        case Group::Type::HELIX:
        case Group::Type::ROTATE:
        case Group::Type::TRANSLATE:
            return true;
        default:
            return false;
    }
}
```

CHAMFER and FILLET must be added here. Without this, `GenerateShellAndMesh()` won't be called for them.

---

## 12. showFaces Flag

`SS.GW.showFaces` (bool, in GraphicsWindow): toggles whether faces are selectable. When false:
- `HitTestMakeSelection` skips face ray-casting
- Face hit-test in line 441: `if(e.type == Entity::Type::IMAGE && !showFaces) continue;`
- Face selection at draw.cpp:495: only when `showShaded && showFaces`

For chamfer group creation, `showFaces` must be true (it's the normal mode). No code change needed here.

---

## 13. Complete Call Stack for Face Selection

```
mouse move → GraphicsWindow::MouseMoved()
  → HitTestMakeSelection(mp)  [draw.cpp:418]
    → For entities: canvas.Pick(e.Draw(DEFAULT, &canvas))  [picks wireframe/points]
    → For faces: SMesh::FirstIntersectionWith(mp)  [mesh.cpp:342]
      → ray-cast through all STriangles of displayMesh
      → return tr.meta.face (= uint32_t face entity handle)
    → hover.entity.v = v

mouse click → GraphicsWindow::MouseLeftDown()
  → MakeSelected(&hover)  [draw.cpp:183]
    → if IsFace(): enforce MAX_SELECTABLE_FACES limit
    → selection.Add(...)

user invokes "New Chamfer" → Group::MenuGroup(Command::GROUP_CHAMFER)  [group.cpp:~70]
  → SS.GW.GroupSelection()  [draw.cpp:248]
    → gs.faces = 2, gs.face[0]/[1] = face handles
  → create Group with type=CHAMFER, predef.entityB=face[0], predef.entityC=face[1]
  → SS.GenerateAll()
    → Group::Generate()  [group.cpp]
    → Group::GenerateShellAndMesh()  [groupmesh.cpp]
      → SShell::MakeFromChamferOf()  [NEW CODE]
```

---

## Summary for Implementation

| Concern | Answer |
|---------|--------|
| How does user select target faces? | Click on solid surface → `SMesh::FirstIntersectionWith` → face entity handle |
| How many faces can be selected? | MAX_SELECTABLE_FACES=3 (ui.h); chamfer needs exactly 2 |
| Where are face handles stored? | `SS.GW.gs.face[0]` and `gs.face[1]` after `GroupSelection()` |
| Where to store for chamfer group? | `g.predef.entityB` and `g.predef.entityC` |
| No changes needed to selection system? | Correct — face selection already works |
| Do FACE entities render themselves? | No, they return immediately in `Entity::Draw` |
| How are faces highlighted? | Via `canvas->DrawFaces(displayMesh, faces, hcf)` in `Group::DrawMesh` |
| Does chamfer/fillet need to be in `IsMeshGroup()`? | YES — must add CHAMFER and FILLET cases |
| Where does the new group menu entry go? | In `Menu[]` table in graphicswin.cpp (same place as EXTRUDE etc.) |
| Menu handler? | `mGrp` → `Group::MenuGroup` — add case `Command::GROUP_CHAMFER` there |

---

## Specific Code Locations

- `draw.cpp:418` — `HitTestMakeSelection` (face hit-test, entity hit-test)
- `draw.cpp:183` — `MakeSelected` (MAX_SELECTABLE_FACES enforcement)
- `draw.cpp:248` — `GroupSelection` (populates gs.face[])
- `groupmesh.cpp:540` — `IsMeshGroup` (CHAMFER/FILLET must be added)
- `groupmesh.cpp:560` — `DrawMesh` (CHAMFER/FILLET need no changes here — auto-works)
- `graphicswin.cpp:42` — `Menu[]` table (add GROUP_CHAMFER, GROUP_FILLET entries)
- `group.cpp:70` — `Group::MenuGroup` (add cases for GROUP_CHAMFER, GROUP_FILLET)
- `mesh.cpp:342` — `SMesh::FirstIntersectionWith` (returns face handle from ray-cast)
- `drawentity.cpp:840` — FACE_ entity types return immediately (draw nothing)
