# Synthesis Part 3: Proposed Architecture for Chamfer/Fillet in SolveSpace
## (Synthesizing Findings 21–35)

---

## 1. Overview

This document synthesizes the detailed research from findings 21 through 35, covering:
- Group type designs for CHAMFER (finding 21) and FILLET (finding 22)
- Edge/entity selection strategy (finding 23)
- STEP export compatibility (finding 24)
- TextWindow UI design (finding 25)
- ISO standards and annotations (finding 26)
- NURBS cylinder representation for fillet surfaces (finding 27)
- Existing implementation attempts — phkahler PR #1501 (finding 28)
- Blender bevel algorithm architecture (finding 29)
- CGAL and geometric algorithms (finding 30)
- sketch.h analysis with extension points (finding 31)
- group.cpp code path trace (finding 32)
- boolean.cpp deep dive (finding 33)
- Selection mechanism source analysis (finding 34)
- Solver vs. stored params analysis (finding 35)

---

## 2. High-Level Architecture: The "DressUp" Group Pattern

The fundamental architectural decision is:

**CHAMFER and FILLET are new Group types in SolveSpace's Group pipeline.**

They follow the "DressUp" pattern (FreeCAD terminology): a Group that takes an existing
solid as input and modifies it by applying an edge treatment operation. This is structurally
analogous to how FreeCAD's `PartDesign::Chamfer` and `PartDesign::Fillet` work — they
consume the previous feature and produce a modified solid.

### Key Architectural Decisions Made

1. **Direct topology injection** (NOT boolean pipeline): The standard `MakeFromBoolean/MakeFromDifferenceOf` cannot be used due to COINC_OPP degeneracy when the chamfer/fillet surface is coincident with adjacent faces. Confirmed by phkahler's crashed PR #1501.

2. **Face-pair selection** (NOT edge entities): No new Entity types needed for MVP. User selects 2 adjacent faces → the system derives the shared SCurve. All infrastructure already exists.

3. **Single solver param, no equations**: `AddParam(h.param(0), valA)` in `Group::Generate()`, no equations in `GenerateEquations()` → param stays free at `valA`. Same pattern as REVOLVE's angle.

4. **Two new Group types**: `CHAMFER = 5400`, `FILLET = 5401` — fit naturally after `LINKED = 5300`.

5. **No new file format entries**: All required storage is in existing `SAVED[]` fields (`valA`, `opA`, `predef.entityB/C`, `meshCombine`, `remap`).

6. **New file `src/srf/chamfer.cpp`**: Modeled after phkahler's PR #1501 structure, contains `SShell::MakeFromChamferOf()` and `SShell::MakeFromFilletOf()`.

---

## 3. Data Model Design

### Group Struct Usage (No New Fields Needed)

```
Group {
    type    = Type::CHAMFER (5400) or Type::FILLET (5401)
    h       = {group_handle}
    opA     = hGroup of source solid group
    name    = "chamfer" or "fillet"
    visible = true
    color   = {inherited from source}
    meshCombine = CombineAs::ASSEMBLE  // bypassed anyway in direct injection
    
    valA    = chamfer distance d (or fillet radius r) in model units (mm)
    valB    = second distance d2 (unequal-leg chamfer, else 0); FILLET: unused in MVP
    
    predef.entityB = hEntity of first selected face (face 0)
    predef.entityC = hEntity of second selected face (face 1)
    
    // Derived (generated every regen, not persisted):
    thisShell    = complete chamfered/filleted B-rep
    runningShell = copy of thisShell (complete modified solid)
    displayMesh  = triangulated for rendering
    booleanFailed = error flag if geometry computation fails
    
    // Solver params (generated in Generate()):
    h.param(0) = distance/radius (free param, initialized to valA)
    
    // Entity map for stable face IDs:
    remap = EntityMap { 
        {predef.entityB, REMAP_CHAMFER_FACE} → chamfer_surface_face_entity_h 
    }
}
```

### sketch.h Extensions Required

```cpp
// In Group::Type enum (~line 181):
CHAMFER = 5400,   // flat angled cut at a solid edge
FILLET  = 5401,   // rounded blend at a solid edge

// In REMAP constants (~line 305):
REMAP_CHAMFER_FACE = 1011,  // face entity for chamfer surface
REMAP_FILLET_FACE  = 1012,  // face entity for fillet surface
```

---

## 4. The Core Algorithm: SShell::MakeFromChamferOf()

This is the primary new function. It takes a source shell and surgically modifies it
to apply a chamfer between two specified faces.

### Location: `src/srf/chamfer.cpp` (new file)

```cpp
void SShell::MakeFromChamferOf(SShell *src, Group *g, double dist) {
    // STEP 1: Copy source shell (preserving original IDs)
    MakeFromCopyOf(src);  // preserves all hSSurface and hSCurve IDs
    
    // STEP 2: Find the two target surfaces by face handle
    hEntity face1h = g->predef.entityB;
    hEntity face2h = g->predef.entityC;
    
    SSurface *surf1 = nullptr, *surf2 = nullptr;
    for(SSurface &ss : surface) {
        if(ss.face == face1h.v) surf1 = &ss;
        if(ss.face == face2h.v) surf2 = &ss;
    }
    if(!surf1 || !surf2) { g->booleanFailed = true; return; }
    
    // STEP 3: Validate flat faces (MVP constraint)
    if(!surf1->IsFlat() || !surf2->IsFlat()) {
        g->booleanFailed = true;
        return; // "Chamfer MVP requires flat (planar) faces"
    }
    
    // STEP 4: Find shared SCurve (the edge to chamfer)
    SCurve *sharedSC = nullptr;
    for(SCurve &sc : curve) {
        if((sc.surfA == surf1->h && sc.surfB == surf2->h) ||
           (sc.surfA == surf2->h && sc.surfB == surf1->h)) {
            sharedSC = &sc;
            break;
        }
    }
    if(!sharedSC) { g->booleanFailed = true; return; }
    
    // STEP 5: Get edge geometry
    Vector V1 = sharedSC->pts.l.First()->p;
    Vector V2 = sharedSC->pts.l.Last()->p;
    Vector t  = V2.Minus(V1).WithMagnitude(1.0);  // edge unit tangent
    
    // STEP 6: Get face normals (outward)
    Point2d uv1, uv2;
    surf1->ClosestPointTo(V1.Plus(V2).ScaledBy(0.5), &uv1, NULL);
    surf2->ClosestPointTo(V1.Plus(V2).ScaledBy(0.5), &uv2, NULL);
    Vector n1 = surf1->NormalAt(uv1);
    Vector n2 = surf2->NormalAt(uv2);
    
    // STEP 7: Compute inward offset directions (into each face, perpendicular to edge)
    Vector d1 = t.Cross(n1).WithMagnitude(1.0);   // inward along face 1
    Vector d2 = n2.Cross(t).WithMagnitude(1.0);   // inward along face 2
    double d  = dist;  // equal-leg chamfer (d2 = d for MVP)
    
    // STEP 8: Compute 4 chamfer corner points
    Vector A = V1.Plus(d1.ScaledBy(d));  // start on face 1
    Vector B = V2.Plus(d1.ScaledBy(d));  // end on face 1
    Vector C = V2.Plus(d2.ScaledBy(d));  // end on face 2
    Vector D = V1.Plus(d2.ScaledBy(d));  // start on face 2
    
    // STEP 9: Create chamfer surface
    SSurface chamferSurf = SSurface::FromPlane(A, B.Minus(A), D.Minus(A));
    chamferSurf.face = g->Remap(g->predef.entityB, REMAP_CHAMFER_FACE).v;
    chamferSurf.color = surf1->color;  // inherit color
    hSSurface hChamfer = surface.AddAndAssignId(&chamferSurf);
    
    // STEP 10: Create 2 new contact SCurves
    //   curve1: A→B on face1 / chamfer surface
    SCurve sc1 = {};
    sc1.isExact = true;
    sc1.exact = SBezier::From(A, B);  // degree-1 line
    sc1.exact.MakePwlInto(&sc1.pts);
    sc1.surfA = surf1->h;
    sc1.surfB = hChamfer;
    hSCurve hCurve1 = curve.AddAndAssignId(&sc1);
    
    //   curve2: D→C on face2 / chamfer surface
    SCurve sc2 = {};
    sc2.isExact = true;
    sc2.exact = SBezier::From(D, C);  // degree-1 line
    sc2.exact.MakePwlInto(&sc2.pts);
    sc2.surfA = surface.FindById(hChamfer)->h;  // or: hChamfer for the new surf
    sc2.surfB = surf2->h;
    hSCurve hCurve2 = curve.AddAndAssignId(&sc2);
    
    // STEP 11: Create 2 cap SCurves at the ends (A→D and B→C)
    //   capStart: A→D (at V1 end of chamfer)
    SCurve scCapStart = {};
    scCapStart.isExact = true;
    scCapStart.exact = SBezier::From(A, D);
    scCapStart.exact.MakePwlInto(&scCapStart.pts);
    // surfA/B = chamfer surface + ??? (edge between chamfer and adjacent faces at vertex)
    // For MVP: leave unassigned; handle vertex caps as future work
    hSCurve hCapStart = curve.AddAndAssignId(&scCapStart);
    
    //   capEnd: B→C (at V2 end of chamfer)
    SCurve scCapEnd = {};
    scCapEnd.isExact = true;
    scCapEnd.exact = SBezier::From(B, C);
    scCapEnd.exact.MakePwlInto(&scCapEnd.pts);
    hSCurve hCapEnd = curve.AddAndAssignId(&scCapEnd);
    
    // STEP 12: Add trim polygon to chamfer surface
    SSurface *cSurf = surface.FindById(hChamfer);
    cSurf->trim.Add(STrimBy::EntireCurve(this, hCurve1, /*backwards=*/false));  // A→B
    cSurf->trim.Add(STrimBy::EntireCurve(this, hCapEnd, /*backwards=*/false));  // B→C
    cSurf->trim.Add(STrimBy::EntireCurve(this, hCurve2, /*backwards=*/true));   // C→D (backwards)
    cSurf->trim.Add(STrimBy::EntireCurve(this, hCapStart, /*backwards=*/true)); // D→A (backwards)
    
    // STEP 13: Update surf1 trim polygon
    //   Replace the STrimBy referencing the old shared SCurve with one for hCurve1
    for(STrimBy &stb : surf1->trim) {
        if(stb.curve == sharedSC->h) {
            // Replace: update endpoint to match setback points A,B and new curve
            stb.curve    = hCurve1;
            stb.start    = A;
            stb.finish   = B;
            // backwards flag stays the same
            break;
        }
    }
    
    // STEP 14: Update surf2 trim polygon  
    for(STrimBy &stb : surf2->trim) {
        if(stb.curve == sharedSC->h) {
            stb.curve  = hCurve2;
            stb.start  = D;   // or C depending on direction
            stb.finish = C;
            break;
        }
    }
    
    // STEP 15: Remove old shared SCurve from shell
    sharedSC->tag = 1;
    curve.RemoveTagged();
    
    // Step 16: The chamfer shell is complete
    // Watertightness check: every SCurve should be in 2 trim lists
    // (handled by AssemblePolygon in next GenerateDisplayItems)
}
```

**NOTE on vertex caps**: Steps 11 requires cap curves connecting the chamfer surface to
the adjacent faces at each vertex. For MVP, these can be left as open edges (not fully
watertight at the vertices). The `AssemblePolygon` failure at vertex gaps is deferred.

### SShell::MakeFromFilletOf() — Analogous Structure

Same structure as `MakeFromChamferOf()`, but:
1. Computes `setback = r / tan(half_angle)` where `half_angle = acos(-n1.Dot(n2))/2`
2. Computes `arc_weight = sin(half_angle)` for exact rational NURBS
3. Creates fillet arc: `SBezier arc; arc.deg=2; arc.ctrl[0]=A0; arc.ctrl[1]=E0; arc.ctrl[2]=B0; arc.weight[1]=arc_weight`
4. Creates fillet surface: `SSurface::FromExtrusionOf(&arc, zero_vec, edge_vec)` — degree (2,1) cylinder
5. Same trim polygon surgery as chamfer (Steps 12-15)

---

## 5. The GenerateShellAndMesh() Integration

### Location: `src/groupmesh.cpp` — `Group::GenerateShellAndMesh()`

Add after existing `else if` branches for EXTRUDE, LATHE, REVOLVE, HELIX, LINKED:

```cpp
} else if(type == Type::CHAMFER) {
    double dist = SK.GetParam(h.param(0))->val;
    if(dist <= 0) {
        booleanFailed = true;
        return;
    }
    
    Group *srcg = RunningMeshGroup()->PreviousGroup();
    // Actually: use PreviousGroup() for the source of the CHAMFER:
    Group *prevg = SK.GetGroup(opA);
    SShell *prevShell = &(prevg->runningShell);
    
    if(prevShell->IsEmpty()) {
        return;  // source has no solid
    }
    
    // Direct topology injection — bypass boolean pipeline
    thisShell.MakeFromChamferOf(prevShell, this, dist);
    booleanFailed = thisShell.booleanFailed;
    
    // CHAMFER is the complete modified solid — assign directly
    runningShell.MakeFromCopyOf(&thisShell);
    displayDirty = true;
    return;  // SKIP GenerateForBoolean entirely
    
} else if(type == Type::FILLET) {
    double r = SK.GetParam(h.param(0))->val;
    if(r <= 0) {
        booleanFailed = true;
        return;
    }
    
    Group *prevg = SK.GetGroup(opA);
    SShell *prevShell = &(prevg->runningShell);
    
    if(prevShell->IsEmpty()) {
        return;
    }
    
    thisShell.MakeFromFilletOf(prevShell, this, r);
    booleanFailed = thisShell.booleanFailed;
    
    runningShell.MakeFromCopyOf(&thisShell);
    displayDirty = true;
    return;  // SKIP GenerateForBoolean entirely
}
```

### IsMeshGroup() Extension

```cpp
// src/groupmesh.cpp (~line 545):
bool Group::IsMeshGroup() {
    switch(type) {
        case Type::EXTRUDE:
        case Type::LATHE:
        case Type::REVOLVE:
        case Type::HELIX:
        case Type::ROTATE:
        case Type::TRANSLATE:
        case Type::LINKED:
        case Type::CHAMFER:  // NEW
        case Type::FILLET:   // NEW
            return true;
        default:
            return false;
    }
}
```

### RunningMeshGroup() Note

`RunningMeshGroup()` returns `PreviousGroup()` for CHAMFER/FILLET (same as most groups).
This is correct — the "previous running mesh" is the cumulative solid before the chamfer.
No change needed.

---

## 6. Group::Generate() — Solver Parameter Registration

### Location: `src/group.cpp` — `Group::Generate()`

```cpp
// Add cases for CHAMFER and FILLET:
case Type::CHAMFER:
    // Single solver param: the chamfer offset distance
    // Initial value = valA (the stored/saved value from file)
    AddParam(param, h.param(0), valA);
    // No entity copies needed — geometry lives entirely in SShell
    return;

case Type::FILLET:
    // Single solver param: the fillet radius
    AddParam(param, h.param(0), valA);
    return;
```

### Group::GenerateEquations() — No Changes Needed

No equations are added for CHAMFER/FILLET. The param `h.param(0)` is fully free.
The solver sees: 1 unknown, 0 equations → 1 DOF → param remains at `valA`.
This is semantically correct: the chamfer distance IS the free degree of freedom.

---

## 7. MenuGroup() Handler

### Location: `src/group.cpp:70`

```cpp
case Command::GROUP_CHAMFER: {
    // Validate: active group must be a mesh group (solid)
    Group *srcg = SK.GetGroup(SS.GW.activeGroup);
    if(!srcg->IsMeshGroup()) {
        Error(_("Chamfer requires an active solid group (extrude, lathe, etc.)\n"));
        return;
    }
    
    // Validate: exactly 2 faces must be selected
    SS.GW.GroupSelection();
    auto const &gs = SS.GW.gs;
    if(gs.faces != 2) {
        Error(_("Chamfer requires exactly 2 adjacent faces to be selected.\n"
                "Click a face, then Ctrl+click the adjacent face.\n"));
        return;
    }
    
    // Create the CHAMFER group
    Group g = {};
    g.type = Type::CHAMFER;
    g.opA  = SS.GW.activeGroup;
    g.predef.entityB = gs.face[0];   // first selected face handle
    g.predef.entityC = gs.face[1];   // second selected face handle
    g.valA = SS.MmToLength(1.0);     // default 1mm chamfer offset
    g.valB = 0.0;                    // equal-leg (d1 == d2)
    g.meshCombine = CombineAs::ASSEMBLE;
    g.name = C_("group-name", "chamfer");
    g.visible = true;
    g.color = SK.GetGroup(g.opA)->color;
    
    // Standard group creation boilerplate:
    SK.group.AddAndAssignId(&g);
    SS.GW.activeGroup = SK.group.Last()->h;
    SK.GetGroup(SS.GW.activeGroup)->Activate();
    SS.GW.ClearSelection();
    SS.GenerateAll(SolveSpaceUI::Generate::DIRTY);
    SS.ScheduleShowTW();
    break;
}

case Command::GROUP_FILLET: {
    // Similar to CHAMFER but with different defaults
    Group *srcg = SK.GetGroup(SS.GW.activeGroup);
    if(!srcg->IsMeshGroup()) {
        Error(_("Fillet requires an active solid group.\n"));
        return;
    }
    
    SS.GW.GroupSelection();
    auto const &gs = SS.GW.gs;
    if(gs.faces != 2) {
        Error(_("Fillet requires exactly 2 adjacent faces to be selected.\n"));
        return;
    }
    
    Group g = {};
    g.type = Type::FILLET;
    g.opA  = SS.GW.activeGroup;
    g.predef.entityB = gs.face[0];
    g.predef.entityC = gs.face[1];
    g.valA = SS.MmToLength(1.0);   // default 1mm fillet radius
    g.valB = 0.0;
    g.meshCombine = CombineAs::ASSEMBLE;
    g.name = C_("group-name", "fillet");
    g.visible = true;
    g.color = SK.GetGroup(g.opA)->color;
    
    SK.group.AddAndAssignId(&g);
    SS.GW.activeGroup = SK.group.Last()->h;
    SK.GetGroup(SS.GW.activeGroup)->Activate();
    SS.GW.ClearSelection();
    SS.GenerateAll(SolveSpaceUI::Generate::DIRTY);
    SS.ScheduleShowTW();
    break;
}
```

---

## 8. TextWindow UI (textscreens.cpp)

### ShowGroupInfo() Additions

```cpp
// In the if-chain at textscreens.cpp:394:
} else if(g->type == Group::Type::CHAMFER) {
    Printf(true, " %Ft chamfer edge of solid%E");
    Printf(false, "");
    Printf(false, "%Ft source solid%E");
    Printf(false, "%Ba   %s", SK.GetGroup(g->opA)->DescriptionString().c_str());
    Printf(false, "");
    Printf(false, "%Ft offset distance%E");
    Printf(false, "%Ba   %s  %Fl%Ll%f%D[change]%E",
        SS.MmToString(SK.GetParam(g->h.param(0))->val).c_str(),
        &TextWindow::ScreenChangeChamferOffset, g->h.v);

} else if(g->type == Group::Type::FILLET) {
    Printf(true, " %Ft fillet edge of solid%E");
    Printf(false, "");
    Printf(false, "%Ft source solid%E");
    Printf(false, "%Ba   %s", SK.GetGroup(g->opA)->DescriptionString().c_str());
    Printf(false, "");
    Printf(false, "%Ft fillet radius%E");
    Printf(false, "%Ba   %s  %Fl%Ll%f%D[change]%E",
        SS.MmToString(SK.GetParam(g->h.param(0))->val).c_str(),
        &TextWindow::ScreenChangeFilletRadius, g->h.v);
```

### New Edit Meanings in ui.h

```cpp
// In struct TextWindow, enum class Meaning : uint32_t (after HELIX_PITCH=802):
CHAMFER_OFFSET = 803,
FILLET_RADIUS  = 804,
```

### New Callbacks (textscreens.cpp)

```cpp
void TextWindow::ScreenChangeChamferOffset(int link, uint32_t v) {
    Group *g = SK.GetGroup({v});
    SS.TW.ShowEditControl(/*row=*/g->h.v, SS.MmToString(g->valA), g->h);
    SS.TW.edit.meaning = Edit::Meaning::CHAMFER_OFFSET;
    SS.TW.edit.group = g->h;
}

void TextWindow::ScreenChangeFilletRadius(int link, uint32_t v) {
    Group *g = SK.GetGroup({v});
    SS.TW.ShowEditControl(/*row=*/g->h.v, SS.MmToString(g->valA), g->h);
    SS.TW.edit.meaning = Edit::Meaning::FILLET_RADIUS;
    SS.TW.edit.group = g->h;
}
```

### EditControlDone() Additions

```cpp
// After HELIX_PITCH case:
case Edit::Meaning::CHAMFER_OFFSET: {
    Expr *e = Expr::From(s, true);
    if(!e) { Error("Invalid expression."); break; }
    double ev = SS.ExprToMm(e);
    if(ev <= 0) { Error("Chamfer offset must be positive."); break; }
    SS.UndoRemember();
    Group *g = SK.GetGroup(edit.group);
    g->valA = ev;
    SS.MarkGroupDirty(g->h);
    break;
}
case Edit::Meaning::FILLET_RADIUS: {
    Expr *e = Expr::From(s, true);
    if(!e) { Error("Invalid expression."); break; }
    double ev = SS.ExprToMm(e);
    if(ev <= 0) { Error("Fillet radius must be positive."); break; }
    SS.UndoRemember();
    Group *g = SK.GetGroup(edit.group);
    g->valA = ev;
    SS.MarkGroupDirty(g->h);
    break;
}
```

### Extend Existing Conditions

Extend suppress/color/opacity conditions to include CHAMFER/FILLET:
```cpp
// In ShowGroupInfo() where suppress checkbox is shown:
if(g->type == Type::EXTRUDE || ... || 
   g->type == Type::CHAMFER || g->type == Type::FILLET) {
    // show suppress checkbox
}

// In ShowGroupInfo() where color/opacity is shown:
if(g->type == Type::EXTRUDE || ... || 
   g->type == Type::CHAMFER || g->type == Type::FILLET) {
    // show color picker
}
```

Do NOT add CHAMFER/FILLET to the meshCombine radio buttons — always ASSEMBLE (bypassed).

---

## 9. Menu System (graphicswin.cpp and ui.h)

### Command Enum (ui.h)

```cpp
// After GROUP_RECENT:
GROUP_CHAMFER,   // NEW
GROUP_FILLET,    // NEW
```

### Menu Entries (graphicswin.cpp)

```cpp
// In the Group submenu, near EXTRUDE/LATHE/REVOLVE:
{ 1, N_("New C&hamfer"), Command::GROUP_CHAMFER, 0, KN, mGrp },
{ 1, N_("New &Fillet"),  Command::GROUP_FILLET,  0, KN, mGrp },
```

---

## 10. Face Selection Pipeline (No New Infrastructure Needed)

The complete user workflow for creating a chamfer group:

```
1. User has an active extrude/lathe/revolve group (solid is visible)
2. User clicks on face A → HitTestMakeSelection → SMesh::FirstIntersectionWith(mp)
                         → Returns tr.meta.face (= uint32_t face entity handle)
                         → sel.entity.v = face_handle_A
   MakeSelected() → selection.Add(sel)
   
3. User Ctrl+clicks face B → same path → face_handle_B added to selection
   MAX_SELECTABLE_FACES=3 ensures 3-face limit is respected
   
4. User invokes Group > New Chamfer (or shortcut)
   Group::MenuGroup(Command::GROUP_CHAMFER):
     SS.GW.GroupSelection() → gs.faces=2, gs.face[0]=faceA, gs.face[1]=faceB
     Verify gs.faces == 2 → OK
     Create Group g {type=CHAMFER, opA=activeGroup, entityB=faceA, entityC=faceB, valA=1.0}
     SK.group.AddAndAssignId(&g)
     SS.GenerateAll()
     
5. GenerateAll() for CHAMFER group:
   genForBBox: Generate() → AddParam(h.param(0), 1.0)
               Solver: 1 free param, 0 equations → param stays at 1.0
   main:       GenerateShellAndMesh() → MakeFromChamferOf(prevShell, this, 1.0)
               → chamfered shell assembled, runningShell = result
               
6. GenerateDisplayItems() → TriangulateInto(displayMesh) → new surface triangulated
   Chamfer surface (degree 1,1) → UvTriangulateInto → 2 triangles (trivial)
   
7. TextWindow shows "chamfer edge of solid" + offset distance field
```

---

## 11. Surface Generation: The Exact NURBS Math

### Chamfer Surface (Planar, Degree 1,1)

```
Input: edge endpoints V1, V2; face normals n1, n2; offset distance d
t = normalize(V2 - V1)          // edge unit tangent
d1 = t.Cross(n1).WithMagnitude(1)  // inward direction on face 1
d2 = n2.Cross(t).WithMagnitude(1)  // inward direction on face 2

A = V1 + d * d1   (start on face 1)
B = V2 + d * d1   (end on face 1)
C = V2 + d * d2   (end on face 2)
D = V1 + d * d2   (start on face 2)

chamferSurface = SSurface::FromPlane(A, B-A, D-A)
// ctrl[0][0]=A, ctrl[1][0]=B, ctrl[0][1]=D, ctrl[1][1]=C
// all weights = 1.0
// degm=1, degn=1
```

### Fillet Surface (Cylindrical, Degree 2,1, Rational)

```
Input: edge endpoints V1, V2; face normals n1, n2; radius r
t = normalize(V2 - V1)
d1 = t.Cross(n1).WithMagnitude(1)
d2 = n2.Cross(t).WithMagnitude(1)

// Exterior dihedral angle and setback:
half_angle = acos(CLAMP(-n1.Dot(n2), -1, 1)) / 2.0
setback    = r / tan(half_angle)          // setback distance along each face
arc_weight = sin(half_angle)              // = cos(arc_span/2) for exact circle

// Setback points (tangent lines):
A0 = V1 + d1.ScaledBy(setback)   (tangent on face 1 at edge start)
B0 = V1 + d2.ScaledBy(setback)   (tangent on face 2 at edge start)
A1 = V2 + d1.ScaledBy(setback)   (tangent on face 1 at edge end)
B1 = V2 + d2.ScaledBy(setback)   (tangent on face 2 at edge end)

// Fillet arc cross-section at edge start:
SBezier arc;
arc.deg       = 2;
arc.ctrl[0]   = A0;   arc.weight[0] = 1.0
arc.ctrl[1]   = V1;   arc.weight[1] = arc_weight   // < 1
arc.ctrl[2]   = B0;   arc.weight[2] = 1.0

// Extrude arc along edge to get cylinder:
filletSurface = SSurface::FromExtrusionOf(&arc, Vector::From(0,0,0), V2.Minus(V1))
// degm=2, degn=1 — IsCylinder() will detect this
```

**Verification for 90° edge:**
- `n1·n2 = 0`, `half_angle = 45°`, `setback = r`, `arc_weight = sin(45°) = √2/2 ≈ 0.7071`
- Arc sweeps 90° between the two faces → quarter cylinder ✓
- `IsCylinder()` passes → exact raycast intersections in `raycast.cpp` ✓

---

## 12. Trim Polygon Surgery — The Hard Part

The most algorithmically complex aspect of both chamfer and fillet implementation is
modifying the trim polygons of the two adjacent faces.

### Current State (Before Chamfer)
- `surf1` trim list includes `sharedSC` (traversed forward/backward depending on face orientation)
- `surf2` trim list includes `sharedSC` (opposite direction)
- `sharedSC` separates surf1 and surf2

### Required State (After Chamfer)
- `surf1` trim list: `sharedSC` replaced by `hCurve1` (new A→B line)
- `surf2` trim list: `sharedSC` replaced by `hCurve2` (new D→C line)
- New chamfer surface trim list: closed polygon `hCurve1 + hCapEnd + hCurve2(backward) + hCapStart(backward)`
- `sharedSC` removed from `shell.curve`

### Algorithm for Trim Update

```cpp
// Find the STrimBy in surf1 referencing sharedSC:
for(STrimBy &stb : surf1->trim) {
    if(stb.curve == sharedSC->h) {
        bool wasBackwards = stb.backwards;
        // Replace with new curve (same direction convention):
        stb.curve = hCurve1;
        stb.start  = wasBackwards ? B : A;  // endpoints of the new curve
        stb.finish = wasBackwards ? A : B;
        break;
    }
}

// Find the STrimBy in surf2 referencing sharedSC (opposite direction):
for(STrimBy &stb : surf2->trim) {
    if(stb.curve == sharedSC->h) {
        bool wasBackwards = stb.backwards;
        stb.curve = hCurve2;
        stb.start  = wasBackwards ? C : D;
        stb.finish = wasBackwards ? D : C;
        break;
    }
}
```

### Endpoint Matching Requirement

SolveSpace's `AssemblePolygon()` requires that consecutive trim curve endpoints match
within `LENGTH_EPS` tolerance. After the chamfer surgery:
- The chamfer surface's trim must close: `A → B → C → D → A`
- surf1's trim must close: ...(other edges)... → A → B → ...(continues)
- surf2's trim must close: ...(other edges)... → D → C → ...(continues)

This means the neighboring trim curves on surf1 that previously ended at V1 or V2 must
now end at A or B respectively. **This is the true hard part**: finding and updating those
neighboring trim curves.

### MVP Simplification for Endpoint Matching

For the MVP (simple rectangular faces), the adjacent trim curves are straight lines from
the face's other corners to V1 and V2. These need to be truncated or extended to A, B, C, D.

**Option 1** (exact): Update neighboring trim `STrimBy.start`/`finish` to the new endpoints.
If the neighboring trim is a line, simply update its endpoint — no new curves needed.

**Option 2** (approximate): Accept small geometry errors at the vertex endpoints for MVP.
The chamfer surface is correct; the hole at each vertex end is visually small.

**Recommended approach**: Use Option 1 for the MVP with a caveat — only support chamfering
edges where the two faces have their shared edge as a complete side (full-face edge, not
partial). For such cases, the adjacent neighboring trim curves on each face are the
other sides of the face, and the endpoint update is a simple assignment.

---

## 13. Geometry Validation and Error Handling

### Validation Checks in MakeFromChamferOf()

1. **Face lookup**: If `surf1` or `surf2` not found → `booleanFailed = true`; return
2. **Flat face requirement**: `if(!surf1->IsFlat() || !surf2->IsFlat())` → `booleanFailed = true`
3. **Shared edge**: If no SCurve connecting surf1 and surf2 → `booleanFailed = true`
4. **Positive distance**: `if(dist <= 0)` → `booleanFailed = true`
5. **Clamp overflow**: `if(dist > edgeLength/2)` → clamp to `edgeLength/2` and warn

### Error Display

If `booleanFailed = true`, the existing error message in `ShowGroupInfo()` automatically shows:
```
The Boolean operation failed. It may be possible to fix the problem by
choosing 'force NURBS surfaces to triangle mesh'.
```
This message is reused verbatim for chamfer/fillet geometry failures.

### SSurface::IsFlat() Helper

May need to be added or verified exists:
```cpp
bool SSurface::IsFlat() const {
    return (degm == 1 && degn == 1);  // degree (1,1) = planar surface
    // All weights must be 1.0 (non-rational) for a true plane
}
```

---

## 14. Build System Changes

### CMakeLists.txt

```cmake
# In src/CMakeLists.txt, add to the source file list:
srf/chamfer.cpp
```

### New File: src/srf/chamfer.cpp

This file contains:
- `SShell::MakeFromChamferOf(SShell *src, Group *g, double dist)`
- `SShell::MakeFromFilletOf(SShell *src, Group *g, double r)`
- Helper: `SCurve MakeLineSCurve(Vector a, Vector b, hSSurface hA, hSSurface hB)`

### src/srf/surface.h Declarations

```cpp
// Add to SShell class declaration:
void MakeFromChamferOf(SShell *src, Group *g, double dist);
void MakeFromFilletOf(SShell *src, Group *g, double r);
```

---

## 15. What Does NOT Change

These files require **zero changes** for chamfer/fillet MVP:

| File | Reason |
|------|--------|
| `src/file.cpp` | `valA`, `valB`, `predef.entityB/C`, `opA`, `meshCombine`, `remap` all in SAVED[] already |
| `src/exportstep.cpp` | Surface-agnostic; degree (1,1) and (2,1) both handled by RATIONAL_B_SPLINE_SURFACE |
| `src/srf/triangulate.cpp` | `UvTriangulateInto` handles chamfer (1,1); `UvGridTriangulateInto` handles fillet (2,1) |
| `src/srf/surface.cpp` | `FromPlane()` and `FromExtrusionOf()` already exist |
| `src/srf/boolean.cpp` | Not used for direct topology injection |
| `src/draw.cpp` | Face selection already implemented |
| `src/mesh.cpp` | `FirstIntersectionWith()` already returns face handles |
| `src/system.cpp` | No new constraint equation types |
| `src/drawentity.cpp` | Face entities render nothing; chamfer face handled via triangle metadata |

---

## 16. Complete File-Level Change Summary

### Files That Need Changes

```
src/sketch.h
  :181 — Add CHAMFER=5400, FILLET=5401 to Group::Type enum
  :305 — Add REMAP_CHAMFER_FACE=1011, REMAP_FILLET_FACE=1012

src/group.cpp
  :70  — Add case Command::GROUP_CHAMFER and GROUP_FILLET in MenuGroup()
  :~340 — Add case Type::CHAMFER and FILLET in Generate() for AddParam

src/groupmesh.cpp
  :~212 — Add else-if branches for Type::CHAMFER and FILLET in GenerateShellAndMesh()
  :~545 — Add CHAMFER and FILLET to IsMeshGroup()

src/srf/chamfer.cpp (NEW FILE)
  SShell::MakeFromChamferOf(SShell *src, Group *g, double dist)
  SShell::MakeFromFilletOf(SShell *src, Group *g, double r)

src/srf/surface.h
  — Declare MakeFromChamferOf() and MakeFromFilletOf() in SShell class

src/ui.h
  :~170 — Add GROUP_CHAMFER, GROUP_FILLET to Command enum
  :~420 — Add CHAMFER_OFFSET=803, FILLET_RADIUS=804 to TextWindow::Meaning enum
  :~445 — Add static callback declarations ScreenChangeChamferOffset, ScreenChangeFilletRadius

src/graphicswin.cpp
  :42+ — Add menu entries for GROUP_CHAMFER, GROUP_FILLET in the Group submenu

src/textscreens.cpp
  :394 — Add Type::CHAMFER and Type::FILLET branches in ShowGroupInfo() if-chain
  :~700 — Add CHAMFER_OFFSET and FILLET_RADIUS cases in EditControlDone()
  :~595 — Add ScreenChangeChamferOffset and ScreenChangeFilletRadius implementations
  :~525 — Extend color/opacity condition to include CHAMFER/FILLET
  :~537 — Extend suppress condition to include CHAMFER/FILLET

src/CMakeLists.txt
  — Add srf/chamfer.cpp to source list
```

---

## 17. CHAMFER vs. FILLET: Architecture Comparison

| Property | CHAMFER | FILLET |
|----------|---------|--------|
| Group type value | 5400 | 5401 |
| Surface type | Planar, degree (1,1) | Cylindrical, degree (2,1) |
| NURBS weights | All 1.0 | weight[1][j] = sin(half_angle) |
| G-continuity | G0 (sharp join) | G1 (tangent join) |
| Setback = parameter? | Yes: setback = valA | No: setback = r/tan(half_angle) |
| Surface factory | `SSurface::FromPlane()` | `SSurface::FromExtrusionOf(&arc)` |
| Triangulation path | `UvTriangulateInto` (ear-clip) | `UvGridTriangulateInto` (adaptive grid) |
| STEP export | Planar B-spline face | Exact cylindrical NURBS face |
| Vertex caps | Planar triangle (future) | Spherical (complex, skip MVP) |
| IsCylinder() | No | Yes — enables exact raycast |
| Hardest part | Trim polygon surgery | Trim polygon surgery + arc weight |
| Implement first? | YES | After CHAMFER |
| MVP difficulty | Lower | Higher |

---

## 18. Solver Interaction Summary

```
Group::Generate() for CHAMFER:
  AddParam(param, h.param(0), valA)  // 1 free param
  → No equations in GenerateEquations()
  → Solver reports 1 DOF (semantically correct: distance is the free DOF)
  → param(0) = valA after solve

GenerateShellAndMesh() for CHAMFER:
  dist = SK.GetParam(h.param(0))->val  // = valA
  → MakeFromChamferOf(prevShell, this, dist)
  
TextWindow callback to change distance:
  g->valA = newDist                     // update stored value
  SS.MarkGroupDirty(g->h)              // trigger regen
  → Next regen: AddParam re-initializes param(0) to new valA
```

No changes to `src/system.cpp` are needed.

---

## 19. The Watertightness Invariant

Every B-rep operation must maintain:
**Every SCurve is referenced by exactly 2 surface trim lists.**

For chamfer/fillet surgery:
- `sharedSC` was in surf1.trim and surf2.trim (2 references) → remove from both + delete from shell.curve
- `hCurve1` is in surf1.trim and chamferSurf.trim (2 references) ✓
- `hCurve2` is in surf2.trim and chamferSurf.trim (2 references) ✓
- Cap curves at vertices: must be in 2 surface trim lists → partially open in MVP

If violated, `AssemblePolygon()` fails → `booleanFailed = true` → error message shown.
`AssemblePolygon()` can be called explicitly to validate after construction.

---

## 20. Phkahler PR #1501 Lessons Applied

The architecture above directly addresses the root causes of PR #1501's crash:

| PR #1501 Problem | This Architecture's Solution |
|-----------------|------------------------------|
| `srfC` has zero ctrl points | `SSurface::FromPlane()` and `FromExtrusionOf()` set all ctrl pts correctly |
| New curves are copies at same position | New curves computed from offset points A,B,C,D (setback from edge) |
| No TrimBy entries for srfC | Explicit trim polygon construction with all 4 sides |
| Hook in wrong place | GenerateShellAndMesh() injection point (not ModifyEdges()) |
| No face handle for selection | `ss.face = g->Remap(predef.entityB, REMAP_CHAMFER_FACE).v` |
| Standard boolean pipeline used? | NOT used — direct topology injection only |

The architecture is phkahler's original design (surfA ── curve1 ── srfC ── curve2 ── surfB)
but with correct geometry and complete trim polygon wiring.

---

## 21. Summary of Key Architectural Decisions

1. **Two new Group types** (CHAMFER=5400, FILLET=5401) following the DressUp pattern
2. **Direct topology injection** bypassing the boolean pipeline (COINC_OPP issue)
3. **Face-pair selection** using existing infrastructure (no new edge entity types)
4. **Single free solver param** per group (valA → h.param(0), no equations)
5. **New file `src/srf/chamfer.cpp`** for `MakeFromChamferOf()` and `MakeFromFilletOf()`
6. **No file format changes** — all storage fields already exist in SAVED[]
7. **No triangulation changes** — existing paths handle degree (1,1) and (2,1)
8. **No STEP export changes** — RATIONAL_B_SPLINE_SURFACE already handles both surface types
9. **MVP scope**: flat faces only, single edge, equal-leg, convex edges, no vertex caps
10. **CHAMFER first, FILLET second** — chamfer validates the infrastructure

---

## Sources

This synthesis draws on all findings 21–35:
- Finding 21: CHAMFER group design (complete)
- Finding 22: FILLET group design (complete)
- Finding 23: Edge/entity selection design
- Finding 24: STEP export compatibility
- Finding 25: TextWindow UI design
- Finding 26: Standards and annotations
- Finding 27: NURBS cylinder and fillet surface math
- Finding 28: phkahler PR #1501 analysis
- Finding 29: Blender bevel algorithm
- Finding 30: CGAL and geometric algorithms
- Finding 31: sketch.h complete analysis
- Finding 32: group.cpp/groupmesh.cpp code path trace
- Finding 33: boolean.cpp deep dive
- Finding 34: Selection mechanism source analysis
- Finding 35: Solver vs. stored params analysis
- AGENT.md Known Learnings (accumulated)
