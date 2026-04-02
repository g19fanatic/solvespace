# Task 7: Menu and UI Integration for New Group Types in SolveSpace

## Sources Examined
- `src/graphicswin.cpp` — Menu table definitions, Group::MenuGroup dispatch
- `src/group.cpp` — Group::MenuGroup() implementation (all group creation cases)
- `src/textscreens.cpp` — TextWindow::ShowGroupInfo(), ShowListOfGroups()
- `src/toolbar.cpp` — Toolbar icon table  
- `src/ui.h` — Command enum, Edit enum definitions

---

## 1. Menu System Architecture

### Menu Table (graphicswin.cpp:1–200)
SolveSpace uses a **static const MenuEntry table** defined at the top of `graphicswin.cpp`:
```cpp
struct MenuEntry {
    int          level;     // 0 = top-level bar, 1 = submenu
    const char  *label;     // or NULL for separator
    Command      cmd;       // Command enum value
    int          accel;     // keyboard shortcut
    MenuKind     kind;      // NONE, CHECK_MARK, or RADIO_MARK
    MenuHandler *fn;        // function pointer
};
```

The "New Group" menu section (graphicswin.cpp:110–124):
```cpp
{ 0, N_("&New Group"),      Command::NONE,         0,    KN, mGrp   },
{ 1, N_("Sketch In &3d"),   Command::GROUP_3D,     S|'3',KN, mGrp   },
{ 1, N_("Sketch In New &Workplane"), Command::GROUP_WRKPL, S|'w', KN, mGrp },
{ 1, NULL,                  Command::NONE,         0,    KN, NULL   },
{ 1, N_("Step &Translating"),Command::GROUP_TRANS, S|'t',KN, mGrp   },
{ 1, N_("Step &Rotating"),  Command::GROUP_ROT,    S|'r',KN, mGrp   },
{ 1, NULL,                  Command::NONE,         0,    KN, NULL   },
{ 1, N_("E&xtrude"),        Command::GROUP_EXTRUDE,S|'x',KN, mGrp   },
{ 1, N_("&Helix"),          Command::GROUP_HELIX,  S|'h',KN, mGrp   },
{ 1, N_("&Lathe"),          Command::GROUP_LATHE,  S|'l',KN, mGrp   },
{ 1, N_("Re&volve"),        Command::GROUP_REVOLVE,S|'v',KN, mGrp   },
{ 1, NULL,                  Command::NONE,         0,    KN, NULL   },
{ 1, N_("Link / Assemble..."),Command::GROUP_LINK, S|'i',KN, mGrp   },
{ 1, N_("Link Recent"),     Command::GROUP_RECENT, 0,    KN, mGrp   },
```

The handler for all of these is `mGrp` which maps to `&Group::MenuGroup`.

### Adding Chamfer/Fillet to the Menu
**TO ADD**: Insert after the revolve entry in graphicswin.cpp (after line 121):
```cpp
{ 1, NULL,                  Command::NONE,          0,    KN, NULL   },
{ 1, N_("&Chamfer"),        Command::GROUP_CHAMFER, 0,    KN, mGrp   },
{ 1, N_("&Fillet"),         Command::GROUP_FILLET,  0,    KN, mGrp   },
```

---

## 2. Command Enum (ui.h:80–160)
The `Command` enum is defined as:
```cpp
enum class Command : uint32_t {
    NONE = 0,
    // File (100+)
    NEW = 100, OPEN, OPEN_RECENT, ...
    // View
    ZOOM_IN, ZOOM_OUT, ...
    // Edit
    UNDO, REDO, CUT, COPY, PASTE, ...
    // Request
    SEL_WORKPLANE, FREE_IN_3D, DATUM_POINT, ...
    LINE_SEGMENT, CONSTR_SEGMENT, CIRCLE, ARC, ...
    TANGENT_ARC, CONSTRUCTION,
    // Group
    GROUP_3D,
    GROUP_WRKPL,
    GROUP_EXTRUDE,
    GROUP_HELIX,
    GROUP_LATHE,
    GROUP_REVOLVE,  // ui.h:156
    GROUP_ROT,
    GROUP_TRANS,
    GROUP_LINK,
    GROUP_RECENT,
    // Constrain
    DISTANCE_DIA, REF_DISTANCE, ANGLE, ...
};
```

**TO ADD** in ui.h after GROUP_RECENT:
```cpp
GROUP_CHAMFER,   // new
GROUP_FILLET,    // new
```

---

## 3. Group::MenuGroup() Implementation (group.cpp:69–340)

### Dispatch Pattern
```cpp
void Group::MenuGroup(Command id) {
    MenuGroup(id, Platform::Path());
}
void Group::MenuGroup(Command id, Platform::Path linkFile) {
    Group g = {};
    // ... boilerplate initialization ...
    SS.GW.GroupSelection();
    auto const &gs = SS.GW.gs;
    
    switch(id) {
        case Command::GROUP_EXTRUDE:
            // ... set g.type, g.opA, g.name, etc. ...
            break;
        case Command::GROUP_LATHE:
            // ...
            break;
        // etc.
    }
    // At end:
    g.h = SK.group.AddAndAssignId(&g);
    SS.GenerateAll();
    ...
}
```

### What GROUP_EXTRUDE Case Does (group.cpp:170–183):
```cpp
case Command::GROUP_EXTRUDE:
    g.type = Type::EXTRUDE;
    if(!SS.GW.LockedInWorkplane()) {
        Error(_("Extrude must be done in a workplane."));
        return;
    }
    g.opA = SS.GW.activeGroup;
    g.predef.entityB = SS.GW.ActiveWorkplane();
    g.subtype = Subtype::ONE_SIDED;
    g.meshCombine = CombineAs::UNION;
    g.name = C_("group-name", "extrude");
    break;
```

### What HELIX Case Does (group.cpp:238–260):
```cpp
case Command::GROUP_HELIX:
    // requires (point+vector) or line segment selection
    if(gs.points == 1 && gs.vectors == 1 && gs.n == 2) {
        g.predef.origin = gs.point[0];
        g.predef.entityB = gs.vector[0];
    } else if(gs.lineSegments == 1 && gs.n == 1) { ... }
    else { Error(...); return; }
    g.type    = Type::HELIX;
    g.opA     = SS.GW.activeGroup;
    g.valA    = 2;
    g.subtype = Subtype::ONE_SIDED;
    g.name    = C_("group-name", "helix");
    break;
```

### Proposed GROUP_CHAMFER Case
For chamfer/fillet, the user must have selected one or more edges (faces) from the previous group:
```cpp
case Command::GROUP_CHAMFER:
    if(gs.faces == 0 && gs.n == 0) {
        // No edges selected; allow proceeding with user selecting edges later
        // OR: require face/edge selection before invoking
    }
    g.type = Type::CHAMFER;
    g.opA = SS.GW.activeGroup;  // the group to chamfer
    g.valA = SS.MmPerUnit() * 1.0;  // default 1mm offset distance
    g.meshCombine = CombineAs::DIFFERENCE;
    g.name = C_("group-name", "chamfer");
    break;

case Command::GROUP_FILLET:
    g.type = Type::FILLET;
    g.opA = SS.GW.activeGroup;
    g.valA = SS.MmPerUnit() * 1.0;  // default 1mm radius
    g.meshCombine = CombineAs::DIFFERENCE;
    g.name = C_("group-name", "fillet");
    break;
```

---

## 4. Toolbar (toolbar.cpp)

### Toolbar Table Structure
```cpp
struct ToolIcon {
    std::string name;      // icon filename (no extension)
    Command     command;   // linked command
    const char *tooltip;
    std::shared_ptr<Pixmap> pixmap;
};
static ToolIcon Toolbar[] = {
    // Sketch tools...
    { "line", Command::LINE_SEGMENT, N_("Sketch line segment"), {} },
    { "", Command::NONE, "", {} },  // separator
    // Constraint tools...
    { "length", Command::DISTANCE_DIA, ... },
    { "", Command::NONE, "", {} },  // separator
    // Group tools (at end):
    { "extrude",  Command::GROUP_EXTRUDE,  N_("New group extruding active sketch"), {} },
    { "lathe",    Command::GROUP_LATHE,    N_("New group rotating active sketch"),  {} },
    { "helix",    Command::GROUP_HELIX,    N_("New group helix from active sketch"),{} },
    { "revolve",  Command::GROUP_REVOLVE,  N_("New group revolve active sketch"),   {} },
    { "step-rotate",    Command::GROUP_ROT,   ... },
    { "step-translate", Command::GROUP_TRANS, ... },
};
```

**TO ADD** in toolbar.cpp (optional — chamfer/fillet icons):
```cpp
{ "chamfer",  Command::GROUP_CHAMFER, N_("New group chamfer solid edges"), {} },
{ "fillet",   Command::GROUP_FILLET,  N_("New group fillet solid edges"),  {} },
```
PNG icon resources would need to be added to `src/` and `CMakeLists.txt`. If no icons are added, the toolbar entries are omitted (menu access only).

---

## 5. TextWindow::ShowGroupInfo() (textscreens.cpp:380–636)

### Pattern for Group Type Display
The `ShowGroupInfo()` function switches on `g->type` to display relevant parameters. 

Key locations and patterns:

**textscreens.cpp:380** — Function start:
```cpp
void TextWindow::ShowGroupInfo() {
    Group *g = SK.GetGroup(shown.group);
    // Show header with group name
    Printf(true, "%FtGROUP  %E%s [%Fl%Ll%D%frename%E/%Fl%Ll%D%fdel%E]",
        g->DescriptionString().c_str(), ...);
```

**textscreens.cpp:394** — Group-type-specific description:
```cpp
if(g->type == Group::Type::LATHE) {
    Printf(true, " %Ftlathe plane sketch");
} else if(g->type == Group::Type::EXTRUDE || g->type == Group::Type::ROTATE ||
          g->type == Group::Type::TRANSLATE || g->type == Group::Type::REVOLVE ||
          g->type == Group::Type::HELIX) {
    if(g->type == Group::Type::EXTRUDE) s = "extrude plane sketch";
    else if(g->type == Group::Type::HELIX) s = "create helical extrusion";
    else if(g->type == Group::Type::REVOLVE) s = "revolve original sketch";
    ...
    Printf(true, " %Ft%s%E", s);
    // Show one-sided/two-sided options
    ...
} else if(g->type == Group::Type::LINKED) {
    ...
} else if(g->type == Group::Type::DRAWING_3D) {
    Printf(true, " %Ftsketch in 3d%E");
} else {
    Printf(true, "???");   // <-- unknown type fallback
}
```

**textscreens.cpp:498** — MeshCombine display (union/difference/intersection/assemble):
```cpp
if(g->type == Group::Type::EXTRUDE || g->type == Group::Type::LATHE ||
   g->type == Group::Type::REVOLVE || g->type == Group::Type::LINKED ||
   g->type == Group::Type::HELIX) {
    bool un   = (g->meshCombine == Group::CombineAs::UNION);
    bool diff = (g->meshCombine == Group::CombineAs::DIFFERENCE);
    bool intr = (g->meshCombine == Group::CombineAs::INTERSECTION);
    bool asy  = (g->meshCombine == Group::CombineAs::ASSEMBLE);
    Printf(false, "%Fd %f%LU%s union%E  %f%LD%s diff%E  ...",
        &ScreenChangeCombineAs, un   ? RADIO_TRUE : RADIO_FALSE,
        &ScreenChangeCombineAs, diff ? RADIO_TRUE : RADIO_FALSE, ...);
    ...
}
```

### What to Add for CHAMFER/FILLET in ShowGroupInfo()

**Proposed addition** in textscreens.cpp:

After the HELIX case in the if-chain (around line 407):
```cpp
} else if(g->type == Group::Type::CHAMFER) {
    Printf(true, " %Ftchamfer edges of solid%E");
} else if(g->type == Group::Type::FILLET) {
    Printf(true, " %Ftfillet edges of solid%E");
```

**Parameter display** for chamfer (offset distance):
```cpp
if(g->type == Group::Type::CHAMFER) {
    Printf(false, "%Ft chamfer offset%E");
    Printf(false, "  %Ba %s %Fl%Ll%f%D[change]%E",
        SS.MmToString(g->valA, true).c_str(),
        &TextWindow::ScreenChangeChamferOffset, g->h.v);
}
if(g->type == Group::Type::FILLET) {
    Printf(false, "%Ft fillet radius%E");
    Printf(false, "  %Ba %s %Fl%Ll%f%D[change]%E",
        SS.MmToString(g->valA, true).c_str(),
        &TextWindow::ScreenChangeFilletRadius, g->h.v);
}
```

**Add to the meshCombine display condition** (textscreens.cpp:498):
```cpp
if(g->type == Group::Type::EXTRUDE || ... || g->type == Group::Type::CHAMFER ||
   g->type == Group::Type::FILLET) {
    // show DIFFERENCE radio button as locked/default for chamfer/fillet
    // OR: simply don't show it (always DIFFERENCE for chamfer/fillet)
}
```

**RECOMMENDATION**: For MVP, don't show the meshCombine selector for CHAMFER/FILLET — it's always DIFFERENCE for convex edges. Just hardcode it.

---

## 6. Edit Enum Extension (ui.h:317–373)

### Current Edit values
```cpp
struct TextWindow {
    struct Edit {
        enum class Meaning : uint32_t {
            NOTHING               = 0,
            TIMES_REPEATED        = 1,
            ...
            GROUP_COLOR           = 4,
            GROUP_NAME            = 5,
            GROUP_SCALE           = 6,
            GROUP_OPACITY         = 7,
            ...
            PASTE_TIMES_REPEATED  = 600,
            TANGENT_ARC_RADIUS    = 800,
            ...
            HELIX_PITCH           = 802
        };
    };
};
```

**TO ADD**:
```cpp
CHAMFER_OFFSET = 803,   // or 820 to avoid conflicts
FILLET_RADIUS  = 804,
```

### Screen Callback Functions to Add
In textscreens.cpp:
```cpp
void TextWindow::ScreenChangeChamferOffset(int link, uint32_t v) {
    Group *g = SK.GetGroup(SS.TW.shown.group);
    SS.TW.ShowEditControl(3, SS.MmToString(g->valA, true));
    SS.TW.edit.meaning = Edit::CHAMFER_OFFSET;
    SS.TW.edit.group.v = v;
}

void TextWindow::ScreenChangeFilletRadius(int link, uint32_t v) {
    Group *g = SK.GetGroup(SS.TW.shown.group);
    SS.TW.ShowEditControl(3, SS.MmToString(g->valA, true));
    SS.TW.edit.meaning = Edit::FILLET_RADIUS;
    SS.TW.edit.group.v = v;
}
```

And in the edit completion switch at textscreens.cpp:~836:
```cpp
case Edit::CHAMFER_OFFSET:
case Edit::FILLET_RADIUS: {
    Expr *e = Expr::From(edit.str.c_str(), true);
    if(e) {
        double v = e->Eval();
        if(v > 0) {
            SS.UndoRemember();
            g->valA = v * SS.MmPerUnit();
            SS.MarkGroupDirty(g->h);
            SS.GW.ClearSuper();
        }
    }
    break;
}
```

---

## 7. List-of-Groups Warning Display (textscreens.cpp:118–120)

Currently the group list shows a warning icon when EXTRUDE or LATHE has a polyError in their source group:
```cpp
bool warn = (g->type == Group::Type::DRAWING_WORKPLANE && ...) ||
            ((g->type == Group::Type::EXTRUDE ||
              g->type == Group::Type::LATHE) &&
             SK.GetGroup(g->opA)->polyError.how != PolyError::GOOD);
```

**CHAMFER/FILLET warning**: Should also extend this to check if source group has errors:
```cpp
((g->type == Group::Type::EXTRUDE ||
  g->type == Group::Type::LATHE   ||
  g->type == Group::Type::CHAMFER ||
  g->type == Group::Type::FILLET) &&
 SK.GetGroup(g->opA)->polyError.how != PolyError::GOOD);
```

However, chamfer/fillet don't operate on sketch groups — they operate on solid groups. The polyError for a solid group would be populated differently. This will need special handling.

---

## 8. DescriptionString() in group.cpp

The `Group::DescriptionString()` method returns the human-readable group name for display. It typically formats as `"g%03d-name"`. Adding new cases is required in group.cpp:
```cpp
// In Group::DescriptionString() (search for this in group.cpp)
// Add:
case Type::CHAMFER: return ssprintf("g%03d-chamfer", id);
case Type::FILLET:  return ssprintf("g%03d-fillet", id);
```

---

## 9. Complete File Change Summary for Menu/UI

| File | Change | Line Approx |
|------|--------|-------------|
| `src/ui.h` | Add `GROUP_CHAMFER`, `GROUP_FILLET` to `Command` enum | ~156 (after GROUP_RECENT) |
| `src/ui.h` | Add `CHAMFER_OFFSET`, `FILLET_RADIUS` to `Edit::Meaning` | ~373 (after HELIX_PITCH) |
| `src/graphicswin.cpp` | Add 2 menu entries in "New Group" section | ~122 (after GROUP_REVOLVE) |
| `src/toolbar.cpp` | Add optional toolbar icons | ~79 (after GROUP_REVOLVE entry) |
| `src/group.cpp` | Add `GROUP_CHAMFER`, `GROUP_FILLET` cases in `MenuGroup()` switch | ~260 |
| `src/group.cpp` | Add CHAMFER/FILLET to `DescriptionString()` | varies |
| `src/textscreens.cpp` | Add CHAMFER/FILLET branch in `ShowGroupInfo()` | ~407 |
| `src/textscreens.cpp` | Add `ScreenChangeChamferOffset()`, `ScreenChangeFilletRadius()` | ~350 |
| `src/textscreens.cpp` | Add CHAMFER/FILLET in edit completion switch | ~836 |
| `src/textscreens.cpp` | Update warn condition in `ShowListOfGroups()` | ~118 |
| `src/textwin.h` (or ui.h) | Declare new Screen callback methods | varies |

---

## 10. Key Design Insights

### Selection Model for Chamfer/Fillet
The biggest question is **how the user specifies which edges to chamfer/fillet**. Options:
1. **All edges**: The group chamfers ALL edges of the source solid. Simplest to implement, less flexible.
2. **Selected faces**: User selects a face before invoking chamfer, and the group remembers the face. The predef.entityB could store the face entity handle.
3. **Edge list**: User can add/remove edges after group creation via text panel. Most flexible, most complex.

For MVP: **chamfer all convex edges** of the source group is the most practical starting approach. The user can control which edges get chamfered by adjusting which solid group precedes the chamfer group.

### "No sketch" Group
Unlike EXTRUDE/REVOLVE, CHAMFER/FILLET don't require an active workplane or sketch group. They only need the previous solid group (`opA = SS.GW.activeGroup`). The check in MenuGroup() should verify that the active group is a mesh-producing group (IsMeshGroup()).

### Verification That Active Group Has a Mesh
```cpp
case Command::GROUP_CHAMFER: {
    Group *prev = SK.GetGroup(SS.GW.activeGroup);
    if(!prev->IsMeshGroup()) {
        Error(_("Chamfer can only be applied to a solid (extruded, lathed, etc.) group."));
        return;
    }
    g.type = Type::CHAMFER;
    g.opA = SS.GW.activeGroup;
    g.valA = SS.MmPerUnit() * 1.0;  // 1mm default
    g.meshCombine = CombineAs::DIFFERENCE;
    g.name = C_("group-name", "chamfer");
    break;
}
```

---

## Summary of Key Learnings

1. **Menu table is a static const array** in `graphicswin.cpp` — append entries with `mGrp` handler
2. **Command enum** is in `ui.h` under `// Group` comment — append `GROUP_CHAMFER`, `GROUP_FILLET`
3. **Group::MenuGroup() switch** in `group.cpp:73+` — add cases to set group fields then fall through to common `SK.group.AddAndAssignId()` logic
4. **ShowGroupInfo()** needs new `else if` branches for display — show distance/radius with `[change]` links
5. **Edit::Meaning enum** needs 2 new values and corresponding `case` in `EditControl()` handler
6. **Toolbar** is optional — icon PNG files would need to exist, otherwise skip
7. **No "active workplane" required** — chamfer/fillet only need `SS.GW.activeGroup` to be a mesh group
8. **DescriptionString()** must cover CHAMFER/FILLET to avoid falling into the default case
