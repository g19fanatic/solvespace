# SolveSpace Code Patterns

> Recurring design patterns and idioms used throughout the SolveSpace codebase.
> Cross-reference: [architecture.md](architecture.md), [subsystems/sketch.md](subsystems/sketch.md)

---

## 1. Handle / IdList Pattern

**Purpose:** Type-safe integer handles that reference objects in sorted, O(log n) lookup lists. Prevents dangling pointers; all cross-references are by stable ID, not raw pointer.

### Handle Types

Defined in `src/sketch.h:61-120`. Every domain object has a corresponding handle class with a single `uint32_t v` member:

| Handle Type  | Refers to    | Notable bit encoding                   |
|--------------|--------------|----------------------------------------|
| `hGroup`     | `Group`      | bits 15:0 — group index                |
| `hRequest`   | `Request`    | bits 15:0 — request index              |
| `hEntity`    | `Entity`     | bits 15:0 entity index, 31:16 request  |
| `hConstraint`| `Constraint` | —                                      |
| `hParam`     | `Param`      | —                                      |
| `hStyle`     | `Style`      | —                                      |
| `hEquation`  | `Equation`   | encodes constraint origin              |

All handle types register via `IsHandleOracle<hFoo> : std::true_type` (`src/handle.h`), enabling generic equality/comparison operators.

### IdList Template

`src/dsc.h:383` — `template<class T, class H> class IdList`

Internally: `std::vector<T> elemstore` + sorted `std::vector<int> elemidx` for binary-search by `h.v`. Supports O(log n) `FindById`, range-for iteration, `Add`, `RemoveTagged`, `MoveSelfInto`, `DeepCopyInto`.

**Usage pattern:**
```cpp
// Declaration (src/solvespace.h:401-408)
IdList<Group,hGroup>            group;
IdList<Request,hRequest>        request;
IdList<ENTITY,hEntity>          entity;
IdList<CONSTRAINT,hConstraint>  constraint;

// Lookup by handle
Entity *e = SK.entity.FindById(he);

// Range iteration
for(Group &g : SK.group) { ... }
```

**Key files:** `src/handle.h`, `src/dsc.h:355-580`, `src/sketch.h:61-120`, `src/solvespace.h:401-408`

---

## 2. TextWindow Printf DSL

**Purpose:** The text window (left panel) is rendered entirely through a custom `Printf`-like function that encodes text color, links, and interactive callbacks inline in format strings.

### Printf Signature

`src/ui.h:280` / `src/textwin.cpp:360`
```cpp
void TextWindow::Printf(bool halfLine, const char *fmt, ...);
```
- `halfLine=true` adds a half-line vertical spacer after output

### Format Codes (src/textwin.cpp:401-480)

| Code    | Type   | Effect                                      |
|---------|--------|---------------------------------------------|
| `%s`    | char*  | Insert string                               |
| `%d`    | int    | Insert decimal integer                      |
| `%x`    | uint   | Insert 8-digit hex                          |
| `%@`    | double | Insert `%.2f`                               |
| `%2`    | double | Insert `%.2f` with sign-space padding       |
| `%3`    | double | Insert `%.3f` with sign-space padding       |
| `%#`    | double | Insert `%.3f`                               |
| `%c`    | int    | Insert single character                     |
| `%Fx`   | —      | Set foreground color to palette char `x`    |
| `%Bx`   | —      | Set background color to palette char `x`    |
| `%Fp`   | int    | Set fg to runtime palette index             |
| `%Bz`   | RgbaColor* | Set bg to explicit RGB                 |
| `%Ll`   | char   | Begin hyperlink with tag char `l`           |
| `%Lp`   | int    | Begin hyperlink with runtime tag int        |
| `%E`    | —      | Reset fg color and clear link               |
| `%f`    | fn ptr | Attach click callback function              |
| `%h`    | fn ptr | Attach hover callback function              |

**Example usage (src/textwin.cpp:553):**
```cpp
Printf(true, "%Fl%f%Ll(cancel operation)%E",
       &TextWindow::ScreenNavigation, ...);
```

**Key files:** `src/textwin.cpp:360-530`, `src/ui.h:275-295`

---

## 3. Screen Navigation Pattern

**Purpose:** The text window displays one "screen" at a time. Navigation is stateful: `SS.TW.shown.screen` holds the current screen enum, with additional context (group/style/constraint handle) for detail screens.

### Screen Enum

`src/ui.h:284-294` (`TextWindow::Screen`):

```cpp
enum class Screen : uint32_t {
    LIST_OF_GROUPS      = 0,   // default view: list all groups
    GROUP_INFO          = 1,   // group detail, uses shown.group
    GROUP_SOLVE_INFO    = 2,   // solve result for one group
    CONFIGURATION       = 3,   // global config (confscreen.cpp)
    STEP_DIMENSION      = 4,   // step/dimension tool
    LIST_OF_STYLES      = 5,   // style list
    STYLE_INFO          = 6,   // style detail, uses shown.style
    PASTE_TRANSFORMED   = 7,   // paste transform dialog
    EDIT_VIEW           = 8,   // view settings
    TANGENT_ARC         = 9    // tangent arc params
};
```

### ShownState Context

`src/ui.h:295-315`: `TextWindow::ShownState shown` carries the active screen plus context:
```cpp
typedef struct {
    Screen      screen;
    hGroup      group;      // for GROUP_INFO/GROUP_SOLVE_INFO
    hStyle      style;      // for STYLE_INFO
    hConstraint constraint; // for describe screen
    struct { int times; Vector trans; ... } paste; // for PASTE_TRANSFORMED
} ShownState;
```

### Navigation API

- `SS.TW.shown.screen = TextWindow::Screen::GROUP_INFO;`
- `SS.TW.shown.group = hg;`
- `SS.TW.Show();` (triggers repaint via `SS.ScheduleShowTW()`)

Screens are rendered by `TextWindow::Show()` dispatching to per-screen functions in `src/textscreens.cpp` and `src/describescreen.cpp`.

**Key files:** `src/ui.h:275-330`, `src/textwin.cpp`, `src/textscreens.cpp`, `src/describescreen.cpp`

---

## 4. Group/Entity/Constraint Solve Lifecycle

**Purpose:** Whenever the sketch changes, all groups must regenerate their entities, equations, and be re-solved. This is orchestrated by `GenerateAll`.

### Flow

```
User action (mouse.cpp / graphicswin.cpp)
    │
    ▼
SS.UndoRemember()   ← snapshot current state for undo
    │
    ▼
Modify SK (group, request, constraint, param)
    │
    ▼
SS.ScheduleGenerateAll() or SS.GenerateAll()
    │     [src/solvespace.h:689, src/generate.cpp]
    ▼
For each group (in order):
    Group::Generate()        ← build entities from requests
    Group::GenerateEquations() ← emit constraint equations
    System::Solve()          ← Newton-Raphson on equations
    Group::Draw()            ← update display mesh/shell
    │
    ▼
SS.ScheduleShowTW()  ← refresh text window
```

**`GenerateAll` types** (`src/solvespace.h:689`):
- `Generate::DIRTY` — only groups marked dirty (default)
- `Generate::ALL` — rebuild everything (used after undo/redo)
- `Generate::REGEN` — regenerate without solving

**Key files:** `src/generate.cpp`, `src/solvespace.h:689-720`, `src/group.cpp`, `src/system.cpp`

---

## 5. Undo/Redo Pattern

**Purpose:** Full sketch state snapshot stored in a circular buffer. Undo restores a prior snapshot; redo stores current state and restores a forward snapshot.

### Data Structures (`src/solvespace.h:434-458`)

```cpp
typedef struct UndoState {
    IdList<Group,hGroup>            group;
    IdList<Request,hRequest>        request;
    IdList<Constraint,hConstraint>  constraint;
    IdList<Param,hParam>            param;
    IdList<Style,hStyle>            style;
    hGroup                          activeGroup;
} UndoState;

enum { MAX_UNDO = 100 };

typedef struct {
    UndoState   d[MAX_UNDO];  // circular ring buffer
    int         cnt, write;
} UndoStack;

UndoStack   undo;  // history stack
UndoStack   redo;  // redo stack
```

### Usage Protocol (`src/undoredo.cpp`)

1. **Before any mutating operation:** `SS.UndoRemember()` — pushes current `SK` onto `undo` ring, clears `redo`.
2. **Ctrl+Z:** `SS.UndoUndo()` — pushes current onto `redo`, pops from `undo`, restores `SK`, calls `GenerateAll(ALL)`.
3. **Ctrl+Y:** `SS.UndoRedo()` — symmetric.

**Key detail:** `PushFromCurrentOnto` does a **shallow copy** of each `Group`, then explicitly zeros out regeneratable fields (`thisMesh`, `runningMesh`, etc.) — only the parametric data (requests, constraints, params, remap) is preserved.

**Key files:** `src/undoredo.cpp`, `src/solvespace.h:434-471`

---

## 6. Platform Abstraction Pattern

**Purpose:** All OS-specific GUI, filesystem, and timer operations route through the `Platform` namespace (`src/platform/`), giving a single API surface for GTK, Win32, Cocoa, Qt, and WASM backends.

### Namespace Structure

`src/platform/platform.h:14` — `namespace Platform` provides:

| Abstraction    | Description                                     |
|----------------|-------------------------------------------------|
| `Platform::Path` | Cross-platform file path with `Open()`, `Parent()` |
| `Platform::OpenFile` / `SaveFile` | File dialog with filter support |
| `Platform::Timer` | One-shot and recurring timer                  |
| `Platform::WindowRef` | Opaque window handle                       |
| `Platform::Gui` | Defined in `src/platform/gui.h`, exposes `CreateWindow`, `RunMessageLoop`, menu construction |

### Backend Implementations

| File                                  | Platform       |
|---------------------------------------|----------------|
| `src/platform/guigtk.cpp`            | Linux GTK3     |
| `src/platform/guiwin.cpp`            | Windows Win32  |
| `src/platform/guicocoa.mm`           | macOS Cocoa    |
| `src/platform/guiqt.cpp`             | Qt (all OS)    |
| `src/platform/guiemscripten.cpp`     | WebAssembly    |

### Entry Points

- GUI: `src/platform/entrygui.cpp` — calls `SS.Init()` then `Platform::RunMessageLoop()`
- CLI: `src/platform/entrycli.cpp` — headless mode for export/batch

**Key files:** `src/platform/platform.h`, `src/platform/gui.h`, `src/platform/entrygui.cpp`

---

## 7. Tag-Based Bulk Delete Pattern

**Purpose:** `IdList` objects support marking items with a `tag` field, then bulk-removing all tagged items in a single pass. Used pervasively throughout constraint generation and cleanup.

```cpp
// Pattern (dsc.h:List / IdList):
SK.entity.ClearTags();          // zero all tags
for(Entity &e : SK.entity) {
    if(e.shouldDelete) e.tag = 1;
}
SK.entity.RemoveTagged();       // single-pass compact
```

**Key files:** `src/dsc.h:248-275` (`RemoveTagged`, `ClearTags`)

---

## 8. Global Singleton Access (`SS` and `SK`)

**Purpose:** The entire sketch state is stored in two global singletons accessed throughout the codebase without passing parameters.

```cpp
extern SolveSpaceUI SS;   // declared in solvespace.h — app state, UI, solve
extern class Sketch SK;   // declared in solvespace.h — all sketch data lists
```

- `SK.group`, `SK.entity`, `SK.constraint`, etc. — the live sketch IdLists
- `SS.GW` — `GraphicsWindow` instance
- `SS.TW` — `TextWindow` instance
- `SS.GenerateAll(...)` — trigger regeneration

**Key files:** `src/solvespace.h:395-460` (Sketch class), `src/solvespace.h:480-720` (SolveSpaceUI class)

---

## 9. Direct Topology Injection (Chamfer/Fillet)

**Purpose:** Apply an edge chamfer or fillet to a solid shell without going through the
full boolean intersection pipeline. Instead, deep-copy the source shell and surgically
modify the topology: splice out the shared edge between two faces, insert a new
chamfer/fillet surface, and re-wire the trim loops of all affected surfaces.

**Where used:** `src/srf/chamfer.cpp` — `SShell::MakeFromChamferOf()` / `SShell::MakeFromFilletOf()`

### Core Steps

```
1. MakeFromCopyOf(src)          — deep-copy the entire source shell
2. Find hSurf1, hSurf2          — look up face entities by ss.face == entityB.v / entityC.v
3. Validate flat faces           — DepartureFromCoplanar check
4. Find shared SCurve            — sc.surfA == hSurf1 && sc.surfB == hSurf2 (or swapped)
5. Compute endpoints V1, V2      — from shared curve, check dist < edgeLen/2
6-7. Compute face normals,       — cross-product of ctrl points, centroid-based
     inward directions d1/d2
8. Compute corners A,B,C,D       — A=V1+d1·dist, B=V2+d1·dist, C=V2+d2·dist, D=V1+d2·dist
9. Create chamfer surface        — SSurface::FromPlane(A, B-A, D-A), remap face entity
10. Cap surface detection        — scan neighbor curves at V1/V2 for cap surfaces
    + create hCapV1 (A→D), hCapV2 (B→C) via AddLinearCurve()
11. Build chamfer surface trims  — CW winding: A→D (fwd), D→C (fwd), C→B (bkw), B→A (bkw)
12. Update surf1 trims           — replace hShared with hCurve1; V1→A, V2→B via InsertPointIntoCurvePts()
13. Update surf2 trims           — replace hShared with hCurve2; V1→D, V2→C
14. Remove old shared SCurve     — tag + RemoveTagged()
15. Update cap surface trims     — patch neighbor endpoints V1→A/D (capSurf1), V2→B/C (capSurf2);
                                   add cap curve as new STrimBy entry with dynamic backwards flag
```

### ASSEMBLE-Skip (Key Design Decision)

After `MakeFromChamferOf()` produces `thisShell` (a complete modified copy of the source),
the generic `GenerateShellAndMesh()` code would normally assemble `thisShell` with the
previous group's shell via `MakeFromAssemblyOf()`. But since `thisShell` already IS a full
copy of the source shell, the assembly would **double all geometry** — causing face-handle
collisions when chaining multiple chamfers/fillets.

The fix at `src/groupmesh.cpp:419` early-returns for CHAMFER/FILLET:

```cpp
if(type == Type::CHAMFER || type == Type::FILLET) {
    if(!IsForcedToMesh()) {
        runningShell.MakeFromCopyOf(&thisShell);
        booleanFailed = thisShell.booleanFailed;
    } else {
        runningMesh.MakeFromCopyOf(&thisMesh);
        thisShell.TriangulateInto(&runningMesh);
    }
    displayDirty = true;
    return;
}
```

This makes `runningShell = thisShell` directly, enabling chaining without geometry
duplication. Each subsequent chamfer/fillet sees exactly one copy of each face handle.

### Helper Functions

**`InsertPointIntoCurvePts(SCurve *sc, Vector P)`** — `chamfer.cpp:19`  
Splits a piecewise-linear curve at point P, inserting P into the `pts` list at the
correct segment. Used when a chamfer setback point lies on an existing curve.

**`AddLinearCurve(SShell*, Vector from, Vector to, hSSurface, hSSurface)`** — `chamfer.cpp:66`  
Creates a new straight-line `SCurve` between two 3D points, assigns surface handles,
populates `pts`, and inserts with a new handle. Used for the 4 chamfer boundary curves.

### Key Files

| File | Content |
|------|---------|
| `src/srf/chamfer.cpp` | Full implementation: helpers, MakeFromChamferOf, MakeFromFilletOf |
| `src/groupmesh.cpp:392-435` | CHAMFER/FILLET dispatch + ASSEMBLE-skip |
| `src/groupmesh.cpp:583-597` | RunningMeshGroup + IsMeshGroup for CHAMFER/FILLET |
| `src/sketch.h:190-191` | CHAMFER=5400, FILLET=5401 |
| `src/sketch.h:318-319` | REMAP_CHAMFER_FACE=1011, REMAP_FILLET_FACE=1012 |
| `src/srf/surface.h:432-436` | MakeFromChamferOf/FilletOf declarations |
| `test/group/chamfer/test.cpp` | 12+ tests; chaining_test.slvs for multi-chamfer scenario |

> **Cross-reference**: See [subsystems/chamfer-fillet.md](subsystems/chamfer-fillet.md) for full step-by-step documentation.
