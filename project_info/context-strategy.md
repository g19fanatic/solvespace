# SolveSpace — LLM Context Optimization Guide

> **Purpose**: Which files to load for which tasks, key entry points with `filepath:line`
> references, and subsystem boundaries for focused work with minimal context overhead.
>
> **Cross-references**: [overview.md](overview.md) | [architecture.md](architecture.md) |
> [code-patterns.md](code-patterns.md)

---

## Quick Reference: Minimal Load Sets by Task

This table gives the **minimum viable file set** to load in an LLM context window
for each common task category. Load the "Core Always" set first, then add the
task-specific files.

### Core Always (load for any task)

| File | Why |
|------|-----|
| `src/solvespace.h` | Defines `SolveSpaceUI`, `Sketch`, `System`, `SolveResult` enum, `Generate` enum, undo structures — the top-level types everything depends on |
| `src/sketch.h` | Defines `Group`, `Request`, `EntityBase`, `ConstraintBase`, handle types (`hGroup`, `hEntity`, …), `Style` — the sketch data model |
| `src/dsc.h` | `IdList<T,H>`, `List<T>`, `Vector`, `Quaternion`, `RgbaColor`, `BBox` — all core containers and math types |

### Task-Specific Load Sets

#### Constraint Solving / DOF Issues

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/system.cpp` | full file (~21KB) | Newton-Raphson solver, Jacobian, `System::Solve()`, substitution |
| `src/constrainteq.cpp` | full file (~42KB) | How each `Constraint::Type` translates to symbolic equations |
| `src/solvespace.h` | `44–50` (SolveResult), `689–720` (GenerateAll, SolveGroup) | SolveResult values; solve trigger API |
| `src/generate.cpp` | `SolveSpaceUI::SolveGroup`, `WriteEqSystemForGroup` | How equations are assembled per group before calling `System::Solve()` |
| `src/expr.h` + `src/expr.cpp` | full | Symbolic expression algebra used in Jacobian; `Expr::PartialWrt`, `Eval`, `Substitute` |
| `src/param.h` | full (tiny) | `Param` struct: `h`, `val`, `known`, `free`, `tag` |

#### Adding/Modifying a Geometric Entity

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/sketch.h` | `EntityBase::Type` enum, `EntityBase` class | Entity type codes, field layout |
| `src/request.cpp` | full (~9KB) | `Request::Generate()` — how requests create entities + params |
| `src/entity.cpp` | full (~35KB) | `EntityBase` method implementations: `PointGetNum()`, `VectorGetNum()`, `GenerateEquations()` |
| `src/constrainteq.cpp` | relevant constraint section | If also touching constraints tied to the entity |
| `src/drawentity.cpp` | full (~30KB) | `Entity::Draw()` — how to add rendering for new entity types |

#### Adding/Modifying a Constraint

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/sketch.h` | `ConstraintBase::Type` enum, `ConstraintBase` class | Constraint types; field layout (`ptA/ptB/entityA-D/valA`) |
| `src/constrainteq.cpp` | relevant type block | Equation generation; each type has a `case` block |
| `src/constraint.cpp` | full (~44KB) | Drawing, factory methods (`Constraint::Constrain`), static helpers |
| `src/drawconstraint.cpp` | full (~55KB) | `Constraint::Draw()` — label placement, arrow rendering |
| `src/describescreen.cpp` | relevant section | TextWindow describe panel for the constraint |

#### Group Types / Solid Modeling

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/sketch.h` | `Group::Type` enum, `Group` class | Group types: DRAWING_3D/WORKPLANE/EXTRUDE/LATHE/REVOLVE/HELIX/ROTATE/TRANSLATE/LINKED |
| `src/group.cpp` | full (~49KB) | `Group::Generate()`, `GenerateLoops()`, `GenerateShellAndMesh()`, mesh boolean ops |
| `src/groupmesh.cpp` | full (~31KB) | Mesh generation helpers for 3D groups (extrude, lathe, revolve) |
| `src/srf/surface.h` | `SSurface`, `SShell` class | NURBS/B-rep surface types |
| `src/srf/boolean.cpp` | full | Shell boolean operations (union/difference/intersection) |
| `src/generate.cpp` | `GenerateAll()` | The orchestration loop for per-group solve+mesh |

#### UI / Text Window Screens

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/ui.h` | `TextWindow` class, `Screen` enum (284–294), `ShownState` (295–315), `Edit` enum | Screen navigation state machine |
| `src/textwin.cpp` | `Printf()` impl (~360–530) | Printf DSL format codes |
| `src/textscreens.cpp` | full (~37KB) | All screen renderers: `ShowListOfGroups`, `ShowGroupInfo`, etc. |
| `src/describescreen.cpp` | full (~25KB) | Entity/constraint describe panel |
| `src/confscreen.cpp` | full (~22KB) | Configuration screen |

#### Graphics Window / Mouse Interaction

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/ui.h` | `GraphicsWindow` class, `Pending` enum | GW state: selection, pending operations, view params |
| `src/graphicswin.cpp` | full (~62KB) | `Draw()`, `Paint()`, `KeyboardEvent()`, menu handlers |
| `src/mouse.cpp` | full (~62KB) | `MouseLeftDown()`, `MouseMoved()`, dragging logic |
| `src/draw.cpp` | full (~35KB) | `DrawEntities()`, scene assembly |
| `src/drawentity.cpp` | full | Entity-specific draw via Canvas |
| `src/drawconstraint.cpp` | full | Constraint-specific draw |

#### File Format / Persistence

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/file.cpp` | full (~42KB) | `.slvs` load/save; `SAVED[]` table; `SaveUsingTable` / `LoadUsingTable` |
| `src/solvespace.h` | `SAVED[]` array declaration, `sv` struct | Field name to pointer mappings |

#### Export (DXF, SVG, STEP, STL, …)

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/export.cpp` | full (~47KB) | Export dispatcher; format selection |
| `src/exportvector.cpp` | full (~47KB) | SVG, DXF, EPS, PDF, HPGL, G-code writers |
| `src/exportstep.cpp` | full (~25KB) | STEP surface export |
| `src/solvespace.h` | `ExportMeshTo`, `ExportViewOrWireframeTo` | Export entry points |

#### Platform / Cross-Platform Work

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/platform/gui.h` | full (~8KB) | `Platform::Window`, `Timer`, `Settings`, `MenuItem`, `Menu`, `FileDialog` interfaces |
| `src/platform/platform.h` | full | `Platform::Path`, `Settings`, arena allocator, CLI init |
| `src/platform/gui{gtk,qt,win,mac,html}.cpp` | relevant platform file | Implementation for the target platform |
| `src/platform/entrygui.cpp` | full (small) | GUI `main()` — startup sequence |
| `src/platform/entrycli.cpp` | full (small) | CLI `main()` — headless startup |

#### Rendering Backends

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/render/render.h` | full | `Canvas`, `Camera`, `Lighting`, `BatchCanvas`, `ViewportCanvas` interfaces |
| `src/render/rendergl1.cpp` or `rendergl3.cpp` | relevant | OpenGL backend implementation |
| `src/render/rendercairo.cpp` | full | Cairo backend for export/CLI thumbnail |

#### C API / Python Bindings

| File | Lines of interest | Why |
|------|------------------|-----|
| `include/slvs.h` | full | Public C API: `Slvs_MakeParam`, `Slvs_AddConstraint`, `Slvs_Solve`, etc. |
| `src/slvs/slvs.cpp` | full | C API bridge to internal solver |
| `exposed/CDemo.c` | full | Usage example for C API |
| `python/` | relevant `.py`/`.pyi` files | Python ctypes wrappers |

#### Chamfer/Fillet Development

| File | Lines of interest | Why |
|------|------------------|-----|
| `src/srf/chamfer.cpp` | full (~960 lines) | Core algorithm: `MakeFromChamferOf`, `MakeFromFilletOf`, helpers |
| `src/groupmesh.cpp` | `389–437` | CHAMFER/FILLET dispatch in `GenerateShellAndMesh` + ASSEMBLE-skip fix |
| `src/group.cpp` | `324–362` | `MenuGroup` command handlers for GROUP_CHAMFER / GROUP_FILLET |
| `src/sketch.h` | `186–191`, `313–319` | `Group::Type::CHAMFER=5400`, `FILLET=5401`; `REMAP_CHAMFER_FACE=1011`, `REMAP_FILLET_FACE=1012` |
| `src/srf/surface.h` | `432–436` | `MakeFromChamferOf` / `MakeFromFilletOf` declarations; `int tag` on `SCurve` |
| `src/textscreens.cpp` | `488–517`, `948–985` | `ShowGroupInfo` CHAMFER/FILLET panels; edit control handlers |
| `src/ui.h` | `157–161`, `372–378`, `512–513` | `Command::GROUP_CHAMFER/FILLET`; `Edit::CHAMFER_OFFSET/FILLET_RADIUS` |
| `test/group/chamfer/test.cpp` | full (2863 lines) | **63 tests**: basic, mesh, chaining, stale-vertex, backface, origin-line regression |

**Key subsystem doc**: `project_info/subsystems/chamfer-fillet.md` — full 15-step algorithm, 7 helper functions, 63-test coverage summary, ASSEMBLE-skip design rationale, backface/origin-line regression notes.

**Debug note**: All `CHAMFER_DEBUG` lines have been stripped (0 remaining). No action needed.

---

## Key Entry Points: `filepath:line`

### Application Startup

| Symbol | Location | Purpose |
|--------|----------|---------|
| `main()` (GUI) | `src/platform/entrygui.cpp:14` | GUI entry; calls `Platform::InitGui`, `SS.Init()`, `Platform::RunGui()` |
| `main()` (CLI) | `src/platform/entrycli.cpp:1` | CLI entry; parses commands, calls headless export |
| `SolveSpaceUI::Init()` | `src/solvespace.cpp` (search `void SolveSpaceUI::Init`) | Loads settings, creates windows, registers menus |
| Global `SS` (SolveSpaceUI) | `src/solvespace.cpp:11` | App controller singleton |
| Global `SK` (Sketch) | `src/solvespace.cpp:12` | Sketch data singleton |

### Core Solve Loop

| Symbol | Location | Purpose |
|--------|----------|---------|
| `SolveSpaceUI::GenerateAll()` | `src/generate.cpp` + `src/solvespace.h:689` | Main regeneration entry; takes `Generate::DIRTY/ALL/REGEN/UNTIL_ACTIVE` |
| `SolveSpaceUI::SolveGroup()` | `src/generate.cpp` (search `void SolveSpaceUI::SolveGroup`) | Solve one group: calls `WriteEqSystemForGroup`, then `sys.Solve()` |
| `SolveSpaceUI::WriteEqSystemForGroup()` | `src/generate.cpp` | Populates `sys.entity`, `sys.param`, `sys.eq` from a group |
| `System::Solve()` | `src/system.cpp` (search `SolveResult System::Solve`) | Newton's method solver; returns `SolveResult` |
| `System::WriteJacobian()` | `src/system.cpp` (search `bool System::WriteJacobian`) | Builds symbolic Jacobian from active equations |
| `System::NewtonSolve()` | `src/system.cpp` (search `bool System::NewtonSolve`) | Iterative Newton's method; max 50 iterations |
| `SolveResult` enum | `src/solvespace.h:44–50` | `OKAY`, `DIDNT_CONVERGE`, `REDUNDANT_OKAY`, `REDUNDANT_DIDNT_CONVERGE`, `TOO_MANY_UNKNOWNS` |

### Sketch Data Model

| Symbol | Location | Purpose |
|--------|----------|---------|
| `class Sketch` | `src/solvespace.h` (search `class Sketch`) | `SK`: holds `group`, `request`, `constraint`, `entity`, `param`, `style` |
| `IdList<T,H>` | `src/dsc.h:383` | Handle-indexed sorted list; `FindById`, `RemoveTagged`, `Add` |
| Handle types | `src/sketch.h:61–120` | `hGroup`, `hRequest`, `hEntity`, `hConstraint`, `hParam`, `hStyle`, `hEquation` |
| `Group::Type` | `src/sketch.h` (search `enum class Type`) | DRAWING_3D, DRAWING_WORKPLANE, EXTRUDE, LATHE, REVOLVE, HELIX, ROTATE, TRANSLATE, LINKED |
| `EntityBase::Type` | `src/sketch.h` (search `enum class Type` under `EntityBase`) | POINT_IN_3D, LINE_SEGMENT, CIRCLE, ARC_OF_CIRCLE, WORKPLANE, etc. |
| `ConstraintBase::Type` | `src/sketch.h` (search `enum class Type` under `ConstraintBase`) | ~40 constraint types: POINTS_COINCIDENT, PT_PT_DISTANCE, ANGLE, PARALLEL, etc. |
| `Request::Type` | `src/sketch.h` (search `enum class Type` under `Request`) | WORKPLANE, DATUM_POINT, LINE_SEGMENT, CUBIC, CIRCLE, ARC_OF_CIRCLE, TTF_TEXT, IMAGE |

### Undo/Redo

| Symbol | Location | Purpose |
|--------|----------|---------|
| `UndoStack` / `UndoState` | `src/solvespace.h:434–471` | Ring buffer of up to 100 sketch snapshots |
| `SS.UndoRemember()` | `src/undoredo.cpp` | Call before any mutating operation |
| `SS.UndoUndo()` / `SS.UndoRedo()` | `src/undoredo.cpp` | Ctrl+Z / Ctrl+Y |

### Text Window Navigation

| Symbol | Location | Purpose |
|--------|----------|---------|
| `TextWindow::Screen` | `src/ui.h:284–294` | 10 screen enum values |
| `TextWindow::ShownState` | `src/ui.h:295–315` | Active screen + context handles |
| `TextWindow::Printf()` | `src/ui.h:280` (decl), `src/textwin.cpp:360` (impl) | Printf DSL for text panel |
| `TextWindow::Show()` | `src/textwin.cpp` (search `void TextWindow::Show`) | Dispatch to per-screen renderer |
| `SS.ScheduleShowTW()` | `src/solvespace.h:719` | Trigger text window repaint |

### Rendering

| Symbol | Location | Purpose |
|--------|----------|---------|
| `Canvas` interface | `src/render/render.h` | Backend-agnostic drawing API |
| `GraphicsWindow::Paint()` | `src/graphicswin.cpp` | Triggers scene redraw via `Draw(canvas)` |
| `GraphicsWindow::Draw()` | `src/graphicswin.cpp` | Scene draw: entities + constraints + mesh + UI |

---

## Subsystem Boundaries

Use these when you want to limit scope to a single subsystem:

```
Solver subsystem:       src/system.cpp
                        src/constrainteq.cpp
                        src/expr.h + src/expr.cpp
                        src/param.h

Sketch data model:      src/sketch.h
                        src/dsc.h
                        src/handle.h
                        src/param.h

Group/Mesh pipeline:    src/group.cpp
                        src/groupmesh.cpp
                        src/generate.cpp
                        src/srf/

Entity handling:        src/entity.cpp
                        src/request.cpp
                        src/drawentity.cpp

Constraint handling:    src/constraint.cpp
                        src/constrainteq.cpp
                        src/drawconstraint.cpp

Text UI:                src/ui.h            (TextWindow declaration)
                        src/textwin.cpp     (Printf DSL + Show() dispatch)
                        src/textscreens.cpp (per-screen renderers)
                        src/describescreen.cpp
                        src/confscreen.cpp

Graphics UI:            src/ui.h            (GraphicsWindow declaration)
                        src/graphicswin.cpp (draw + keyboard)
                        src/mouse.cpp       (mouse events + dragging)
                        src/draw.cpp        (scene assembly)

Rendering:              src/render/render.h
                        src/render/render.cpp
                        src/render/rendergl1.cpp  (or rendergl3.cpp)
                        src/render/rendercairo.cpp

Platform:               src/platform/gui.h
                        src/platform/platform.h
                        src/platform/gui<platform>.cpp

File I/O:               src/file.cpp
NURBS/surfaces:         src/srf/surface.h + src/srf/*.cpp
```

---

## Common Pitfalls & LLM Guidance

1. **`SK.entity` and `SK.param` are generated, not user-editable.** Do not search for user
   modifications to these lists. User data lives in `SK.group`, `SK.request`,
   `SK.constraint`, `SK.style`. See `src/solvespace.h` (`class Sketch`).

2. **Handles are stable; pointers are not.** All cross-references use `hEntity`,
   `hParam`, etc. — never raw pointers. Use `SK.GetEntity(he)` / `SK.entity.FindById(he)`
   to resolve. See `src/dsc.h:383` and `src/sketch.h:61–120`.

3. **`Generate::DIRTY` is the default trigger.** Call `SS.MarkGroupDirty(hg)` to queue a
   solve, then `SS.ScheduleGenerateAll()`. `GenerateAll(ALL)` forces everything. See
   `src/generate.cpp` and `src/solvespace.h:689`.

4. **`TextWindow::Printf` is not `std::printf`.** It accepts custom format codes for
   colors, links, and callbacks. See `src/textwin.cpp:401–480` for full code table.

5. **The `Sketch` class is declared with `class`, not `struct`.** Searching for
   `struct Sketch` returns nothing. Use `class Sketch` in `src/solvespace.h`.

6. **Entity types ≠ Request types.** A `Request::Type::LINE_SEGMENT` generates an
   `EntityBase::Type::LINE_SEGMENT` plus two `POINT_IN_3D` entities. See
   `src/request.cpp` and `src/entity.cpp`.

7. **Group `solved.how` holds the `SolveResult`.** Check
   `g->solved.how == SolveResult::OKAY` or `g->IsSolvedOkay()` after solve. See
   `src/sketch.h` (`Group::solved` struct).

8. **`System` is owned by `SolveSpaceUI`.** Access via `SS.sys` (ref) or `SS.pSys`
   (pointer). It is populated per-group in `WriteEqSystemForGroup()` before each solve.

---

## File Size Reference (load cost estimation)

| File | Size | Load priority |
|------|------|---------------|
| `src/solvespace.h` | ~24KB | Always |
| `src/sketch.h` | ~32KB | Always |
| `src/dsc.h` | ~21KB | Always |
| `src/ui.h` | ~28KB | UI tasks |
| `src/system.cpp` | ~21KB | Solver tasks |
| `src/constrainteq.cpp` | ~42KB | Constraint tasks |
| `src/constraint.cpp` | ~44KB | Constraint tasks |
| `src/group.cpp` | ~49KB | Group/mesh tasks |
| `src/graphicswin.cpp` | ~62KB | Graphics/mouse tasks |
| `src/mouse.cpp` | ~62KB | Mouse/drag tasks |
| `src/textscreens.cpp` | ~37KB | Text UI tasks |
| `src/generate.cpp` | ~20KB | Solve pipeline tasks |
| `src/entity.cpp` | ~35KB | Entity tasks |
| `src/file.cpp` | ~42KB | File I/O tasks |
| `src/platform/gui.h` | ~8KB | Platform tasks |
| `src/render/render.h` | moderate | Rendering tasks |

---

## Developer Docs (Internal Reference)

Two brief developer notes in `developer_docs/` are worth loading for:
- **IdList / entity remap**: `developer_docs/IdLists_Entities_and_Remap.txt`
- **Solver transforms**: `developer_docs/Solver_Transforms.txt`
