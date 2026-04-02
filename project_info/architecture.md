# SolveSpace Architecture

> **Purpose**: Module layout, data flow, and key class relationships for the SolveSpace parametric CAD codebase.
> **Cross-references**: [overview.md](overview.md) | [subsystems/solver.md](subsystems/solver.md) | [subsystems/sketch.md](subsystems/sketch.md) | [subsystems/ui.md](subsystems/ui.md) | [subsystems/platform.md](subsystems/platform.md)

---

## Module Layout

```
solvespace/
├── src/
│   ├── solvespace.h          # Top-level declarations: SolveSpaceUI, Sketch, System, SolveResult
│   ├── solvespace.cpp        # SolveSpaceUI implementation: Init, file I/O, export, menus
│   ├── sketch.h              # Sketch data model: Group, Request, Entity, Constraint, Style, handles
│   ├── dsc.h                 # Core data structures: Vector, Quaternion, IdList<T,H>, List<T>, RgbaColor
│   ├── ui.h                  # UI class declarations: TextWindow, GraphicsWindow, Command enum
│   ├── expr.h / expr.cpp     # Symbolic expression algebra (used in constraint equations)
│   ├── param.h               # Param (a named double in the solve system)
│   ├── handle.h              # Handle type infrastructure (hEntity, hParam, etc.)
│   │
│   ├── generate.cpp          # GenerateAll(): per-group solve → entity generation → mesh/NURBS
│   ├── group.cpp             # Group methods: Generate(), GenerateLoops(), GenerateShellAndMesh()
│   ├── groupmesh.cpp         # Mesh generation for groups (extrude, lathe, revolve, boolean)
│   ├── system.cpp            # Constraint solver: Newton's method, Jacobian, rank detection
│   ├── constrainteq.cpp      # Constraint → equation generation
│   ├── constraint.cpp        # Constraint drawing, static factory methods
│   ├── entity.cpp            # Entity numerical evaluation, bezier generation
│   ├── request.cpp           # Request → entity parameter generation
│   │
│   ├── graphicswin.cpp       # GraphicsWindow: mouse/keyboard input, selection, drawing
│   ├── textwin.cpp           # TextWindow: Printf DSL renderer, scrolling, click handlers
│   ├── textscreens.cpp       # TextWindow screens: groups, styles, config, step-dimension
│   ├── describescreen.cpp    # TextWindow: describe selected entity/constraint
│   ├── confscreen.cpp        # TextWindow: configuration screen
│   ├── draw.cpp              # Scene drawing: entities, constraints, overlays
│   ├── drawentity.cpp        # Entity-specific drawing via Canvas API
│   ├── drawconstraint.cpp    # Constraint-specific drawing via Canvas API
│   ├── mouse.cpp             # Mouse event handling and dragging logic
│   ├── modify.cpp            # Sketch modification operations
│   ├── clipboard.cpp         # Copy/paste operations
│   ├── undoredo.cpp          # Undo/redo stack (UndoState snapshots of Sketch)
│   ├── file.cpp              # .slvs file load/save (text-based key=value format)
│   ├── export.cpp            # Export dispatch (PNG, mesh, vector, section, wireframe)
│   ├── exportvector.cpp      # Vector format writers: SVG, DXF, EPS, PDF, HPGL, G-code
│   ├── exportstep.cpp        # STEP export
│   ├── style.cpp             # Style definitions and rendering properties
│   ├── toolbar.cpp           # Toolbar draw/hit-test
│   ├── view.cpp              # View manipulation (zoom, pan, orient)
│   ├── mesh.cpp              # Polygon mesh (SMesh) operations
│   ├── bsp.cpp               # BSP tree for mesh operations
│   ├── polygon.cpp           # 2D polygon (SPolygon, SContour, SEdge) operations
│   ├── polyline.cpp          # Polyline approximation helpers
│   ├── ttf.cpp               # TrueType font rendering
│   ├── resource.cpp          # Embedded resource loading
│   ├── util.cpp              # Utility functions (math, string formatting)
│   ├── importdxf.cpp         # DXF import
│   ├── importidf.cpp         # IDF import
│   ├── importmesh.cpp        # Mesh file import (STL, OBJ, etc.)
│   │
│   ├── render/               # Rendering subsystem (backend-agnostic Canvas API)
│   │   ├── render.h          # Canvas, Camera, Lighting, BatchCanvas, ViewportCanvas interfaces
│   │   ├── render.cpp        # Canvas base implementation
│   │   ├── rendergl1.cpp     # OpenGL 1.x backend
│   │   ├── rendergl3.cpp     # OpenGL 3.x backend (shader-based)
│   │   ├── render2d.cpp      # 2D software canvas (for UI elements)
│   │   └── rendercairo.cpp   # Cairo backend (for export/print)
│   │
│   ├── srf/                  # NURBS/B-rep surface subsystem
│   │   ├── surface.h         # SSurface, SShell, SBezier, SBezierList, SBezierLoop
│   │   ├── surface.cpp       # Surface evaluation
│   │   ├── boolean.cpp       # Boolean operations on SShell (union/difference/intersect)
│   │   ├── curve.cpp         # Curve intersection, trimming
│   │   ├── chamfer.cpp       # Chamfer and fillet via direct topology injection (NEW)
│   │   ├── shell.cpp         # Shell assembly and trimming
│   │   ├── surfinter.cpp     # Surface-surface intersection
│   │   ├── triangulate.cpp   # Triangulation of NURBS faces → SMesh
│   │   ├── raycast.cpp       # Ray-surface intersection (for selection)
│   │   ├── ratpoly.cpp       # Rational polynomial arithmetic
│   │   └── merge.cpp         # Curve/surface merge helpers
│   │
│   └── platform/             # Platform abstraction layer
│       ├── platform.h        # Path, file I/O, memory arena, CLI init
│       ├── gui.h             # Platform::Gui, Window, Timer, Menu, FileDialog interfaces
│       ├── gui.cpp           # Common gui helpers
│       ├── platform.cpp      # Platform-independent implementations
│       ├── platformbase.cpp  # Base implementations shared across platforms
│       ├── entrygui.cpp      # GUI entry point (calls Platform::Gui::Create → SS.Init())
│       ├── entrycli.cpp      # CLI entry point (headless export/analysis)
│       ├── guigtk.cpp        # GTK3 backend
│       ├── guiqt.cpp         # Qt5/6 backend
│       ├── guiwin.cpp        # Win32 backend
│       ├── guimac.mm         # Cocoa (macOS) backend
│       ├── guihtml.cpp       # HTML/WASM (Emscripten) backend
│       └── guinone.cpp       # Stub backend (headless/testing)
```

---

## Global Singletons

Two global objects own all application state:

| Global | Type | Header | Purpose |
|--------|------|--------|---------|
| `SS` | `SolveSpaceUI` | `src/solvespace.h` | Application controller: UI, file I/O, solve trigger, undo stack |
| `SK` | `Sketch` | `src/solvespace.h` | All sketch data: groups, requests, entities, constraints, params, styles |

`SS` contains `SS.sys` (`System`) for constraint solving and `SS.TW` (`TextWindow`) + `SS.GW` (`GraphicsWindow`) for the two UI panes.

---

## Key Class Relationships

```
SolveSpaceUI (SS)                    Sketch (SK)
├── GraphicsWindow GW                ├── IdList<Group,hGroup>       group
├── TextWindow& TW                   ├── List<hGroup>               groupOrder
├── System& sys                      ├── IdList<Constraint,hConstr> constraint
├── UndoStack undo, redo             ├── IdList<Request,hRequest>   request
└── TtfFontList fonts                ├── IdList<Style,hStyle>       style
                                     ├── IdList<Entity,hEntity>     entity   [generated]
System                               └── ParamList                  param    [generated]
├── EntityList   entity
├── ParamList    param
├── IdList<Equation,hEquation> eq
└── Jacobian matrix (Eigen sparse)

Group                                Entity (EntityBase)
├── Group::Type (DRAWING_3D,         ├── EntityBase::Type (POINT_IN_3D,
│   EXTRUDE, LATHE, REVOLVE, ...)    │   LINE_SEGMENT, CIRCLE, ARC, ...)
├── hEntity activeWorkplane          ├── hGroup group
├── SShell thisShell, runningShell   ├── hEntity workplane
├── SMesh  thisMesh,  runningMesh    ├── hEntity point[12]
└── solved.{how, dof, ...}           └── hParam  param[8]

Constraint (ConstraintBase)
├── Constraint::Type (POINTS_COINCIDENT,
│   PT_PT_DISTANCE, ANGLE, PARALLEL, ...)
├── hEntity ptA, ptB, entityA..D
└── double valA
```

---

## Data Flow: User Input → Constraint Solve → Render

```
┌──────────────────────────────────────────────────────────────────────────┐
│  User Input (mouse/keyboard)                                             │
│  GraphicsWindow::MouseLeftDown() / MouseMoved()     src/mouse.cpp        │
│  GraphicsWindow::KeyboardEvent()                    src/graphicswin.cpp  │
└───────────────────┬──────────────────────────────────────────────────────┘
                    │ Modifies SK (requests/constraints/params)
                    │ Calls SS.MarkGroupDirty() → SS.ScheduleGenerateAll()
                    ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  SolveSpaceUI::GenerateAll()                        src/generate.cpp     │
│  ─────────────────────────────────────────────────────────────────────  │
│  For each Group in SK.groupOrder (in order):                             │
│    1. PruneOrphans() — remove dangling refs                              │
│    2. Request::Generate() — create Entity + Param entries in SK          │
│    3. Group::GenerateEquations() — fill System::eq from constraints      │
│    4. SS.SolveGroup(hg) → System::Solve()                                │
│       - WriteJacobian() — build sparse Jacobian matrix                   │
│       - NewtonSolve() — iterative Newton's method (Eigen SparseQR)       │
│       - Returns SolveResult: OKAY / DIDNT_CONVERGE / REDUNDANT_* / ...  │
│    5. Group::Generate() — compute Entity actual positions from params    │
│    6. Group::GenerateLoops() — assemble polygon loops from edges         │
│    7. Group::GenerateShellAndMesh() — boolean ops on SShell/SMesh        │
└───────────────────┬──────────────────────────────────────────────────────┘
                    │ SK.entity and SK.param now have numerical values
                    ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  Render                                                                  │
│  GraphicsWindow::Paint() → Draw(canvas)             src/graphicswin.cpp  │
│    Entity::Draw(canvas)                             src/drawentity.cpp   │
│    Group::DrawMesh(canvas)                          src/group.cpp        │
│    Constraint::Draw(canvas)                         src/drawconstraint.cpp│
│  Canvas API → OpenGL 1/3 / Cairo backend            src/render/          │
└──────────────────────────────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  TextWindow::Show() → Printf() DSL renders group/entity/constraint info  │
│  Screens: LIST_OF_GROUPS, GROUP_INFO, CONFIGURATION, STYLE_INFO, ...     │
│  Defined in: src/textwin.cpp, src/textscreens.cpp, src/describescreen.cpp│
└──────────────────────────────────────────────────────────────────────────┘
```

---

## Subsystem Overview

### Solver (`src/system.cpp`, `src/constrainteq.cpp`)
- `System::Solve()` performs Newton's method on the constraint equations
- Each `Constraint` generates `Equation` entries via `GenerateEquations()` (`src/constrainteq.cpp`)
- The Jacobian is built symbolically (`Expr*`), then evaluated numerically
- Uses Eigen's `SparseQR` for the linear solve step
- Returns `SolveResult` enum (`src/solvespace.h:44-50`)
- See [subsystems/solver.md](subsystems/solver.md) for details

### Sketch Data Model (`src/sketch.h`, `src/dsc.h`)
- Handle types: `hGroup`, `hEntity`, `hParam`, `hConstraint`, `hRequest` — all `uint32_t` wrappers
- `IdList<T,H>` — sorted list with O(log n) lookup by handle (`src/dsc.h`)
- `SK.group`, `SK.request`, `SK.constraint` are user-editable; `SK.entity`, `SK.param` are generated
- See [subsystems/sketch.md](subsystems/sketch.md) for details

### Rendering (`src/render/`)
- Backend-agnostic `Canvas` interface in `src/render/render.h`
- Backends: OpenGL 1 (`rendergl1.cpp`), OpenGL 3 (`rendergl3.cpp`), Cairo (`rendercairo.cpp`)
- `Camera` class handles 3D→2D projection (axonometric with optional perspective tangent)
- `ViewportCanvas` wraps a platform window; `BatchCanvas` allows deferred/cached draw calls

### NURBS/Surface (`src/srf/`)
- `SShell` — collection of `SSurface` patches forming a closed solid
- `Group::GenerateShellAndMesh()` calls boolean operations (`srf/boolean.cpp`) per group
- `SMesh` — triangle mesh derived from `SShell` triangulation (`srf/triangulate.cpp`)
- Both `thisShell`/`thisMesh` (per-group) and `runningShell`/`runningMesh` (accumulated) are maintained

### Platform Abstraction (`src/platform/`)
- `Platform::Gui` interface in `src/platform/gui.h` — windows, menus, dialogs, timers
- One backend per OS: GTK (`guigtk.cpp`), Win32 (`guiwin.cpp`), Cocoa (`guimac.mm`), Qt (`guiqt.cpp`), HTML/WASM (`guihtml.cpp`)
- Entry points: `src/platform/entrygui.cpp` (GUI), `src/platform/entrycli.cpp` (headless CLI)
- See [subsystems/platform.md](subsystems/platform.md) for details

---

## Generate Enum (Trigger Types)

Defined at `src/solvespace.h` in `SolveSpaceUI::Generate`:

| Value | Meaning |
|-------|---------|
| `DIRTY` | Re-solve only groups marked dirty (default) |
| `ALL` | Re-solve all groups unconditionally |
| `REGEN` | Regenerate all entities without solving |
| `UNTIL_ACTIVE` | Solve only up to the active group (for perf) |

---

## File Format

- Extension: `.slvs` (defined at `src/solvespace.h` in `SolveSpaceUI::SKETCH_EXT`)
- Text-based key=value format, loaded/saved in `src/file.cpp` via `SaveUsingTable()` / `LoadUsingTable()`
- The `SAVED[]` table in `SolveSpaceUI` maps field names to C++ struct members
