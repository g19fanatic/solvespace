# SolveSpace Documentation — Session Todos

> **Purpose**: Session tracking file for the `/init` documentation pass on SolveSpace v3.2.
> Records completed documentation, cross-links between files, and suggested areas for future
> documentation work. Start here when resuming documentation tasks.

---

## Documentation Status

### Completed Files

| File | Description | Status |
|------|-------------|--------|
| `project_info/overview.md` | High-level project purpose, features, entry points, source tree layout, build options, platform support, programmer APIs | ✅ Complete |
| `project_info/architecture.md` | Full module layout diagram, key class relationships, data flow (input → solve → render), subsystem overviews, file format notes | ✅ Complete |
| `project_info/tech-stack.md` | C++11, CMake, OpenGL 1/3, Cairo, FreeType, Eigen3, libpng, zlib, mimalloc, libdxfrw, GTK/Qt/Win32/Cocoa, WASM/Emscripten, Python bindings | ✅ Complete |
| `project_info/code-patterns.md` | 8 patterns: Handle/IdList, Printf DSL, Screen navigation, solve lifecycle, undo/redo, platform abstraction, tag-based bulk delete, global singletons (SS/SK) | ✅ Complete |
| `project_info/context-strategy.md` | LLM context optimization guide: per-task minimal load sets, key `filepath:line` entry points, subsystem boundaries, common pitfalls, file size reference | ✅ Complete |
| `project_info/subsystems/solver.md` | `System` class, Jacobian, Newton iteration, DOF calculation, substitution optimization, `SolveResult` enum, error diagnostics | ✅ Complete |
| `project_info/subsystems/sketch.md` | Sketch data model, handle system (hGroup/hEntity/hConstraint/hRequest), IdList, Group/Request/Entity/Constraint/Param types and lifecycles | ✅ Complete |
| `project_info/subsystems/ui.md` | TextWindow (Printf DSL, Screen enum, Edit enum, link callbacks) and GraphicsWindow (projection, Selection/Hover, Pending ops, mouse flow, display toggles) | ✅ Complete |
| `project_info/subsystems/platform.md` | Platform abstraction layer: gui.h/platform.h interfaces, Timer, Settings, FileDialog, Window, rendering backends, CLI commands, utility subsystems | ✅ Complete |
| `project_info/subsystems/chamfer-fillet.md` | CHAMFER/FILLET group types, direct topology injection architecture, 15-step algorithm, ASSEMBLE-skip fix, chaining, Step 15 cap trim logic, all key code locations | ✅ Complete |
| `project_info/build-notes.md` | FORCE_VENDORED_LIBS cmake option, FindPkgConfig workaround for cmake 3.22, libjson-c-dev startup fix, standard build recipe | ✅ Complete |
| `project_info/todos.md` | This file — session tracking, cross-links, suggestions | ✅ Complete |

---

## Cross-Link Map

This map shows which files reference each other for quick navigation:

```
overview.md
  → architecture.md        (module layout, data flow)
  → tech-stack.md          (build dependencies)
  → code-patterns.md       (coding patterns)
  → context-strategy.md    (LLM context optimization)
  → subsystems/            (deep-dives)

architecture.md
  → subsystems/solver.md   (solver subsystem detail)
  → subsystems/sketch.md   (sketch data model detail)
  → subsystems/ui.md       (UI subsystem detail)
  → subsystems/platform.md (platform abstraction detail)
  → code-patterns.md       (cross-cutting patterns)

context-strategy.md
  → overview.md            (project orientation)
  → architecture.md        (module map)
  → code-patterns.md       (pattern usage)
  (all subsystem files implicitly referenced via per-task load sets)

code-patterns.md
  → subsystems/solver.md   (solve lifecycle)
  → subsystems/sketch.md   (handle/IdList)
  → subsystems/ui.md       (Printf DSL, Screen enum)
  → subsystems/platform.md (platform abstraction)

subsystems/solver.md
  → code-patterns.md       (solve lifecycle pattern)
  → subsystems/sketch.md   (handle system, equation generation)
  → architecture.md        (data flow diagram)

subsystems/sketch.md
  → subsystems/solver.md   (param/equation lifecycle)
  → code-patterns.md       (Handle/IdList, tag-delete patterns)
  → architecture.md        (data flow)

subsystems/chamfer-fillet.md
  → architecture.md        (srf/ module, NURBS/surface subsystem)
  → subsystems/sketch.md   (Group::Type enum, remap constants)
  → code-patterns.md       (Pattern 9: direct topology injection)
  → context-strategy.md    (Chamfer/Fillet Development load set)
  → build-notes.md         (build/test instructions)

subsystems/ui.md
  → subsystems/platform.md (Platform::Window, event types)
  → code-patterns.md       (Printf DSL, Screen navigation patterns)
  → architecture.md        (data flow to TextWindow)

subsystems/platform.md
  → subsystems/ui.md       (how GW/TW consume Platform::Window)
  → architecture.md        (platform layer in module diagram)
  → context-strategy.md    (platform task load sets)
```

---

## Suggested Future Documentation Tasks

These topics are worth documenting in future sessions. Prioritized by LLM usefulness:

### High Priority

- [ ] **`project_info/subsystems/render.md`** — Deep-dive into the `Canvas` API (`src/render/render.h`):
  `Canvas`, `ViewportCanvas`, `BatchCanvas` interface methods; Camera projection math; how `Draw()` scene assembly works; `Camera` `offset`/`scale`/`projRight`/`projUp` fields; lighting model; comparison of GL1 vs GL3 renderers; `CairoPixmapRenderer` for offline rendering. Key files: `src/render/render.h`, `src/render/rendergl3.cpp`, `src/draw.cpp`.

- [ ] **`project_info/subsystems/export.md`** — Export pipeline documentation: format dispatcher in `src/export.cpp`; vector export formats (SVG/DXF/EPS/PDF/HPGL) in `src/exportvector.cpp`; STEP export in `src/exportstep.cpp`; STL/mesh export; how the CLI export commands use the same pipeline; key classes `VectorFileWriter`, `DxfFileWriter`, `SvgFileWriter`. Key files: `src/export.cpp`, `src/exportvector.cpp`, `src/exportstep.cpp`.

- [ ] **`project_info/subsystems/nurbs.md`** — NURBS/B-rep surface subsystem (`src/srf/`): `SSurface`, `SShell`, `SBezier` types; shell boolean operations (union/difference/intersection); surface evaluation pipeline; triangulation; ray-surface intersection (used for face selection); `curve.cpp` intersection curves. Key files: `src/srf/surface.h`, `src/srf/boolean.cpp`, `src/srf/triangulate.cpp`.

- [ ] **`project_info/subsystems/groups.md`** — Group generation pipeline detail: per-type `Generate()` implementations (EXTRUDE, LATHE, REVOLVE, HELIX, ROTATE, TRANSLATE, LINKED); how `opA`/`opB` group references work for booleans; `GenerateLoops()` polygon assembly; `remap` table for step-and-repeat; mesh accumulation (`runningMesh`/`runningShell`). Key files: `src/group.cpp`, `src/groupmesh.cpp`, `src/generate.cpp`.

### Medium Priority

- [ ] **`project_info/subsystems/file-format.md`** — `.slvs` file format specification: text-based key=value line format; `SAVED[]` table mapping (`src/solvespace.h`); load/save round-trip (`src/file.cpp`); versioning and migration; binary embedded resources within `.slvs`; linked file references and `LINKED` groups.

- [ ] **`project_info/subsystems/expressions.md`** — Symbolic algebra system (`src/expr.h` / `src/expr.cpp`): `Expr` tree structure and node types; `PartialWrt()` for analytical Jacobian differentiation; `FoldConstants()` optimization; `DeepCopyWithParamsAsPointers()` for faster evaluation; `ExprVector` / `ExprQuaternion` composite types; how expressions are used in `constrainteq.cpp`.

- [ ] **`project_info/subsystems/c-api.md`** — Public C API (`include/slvs.h`): `Slvs_MakeParam`, `Slvs_AddEntity`, `Slvs_AddConstraint`, `Slvs_Solve`; how the API bridges to internal `System`; Python binding structure in `python/`; usage examples in `exposed/CDemo.c`.

- [ ] **`project_info/subsystems/style.md`** — Style system: `Style` struct in `src/sketch.h`; default style constants; how entities/requests reference styles; color/line-width/stipple rendering; `hStyle::ACTIVE_GRP`, `CONSTRUCTION`, `DATUM` built-in styles; `src/style.cpp`.

### Lower Priority

- [ ] **`project_info/subsystems/mesh.md`** — `SMesh` (triangle mesh) and `SPolygon`/`SContour` (2D polygon) types; mesh boolean with BSP trees (`src/bsp.cpp`); edge list types (`SEdgeList`, `SBezierList`); how NURBS triangulation populates `SMesh`.

- [ ] **`project_info/subsystems/ttf.md`** — TrueType text rendering (`src/ttf.cpp`): how `TTF_TEXT` requests generate bezier curves from font outlines; `TtfFont`, `TtfFontList`; font loading on each platform.

- [ ] **`project_info/import-formats.md`** — Import pipeline: DXF import (`src/importdxf.cpp` using `libdxfrw`); IDF import (`src/importidf.cpp`); mesh import STL/OBJ (`src/importmesh.cpp`); how imported geometry enters the sketch as a `LINKED` group.

- [ ] **`project_info/testing.md`** — Test harness and test structure: `test/harness.h` / `test/harness.cpp`; how tests create headless `SolveSpaceUI` instance; constraint test structure; `solvespace-testsuite` binary; how to add new tests; CI integration hints.

---

## Key Source Locations Quick Reference

> For detailed `filepath:line` references, see `project_info/context-strategy.md`.

| Topic | Primary Source | Line / Note |
|-------|---------------|-------------|
| App singleton `SS` | `src/solvespace.cpp:11` | `SolveSpaceUI SS` |
| Sketch singleton `SK` | `src/solvespace.cpp:12` | `Sketch SK` |
| `SolveResult` enum | `src/solvespace.h:44-50` | 5 values |
| `System` class | `src/solvespace.h:75-155` | MAX_UNKNOWNS=2048 |
| `GenerateAll()` | `src/solvespace.h:689` | Trigger types: DIRTY/ALL/REGEN |
| Handle types | `src/sketch.h:61-120` | hGroup/hRequest/hEntity/hConstraint/hParam |
| `IdList<T,H>` | `src/dsc.h:383` | Sorted handle-indexed container |
| `UndoStack` | `src/solvespace.h:434-471` | MAX_UNDO=100 ring buffer |
| `TextWindow::Screen` | `src/ui.h:284-294` | 10 screen values |
| `TextWindow::Printf` | `src/textwin.cpp:360` | Custom format codes |
| `GraphicsWindow::Pending` | `src/ui.h:641` | Pending op enum |
| GUI entry point | `src/platform/entrygui.cpp:14` | `main()` |
| CLI entry point | `src/platform/entrycli.cpp:1` | `main()` |
| `Platform::Window` | `src/platform/gui.h:211-280` | Virtual interface |
| `Canvas` interface | `src/render/render.h:73` | Backend-agnostic drawing |

---

## Notes for Future Sessions

1. **Build the project first** to get IDE-quality navigation. Use:
   ```sh
   mkdir build && cd build && cmake .. -DENABLE_TESTS=ON && cmake --build . -j$(nproc)
   ```

2. **Load `context-strategy.md`** at the start of any development session — it provides pre-curated minimal file sets per task type that fit in an LLM context window efficiently.

3. **`SK.entity` and `SK.param` are generated** — never user-modified directly. All user changes go via `SK.group`, `SK.request`, `SK.constraint`, `SK.style` and are regenerated by `GenerateAll()`.

4. **Handle stability**: All cross-references use typed handles (`hEntity`, `hParam`, etc.), not pointers. Resolve with `SK.GetEntity(he)`, `SK.GetParam(hp)`, etc.

5. **The `developer_docs/` directory** has two brief but useful internal notes:
   - `developer_docs/IdLists_Entities_and_Remap.txt` — IdList/entity remap system
   - `developer_docs/Solver_Transforms.txt` — solver coordinate transform details

6. **To find a constraint type's equations**, search `src/constrainteq.cpp` for the `case Constraint::Type::YOUR_TYPE:` block.

7. **Two GUI panes** — `SS.GW` (GraphicsWindow, 3D viewport) and `SS.TW` / `SS.pTW` (TextWindow, properties panel) — are created in `SolveSpaceUI::Init()`.
