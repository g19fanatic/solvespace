# SolveSpace — Project Overview

## Purpose

SolveSpace is a **parametric 2D/3D CAD tool** built in C++11. It allows users to
draw geometry, apply geometric constraints (distances, angles, coincidences, etc.),
and have the solver automatically enforce those constraints. The same solver
underpins both 2D sketching and 3D solid modeling via extrusion, revolution, and
boolean operations.

- **Website**: https://solvespace.com
- **Repository**: https://github.com/solvespace/solvespace
- **Version**: 3.2 (see `CMakeLists.txt:57`)
- **Git hash at this checkout**: b524199e (fix: Remove debug fprintf instrumentation from chamfer/fillet)
- **License**: GNU GPL v3 or later (`COPYING.txt`)

---

## Key Features

| Feature | Description |
|---|---|
| Parametric constraint solving | Distance, angle, parallel, coincident, symmetric, etc. |
| 2D sketching | Workplane-based 2D drawing with constraint DOF readout |
| 3D solid modeling | Extrude, revolve, helix, boolean union/difference/intersection |
| Chamfer & Fillet | Edge chamfer/fillet on flat faces via direct topology injection |
| Assembly support | Link `.slvs` files as components; constrain assembly positions |
| Export formats | PNG thumbnail, DXF/SVG/EPS/PDF vector, STEP/STL/OBJ/3MF mesh, NURBS surfaces |
| CLI headless mode | Batch export and regeneration via command line |
| Programmable C API | `include/slvs.h` for embedding the solver in other tools |
| Python bindings | `python/` package wrapping the C API |
| WASM/Web build | Experimental browser-based version via Emscripten (`src/platform/guihtml.cpp`) |

---

## Entry Points

### GUI Application
**File**: `src/platform/entrygui.cpp`

```
main() → Platform::InitGui() → Platform::Open3DConnexion()
       → SS.Init()           (loads settings, creates windows)
       → SS.Load(file)       (optional, if file passed on cmdline)
       → Platform::RunGui()  (event loop)
       → SS.Clear() / SK.Clear() / Platform::ClearGui()
```

Key references:
- `src/platform/entrygui.cpp:14` — `main()` for graphical interface
- `src/solvespace.cpp:11` — global singleton `SS` (SolveSpaceUI)
- `src/solvespace.cpp:12` — global singleton `SK` (Sketch)
- `src/solvespace.cpp:14` — `SolveSpaceUI::Init()` implementation

### CLI Tool
**File**: `src/platform/entrycli.cpp`

Supports commands: `version`, `thumbnail`, `export-view`, `export-wireframe`,
`export-mesh`, `export-surfaces`, `regenerate`.

```
main() → Platform::InitCli() → RunCommand(args)
       → SS.Init() → SS.LoadFromFile() → SS.AfterNewFile()
       → runner(outputFile)
       → SK.Clear() / SS.Clear()
```

Key references:
- `src/platform/entrycli.cpp:1` — CLI main entry
- `src/platform/entrycli.cpp:13` — `ShowUsage()` (all CLI commands documented)

---

## Global Singletons

Two global objects anchor the application state (both in `SolveSpace` namespace):

| Symbol | Type | Location | Purpose |
|--------|------|----------|---------|
| `SS` | `SolveSpaceUI` | `src/solvespace.cpp:11` | Application controller: windows, settings, undo/redo, generate |
| `SK` | `Sketch` | `src/solvespace.cpp:12` | All sketch data: groups, entities, constraints, requests, params |

`SolveSpaceUI` is declared at `src/solvespace.h:427`. It holds:
- `GW` — `GraphicsWindow` (3D viewport)
- `TW` / `*pTW` — `TextWindow` (property/text panel)
- `undo` / `redo` — `UndoStack` (up to 100 states)
- All rendering and export settings

---

## Source Tree Layout

```
solvespace/
├── src/                     Main C++ source
│   ├── solvespace.h         Top-level declarations; SolveSpaceUI class (line 427)
│   ├── solvespace.cpp       SS/SK globals, Init(), GenerateAll(), file I/O
│   ├── sketch.h             Sketch data model: Group, Entity, Constraint, Request
│   ├── ui.h                 TextWindow, GraphicsWindow declarations
│   ├── system.cpp           Constraint solver (Newton-Raphson, Jacobian)
│   ├── generate.cpp         Group regeneration orchestration
│   ├── group.cpp            Group types and mesh generation
│   ├── entity.cpp           Entity geometry evaluation
│   ├── constraint.cpp       Constraint drawing and interaction
│   ├── constrainteq.cpp     Constraint equation generation
│   ├── graphicswin.cpp      3D viewport: draw, mouse, keyboard
│   ├── textwin.cpp          Text-panel UI and Printf DSL engine
│   ├── textscreens.cpp      Individual screen renderers (Screen enum)
│   ├── mouse.cpp            Mouse event dispatch for graphics window
│   ├── file.cpp             .slvs file format read/write
│   ├── export.cpp           Export dispatcher
│   ├── exportvector.cpp     2D vector export (DXF, SVG, EPS, PDF, HPGL)
│   ├── exportstep.cpp       STEP surface export
│   ├── undoredo.cpp         Undo/redo stack management
│   ├── expr.h / expr.cpp    Symbolic expression tree (for constraint equations)
│   ├── dsc.h                Core containers: List<>, IdList<>, BBox, Vector, etc.
│   ├── handle.h             Handle type trait/operator infrastructure
│   ├── param.h              Param struct (solver unknowns)
│   ├── polygon.h / .cpp     Polygon/polyline geometry utilities
│   ├── mesh.cpp             Triangle mesh operations
│   ├── bsp.cpp              Binary space partition for mesh boolean
│   ├── style.cpp            Style system (colors, line widths)
│   ├── resource.cpp         Embedded resource loader
│   ├── ttf.cpp              TrueType font rendering (for sketch text)
│   ├── platform/            Platform abstraction layer
│   │   ├── platform.h       Platform::Path, Settings, FileFilter types
│   │   ├── platform.cpp     Platform-independent platform utilities
│   │   ├── gui.h            Platform::Gui, Window, Timer, Menu abstractions
│   │   ├── gui.cpp          Shared GUI logic
│   │   ├── guigtk.cpp       GTK3 implementation (~56KB)
│   │   ├── guiqt.cpp        Qt6 implementation
│   │   ├── guiwin.cpp       Win32 implementation
│   │   ├── guimac.mm        macOS/Cocoa implementation
│   │   ├── guihtml.cpp      Emscripten/WASM implementation
│   │   ├── guinone.cpp      Headless (no GUI) stub
│   │   ├── entrygui.cpp     GUI main()
│   │   └── entrycli.cpp     CLI main()
│   ├── render/              Rendering backends
│   │   ├── render.h         Canvas/Renderer interface
│   │   ├── render.cpp       Base renderer implementation
│   │   ├── rendergl1.cpp    OpenGL 1.x renderer
│   │   ├── rendergl3.cpp    OpenGL 3.x renderer
│   │   ├── rendercairo.cpp  Cairo offscreen renderer (for CLI thumbnail)
│   │   └── render2d.cpp     2D export canvas
│   └── srf/                 NURBS/surface geometry
│       ├── surface.h        SSurface, SShell types
│       ├── surface.cpp      Surface evaluation
│       ├── boolean.cpp      Shell boolean operations
│       ├── curve.cpp        Intersection curves
│       ├── triangulate.cpp  Surface triangulation
│       └── ...
├── include/
│   └── slvs.h               Public C API for embedding the solver
├── python/                  Python bindings (wraps slvs.h)
├── exposed/                 C API implementation (exposed/CDemo.c)
├── test/                    Unit tests
│   ├── harness.cpp/.h       Test harness
│   ├── constraint/          Constraint-specific tests
│   ├── request/             Request-specific tests
│   ├── group/               Group tests
│   ├── core/                Core utility tests
│   └── analysis/            Analysis tests
├── extlib/                  Vendored dependencies
│   ├── eigen/               Eigen3 (sparse linear algebra)
│   ├── cairo/               Cairo (2D graphics for offscreen render)
│   ├── freetype/            FreeType font engine
│   ├── libpng/              PNG read/write
│   ├── zlib/                Zlib compression
│   ├── mimalloc/            Microsoft mimalloc allocator
│   └── libdxfrw/            DXF import/export library
├── js/                      JavaScript glue for WASM build
├── res/                     Application resources (icons, shaders, fonts)
├── CMakeLists.txt           Top-level build definition
├── CHANGELOG.md             Version history
├── CONTRIBUTING.md          Contribution guide
└── developer_docs/          Internal developer notes
    ├── IdLists_Entities_and_Remap.txt
    └── Solver_Transforms.txt
```

---

## Build System Summary

- **Build tool**: CMake 3.18+ (`CMakeLists.txt:2`)
- **C++ standard**: C++11 (`CMakeLists.txt:16`)
- **Key CMake options**:

| Option | Effect |
|--------|--------|
| `-DCMAKE_BUILD_TYPE=Release` | Optimized build |
| `-DENABLE_TESTS=ON` | Build test suite (`solvespace-testsuite`) |
| `-DUSE_QT_GUI=ON` | Build Qt6 GUI instead of GTK3 |
| `-DENABLE_GUI=OFF` | CLI only (no GUI) |
| `-DENABLE_OPENMP=ON` | Multi-core mesh generation |
| `-DENABLE_LTO=ON` | Link-time optimization |

- **Output binaries**:
  - `build/bin/solvespace` — GTK GUI
  - `build/bin/solvespace-qt` — Qt GUI
  - `build/bin/solvespace-cli` — CLI
  - `build/bin/solvespace-testsuite` — test runner

### Quick Build
```sh
mkdir build && cd build
cmake .. -DENABLE_TESTS=ON
cmake --build . -j$(nproc)
ctest --output-on-failure
```

---

## Platforms Supported

| Platform | GUI Backend | Notes |
|----------|-------------|-------|
| Linux | GTK3 (`guigtk.cpp`) | Default; requires gtkmm 3.16+ |
| Linux | Qt6 (`guiqt.cpp`) | `USE_QT_GUI=ON` |
| Windows | Win32 (`guiwin.cpp`) | VS2015+ or MinGW |
| macOS | Cocoa (`guimac.mm`) | macOS 10.6+ 64-bit |
| Browser | HTML/Emscripten (`guihtml.cpp`) | Experimental; many bugs |
| OpenBSD | GTK3 | Requires install before use |

---

## Programmer APIs

### C API
`include/slvs.h` — Embeddable solver API. Allows external tools to define
parameters, entities, constraints, and call the solver without the GUI.
Used by the Python bindings and the `exposed/` C demo.

### Python Bindings
`python/` — Python package wrapping the C API via ctypes/cffi. Configured
via `pyproject.toml`.

### File Format
`.slvs` files are text-based (custom line-oriented format); read/write in
`src/file.cpp`.

---

## Key Data Structures Quick Reference

| Name | File | Role |
|------|------|------|
| `SolveSpaceUI` (`SS`) | `src/solvespace.h:427` | Top-level app controller |
| `Sketch` (`SK`) | `src/solvespace.h` (near top of Sketch section) | All sketch data |
| `Group` | `src/sketch.h` | Parametric group (sketch, extrude, revolve, import…) |
| `Entity` | `src/sketch.h` | Geometric entity (point, line, arc, face…) |
| `Constraint` | `src/sketch.h` | Geometric constraint |
| `Request` | `src/sketch.h` | User-level geometry request (generates entities) |
| `System` | `src/system.cpp` | Constraint solver state machine |
| `hGroup/hEntity/hConstraint/hRequest` | `src/sketch.h` | Typed integer handles |
| `IdList<T,H>` | `src/dsc.h` | Handle-indexed collection |

---

## Cross-References

- Architecture and data flow: see `project_info/architecture.md`
- Technology stack details: see `project_info/tech-stack.md`
- Code patterns (handles, DSL, screens): see `project_info/code-patterns.md`
- LLM context optimization: see `project_info/context-strategy.md`
- Subsystem deep-dives: `project_info/subsystems/`
