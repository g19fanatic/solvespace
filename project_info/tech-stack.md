# SolveSpace Technology Stack

> Comprehensive reference of all technologies, libraries, and build systems used in SolveSpace 3.2.

## Related Documentation
- [overview.md](overview.md) — Project purpose and entry points
- [architecture.md](architecture.md) — Module layout and data flow
- [subsystems/platform.md](subsystems/platform.md) — Platform abstraction detail

---

## Build System

| Component | Details |
|-----------|---------|
| **CMake** | 3.18–3.25 required (`CMakeLists.txt:1`) |
| **C++ Standard** | C++11 required; WASM/Emscripten targets require C++17 (`CMakeLists.txt:16-17`) |
| **C Standard** | C99 (used in exposed C API) |
| **ASM** | Assembly language support enabled in `project()` declaration |
| **Generators** | Makefiles, Ninja, Xcode (macOS), Visual Studio (Windows) |

### Key CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `ENABLE_GUI` | ON | Build graphical interface |
| `USE_QT_GUI` | OFF | Use Qt instead of GTK (Linux) |
| `ENABLE_CLI` | ON (native) | Build command-line tool |
| `ENABLE_TESTS` | ON (native) | Build test suite |
| `OPENGL` | 3 | OpenGL version to use (`1` or `3`) |
| `ENABLE_OPENMP` | OFF | Parallelize geometric ops |
| `ENABLE_LTO` | OFF | Link-time optimization |
| `ENABLE_SANITIZERS` | OFF | ASan / UBSan |
| `ENABLE_COVERAGE` | OFF | gcov/llvm-cov coverage |
| `FORCE_VENDORED_Eigen3` | OFF | Always use bundled Eigen |
| `ENABLE_PYTHON_LIB` | OFF | Build Python (Cython) bindings |

Reference: `CMakeLists.txt:55-76`

---

## Core Language & Runtime

| Technology | Version | Role |
|------------|---------|------|
| **C++11** | required | All core source code |
| **GCC** | 5.0+ required | Linux/MINGW compiler |
| **Clang** | supported | macOS / optional Linux |
| **MSVC** | supported | Windows (Visual Studio) |
| **mimalloc** | vendored (`extlib/mimalloc`) | Custom allocator for temporary heap; used via `Platform::AllocTemporary` |

---

## Graphics & Rendering

| Technology | Role | Source |
|------------|------|--------|
| **OpenGL 1.x** | Legacy rendering backend | `src/render/rendergl1.cpp` |
| **OpenGL 3.x** | Modern rendering backend (default) | `src/render/rendergl3.cpp`, `src/render/gl3shader.cpp` |
| **ANGLE** | OpenGL ES over D3D9/D3D11 (Windows + OpenGL 3) | `extlib/angle/` |
| **Cairo** | 2D software rendering; CLI/headless export | `extlib/cairo/` (vendored on Win/Mac/WASM), system on Linux |
| **pixman** | Pixel compositing (Cairo dependency) | `extlib/pixman/` (vendored on Win/Mac/WASM) |

### Rendering Abstraction
All rendering goes through the `Canvas` / `ViewportCanvas` / `BatchCanvas` interfaces:
- `src/render/render.h` — base `Canvas` class, `Camera`, `Lighting`, layer model
- `src/render/render.cpp` — shared rendering logic
- `src/render/render2d.cpp` — 2D surface renderer (`SurfaceRenderer`)
- `src/render/rendercairo.cpp` — `CairoRenderer` for headless export

---

## Platform GUI Backends

Each platform has a single implementation file:

| Platform | File | Toolkit |
|----------|------|---------|
| **Linux (default)** | `src/platform/guigtk.cpp` | gtkmm-3.0 ≥ 3.18, pangomm-1.4, X11, fontconfig, json-c |
| **Linux Qt** | `src/platform/guiqt.cpp` | Qt6 (Core, Gui, OpenGLWidgets, Widgets) |
| **Windows** | `src/platform/guiwin.cpp` | Win32 API, comctl32 |
| **macOS** | `src/platform/guimac.mm` | Objective-C++, AppKit, Cocoa |
| **WASM/Browser** | `src/platform/guihtml.cpp` | Emscripten, HTML5 |
| **Headless (CLI)** | `src/platform/guinone.cpp` | No GUI; used with `solvespace-headless` |

Platform abstraction interfaces:
- `src/platform/platform.h` — `Platform::Path`, file I/O, `AllocTemporary`
- `src/platform/gui.h` — `Platform::Window`, `Timer`, `Settings`, `MenuItem`, `FileDialog`
- `src/platform/platform.cpp` — settings, font file discovery, TTF loading
- `src/platform/platformbase.cpp` — minimal base for solver-only builds

---

## Image & Font Libraries

| Library | Role | Source |
|---------|------|--------|
| **FreeType** | TrueType/OpenType font rasterization | `extlib/freetype/` (vendored Win/Mac/WASM), system on Linux |
| **libpng** | PNG image loading/saving | `extlib/libpng/` (vendored Win/Mac/WASM), system on Linux |
| **zlib** | Compression (libpng dependency, also used directly) | `extlib/zlib/` (vendored Win/Mac/WASM), system on Linux |
| **Bitstream Vera Sans** | Embedded font for UI text | `res/fonts/BitstreamVeraSans-Roman-builtin.ttf` |

---

## Math & Geometry

| Library | Role | Source |
|---------|------|--------|
| **Eigen3** | Matrix/vector math in constraint solver | `extlib/eigen/` (vendored fallback), or system |
| **Custom BSP** | Binary space partitioning for mesh ops | `src/bsp.cpp` |
| **NURBS / SRF** | Rational polynomial surface math | `src/srf/` (surface.h, ratpoly.cpp, curve.cpp, etc.) |

---

## File Format Support

| Format | Direction | File |
|--------|-----------|------|
| **DXF** | Import/Export | `extlib/libdxfrw/`, `src/importdxf.cpp`, `src/exportvector.cpp` |
| **IDF** | Import | `src/importidf.cpp` |
| **STEP** | Export | `src/exportstep.cpp` |
| **STL / Mesh** | Import/Export | `src/importmesh.cpp`, `src/export.cpp` |
| **SVG / PDF / EPS / HPGL / G-Code** | Export (vector) | `src/exportvector.cpp` |
| **PNG** | Export (raster) | `src/export.cpp` via Cairo/pixmap |
| **SLVS** | Native save format (JSON-like text) | `src/file.cpp` |

---

## Internationalization

| Tool | Role |
|------|------|
| **gettext** (`xgettext`, `msginit`, `msgmerge`) | Translation extraction and merging |
| **PO/POT files** | `res/locales/*.po`, `res/messages.pot` |
| **Macros** | `_()`, `N_()`, `C_()`, `CN_()` throughout source |

---

## External / Public APIs

### C API (`libslvs`)
- Header: `include/slvs.h`
- Implementation: `src/slvs/lib.cpp`
- Shared library: `libslvs.so` / `slvs.dll`
- Purpose: Expose constraint solver to external programs without SolveSpace internals
- C demo: `exposed/CDemo.c`, documentation: `exposed/DOC.txt`
- Key types: `Slvs_Param`, `Slvs_Entity`, `Slvs_Constraint`, `Slvs_System`
- Key result codes (`include/slvs.h`): `SLVS_RESULT_OKAY=0`, `SLVS_RESULT_INCONSISTENT=1`, `SLVS_RESULT_DIDNT_CONVERGE=2`, `SLVS_RESULT_TOO_MANY_UNKNOWNS=3`, `SLVS_RESULT_REDUNDANT_OKAY=4`

### Python Bindings (`slvs` package)
- Build: Cython (`src/slvs/lib.pyx`) compiled via scikit-build-core (`pyproject.toml`)
- Package: `python/slvs/` — `__init__.py`, `solvespace.pyi`
- Python ≥ 3.7 supported
- Key types: `ConstraintType`, `EntityType`, `ResultFlag` enums; `Slvs_Entity`, `Slvs_Constraint` TypedDicts
- Entry: `python/slvs/__init__.py`

### JavaScript / WASM API
- Build: Emscripten + `--bind` (embind), requires Emscripten ≥ 4.0.8
- Source: `src/slvs/jslib.cpp`
- Output: `slvs.js` (single-file WASM bundle)
- TypeScript types: `js/slvs.d.ts`
- Usage example: `js/README.md`
- Exposes same constraint solver API (entities, constraints, `solveSketch`, `clearSketch`)

---

## Testing

| Component | Details |
|-----------|---------|
| **Test binary** | `solvespace-testsuite` |
| **Runner** | `ctest --output-on-failure` |
| **Test dir** | `test/` (constraints, requests, groups, core utilities) |
| **Enabled via** | `ENABLE_TESTS=ON` (default for native builds) |

---

## Optional / Peripheral

| Technology | Role | Condition |
|------------|------|-----------|
| **OpenMP** | Parallelize geometric operations | `ENABLE_OPENMP=ON` |
| **SpaceWare / 3DConnexion** | 6-DOF input device support | `extlib/si/` (Windows), `find_package(SpaceWare)` (Linux) |
| **fontconfig** | System font discovery (Linux) | Required on Linux GUI builds |
| **Backtrace** | Stack traces on crash | Optional (`find_package(Backtrace)`) |

---

## Platform-Specific Notes

- **Windows**: Uses ANGLE (D3D9/D3D11 backend for OpenGL 3), static runtime (`/MT`), `LARGE_ADDRESS_AWARE`; all deps vendored
- **macOS**: All deps vendored; Objective-C++ for Cocoa backend; deployment target 10.12; ARC enabled; Hardened Runtime
- **Linux**: System libraries preferred (GTK, Qt, Cairo, FreeType, libpng, zlib); fontconfig + json-c required for GTK GUI
- **FreeBSD**: Uses libc++; otherwise similar to Linux
- **WASM**: Emscripten ≥ 4.0.8; `ASYNCIFY=1`, `ALLOW_MEMORY_GROWTH=1`; all deps vendored; `HEADLESS` not set (GUI via HTML)

---

## Compiler Requirements

| Compiler | Minimum | Notes |
|----------|---------|-------|
| GCC | 5.0 | `CMakeLists.txt:92-96` |
| Clang | any modern | `-Wfloat-conversion` added |
| MSVC | VS 16.9+ for sanitizers | `/MP`, `/we4062` (`-Werror=switch` equivalent) |

All builds enforce `-Werror=switch` (or `/we4062`) to ensure exhaustive enum handling.
