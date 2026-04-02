# Platform Abstraction Layer

> **Purpose**: Describes the `SolveSpace::Platform` namespace that insulates all OS-specific
> GUI, filesystem, timer, and settings operations from the core application logic. Every
> platform provides the same set of virtual interfaces; the core never calls OS APIs directly.

## Related docs
- `project_info/subsystems/ui.md` — how `GraphicsWindow`/`TextWindow` consume the platform Window
- `project_info/architecture.md` — where the platform layer sits in the overall module diagram
- `project_info/context-strategy.md` — which platform files to load for specific tasks

---

## Source Layout

```
src/platform/
  platform.h          # Path, file I/O, InitCli, AllocTemporary
  platform.cpp        # Path manipulation, file ops, LoadResource, InitCli impls
  platformbase.cpp    # DebugPrint, AllocTemporary (mimalloc arena)
  gui.h               # All GUI abstractions (Window, Menu, Timer, FileDialog, …)
  gui.cpp             # Cross-platform shared helpers (Settings, FileFilter lists)
  entrygui.cpp        # GUI binary main() — calls InitGui/RunGui/ClearGui
  entrycli.cpp        # CLI binary main() — batch export, regenerate commands
  guigtk.cpp          # GTK 3 + OpenGL backend (~1700 lines)
  guiwin.cpp          # Win32 + DirectX/OpenGL backend (~1750 lines)
  guimac.mm           # Cocoa/AppKit + OpenGL backend (~1700 lines)
  guiqt.cpp           # Qt5/6 backend (~1050 lines)
  guihtml.cpp         # Emscripten/HTML5 Canvas backend (~1700 lines)
  guinone.cpp         # Headless/CLI stub (no window, used by solvespace-cli)
```

---

## CMake Backend Selection (`src/CMakeLists.txt`)

| Condition | Backend file |
|-----------|-------------|
| `WIN32 AND NOT USE_QT_GUI` | `guiwin.cpp` |
| `APPLE` | `guimac.mm` |
| `EMSCRIPTEN` | `guihtml.cpp` |
| `USE_QT_GUI` (any platform) | `guiqt.cpp` |
| Linux / other UNIX (default) | `guigtk.cpp` |
| CLI-only target | `guinone.cpp` |

The selection happens at lines `src/CMakeLists.txt:292-405`.

---

## Two-Header Split

### `src/platform/platform.h`
Filesystem, path utilities, resource loading, and CLI startup.

Key types and functions:

| Symbol | Description |
|--------|-------------|
| `Platform::Path` | Cross-platform path wrapper (`std::string raw`). Methods: `Expand`, `Parent`, `Join`, `RelativeTo`, `ToPortable`, `FromPortable` |
| `Platform::FileExists` | Thin wrapper over `fopen` |
| `Platform::OpenFile` | Opens file; on Windows uses `_wfopen` with UNC prefix expansion |
| `Platform::ReadFile` / `WriteFile` | Slurp/flush file to `std::string` |
| `Platform::LoadResource` | Loads bundled resource (Windows: `FindResource`; POSIX: reads from `res/` dir relative to binary) |
| `Platform::InitCli` | Returns `argv` as `std::vector<std::string>`; Windows variant uses `CommandLineToArgvW` |
| `Platform::AllocTemporary` / `FreeAllTemporary` | Thread-local mimalloc heap arena for scratch memory |

### `src/platform/gui.h`
GUI abstractions: windows, menus, timers, dialogs, file pickers, event types.

All classes in this header are pure-virtual interfaces (abstract base classes with `virtual ~X = default`). Platform backends implement them by returning `std::shared_ptr<ConcreteClass>` from factory functions.

---

## Core GUI Abstractions (`gui.h`)

### Event Types

```
Platform::MouseEvent        — Type: MOTION/PRESS/DBL_PRESS/RELEASE/SCROLL_VERT/LEAVE
                              Button: NONE/LEFT/MIDDLE/RIGHT
Platform::SixDofEvent       — 3DConnexion 6-DOF input (MOTION/PRESS/RELEASE)
Platform::KeyboardEvent     — Type: PRESS/RELEASE; Key: CHARACTER/FUNCTION
```
> `src/platform/gui.h:27-110`

### `Platform::Settings`
Key/value persistent settings store (`Freeze*` / `Thaw*` for int/float/string/bool/RgbaColor).
Factory: `Platform::GetSettings()` — returns `SettingsRef` (`shared_ptr<Settings>`).
> `src/platform/gui.h:116-140`

### `Platform::Timer`
Single-shot timer: set `onTimeout` callback, call `RunAfter(ms)`.
Factory: `Platform::CreateTimer()` — returns `TimerRef`.
> `src/platform/gui.h:142-155`

### `Platform::MenuItem` / `Menu` / `MenuBar`
- `MenuItem`: set accelerator, indicator (NONE/CHECK_MARK/RADIO_MARK), enabled/active state, `onTrigger` callback
- `Menu`: `AddItem`, `AddSubMenu`, `AddSeparator`, `PopUp` (context menus), `Clear`
- `MenuBar`: `AddSubMenu`, `Clear`
- Factories: `Platform::CreateMenu()`, `Platform::GetOrCreateMainMenu(bool *unique)`
> `src/platform/gui.h:158-209`

### `Platform::Window`
The primary native window interface. Each backend creates an OpenGL-capable window.

**Key enums:**

| Enum | Values |
|------|--------|
| `Window::Kind` | `TOPLEVEL`, `TOOL` |
| `Window::Cursor` | `POINTER`, `HAND` |

**Event callbacks** (set by application code):
```cpp
std::function<void()>               onClose;
std::function<void(bool)>           onFullScreen;
std::function<bool(MouseEvent)>     onMouseEvent;
std::function<void(SixDofEvent)>    onSixDofEvent;
std::function<bool(KeyboardEvent)>  onKeyboardEvent;
std::function<void(std::string)>    onEditingDone;  // inline editor done
std::function<void(double)>         onScrollbarAdjusted;
std::function<void()>               onContextLost;  // GL context lost
std::function<void()>               onRender;       // repaint trigger
```
> `src/platform/gui.h:222-233`

**Key virtual methods:**

| Method | Purpose |
|--------|---------|
| `GetPixelDensity()` | Physical DPI |
| `GetDevicePixelRatio()` | HiDPI scale factor (logical→physical pixels) |
| `SetVisible(bool)` / `Focus()` | Show/hide and focus |
| `SetFullScreen(bool)` | Toggle fullscreen |
| `SetTitle(string)` / `SetTitleForFilename(Path)` | Window title |
| `SetMenuBar(MenuBarRef)` | Attach menu bar |
| `GetContentSize(w,h)` / `SetMinContentSize(w,h)` | Content area dimensions |
| `FreezePosition(settings,key)` / `ThawPosition` | Persist window geometry |
| `SetCursor(Cursor)` | Change mouse cursor |
| `SetTooltip(text,x,y,w,h)` | Tooltip popup |
| `ShowEditor(x,y,fontH,minW,mono,text)` / `HideEditor()` | Inline text edit overlay |
| `IsEditorVisible()` | Query editor state |
| `SetScrollbarVisible(bool)` / `ConfigureScrollbar(min,max,page)` | Scrollbar for TextWindow |
| `GetScrollbarPosition()` / `SetScrollbarPosition(pos)` | Scroll position |
| `Invalidate()` | Schedule repaint (calls `onRender`) |

Factory: `Platform::CreateWindow(kind, parentWindow)` → `WindowRef`
> `src/platform/gui.h:211-280`

### 3DConnexion Support
```cpp
void Open3DConnexion();
void Close3DConnexion();
void Request3DConnexionEventsForWindow(WindowRef window);
```
GTK stub (`guigtk.cpp:1167-1168`): empty no-ops. Win32 (`guiwin.cpp:1434`) has real implementation with full SpaceMouse SDK.
> `src/platform/gui.h:286-289`

### `Platform::MessageDialog`
Modal/non-modal alert/question dialogs.
- Types: `INFORMATION`, `QUESTION`, `WARNING`, `ERROR`
- Responses: `NONE`, `OK`, `YES`, `NO`, `CANCEL`
- `AddButton(label, response, isDefault)`, `RunModal()`, `ShowModal()` (with `onResponse` callback)

Factory: `Platform::CreateMessageDialog(parentWindow)` → `MessageDialogRef`
> `src/platform/gui.h:292-333`

### `Platform::FileDialog`
Open/save file picker.
- `SetTitle`, `SetCurrentName`, `SetFilename`, `SuggestFilename`
- `GetFilename()` → `Platform::Path`
- `AddFilter(name, extensions)` / `AddFilters(vector<FileFilter>)`
- `FreezeChoices` / `ThawChoices` — persist last-used directory
- `RunModal()` → bool

Pre-defined filter lists (in `gui.cpp`):

| Variable | Formats |
|----------|---------|
| `SolveSpaceModelFileFilters` | `.slvs` |
| `SolveSpaceLinkFileFilters` | `.slvs`, `.emn`, `.stl` |
| `RasterFileFilters` | `.png` |
| `MeshFileFilters` | `.stl` |
| `SurfaceFileFilters` | NURBS surface formats |
| `VectorFileFilters` | 2D vector formats |
| `Vector3dFileFilters` | 3D wireframe formats |
| `ImportFileFilters` | All importable |

Factories: `Platform::CreateOpenFileDialog(window)`, `Platform::CreateSaveFileDialog(window)`
> `src/platform/gui.h:359-391`

---

## Application-Level Lifecycle Functions

These four functions form the top-level GUI lifecycle, called from `entrygui.cpp`:

```cpp
// entrygui.cpp:
std::vector<std::string> args = Platform::InitGui(argc, argv);   // init toolkit
Platform::Open3DConnexion();
SS.Init();
if (args.size() >= 2) SS.Load(Platform::Path::From(args.back()));
Platform::RunGui();                                               // event loop
Platform::Close3DConnexion();
SS.Clear(); SK.Clear();
Platform::ClearGui();                                             // cleanup
```
> `src/platform/entrygui.cpp:12-33`

| Function | GTK line | Win32 line | macOS line |
|----------|----------|------------|------------|
| `InitGui` | `guigtk.cpp:1622` | `guiwin.cpp:1709` | `guimac.mm:1641` |
| `RunGui` | `guigtk.cpp:1666` | `guiwin.cpp:1724` | `guimac.mm:1674` |
| `ExitGui` | `guigtk.cpp:1670` | `guiwin.cpp:1733` | `guimac.mm:1678` |
| `ClearGui` | `guigtk.cpp:1674` | `guiwin.cpp:1737` | _(no-op/AppKit)_ |

**`InitGui`** parses arguments, initializes the native toolkit (GTK/Win32/Cocoa/Qt), and creates the two main windows (`SS.TW` and `SS.GW`).

**`RunGui`** enters the platform event loop (`gtk_main()`, `GetMessage`/`DispatchMessage`, `[NSApp run]`, etc.).

**`ExitGui`** sends a quit signal to the event loop.

---

## Rendering Backends (render subsystem connection)

The `Platform::Window::onRender` callback is called when the window needs to repaint. The platform backend then activates the OpenGL context and calls application drawing code.

### OpenGL Renderers (interactive GUI)
| Class | File | Notes |
|-------|------|-------|
| `OpenGl3Renderer` | `src/render/rendergl3.cpp:43` | OpenGL 3.3 core profile; used when GL3 available |
| `OpenGl3RendererBatch` | `src/render/rendergl3.cpp:939` | Batched draw calls for same renderer |
| `OpenGl1Renderer` | `src/render/rendergl1.cpp:169` | OpenGL 1.x fallback |

### Cairo Renderer (CLI / offline)
| Class | File | Notes |
|-------|------|-------|
| `CairoPixmapRenderer` | `src/render/render.h:388` | Used for `thumbnail` CLI command and tests |
| `CairoRenderer` | `src/render/render.h:356` | Base for all Cairo-backed rendering |

Factory: `std::shared_ptr<ViewportCanvas> CreateRenderer()` — `src/render/render.h:404`  
Returns `OpenGl3Renderer` or `OpenGl1Renderer` depending on runtime GL capability.

### Canvas Hierarchy
```
Canvas (abstract)
├── ViewportCanvas       — has SetCamera/SetLighting, used for 3D scene
│   ├── OpenGl3Renderer  — hardware-accelerated GL3
│   ├── OpenGl1Renderer  — legacy GL1
│   └── SurfaceRenderer  — software base
│       └── CairoRenderer
│           └── CairoPixmapRenderer  — off-screen PNG output
└── BatchCanvas          — deferred command buffer
    └── OpenGl3RendererBatch
```
> `src/render/render.h:73-404`

---

## CLI Entry Point (`entrycli.cpp`)

The `solvespace-cli` binary uses `Platform::InitCli` (not `InitGui`) and runs headless:
```cpp
// entrycli.cpp:main
std::vector<std::string> args = Platform::InitCli(argc, argv);
RunCommand(args);  // dispatches: thumbnail / export-view / export-mesh / etc.
```

Commands call `SS.Init()` + `SS.LoadFromFile()` + `SS.GenerateAll()` then render/export via Cairo or vector writers.

### CLI commands

| Command | Description |
|---------|-------------|
| `version` | Print version string |
| `thumbnail --size WxH --view DIR --output FILE` | Render PNG via `CairoPixmapRenderer` |
| `export-view --view DIR --output FILE` | Export 2D vector |
| `export-wireframe --output FILE` | Export 3D wireframe |
| `export-mesh --output FILE` | Export STL/triangle mesh |
| `export-surfaces --output FILE` | Export exact NURBS surfaces (STEP) |
| `regenerate` | Reload linked files, re-solve, re-save |

> `src/platform/entrycli.cpp:17-370`

---

## Utility Subsystems

### `Platform::DebugPrint`
`void DebugPrint(const char *fmt, ...)` — `stderr` on POSIX; `OutputDebugString` on Windows.
Exposed as the `dbp(...)` macro throughout codebase.
> `src/platform/platformbase.cpp:28` (Win32), `:60` (POSIX)

### Temporary Memory Arena
Thread-local mimalloc heap. Used for scratch buffers during constraint solving and mesh operations.
```cpp
void *AllocTemporary(size_t size);   // alloc from thread-local heap
void FreeAllTemporary();              // destroy+recreate entire heap
```
> `src/platform/platformbase.cpp:73-94`

### Resource Loading
Bundles fonts and shaders as compiled-in resources or files adjacent to the binary:
- **Windows**: `FindResource` / `LoadResource` (RC file embeds `res/` content)
- **macOS**: `CFBundleCopyResourceURL` → then file path
- **Linux/POSIX**: searches `../res/`, `../share/solvespace/`, then `UNIX_DATADIR`
- **Emscripten**: `res/<name>` relative path in virtual FS

> `src/platform/platform.cpp:219-364`
