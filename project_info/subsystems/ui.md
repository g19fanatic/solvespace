# UI Subsystem — TextWindow & GraphicsWindow

## Purpose

The UI subsystem provides SolveSpace's two-window user interface:
- **GraphicsWindow** — the 3D/2D viewport for sketch editing, selection, and rendering
- **TextWindow** — the side-panel browser showing groups, constraints, styles, and configuration

Both classes are declared in `src/ui.h` (889 lines) and implemented across:

| File | Contents |
|------|----------|
| `src/graphicswin.cpp` | GraphicsWindow mouse/keyboard/draw logic |
| `src/textwin.cpp` | TextWindow Printf rendering, screen drawing, edit control |
| `src/textscreens.cpp` | All `Show*()` screen implementations (GroupInfo, Config, etc.) |
| `src/toolbar.cpp` | GraphicsWindow toolbar draw/hit-test |
| `src/ui.h` | Declarations for both classes + Command enum + Locale |

---

## Command Enum

`src/ui.h:82-196` — `enum class Command : uint32_t`

Groups all menu actions into a single enum dispatched via `ActivateCommand(Command id)`.

| Category | Range | Examples |
|----------|-------|---------|
| File | 100–114 | NEW, OPEN, SAVE, EXPORT_* |
| View | 115–135 | ZOOM_IN, SHOW_GRID, PERSPECTIVE_PROJ |
| Edit | 136–155 | UNDO, REDO, CUT, COPY, PASTE, DELETE |
| Request | 156–171 | LINE_SEGMENT, CIRCLE, ARC, CUBIC, TTF_TEXT |
| Group | 172–182 | GROUP_EXTRUDE, GROUP_LATHE, GROUP_LINK |
| Constrain | 183–202 | DISTANCE_DIA, ANGLE, EQUAL, HORIZONTAL |
| Analyze | 203–213 | VOLUME, NAKED_EDGES, SHOW_DOF |
| Help | 214–218 | LOCALE, WEBSITE, ABOUT |

---

## TextWindow

### Overview

`src/ui.h:198` — `class TextWindow`

A custom monospace character-cell terminal rendered via OpenGL. Content is built by calling `Printf()` repeatedly to populate a 2D text buffer, then rendered per-frame. Clicking a "link" in the text triggers a callback.

### Buffer Model

```
text[MAX_ROWS][MAX_COLS]      // uint32_t character array  src/ui.h:214
meta[MAX_ROWS][MAX_COLS]      // per-cell: fg, bg, link, data, f, h callbacks
top[MAX_ROWS]                 // y-position in half-line units
rows                          // current row count
scrollPos                     // scroll offset in half-row units
```

- `MAX_ROWS = 4000`, `MAX_COLS = 100`, `MIN_COLS = 45` (`src/ui.h:199-203`)
- `CHAR_WIDTH_ = 9`, `CHAR_HEIGHT = 16`, `LINE_HEIGHT = 20`, `LEFT_MARGIN = 6` (`src/ui.h:209-214`)
- Half-line rows: `top[r] = top[r-1] + (halfLine ? 3 : 2)` — supports compact spacing

### Printf DSL

`src/ui.h:280` (declaration), `src/textwin.cpp:360` (implementation)

Signature: `void Printf(bool halfLine, const char *fmt, ...)`

Custom format codes parsed in the switch at `src/textwin.cpp:395`:

| Code | Type | Effect |
|------|------|--------|
| `%d` | int | Decimal integer |
| `%x` | uint | Hex, 8 chars (`%08x`) |
| `%@` | double | Float `%.2f` |
| `%2` | double | `"%.2f"` with leading space if positive |
| `%3` | double | `"%.3f"` with leading space if positive |
| `%#` | double | Float `%.3f` (no leading space) |
| `%s` | char* | String |
| `%c` | int | Single char (0 → empty) |
| `%E` | — | Reset foreground color + link to default |
| `%Fx` | — | Set foreground color to named color `x` |
| `%Bx` | — | Set background color to named color `x` |
| `%Fp` | int | Set fg color from int arg |
| `%Bp` | int | Set bg color from int arg |
| `%Fz` | RgbaColor* | Set fg to RGB pointer |
| `%Bz` | RgbaColor* | Set bg to RGB pointer |
| `%Lx` | — | Set link ID to char `x` |
| `%Lp` | int | Set link ID from int arg |
| `%f` | fn ptr | Set click callback (`LinkFunction*`) |
| `%h` | fn ptr | Set hover callback (`LinkFunction*`) |
| `%D` | uint32 | Set link data (`uint32_t`) |

Color codes for `%Fx`/`%Bx` are single chars mapping to `fgColors[]`/`bgColors[]` tables.

Built-in checkbox/radio glyphs (PUA Unicode):
- `CHECK_FALSE` = U+E000, `CHECK_TRUE` = U+E001
- `RADIO_FALSE` = U+E002, `RADIO_TRUE` = U+E003

### Screen Navigation

`src/ui.h:283` — `enum class Screen : uint32_t` (10 values)

| Value | Name | Shows |
|-------|------|-------|
| 0 | `LIST_OF_GROUPS` | Group list with solve status |
| 1 | `GROUP_INFO` | Active group parameters |
| 2 | `GROUP_SOLVE_INFO` | Solve failure details |
| 3 | `CONFIGURATION` | App settings panel |
| 4 | `STEP_DIMENSION` | Step-and-repeat dimension control |
| 5 | `LIST_OF_STYLES` | Style browser |
| 6 | `STYLE_INFO` | Style parameter editor |
| 7 | `PASTE_TRANSFORMED` | Paste transform controls |
| 8 | `EDIT_VIEW` | Camera/view parameter editor |
| 9 | `TANGENT_ARC` | Tangent arc radius control |

Navigation via `GoToScreen(Screen s)` (`src/ui.h:419`), which sets `shown.screen` and calls `Show()`.

`ShownState shown` (`src/ui.h:295-312`) persists: current screen + active hGroup/hStyle/hConstraint + paste transform params.

### Screen Implementations

`src/ui.h:402-418` — declared methods, implemented in `src/textscreens.cpp`:

```
ShowListOfGroups()       ShowGroupInfo()        ShowGroupSolveInfo()
ShowConfiguration()      ShowListOfStyles()     ShowStyleInfo()
ShowStepDimension()      ShowPasteTransformed() ShowEditView()
ShowTangentArc()         DescribeSelection()    ShowHeader(bool withNav)
```

### Edit Control

`src/ui.h:375-398` — `struct edit` + `struct editControl`

In-line text input is activated by `ShowEditControl(col, str, halfRow)` → calls `window->ShowEditor(...)`. Color picker overlay supported via `ShowEditControlWithColorPicker(col, rgb)`.

`enum class Edit : uint32_t` (`src/ui.h:315-374`) identifies what is being edited:
- Group: `TIMES_REPEATED(1)`, `GROUP_NAME(2)`, `GROUP_SCALE(3)`, `GROUP_COLOR(4)`, `GROUP_OPACITY(5)`  
- Configuration: `LIGHT_DIRECTION(100)` … `ANIMATION_SPEED(121)`
- TTF: `TTF_TEXT(300)`
- Step dim: `STEP_DIM_FINISH(400)`, `STEP_DIM_STEPS(401)`
- Styles: `STYLE_WIDTH(500)` … `STYLE_STIPPLE_PERIOD(508)`
- Paste: `PASTE_TIMES_REPEATED(600)`, `PASTE_ANGLE(601)`, `PASTE_SCALE(602)`
- View: `VIEW_SCALE(700)` … `VIEW_PROJ_UP(703)`
- Tangent arc: `TANGENT_ARC_RADIUS(800)`, `HELIX_PITCH(802)`

Edit completion dispatches via `EditControlDone(string)` → category-specific handlers:
`EditControlDoneForStyles`, `EditControlDoneForConfiguration`, `EditControlDoneForPaste`, `EditControlDoneForView` (`src/ui.h:542-546`).

### Link Callbacks (Screen* methods)

All interactive links in the text window are static `Screen*` callbacks taking `(int link, uint32_t v)`. Examples:
- `ScreenSelectGroup`, `ScreenActivateGroup`, `ScreenToggleGroupShown` — group list interaction
- `ScreenHoverEntity`, `ScreenSelectConstraint` — entity/constraint hover and selection
- `ScreenShowConfiguration`, `ScreenShowStyleInfo` — navigation jumps
- `ScreenChangeGroupOption`, `ScreenColor`, `ScreenOpacity` — property toggles

---

## GraphicsWindow

### Overview

`src/ui.h:549` — `class GraphicsWindow`

The main 3D viewport. Manages the axonometric camera projection, entity selection/hover, mouse/keyboard input dispatch, toolbar drawing, and canvas rendering.

Implemented in `src/graphicswin.cpp` and toolbar logic in `src/toolbar.cpp`.

### Projection Model

```cpp
Vector  offset;    // world-space pan offset
Vector  projRight; // screen X axis in world space
Vector  projUp;    // screen Y axis in world space
double  scale;     // units per pixel
```
(`src/ui.h:593-598`)

Projection helpers (`src/ui.h:614-620`):
- `ProjectPoint(Vector p) → Point2d` — world → screen
- `UnProjectPoint(Point2d p) → Vector` — screen → world plane
- `ProjectPoint4(Vector p, double *w)` — homogeneous with depth

Cached projection snapshot at `orig` and `cached` structs detects when invalidation is needed.

### Selection Model

`src/ui.h:719-783` — inner classes `Selection` and `Hover`

```cpp
class Selection {
    int tag;              // tag for bulk operations
    hEntity entity;
    hConstraint constraint;
    bool emphasized;
    // Draw(), Clear(), IsEmpty(), Equals(), HasEndpoints()
};

class Hover {
    int zIndex;      // rendering layer
    double distance; // cursor distance
    double depth;    // depth from camera
    Selection selection;
};
```

State:
- `List<Selection> selection` — multi-selection list
- `Selection hover` — single current hover
- `List<Hover> hoverList` — candidates from hit test (sorted by depth/distance)
- `MAX_SELECTABLE_FACES = 3`

Selection pipeline:
1. `HitTestMakeSelection(Point2d mp)` — tests all entities/constraints, populates `hoverList`
2. `ChooseFromHoverToSelect()` / `ChooseFromHoverToDrag()` — picks best candidate
3. `MakeSelected(he)` / `MakeUnselected(he, coincidentPointTrick)` — modifies selection list

Grouped selection summary in `gs` struct (`src/ui.h:752-771`), populated by `GroupSelection()`:
counts of points, entities, workplanes, faces, lineSegments, circlesOrArcs, constraints, etc.

### Pending Operations

`src/ui.h:641-680` — `enum class Pending` + `struct pending`

Mouse-driven creation operations are queued here:

| Value | Meaning |
|-------|---------|
| `NONE` | Default idle state |
| `COMMAND` | Menu command awaiting first click |
| `DRAGGING_POINTS` | Moving existing points |
| `DRAGGING_NEW_POINT` | Placing a new datum point |
| `DRAGGING_NEW_LINE_POINT` | Drawing a line segment |
| `DRAGGING_NEW_CUBIC_POINT` | Drawing a cubic bezier |
| `DRAGGING_NEW_ARC_POINT` | Drawing an arc |
| `DRAGGING_CONSTRAINT` | Moving a constraint label |
| `DRAGGING_RADIUS` | Adjusting circle radius |
| `DRAGGING_NORMAL` | Rotating a normal entity |
| `DRAGGING_MARQUEE` | Rubber-band selection |

Cleared via `ClearPending(bool scheduleShowTW)`.

### Mouse Event Flow

Platform event → `MouseEvent(Platform::MouseEvent)` dispatcher → specific handlers:

```
MouseMoved(x, y, leftDown, middleDown, rightDown, shiftDown, ctrlDown)
MouseLeftDown(x, y, shiftDown, ctrlDown)
MouseLeftUp(x, y, shiftDown, ctrlDown)
MouseLeftDoubleClick(x, y)
MouseMiddleOrRightDown(x, y)   -- pan / rotate view
MouseRightUp(x, y)             -- context menu
MouseScroll(delta)             -- zoom
```
(`src/ui.h:859-870`)

Drag throttle: `havePainted` flag prevents re-solving until a repaint has occurred (avoid wasted solves during fast dragging).

### Display Toggles

`src/ui.h:833-855` — boolean display flags:

```cpp
bool showWorkplanes, showNormals, showPoints, showConstruction;
ShowConstraintMode showConstraints; // SCM_NOSHOW, SCM_SHOW_ALL, SCM_SHOW_DIM
bool showTextWindow, showShaded, showEdges, showOutlines;
bool showFaces, showFacesDrawing, showFacesNonDrawing, showMesh;
bool showSnapGrid, dimSolidModel;
DrawOccludedAs drawOccludedAs; // INVISIBLE, STIPPLED, VISIBLE
```

### Rendering Pipeline

```
Draw(Canvas*)           -- full scene render
DrawPersistent(Canvas*) -- cached persistent elements
DrawEntities(Canvas*, bool persistent) -- entity geometry
DrawSnapGrid(Canvas*)   -- background grid
Paint()                 -- trigger repaint cycle
Invalidate(bool clearPersistent) -- mark dirty
```

### Active Group & Workplane

```cpp
hGroup activeGroup;
void EnsureValidActives();
bool LockedInWorkplane();
hEntity ActiveWorkplane();
void SetWorkplaneFreeIn3d();
```
(`src/ui.h:682-690`)

The active group determines which entities are editable. `LockedInWorkplane()` returns true when constrained to a 2D workplane.

### Toolbar

`src/ui.h:822-830`, implemented in `src/toolbar.cpp`:

```cpp
bool ToolbarDrawOrHitTest(int x, int y, UiCanvas*, Command*, int* hitX, int* hitY);
void ToolbarDraw(UiCanvas*);
bool ToolbarMouseMoved(int x, int y);
bool ToolbarMouseDown(int x, int y);
Command toolbarHovered;
```

### 6-DoF Device Support

`src/ui.h:881-884` — `SixDofEvent(Platform::SixDofEvent)` for SpaceMouse/6-axis input.

---

## Global Instances

Both windows are accessed via the global `SS` object (see `project_info/code-patterns.md`):

```cpp
SS.GW   // GraphicsWindow
SS.TW   // TextWindow
```

`SS.TW.Printf(...)` is used throughout the codebase to output text-window content.  
`SS.GW.selection` / `SS.GW.hover` are used by constraint creation code.

---

## Key Source Locations

| Symbol | Location |
|--------|----------|
| `enum class Command` | `src/ui.h:82` |
| `class TextWindow` | `src/ui.h:198` |
| `TextWindow::Screen enum` | `src/ui.h:283` |
| `TextWindow::ShownState` | `src/ui.h:295` |
| `TextWindow::Edit enum` | `src/ui.h:315` |
| `TextWindow::Printf()` impl | `src/textwin.cpp:360` |
| `TextWindow::ClearScreen()` | `src/textwin.cpp:341` |
| `TextWindow::GoToScreen()` | `src/ui.h:419` |
| `class GraphicsWindow` | `src/ui.h:549` |
| `GraphicsWindow::Selection` | `src/ui.h:719` |
| `GraphicsWindow::Hover` | `src/ui.h:735` |
| `GraphicsWindow::Pending` | `src/ui.h:641` |
| `GroupSelection()` / `gs` struct | `src/ui.h:752` |
| `MouseEvent()` dispatcher | `src/ui.h:859` |
| `Screen Show*()` impls | `src/textscreens.cpp` |
| Toolbar implementation | `src/toolbar.cpp` |

---

## Related Documentation

- `project_info/subsystems/platform.md` — Platform::WindowRef, Platform::MouseEvent, Timer
- `project_info/subsystems/sketch.md` — hEntity/hConstraint handles used in Selection
- `project_info/code-patterns.md` — Printf DSL pattern, Screen navigation pattern, global SS/SK singletons
- `project_info/context-strategy.md` — Which files to load for UI work
