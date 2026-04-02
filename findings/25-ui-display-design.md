# 25 — TextWindow UI Display Design for Chamfer/Fillet Groups

## Overview

This document covers how the SolveSpace TextWindow (`textscreens.cpp`) should be extended
to display properties for new CHAMFER and FILLET group types, based on full source analysis
of `src/textscreens.cpp` and `src/ui.h`.

---

## 1. How ShowGroupInfo() Currently Works

### Location
`src/textscreens.cpp` — `TextWindow::ShowGroupInfo()` starting at approximately line 380.

### Structure of the if-chain (lines ~394–490)
The function dispatches on `g->type`:

```
if(g->type == Group::Type::LATHE) {
    Printf(true, " %Ftlathe plane sketch");
} else if(g->type == EXTRUDE || ROTATE || TRANSLATE || REVOLVE || HELIX) {
    // show subtype (one/two-sided), repeat count, etc.
} else if(g->type == LINKED) {
    // show link file path, scale
} else if(g->type == DRAWING_3D) {
    Printf(true, " %Ftsketch in 3d%E");
} else if(g->type == DRAWING_WORKPLANE) {
    Printf(true, " %Ftsketch in new workplane%E");
} else {
    Printf(true, "???");   // <-- fallback for unknown type
}
```

**The "???" fallback** is triggered for any group type not explicitly handled. Adding
CHAMFER/FILLET without modifying this function will display "???" in the text panel —
functional but confusing.

### Mesh Combine radio buttons (lines ~498–520)
Shown only for: `EXTRUDE || LATHE || REVOLVE || LINKED || HELIX`.
This block shows UNION / ASSEMBLE / DIFFERENCE / INTERSECTION radio buttons.

For CHAMFER/FILLET, meshCombine is always DIFFERENCE (material is removed) so these
radio buttons should be **skipped or shown as read-only**. The MVP can simply omit them.

### Color/Opacity (lines ~525–535)
Currently shown only for EXTRUDE, LATHE, REVOLVE, HELIX. CHAMFER/FILLET should also
show color/opacity since they add new surfaces to the solid.

### Suppress checkbox (lines ~537–542)
Already generic — shown for EXTRUDE/LATHE/REVOLVE/LINKED/HELIX. Add CHAMFER/FILLET to
this condition so their geometry can be suppressed for preview.

---

## 2. Required New Edit Control Meanings

From `src/ui.h`, the `Edit` enum (inside `TextWindow` struct) currently ends at:
```cpp
// For helix pitch
HELIX_PITCH = 802
```

New entries needed:
```cpp
// For chamfer offset
CHAMFER_OFFSET = 803,
// For fillet radius
FILLET_RADIUS  = 804
```

These drive `ShowEditControl()` calls and the `EditControlDone()` switch statement.

---

## 3. New Static Screen Callbacks Needed

Pattern from existing group types:
- `ScreenChangeHelixPitch` → shows edit control, sets `edit.meaning = Edit::HELIX_PITCH`
- `EditControlDone` case `Edit::HELIX_PITCH` → reads value, sets `g->valB`, marks dirty

For CHAMFER:
```cpp
static void ScreenChangeChamferOffset(int link, uint32_t v);
// In EditControlDone:
case Edit::CHAMFER_OFFSET:
    if(Expr *e = Expr::From(s, true)) {
        double ev = SS.ExprToMm(e);
        if(ev <= 0) { Error("Chamfer offset must be positive."); break; }
        Group *g = SK.GetGroup(edit.group);
        g->valA = ev;
        SS.MarkGroupDirty(g->h);
    }
    break;
```

For FILLET:
```cpp
static void ScreenChangFilletRadius(int link, uint32_t v);
// In EditControlDone:
case Edit::FILLET_RADIUS:
    if(Expr *e = Expr::From(s, true)) {
        double ev = SS.ExprToMm(e);
        if(ev <= 0) { Error("Fillet radius must be positive."); break; }
        Group *g = SK.GetGroup(edit.group);
        g->valA = ev;
        SS.MarkGroupDirty(g->h);
    }
    break;
```

---

## 4. Proposed ShowGroupInfo() Block for CHAMFER

Insert in the if-chain between HELIX and LINKED:

```cpp
} else if(g->type == Group::Type::CHAMFER) {
    Printf(true, " %Ftchamfer edge of solid%E");
    Printf(false, "");

    // Show the selected face pair
    Printf(false, "%Ft selected face pair%E");
    // face entities are stored in predef.entityB and predef.entityC
    // (both are face entities in the source group)
    hEntity faceA = g->predef.entityB;
    hEntity faceB = g->predef.entityC;
    if(faceA.v && faceB.v) {
        Printf(false, "%Ba   face A: e%08x", faceA.v);
        Printf(false, "%Bd   face B: e%08x", faceB.v);
    } else {
        Printf(false, "%Ba   (no faces selected)");
    }
    Printf(false, "");

    // Show offset distance (valA)
    Printf(false, "%Ft offset distance%E");
    Printf(false, "%Ba   %s %Fl%Ll%f%D[change]%E",
        SS.MmToString(g->valA).c_str(),
        &TextWindow::ScreenChangeChamferOffset, g->h.v);
```

Then below the if-chain, include CHAMFER in the "color" and "suppress" blocks:

```cpp
// Extend mesh-combine suppress to include CHAMFER/FILLET
if(g->type == Group::Type::EXTRUDE || ... || g->type == Group::Type::CHAMFER ||
   g->type == Group::Type::FILLET) {
    Printf(false, "   %Fd%f%LP%s  suppress this group's solid model",
        &TextWindow::ScreenChangeGroupOption,
        g->suppress ? CHECK_TRUE : CHECK_FALSE);
}

// Extend color/opacity to include CHAMFER/FILLET
if(g->type == Group::Type::EXTRUDE || ... || g->type == Group::Type::CHAMFER ||
   g->type == Group::Type::FILLET) {
    Printf(false,
        "%Bd   %Ftcolor   %E%Bz  %Bd (%@, %@, %@) %f%D%Lf%Fl[change]%E",
        &g->color, ...);
    Printf(false, "%Bd   %Ftopacity%E %@ %f%Lf%Fl[change]%E", ...);
}
```

---

## 5. Proposed ShowGroupInfo() Block for FILLET

```cpp
} else if(g->type == Group::Type::FILLET) {
    Printf(true, " %Ftfillet edge of solid%E");
    Printf(false, "");

    Printf(false, "%Ft selected face pair%E");
    hEntity faceA = g->predef.entityB;
    hEntity faceB = g->predef.entityC;
    if(faceA.v && faceB.v) {
        Printf(false, "%Ba   face A: e%08x", faceA.v);
        Printf(false, "%Bd   face B: e%08x", faceB.v);
    } else {
        Printf(false, "%Ba   (no faces selected)");
    }
    Printf(false, "");

    Printf(false, "%Ft fillet radius%E");
    Printf(false, "%Ba   %s %Fl%Ll%f%D[change]%E",
        SS.MmToString(g->valA).c_str(),
        &TextWindow::ScreenChangeFilletRadius, g->h.v);
```

---

## 6. The ShowListOfGroups() Warning Block

`ShowListOfGroups()` has a `warn` flag that shows "err" next to groups with polygon
errors. Currently it checks:

```cpp
bool warn = (g->type == Group::Type::DRAWING_WORKPLANE &&
             g->polyError.how != PolyError::GOOD) ||
            ((g->type == Group::Type::EXTRUDE ||
              g->type == Group::Type::LATHE) &&
             SK.GetGroup(g->opA)->polyError.how != PolyError::GOOD);
```

CHAMFER/FILLET do not operate on a sketch profile — they operate on a solid. The
`polyError` concept doesn't apply. **No change needed to this block** for CHAMFER/FILLET.

However, if the chamfer/fillet geometry generation fails (e.g., degenerate edge),
`g->booleanFailed` should be set to true. The existing "booleanFailed" error display
block at the bottom of `ShowGroupInfo()` will automatically show the error message:
```
The Boolean operation failed. It may be possible to fix the problem by choosing
'force NURBS surfaces to triangle mesh'.
```
This is reused verbatim for chamfer/fillet failures.

---

## 7. MeshCombine Radio Buttons — Skip for CHAMFER/FILLET

The current condition for showing meshCombine radio buttons:
```cpp
if(g->type == Group::Type::EXTRUDE || g->type == Group::Type::LATHE ||
   g->type == Group::Type::REVOLVE || g->type == Group::Type::LINKED ||
   g->type == Group::Type::HELIX) {
```

For MVP: **do NOT add CHAMFER/FILLET here**. The meshCombine is always DIFFERENCE for
chamfer/fillet — it makes no sense for the user to change it. The group sets
`meshCombine = CombineAs::DIFFERENCE` unconditionally in `MenuGroup()`.

Post-MVP: Could add a UNION option for "additive fillet" (fillet bead), but that's exotic.

---

## 8. DescriptionString() Method

Each Group type has a `DescriptionString()` method in `group.cpp`. Pattern:

```cpp
std::string Group::DescriptionString() const {
    const char *s;
    if(name.empty()) {
        switch(type) {
            case Type::EXTRUDE: s = "g???-extrude"; break;
            // ...
        }
    }
    return ssprintf("g%03d-%s", h.v & 0xfff, s);
}
```

Need to add:
```cpp
case Type::CHAMFER: s = name.empty() ? "g???-chamfer" : name.c_str(); break;
case Type::FILLET:  s = name.empty() ? "g???-fillet"  : name.c_str(); break;
```

This affects the group list display and the ShowGroupInfo() header line.

---

## 9. Complete List of Files to Modify for UI

| File | Change |
|------|--------|
| `src/ui.h` | Add `CHAMFER_OFFSET=803`, `FILLET_RADIUS=804` to `Edit` enum; add `GROUP_CHAMFER`, `GROUP_FILLET` to `Command` enum |
| `src/textscreens.cpp` | Add CHAMFER/FILLET branches to `ShowGroupInfo()` if-chain (~line 394); add `ScreenChangeChamferOffset`, `ScreenChangeFilletRadius` static methods; add `case Edit::CHAMFER_OFFSET`, `Edit::FILLET_RADIUS` to `EditControlDone()`; extend suppress/color/opacity conditions |
| `src/group.cpp` | Add `DescriptionString()` cases for CHAMFER/FILLET; add `MenuGroup()` cases |
| `src/graphicswin.cpp` | Add menu entries for `GROUP_CHAMFER`, `GROUP_FILLET` in the Group submenu |

---

## 10. The "Source Group" Display Pattern

CHAMFER/FILLET have `opA` as the source group (like EXTRUDE uses `opA` for the sketch
group). The text panel should not show "extrude plane sketch" but something like:
- "chamfer edge of solid (from group g001-extrude)"

Currently the EXTRUDE block doesn't show the source group name. For MVP, the source group
is visible from context (it's the previous group). Showing "chamfer edge of solid" as
the description is sufficient.

---

## 11. Helix as Model for Numerical Parameter Display

The HELIX group's pitch display is the closest analog to chamfer's offset display:

```cpp
// From textscreens.cpp, HELIX pitch display:
if(g->type == Group::Type::HELIX) {
    Printf(false, "%Ft pitch - length per turn%E");
    if(fabs(g->valB) != 0.0) {
        Printf(false, "  %Ba %# %Fl%Ll%f%D[change]%E",
            g->valB / SS.MmPerUnit(),
            &TextWindow::ScreenChangeHelixPitch, g->h.v);
    } else {
        Printf(false, "  %Ba %# %E",
            SK.GetParam(g->h.param(7))->val * PI / ...);
    }
    Printf(false, "   %Fd%f%LP%s  fixed",
        &TextWindow::ScreenChangePitchOption,
        g->valB != 0 ? CHECK_TRUE : CHECK_FALSE);
}
```

For CHAMFER's offset, the analog is:
```cpp
if(g->type == Group::Type::CHAMFER) {
    Printf(false, "%Ft chamfer offset distance%E");
    Printf(false, "  %Ba %s %Fl%Ll%f%D[change]%E",
        SS.MmToString(g->valA).c_str(),
        &TextWindow::ScreenChangeChamferOffset, g->h.v);
}
```

Key difference: HELIX uses `%#` format (raw double), CHAMFER should use `SS.MmToString()`
to respect the current unit system (mm vs inches).

---

## 12. Summary of All Required textscreens.cpp Changes

### A. In `ShowGroupInfo()` if-chain, add two new branches:
- After LINKED block, before the final `else { Printf(true, "???"); }`:
  ```
  } else if(g->type == Group::Type::CHAMFER) { ... }
  else if(g->type == Group::Type::FILLET) { ... }
  ```

### B. Extend existing condition blocks to include CHAMFER/FILLET:
- Suppress checkbox condition: add `|| g->type == Group::Type::CHAMFER || FILLET`
- Color/Opacity condition: add `|| g->type == Group::Type::CHAMFER || FILLET`
- **Do NOT add** to meshCombine radio buttons (always DIFFERENCE, no user choice)

### C. New static callback methods (add to class declaration in `ui.h` and implement in `textscreens.cpp`):
```cpp
static void ScreenChangeChamferOffset(int link, uint32_t v);
static void ScreenChangeFilletRadius(int link, uint32_t v);
```

### D. New `EditControlDone()` switch cases:
```cpp
case Edit::CHAMFER_OFFSET: { ... g->valA = SS.ExprToMm(e); ... } break;
case Edit::FILLET_RADIUS:  { ... g->valA = SS.ExprToMm(e); ... } break;
```

---

## 13. Unit Handling

`SS.MmToString(val)` already handles mm/inches display. `SS.ExprToMm(e)` converts
user-entered expressions to internal millimeter units. Both are already used throughout
`textscreens.cpp` and `EditControlDone()` for distances. No new unit-handling code needed.

---

## 14. Error Display

If chamfer/fillet geometry fails (edge not found, degenerate geometry, etc.):
- Set `g->booleanFailed = true` in `GenerateShellAndMesh()`
- The existing `booleanFailed` error message block in `ShowGroupInfo()` will
  automatically show the error to the user — no new error display code needed.

Additional validation errors (e.g., "no shared edge between selected faces") should
be surfaced via `SS.ReportError()` at MenuGroup() time, before the group is created.

---

## Key Line References

| Location | Purpose |
|----------|---------|
| `textscreens.cpp:380` | `ShowGroupInfo()` function start |
| `textscreens.cpp:394` | if-chain dispatch on `g->type` |
| `textscreens.cpp:498` | meshCombine radio button block condition |
| `textscreens.cpp:525` | Color/opacity display block |
| `textscreens.cpp:537` | Suppress checkbox display block |
| `textscreens.cpp:~610` | `booleanFailed` error display |
| `textscreens.cpp:~700` | `EditControlDone()` function start |
| `textscreens.cpp:~802` | `HELIX_PITCH` case in `EditControlDone()` (last before default) |
| `ui.h:~420` | `Edit` enum in `TextWindow` struct |
| `ui.h:~170` | `Command` enum — Group section |
| `ui.h:~445` | Static method declarations in `TextWindow` |
