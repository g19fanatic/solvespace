# Finding 06: Group Serialization — How Group Types Are Stored

## Source Files Examined
- `src/file.cpp` (1030 lines) — the complete file I/O system
- `src/sketch.h` — Group struct definition
- `src/group.cpp` — Group::MenuGroup creation and Generate usage

## Overview

SolveSpace uses a single flat text-based file format (`.slvs`). The serialization
system is a static table of `SaveTable` entries (`SAVED[]`) that map field paths
to raw memory pointers via a `SAVEDptr` union. This table covers Groups, Params,
Requests, Entities, Constraints, and Styles. Geometry (mesh/shell) is saved
separately and regenerated on load.

---

## 1. The SAVED[] Table Mechanism

```c
// src/file.cpp:89-123
const SolveSpaceUI::SaveTable SolveSpaceUI::SAVED[] = {
    // { record_type, "Field.name", format_char, &SS.sv.g.field },
    { 'g',  "Group.h.v",                'x',    &(SS.sv.g.h.v)               },
    { 'g',  "Group.type",               'd',    &(SS.sv.g.type)              },
    ...
    { 0, NULL, 0, NULL }
};
```

**Format characters:**
| Char | Type | Notes |
|------|------|-------|
| `'x'` | uint32_t hex | handles, IDs |
| `'d'` | int (decimal) | enum values, small ints |
| `'f'` | double (%.20f) | distances, angles, scales |
| `'b'` | bool | flags |
| `'S'` | std::string | names, comments |
| `'c'` | RgbaColor | packed hex |
| `'P'` | Platform::Path | portable relative path |
| `'M'` | EntityMap | special multi-line format |
| `'i'` | ignored | legacy fields, skipped |

**Zero optimization:** Fields that are zero/empty are NOT written (see `SaveUsingTable`
at file.cpp:~230). This means a CHAMFER/FILLET group that only uses `opA`, `valA`,
and `meshCombine` (all non-zero) will serialize cleanly.

---

## 2. Complete Group SAVED[] Entries

All Group fields currently serialized (src/file.cpp:90-123):

```
Group.h.v           'x'   — group handle (ID)
Group.type          'd'   — Group::Type enum value (uint32_t cast)
Group.order         'd'   — display order
Group.name          'S'   — human-readable name
Group.activeWorkplane.v 'x' — active workplane handle
Group.opA.v         'x'   — source group handle (CRITICAL for chamfer/fillet)
Group.opB.v         'x'   — second source group handle (unused by most groups)
Group.valA          'f'   — primary scalar parameter (depth, radius, angle, steps)
Group.valB          'f'   — secondary scalar parameter
Group.valC          'f'   — tertiary scalar parameter (NOTE: BUG - maps to &valB!)
Group.color         'c'   — body color
Group.subtype       'd'   — Group::Subtype enum
Group.skipFirst     'b'   — for step-and-repeat
Group.meshCombine   'd'   — CombineAs enum (UNION=0, DIFFERENCE=1, ASSEMBLE=2, INTERSECTION=3)
Group.forceToMesh   'd'   — bool, forces mesh triangulation
Group.predef.q.w    'f'   — orientation quaternion W
Group.predef.q.vx   'f'   — orientation quaternion VX
Group.predef.q.vy   'f'   — orientation quaternion VY
Group.predef.q.vz   'f'   — orientation quaternion VZ
Group.predef.origin.v 'x' — origin entity handle
Group.predef.entityB.v 'x' — secondary entity handle
Group.predef.entityC.v 'x' — tertiary entity handle
Group.predef.swapUV 'b'   — UV swap flag
Group.predef.negateU 'b'  — U negate flag
Group.predef.negateV 'b'  — V negate flag
Group.visible       'b'   — display visibility
Group.suppress      'b'   — suppress from solid model
Group.relaxConstraints 'b' — allow under-constrained
Group.allowRedundant 'b'  — allow redundant constraints
Group.allDimsReference 'b' — all dims are reference
Group.scale         'f'   — scale (default 1)
Group.remap         'M'   — EntityMap (input->copyNumber to entity index)
Group.impFile       'i'   — LEGACY (ignored on save/load)
Group.impFileRel    'P'   — linked file path (for LINKED groups)
```

**KEY BUG FOUND:** Line 99 in file.cpp has a typo:
```c
{ 'g',  "Group.valC",  'f',  &(SS.sv.g.valB) },   // <-- should be valC, maps to valB!
```
This means `Group.valC` is never actually read/written correctly — it always
reads/writes `valB`. For CHAMFER/FILLET, we should use `valA` and `valB` only,
or fix this bug when adding the new feature.

---

## 3. How Existing Group Types Use These Fields

### EXTRUDE (Type=5100)
```
opA   = source group handle (the 2D sketch group)
valA  = NOT the depth — depth is a Param (h.param(0/1))
        valA is actually used via a solver parameter
subtype = ONE_SIDED(7000) or TWO_SIDED(7001) or ONE_SKEWED(7004) or TWO_SKEWED(7005)
meshCombine = UNION or DIFFERENCE (set by user)
```
(src/group.cpp:177-183, Generate at :499-538)

### LATHE (Type=5101)
```
opA      = source group handle
predef.entityB = axis entity
valA     = not used for depth (solver param)
```
(src/group.cpp:205)

### REVOLVE (Type=5102)
```
opA   = source group handle
valA  = 2 (initial step count for param generation)
subtype = ONE_SIDED
predef.origin = origin entity
predef.entityB = axis direction entity
```
(src/group.cpp:231-237)

### HELIX (Type=5103)
```
opA   = source group handle
valA  = 2 (steps)
valB  = pitch (via Edit::HELIX_PITCH)
subtype = ONE_SIDED
predef.origin = origin entity
predef.entityB = axis direction entity
```
(src/group.cpp:259-264)

### ROTATE (Type=5200)
```
opA   = source group handle
valA  = n (number of copies, integer)
subtype = ONE_SIDED or TWO_SIDED
predef.origin = pivot entity
predef.entityB = axis direction
```

### TRANSLATE (Type=5201)
```
opA   = source group handle
valA  = n (number of copies)
subtype = ONE_SIDED or TWO_SIDED
predef.entityB = direction entity
```

### LINKED (Type=5300)
```
linkFile = path to linked .slvs file
meshCombine = ASSEMBLE (set in MenuGroup)
```

---

## 4. How Param Values Are Used (Critical for Chamfer)

For EXTRUDE, the depth is NOT stored in `valA` — it's a **solver parameter**
(a `Param` in `SK.param`). The group adds params via:
```c
// group.cpp Generate() for EXTRUDE
AddParam(param, h.param(0), 0.0);  // extrude distance
```
These params are saved as:
```
Param.h.v     'x'  — param handle
Param.val     'f'  — current value
```

**For CHAMFER/FILLET**: The radius/distance could be stored as either:
1. A solver `Param` (like extrude depth) — allows constraint-driven parametrics
2. Directly in `valA` (like step-count for ROTATE) — simpler, no solver needed

Looking at the evidence:
- REVOLVE stores `valA = 2` as just an initial value; actual angle is a Param
- ROTATE stores `valA = n` as an integer count (non-parametric)
- **Recommendation:** For MVP, store chamfer distance in `valA` directly (like
  rotate step count). Add a Param for future constraint support.

---

## 5. The remap Field

The `Group.remap` field is an `EntityMap` — maps `(input_entity, copy_number)` to
output entity IDs. This is how stable entity IDs are maintained across regeneration.

For CHAMFER/FILLET:
- We need REMAP entries for: chamfer face, fillet face (at minimum)
- REMAP constants already in sketch.h (src/sketch.h:305-315):
  ```
  REMAP_LAST              = 1000
  REMAP_TOP               = 1001
  REMAP_BOTTOM            = 1002
  REMAP_PT_TO_LINE        = 1003
  REMAP_LINE_TO_FACE      = 1004
  REMAP_LATHE_START       = 1006
  REMAP_LATHE_END         = 1007
  REMAP_PT_TO_ARC         = 1008
  REMAP_PT_TO_NORMAL      = 1009
  REMAP_LATHE_ARC_CENTER  = 1010
  ```
- Next available: 1011, 1012
- **Add:** `REMAP_CHAMFER_FACE = 1011`, `REMAP_FILLET_FACE = 1012`

The remap is serialized via the `'M'` format which writes multi-line blocks:
```
Group.remap={
    0 0001fffe 0
    1 0001fffe 1
    ...
}
```

---

## 6. File Format Protocol

Save path (file.cpp:220-355):
1. Writes VERSION_STRING header
2. For each group: `SaveUsingTable('g')`, then `AddGroup\n`
3. For each param: `SaveUsingTable('p')`, then `AddParam\n`
4. For each request: `SaveUsingTable('r')`, then `AddRequest\n`
5. For each entity: calculates numerical, `SaveUsingTable('e')`, then `AddEntity\n`
6. For each constraint: `SaveUsingTable('c')`, then `AddConstraint\n`
7. For each style: `SaveUsingTable('s')`, then `AddStyle\n`
8. Geometry: Triangle/Surface/SCtrl/TrimBy/Curve/CCtrl/CurvePt/AddSurface/AddCurve records

Load path (file.cpp:~430):
1. Reads line by line
2. `key=val` lines → `LoadUsingTable(key, val)` → memcpy into `SS.sv.g` etc.
3. `AddGroup` → `SK.group.Add(&sv.g)`, reset `sv.g`
4. `AddParam` → `SK.param.Add(&sv.p)`, reset `sv.p`
5. Geometry records are **ignored** (regenerated by GenerateAll)
6. Unknown keys → `fileLoadError = true` (warning, not fatal)

**Critical: Unknown fields cause `fileLoadError` but NOT file rejection** — the
file still loads. This means adding new fields `Group.valA`, `Group.opA` etc.
for a new group type is backward compatible (old SolveSpace will load file with
warning, new group shows as unknown type).

---

## 7. Param Serialization

Params (solver variables) ARE saved independently:
```
Param.h.v  = handle
Param.val  = initial/current value
```
On load, these are added to `SK.param` as initial guesses for the next solve.
Entities are NOT saved (they're regenerated), but params are saved so the solver
starts from the last solved state.

For CHAMFER/FILLET with solver params:
```c
// In Group::Generate() for CHAMFER:
AddParam(param, h.param(0), valA);  // chamfer distance as solver param
```
This would create a `Param.h.v=xxxxxxxx, Param.val=5.0` entry in the file.

---

## 8. What CHAMFER/FILLET Need to Serialize

Based on the analysis, the minimum serialization for CHAMFER/FILLET groups:

```
Group.h.v           — standard (auto-assigned)
Group.type          — 5400 (CHAMFER) or 5401 (FILLET)
Group.order         — standard
Group.name          — "chamfer" or "fillet"
Group.opA.v         — the source solid group (e.g., EXTRUDE group handle)
Group.valA          — distance/radius value (in mm/inches)
Group.valB          — second leg distance (for asymmetric chamfer), or 0
Group.color         — inherited or user-set
Group.meshCombine   — DIFFERENCE (1) — we subtract material
Group.visible       — true
Group.remap         — for stable face entity IDs across regeneration
```

Fields NOT needed for chamfer/fillet:
- `subtype` — not needed (single mode)
- `predef.*` — not needed (no workplane, no axis)
- `skipFirst` — not needed (not a repeat operation)
- `forceToMesh` — optional, false by default
- `linkFile` — not needed (not a LINKED group)

**Edge selection storage:** The target edge(s) are the hardest part to serialize.
Options:
1. Store edge reference as an entity handle in `predef.entityB.v` (repurpose the
   predef struct — this is how REVOLVE stores its axis)
2. Store multiple edges in the remap (requires architectural change)
3. Store edge as a new field type (not in current SAVED[] schema)

**Recommendation for MVP:** Use `predef.entityB.v` to store the single selected
edge entity handle. This is already serialized, requires zero file format changes.

---

## 9. File Format Extension Strategy

If additional fields beyond `predef.entityB` are needed:

**Option A: Reuse existing fields** (zero schema changes)
- `predef.entityB.v` — edge entity handle
- `predef.entityC.v` — secondary edge or face reference
- `valB` — second chamfer distance (for asymmetric chamfer)

**Option B: Add new SAVED[] entries** (requires new file format key)
- Add `{ 'g', "Group.chamferEdge.v", 'x', &(SS.sv.g.predef.entityB.v) }`
  with a NEW key name; old loaders will set `fileLoadError` but continue
- This is cleanly extensible

The current SAVED[] keys are doc strings; adding a new key like
`"Group.chamferEdge.v"` that points to an existing field is perfectly valid and
provides semantic clarity.

---

## 10. Summary of Key Findings for Implementation

1. **No new file format types needed** — CHAMFER/FILLET use existing 'g' record
2. **valA = distance/radius** — main parameter, serialized as `'f'` (double)
3. **valB = second leg** — for asymmetric chamfer, serialized as `'f'`
4. **opA = source group** — the solid being modified, serialized as `'x'` (hex handle)
5. **meshCombine = DIFFERENCE** — hardcoded in MenuGroup setup
6. **remap = EntityMap** — for stable face IDs, serialized as `'M'` block
7. **BUG in Group.valC:** Line 99 of file.cpp maps valC → valB (typo). Fix this
   when implementing if valC is needed.
8. **predef.entityB** — available to store the selected edge handle (already
   serialized, just repurposed for chamfer context)
9. **Backward compatibility** — adding new Group.type values is safe: old SolveSpace
   will set fileLoadError but won't crash (SK.group.Add is unconditional)
10. **Params** — if chamfer radius needs to be constrainable, add a `Param` via
    `AddParam(param, h.param(0), valA)` in Generate(); the param is saved automatically

### Minimal SAVED[] addition required for CHAMFER/FILLET:
**NONE** — all needed fields already exist in the SAVED[] table. The only change
is adding a new `Group::Type::CHAMFER = 5400` and `FILLET = 5401` to sketch.h,
and possibly adding semantic aliases in SAVED[] for clarity (optional).
