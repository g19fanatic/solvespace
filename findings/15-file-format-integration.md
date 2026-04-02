# Task 15: SolveSpace File Format — How Group Parameters Are Stored

## Overview

`src/file.cpp` implements SolveSpace's `.slvs` file format. It uses a **table-driven** approach: a static array `SAVED[]` of `SaveTable` structs maps field names to C++ struct fields via raw pointers. Save and load iterate over this single table.

---

## The SAVED[] Table Structure

```cpp
const SolveSpaceUI::SaveTable SolveSpaceUI::SAVED[] = {
    { type_char, "field.path.name", format_char, &(SS.sv.field.path) },
    ...
    { 0, NULL, 0, NULL }  // sentinel
};
```

**Type chars** (record type prefix):
- `'g'` = Group
- `'p'` = Param
- `'r'` = Request
- `'e'` = Entity
- `'c'` = Constraint
- `'s'` = Style

**Format chars** (data type):
| Char | Type | Example |
|------|------|---------|
| `'x'` | uint32_t (hex) | handle `.v` fields |
| `'d'` | int (decimal) | enum values |
| `'f'` | double (20 decimal places) | floating point params |
| `'b'` | bool (0/1) | flags |
| `'S'` | std::string | names |
| `'c'` | RgbaColor (hex ARGB) | colors |
| `'P'` | Platform::Path (relative) | file paths |
| `'M'` | EntityMap (multi-line) | remap tables |
| `'i'` | ignored (legacy) | deprecated fields |

---

## Complete Group SAVED[] Entries

From `src/file.cpp:88–122`:

```cpp
{ 'g',  "Group.h.v",                'x',    &(SS.sv.g.h.v)                },
{ 'g',  "Group.type",               'd',    &(SS.sv.g.type)               },
{ 'g',  "Group.order",              'd',    &(SS.sv.g.order)              },
{ 'g',  "Group.name",               'S',    &(SS.sv.g.name)               },
{ 'g',  "Group.activeWorkplane.v",  'x',    &(SS.sv.g.activeWorkplane.v)  },
{ 'g',  "Group.opA.v",              'x',    &(SS.sv.g.opA.v)              },
{ 'g',  "Group.opB.v",              'x',    &(SS.sv.g.opB.v)              },
{ 'g',  "Group.valA",               'f',    &(SS.sv.g.valA)               },
{ 'g',  "Group.valB",               'f',    &(SS.sv.g.valB)               },
{ 'g',  "Group.valC",               'f',    &(SS.sv.g.valB)               },  // BUG: points to valB!
{ 'g',  "Group.color",              'c',    &(SS.sv.g.color)              },
{ 'g',  "Group.subtype",            'd',    &(SS.sv.g.subtype)            },
{ 'g',  "Group.skipFirst",          'b',    &(SS.sv.g.skipFirst)          },
{ 'g',  "Group.meshCombine",        'd',    &(SS.sv.g.meshCombine)        },
{ 'g',  "Group.forceToMesh",        'd',    &(SS.sv.g.forceToMesh)        },
{ 'g',  "Group.predef.q.w",         'f',    &(SS.sv.g.predef.q.w)         },
{ 'g',  "Group.predef.q.vx",        'f',    &(SS.sv.g.predef.q.vx)        },
{ 'g',  "Group.predef.q.vy",        'f',    &(SS.sv.g.predef.q.vy)        },
{ 'g',  "Group.predef.q.vz",        'f',    &(SS.sv.g.predef.q.vz)        },
{ 'g',  "Group.predef.origin.v",    'x',    &(SS.sv.g.predef.origin.v)    },
{ 'g',  "Group.predef.entityB.v",   'x',    &(SS.sv.g.predef.entityB.v)   },
{ 'g',  "Group.predef.entityC.v",   'x',    &(SS.sv.g.predef.entityC.v)   },
{ 'g',  "Group.predef.swapUV",      'b',    &(SS.sv.g.predef.swapUV)      },
{ 'g',  "Group.predef.negateU",     'b',    &(SS.sv.g.predef.negateU)     },
{ 'g',  "Group.predef.negateV",     'b',    &(SS.sv.g.predef.negateV)     },
{ 'g',  "Group.visible",            'b',    &(SS.sv.g.visible)            },
{ 'g',  "Group.suppress",           'b',    &(SS.sv.g.suppress)           },
{ 'g',  "Group.relaxConstraints",   'b',    &(SS.sv.g.relaxConstraints)   },
{ 'g',  "Group.allowRedundant",     'b',    &(SS.sv.g.allowRedundant)     },
{ 'g',  "Group.allDimsReference",   'b',    &(SS.sv.g.allDimsReference)   },
{ 'g',  "Group.scale",              'f',    &(SS.sv.g.scale)              },
{ 'g',  "Group.remap",              'M',    &(SS.sv.g.remap)              },
{ 'g',  "Group.impFile",            'i',    NULL                          },   // legacy, ignored
{ 'g',  "Group.impFileRel",         'P',    &(SS.sv.g.linkFile)           },
```

### ⚠️ Bug in Existing Code
`Group.valC` maps to `&(SS.sv.g.valB)` — **copy-paste bug**. valC saves/loads into valB's storage. This means `Group.valC` is effectively unusable for storing independent data in the file. A fix would be needed before valC can be used.

---

## How Save Works

```cpp
// src/file.cpp:225-230
for(auto &g : SK.group) {
    sv.g = g;                     // Copy group to scratch buffer
    SaveUsingTable(filename, 'g'); // Emit "Key=Value\n" lines for all 'g' fields
    fprintf(fh, "AddGroup\n\n");  // Record terminator
}
```

**SaveUsingTable** skips zero/empty/null fields automatically:
```cpp
if(fmt == 'f' && EXACT(p->f() == 0.0)) continue;  // omit zero floats
if(fmt == 'x' && p->x() == 0)         continue;  // omit zero handles
if(fmt == 'd' && p->d() == 0)         continue;  // omit zero ints
```

This means **zero values need no file entry** — very efficient. Only non-default values are written.

### Example Group Block in .slvs File

```
Group.h.v=00000002
Group.type=5000
Group.order=1
Group.name=extrude
Group.opA.v=00000001
Group.valA=10.00000000000000000000
Group.meshCombine=1
AddGroup
```

---

## How Load Works

```cpp
// src/file.cpp:413-415
char *e = strchr(line, '=');
if(e) {
    *e = '\0';
    char *key = line, *val = e+1;
    LoadUsingTable(filename, key, val);  // Look up key in SAVED[], cast and set
}
```

**AddGroup** handler:
```cpp
} else if(strcmp(line, "AddGroup")==0) {
    if(sv.g.type == Group::Type::LINKED)
        sv.g.opA.v = 0;  // Legacy cleanup
    SK.group.Add(&(sv.g));
    sv.g = {};              // Reset scratch buffer
    sv.g.scale = 1;         // Reset to non-zero default
}
```

**Unknown keys trigger a load error warning** (but loading continues):
```cpp
if(SAVED[i].type == 0) {  // Reached sentinel without finding key
    fileLoadError = true;
}
```
The program shows "Unrecognized data in file" dialog if any unknown key is encountered.

---

## Param Serialization (for solver params)

```cpp
{ 'p',  "Param.h.v.",   'x',    &(SS.sv.p.h.v)  },
{ 'p',  "Param.val",    'f',    &(SS.sv.p.val)   },
```

Params are **loaded for initial solver guesses**, then regenerated. The comment says:
```
// params are regenerated, but we want to preload the values
// for initial guesses
```

This is crucial: when loading, old param values seed the solver, allowing it to converge to the previous solution. **Chamfer/fillet params would need to be stored here too**.

---

## The B-Rep Shell Is Also Persisted (But Ignored on Load)

The B-rep (SShell) of the **last group** is written as:
- `Surface <h.v> <color> <face> <degm> <degn>` + `SCtrl` lines + `TrimBy` lines
- `Curve <h.v> <isExact> <deg> <surfA.v> <surfB.v>` + `CCtrl` lines + `CurvePt` lines
- `Triangle` entries for the mesh

On load, **all of these are ignored** (they're regenerated). The save serves as a visual preview/cache but the topology is rebuilt from scratch on regeneration.

---

## Key Insight: What Chamfer/Fillet Groups Need to Persist

For `Group::Type::CHAMFER` and `Group::Type::FILLET`, the following data needs serialization:

### 1. Already Covered by Existing SAVED[] Entries
All of these fields are already in SAVED[] — no new entries needed:

| Field | SAVED key | Notes |
|-------|-----------|-------|
| `g.type` | `Group.type` | CHAMFER=5400, FILLET=5401 |
| `g.opA.v` | `Group.opA.v` | Source group handle |
| `g.valA` | `Group.valA` | Chamfer distance / Fillet radius |
| `g.valB` | `Group.valB` | Second distance (unequal chamfer) or 0 |
| `g.remap` | `Group.remap` | 'M' format — EntityMap, critical! |
| `g.visible` | `Group.visible` | Display toggle |
| `g.suppress` | `Group.suppress` | Suppress geometry |
| `g.meshCombine` | `Group.meshCombine` | Should be ASSEMBLE for chamfer/fillet |
| `g.color` | `Group.color` | Face color |
| `g.name` | `Group.name` | User-visible name |

### 2. Edge Selection — The Hard Part

The key missing piece is: **how do we store which edge(s) to chamfer/fillet?**

**Option A: Use `Group.predef.entityB.v`**
- Already in SAVED[] as `hEntity`
- Could hold the handle of a selected edge entity/face
- Reuses existing field, no new SAVED[] entries

**Option B: Use `Group.remap` (EntityMap, 'M' format)**
- Already in SAVED[] and handles an arbitrary map of `{EntityKey → EntityId}` entries
- Most robust for multi-edge selection
- The 'M' format is multi-line in file:
  ```
  Group.remap={
      1 00000123 0
      2 00000456 0
  }
  ```
- `EntityKey` = `{input: hEntity, copyNumber: int}`; copyNumber could encode "which edge on this face"
- **This is the mechanism SolveSpace already uses for remapping copies**

**Option C: New dedicated field**
- Add a new field to Group struct (e.g., `selectedEdges: std::vector<hEntity>`)  
- Would require new SAVED[] format type (e.g., `'L'` for list) — complex
- Not recommended for MVP

**Recommended MVP approach**: Use `g.predef.entityB` for a single edge handle (MVP: one edge at a time). The existing `Group.predef.entityB.v` entry covers this.

---

## Format Upgrade Considerations

### fileLoadError on Unknown Keys
If a new SolveSpace version adds SAVED[] entries, loading an old file that doesn't have those keys is fine — the fields remain at their zero-initialized defaults. This is the **forward-compatible** direction.

If an OLD SolveSpace loads a NEW file with unknown keys (e.g., `Group.chamferEdge.v`), it would set `fileLoadError = true` and warn, but **not crash**. This is acceptable for new features.

### UpgradeLegacyData()
`src/file.cpp:514` contains `UpgradeLegacyData()`. This function handles format migrations for old files (e.g., TTF_TEXT extra points, constraint params). For chamfer/fillet:
- A migration hook here could handle old files that have `Group.type=5400` but missing new fields
- In practice, since chamfer/fillet is new (no old files have it), no migration is needed initially

---

## The `remap` EntityMap — Critical for Chamfer/Fillet

The 'M' format writes/reads an `EntityMap` (which is `std::unordered_map<EntityKey, EntityId>`):

```cpp
case 'M': {
    fprintf(fh, "{\n");
    // Sort and emit:  "    <EntityId.v> <EntityKey.input.v> <copyNumber>\n"
    // ...
    fprintf(fh, "}");
    break;
}
```

**Usage**: When a group produces copies of entities (e.g., EXTRUDE copies the sketch profile), the remap maps `{original_entity_handle, copy_number}` → `new_entity_id`. This ensures stable entity references across regeneration.

For chamfer/fillet, `remap` could be used to store the mapping from:
- `{source_scurve_handle, 0}` → `chamfer_face_entity_id`
- `{source_scurve_handle, 1}` → `chamfer_trim_curve_entity_id`

This would give **stable entity handles** for the generated chamfer geometry, which is exactly what the Toponaming problem requires.

---

## Param Storage for Chamfer/Fillet

When `Group::Generate()` calls `AddParam(h.param(0), valA)`, it creates a `Param` with:
- `h.v` = derived from group handle (e.g., `g.h.param(0).v`)  
- `val` = initial value (from `g.valA` = the chamfer distance)

This param is stored as:
```
Param.h.v=<handle>
Param.val=<distance_value>
AddParam
```

The solver can then optimize this param value if it's constrained. On file load, the stored `Param.val` seeds the solver.

**For chamfer**: `h.param(0)` = chamfer offset distance
**For fillet**: `h.param(0)` = fillet radius

Both follow exactly the same pattern as EXTRUDE uses for `h.param(0)` = extrude depth.

---

## Summary: What Changes Are Required in file.cpp

**For MVP chamfer/fillet, NO new SAVED[] entries are required.**

All necessary data maps to existing entries:
1. `Group.type` → stores CHAMFER/FILLET type integer
2. `Group.opA.v` → source group handle
3. `Group.valA` → distance/radius (the primary parameter)
4. `Group.valB` → second distance (for unequal chamfer) or 0
5. `Group.predef.entityB.v` → selected edge handle (single edge MVP)
6. `Group.remap` → EntityMap for stable entity IDs of generated surfaces/curves
7. `Group.meshCombine` → combine mode (ASSEMBLE recommended)
8. `Group.visible`, `Group.suppress`, `Group.color` → display options
9. Param via `Param.h.v` + `Param.val` → chamfer distance solver param

**If multi-edge selection is needed later:**
The `Group.remap` EntityMap could store multiple edge references in its copyNumber field, but this is non-obvious. A better long-term approach would be a dedicated entity list serialization.

---

## Notable Bug to Fix

```cpp
{ 'g',  "Group.valC",  'f',  &(SS.sv.g.valB)  },  // BUG: should be &(SS.sv.g.valC)
```

`Group.valC` points to `valB`'s storage. This is a pre-existing bug. For chamfer's equal-distance MVP, we only need `valA`, so it's not blocking. But if unequal-leg chamfer is desired (valA=d1, valB=d2), both are available and work correctly (valB is correct). Only `valC` is broken.

---

## File Format Example: What a Chamfer Group Would Look Like

```
Group.h.v=00000005
Group.type=5400
Group.order=2
Group.name=chamfer
Group.opA.v=00000004
Group.valA=2.00000000000000000000
Group.meshCombine=4
Group.visible=1
Group.remap={
    1 000002a4 0
}
AddGroup

Param.h.v=00050001
Param.val=2.00000000000000000000
AddParam
```

Where:
- `Group.type=5400` = CHAMFER
- `Group.opA.v=00000004` = handle of the source EXTRUDE group
- `Group.valA=2.0` = 2mm chamfer distance
- `Group.meshCombine=4` = ASSEMBLE (chamfer adds faces alongside existing solid)
- `Group.remap` = maps generated entity handle back to source SCurve
- `Param.h.v=00050001` = param handle derived from group h.param(0).v

---

## Conclusion

The SolveSpace file format is **table-driven**, highly extensible, and **zero-is-default** (only non-zero fields are written). For chamfer/fillet:

1. **No new SAVED[] entries are needed for MVP** — all required fields exist
2. `valA` stores the primary parameter (distance/radius)
3. `opA` stores the source group handle
4. `predef.entityB` stores the selected edge handle (single-edge MVP)
5. `remap` stores stable entity IDs for generated geometry
6. Params are stored/loaded for solver initialization
7. The serialized B-rep (Surface/Curve lines) is regenerated on load — chamfer geometry need not be specially serialized
