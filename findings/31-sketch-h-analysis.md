# Task 31: Full Analysis of src/sketch.h — Extension Points for Chamfer/Fillet

## Overview

`src/sketch.h` is the core data model header for SolveSpace. It defines all Group, Entity, 
Request, and Constraint types. This analysis catalogs every relevant type and identifies 
precise extension points for chamfer/fillet.

---

## 1. Group::Type Enum — Complete Catalog

Located at `src/sketch.h:181-189`:

```cpp
enum class Type : uint32_t {
    DRAWING_3D       = 5000,
    DRAWING_WORKPLANE = 5001,
    EXTRUDE          = 5100,
    LATHE            = 5101,
    REVOLVE          = 5102,
    HELIX            = 5103,
    ROTATE           = 5200,
    TRANSLATE        = 5201,
    LINKED           = 5300
};
```

**Extension Point:** Add after LINKED=5300:
```cpp
    CHAMFER          = 5400,  // flat angled cut at a solid edge
    FILLET           = 5401,  // rounded blend at a solid edge
```

These values are numerically consistent with the existing pattern and avoid collisions.

---

## 2. Group Struct — Complete Field Catalog

Located at `src/sketch.h` (Group class definition):

### Identity Fields
| Field | Type | Purpose |
|-------|------|---------|
| `tag` | `int` | used by IdList for mark/sweep |
| `h` | `hGroup` | handle (identity) of this group |
| `type` | `Group::Type` | which kind of group this is |
| `order` | `int` | position in group sequence |
| `name` | `std::string` | user-visible name |

### Operation Fields  
| Field | Type | Purpose for Chamfer/Fillet |
|-------|------|---------|
| `opA` | `hGroup` | **source group** — the solid being chamfered/filleted |
| `opB` | `hGroup` | unused for chamfer/fillet |
| `valA` | `double` | **chamfer distance OR fillet radius** (PRIMARY PARAM) |
| `valB` | `double` | second distance for unequal-leg chamfer (MVP: 0) |
| `valC` | `double` | ⚠️ BUG: copy-paste bug in file.cpp points to valB storage — avoid |
| `scale` | `double` | unused for chamfer/fillet |
| `color` | `RgbaColor` | group color — copy from source group |

### Boolean/Mesh Operation
| Field | Type | Purpose |
|-------|------|---------|
| `meshCombine` | `CombineAs` | how this group combines with previous; DIFFERENCE for chamfer/fillet |
| `forceToMesh` | `bool` | force triangle mesh instead of NURBS shell |
| `suppress` | `bool` | suppress this group from boolean pipeline |

### Predefined Selection
| Field | Type | Purpose for Chamfer/Fillet |
|-------|------|---------|
| `predef.q` | `Quaternion` | unused for chamfer/fillet |
| `predef.origin` | `hEntity` | unused for chamfer/fillet |
| `predef.entityB` | `hEntity` | **face 0 handle** (first selected face) |
| `predef.entityC` | `hEntity` | **face 1 handle** (second selected face) |
| `predef.swapUV` | `bool` | unused for chamfer/fillet |
| `predef.negateU` | `bool` | unused for chamfer/fillet |
| `predef.negateV` | `bool` | unused for chamfer/fillet |

### Derived/Computed Data (NOT persisted; regenerated every regen)
| Field | Type | Notes |
|-------|------|-------|
| `thisShell` | `SShell` | the chamfer/fillet geometry (surfaces + curves) |
| `runningShell` | `SShell` | cumulative boolean result through this group |
| `thisMesh` | `SMesh` | triangle mesh version (used if forceToMesh) |
| `runningMesh` | `SMesh` | cumulative mesh |
| `displayMesh` | `SMesh` | what actually gets rendered |
| `displayOutlines` | `SOutlineList` | edges drawn in viewport |
| `displayDirty` | `bool` | whether display needs refresh |
| `booleanFailed` | `bool` | whether the geometry operation failed |
| `clean` | `bool` | whether this group has been regenerated |
| `polyLoops` | `SPolygon` | irrelevant for 3D groups |
| `bezierLoops` | `SBezierLoopSetSet` | irrelevant for 3D groups |
| `bezierOpens` | `SBezierLoopSet` | irrelevant for 3D groups |
| `polyError` | `struct` | irrelevant for 3D groups |

### EntityMap
| Field | Type | Purpose |
|-------|------|---------|
| `remap` | `EntityMap` | **stable topological name table** — maps (hEntity, copyNumber) → EntityId |

The remap table is the key to stable face references after regeneration. For chamfer/fillet,
this is critical because face handles may shift across regenerations if not properly remapped.

### Linked File Fields (irrelevant for chamfer/fillet)
- `linkFile`, `impMesh`, `impShell`, `impEntity`

---

## 3. REMAP Constants — Extension Points

Located at `src/sketch.h:305-315`:

```cpp
enum {
    REMAP_LAST             = 1000,
    REMAP_TOP              = 1001,
    REMAP_BOTTOM           = 1002,
    REMAP_PT_TO_LINE       = 1003,
    REMAP_LINE_TO_FACE     = 1004,
    REMAP_LATHE_START      = 1006,
    REMAP_LATHE_END        = 1007,
    REMAP_PT_TO_ARC        = 1008,
    REMAP_PT_TO_NORMAL     = 1009,
    REMAP_LATHE_ARC_CENTER = 1010,
};
```

**Next available values:** 1011, 1012

**Extension Point:** Add:
```cpp
    REMAP_CHAMFER_FACE = 1011,  // face entity for chamfer surface
    REMAP_FILLET_FACE  = 1012,  // face entity for fillet surface
```

These are used in `GenerateShellAndMesh()` when setting `ss->face` for the new surfaces, 
enabling those surfaces to be selectable/hoverable in the viewport.

---

## 4. Group Methods — Complete Catalog

### Core Lifecycle
| Method | Purpose | Chamfer/Fillet relevance |
|--------|---------|------------------------|
| `Clear()` | Free dynamic memory | no change needed |
| `IsVisible()` | visibility check | no change needed |
| `Activate()` | called when group becomes active | may need face selection mode setup |
| `IsMeshGroup()` | whether this group contributes to solid model | **MUST add CHAMFER/FILLET** |
| `IsSolvedOkay()` | solver status check | no change needed |

### Generation Pipeline
| Method | Purpose | Chamfer/Fillet relevance |
|--------|---------|------------------------|
| `Generate(entity, param)` | create Entity objects from this group | minimal: only `AddParam(h.param(0), valA)` needed |
| `GenerateEquations(l)` | emit constraint equations | probably none needed for MVP |
| `AssembleLoops(...)` | build 2D sketch loops | irrelevant for 3D dressup groups |
| `GenerateLoops()` | populate bezierLoops | irrelevant for 3D dressup groups |
| `GenerateShellAndMesh()` | **build 3D geometry** | **PRIMARY HOOK: implement CHAMFER/FILLET geometry here** |
| `GenerateForBoolean(a,b,o,how)` | apply boolean to running shell | template; reused via DIFFERENCE |
| `GenerateForStepAndRepeat(...)` | step-and-repeat aggregation | irrelevant |
| `GenerateDisplayItems()` | triangulate for display | no change needed; auto-handles new surfaces |

### Entity Generation Helpers  
| Method | Purpose | Chamfer/Fillet relevance |
|--------|---------|------------------------|
| `AddParam(param, hp, v)` | register a solver param | **used in Generate() for chamfer distance/radius** |
| `Remap(in, copyNumber)` | stable entity ID mapping | **used to assign face field on new surfaces** |
| `CopyEntity(...)` | copy entity with transformation | not used for chamfer/fillet |
| `MakeExtrusionLines(...)` | create extrusion side lines | not used |
| `MakeLatheCircles(...)` | create arc entities for lathe | not used |
| `MakeLatheSurfacesSelectable(...)` | create face entities for lathe | not used |
| `MakeRevolveEndFaces(...)` | create end cap face entities | not used |
| `MakeExtrusionTopBottomFaces(...)` | top/bottom face entities for extrude | not used |

### Mesh Group Traversal
| Method | Purpose |
|--------|---------|
| `PreviousGroup()` | find the previous group in order |
| `RunningMeshGroup()` | find the cumulative solid mesh group |

### Drawing
| Method | Chamfer/Fillet relevance |
|--------|------------------------|
| `DrawMesh(how, canvas)` | renders solid; handles HOVERED/SELECTED face highlighting — no change needed |
| `Draw(canvas)` | main draw call — no change needed |
| `DrawPolyError(canvas)` | 2D sketch error display — irrelevant for 3D groups |
| `DrawFilledPaths(canvas)` | 2D contour fill — irrelevant |
| `DrawContourAreaLabels(canvas)` | area labels — irrelevant |

### UI/Menu
| Method | Purpose | Chamfer/Fillet relevance |
|--------|---------|------------------------|
| `MenuGroup(Command id)` | **create new group from menu action** | **ADD cases for GROUP_CHAMFER, GROUP_FILLET** |
| `MenuGroup(Command id, Platform::Path)` | overload with linked file path | not relevant |
| `DescriptionString()` | short description for UI | no change needed |

---

## 5. Group::Subtype Enum

Located in sketch.h Group class:
```cpp
enum class Subtype : uint32_t {
    WORKPLANE_BY_POINT_ORTHO   = 6000,
    WORKPLANE_BY_LINE_SEGMENTS = 6001,
    WORKPLANE_BY_POINT_NORMAL  = 6002,
    ONE_SIDED                  = 7000,
    TWO_SIDED                  = 7001,
    ONE_SKEWED                 = 7004,
    TWO_SKEWED                 = 7005
};
```

**Chamfer/Fillet**: No new Subtype needed for MVP. Subtype is unused by CHAMFER/FILLET groups.

---

## 6. Group::CombineAs Enum

```cpp
enum class CombineAs : uint32_t {
    UNION        = 0,
    DIFFERENCE   = 1,
    ASSEMBLE     = 2,
    INTERSECTION = 3
};
```

**Chamfer uses `DIFFERENCE` always.** This is set in `MenuGroup()` when creating the group:
```cpp
g.meshCombine = CombineAs::DIFFERENCE;
```

The chamfer group's `thisShell` will contain the chamfer surface geometry (the new face being 
added), but the boolean operation is how we replace the modified faces. Wait — actually, the 
approach for direct topology injection is different: we bypass `meshCombine` entirely and 
directly modify the shell. See task 32 findings for details.

---

## 7. Request::Type Enum — Complete Catalog

Located at `src/sketch.h` (Request class):

```cpp
enum class Type : uint32_t {
    WORKPLANE       = 100,
    DATUM_POINT     = 101,
    LINE_SEGMENT    = 200,
    CUBIC           = 300,
    CUBIC_PERIODIC  = 301,
    CIRCLE          = 400,
    ARC_OF_CIRCLE   = 500,
    TTF_TEXT        = 600,
    IMAGE           = 700
};
```

**For 2D Sketch Fillet Group:** Generates `ARC_OF_CIRCLE` requests in the source sketch group.
**For 3D Chamfer/Fillet Group:** Generates NO new requests. All geometry goes directly into SShell.

---

## 8. EntityBase::Type Enum — Complete Catalog

Located at `src/sketch.h` EntityBase class:

### Point Types
```cpp
POINT_IN_3D            = 2000,   // free point in 3D, has 3 params
POINT_IN_2D            = 2001,   // point in workplane, has 2 params + workplane ref
POINT_N_TRANS          = 2010,   // numerically translated copy
POINT_N_ROT_TRANS      = 2011,   // numerically rotated+translated copy
POINT_N_COPY           = 2012,   // numeric copy
POINT_N_ROT_AA         = 2013,   // rotated about axis
POINT_N_ROT_AXIS_TRANS = 2014,   // rotated about axis + translated
```

### Normal Types (orientation)
```cpp
NORMAL_IN_3D    = 3000,   // free normal in 3D
NORMAL_IN_2D    = 3001,   // normal in workplane
NORMAL_N_COPY   = 3010,   // numeric copy
NORMAL_N_ROT    = 3011,   // numerically rotated copy
NORMAL_N_ROT_AA = 3012,   // rotated about axis
```

### Distance Types
```cpp
DISTANCE       = 4000,  // distance with a free param
DISTANCE_N_COPY = 4001, // numeric copy
```

### Face Types — Key for Chamfer/Fillet Selection
```cpp
FACE_NORMAL_PT         = 5000,  // flat face: defined by outward normal + a point on face
FACE_XPROD             = 5001,  // flat face: defined by cross product of two vectors
FACE_N_ROT_TRANS       = 5002,  // face that was rotation+translated
FACE_N_TRANS           = 5003,  // face that was translated (extrude side faces)
FACE_N_ROT_AA          = 5004,  // face rotated about axis
FACE_ROT_NORMAL_PT     = 5005,  // face with rotation applied to normal+point
FACE_N_ROT_AXIS_TRANS  = 5006,  // face with helix transform
```

**Key insight**: Face entities are NOT stored by SShell; they live in SK.entity and are 
generated during `GenerateShellAndMesh()`. They link back to SSurface via the `ss->face` field.

For chamfer/fillet: the new chamfer/fillet surface needs a face entity so the user can 
select it. This is done with:
```cpp
Entity en = {};
en.type = Entity::Type::FACE_NORMAL_PT;
en.numNormal = Quaternion::From(0, normal.x, normal.y, normal.z);
en.numPoint = pointOnFace;
en.point[0] = Remap(Entity::NO_ENTITY, REMAP_CHAMFER_FACE);
en.h = Remap(Entity::NO_ENTITY, REMAP_CHAMFER_FACE);
en.group = h;
entity->Add(&en);
ss->face = en.h.v;  // link surface to face entity
```

However, for chamfer/fillet MVP, face entities for the NEW faces are less critical — the 
source faces' handles (stored in predef.entityB/C) are what we need to identify the shared 
edge, and the new chamfer face doesn't need to be selectable by the user initially.

### Curve/Profile Types
```cpp
WORKPLANE    = 10000,
LINE_SEGMENT = 11000,
CUBIC        = 12000,
CUBIC_PERIODIC = 12001,
CIRCLE       = 13000,
ARC_OF_CIRCLE = 14000,
TTF_TEXT     = 15000,
IMAGE        = 16000,
```

**No new Entity types needed for 3D chamfer/fillet MVP.** The geometry lives in SShell, not
in SK.entity.

---

## 9. Constraint Types — Relevant for 2D Fillet

Key constraint types for 2D sketch fillet:

```cpp
ARC_LINE_TANGENT    = 123,  // arc tangent to line at endpoint
CUBIC_LINE_TANGENT  = 124,  // cubic tangent to line
CURVE_CURVE_TANGENT = 125,  // curve tangent to another curve
```

These are used when creating a parametric 2D fillet group that generates `ARC_OF_CIRCLE` 
entities and then constrains them tangent to the adjacent lines.

---

## 10. Extension Points Summary — What Needs to Change in sketch.h

### Changes Required

1. **Add Group::Type values** (after LINKED=5300):
   ```cpp
   CHAMFER = 5400,
   FILLET  = 5401,
   ```

2. **Add REMAP constants** (after REMAP_LATHE_ARC_CENTER=1010):
   ```cpp
   REMAP_CHAMFER_FACE = 1011,
   REMAP_FILLET_FACE  = 1012,
   ```

### What Does NOT Change in sketch.h

- No new Entity types — chamfer/fillet geometry lives in SShell (not SK.entity)
- No new Request types — 3D chamfer/fillet creates no new user-editable sketch entities
- No new Constraint types — the chamfer distance is a stored value (valA), not constrained
- No new Subtype values — chamfer/fillet groups have no meaningful subtypes for MVP
- Group struct fields — all needed fields already exist:
  - `opA` = source group reference ✓
  - `valA` = chamfer distance / fillet radius ✓
  - `valB` = second distance (unequal-leg chamfer) ✓
  - `predef.entityB` = first selected face ✓
  - `predef.entityC` = second selected face ✓
  - `meshCombine` = will be DIFFERENCE ✓
  - `remap` = EntityMap for stable face IDs ✓
  - `thisShell` = where new surfaces go ✓
  - `runningShell` = boolean result ✓
  - `booleanFailed` = error indicator ✓

---

## 11. IsMeshGroup() — Critical Extension

Located at `src/groupmesh.cpp:545`:

```cpp
bool Group::IsMeshGroup() {
    switch(type) {
        case Group::Type::EXTRUDE:
        case Group::Type::LATHE:
        case Group::Type::REVOLVE:
        case Group::Type::HELIX:
        case Group::Type::ROTATE:
        case Group::Type::TRANSLATE:
            return true;
        default:
            return false;
    }
}
```

**Must extend to:**
```cpp
        case Group::Type::CHAMFER:
        case Group::Type::FILLET:
            return true;
```

Without this, chamfer/fillet groups will be treated as sketch groups, and their 
`thisShell`/`runningShell` will never be used in the solid model.

---

## 12. GroupSelection() and Face Selection Context

When the user invokes the chamfer/fillet menu command:

1. `SS.GW.GroupSelection()` is called (in group.cpp MenuGroup) 
2. It populates `gs.face[]` with IsFace() entities that are currently selected
3. Check: `gs.faces == 2` (exactly two faces selected)
4. Store: `g.predef.entityB = gs.face[0]`, `g.predef.entityC = gs.face[1]`
5. Store: `g.opA = SS.GW.activeGroup` (the solid group being modified)

The validation logic in MenuGroup() for CHAMFER case:
```cpp
case Command::GROUP_CHAMFER:
    if(gs.faces != 2) {
        Error(_("Select exactly two adjacent faces for chamfer."));
        return;
    }
    // Verify faces share an edge (done during Generate())
    g.type = Type::CHAMFER;
    g.opA = SS.GW.activeGroup;
    g.predef.entityB = gs.face[0];
    g.predef.entityC = gs.face[1];
    g.meshCombine = CombineAs::DIFFERENCE;
    g.valA = SS.StringToMm("1");  // default 1mm chamfer
    g.name = C_("group-name", "chamfer");
    break;
```

Note: there is NO `LockedInWorkplane()` check for chamfer/fillet — these operate on 3D 
solids, not workplanes.

---

## 13. Full Dependency Diagram

Files that need changes vs. files that need no changes:

### Changes Required (confirmed)
- `src/sketch.h`: Add CHAMFER/FILLET Group types and REMAP_CHAMFER_FACE/REMAP_FILLET_FACE
- `src/groupmesh.cpp`: IsMeshGroup() + GenerateShellAndMesh() CHAMFER/FILLET cases
- `src/group.cpp`: MenuGroup() CHAMFER/FILLET cases, Generate() CHAMFER/FILLET cases
- `src/graphicswin.cpp`: Menu entries for GROUP_CHAMFER, GROUP_FILLET
- `src/ui.h`: Command enum (GROUP_CHAMFER, GROUP_FILLET) + TextWindow Edit enum
- `src/textscreens.cpp`: ShowGroupInfo() CHAMFER/FILLET branches + ScreenChangeChamferOffset
- `src/srf/chamfer.cpp` (NEW FILE): SShell::MakeFromChamferOf() implementation
- `src/CMakeLists.txt`: Add chamfer.cpp to build

### No Changes Required
- `src/sketch.h`: Entity types, Request types, Constraint types (all unchanged)
- `src/srf/surface.h`: No new surface types needed (degree 1,1 and 2,1 already work)
- `src/srf/triangulate.cpp`: Auto-handles new surface degrees
- `src/srf/surface.cpp`: SSurface::FromPlane() and FromExtrusionOf() already exist
- `src/srf/boolean.cpp`: Not used for direct topology injection approach
- `src/file.cpp`: SAVED[] table already covers all needed fields (valA, valB, predef.entityB, predef.entityC, opA, meshCombine, remap)
- `src/exportstep.cpp`: RATIONAL_B_SPLINE_SURFACE export already handles degree 1,1 and 2,1
- `src/srf/raycast.cpp`: IsCylinder() path already handles fillet surfaces

---

## 14. Key Observations

1. **Group struct is sufficient**: All necessary storage fields (opA, valA, valB, predef.entityB/C, meshCombine, remap) already exist in the Group struct. No new fields needed.

2. **Entity system bypass**: 3D chamfer/fillet bypasses the Entity system entirely — geometry goes directly into SShell surfaces and curves. The Entity system is only for 2D sketches.

3. **REMAP mechanism is the key to stable references**: The `remap` EntityMap converts (input_entity, copy_number) → stable EntityId. Using REMAP_CHAMFER_FACE and REMAP_FILLET_FACE with `Remap(Entity::NO_ENTITY, REMAP_CHAMFER_FACE)` creates stable handle for the new face.

4. **IsMeshGroup() is the gating function**: Without adding CHAMFER/FILLET there, the group is invisible to the solid modeling pipeline.

5. **valC is buggy**: Do not use `valC` in Group — it has a known copy-paste bug in file.cpp where it reads from valB's storage location. Use only valA and valB.

6. **Group::CombineAs::DIFFERENCE is NOT used for direct injection**: The planned implementation approach (direct topology injection, like MakeFromAssemblyOf but with modification) bypasses the normal boolean pipeline. The `meshCombine` field may be set to DIFFERENCE but the actual GenerateShellAndMesh() code handles it specially.
