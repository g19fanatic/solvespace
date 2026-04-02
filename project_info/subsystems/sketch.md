# Sketch Data Model

> **Purpose**: Reference for the SolveSpace sketch data model — the in-memory representation of all parametric geometry, constraints, and groups that define a 3D/2D model. Understanding this subsystem is essential for any work involving geometry creation, constraint solving, file I/O, or the generation pipeline.

---

## Overview

A SolveSpace model is stored in a single global `Sketch` object (`SK`, declared via `extern Sketch SK` in `src/solvespace.h`). The sketch holds five ordered `IdList` collections keyed by typed handles, plus derived data generated each solve cycle.

**Primary source file**: `src/sketch.h` (1002 lines)  
**Handle/IdList infrastructure**: `src/dsc.h`, `src/param.h`, `src/handle.h`  
**Sketch class declaration**: `src/solvespace.h:398-420`

---

## The `Sketch` Class (`src/solvespace.h:398`)

```
class Sketch {
    // User-editable (persistent):
    IdList<Group,hGroup>            group;       // :401
    List<hGroup>                    groupOrder;  // :402
    IdList<CONSTRAINT,hConstraint>  constraint;  // :403
    IdList<Request,hRequest>        request;     // :404
    IdList<Style,hStyle>            style;       // :405

    // Generated (ephemeral, rebuilt each solve):
    IdList<ENTITY,hEntity>          entity;      // :408
    ParamList                       param;       // :409
}
```

> **Note**: `ENTITY` and `CONSTRAINT` are macros (`src/solvespace.h:393-396`) — they expand to `EntityBase`/`ConstraintBase` in library mode and `Entity`/`Constraint` in GUI mode.

Accessor methods at `:410-416`:
- `GetGroup(hGroup)`, `GetRequest(hRequest)`, `GetEntity(hEntity)`
- `GetConstraint(hConstraint)`, `GetParam(hParam)`

All call `IdList::FindById()` (O(log n) binary search on sorted handle index).

---

## Handle System

All identifiers are 32-bit typed handles — lightweight wrappers around `uint32_t v`. Each handle type is registered via `IsHandleOracle<T>` specialization (`src/handle.h`).

### Handle Types

| Handle Type | Source Location | Bits Layout | Maps To |
|-------------|----------------|-------------|---------|
| `hGroup` | `src/sketch.h:75` | bits 15:0 = group index | `Group` in `SK.group` |
| `hRequest` | `src/sketch.h:88` | bits 15:0 = request index | `Request` in `SK.request` |
| `hEntity` | `src/sketch.h:100` | bits 31:16 = request, 15:0 = entity index | `Entity` in `SK.entity` |
| `hConstraint` | `src/sketch.h:641` | opaque uint32_t | `Constraint` in `SK.constraint` |
| `hParam` | `src/param.h:13` | bits 31:16 = request, 15:0 = param index | `Param` in `SK.param` |
| `hEquation` | `src/sketch.h:63` | derived from constraint or group | `Equation` (solver) |
| `hStyle` | `src/sketch.h:131` | opaque uint32_t | `Style` in `SK.style` |

### Handle Encoding (inline methods at `src/sketch.h:841-960`)

```cpp
// hGroup generates entity/param/equation handles:
hGroup.entity(i)   → r.v = 0x80000000 | (v << 16) | i
hGroup.param(i)    → r.v = 0x80000000 | (v << 16) | i

// hRequest generates entity/param handles:
hRequest.entity(i) → r.v = (v << 16) | i
hRequest.param(i)  → r.v = (v << 16) | i

// hEntity determines source:
hEntity.isFromRequest() → !(v & 0x80000000)   // bit 31 = group-owned
hEntity.request()       → r.v = (v >> 16)     // upper 16 bits
```

---

## IdList Container (`src/dsc.h:383`)

`IdList<T, H>` is a sorted associative container keyed by handle value `H.v`.

**Internal storage**:
- `std::vector<T> elemstore` — flat storage for all elements
- `std::vector<int> elemidx` — sorted index into elemstore (sorted by `h.v`)
- `std::vector<int> freelist` — recycled slots

**Lookup**: `FindById(H h)` → binary search on `elemidx` via `CompareId<T,H>` → O(log n)

**Tag-based bulk delete**: Set `elem.tag = 1` then call `RemoveTagged()` — used pervasively during regeneration (see `src/code-patterns.md`).

---

## Group (`src/sketch.h:156`)

A `Group` is the top-level organizational unit. Groups are ordered and processed sequentially. Each group generates entities, equations, and mesh from its inputs.

### Group Types (`src/sketch.h:163`)

| Type | Value | Description |
|------|-------|-------------|
| `DRAWING_3D` | 5000 | Free 3D sketch (no workplane) |
| `DRAWING_WORKPLANE` | 5001 | 2D sketch on a workplane |
| `EXTRUDE` | 5100 | Extrusion of a profile |
| `LATHE` | 5101 | Revolution around an axis |
| `REVOLVE` | 5102 | Full revolution |
| `HELIX` | 5103 | Helical extrusion |
| `ROTATE` | 5200 | Step-and-repeat by rotation |
| `TRANSLATE` | 5201 | Step-and-repeat by translation |
| `LINKED` | 5300 | Imported linked file |
| `CHAMFER` | 5400 | Edge chamfer on flat faces (direct topology injection) |
| `FILLET` | 5401 | Edge fillet on flat faces (direct topology injection) |

**Special constant**: `Group::HGROUP_REFERENCES` — the always-present reference group (XY, YZ, ZX planes).

### Group Key Fields (`src/sketch.h:158-240`)

> **Remap constants** used by chamfer/fillet (`src/sketch.h:313-319`):
> `REMAP_CHAMFER_FACE = 1011`, `REMAP_FILLET_FACE = 1012`

```
hGroup      h               // own handle
Group::Type type
int         order           // sequential solve order
hGroup      opA, opB        // source groups for boolean ops
hEntity     activeWorkplane // workplane entity, or Entity::FREE_IN_3D
bool        visible, suppress
bool        relaxConstraints, allowRedundant
bool        suppressDofCalculation
CombineAs   meshCombine     // UNION/DIFFERENCE/ASSEMBLE/INTERSECTION

struct solved {
    SolveResult how          // solve status (OKAY/DIDNT_CONVERGE/etc.)
    int         dof          // remaining degrees of freedom
    List<hConstraint> remove // constraints to highlight as redundant
}

EntityMap   remap            // (hEntity,copyNumber) → EntityId for step-and-repeat
SMesh       thisShell/runningMesh  // per-group and cumulative mesh
```

### Group Subtypes (`src/sketch.h:213-224`)

| Subtype | Value | Usage |
|---------|-------|-------|
| `WORKPLANE_BY_POINT_ORTHO` | 6000 | Workplane from point + axis alignment |
| `WORKPLANE_BY_LINE_SEGMENTS` | 6001 | Workplane from two line segments |
| `WORKPLANE_BY_POINT_NORMAL` | 6002 | Workplane from point + normal |
| `ONE_SIDED` | 7000 | Extrude/rotate in one direction |
| `TWO_SIDED` | 7001 | Extrude/rotate symmetrically |

### Group Key Methods (`src/sketch.h:288-360`)

| Method | Purpose |
|--------|---------|
| `Generate(EntityList*, ParamList*)` | Core generation: creates entities and params |
| `GenerateEquations(IdList<Equation,hEquation>*)` | Creates constraint equations for solver |
| `GenerateShellAndMesh()` | Produces NURBS shell and triangle mesh |
| `GenerateLoops()` | Assembles sketch curves into closed loops |
| `IsSolvedOkay()` | Returns true if last solve succeeded |
| `PreviousGroup()` / `RunningMeshGroup()` | Mesh chaining for boolean ops |
| `Remap(hEntity, copyNumber)` | Maps entity IDs for step-and-repeat copies |

---

## Request (`src/sketch.h:364`)

A `Request` is a user-placed geometric primitive. Requests belong to a group and generate one or more `Entity` objects when processed.

### Request Types (`src/sketch.h:378`)

| Type | Value | Entities Generated |
|------|-------|--------------------|
| `WORKPLANE` | 100 | Workplane + normal + origin |
| `DATUM_POINT` | 101 | Single point |
| `LINE_SEGMENT` | 200 | Line + 2 endpoints |
| `CUBIC` | 300 | Cubic Bézier + 4 control points |
| `CUBIC_PERIODIC` | 301 | Closed cubic Bézier |
| `CIRCLE` | 400 | Circle + center + normal + distance |
| `ARC_OF_CIRCLE` | 500 | Arc + center + 2 endpoints |
| `TTF_TEXT` | 600 | TrueType text curves |
| `IMAGE` | 700 | Embedded image |

### Reference Requests

Three always-present reference requests define the coordinate system:
- `Request::HREQUEST_REFERENCE_XY` — XY plane
- `Request::HREQUEST_REFERENCE_YZ` — YZ plane
- `Request::HREQUEST_REFERENCE_ZX` — ZX plane

Test: `hRequest::IsFromReferences()` (`src/sketch.h:853`)

### Request Key Fields (`src/sketch.h:395-413`)

```
hRequest    h
Request::Type type
hEntity     workplane    // constraint plane, or Entity::FREE_IN_3D
hGroup      group
hStyle      style
bool        construction // construction geometry flag
```

---

## Entity / EntityBase (`src/sketch.h:416`)

Entities are the actual geometric objects in the sketch. There are two classes:
- **`EntityBase`** (`src/sketch.h:416`): core fields and computation methods (used in library mode)
- **`Entity : EntityBase`** (`src/sketch.h:553`): adds rendering data (`beziers`, `edges`, `screenBBox`, `actPoint/actNormal/actDistance`)

### Entity Types (`src/sketch.h:422`)

**Points** (2000-2014): `POINT_IN_3D`, `POINT_IN_2D`, `POINT_N_TRANS`, `POINT_N_ROT_TRANS`, `POINT_N_COPY`, `POINT_N_ROT_AA`, `POINT_N_ROT_AXIS_TRANS`

**Normals** (3000-3012): `NORMAL_IN_3D`, `NORMAL_IN_2D`, `NORMAL_N_COPY`, `NORMAL_N_ROT`, `NORMAL_N_ROT_AA`

**Distances** (4000-4001): `DISTANCE`, `DISTANCE_N_COPY`

**Faces** (5000-5006): `FACE_NORMAL_PT`, `FACE_XPROD`, and variants

**Geometry** (10000+): `WORKPLANE`=10000, `LINE_SEGMENT`=11000, `CUBIC`=12000/12001, `CIRCLE`=13000, `ARC_OF_CIRCLE`=14000, `TTF_TEXT`=15000, `IMAGE`=16000

### Entity Key Fields (`src/sketch.h:436-460`)

```
hGroup      group
hEntity     workplane           // or Entity::FREE_IN_3D
hEntity     point[12]           // MAX_POINTS_IN_ENTITY defining points
hEntity     normal, distance    // auxiliary entities
hParam      param[8]            // solver parameters (points/normals)
Vector      numPoint            // numerical value (computed each solve)
Quaternion  numNormal
double      numDistance
int         timesApplied        // for step-and-repeat transforms
```

### Entity Key Methods (`src/sketch.h:462-545`)

| Method | Purpose |
|--------|---------|
| `IsPoint()` / `PointGetNum()` / `PointGetExprs()` | Point type query and value |
| `IsNormal()` / `NormalGetNum()` / `NormalGetExprs()` | Normal type query and value |
| `IsCircle()` / `CircleGetRadiusNum()` | Circle/arc radius |
| `IsWorkplane()` / `WorkplaneGetPlaneExprs()` | Workplane plane equation |
| `IsDistance()` / `DistanceGetNum()` | Distance scalar value |
| `GenerateEquations(IdList<Equation,hEquation>*)` | Generate constraint equations |
| `CalculateNumerical(bool forExport)` | Evaluate actual coordinates from params |

---

## Constraint / ConstraintBase (`src/sketch.h:648`)

Constraints define relationships between entities. `ConstraintBase` holds the solver interface; `Constraint` extends it with display methods.

### Constraint Handle (`src/sketch.h:641`)

```cpp
class hConstraint { uint32_t v; }
// Also generates: hEquation equation(int i), hParam param(int i)
```

### Constraint Types (`src/sketch.h:656`)

| Type | Value | Description |
|------|-------|-------------|
| `POINTS_COINCIDENT` | 20 | Two points at same location |
| `PT_PT_DISTANCE` | 30 | Distance between two points |
| `PT_PLANE_DISTANCE` | 31 | Point to plane distance |
| `PT_LINE_DISTANCE` | 32 | Point to line distance |
| `PT_ON_LINE` | 42 | Point lies on line |
| `EQUAL_LENGTH_LINES` | 50 | Two lines same length |
| `LENGTH_RATIO` | 51 | Ratio of line lengths |
| `SYMMETRIC` | 60 | Symmetric about plane |
| `HORIZONTAL` | 80 | Line is horizontal |
| `VERTICAL` | 81 | Line is vertical |
| `DIAMETER` | 90 | Circle diameter |
| `ANGLE` | 120 | Angle between lines |
| `PARALLEL` | 121 | Lines parallel |
| `PERPENDICULAR` | 122 | Lines perpendicular |
| `EQUAL_RADIUS` | 130 | Arcs same radius |
| `WHERE_DRAGGED` | 200 | Point location locked (drag anchor) |
| `COMMENT` | 1000 | Text comment (no equations) |

### Constraint Fields (`src/sketch.h:687-700`)

```
hConstraint h
Type        type
hGroup      group
hEntity     workplane
double      valA            // scalar value (distance/angle)
hParam      valP            // parameter handle for driven dims
hEntity     ptA, ptB        // endpoint entities
hEntity     entityA, entityB, entityC, entityD  // geometry entities
bool        reference       // reference dim: generates no equations
std::string comment         // text for COMMENT type
```

---

## Param / hParam (`src/param.h`)

Parameters are the solver's unknowns — scalar `double` values tracked by handle.

```cpp
class hParam { uint32_t v; }  // bits 31:16 = request, 15:0 = param index

class Param {
    hParam  h
    double  val     // current value (updated by solver)
    bool    known   // fixed value (not a DOF)
    bool    free    // can be dragged
    Param  *substd  // solver substitution chain
}
```

`ParamList = IdList<Param, hParam>` — lives in `SK.param`.

---

## EntReqTable (`src/sketch.h:629`)

A utility class mapping between `Request::Type` and `EntityBase::Type`. Used during generation to determine how many points/normals a request generates.

```cpp
class EntReqTable {
    static void GetRequestInfo(Request::Type, int extraPoints,
                               EntityBase::Type*, int* pts, bool* hasNormal, bool* hasDistance);
    static bool GetEntityInfo(EntityBase::Type, int extraPoints,
                              Request::Type*, ...);
    static Request::Type GetRequestForEntity(EntityBase::Type);
}
```

---

## Data Flow: Request → Entity → Param

```
User creates Request (e.g., LINE_SEGMENT)
    ↓
Request::Generate(EntityList*, ParamList*)    [src/request.cpp]
    → Creates 2 endpoint Entity objects (POINT_IN_3D or POINT_IN_2D)
    → Creates hParam entries for each coordinate (x,y,z or u,v)
    → Populates entity.point[], entity.param[]
    ↓
Constraint::GenerateEquations(IdList<Equation>*)  [src/constraint.cpp]
    → Reads entity.PointGetExprs() → symbolic Expr trees
    → Adds equations to solver equation list
    ↓
System::Solve()   [src/system.cpp]
    → Numeric Newton-Raphson on Param values
    → Updates Param.val for each hParam
    ↓
Entity::CalculateNumerical()
    → Reads Param.val → fills entity.actPoint, actNormal, actDistance
    ↓
Entity::Draw()   [renders using actPoint/actNormal values]
```

---

## Global Singletons

- `SK` — `extern Sketch SK` (the live sketch, `src/solvespace.h`)
- `SS` — `extern SolveSpaceUI SS` (UI/app state, `src/solvespace.h`)

Access pattern: `SK.GetEntity(h)`, `SK.group.FindById(hg)`, etc.

---

## Key File References

| File | Content |
|------|---------|
| `src/sketch.h:61-130` | All handle type declarations (hGroup, hRequest, hEntity, hConstraint, hEquation) |
| `src/sketch.h:156-360` | `Group` class: types, fields, key methods |
| `src/sketch.h:364-414` | `Request` class: types and fields |
| `src/sketch.h:416-600` | `EntityBase` + `Entity`: types, fields, computation methods |
| `src/sketch.h:629-720` | `hConstraint`, `ConstraintBase`: types and fields |
| `src/param.h:13-50` | `hParam` and `Param` class |
| `src/solvespace.h:398-420` | `Sketch` class (master container) |
| `src/dsc.h:351-730` | `IdList<T,H>` template implementation |
| `src/handle.h` | `IsHandleOracle` trait registration |

---

## Cross-References

- **Solver lifecycle**: see [`project_info/subsystems/solver.md`](solver.md) — how params and equations flow through `System::Solve()`
- **Code patterns**: see [`project_info/code-patterns.md`](../code-patterns.md) — Handle/IdList pattern, tag-based deletion, global singletons
- **Architecture**: see [`project_info/architecture.md`](../architecture.md) — where sketch fits in the overall data flow
