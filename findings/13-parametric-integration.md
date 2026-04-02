# Task 13: Parametric Integration for Chamfer/Fillet in SolveSpace

## Overview

This document analyzes how SolveSpace's constraint solver (Newton-Raphson via Eigen sparse QR)
would be used — or NOT used — for chamfer/fillet parameters. The key finding is that
**chamfer/fillet parameters do NOT need to be solver variables** for an MVP implementation.
They behave more like the helix pitch (`valB`) or extrusion depth: stored values read during
geometry generation, not constrained in the symbolic algebra system.

---

## 1. The Param System (`src/param.h`)

```cpp
class hParam {
public:
    // bits 15:0  -- param index
    // bits 31:16 -- request index
    uint32_t v;
    inline hRequest request() const;
};

class Param {
public:
    int    tag;
    hParam h;
    double val;
    bool   known;
    bool   free;
    Param *substd;  // used only in the solver
};
```

`hParam` is a 32-bit handle. For **group params**, `h.param(i)` is generated as:
```cpp
// From sketch.h:942
inline hParam hGroup::param(int i) const {
    hParam r; r.v = 0x80000000 | (v << 16) | (uint32_t)i; return r;
}
```
This means group `g` with handle `v` owns params at indices `0x80000000 | (v<<16) | i`.

The global param table is `SK.param` (an `IdList<Param, hParam>`). Params are retrieved via
`SK.GetParam(hParam)`. They are populated during `Group::Generate()` via `Group::AddParam()`.

---

## 2. `Group::AddParam()` — How Params Are Created

```cpp
// src/group.cpp:39
void Group::AddParam(ParamList *param, hParam hp, double v) {
    Param pa = {};
    pa.h = hp;
    pa.val = v;
    param->Add(&pa);
}
```

Called during `Group::Generate()` to register that group `h` "owns" parameter `h.param(i)` with
initial value `v`. The solver can then modify this value freely (within constraints).

---

## 3. How Different Groups Use Params

### EXTRUDE — Params 0,1,2 = translation vector (x,y,z)

```cpp
// src/group.cpp:500-502 (Generate)
AddParam(param, h.param(0), gn.x);  // extrusion direction X
AddParam(param, h.param(1), gn.y);  // extrusion direction Y
AddParam(param, h.param(2), gn.z);  // extrusion direction Z
```

During `GenerateShellAndMesh()`, the extrusion direction is read back:
```cpp
// src/groupmesh.cpp:250
Vector translate = Vector::From(h.param(0), h.param(1), h.param(2));
```

`Vector::From(hParam, hParam, hParam)` calls `SK.GetParam(hp)->val` for each.

In `GenerateEquations()`, the extrusion direction is **constrained** to be normal to the
workplane:
```cpp
AddEq(l, u.Dot(extruden), 0);  // must be perpendicular to workplane u-axis
AddEq(l, v.Dot(extruden), 1);  // must be perpendicular to workplane v-axis
```

The solver adjusts h.param(0..2) to satisfy these constraints. The **magnitude** of the
translation vector encodes the extrusion depth — it is **free** (no magnitude constraint),
so the user sets it by dragging in the viewport.

### HELIX — Params 0..7 (axis, rotation, translation)

```cpp
// Generate():
AddParam(param, h.param(0), axis_pos.x);  // pivot X
AddParam(param, h.param(1), axis_pos.y);  // pivot Y
AddParam(param, h.param(2), axis_pos.z);  // pivot Z
AddParam(param, h.param(3), 30*PI/180);   // rotation angle (free)
AddParam(param, h.param(4), axis_dir.x);  // axis dir X
AddParam(param, h.param(5), axis_dir.y);  // axis dir Y
AddParam(param, h.param(6), axis_dir.z);  // axis dir Z
AddParam(param, h.param(7), 20);          // translation distance (free)
```

The pitch `valB` is a **stored value** (not a solver param):
```cpp
// GenerateEquations():
if(type == Type::HELIX && valB != 0.0) {
    // pitch = translation / (angle / PI) → enforces helical relationship
    AddEq(l, Expr::From(h.param(7))->Times(Expr::From(PI))
             ->Minus(Expr::From(h.param(3))->Times(Expr::From(valB))), 6);
}
```

When `valB != 0`, the solver enforces `h.param(7)*PI = h.param(3)*valB` (the pitch constraint).
When `valB == 0`, both angle and translation are free independently.

**Key insight**: `valB` is a **stored scalar** in the Group struct (not a hParam), used as a
constant in constraint equations. This is the pattern chamfer/fillet should follow.

### ROTATE/REVOLVE/LINKED — Similar pattern

ROTATE/REVOLVE add 6-7 params for rotation axis + angle. LINKED adds 7 params for position +
quaternion. All read back via `SK.GetParam(h.param(i))->val` in GenerateShellAndMesh.

---

## 4. The Solver's Role (`src/system.cpp`)

The solver operates on a **symbolic algebra system**:

1. `WriteEquationsExceptFor()` — collects all constraint equations from:
   - `c->GenerateEquations(&eq)` — constraint equation
   - `e->GenerateEquations(&eq)` — entity equation
   - `g->GenerateEquations(&eq)` — group equation (the important one for us)

2. `SolveBySubstitution()` — fast pre-pass for simple `a = b` equalities

3. `NewtonSolve()` — Newton-Raphson iteration: adjusts param values until
   all equations converge to zero within `CONVERGE_TOLERANCE = LENGTH_EPS/100`

4. Results written back to `SK.param` table

**Chamfer/fillet parameters do NOT participate in the solver** because:
- They are scalar distances/radii, not geometric degrees of freedom
- They have no geometric constraint that "locks" them to other params
- The extrusion depth similarly has no magnitude constraint
- The helix pitch `valB` is a stored value, not a solver param

---

## 5. Two Storage Patterns for Parameters

### Pattern A: Solver Parameters (h.param(i))
Used when: the value is a geometric quantity that the solver might adjust
Examples: extrusion vector (x,y,z), rotation axis (x,y,z), rotation angle

```cpp
// In Generate():
AddParam(param, h.param(0), initial_value);
// In GenerateEquations(): optionally constrain
AddEq(l, some_expr_involving(h.param(0)), eq_index);
// In GenerateShellAndMesh(): read back
double val = SK.GetParam(h.param(0))->val;
```

### Pattern B: Stored Scalars (valA/valB/valC)
Used when: the value is user-specified and stored directly in the Group struct
Examples: helix pitch `valB`, repeat count `valA`, scale factor `scale`

```cpp
// In menuGroup() or textscreens editing:
g->valA = user_input_value;
// In GenerateShellAndMesh(): read directly
double distance = valA;  // (in model units)
```

**For chamfer/fillet MVP: use Pattern B** — `valA` stores the chamfer offset distance or
fillet radius. This matches how helix pitch is handled.

---

## 6. Decision: How Chamfer/Fillet Parameters Are Stored

### Option 1: Pure stored values (valA only)
```cpp
case Type::CHAMFER:
    // Generate() — no params needed for MVP
    // GenerateShellAndMesh():
    double d = valA;  // read chamfer distance from stored value
    thisShell.MakeFromChamferOf(..., d);
```

**Pros**: Simplest, consistent with helix pitch `valB`  
**Cons**: Cannot be constrained by solver (e.g., "chamfer distance = 5mm")

### Option 2: One solver param (h.param(0) = distance/radius)
```cpp
case Type::CHAMFER:
    // Generate():
    AddParam(param, h.param(0), valA);  // initial value from stored valA
    // GenerateShellAndMesh():
    double d = SK.GetParam(h.param(0))->val;
    // GenerateEquations(): nothing to constrain (parameter is free)
```

**Pros**: Distance becomes a "solver variable" that could eventually be constrained
by user-added dimension constraints (e.g., a future `CHAMFER_DISTANCE` constraint type)  
**Cons**: One free variable, solver must treat it as free DOF

**Recommendation**: **Option 2** for correct architecture. The AGENT.md notes say
"CHAMFER/FILLET group's Generate() only needs AddParam(h.param(0), valA)" — this is
consistent with Option 2. The helix angle `h.param(3)` is also free (no angle constraint),
analogous to chamfer distance.

For **equal-leg chamfer**: 1 param (h.param(0) = offset distance d)  
For **unequal-leg chamfer**: 2 params (h.param(0) = d1, h.param(1) = d2)  
For **fillet**: 1 param (h.param(0) = radius r)

---

## 7. GenerateEquations() for CHAMFER/FILLET

For the MVP, `GenerateEquations()` for CHAMFER/FILLET adds **no equations**:
```cpp
void Group::GenerateEquations(IdList<Equation, hEquation> *l) {
    // ... existing cases ...
    // CHAMFER and FILLET: no equations needed
    // The distance/radius parameter h.param(0) is free
}
```

This is valid — the EXTRUDE group also adds no equations when using FREE_IN_3D workplane.
The free param simply remains at `valA` (the user-set value) after every solve.

---

## 8. The ValA/ValB/ValC Field Semantics for CHAMFER/FILLET

Looking at existing groups:
- `valA` = primary scalar parameter (repeat count for TRANSLATE/ROTATE, pitch for HELIX)
- `valB` = secondary scalar (helix pitch fixed value, `0` = free)
- `valC` = unused in current groups

For chamfer/fillet:
```
Group::Type::CHAMFER:
    valA = chamfer distance d (in model units, same as SK.MmPerUnit())
    valB = chamfer distance d2 (for unequal-leg; 0 = equal-leg)
    valC = unused

Group::Type::FILLET:
    valA = fillet radius r (in model units)
    valB = unused (could be used for variable radius later)
    valC = unused
```

The pattern for displaying/editing valA in the text panel:
```cpp
// textscreens.cpp pattern for showing a distance:
Printf(false, "  %Ba %# %Fl%Ll%f%D[change]%E",
    g->valA / SS.MmPerUnit(),    // display in user units
    &TextWindow::ScreenChangeChamferDistance, g->h.v);
```

And for editing:
```cpp
void TextWindow::ScreenChangeChamferDistance(int link, uint32_t v) {
    Group *g = SK.GetGroup(SS.TW.shown.group);
    SS.TW.ShowEditControl(3, ssprintf("%.8f", g->valA / SS.MmPerUnit()));
    SS.TW.edit.meaning = Edit::CHAMFER_OFFSET;  // = 803
    SS.TW.edit.group.v = v;
}
// In EditControlDone():
case Edit::CHAMFER_OFFSET:
    if(Expr *e = Expr::From(s, true)) {
        double ev = e->Eval();
        Group *g = SK.GetGroup(edit.group);
        g->valA = ev * SS.MmPerUnit();  // store in model units
        SS.MarkGroupDirty(g->h);
    }
    break;
```

---

## 9. Target Selection: How the Source Group Is Referenced

A chamfer/fillet group operates on a **preceding mesh group** (the solid being chamfered).
This is referenced via `opA` (= source group handle), same as EXTRUDE uses `opA` for the sketch group.

```cpp
// MenuGroup() creates the group:
g.opA = SS.GW.activeGroup;  // the solid group being chamfered
g.type = Type::CHAMFER;
g.valA = DEFAULT_CHAMFER_DISTANCE;  // e.g., 2.0mm
```

The "target edge" reference is the trickier part. Options:
1. **Chamfer all convex edges** (simplest, no per-edge selection)
2. **chamfer selected face-pairs** (store face handles in predef.entityB/entityC)
3. **Store a list of edge-curve handles** (needs new Group field or entity list)

For MVP, option 1 (chamfer all convex edges) avoids edge selection entirely.

---

## 10. The LINKED Group as a Parallel

The LINKED group is the best analogy for CHAMFER/FILLET:
- Takes an existing shell (`impShell`) and transforms it
- Does NOT generate any sketch entities
- Has NO solver equations (quaternion normalization is the only equation)
- Uses `h.param(0..6)` for position+rotation (equivalent to our distance/radius)
- Reads params back in GenerateShellAndMesh()

CHAMFER/FILLET similarly:
- Takes an existing shell (`runningShell` of `opA` group) and modifies it
- Does NOT generate any sketch entities (Generate() is near-empty)
- Has NO solver equations (distance/radius is free)
- Uses `h.param(0)` for the distance/radius

---

## 11. Summary: Parametric Integration Plan

| Aspect | CHAMFER | FILLET |
|--------|---------|--------|
| Stored field | `valA=d` | `valA=r` |
| Solver param | `h.param(0)=d` (free) | `h.param(0)=r` (free) |
| Equations | none | none |
| Group ref | `opA` = source mesh group | `opA` = source mesh group |
| Edge selection | all convex edges (MVP) | all convex edges (MVP) |
| Generate() | AddParam(h.param(0), valA) | AddParam(h.param(0), valA) |
| GenerateShellAndMesh() | MakeFromChamferOf(..., val) | MakeFromFilletOf(..., val) |
| GenerateEquations() | no equations | no equations |
| IsMeshGroup() | true | true |
| meshCombine | DIFFERENCE | DIFFERENCE |
| TextWindow display | "offset: X mm [change]" | "radius: X mm [change]" |
| Edit::Meaning | CHAMFER_OFFSET = 803 | FILLET_RADIUS = 804 |
| Serialization | `{ "valA", &g.valA }` | `{ "valA", &g.valA }` |

### Integration with the Constraint System

- CHAMFER/FILLET groups do NOT add new constraint equations
- The distance/radius `h.param(0)` is **always free** (the solver leaves it at its current value)
- The user changes it only via the text panel edit box
- Future: a `Constraint::Type::CHAMFER_DISTANCE` could pin `h.param(0)` to a specific value
  using `AddEq(l, Expr::From(h.param(0))->Minus(Expr::From(target_val)), 0)`
  but this is NOT needed for MVP

---

## 12. Code Location Reference

| File | Line | Relevance |
|------|------|-----------|
| `src/param.h` | 14-34 | `hParam` and `Param` struct |
| `src/sketch.h` | 940-944 | `hGroup::param(i)` — param handle encoding |
| `src/group.cpp` | 39-45 | `Group::AddParam()` implementation |
| `src/group.cpp` | 500-502 | EXTRUDE: AddParam for translation vector |
| `src/group.cpp` | 634-643 | HELIX: AddParam for 8 params including translation |
| `src/group.cpp` | 803-835 | `Group::GenerateEquations()` — constraint generation |
| `src/group.cpp` | 829-831 | HELIX pitch constraint (valB pattern) |
| `src/groupmesh.cpp` | 250 | EXTRUDE reads translation from solver param |
| `src/groupmesh.cpp` | 331,357 | HELIX reads angle and dist from solver params |
| `src/groupmesh.cpp` | 545-558 | `IsMeshGroup()` — must extend for CHAMFER/FILLET |
| `src/textscreens.cpp` | 343-357 | `ScreenChangeHelixPitch()` — pattern for CHAMFER edit |
| `src/textscreens.cpp` | 893-901 | `case Edit::HELIX_PITCH` — pattern for CHAMFER apply |
| `src/ui.h` | 371-373 | `Edit::Meaning` enum — add CHAMFER_OFFSET=803 |
| `src/system.cpp` | 1-end | Solver: Newton-Raphson, not involved for CHAMFER/FILLET |

---

## Key Conclusions

1. **No solver involvement needed for MVP**: chamfer distance and fillet radius are stored
   values that the solver treats as free variables. The user sets them explicitly.

2. **`valA` is the natural storage field**: same pattern as EXTRUDE depth (encoded in the
   solver-free translation magnitude), helix pitch (stored in `valB`).

3. **One solver param `h.param(0)` is architecturally correct**: allows future constraint-
   driven control without redesign. The EXTRUDE, HELIX, REVOLVE all use solver params for
   their key geometric quantities.

4. **No new Constraint types needed for MVP**: `GenerateEquations()` adds nothing for
   CHAMFER/FILLET. The param is free.

5. **The text panel edit box is the UI for changing the value**: same as helix pitch.
   `ScreenChangeChamferDistance()` → `Edit::CHAMFER_OFFSET=803` → `g->valA = ev * MmPerUnit()`

6. **Total new lines for parametric integration**: ~40 lines across group.cpp,
   groupmesh.cpp, textscreens.cpp, and ui.h.
