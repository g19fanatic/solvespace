# Task 35: Solver vs. Stored Params — How Should Chamfer/Fillet Parameters Be Handled?

## Executive Summary

**The chamfer offset and fillet radius should be stored in `Group::valA` (a plain `double`), NOT as solver parameters accessed via `h.param(0)`.** This conclusion is based on deep analysis of how existing 3D group types (EXTRUDE, LATHE, REVOLVE, HELIX) use the solver vs. stored values.

---

## 1. How the SolveSpace Solver Pipeline Works

### The Two-Phase Execution (from `generate.cpp`)

The `GenerateAll()` function in `generate.cpp` runs in two modes per group:

1. **`genForBBox` phase** (first pass, to calculate chord tolerance):
   - Calls `SolveGroupAndReport(hg, andFindFree)` — runs the constraint solver
   - Then calls `g->GenerateLoops()` — assembles bezier loops from sketch entities
   
2. **Main phase** (second pass, actual geometry generation):
   - Calls `g->GenerateShellAndMesh()` — builds B-rep shell/mesh
   - Sets `g->clean = true`

Key insight: **`SolveGroupAndReport` is called ONLY during the `genForBBox` phase**, not directly before `GenerateShellAndMesh()`. The solver results (solved parameters in `SK.param`) are used by `GenerateShellAndMesh()` via `SK.GetParam(h.param(N))->val`.

### The `SolveGroup` Flow (`generate.cpp`)

```
SolveGroup(hg) {
    WriteEqSystemForGroup(hg):  // generates params from g->Generate() into sys.param
        g->Generate(&sys.entity, &sys.param)  // calls Group::Generate()
    sys.Solve(g, ...)             // Newton-Raphson solver
    // solved values written back to SK.param
}
```

`Group::Generate()` is the function that calls `AddParam(param, h.param(N), initial_value)`. Each call to `AddParam` registers a new solver parameter with an initial value guess.

---

## 2. When Groups Need Solver Parameters

### EXTRUDE (uses solver params — needs constraint interaction)

```cpp
// group.cpp:499 — Group::Generate() for EXTRUDE
case Type::EXTRUDE: {
    AddParam(param, h.param(0), gn.x);  // extrusion vector x
    AddParam(param, h.param(1), gn.y);  // extrusion vector y
    AddParam(param, h.param(2), gn.z);  // extrusion vector z
    ...
}
```

```cpp
// group.cpp:834 — GenerateEquations() for EXTRUDE
} else if((type == Type::EXTRUDE) && ...) {
    if(predef.entityB != Entity::FREE_IN_3D) {
        Entity *w = SK.GetEntity(predef.entityB);
        ExprVector u = w->Normal()->NormalExprsU();
        ExprVector v = w->Normal()->NormalExprsV();
        ExprVector extruden = {
            Expr::From(h.param(0)),
            Expr::From(h.param(1)),
            Expr::From(h.param(2)) };
        AddEq(l, u.Dot(extruden), 0);  // constrain direction to workplane normal
        AddEq(l, v.Dot(extruden), 1);
    }
}
```

**Why EXTRUDE uses solver params:**
- The extrusion vector `h.param(0,1,2)` is a free 3D vector
- It's constrained by 2 equations: "must be perpendicular to workplane U and V axes"
- The solver satisfies these 2 constraints, leaving 1 DOF (the extrusion depth/magnitude)
- The user drags to set depth — `pending.point` drag updates the vector component
- This is NOT stored as a single scalar — the full (x,y,z) extrusion vector is stored in 3 params

**`GroupSelection()` for EXTRUDE doesn't check `gs.params` — it uses `h.param(0/1/2)` for the extrusion direction vector.**

### REVOLVE (uses solver params — angle is free)

```cpp
// group.cpp:581
case Type::REVOLVE: {
    AddParam(param, h.param(0), axis_pos.x);  // rotation center
    AddParam(param, h.param(1), axis_pos.y);
    AddParam(param, h.param(2), axis_pos.z);
    AddParam(param, h.param(3), 30 * PI / 180);  // rotation ANGLE — free param!
    AddParam(param, h.param(4), axis_dir.x);  // axis direction
    AddParam(param, h.param(5), axis_dir.y);
    AddParam(param, h.param(6), axis_dir.z);
```

```cpp
// group.cpp:803 — GenerateEquations() for REVOLVE
} else if(type == Type::ROTATE || type == Type::REVOLVE || type == Type::HELIX) {
    // Constrain center and axis to specific values (already known numerically)
    AddEq(l, (orig.x)->Minus(EP(0)), 0);
    AddEq(l, (orig.y)->Minus(EP(1)), 1);
    AddEq(l, (orig.z)->Minus(EP(2)), 2);
    // param 3 is the angle, which is free
    AddEq(l, (EC(axis.x))->Minus(EP(4)), 3);
    ...
```

**Why REVOLVE uses solver params:**
- Params 0-2 (center) and 4-6 (axis) are locked to specific entity values by equations
- Param 3 (angle) is FREE — the solver leaves it alone; user controls it via textbox/constraint
- `SK.GetParam(h.param(3))->val` gives the current angle in `GenerateShellAndMesh()`
- The angle is a genuinely draggable/constrainable parameter

### HELIX (extends REVOLVE, also has pitch)

```cpp
AddParam(param, h.param(7), 20);  // helical distance
```

```cpp
// group.cpp:829 — GenerateEquations() for HELIX
if(valB != 0.0) {
    AddEq(l, Expr::From(h.param(7))->Times(Expr::From(PI))->
    Minus(Expr::From(h.param(3))->Times(Expr::From(valB))), 6);
}
```

`valB` here is the pitch multiplier — a **stored value** (not a solver param). `h.param(7)` (translation distance) IS a solver param, but it's constrained by an equation relating it to `valB * angle`.

---

## 3. When Groups Do NOT Need Solver Parameters

### LATHE (uses stored entities, no free params of its own)

```cpp
// group.cpp — Group::Generate() for LATHE
case Type::LATHE: {
    // NO AddParam calls!
    Vector pt   = SK.GetEntity(predef.origin)->PointGetNum();
    Vector axis = SK.GetEntity(predef.entityB)->VectorGetNum();
    // Uses existing entity values directly
    for(i = 0; i < entity->n; i++) {
        ...
        CopyEntity(entity, SK.GetEntity(he), 0, REMAP_LATHE_START,
            NO_PARAM, NO_PARAM, NO_PARAM, ...);  // no params!
```

LATHE has **zero solver params**. It uses entity handles (`predef.origin`, `predef.entityB`) to look up axis position and direction. These come from pre-existing entities in earlier groups.

LATHE's geometry is 360° revolution — no free angular parameter. The degree of revolution is implicit (full circle). There's nothing for the solver to solve.

### The Key Question: Does LATHE Have a `valA`?

Looking at `groupmesh.cpp` where LATHE geometry is built:

```cpp
} else if(type == Type::LATHE && haveSrc) {
    Group *src = SK.GetGroup(opA);
    Vector pt   = SK.GetEntity(predef.origin)->PointGetNum();
    Vector axis = SK.GetEntity(predef.entityB)->VectorGetNum();
    axis = axis.WithMagnitude(1);
    ...
    thisShell.MakeFromRevolutionOf(sbls, pt, axis, color, this);
```

No `valA` used, no `h.param(0)` used. Everything comes from entity lookups.

---

## 4. What This Means for Chamfer/Fillet

### The Critical Insight

The chamfer offset `d` (or fillet radius `r`) is a **single scalar value**. It:
1. Does NOT need to be constrained to anything else
2. Is NOT needed by any equation in `GenerateEquations()`
3. Does NOT need to interact with the constraint system
4. Cannot be "dragged" — it's set via the TextWindow edit box
5. Is NOT linked to any geometric entity in the sketch

This is exactly like LATHE's behavior — there's nothing for the solver to solve.

### Confirmed: Use `Group::valA` (Stored Double)

The correct pattern for chamfer/fillet is:

```cpp
// group.cpp — Group::Generate() for CHAMFER
case Type::CHAMFER:
    AddParam(param, h.param(0), valA);  // initial guess for solver param
    return;
```

Wait — even this is questionable. Let me verify further.

### Why Even ONE Solver Param?

Looking at `generate.cpp` carefully:

```cpp
// SolveGroup is called in genForBBox phase only:
if(genForBBox) {
    SolveGroupAndReport(hg, andFindFree);  // runs solver
    g->GenerateLoops();
} else {
    g->GenerateShellAndMesh();  // uses solved params
    g->clean = true;
}
```

For `genForBBox`, `SolveGroupAndReport` is called to solve the sketch. For a CHAMFER/FILLET group, there are no sketch entities (no requests in the group), so no equations from requests/constraints. The only equations would come from `Group::GenerateEquations()`.

**If CHAMFER's `GenerateEquations()` generates NO equations, and the only param added by `Generate()` is `h.param(0)` (the distance), then:**
- The solver sees 1 unknown, 0 equations
- Result: 1 DOF (fully free)
- `g->solved.how` = `OKAY` (system is consistent with 0 equations)
- `h.param(0)` retains its initial value `valA`

**So adding `AddParam(param, h.param(0), valA)` and NOT adding any equations effectively means `h.param(0)` always equals `valA` after solving.** This is the pattern for REVOLVE's angle param when not explicitly constrained.

### But Wait — Is the Solver Param Even Necessary?

The REAL question: in `GenerateShellAndMesh()`, does the CHAMFER code read from `SK.GetParam(h.param(0))->val` or from `valA` directly?

Looking at how EXTRUDE does it:
```cpp
Vector translate = Vector::From(h.param(0), h.param(1), h.param(2));  // reads solver params
```

And how the TextWindow sets extrusion depth:
```cpp
// textscreens.cpp — when user types extrusion depth:
void TextWindow::ScreenChangeExtrusionDepth(int link, uint32_t v) {
    ...
    double d = SS.ExprToMm(SS.TW.editControl.str);
    ...
    // Force the extrusion vector to the desired length
    g->ExtrusionForceVectorTo(gn.WithMagnitude(d));
    // ExtrusionForceVectorTo writes to SK.GetParam(h.param(0/1/2))->val
```

This shows that **extrusion depth is SET by directly writing to the solver param `h.param(0/1/2)`** — not to `valA`. The solver then reads that value back from `SK.param`.

But there's an alternative: use `valA` as the sole ground truth, no solver param at all.

### The Cleanest Approach for Chamfer/Fillet

**Option A: Use solver param `h.param(0)`** (like EXTRUDE distance pattern)
- Pro: Consistent with how distance-like quantities work in EXTRUDE
- Pro: `h.param(0)` is automatically persisted (the file format saves Param values)
- Con: Solver sees 1 free param → solver reports 1 DOF in group; user may see "1 DOF" message

**Option B: Use `valA` only, NO solver param** (like LATHE with no free params)
- Pro: Solver sees 0 unknowns, 0 equations → 0 DOF → solver reports "fully constrained"
- Pro: Simpler
- Con: `valA` must be separately persisted (but it IS already saved via the `valA` SAVED[] entry)

### Best Practice: ONE Solver Param with Zero Equations (Option A)

Looking at REVOLVE's angle param (`h.param(3)`): it's added with `AddParam()` but NOT constrained by any equation. This makes it a "free" param with 1 DOF. The solver leaves it at whatever value was stored from `prev`. This is how the revolve angle "remembers" itself across regenerations.

For chamfer, this same pattern works:
- `Group::Generate()`: `AddParam(param, h.param(0), valA)` — registers param with initial guess
- `GenerateEquations()`: adds NO equations for CHAMFER/FILLET
- `GenerateShellAndMesh()`: reads `double dist = SK.GetParam(h.param(0))->val`
- TextWindow callback: `SK.GetParam(g->h.param(0))->val = newDist; SS.MarkGroupDirty(g->h);`
- No need to update `valA` at all — `h.param(0)` is the ground truth

Actually, looking more carefully: `valA` IS the serialized value. The solver param `h.param(0)` gets re-initialized from `valA` every time. So:

**Recommended pattern:**
1. `Generate()`: `AddParam(param, h.param(0), valA)` → registers initial value
2. Solver runs, param stays at `valA` (no equations to change it)  
3. `GenerateShellAndMesh()`: reads `double dist = SK.GetParam(h.param(0))->val`
4. TextWindow callback: sets `g->valA = newDist`, marks group dirty → next regen re-initializes `h.param(0)` to new `valA`

This means `valA` is authoritative for serialization, and `h.param(0)` is the in-memory working copy used during generation. This is consistent with how REVOLVE handles its angle.

---

## 5. What Does NOT Need the Solver at All

### Face Pair (predef.entityB, predef.entityC)

The face handles stored in `predef.entityB` and `predef.entityC` are entity handles (`hEntity`), not solver parameters. They're looked up from the SShell to find the shared edge — no solver involvement.

### Edge Geometry

The actual edge geometry (endpoints, tangent vectors) is computed at generation time by looking up the SShell surface topology. This is pure deterministic computation from source geometry — no solver needed.

---

## 6. Comparison Table: Solver vs. Stored Values

| Group Type | Free Solver Params | Stored Values | Why |
|---|---|---|---|
| EXTRUDE | h.param(0,1,2) = extrusion vector | None for depth | User drags endpoints to set depth; vector must be perpendicular to workplane |
| REVOLVE | h.param(3) = angle | None for angle | Angle can be constrained; entity axis/center locked by equations |
| HELIX | h.param(3) = angle, h.param(7) = dist | valB = pitch | Angle and dist linked by equation involving valB |
| LATHE | NONE | predef.origin, entityB | Full revolution, no free params |
| TRANSLATE/ROTATE (S&R) | h.param(0,1,2) = delta | valA = count | Count is integer, not solver param |
| **CHAMFER** | **h.param(0) = distance** | **valA = initial dist** | **Single scalar, no equations; valA = ground truth** |
| **FILLET** | **h.param(0) = radius** | **valA = initial radius** | **Single scalar, no equations; valA = ground truth** |

---

## 7. What Actually Happens in `GenerateAll()` for a CHAMFER Group

1. **`genForBBox` phase:**
   - `g->Generate(&sys.entity, &sys.param)` → registers `h.param(0)` with initial `valA`
   - `SolveGroupAndReport()` → solver sees 1 free param, 0 equations → OKAY, param unchanged
   - `g->GenerateLoops()` → no loops (no sketch entities in CHAMFER group) → empty
   
2. **Main phase:**
   - `g->GenerateShellAndMesh()`:
     - Looks up `SK.GetParam(h.param(0))->val` → gets `valA`
     - Finds source SShell from `PreviousGroup()->runningShell`
     - Finds shared edge via `predef.entityB` / `predef.entityC` face lookups
     - Calls `thisShell.MakeFromChamferOf(srcShell, this, dist)`
     - Sets `runningShell = thisShell` (bypassing GenerateForBoolean)
   - `g->clean = true`

---

## 8. DOF Reporting Consideration

When `GenerateEquations()` adds no equations but `Generate()` adds 1 param, the solver reports `1 DOF`. The text window will show "1 DOF" for the chamfer group.

**Solution options:**
1. **Use `suppressDofCalculation = true`** — tells solver not to report DOF for this group
2. **Don't add any solver param** — use `valA` directly in `GenerateShellAndMesh()` — 0 DOF
3. **Accept 1 DOF** — actually fine since the "DOF" is the chamfer distance (intentionally free)

Looking at existing code: REVOLVE also shows a free angle param. For the UI, showing 1 DOF for "chamfer has 1 degree of freedom (its offset distance)" is semantically correct.

**RECOMMENDED: Option 3** — accept 1 DOF, it's semantically meaningful.

Alternatively: use `g->suppressDofCalculation = true` in `MenuGroup()` when creating the CHAMFER/FILLET group, same as some other groups might do.

---

## 9. The Key Bottom Line for Implementation

```cpp
// src/group.cpp — Group::Generate() CHAMFER case
case Type::CHAMFER:
    // Only 1 solver param: the chamfer offset distance
    // Initial value = valA (the stored/saved distance)
    AddParam(param, h.param(0), valA);
    // No entity copies needed
    return;
```

```cpp
// src/groupmesh.cpp — GenerateShellAndMesh() CHAMFER injection
} else if(type == Type::CHAMFER) {
    // Get the chamfer distance from solved param (== valA, no equations constrain it)
    double dist = SK.GetParam(h.param(0))->val;
    
    // Get source shell from previous running group
    Group *prev = RunningMeshGroup();  // same as PreviousGroup() for CHAMFER
    SShell *src = &(prev->runningShell);
    
    // Build chamfered shell
    thisShell.MakeFromChamferOf(src, this, dist);
    
    // BYPASS GenerateForBoolean — directly set runningShell
    runningShell.MakeFromCopyOf(&thisShell);
    return;  // skip the normal GenerateForBoolean call
}
```

```cpp
// src/group.cpp — Group::GenerateEquations() — NO changes needed for CHAMFER
// The existing else-if chain falls through to default (no equations added)
// since CHAMFER has no workplane equations, no rotation equations, etc.
```

```cpp
// src/textscreens.cpp — TextWindow callback to change chamfer distance
static void ScreenChangeChamferOffset(int link, uint32_t v) {
    Group *g = SK.GetGroup(SS.TW.shown.group);
    double d = SS.ExprToMm(SS.TW.editControl.str);
    if(isnan(d) || d <= 0) {
        Error("Invalid chamfer offset.");
        return;
    }
    SS.UndoRemember();
    g->valA = d;
    // No need to update h.param(0) directly — it's re-initialized from valA every regen
    SS.MarkGroupDirty(g->h);
    SS.ScheduleShowTW();
}
```

---

## 10. Summary of Key Facts

1. **`valA` is the serialized, authoritative value for chamfer distance/fillet radius**
2. **`h.param(0)` is the in-memory solver copy, initialized from `valA` each regen via `AddParam()`**
3. **No equations are added for CHAMFER/FILLET in `GenerateEquations()`** — the param stays free
4. **`GenerateShellAndMesh()` reads `SK.GetParam(h.param(0))->val` to get the distance**
5. **TextWindow callback updates `g->valA` only; `h.param(0)` will be re-initialized next regen**
6. **The CHAMFER group gets 1 DOF — semantically correct (the distance is the free DOF)**
7. **SolveGroup() call still happens (`genForBBox` phase) but does nothing useful for CHAMFER** — the CHAMFER group has an empty sketch (no requests), so solver runs trivially
8. **IsMeshGroup() MUST return true for CHAMFER/FILLET** or `GenerateShellAndMesh()` is never called
9. **No changes needed to the constraint solver itself** (`system.cpp`) — no new equation types needed

---

## 11. Comparison with LATHE vs. EXTRUDE Pattern

| Aspect | EXTRUDE | LATHE | CHAMFER |
|---|---|---|---|
| Solver params | 3 (x,y,z vector) | 0 | 1 (distance) |
| Equations | 2 (perpendicularity) | 0 | 0 |
| DOF | 1 (length) | 0 | 1 (distance) |
| Ground truth | `h.param(0/1/2)` | `predef.entityB` | `valA` |
| User edits via | Dragging or textbox | N/A | Textbox only |

CHAMFER is most similar to a **constrained version of EXTRUDE** where the user can only edit depth via textbox, not dragging. The single solver param with no equations is the correct minimal pattern.

---

## Sources

- `src/group.cpp` (directly read): `Group::Generate()`, `Group::GenerateEquations()` for all group types
- `src/groupmesh.cpp` (directly read): `Group::GenerateShellAndMesh()`, `Group::IsMeshGroup()`
- `src/generate.cpp` (directly read): `SolveSpaceUI::GenerateAll()`, `SolveGroup()`, `WriteEqSystemForGroup()`
- `src/system.cpp` (directly read): `System::Solve()`, `System::NewtonSolve()`
- `src/param.h` (directly read): `Param` struct definition
- Prior findings: task 13 (parametric integration), task 15 (file format), task 32 (group generate trace)
