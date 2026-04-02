# Task 5: Existing Fillet-Related Code in SolveSpace

## Summary

SolveSpace has **no explicit chamfer or fillet feature** as of the current codebase.
A search of all `.cpp` and `.h` files in `src/` for the strings `fillet`, `chamfer`,
`FILLET`, or `CHAMFER` returns zero results. However, there is a closely related
**"Tangent Arc at Point"** feature (`MakeTangentArc()`), and extensive infrastructure
for arcs and tangency constraints that would underpin any 2D fillet implementation.

---

## 1. No Chamfer/Fillet Code Found

```
grep -rn "fillet|chamfer|Fillet|Chamfer" src/  → 0 results
```

This confirms the feature is entirely absent and must be built from scratch.

---

## 2. Closest Existing Feature: `MakeTangentArc()` — src/modify.cpp:252

This is SolveSpace's "Sketch → Tangent Arc at Point" operation. It is the closest analog
to a **2D sketch fillet**, and understanding it in detail is critical.

### What It Does

`GraphicsWindow::MakeTangentArc()` (src/modify.cpp:252) implements a non-parametric
operation that:

1. The user selects a **point where two line segments meet** (or two arcs, or line+arc)
2. SolveSpace numerically finds an arc that is tangent to both curves at a specified radius
3. Creates an `ARC_OF_CIRCLE` request positioned between the two original curves
4. Either modifies or marks-construction the two original curves
5. Applies `ARC_LINE_TANGENT` or `CURVE_CURVE_TANGENT` constraints automatically

### Key Code Path (src/modify.cpp)

```cpp
// src/modify.cpp:252
void GraphicsWindow::MakeTangentArc() {
    // Must be in workplane — 2D only
    if(!LockedInWorkplane()) {
        Error("Must be sketching in workplane to create tangent arc.");
        return;
    }
    
    // Find the vertex point to be rounded
    Vector pshared = SK.GetEntity(gs.point[0])->PointGetNum();
    
    // Find two non-construction line segments or arcs ending at pshared
    // (supports LINE_SEGMENT and ARC_OF_CIRCLE only)
    for(auto &r : SK.request) {
        if(r.type != Request::Type::LINE_SEGMENT 
            && r.type != Request::Type::ARC_OF_CIRCLE) continue;
        // ... find ent[0] and ent[1]
    }
    
    // Build ParametricCurve objects to represent each curve numerically
    ParametricCurve pc[0], pc[1];
    pc[0].MakeFromEntity(ent[0]->h, pointf[0]);
    pc[1].MakeFromEntity(ent[1]->h, pointf[1]);
    
    // Newton iterations: find t[0], t[1] on the two curves where the arc fits
    for(i = 0; i < 1020; i++) {
        // ... 1000-step Newton iteration to converge on arc center and endpoints
        // Computes: pinter (intersection of tangent lines), r (radius), el (distance)
        double el = r / tan(theta/2);   // distance along line from corner to arc endpoint
        t[0] += ...;  // converge
        t[1] += ...;
    }
    
    // Create the new ARC_OF_CIRCLE request
    hRequest harc = AddRequest(Request::Type::ARC_OF_CIRCLE);
    SK.GetEntity(earc->point[0])->PointForceTo(center);      // arc center
    SK.GetEntity(earc->point[a])->PointForceTo(pc[0].PointAt(t[0])); // start
    SK.GetEntity(earc->point[b])->PointForceTo(pc[1].PointAt(t[1])); // end
    
    // Trim/modify the original curves and connect them with tangency constraints
    pc[0].CreateRequestTrimmedTo(t[0], SS.tangentArcModify, hent[0], hearc, ...);
    pc[1].CreateRequestTrimmedTo(t[1], SS.tangentArcModify, hent[1], hearc, ...);
}
```

### Critical Observation: Non-Parametric

`MakeTangentArc()` is **non-parametric** — the result is just an arc entity with tangency
constraints applied. The radius is fixed at creation time (either automatic or
`SS.tangentArcRadius`). There is no "FILLET group" — the arc just lives in the active sketch
group like any other entity.

### Parametric State

Three global settings (src/solvespace.h:552-556):
```cpp
double tangentArcRadius;   // manual radius value (default 10.0mm, see solvespace.cpp:23)
bool tangentArcManual;     // if false, auto-compute radius to fit
bool tangentArcModify;     // if true, modify originals; if false, mark them construction
```

### UI Integration (src/textscreens.cpp:786-820)

The TextWindow screen `Screen::TANGENT_ARC` shows the "TANGENT ARC PARAMETERS" panel with:
- Radius field (editable if manual mode)
- "choose radius automatically" toggle
- "modify original entities" toggle

Triggered from `graphicswin.cpp:1339-1348`:
```cpp
case Command::TANGENT_ARC:
    if(gs.points == 1 && gs.n == 1) {
        SS.GW.MakeTangentArc();
    } else {
        SS.TW.GoToScreen(TextWindow::Screen::TANGENT_ARC);
    }
```

---

## 3. ARC_OF_CIRCLE Entity Structure

### Points Layout (src/request.cpp:33)
```
{ Request::Type::ARC_OF_CIRCLE, Entity::Type::ARC_OF_CIRCLE, 3, false, true, false }
```
An arc has exactly **3 points** and a **normal** (for workplane orientation):
- `point[0]` = center
- `point[1]` = start endpoint (thetaa)
- `point[2]` = end endpoint (thetab)

### Geometry Functions (src/entity.cpp:126-172)

```cpp
// Radius computation — uses distance from center to start endpoint
Expr *EntityBase::CircleGetRadiusExpr() const {
    if(type == Type::ARC_OF_CIRCLE) {
        return Constraint::Distance(workplane, point[0], point[1]);
    }
}

// Arc angles — uses the workplane normal's u,v axes
void EntityBase::ArcGetAngles(double *thetaa, double *thetab, double *dtheta) const {
    Quaternion q = Normal()->NormalGetNum();
    Vector u = q.RotationU(), v = q.RotationV();
    Vector c  = SK.GetEntity(point[0])->PointGetNum();  // center
    Vector pa = SK.GetEntity(point[1])->PointGetNum();  // start
    Vector pb = SK.GetEntity(point[2])->PointGetNum();  // end
    // Project into u,v plane and compute atan2
    *thetaa = atan2(pa2.y, pa2.x);
    *thetab = atan2(pb2.y, pb2.x);
    *dtheta = *thetab - *thetaa;
    while(*dtheta < 1e-6) *dtheta += 2*PI;  // wrap to positive
    while(*dtheta > (2*PI)) *dtheta -= 2*PI;
}
```

### Radius Constraint (src/entity.cpp:946)
The `ARC_OF_CIRCLE` entity's `GenerateEquations()` automatically adds an equation:
```cpp
case Type::ARC_OF_CIRCLE: {
    // Ensures both endpoints are equidistant from center
    Expr *ra = Constraint::Distance(workplane, point[0], point[1]);
    Expr *rb = Constraint::Distance(workplane, point[0], point[2]);
    AddEq(l, ra->Minus(rb), 0);  // ra = rb (equal radius)
}
```
This is the constraint that makes it a true arc (not just 3 unrelated points).

### Bezier Curve Generation (src/entity.cpp ~730-800)
This is crucial for export and rendering:
```cpp
case Type::CIRCLE:
case Type::ARC_OF_CIRCLE: {
    // Split into 1-4 rational quadratic Bezier segments based on arc angle
    int n = (dtheta > 3π/2) ? 4 : (dtheta > π) ? 3 : (dtheta > π/2) ? 2 : 1;
    dtheta /= n;
    
    for(i = 0; i < n; i++) {
        // Start and end points on circle
        Vector p0 = center + u*r*cos(thetaa) + v*r*sin(thetaa);
        Vector p2 = center + u*r*cos(thetaa+dtheta) + v*r*sin(thetaa+dtheta);
        // Control point at intersection of tangent lines
        Vector p1 = intersect(p0+t0, p2+t2);
        
        SBezier sb = SBezier::From(p0, p1, p2);  // degree-2 rational Bezier
        sb.weight[1] = cos(dtheta/2);  // exact NURBS weight for circle
        sbl->l.Add(&sb);
    }
}
```
The weight `cos(dtheta/2)` makes this an **exact rational NURBS representation** of the arc
(not an approximation). This is the same formula used for cylindrical NURBS surfaces.

---

## 4. Tangency Constraints

### ARC_LINE_TANGENT (src/constrainteq.cpp:938-948)

```cpp
case Type::ARC_LINE_TANGENT: {
    EntityBase *arc  = SK.GetEntity(entityA);
    EntityBase *line = SK.GetEntity(entityB);
    
    ExprVector ac = SK.GetEntity(arc->point[0])->PointGetExprs();  // center
    ExprVector ap = SK.GetEntity(arc->point[other ? 2 : 1])->PointGetExprs(); // endpoint
    ExprVector ld = line->VectorGetExprs();
    
    // Tangency: line is perpendicular to the radius at the endpoint
    // ld · (center - endpoint) = 0
    AddEq(l, ld.Dot(ac.Minus(ap)), 0);
}
```

The `other` field selects which arc endpoint: `other=false` → `point[1]` (start),
`other=true` → `point[2]` (end).

The mathematics: for a circle, the tangent at any point is perpendicular to the radius
vector. This constraint enforces `line_direction · (center - endpoint) = 0`.

### CURVE_CURVE_TANGENT (src/constrainteq.cpp:977-1005)

For two curves (arcs or cubics) to be tangent:
- For an arc: use `endpoint.Minus(center)` as the "direction" (normal to tangent)
- For a cubic: use `CubicGetStartTangentExprs()` or `CubicGetFinishTangentExprs()`

If both are arcs, the two radius vectors must be parallel (not perpendicular, because
center-to-endpoint is already perpendicular to tangent). If one is arc and one is cubic,
it mixes the conventions appropriately.

### Constraint Creation in constraint.cpp:

In `MenuConstrain()` → `Command::PARALLEL` case (src/constraint.cpp), the user selects
"Parallel / Tangent" which handles:
- Line + Arc → `ARC_LINE_TANGENT` 
- Cubic + Line → `CUBIC_LINE_TANGENT`
- Arc + Arc, Arc + Cubic, Cubic + Cubic → `CURVE_CURVE_TANGENT`

The user must select the appropriate **endpoint** of the arc (via `ConstrainArcLineTangent()`
which checks which endpoint of the arc is coincident with the line's endpoint).

---

## 5. ParametricCurve Helper Class

Defined in `src/ui.h:730` (nested in `GraphicsWindow`):
```cpp
struct ParametricCurve {
    bool isLine;
    Vector p0, p1;    // for line: endpoints
    double r;         // for arc: radius
    double theta0, theta1, dtheta; // for arc: angles
    Vector u, v;      // arc plane basis vectors
    
    void MakeFromEntity(hEntity he, bool reverse);
    double LengthForAuto();     // length constraint for auto-radius
    Vector PointAt(double t);   // t ∈ [0,1]
    Vector TangentAt(double t);
    void CreateRequestTrimmedTo(double t, bool reuseOrig, hEntity orig, 
                                 hEntity arc, bool arcFinish, bool pointf);
    void ConstrainPointIfCoincident(hEntity hpt);
};
```

This class is used internally by `MakeTangentArc()` and would be **directly reusable** for
implementing a 2D fillet that operates on existing corner points.

---

## 6. Key Differences: MakeTangentArc vs. True Parametric 2D Fillet

| Aspect | MakeTangentArc (existing) | True Parametric 2D Fillet |
|--------|--------------------------|--------------------------|
| Parametric? | **No** — radius fixed at creation | **Yes** — radius is a solver param |
| History | Creates entities in active group | Could create sub-requests |
| Modifies originals | Yes (optionally) | Would need to trim originals |
| Works on arcs | Yes | Yes |
| Radius | stored in `SS.tangentArcRadius` | stored in `Group::valA` |
| Reverts? | Via undo | Delete FILLET group |
| Multiple fillets | One at a time | All corners at once |

---

## 7. Request and Entity Mechanism for Arcs

From `src/request.cpp`, the `EntReqMap[]` table shows:
```cpp
{ Request::Type::ARC_OF_CIRCLE,  Entity::Type::ARC_OF_CIRCLE,  
  pts=3, useExtraPoints=false, hasNormal=true, hasDistance=false }
```

The `Request::Generate()` function creates:
- 1 entity of type `ARC_OF_CIRCLE` at `h.entity(0)`
- 3 point entities at `h.entity(1)`, `h.entity(2)`, `h.entity(3)`
- 1 normal entity at `h.entity(32)` (the workplane orientation)

For a FILLET group (2D approach), a new parametric 2D fillet would create ARC_OF_CIRCLE
requests directly (similar to MakeTangentArc but parametric), while a 3D FILLET group
would instead generate NURBS surfaces via `SShell::MakeFromFilletOf()`.

---

## 8. Constraint Infrastructure for 2D Fillets

The constraint types that would support a 2D parametric fillet:

1. **`ARC_LINE_TANGENT`** — arc tangent to a line at shared endpoint (already exists)
2. **`CURVE_CURVE_TANGENT`** — two curves tangent at shared endpoint (already exists)
3. **`POINTS_COINCIDENT`** — arc endpoint on line endpoint (already exists)
4. **`EQUAL_RADIUS`** — two arcs same radius (already exists, useful for multi-corner fillet)

These existing constraints are **entirely sufficient** to specify a 2D fillet geometrically.
No new constraint types are needed. The work is in automating their creation (like
`MakeTangentArc()` does, but parametrically with `Group::valA` as the radius parameter).

---

## 9. What a Parametric 2D Fillet Would Look Like

**Input:** User selects a DRAWING_WORKPLANE group, sets fillet radius r = valA

**Group::Generate()** would:
1. Iterate over `SK.request` in the source sketch group
2. Find all corners (shared endpoints of two non-construction segments)
3. For each corner, compute the arc center, start/end points using the same
   Newton iteration as `MakeTangentArc()`, but using `valA` as the fixed radius
4. Create a `DATUM_POINT` for the arc center (h.param(0) for the radius)
5. Create `ARC_OF_CIRCLE` requests for each corner
6. Apply `ARC_LINE_TANGENT` or `CURVE_CURVE_TANGENT` constraints
7. Trim the original line segments to the arc endpoints

**Parameters needed:**
- `valA` = fillet radius (one parameter for all corners in the group)
- The source group handle in `opA`

**Problem:** The trimming approach — modifying existing entities — conflicts with SolveSpace's
group model where each group generates its own entities. The FILLET group would need to either:
(a) Regenerate all the base entities with trimming applied (complex), or  
(b) Act as a post-processing operation that creates new entities replacing old ones

The 3D approach (modifying the SShell directly) is cleaner because there's no trimming issue —
the shell boolean directly produces the correct topology.

---

## 10. Existing Tangent Arc Settings in File Format

From `src/solvespace.cpp:23`:
```cpp
SS.tangentArcRadius = 10.0;  // default
SS.tangentArcManual = false;
SS.tangentArcModify = false;
```

These are **not persisted** in the file — they're session-level settings. A new parametric
fillet would store its radius in `Group::valA` which IS serialized via the `SAVED[]` table
in `src/file.cpp`.

---

## 11. Menu Integration Point

From `src/graphicswin.cpp:144`:
```cpp
{ 1, N_("Ta&ngent Arc at Point"),  Command::TANGENT_ARC, S|'a', KN, mReq },
```

The `Sketch` menu currently has `TANGENT_ARC` as a request-level operation under `mReq`
(the request menu handler). A true parametric FILLET/CHAMFER would instead go under `mGrp`
(the group menu handler) in the `Sketch` menu, after the EXTRUDE/REVOLVE/HELIX operations.

---

## 12. Key Findings Summary

1. **No existing fillet/chamfer** in SolveSpace — must be built from scratch
2. **`MakeTangentArc()`** is the best analog — a non-parametric 2D corner rounding operation
3. The `ParametricCurve` helper class is reusable for 2D fillet geometry computation
4. **`ARC_LINE_TANGENT`** and **`CURVE_CURVE_TANGENT`** constraints fully define 2D arc tangency
5. `ARC_OF_CIRCLE` entities have 3 points (center, start, end) + a normal
6. Arc bezier generation uses `weight[1] = cos(dtheta/2)` for exact NURBS representation
7. The same formula applies to 3D fillet cylindrical NURBS surfaces
8. A 2D fillet is significantly simpler than 3D — `MakeTangentArc()` already does 95% of it
9. The main challenge for parametric 2D fillet is the **trimming problem**: adjusting existing
   line segments to end at the arc tangency points
10. 3D fillet/chamfer avoids the trimming problem by operating directly on `SShell` topology

## Files Examined
- `src/constraint.cpp` — Constraint menu handling, `ConstrainArcLineTangent()`
- `src/constrainteq.cpp` — `ARC_LINE_TANGENT`, `CURVE_CURVE_TANGENT` equations
- `src/sketch.h` — Entity types, Request types, Constraint types
- `src/modify.cpp` — `MakeTangentArc()` full implementation
- `src/entity.cpp` — `ArcGetAngles()`, `CircleGetRadiusExpr()`, `GenerateBezierCurves()`
- `src/request.cpp` — `EntReqMap[]`, `Request::Generate()`
- `src/graphicswin.cpp` — menu definitions, `TANGENT_ARC` command
- `src/textscreens.cpp` — `ShowTangentArc()` UI panel
