# Task 19: 2D Sketch Fillet Implementation — Algorithm and Tangent Constraints

## Summary

A 2D sketch fillet replaces a sharp corner between two line segments (or between a line and an arc) with a smooth arc that is tangent to both input entities. This is the simplest form of a fillet and serves as an excellent starting point before tackling 3D edge fillets in SolveSpace.

---

## Part 1: The Mathematical Algorithm

### Closed-Form Algorithm for Arc Tangent to Two Lines

Given two line segments meeting at a corner vertex V, to place a fillet arc of radius r:

**Step 1: Find unit tangent vectors of each line at the corner**
- Line 1: `t1 = normalize(V - A)` (pointing away from the corner, toward line start)
- Line 2: `t2 = normalize(V - B)` (pointing away from the corner, toward line start)

**Step 2: Compute the half-angle bisector and offset distance**
- The arc center lies on the bisector of the angle between the two lines
- Let `θ = angle between t1 and t2` at the corner
- Distance along each line from corner to tangent point: `el = r / tan(θ/2)`
- This is the "leg length" of the fillet

**Step 3: Compute tangent points**
- Point on Line 1: `P1 = V + t1 * el`
- Point on Line 2: `P2 = V + t2 * el`

**Step 4: Compute arc center**
- The center is at distance `r` from each tangent point, perpendicular to the line
- `C = V + bisector * r/sin(θ/2)`
- Or equivalently: `C = V + normalize(t1 + t2) * r / sin(θ/2)`

**Step 5: Create the arc**
- Arc center: `C`
- Arc endpoints: `P1` and `P2`
- Arc sweeps from `P1` to `P2` around `C`

**Step 6: Trim the original lines**
- Shorten Line 1 so it ends at `P1` (delete the segment from `P1` to `V`)
- Shorten Line 2 so it starts at `P2` (delete the segment from `V` to `P2`)

**Step 7: Apply tangency constraints**
- `ARC_LINE_TANGENT` constraint between the arc and Line 1 at `P1`
- `ARC_LINE_TANGENT` constraint between the arc and Line 2 at `P2`

### Special Cases
- **Parallel lines**: `sin(θ/2) → 0`, arc center goes to infinity — no fillet possible
- **Very short lines**: The fillet may not fit if `el > line_length`; must clamp or error
- **Arc-line fillet**: Extend the algorithm using the arc's tangent vector at the endpoint
- **Arc-arc fillet**: More complex — use Newton iteration (as SolveSpace already does in `MakeTangentArc`)

### Inscribed Circle Interpretation
The fillet arc is the **inscribed circle** of the corner angle: a circle of radius r that is tangent to both half-lines extending from the corner. The term "inscribed" comes from this relationship to the angle's geometry. The arc is a portion of that inscribed circle cut off by the tangent points.

---

## Part 2: SolveSpace's Existing 2D Fillet — `MakeTangentArc()`

### Overview

SolveSpace already implements a 2D sketch fillet via `GraphicsWindow::MakeTangentArc()` in `src/modify.cpp:252`. This is a **non-parametric** operation that runs once and then becomes fixed geometry with constraints.

### How It Works (Code Analysis)

**Input**: A single selected point at a corner where two line segments (or arcs) meet in the same group and workplane (modify.cpp:252).

**Step 1: Find the two input entities** (modify.cpp:261-296)
```cpp
// Searches SK.request for all non-construction LINE_SEGMENT or ARC_OF_CIRCLE 
// in the same group+workplane that have an endpoint at pshared
// Must find exactly 2 such entities (c == 2)
```

**Step 2: Create `ParametricCurve` helpers** (modify.cpp:312-314)
```cpp
ParametricCurve pc[2];
pc[0].MakeFromEntity(ent[0]->h, pointf[0]);
pc[1].MakeFromEntity(ent[1]->h, pointf[1]);
```
The `ParametricCurve` struct (src/ui.h:730) abstracts over line and arc, providing:
- `PointAt(t)` — position at parameter t ∈ [0,1], where t=0 is at the corner
- `TangentAt(t)` — tangent direction at parameter t
- `LengthForAuto()` — length limit for auto-radius (1/3 of line, 1/20 of arc)

**Step 3: Newton iteration to find the arc** (modify.cpp:327-377)

This is the sophisticated part. SolveSpace does NOT use the closed-form formula above for the general case (which handles arcs and more complex cases). Instead it uses numerical iteration:

```cpp
// Start: t[0] = t[1] = 0 (at the corner)
// Iterate 1000 times:
//   p0 = pc[0].PointAt(t[0])   — current position on curve 0
//   p1 = pc[1].PointAt(t[1])   — current position on curve 1
//   t0 = pc[0].TangentAt(t[0]) — tangent at that position
//   t1 = pc[1].TangentAt(t[1]) — tangent at that position
//   
//   pinter = intersection of tangent lines through p0 and p1
//   el = r / tan(theta/2)    — leg length (using angle between tangents)
//   pa0 = pinter + t0 * el   — target point on curve 0
//   pa1 = pinter + t1 * el   — target point on curve 1
//   
//   t[0] += (pa0 - p0) / |t0|  — step parameter toward target
//   t[1] += (pa1 - p1) / |t1|  — step parameter toward target
```

The iteration converges because at the final solution, `p0` and `p1` will be the tangent points, and the two tangent lines will intersect exactly at `pinter`. The radius `r` is either:
- Automatic: `r = min(200/scale, pc[0].LengthForAuto()*tan(θ/2), pc[1].LengthForAuto()*tan(θ/2))`
- Manual: `r = SS.tangentArcRadius` (set by the user via Sketch menu option)

**Step 4: Validate convergence** (modify.cpp:380-390)
- If t[0] or t[1] diverged, or are out of [0.01, 0.99], error and return

**Step 5: Compute arc center** (modify.cpp:392-402)
```cpp
// The center is offset perpendicularly from the first tangent point
// vv determines clockwise vs counterclockwise
Vector center = pc[0].PointAt(t[0]);
Vector v0inter = pinter.Minus(center);
if(vv < 0) {
    center = center.Minus(v0inter.Cross(wn).WithMagnitude(r));
} else {
    center = center.Plus(v0inter.Cross(wn).WithMagnitude(r));
}
```

**Step 6: Modify or mark construction** (modify.cpp:404-417)
Two modes (controlled by `SS.tangentArcModify`):
1. **Modify mode**: Truncate the original line endpoints to the tangent points
2. **Non-modify mode**: Mark the original lines as construction geometry

**Step 7: Create the arc request** (modify.cpp:419-428)
```cpp
hRequest harc = AddRequest(Request::Type::ARC_OF_CIRCLE);
// Sets:
//   earc->point[0] = center
//   earc->point[a] = pc[0].PointAt(t[0])  // start
//   earc->point[b] = pc[1].PointAt(t[1])  // end
```

**Step 8: Trim and constrain** (modify.cpp:440-443)
```cpp
pc[0].CreateRequestTrimmedTo(t[0], SS.tangentArcModify, 
    hent[0], hearc, /*arcFinish=*/(b==1), pointf[0]);
pc[1].CreateRequestTrimmedTo(t[1], SS.tangentArcModify, 
    hent[1], hearc, /*arcFinish=*/(a==1), pointf[1]);
```
`CreateRequestTrimmedTo()` either modifies the original entity endpoint OR creates a new request, and adds the tangency constraint:
- For line: `ARC_LINE_TANGENT` constraint → `ld.Dot(ac.Minus(ap)) = 0`
- For arc: `CURVE_CURVE_TANGENT` constraint

---

## Part 3: `ARC_LINE_TANGENT` Constraint

Defined in `src/constrainteq.cpp:938`:
```cpp
case Type::ARC_LINE_TANGENT: {
    EntityBase *arc  = SK.GetEntity(entityA);
    EntityBase *line = SK.GetEntity(entityB);
    
    ExprVector ac = SK.GetEntity(arc->point[0])->PointGetExprs();  // center
    ExprVector ap = SK.GetEntity(arc->point[other ? 2 : 1])->PointGetExprs();  // endpoint
    
    ExprVector ld = line->VectorGetExprs();  // line direction
    
    // The line is perpendicular to the radius at the tangent point
    // => line is tangent to the circle
    AddEq(l, ld.Dot(ac.Minus(ap)), 0);  // line · (center - endpoint) = 0
    return;
}
```
This is the mathematical condition for tangency: the line direction is perpendicular to the radius vector at the intersection point.

---

## Part 4: `CURVE_CURVE_TANGENT` Constraint

Defined in `src/constrainteq.cpp:977`:
```cpp
case Type::CURVE_CURVE_TANGENT: {
    // For arcs: direction = endpoint - center (the radius vector)
    // For cubics: direction = tangent vector at endpoint
    // 
    // If both are arcs, the radius vectors must be parallel (perpendicular 
    // to the common tangent) → uses PARALLEL equation
    // If arc + cubic: radius perpendicular to cubic tangent → DOT = 0
```

---

## Part 5: `ARC_OF_CIRCLE` Request — Data Model

From `src/request.cpp:33`:
```cpp
{ Request::Type::ARC_OF_CIRCLE, Entity::Type::ARC_OF_CIRCLE, 3, false, true, false }
// Type    entity-type    points  hasNormal  inWorkplane  hasDistance
```
- 3 points: `point[0]` = center, `point[1]` = start, `point[2]` = end
- Always in a workplane (no free-3D arc requests)
- Has normal (inherited from workplane)
- The constraint `ARC_OF_CIRCLE` generates equation: `|center-start| = |center-end|` (equal radius)

---

## Part 6: Global State for Tangent Arc

From `src/solvespace.h:554-556`:
```cpp
double tangentArcRadius;   // manual radius value (mm)
bool tangentArcManual;     // true = use user-specified radius
bool tangentArcModify;     // true = modify original entities; false = mark as construction
```
These are **session globals** — NOT saved to the .slvs file. They persist only while SolveSpace is running.

Key implication for a **parametric** 2D fillet group: the radius would need to be stored in `Group::valA` (which IS saved to the file), making the parametric fillet fundamentally different from the non-parametric `MakeTangentArc`.

---

## Part 7: Comparison — Non-Parametric vs Parametric 2D Fillet

| Aspect | `MakeTangentArc()` (existing) | Parametric Fillet Group (proposed) |
|--------|-------------------------------|-------------------------------------|
| When applied | Once, at user click | Regenerated every time model updates |
| Radius | `tangentArcRadius` global (not saved) | `Group::valA` (saved to file) |
| Result | Actual requests in sketch (line+arc) | Generated entities in Group |
| Parametric | No — fixed after creation | Yes — drag to change radius |
| Works on | Intersecting lines/arcs in same group | Any 2 sketch entities at corner |
| Constraints added | `ARC_LINE_TANGENT` | Implicit in generation algorithm |
| Undo | Single undo step | Undo changes group param |

---

## Part 8: Algorithm for Parametric 2D Fillet Group (Design)

A parametric 2D sketch fillet group would work differently:

1. **Input**: Two entity handles (lines/arcs forming a corner) + radius parameter `valA`
2. **Generate()**: Compute tangent points analytically (closed-form for line-line case):
   - Compute leg length `el = valA / tan(θ/2)` where θ = angle between lines
   - Compute tangent points P1, P2 on each line
   - Compute arc center C
   - Create `Entity::ARC_OF_CIRCLE` with `point[0]=C, point[1]=P1, point[2]=P2`
3. **No solver needed**: The geometry is fully determined by the input entities and radius
4. **The trimmed lines**: Either (a) add shortened line entities, or (b) use "virtual trimming" by rendering only the non-fillet part of the original lines

Option (b) is conceptually cleaner — don't modify the input sketch, just render the fillet arc overlay. This is how Onshape and Fusion 360 handle it.

---

## Part 9: 2D Sketch Chamfer Algorithm (for Comparison)

A 2D sketch chamfer replaces a corner with a line segment:
1. Compute tangent points `P1 = V + t1 * d` and `P2 = V + t2 * d` (distance d from corner)
2. Add a new `LINE_SEGMENT` from P1 to P2
3. Trim/shorten the original lines to P1 and P2
4. No tangency constraints needed (chamfer is G0, not G1)

This is simpler than a fillet because:
- No inscribed circle calculation
- Both legs are the same length (equal-distance chamfer)
- Result is a straight line, not an arc

---

## Key Findings for Implementation

1. **SolveSpace already has 2D fillet** (`MakeTangentArc`), but it's NON-parametric
2. **A parametric 2D fillet** would require a new Group type that generates an `ARC_OF_CIRCLE` entity
3. **The mathematical core** is straightforward for line-line case: compute leg length, tangent points, arc center
4. **`ParametricCurve` helper struct** in `src/ui.h:730` can be reused for the generation code
5. **Constraint equation** for tangency is simple: `ld.Dot(center - endpoint) = 0`
6. **The arc entity** is 3 points: center + 2 endpoints (request.cpp:33)
7. **Key decision**: parametric fillet group vs. non-parametric operation — parametric is harder but more useful

---

## Source References

- `src/modify.cpp:252` — `MakeTangentArc()` implementation (non-parametric 2D fillet)
- `src/modify.cpp:109` — `ParametricCurve::MakeFromEntity()` (line/arc parameterization)
- `src/modify.cpp:170` — `ParametricCurve::CreateRequestTrimmedTo()` (trim + constrain)
- `src/ui.h:730` — `ParametricCurve` struct definition
- `src/constrainteq.cpp:938` — `ARC_LINE_TANGENT` constraint equation
- `src/constrainteq.cpp:977` — `CURVE_CURVE_TANGENT` constraint equation
- `src/request.cpp:33` — `ARC_OF_CIRCLE` request definition (3 points, workplane-only)
- `src/solvespace.h:554-556` — `tangentArcRadius/Manual/Modify` globals (not saved)
- `src/entity.cpp:~ArcGetAngles` — arc angle computation from point[0,1,2]
