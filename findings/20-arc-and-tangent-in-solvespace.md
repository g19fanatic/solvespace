# Task 20: ARC_OF_CIRCLE in SolveSpace — Entity, Request, and Tangent Constraints

## Summary

This research traces the complete lifecycle of an `ARC_OF_CIRCLE` entity in SolveSpace, from
request generation through geometry evaluation, Bezier curve generation, tangent constraint
enforcement, and the `ParametricCurve` helper — all of which are directly reusable for
implementing a parametric 2D sketch fillet.

---

## 1. ARC_OF_CIRCLE Data Model

### 1.1 Request Type Definition

**File**: `src/sketch.h:384`
```cpp
// Request::Type enum
ARC_OF_CIRCLE = 500,
```

**File**: `src/sketch.h:453`
```cpp
// Entity::Type enum
ARC_OF_CIRCLE = 14000,
```

### 1.2 Request → Entity Mapping

**File**: `src/request.cpp` (EntReqMap table)
```cpp
{ Request::Type::ARC_OF_CIRCLE, Entity::Type::ARC_OF_CIRCLE, 3, false, true, false },
//                               entity type                  pts  xtra? normal dist
```

An `ARC_OF_CIRCLE` request generates:
- **Entity 0**: The arc itself (the `ARC_OF_CIRCLE` entity type)
- **Entity 1** (`point[0]`): The **center point** of the arc
- **Entity 2** (`point[1]`): The **start endpoint** of the arc
- **Entity 3** (`point[2]`): The **finish endpoint** of the arc
- **Entity 32** (`normal`): The **normal** (workplane orientation quaternion)

**Critical**: The radius is NOT stored as a separate parameter. The radius is computed as
`distance(point[0], point[1])` — it's a derived quantity from the center and start point.

### 1.3 Entity Structure in Memory

From `src/entity.cpp`, `ArcGetAngles()`:
```cpp
void EntityBase::ArcGetAngles(double *thetaa, double *thetab, double *dtheta) const {
    ssassert(type == Type::ARC_OF_CIRCLE, "Unexpected entity type");

    Quaternion q = Normal()->NormalGetNum();
    Vector u = q.RotationU(), v = q.RotationV();

    Vector c  = SK.GetEntity(point[0])->PointGetNum();   // center
    Vector pa = SK.GetEntity(point[1])->PointGetNum();   // start
    Vector pb = SK.GetEntity(point[2])->PointGetNum();   // finish

    Point2d c2  = c.Project2d(u, v);
    Point2d pa2 = (pa.Project2d(u, v)).Minus(c2);
    Point2d pb2 = (pb.Project2d(u, v)).Minus(c2);

    *thetaa = atan2(pa2.y, pa2.x);     // start angle
    *thetab = atan2(pb2.y, pb2.x);     // finish angle
    *dtheta = *thetab - *thetaa;       // arc sweep angle
    // dtheta is kept in (0, 2π]
    while(*dtheta < 1e-6) *dtheta += 2*PI;
    while(*dtheta > (2*PI)) *dtheta -= 2*PI;
}
```

**Key observation**: The arc is defined by **two endpoint positions** (not as center+angles).
The constraint solver keeps the three point entities (center + 2 endpoints) consistent.
The arc always goes **counter-clockwise** from start (`point[1]`) to finish (`point[2]`).

### 1.4 Implicit Arc Equation (GenerateEquations)

**File**: `src/entity.cpp`
```cpp
case Type::ARC_OF_CIRCLE: {
    // If the two endpoints are not POINTS_COINCIDENT, enforce equal radii:
    Expr *ra = Constraint::Distance(workplane, point[0], point[1]);  // radius to start
    Expr *rb = Constraint::Distance(workplane, point[0], point[2]);  // radius to finish
    AddEq(l, ra->Minus(rb), 0);  // ra == rb  (equal radii constraint)
    break;
}
```

The arc entity itself generates exactly **1 equation**: both endpoints must be equidistant from
the center. This is the "arc is circular" constraint. The radius is implicitly determined by the
solver based on all other constraints applied.

---

## 2. Bezier Curve Generation for ARC_OF_CIRCLE

**File**: `src/drawentity.cpp:412-468`

The Bezier generation is elegant: it splits the arc into 1–4 quadratic rational Bezier segments
(depending on sweep angle) and uses the exact NURBS representation of a circular arc:

```cpp
case Type::ARC_OF_CIRCLE: {
    Vector center = SK.GetEntity(point[0])->PointGetNum();
    Quaternion q = SK.GetEntity(normal)->NormalGetNum();
    Vector u = q.RotationU(), v = q.RotationV();
    double r = CircleGetRadiusNum();   // = distance(center, point[1])
    double thetaa, thetab, dtheta;
    ArcGetAngles(&thetaa, &thetab, &dtheta);

    int n;  // number of segments
    if     (dtheta > (3*PI/2 + 0.01)) n = 4;
    else if(dtheta > (PI + 0.01))     n = 3;
    else if(dtheta > (PI/2 + 0.01))   n = 2;
    else                               n = 1;

    dtheta /= n;  // per-segment sweep

    for(i = 0; i < n; i++) {
        double s = sin(thetaa), c = cos(thetaa);
        Vector p0 = center + u*(r*c) + v*(r*s);   // start point
        Vector t0 = u*(-r*s) + v*(r*c);           // tangent at start

        thetaa += dtheta;
        s = sin(thetaa); c = cos(thetaa);
        Vector p2 = center + u*(r*c) + v*(r*s);   // end point
        Vector t2 = u*(-r*s) + v*(r*c);           // tangent at end

        // Control point = intersection of two tangent lines
        Vector p1 = Vector::AtIntersectionOfLines(p0, p0+t0, p2, p2+t2, NULL);

        SBezier sb = SBezier::From(p0, p1, p2);
        sb.weight[1] = cos(dtheta/2);  // EXACT rational NURBS weight for circle
        sbl->l.Add(&sb);
    }
}
```

**Key formula**: `weight[1] = cos(dtheta/2)` gives the **exact** NURBS representation of a
circular arc. This is the standard result from projective geometry: a rational quadratic Bezier
with this weight exactly represents a conic arc.

---

## 3. Tangent Constraints for ARC_OF_CIRCLE

### 3.1 ARC_LINE_TANGENT Constraint

**File**: `src/sketch.h:689`
```cpp
ARC_LINE_TANGENT = 123,
```

**Equation** (from `src/constrainteq.cpp:938`):
```cpp
case Type::ARC_LINE_TANGENT: {
    EntityBase *arc  = SK.GetEntity(entityA);
    EntityBase *line = SK.GetEntity(entityB);

    ExprVector ac = SK.GetEntity(arc->point[0])->PointGetExprs();  // center
    ExprVector ap = SK.GetEntity(arc->point[other ? 2 : 1])->PointGetExprs();  // endpoint

    ExprVector ld = line->VectorGetExprs();  // line direction

    // The line is perpendicular to the radius at the endpoint
    AddEq(l, ld.Dot(ac.Minus(ap)), 0);
    // Geometric meaning: line direction · (center - endpoint) = 0
    // i.e., line is tangent to arc (line ⊥ radius at endpoint)
    return;
}
```

**Interpretation**: A line is tangent to an arc when the line's direction vector is perpendicular
to the arc's radius vector at the shared endpoint. This produces exactly 1 equation.

**Setup** (`src/constraint.cpp:136`):
```cpp
bool Constraint::ConstrainArcLineTangent(Constraint *c, Entity *line, Entity *arc,
                                         Entity *arcendpoint) {
    Vector a1 = SK.GetEntity(arc->point[1])->PointGetNum();
    Vector a2 = SK.GetEntity(arc->point[2])->PointGetNum();
    if(l0.Equals(a1) || l1.Equals(a1)) {
        c->other = false;  // tangent at start endpoint (point[1])
    } else if(l0.Equals(a2) || l1.Equals(a2)) {
        c->other = true;   // tangent at finish endpoint (point[2])
    }
    // ...
}
```

The `other` flag distinguishes which endpoint (start or finish) is the tangent point.

### 3.2 CURVE_CURVE_TANGENT Constraint

**File**: `src/constrainteq.cpp:977`

For arc-arc or arc-cubic tangency. For arcs, uses the radius vector (which is perpendicular to
the tangent, hence reverses the parallel condition):

```cpp
case Type::CURVE_CURVE_TANGENT: {
    bool parallel = true;
    ExprVector dir[2];
    for(i = 0; i < 2; i++) {
        EntityBase *e = SK.GetEntity((i == 0) ? entityA : entityB);
        if(e->type == Entity::Type::ARC_OF_CIRCLE) {
            center   = SK.GetEntity(e->point[0])->PointGetExprs();
            endpoint = SK.GetEntity(e->point[oth ? 2 : 1])->PointGetExprs();
            dir[i] = endpoint.Minus(center);  // RADIUS vector (not tangent)
            parallel = !parallel;  // flip to antiparallel because we used radius not tangent
        } else if(e->type == Entity::Type::CUBIC) {
            dir[i] = cubic tangent vector;
        }
    }
    // For arc-arc: dir[0] and dir[1] are both radii, double-flip → parallel check
    // For arc-line (shouldn't happen here): handled by ARC_LINE_TANGENT
    if(parallel) {
        AddEq(l, ((dir[0]).Cross(dir[1])).Dot(wn), 0);  // cross product = 0
    } else {
        AddEq(l, (dir[0]).Dot(dir[1]), 0);               // dot product = 0
    }
}
```

---

## 4. ParametricCurve Helper — The Key Infrastructure

**File**: `src/graphicswin.cpp:109-228`, declared in `src/ui.h:721`

`ParametricCurve` is a local helper struct in `GraphicsWindow` used by `MakeTangentArc()`.
It provides a unified parametric `t ∈ [0, 1]` interface over both line segments and arcs.

### 4.1 Complete ParametricCurve API

```cpp
class GraphicsWindow::ParametricCurve {
public:
    bool isLine;          // true = line, false = arc
    // Line data:
    Vector p0, p1;        // line endpoints (p0=start, p1=finish)
    // Arc data:
    Vector p0;            // arc center
    Vector u, v;          // workplane basis vectors
    double r;             // radius
    double theta0, theta1, dtheta;  // start angle, finish angle, sweep

    void MakeFromEntity(hEntity he, bool reverse);
    Vector PointAt(double t);    // point on curve at parameter t ∈ [0, 1]
    Vector TangentAt(double t);  // tangent vector at parameter t
    double LengthForAuto();      // appropriate length for auto-radius sizing

    void CreateRequestTrimmedTo(double t, bool reuseOrig,
        hEntity orig, hEntity arc, bool arcFinish, bool pointf);
    void ConstrainPointIfCoincident(hEntity hpt);
};
```

### 4.2 MakeFromEntity — Initialization

```cpp
void ParametricCurve::MakeFromEntity(hEntity he, bool reverse) {
    *this = {};
    Entity *e = SK.GetEntity(he);
    if(e->type == Entity::Type::LINE_SEGMENT) {
        isLine = true;
        p0 = e->EndpointStart();
        p1 = e->EndpointFinish();
        if(reverse) swap(p0, p1);
    } else if(e->type == Entity::Type::ARC_OF_CIRCLE) {
        isLine = false;
        p0 = SK.GetEntity(e->point[0])->PointGetNum();  // center
        Vector pe = SK.GetEntity(e->point[1])->PointGetNum();  // start
        r = (pe.Minus(p0)).Magnitude();
        e->ArcGetAngles(&theta0, &theta1, &dtheta);
        if(reverse) { swap(theta0, theta1); dtheta = -dtheta; }
        EntityBase *wrkpln = SK.GetEntity(e->workplane)->Normal();
        u = wrkpln->NormalU();
        v = wrkpln->NormalV();
    }
}
```

### 4.3 PointAt and TangentAt

```cpp
Vector ParametricCurve::PointAt(double t) {
    if(isLine) {
        return p0.Plus((p1.Minus(p0)).ScaledBy(t));
    } else {
        double theta = theta0 + dtheta*t;
        return p0.Plus(u.ScaledBy(r*cos(theta)).Plus(v.ScaledBy(r*sin(theta))));
    }
}

Vector ParametricCurve::TangentAt(double t) {
    if(isLine) {
        return p1.Minus(p0);  // constant tangent for line
    } else {
        double theta = theta0 + dtheta*t;
        Vector tan = u.ScaledBy(-r*sin(theta)).Plus(v.ScaledBy(r*cos(theta)));
        return tan.ScaledBy(dtheta);  // sign carries direction
    }
}
```

### 4.4 CreateRequestTrimmedTo — Core Trimming Logic

This function is the most complex and important. It creates a new entity (or modifies the original)
such that one end connects to the tangent arc:

```cpp
void ParametricCurve::CreateRequestTrimmedTo(double t, bool reuseOrig,
    hEntity orig, hEntity arc, bool arcFinish, bool pointf)
{
    if(isLine) {
        if(reuseOrig) {
            // Modify original line's endpoint to PointAt(t)
            SK.GetEntity(orig)->point[pointf ? 1 : 0] → PointForceTo(PointAt(t));
        } else {
            // Create new line from PointAt(t) to PointAt(1)
            hRequest hr = AddRequest(Request::Type::LINE_SEGMENT);
            ...
            // Constrain new line's internal endpoint on original line
            Constraint::PT_ON_LINE constraint added
        }
        // Critical: add ARC_LINE_TANGENT constraint between the arc and this line
        Constraint::Constrain(Type::ARC_LINE_TANGENT, arc, line, arcFinish=...);

    } else {  // arc case
        if(reuseOrig) {
            SK.GetEntity(orig)->point[pointf ? 2 : 1] → PointForceTo(PointAt(t));
        } else {
            // Create new arc from PointAt(t) to PointAt(1), same center
            hRequest hr = AddRequest(Request::Type::ARC_OF_CIRCLE);
            ...
        }
        // Critical: add CURVE_CURVE_TANGENT constraint between arcs
        Constraint::Constrain(Type::CURVE_CURVE_TANGENT, arc, orig, arcFinish, dtheta<0);
    }
}
```

### 4.5 LengthForAuto — Radius Capping

```cpp
double ParametricCurve::LengthForAuto() {
    if(isLine) {
        return (p1.Minus(p0)).Magnitude() / 3;  // 1/3 of line length
    } else {
        return (fabs(dtheta)*r) / 20;  // 1/20 of arc length
    }
}
```

Used in `MakeTangentArc` to ensure the fillet arc doesn't consume more than 1/3 of adjacent
line length (or 1/20 of arc length). For a **parametric** fillet Group, the user provides radius
directly, so `LengthForAuto()` serves as a validation bound.

---

## 5. ARC_OF_CIRCLE in Request::Generate

**File**: `src/request.cpp`

```cpp
// ARC_OF_CIRCLE request generates:
// - 1 main entity (the arc, Entity 0)
// - 3 point sub-entities:
//   Entity 1 = point[0] = center
//   Entity 2 = point[1] = start
//   Entity 3 = point[2] = finish
// - 1 normal sub-entity (workplane orientation, Entity 32)
// No separate distance entity — radius is implicit from |point[1] - point[0]|
```

For workplane sketching (type == POINT_IN_2D):
- Each point gets 2 params: `h.param(16 + 3*i + 0)` (u) and `h.param(16 + 3*i + 1)` (v)
- Total params for arc: 2*3 = 6 (u,v for each of 3 points) + 0 (normal = workplane copy)

For 3D (FREE_IN_3D):
- Each point gets 3 params: (x, y, z)
- Normal gets 4 params: quaternion w, vx, vy, vz

---

## 6. CircleGetRadiusNum vs. ArcGetAngles

**File**: `src/entity.cpp:132-164`

```cpp
double EntityBase::CircleGetRadiusNum() const {
    if(type == Type::CIRCLE) {
        return SK.GetEntity(distance)->DistanceGetNum();   // explicit param
    } else if(type == Type::ARC_OF_CIRCLE) {
        Vector c  = SK.GetEntity(point[0])->PointGetNum();  // center
        Vector pa = SK.GetEntity(point[1])->PointGetNum();  // start
        return (pa.Minus(c)).Magnitude();                    // implicit radius
    }
}
```

For `ARC_OF_CIRCLE`, radius = |start - center|. There is **no explicit radius parameter**; the
solver determines the radius by constraining the positions of the three points.

`CircleGetRadiusExpr()` returns `Constraint::Distance(workplane, point[0], point[1])` for arcs —
a symbolic distance expression usable in the solver.

---

## 7. Endpoint Semantics

```cpp
Vector EntityBase::EndpointStart() const {
    if(type == Type::ARC_OF_CIRCLE)
        return SK.GetEntity(point[1])->PointGetNum();  // NOT point[0]!
}

Vector EntityBase::EndpointFinish() const {
    if(type == Type::ARC_OF_CIRCLE)
        return SK.GetEntity(point[2])->PointGetNum();
}
```

**Note**: For arcs:
- `point[0]` = **center** (not an endpoint!)
- `point[1]` = **start** endpoint
- `point[2]` = **finish** endpoint

This is distinct from lines where `point[0]` and `point[1]` are both endpoints.

---

## 8. HasEndpoints() and IsCircle()

```cpp
bool EntityBase::HasEndpoints() const {
    return (type == Type::LINE_SEGMENT) ||
           (type == Type::CUBIC) ||
           (type == Type::ARC_OF_CIRCLE);  // arcs have endpoints; circles do not
}

bool EntityBase::IsCircle() const {
    return (type == Type::CIRCLE) || (type == Type::ARC_OF_CIRCLE);
}
```

`CIRCLE` has no endpoints (closed); `ARC_OF_CIRCLE` has endpoints (open). Both are "circles"
for radius/geometry purposes.

---

## 9. ARC_OF_CIRCLE in drawentity.cpp Rendering

**File**: `src/drawentity.cpp:754`

Arcs are rendered with the same code path as lines and cubics:
```cpp
case Type::ARC_OF_CIRCLE:
    // ... GetOrGenerateBezierCurves() → rational quadratic NURBS segments
    // ... Then either DrawBeziers() or DrawEdges() via Canvas
```

The arc is drawn by converting to piecewise-linear via Bezier and chord-tolerance subdivision.

---

## 10. How MakeTangentArc Uses All This (The Full 2D Fillet Pattern)

**File**: `src/modify.cpp:252-443`

The full non-parametric 2D fillet algorithm:

1. **Input**: User selects a shared vertex point where two entities meet
2. **Find entities**: Scan requests in group/workplane for those with endpoint at vertex
3. **Create ParametricCurve objects**: One per adjacent entity, oriented away from vertex
4. **Newton iteration** (1000 iterations):
   - Compute `PointAt(t[0])`, `PointAt(t[1])`, `TangentAt(t[0])`, `TangentAt(t[1])`
   - Find tangent intersection `pinter`
   - Compute `theta = acos(t0·t1)` — corner angle
   - Compute `el = r/tan(theta/2)` — leg length
   - Update `t[0]` and `t[1]` to move tangent points to correct positions
5. **Compute arc center**: Cross product to determine which side; `center = PointAt(t[0]) ± r*perpendicular`
6. **Create arc request**: `ARC_OF_CIRCLE` with forced center/start/finish positions
7. **Trim/modify adjacent entities**: `pc[0].CreateRequestTrimmedTo(t[0], ...)` which also adds
   the ARC_LINE_TANGENT or CURVE_CURVE_TANGENT constraint

The Newton iteration handles the general case (arc-arc, line-arc, line-line) uniformly.
For the **line-line case** (which is all that's needed for a simple 2D fillet), there is a
**closed-form solution** (no iteration needed):
- `theta` = angle between lines
- `el = r/tan(theta/2)` — exact tangent point distance from corner
- `center = corner + perpendicular_bisector * r/sin(theta/2)`

---

## 11. Key Implications for Parametric 2D Sketch Fillet

### What Changes for a Parametric Group::Type::FILLET_2D

The non-parametric `MakeTangentArc()`:
- Uses `SS.tangentArcRadius` (session global, NOT saved)
- Hard-codes entities via `PointForceTo()` (no solver degrees of freedom)
- Called once; subsequent moves don't update the fillet

A **parametric 2D fillet group** would need:
- `Group::valA` = fillet radius (persisted via file.cpp SAVED[])
- `Group::Generate()` copies adjacent line/arc entities
- Computes fillet geometry using the same math as `MakeTangentArc()`
- But uses `AddParam(h.param(0), valA)` — radius is a solver variable
- The arc `point[0]` (center) position is **derived** from the radius and adjacent entity positions

### Reusable Infrastructure

| Component | Location | Reusability |
|-----------|----------|-------------|
| `ArcGetAngles()` | entity.cpp:148 | ✅ Direct reuse |
| `CircleGetRadiusNum()` | entity.cpp:140 | ✅ Direct reuse |
| `ParametricCurve::PointAt()` | graphicswin.cpp:144 | ✅ Direct reuse |
| `ParametricCurve::TangentAt()` | graphicswin.cpp:152 | ✅ Direct reuse |
| `ARC_LINE_TANGENT` equation | constrainteq.cpp:938 | ✅ Already exists |
| `CURVE_CURVE_TANGENT` equation | constrainteq.cpp:977 | ✅ Already exists |
| Bezier arc generation | drawentity.cpp:412 | ✅ Auto via `GenerateBezierCurves()` |

### What is NOT Reusable

| Component | Why Not |
|-----------|---------|
| Newton iteration in `MakeTangentArc()` | Not needed for line-line (closed form); too slow for parametric |
| `CreateRequestTrimmedTo()` | Non-parametric; creates new requests imperatively |
| `tangentArcRadius` global | Not persisted; for parametric use `Group::valA` |

### Closed-Form Line-Line Fillet (For Simple 2D Fillet Group)

Given two line entities meeting at vertex V, with tangent directions t1 and t2:
```
theta = acos(t1 · t2)          // corner angle
el = r / tan(theta/2)           // leg length
P1 = V + normalize(-t1) * el   // start tangent point  
P2 = V + normalize(-t2) * el   // finish tangent point
C = V + normalize(-t1 + -t2) * r / sin(theta/2)  // arc center

// OR equivalently:
bis = normalize(normalize(-t1) + normalize(-t2))  // bisector
C = V + bis * r / sin(theta/2)
```

This is entirely closed-form for line-line — no Newton iteration required.

---

## 12. ARC_OF_CIRCLE in Full Rendering Pipeline

1. **User creates**: `Request::Type::ARC_OF_CIRCLE` via `GraphicsWindow::AddRequest()`
2. **Entities generated**: 3 points + 1 normal via `Request::Generate()`
3. **Solver runs**: Positions the 3 points using all constraints + `GenerateEquations()` implicit radius equation
4. **Bezier generated**: `DrawEntity::GenerateBezierCurves()` → rational quadratic segments with `weight[1] = cos(dtheta/2)`
5. **Display**: Via `Canvas::DrawBeziers()` or piecewise-linear `Canvas::DrawEdges()`
6. **Export**: Via `SBezier` → STEP/SVG/DXF paths (arc is exact NURBS, no approximation needed)

---

## 13. File Locations Summary

| File | Key Content | Lines |
|------|-------------|-------|
| `src/sketch.h` | Request::Type::ARC_OF_CIRCLE = 500 | :384 |
| `src/sketch.h` | Entity::Type::ARC_OF_CIRCLE = 14000 | :453 |
| `src/sketch.h` | ARC_LINE_TANGENT = 123 | :689 |
| `src/request.cpp` | EntReqMap: 3 pts, has normal | ~line 27 |
| `src/entity.cpp` | ArcGetAngles() | :148 |
| `src/entity.cpp` | CircleGetRadiusNum() | :140 |
| `src/entity.cpp` | GenerateEquations: equal radii | :946 |
| `src/drawentity.cpp` | Bezier generation, weight=cos(dtheta/2) | :412 |
| `src/constrainteq.cpp` | ARC_LINE_TANGENT equation | :938 |
| `src/constrainteq.cpp` | CURVE_CURVE_TANGENT equation | :977 |
| `src/constraint.cpp` | ConstrainArcLineTangent() | :136 |
| `src/graphicswin.cpp` | ParametricCurve::MakeFromEntity() | :109 |
| `src/graphicswin.cpp` | ParametricCurve::PointAt() | :144 |
| `src/graphicswin.cpp` | ParametricCurve::TangentAt() | :152 |
| `src/graphicswin.cpp` | ParametricCurve::CreateRequestTrimmedTo() | :170 |
| `src/modify.cpp` | MakeTangentArc() full algorithm | :252 |

---

## 14. Implications for 3D Fillet SCurve Equivalence

The 2D arc machinery maps directly to 3D:
- 2D `ARC_OF_CIRCLE` → 3D fillet surface's trim boundary is an `SCurve`
- 2D `weight[1] = cos(dtheta/2)` → 3D `SSurface::ctrl[0][1].w = cos(angle/2)` in fillet NURBS
- 2D `ARC_LINE_TANGENT` (ld · radius = 0) → 3D G1 continuity condition (normal1 × edge_tangent = normal2 × edge_tangent)
- 2D `ParametricCurve::PointAt()` → 3D `SSurface::PointAt(u, v)`

The architectural pattern is **identical** — just lifted to 3D B-rep.
