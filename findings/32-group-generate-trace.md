# Task 32: Group::Generate and GenerateShellAndMesh Code Path Trace

## Overview

This document traces the complete code path for an EXTRUDE group through `Group::Generate()`
and `Group::GenerateShellAndMesh()` in SolveSpace, identifying exactly where chamfer/fillet
geometry would need to be injected.

## Source Files Analyzed

- `src/group.cpp` — Group::Generate(), Group::MenuGroup(), Group::GenerateEquations()
- `src/groupmesh.cpp` — Group::GenerateShellAndMesh(), Group::GenerateForBoolean(), IsMeshGroup()
- `src/srf/shell.cpp` — SShell::MakeFromExtrusionOf(), SShell::MakeFromRevolutionOf(), SShell::MakeFromCopyOf()
- `src/srf/surface.cpp` — SSurface::FromPlane(), SSurface::FromExtrusionOf(), SSurface::TriangulateInto()
- `src/srf/boolean.cpp` — SShell::MakeFromBoolean(), SShell::MakeFromAssemblyOf()

---

## 1. Group::Generate() — The Entity and Param Factory

### Location: `src/group.cpp` (entire function starting ~line 340)

`Group::Generate()` is called during `SS.GenerateAll()` to produce:
1. **Solver params** (`param` list) — values the constraint solver will find
2. **Entities** (`entity` list) — geometric primitives (points, lines, normals, faces)

### EXTRUDE Case — Detailed Trace

```cpp
case Type::EXTRUDE: {
    // 1. Create 3 solver params for the extrusion vector
    AddParam(param, h.param(0), gn.x);  // initial guess from screen normal
    AddParam(param, h.param(1), gn.y);
    AddParam(param, h.param(2), gn.z);

    // 2. Determine top/bottom offsets (ONE_SIDED: 0..2*d, TWO_SIDED: -d..+d)
    int ai, af;
    if(ONE_SIDED || ONE_SKEWED)  { ai=0; af=2; }
    else                          { ai=-1; af=1; }

    // 3. Get one arbitrary point in the source sketch for top/bottom face refs
    hEntity pt = {0};

    // 4. Loop over entities in source group (opA)
    for(i = 0; i < entity->n; i++) {
        Entity *e = &(entity->Get(i));
        if(e->group != opA) continue;
        if(e->IsPoint()) pt = e->h;

        e->CalculateNumerical(/*forExport=*/false);
        hEntity he = e->h;  // note: e may be invalidated after CopyEntity

        // Copy entity to top position (ai offset, REMAP_BOTTOM tag)
        CopyEntity(entity, SK.GetEntity(he), ai, REMAP_BOTTOM, 
                   h.param(0), h.param(1), h.param(2), ...);
        // Copy entity to bottom position (af offset, REMAP_TOP tag)
        CopyEntity(entity, SK.GetEntity(he), af, REMAP_TOP, 
                   h.param(0), h.param(1), h.param(2), ...);
        
        // Create the side lines/face entities for each sketch entity
        MakeExtrusionLines(entity, he);
    }
    
    // 5. Create top/bottom face entities using remapped arbitrary point
    MakeExtrusionTopBottomFaces(entity, pt);
    return;
}
```

### Key Insight for Chamfer/Fillet

For a **CHAMFER or FILLET group**, `Generate()` is far simpler:

```cpp
case Type::CHAMFER: {
    // Only ONE parameter: the offset distance
    AddParam(param, h.param(0), valA);  // initial guess = stored value
    // No entities to copy — the geometry is entirely in GenerateShellAndMesh()
    return;
}
```

The CHAMFER group doesn't need to create any new sketch entities — it just
needs the solver to know about the offset parameter, and all the B-rep work
happens in `GenerateShellAndMesh()`.

---

## 2. Group::GenerateEquations()

### Location: `src/group.cpp`

For EXTRUDE, `GenerateEquations()` adds constraints to lock the extrusion vector
normal to the workplane:
```cpp
} else if((type == Type::EXTRUDE) && (subtype != ONE_SKEWED) && ...) {
    // Lock extrusion direction perpendicular to workplane normal
    AddEq(l, u.Dot(extruden), 0);
    AddEq(l, v.Dot(extruden), 1);
}
```

**For CHAMFER/FILLET:** NO equations needed in `GenerateEquations()`. The offset
distance `h.param(0)` is a free parameter with initial value `valA`, and no
constraint equations constrain it further. The solver just "sees" it as unconstrained,
and it retains its initial value (stored in `valA`). This is identical to how REVOLVE
handles the angle parameter (`h.param(3)` is the angle, free to be constrained by
the user or left at its default).

---

## 3. Group::GenerateShellAndMesh() — The Full B-rep Pipeline

### Location: `src/groupmesh.cpp` (entire function)

This is the heart of solid model generation. Here is the complete flow:

### Step 1: Initialization
```cpp
void Group::GenerateShellAndMesh() {
    bool prevBooleanFailed = booleanFailed;
    booleanFailed = false;
    Group *srcg = this;

    thisShell.Clear();
    thisMesh.Clear();
    runningShell.Clear();
    runningMesh.Clear();
```

### Step 2: Source Validation
```cpp
    bool haveSrc = true;
    if(type == Type::EXTRUDE || type == Type::LATHE || type == Type::REVOLVE) {
        Group *src = SK.GetGroup(opA);
        if(src->polyError.how != PolyError::GOOD) {
            haveSrc = false;
        }
    }
```
For CHAMFER/FILLET, we do NOT check `polyError` — instead we check whether:
1. `opA` group's `runningShell` is non-empty
2. The two face handles (predef.entityB, predef.entityC) are valid SSurfaces

### Step 3: Generate `thisShell` (Per-Group Solid Contribution)

For EXTRUDE:
```cpp
    } else if(type == Type::EXTRUDE && haveSrc) {
        Group *src = SK.GetGroup(opA);
        Vector translate = Vector::From(h.param(0), h.param(1), h.param(2));

        // Compute top/bottom vectors
        Vector tbot, ttop;
        if(ONE_SIDED || ONE_SKEWED) { tbot=zero; ttop=translate*2; }
        else                         { tbot=translate*-1; ttop=translate*1; }

        // For each loop in the source bezier loops:
        SBezierLoopSet *sblss = &(src->bezierLoops);
        for(sbls ...) {
            int is = thisShell.surface.n;
            
            // Create the extruded shell (surfaces + trim curves)
            thisShell.MakeFromExtrusionOf(sbls, tbot, ttop, color);

            // Annotate faces with selectable entity handles
            for(i = is; i < thisShell.surface.n; i++) {
                SSurface *ss = &(thisShell.surface[i]);
                // ... assign ss->face = Remap(...) for top, bottom, and side faces
            }
        }
    }
```

**For CHAMFER:** The `thisShell` generation would be:
```cpp
    } else if(type == Type::CHAMFER) {
        Group *src = SK.GetGroup(opA);
        double dist = SK.GetParam(h.param(0))->val;
        
        // Get the source shell (what we're chamfering)
        SShell *srcShell = &(src->runningShell);
        
        // Build chamfer geometry using direct topology injection
        thisShell.MakeFromChamferOf(srcShell, this, dist);
    }
```

### Step 4: Post-Processing — MergeCoincidentSurfaces
```cpp
    if(srcg->meshCombine != CombineAs::ASSEMBLE) {
        thisShell.MergeCoincidentSurfaces();
    }
```
This merges adjacent surfaces that happen to be coplanar. CHAMFER surfaces will
NOT be coplanar with existing surfaces, so this is a no-op for chamfers. Safe to call.

### Step 5: Boolean Combination via GenerateForBoolean
```cpp
    Group *prevg = srcg->RunningMeshGroup();

    if(!IsForcedToMesh()) {
        SShell *prevs = &(prevg->runningShell);
        GenerateForBoolean<SShell>(prevs, &thisShell, &runningShell,
            srcg->meshCombine);
        
        if(srcg->meshCombine != CombineAs::ASSEMBLE) {
            runningShell.MergeCoincidentSurfaces();
        }
        
        booleanFailed = runningShell.booleanFailed;
    } else {
        // Mesh fallback path...
    }
```

**Critical Insight:** For CHAMFER/FILLET, `GenerateForBoolean()` will execute with
`how = CombineAs::DIFFERENCE`. This is the standard boolean pipeline.

BUT — as established in prior research (task 18, 28) — the standard `MakeFromBoolean`
pipeline CANNOT be used for chamfer because:
1. The chamfer surface is coincident with the source shell (shares edges)
2. Coincident faces cause `COINC_OPP` degeneracy → boolean failure
3. This is exactly what phkahler's PR #1501 crashed on

**Therefore:** CHAMFER/FILLET must use a **different pipeline**:
- Either `MakeFromAssemblyOf` (concatenation, no boolean) as the base
- Plus **direct topology injection** into the copied shell

### Step 6: Display Item Generation
```cpp
    displayDirty = true;
```
This flag triggers `GenerateDisplayItems()` to re-triangulate on next draw.
No changes needed for chamfer/fillet — triangulation is fully automatic.

---

## 4. GenerateForBoolean() — The Boolean Template

### Location: `src/groupmesh.cpp:182`

```cpp
template<class T>
void Group::GenerateForBoolean(T *prevs, T *thiss, T *outs, Group::CombineAs how) {
    if(thiss->IsEmpty() || suppress) {
        outs->MakeFromCopyOf(prevs);
        return;
    }
    switch(how) {
        case CombineAs::UNION:      outs->MakeFromUnionOf(prevs, thiss); break;
        case CombineAs::DIFFERENCE: outs->MakeFromDifferenceOf(prevs, thiss); break;
        case CombineAs::INTERSECTION: ...
        case CombineAs::ASSEMBLE:   outs->MakeFromAssemblyOf(prevs, thiss); break;
    }
}
```

**Hook-in for CHAMFER:** The CHAMFER group's `GenerateShellAndMesh()` must produce
a `runningShell` that IS the modified source shell (not combine-and-differentiate).

The cleanest approach (as phkahler confirmed via PR #1501) is:
1. Copy the source `runningShell` into `thisShell` (via `MakeFromCopyOf`)
2. Surgically modify `thisShell` by calling `thisShell.MakeFromChamferOf(srcShell, this, dist)`
3. Then skip `GenerateForBoolean()` and directly do `runningShell.MakeFromCopyOf(&thisShell)`

OR:
1. Set `thisShell` empty initially
2. Set `runningShell` = result of chamfering `prevg->runningShell`
3. Mark `meshCombine = CombineAs::ASSEMBLE` so the boolean step is a no-op copy

The second approach better matches the existing code flow.

---

## 5. IsMeshGroup() — Must Return True for CHAMFER/FILLET

### Location: `src/groupmesh.cpp:545`

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

**Required change:** Add `CHAMFER` and `FILLET` to this list:
```cpp
        case Group::Type::CHAMFER:
        case Group::Type::FILLET:
            return true;
```

Without this, CHAMFER/FILLET groups won't contribute solid geometry to the model.

---

## 6. RunningMeshGroup() — How Groups Chain Together

### Location: `src/groupmesh.cpp:529`

```cpp
Group *Group::RunningMeshGroup() const {
    if(type == Type::TRANSLATE || type == Type::ROTATE) {
        return SK.GetGroup(opA)->RunningMeshGroup();
    } else {
        return PreviousGroup();
    }
}
```

For CHAMFER/FILLET, `RunningMeshGroup()` returns `PreviousGroup()`, which is
correct — the "previous running mesh" is the cumulative solid before the chamfer.

---

## 7. SShell::MakeFromExtrusionOf() — Model for the New MakeFromChamferOf()

### Location: `src/srf/shell.cpp`

This is the best model for how to write `SShell::MakeFromChamferOf()`.

```cpp
void SShell::MakeFromExtrusionOf(SBezierLoopSet *sbls, Vector t0, Vector t1, RgbaColor color)
{
    // 1. Create top and bottom flat surfaces (SSurface::FromPlane)
    SSurface s0 = SSurface::FromPlane(orig.Plus(t0), u, v);
    SSurface s1 = SSurface::FromPlane(orig.Plus(t1).Plus(u), u.ScaledBy(-1), v);
    hSSurface hs0 = surface.AddAndAssignId(&s0);
    hSSurface hs1 = surface.AddAndAssignId(&s1);

    // 2. For each bezier in the input loop:
    for(sb = ...) {
        // Create side surface of extrusion
        SSurface ss = SSurface::FromExtrusionOf(sb, t0, t1);
        hSSurface hsext = surface.AddAndAssignId(&ss);

        // Create trim curves (at t0 and t1 positions)
        SCurve sc = {};
        sc.isExact = true;
        sc.exact = sb->TransformedBy(t0, ...);
        sc.surfA = hs0;  // this curve separates the bottom face from the side
        sc.surfB = hsext;
        hSCurve hc0 = curve.AddAndAssignId(&sc);

        // Add trim references to both surfaces
        stb = STrimBy::EntireCurve(this, hc0, backwards=false);
        surface.FindById(hs0)->trim.Add(&stb);
        stb = STrimBy::EntireCurve(this, hc0, backwards=true);
        surface.FindById(hsext)->trim.Add(&stb);

        // ... and line trim between adjacent extrusion sides
    }
}
```

### Pattern for MakeFromChamferOf()

The new `SShell::MakeFromChamferOf(SShell *src, Group *g, double dist)` would:

1. **Copy all surfaces and curves from source** (via `MakeFromCopyOf`)
2. **Find the target SCurve** (shared edge between the two selected faces):
   - Identify `surfA` = source face 0, `surfB` = source face 1
   - Find SCurve in `src` where `sc.surfA == faceA.h && sc.surfB == faceB.h`
3. **Compute chamfer geometry**:
   - Get edge tangent `t` from SCurve control points
   - Get face normals `n1` (from faceA), `n2` (from faceB)
   - Offset directions: `d1 = t.Cross(n1)`, `d2 = n2.Cross(t)` (inward)
   - For edge start and end vertices (V1, V2):
     - `A = V1 + dist*d1`, `B = V2 + dist*d1`
     - `C = V2 + dist*d2`, `D = V1 + dist*d2`
   - Chamfer surface: `SSurface::FromPlane(A, B-A, D-A)`
4. **Add chamfer surface to shell** as new SSurface
5. **Create 2 new SCurves**:
   - `curve1` connects faceA to chamfer surface (at the d1 offset line)
   - `curve2` connects chamfer surface to faceB (at the d2 offset line)
6. **Update trim polygons of faceA and faceB**:
   - Remove the original shared SCurve from their trim lists
   - Add curve1 to faceA's trim list
   - Add curve2 to faceB's trim list
   - Add both curve1 and curve2 to chamfer surface's trim list
   - Add two line segments (cap curves at V1 and V2 ends) connecting curve1 to curve2
7. **Set face handle**: `chamferSurface.face = g->Remap(Entity::NO_ENTITY, REMAP_CHAMFER_FACE).v`

---

## 8. Complete Injection Point Summary

### Where to Call MakeFromChamferOf()

In `Group::GenerateShellAndMesh()`, after all the existing `else if` branches:

```cpp
// In groupmesh.cpp, inside Group::GenerateShellAndMesh():
} else if(type == Type::CHAMFER) {
    Group *src = SK.GetGroup(opA);
    double dist = SK.GetParam(h.param(0))->val;
    
    // Validate: opA must have a non-empty shell
    if(!src->runningShell.IsEmpty()) {
        // Direct topology injection — NOT a boolean operation
        thisShell.MakeFromChamferOf(&src->runningShell, this, dist);
    }
    
} else if(type == Type::FILLET) {
    Group *src = SK.GetGroup(opA);
    double radius = SK.GetParam(h.param(0))->val;
    
    if(!src->runningShell.IsEmpty()) {
        thisShell.MakeFromFilletOf(&src->runningShell, this, radius);
    }
}
```

Then in the `GenerateForBoolean` call, since `meshCombine == CombineAs::ASSEMBLE`
for chamfer/fillet groups, the call becomes `MakeFromAssemblyOf(prevs, &thisShell)`
which is just a concatenation. But since `thisShell` is already the FULL modified
shell, the result would be double the surfaces.

**Better approach:** Bypass `GenerateForBoolean` entirely for CHAMFER/FILLET:

```cpp
    // After generating thisShell/runningShell...
    if(type == Type::CHAMFER || type == Type::FILLET) {
        // Direct assignment — the chamfer/fillet shell IS the running mesh
        runningShell.MakeFromCopyOf(&thisShell);
    } else if(!IsForcedToMesh()) {
        SShell *prevs = &(prevg->runningShell);
        GenerateForBoolean<SShell>(prevs, &thisShell, &runningShell, srcg->meshCombine);
        ...
    }
```

---

## 9. MakeFromAssemblyOf() — Why It's Simpler than MakeFromBoolean()

### Location: `src/srf/boolean.cpp:860`

```cpp
void SShell::MakeFromAssemblyOf(SShell *a, SShell *b) {
    booleanFailed = false;
    // 1. Copy all curves from a and b, assigning new IDs
    for(SCurve &c : a->curve) { ... curve.AddAndAssignId(&cn); c.newH = ...; }
    for(SCurve &c : b->curve) { ... curve.AddAndAssignId(&cn); c.newH = ...; }
    
    // 2. Copy all surfaces from a and b, rewriting trim curve IDs
    for(SSurface &s : a->surface) {
        sn = SSurface::FromTransformationOf(&s, t, q, 1.0, includingTrims=true);
        for(stb = sn.trim.First(); ...) {
            stb->curve = a->curve.FindById(stb->curve)->newH; // rewrite IDs
        }
        s.newH = surface.AddAndAssignId(&sn);
    }
    // same for b->surface...
    
    // 3. Rewrite curve surface handles (surfA, surfB) to new IDs
    RewriteSurfaceHandlesForCurves(a, b);
}
```

This is simply concatenation with ID rewriting. No intersection computation, no
BSP classification, no boolean logic. This is the model for "merge two shells."

The **direct topology injection** approach for chamfer is essentially:
1. `MakeFromCopyOf(src)` to get all existing surfaces and curves with new IDs
2. Then surgically add new surfaces/curves and update existing trim lists

The ID rewriting issue is the hardest part — after copying, all handles are new,
so adding new surfaces requires careful handle tracking.

---

## 10. SShell::MakeFromCopyOf() — Starting Point for Chamfer Shell

### Location: `src/srf/shell.cpp`

```cpp
void SShell::MakeFromCopyOf(SShell *a) {
    ssassert(this != a, "Can't make from copy of self");
    MakeFromTransformationOf(a, Vector::From(0,0,0), Quaternion::IDENTITY, 1.0);
}

void SShell::MakeFromTransformationOf(SShell *a, Vector t, Quaternion q, double scale) {
    booleanFailed = false;
    surface.ReserveMore(a->surface.n);
    for(SSurface &s : a->surface) {
        SSurface n = SSurface::FromTransformationOf(&s, t, q, scale, includingTrims=true);
        surface.Add(&n);  // keeps the same ID as original!
    }
    curve.ReserveMore(a->curve.n);
    for(SCurve &c : a->curve) {
        SCurve n = SCurve::FromTransformationOf(&c, t, q, scale);
        curve.Add(&n);  // keeps the same ID!
    }
}
```

**Critical:** `MakeFromCopyOf` PRESERVES the original IDs (`surface.Add` not
`surface.AddAndAssignId`). This means after `MakeFromCopyOf`, all surface and
curve handles in `thisShell` are the SAME as in `src->runningShell`.

This is important for chamfer because:
- We can look up surfaces by their original face handles
- We can use original SCurve IDs to find the shared edge
- New SCurves we add will get new IDs via `curve.AddAndAssignId`

---

## 11. The Complete Data Flow for a CHAMFER Group

Here is the complete step-by-step data flow:

```
User Action:
  1. Select 2 faces (face0, face1) from a solid in group G_src
  2. New group G_chamfer is created:
     g.type = CHAMFER
     g.opA = G_src.h (source group)
     g.predef.entityB = face0 (face entity handle from G_src)
     g.predef.entityC = face1
     g.valA = 1.0 (default 1mm chamfer)

SS.GenerateAll() called:

1. Group::Generate(entity, param) for G_chamfer:
   → AddParam(param, h.param(0), valA=1.0)  // offset param
   → (no entity copies)
   → return

2. SS.solver solves: h.param(0) is unconstrained → stays at 1.0

3. Group::GenerateShellAndMesh() for G_chamfer:
   a. srcg = this (G_chamfer)
   b. thisShell.Clear(), runningShell.Clear()
   c. type == CHAMFER branch:
      dist = SK.GetParam(h.param(0))->val  // = 1.0
      src = SK.GetGroup(opA)               // = G_src
      srcShell = &src->runningShell        // the solid we're chamfering
      
      thisShell.MakeFromChamferOf(srcShell, this, dist):
        i.   Copy srcShell into thisShell (preserving IDs)
        ii.  Resolve face0 → surfaceA in thisShell
        iii. Resolve face1 → surfaceB in thisShell
        iv.  Find shared SCurve between surfA and surfB
        v.   Compute chamfer ABCD corners
        vi.  Create chamferSurface = SSurface::FromPlane(A, B-A, D-A)
        vii. Create curve1 (A→B line, between surfA and chamferSurface)
        viii.Create curve2 (D→C line, between chamferSurface and surfB)
        ix.  Create capL (A→D line, between chamferSurface and...)
             Create capR (B→C line, between chamferSurface and...)
        x.   Modify surfA.trim: remove old shared curve, add curve1
        xi.  Modify surfB.trim: remove old shared curve, add curve2
        xii. Set chamferSurface.trim = {curve1, curve2, capL, capR}
        xiii.Remove old shared SCurve from thisShell.curve

   d. Skip GenerateForBoolean (CHAMFER group goes direct to runningShell):
      runningShell.MakeFromCopyOf(&thisShell)
      
   e. booleanFailed = runningShell.booleanFailed
   f. displayDirty = true

4. Group::GenerateDisplayItems():
   → runningShell.TriangulateInto(&displayMesh)
   → chamferSurface (degree 1,1) → UvTriangulateInto (ear-clip, trivial)
   → displayOutlines computed for edges
```

---

## 12. Key Files and Exact Line Numbers for Chamfer Injection

### src/groupmesh.cpp

| Location | What to Add/Change |
|----------|-------------------|
| `IsMeshGroup()` (~line 545) | Add `case Type::CHAMFER:` and `case Type::FILLET:` returning true |
| `GenerateShellAndMesh()` (~line 212) | Add `else if(type == Type::CHAMFER)` branch after existing branches |
| `GenerateShellAndMesh()` (~line 380) | Add special case to bypass `GenerateForBoolean` for CHAMFER/FILLET |
| `Group::DrawMesh()` case `DrawMeshAs::SELECTED` | No changes needed — face selection already works |

### src/group.cpp

| Location | What to Add/Change |
|----------|-------------------|
| `Group::Generate()` switch (~line 340) | Add `case Type::CHAMFER:` and `case Type::FILLET:` |
| `Group::MenuGroup()` switch (~line 70) | Add `case Command::GROUP_CHAMFER:` and `case Command::GROUP_FILLET:` |
| `Group::GenerateEquations()` | No changes needed (no equations for chamfer/fillet) |

### src/srf/shell.cpp

| What to Add | Where |
|------------|-------|
| New method `SShell::MakeFromChamferOf(SShell *src, Group *g, double dist)` | New function after `MakeFromAssemblyOf` |
| New method `SShell::MakeFromFilletOf(SShell *src, Group *g, double radius)` | Same location |

### src/srf/surface.h

| What to Add | Where |
|------------|-------|
| Declarations for `MakeFromChamferOf` and `MakeFromFilletOf` | In SShell class declaration |

### src/sketch.h

| What to Add | Where |
|------------|-------|
| `CHAMFER = 5400` and `FILLET = 5401` in Group::Type enum (~line 181) | After LINKED |
| `REMAP_CHAMFER_FACE = 1011` and `REMAP_FILLET_FACE = 1012` in REMAP constants (~line 305) | After REMAP_LINE_TO_FACE |

---

## 13. Why Direct Injection is Better Than Boolean

The `MakeFromBoolean` pipeline involves:
1. Computing BSPs (classifying planes) for all surfaces of both shells
2. Computing intersection curves between all pairs of surfaces
3. Splitting all curves against all surfaces
4. Classifying each surface region as INSIDE/OUTSIDE/COINC_SAME/COINC_OPP
5. Trimming each surface to keep only the desired regions

For chamfer:
- The chamfer surface **shares edges** with the adjacent faces
- The adjacent faces are **coincident at the shared edge** 
- This causes `COINC_OPP` degeneracy in the boolean classifier
- **Result: boolean failure** (as confirmed by phkahler in PR #1501)

Direct topology injection avoids ALL of this:
- We know exactly which surfaces exist and which topology to create
- We construct the exact NURBS geometry closed-form
- We manually update only the affected trim polygons
- No intersection computation, no BSP classification, no boolean logic
- Watertightness is enforced by construction (manual trim list updates)

---

## 14. The Trim Polygon Update Challenge

This is the hardest algorithmic part, as identified by multiple sources:
- phkahler (PR #1501): "Trim polygon update is the hard part"
- Blender bevel documentation: "80% of complexity is the miter handling"

The specific challenge:

When we chamfer an edge, we need to:
1. **Replace a point on face A's boundary** (the original shared edge endpoint) 
   with a new line segment (the chamfer offset line on face A)
2. **Replace a point on face B's boundary** similarly
3. **Both modifications must remain consistent** (endpoints must match exactly)

The trim polygon is stored as:
```cpp
struct STrimBy {
    hSCurve curve;    // which curve trims this surface
    bool backwards;   // direction of traversal
    Vector start;     // xyz start point
    Vector finish;    // xyz finish point
};
```

The modification is:
- Find the STrimBy in surfaceA that references the old shared SCurve
- Replace it with a STrimBy referencing `curve1` (the d1 offset line)
- Remove the old shared SCurve from surfaceA.trim entirely
- Similarly update surfaceB.trim to reference `curve2` (the d2 offset line)

The start/finish endpoints of the trim must exactly match the control points
of the new curves, or `AssemblePolygon` will fail silently with a topology hole.

---

## Summary of Key Learnings

1. **`Generate()` for CHAMFER**: Only `AddParam(h.param(0), valA)` — no entity copies
2. **`GenerateShellAndMesh()` injection point**: After existing `else if` branches, before `MergeCoincidentSurfaces`
3. **The key new function**: `SShell::MakeFromChamferOf()` in `src/srf/shell.cpp` — modeled after `MakeFromExtrusionOf`
4. **Bypass GenerateForBoolean**: CHAMFER/FILLET groups set `runningShell` directly
5. **IsMeshGroup()** must return true for CHAMFER/FILLET
6. **MakeFromCopyOf preserves IDs**: Use this to get a mutable copy of the source shell
7. **Trim polygon update is the hardest part**: Must update surfA.trim and surfB.trim to remove old SCurve and add new curves
8. **Watertightness invariant**: Every SCurve must be in exactly 2 surfaces' trim lists
9. **Cap curves**: Need line segments at each end of the chamfer edge connecting the two offset lines
10. **Face handle**: New chamfer surfaces need `ss.face = Remap(NO_ENTITY, REMAP_CHAMFER_FACE).v`
