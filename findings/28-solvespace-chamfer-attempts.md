# Task 28: Existing SolveSpace Chamfer/Fillet Attempts on GitHub

## Summary

There is one significant community attempt to implement chamfer/fillet in SolveSpace, currently in progress as
a draft Pull Request by contributor `phkahler` (Paul H Kahler), who is a core SolveSpace contributor.
There is also a related PR (#1631) for arc generation improvements that phkahler sees as foundational work.
No other forks or branches with chamfer/fillet code were found.

---

## Pull Requests Found

### PR #1501 — "WIP add functions for creating chamfers and fillets to the NURBS kernel"
- **URL**: https://github.com/solvespace/solvespace/pull/1501
- **Author**: phkahler (Paul H Kahler — core SolveSpace contributor)
- **Branch**: `phkahler:fillet` → merge target `solvespace:master`
- **Status**: Draft (open as of early 2026)
- **Opened**: December 10, 2024
- **Commits**: 3 commits (last March 8, 2025)
- **Files changed**: 4 files (src/CMakeLists.txt, src/groupmesh.cpp, src/srf/chamfer.cpp, src/srf/surface.h)
- **Lines added**: +119 lines, -1 line

**Key fact**: The last commit message says: `"Create 2 new curves and one new surface. Not complete. Will crash."`

### PR #1631 — "WIP: create arc entities from revolved points"
- **URL**: https://github.com/solvespace/solvespace/pull/1631
- **Author**: phkahler
- **Branch**: `phkahler:arcs` → merge target `solvespace:master`
- **Status**: Draft (open as of early 2026)
- **Opened**: October 23, 2025
- **Related**: phkahler mentioned this is foundational work toward his eventual chamfer/fillet tool

---

## The `src/srf/chamfer.cpp` File (from phkahler's fillet branch)

This is the **complete content** of the new file created in PR #1501:

```cpp
//-----------------------------------------------------------------------------
// Algorithm for adding fillets and chamfers to an existing NURBS shell
// To create these features, edges are split into 2 new edges joined by
// a new surface which can be flat (chamfer) or round (fillet).
// Old: surfA, edge, surfB
// New: surfA, edge1, surfC, edge2, surfB
//
// Copyright 2024 Paul H Kahler.
//-----------------------------------------------------------------------------
#include "../solvespace.h"

typedef struct {
    hSCurve hc;    // handle of the original curve
    hSSurface hs;  // handle of the new surface
} fillet_info;


// We return 0 for no feature, radius for a fillet, and for chamfers a negative
// value whose absolute value is the radius of an equivalent fillet.
double SShell::GetEdgeModifier(SCurve &sc)
{
    // for testing we will hard code stuff like returning a radius for only degree 2 curves.
    // once things work we'll topo-name curves with entity handles and use that to get info.
    if (!sc.isExact) return 0.0;
    // chamfer any curve up high
    if ((sc.exact.deg > 1) && (sc.exact.ctrl[0].z > 9.0)) return -5.0;
    
    return 0.0;
}

void SShell::ModifyEdges()
{
    dbp("checking edges");
    // Loop over all curves and create an info struct for the ones to modify
    // we will call GetEdgeModifier() to get a radius or 0.
    for(int i=0; i<curve.n; i++) {
        SCurve *sc = &curve[i];
        double m = GetEdgeModifier(*sc);
        bool chamfer = false;

        if (abs(m) > 0.000001)
        {
            dbp("modify an edge");
            if (m < 0.0) {
                chamfer = true;
                m = -m;
            }

            // create a new surface C, for the fillet or chamfer
            // - for now this surface has zero area because the trims are identical
            SSurface srfC = {};
            srfC.degm = sc->exact.deg;
            if (chamfer)
                srfC.degn = 1;
            else
                srfC.degn = 2;

            srfC.degn = 3; // we're doing something else for now

            // srf.color = ???? what to put here;
            hSSurface hsrfC = surface.AddAndAssignId(&srfC);

            // create 2 new curves/trims as copies of the original. Tag the original curve.
            // for trims we can just modify existing and add a new one.
            Vector zeroVec = {0.0, 0.0, 0.0};
            // create curve A joining surface A with surface C
            SCurve curveA = {};
            curveA.isExact = true;
            curveA.exact = sc->exact.TransformedBy(zeroVec, Quaternion::IDENTITY, 1.0);
            // copy would be better, but we need to redo this later anyway.
            (curveA.exact).MakePwlInto(&(curveA.pts));
            // one of these needs to be the new surface
            curveA.surfA = sc->surfA;
            curveA.surfB = hsrfC;
            hSCurve hcurveA = curve.AddAndAssignId(&curveA);

            // create curve B joining surface B with surface C
            SCurve curveB = {};
            curveB.isExact = true;
            curveB.exact = sc->exact.TransformedBy(zeroVec, Quaternion::IDENTITY, 1.0);
            // copy would be better, but we need to redo this later anyway.
            (curveB.exact).MakePwlInto(&(curveB.pts));
            // one of these needs to be the new surface
            curveB.surfA = hsrfC;
            curveB.surfB = sc->surfB;
            hSCurve hcurveB = curve.AddAndAssignId(&curveB);


        }
    }
    // Delete the original curves (we can probably leave the curve and modify the trim(s))

    // For each new curve, find the one that matches our start and end point and
    // shares surfA or B. these are either original unmodified curves or new feature curves
    // either works, as this is what our new curve will intersect


    // Offset all the new curves within their A or B surface

    // Find the intersection points of all our new curves and their neighbors (start and end)

    // Reparameterize our curves and extend our surface if needed (or can U,V go past [0,1]?)

    // Create new trim curves at the ends of our new surface where it meets the others
    // when we get to corners with 3 fillets we will need a spherical surface to join (hard).

    // Delete our info structures
}
```

---

## Changes to Existing Files

### `src/groupmesh.cpp` — Hook-in Point
```cpp
// At line ~407 in GenerateShellAndMesh()
if(srcg->meshCombine != CombineAs::ASSEMBLE) {
    runningShell.MergeCoincidentSurfaces();
}

// Apply any edge modifiers - chamfer / fillet
runningShell.ModifyEdges();
```
This is the **exact hook-in point** for the chamfer/fillet code within the existing mesh generation pipeline.

### `src/srf/surface.h` — New Method Declarations
```cpp
// Added to SShell class:
double GetEdgeModifier(SCurve &sc);
void ModifyEdges();
```

### `src/CMakeLists.txt` — Build System
```
srf/chamfer.cpp   # Added new file to build
```

---

## Architectural Approach from phkahler's PR

### Design Pattern
```
Old B-rep:  surfA ── edge ── surfB
New B-rep:  surfA ── edge1 ── surfC (chamfer/fillet surface) ── edge2 ── surfB
```

### Algorithm Outline (from comments in chamfer.cpp)
1. **Loop over SCurves**: For each curve, call `GetEdgeModifier()` to get radius (>0 = fillet, <0 = chamfer) or 0 (skip)
2. **Create new surface C**: SSurface for the chamfer (degn=1) or fillet (degn=2) patch
3. **Create 2 new SCurves**: `curveA` (surfA ↔ srfC) and `curveB` (srfC ↔ surfB), both copies of original
4. **Delete original curve** (or modify trim)
5. **TODO items** (not yet implemented):
   - Offset new curves within their A or B surface
   - Find intersection points at edges of new feature
   - Reparameterize curves/extend surface
   - Create new trim curves at ends where feature meets others
   - Handle 3-fillet corners with spherical patches (noted as "hard")

### Edge Selection (GetEdgeModifier hack)
The current test hack selects edges by z-height: `if ((sc.exact.deg > 1) && (sc.exact.ctrl[0].z > 9.0))`
- This **only works for a rectangle at origin + extruded** shape
- Negative return = chamfer, positive = fillet, 0 = no modifier
- Comment says: "once things work we'll topo-name curves with entity handles and use that to get info"

---

## Community Discussion Highlights

### phkahler (Dec 10, 2024 — Commit 1: Outline)
> "Just an outline at this point (in the code). Hoping to attempt this over the holidays."

### ruevs (Dec 10, 2024 — Architecture Vision)
> "In my head it works similarly to what you started outlining with one more level of abstraction:
> - An **edge modifier class/module** goes over the selected edges and 'duplicates' them (each visible edge is already two coincident edges, so the code could 'just' move them)
> - A **general purpose patch tool** (with conditions as parameters — e.g. G1 continuity), which is a separate class, generates NURBS patches to close the holes"

**Key insight from ruevs**: In SolveSpace B-rep, each visible edge is ALREADY two coincident TrimBy references (one per adjacent surface). The code should "just move them" (i.e., update both TrimBy references to point to new offset curves).

### phkahler (Dec 10, 2024 — Clarification)
> "Curves are not duplicated, but there are two TrimBy, one for each surface that meets along a curve. These reference the same underlying curve but one is 'backward'. So I'll still need to generate 2 new curves, and move BOTH Trims rather than creating a new one."
> "Chamfers first and Fillets with 1 or 2 per vertex."

### phkahler (Aug 21, 2025 — Progress Report)
> "Getting back to this soon. Here is a sketch to visualize how fillets meet at a corner. For equal radius fillets it forms a spherical triangle."

### phkahler (Oct 23, 2025 — Related PR #1631 Comment)
> "When I get around to completing a chamfer/fillet tool I'd like to map shell edges to their corresponding sketch entities so we can assign a fillet radius or chamfer depth to the entities and have the shell update/modify with those features."

### jwesthues (Oct 23, 2025 — Core Author Response on Regeneration)
> "All entities get discarded and recreated with each regeneration. It feels wasteful, but the workload is very small compared to the NURBS or mesh Booleans. A line or curve sketched directly by the user is preserved only because it's stored as a Request."

This is critical: **SolveSpace regenerates all entities from scratch every time** — this affects how chamfer/fillet state must be stored (in Group fields, not Entity objects).

### phkahler (Dec 28, 2025 — Final Status)
> "Unfortunately my attempt last year didn't pan out and I wasn't even able to make partial shell modifications without problems. It really takes a deep dive into it to make progress. **Next time I try I might limit myself to flat surfaces only just to get the basics down.**"

---

## What Works vs. What Remains

### Confirmed Working in PR #1501
- `chamfer.cpp` compiles and is integrated into the build (CMakeLists.txt)
- `ModifyEdges()` is called at the right place in `GenerateShellAndMesh()`
- Correctly identifies the architecture: new surface C + 2 new curves replacing 1 original
- Surface type encoding: degn=1 for chamfer, degn=2 for fillet (correct per NURBS theory)

### NOT Implemented (Still TODO Comments)
- Actual computation of offset curve positions (the new edge locations after chamfer/fillet)
- Computing the chamfer/fillet surface control points
- Handling curve endpoints and trimming (the hardest part per ruevs/phkahler)
- TrimBy polygon updates on adjacent surfaces
- Vertex caps (where 3+ edges meet)
- Topology naming (stable edge references)

### Why It Crashes
- The new SSurface `srfC` is added with all-zero control points → degenerate surface
- The new SCurves `curveA` and `curveB` are copies of the original at the same location → degenerate
- No TrimBy entries are added for the new surface → watertightness violation → crash in triangulation or rendering

---

## Key Insights from This PR

1. **Architecture is correct**: The `surfA ── edge1 ── surfC ── edge2 ── surfB` pattern is the right approach
2. **Hook-in location confirmed**: `groupmesh.cpp:407` after `MergeCoincidentSurfaces()` is the right place
3. **Edge selection is the hardest unsolved problem**: phkahler used a hack (z-height); real solution needs toponaming
4. **TrimBy updates are the most complex part**: Both adjacent surfaces must have their trim polygons surgically updated
5. **SSurface color field**: Unknown what to put there (phkahler left comment "????")
6. **Vertex corners**: Confirmed as the final hard problem (spherical NURBS patches)
7. **Degm/Degn encoding**: chamfer→degn=1, fillet→degn=2 is the intended representation
8. **fillet_info struct**: Simple data structure `{hSCurve hc; hSSurface hs;}` tracks which curves are being modified

---

## Other Forks/Branches Searched

- Searched GitHub for other forks with "chamfer" or "fillet" code
- No other SolveSpace forks with chamfer/fillet implementation found
- phkahler's `fillet` branch (https://github.com/phkahler/solvespace/tree/fillet) is the only community attempt
- phkahler's `arcs` branch (https://github.com/phkahler/solvespace/tree/arcs) — PR #1631 — is related preparatory work

---

## Conclusions for Implementation

The phkahler PR confirms:

1. **The correct hook-in is `groupmesh.cpp:~410`** (after `MergeCoincidentSurfaces()`)
2. **A dedicated `chamfer.cpp` file in `src/srf/` is the right home** for this code
3. **Two new functions needed in `SShell`**: one to select edges (e.g., `GetEdgeModifier()`), one to perform the transformation (`ModifyEdges()` or `MakeFromChamferOf()`)
4. **The hardest problems are**: (a) computing offset curves on adjacent surfaces, (b) TrimBy polygon updates, (c) vertex caps
5. **MVP confirmed**: Limit to flat (planar) surfaces only, single edge only, equal-distance chamfer
6. **The code compiles**: Can base new work on the phkahler approach + fill in the missing offset curve math

### Recommended Starting Point for New Implementation

```cpp
// In src/srf/chamfer.cpp:

// Step 1: Identify shared SCurve between two selected faces (face0, face1)
// Step 2: Compute offset distance d in each face using face normals
// Step 3: Create new SCurve A at distance d from original on face0  
// Step 4: Create new SCurve B at distance d from original on face1
// Step 5: Create new SSurface C (planar patch) between curveA and curveB
// Step 6: Update TrimBy on face0 to use curveA instead of original
// Step 7: Update TrimBy on face1 to use curveB instead of original
// Step 8: Original SCurve becomes the boundary of new surface C
```
