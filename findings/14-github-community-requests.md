# Task 14: SolveSpace GitHub Issues/PRs/Discussions about Chamfer/Fillet

## Summary

The SolveSpace community has been requesting chamfer and fillet features for nearly a decade. There is a clear, long-standing demand, and an active (though stalled) implementation attempt by a core contributor. This is well-known "commonly requested" functionality.

---

## Issues Found (Open)

### #149 — "make chamfers as easy as fillets" (Open since Jan 8, 2017)
**URL:** https://github.com/solvespace/solvespace/issues/149
**Author:** wpwrak
**Labels:** UI, documentation, enhancement, internals

**Body:**
> Right now, it is easy to draw a basic geometry that represents the core function of a piece, and then add fillets for strength, looks, etc., using tangent arcs. For chamfers, there doesn't seem to be an easy way to do the same, i.e., to rapidly apply them as refinements. It would be nice if such a function could be added.

**Key insight:** This issue is specifically about 2D sketch chamfers (making chamfers as easy as the existing tangent-arc fillet). The labels include `internals` and `UI`, indicating it requires both internal geometry support and UI work. It has been open for over 8 years.

**Linked discussion in comments (phkahler, May 2022):** The comments at https://github.com/solvespace/solvespace/issues/149#issuecomment-1112533446 contain the key insight: "what edge do you want to modify?" — this question became the seed for issue #1236 on Topological Naming.

---

### #577 — "Add chamfer and fillet tools" (Closed Mar 31, 2020, not implemented)
**URL:** https://github.com/solvespace/solvespace/issues/577
**Author:** Timmmm
**Labels:** enhancement, NURBS, UI
**Status:** Closed (not because it was implemented — appears to have been closed by bot/maintainer as duplicate or no-progress)

**Body:**
> It should be possible to select an arbitrary set of edges in 3D, and then click "chamfer/fillet", and have it nicely modified. Sometimes you can do this in a sketch with arcs, but often it is very tedious or impossible.
>
> Most professional CAD tools include these as basic operations. For some reason they are usually separate tools but I think it makes more sense to have it as one tool with a "chamfer style" option which is "straight" or "rounded" (and a radius option).

**Key insight:** User desires a single unified tool with "chamfer style" (straight vs. rounded). This suggests the community would accept a combined `Group::CHAMFER_OR_FILLET` with a mode flag rather than two separate Group types. Labels include `NURBS` explicitly — maintainers tagged it correctly.

---

### #1024 — "New feature idea? Missing functionality? Look here first." (Open since Apr 22, 2021)
**URL:** https://github.com/solvespace/solvespace/issues/1024
**Author:** ruevs (maintainer)
**Labels:** documentation, question
**Status:** Open (pinned index issue)

**Relevant excerpt:**
```
Construction tools, solids etc.
  - Chamfer, fillet
    make chamfers as easy as fillets #149
    Add chamfer and fillet tools #577
```

**Key insight:** ruevs (the main maintainer) has explicitly listed chamfer/fillet as a commonly requested feature in the "look here first" index issue. This is the official acknowledgement that this is wanted.

---

### #1208 — "NURBS solid fillet on cone cut fails" (Open since Feb 2, 2022)
**URL:** https://github.com/solvespace/solvespace/issues/1208
**Author:** ghost (deleted account)
**Labels:** NURBS
**Status:** Open

**Body:**
> Expected: There should not be fails in NURBS fillet on cone cut edge.
> Actual: NURBS solid fails.
> (includes test .slvs file: slvs3x-cone_fillet.zip)

**Key insight:** This is a BUG REPORT about an attempt to simulate a fillet by boolean difference with an arc-revolved solid. Even before a dedicated fillet tool exists, users are trying to create fillets manually via revolve + boolean difference, and it fails due to NURBS boolean bugs. This confirms:
1. The boolean pipeline has known bugs when curves are tangent to faces
2. Any fillet implementation will need to fix these bugs too

---

### #1236 — "Improve Remap Functionality - Topological Naming" (Open since May 3, 2022)
**URL:** https://github.com/solvespace/solvespace/issues/1236
**Author:** phkahler (core contributor)
**Labels:** internals
**Milestone:** Breaking Changes

**Body (key excerpts):**
> Surfaces and edges of the solid model should have IDs that are persistent across regeneration.

> See several comments starting here (https://github.com/solvespace/solvespace/issues/149#issuecomment-1112533446) in a discussion about chamfers. i.e. what edge do you want to modify?

> [This writing about FreeCAD](https://github.com/realthunder/asm3-wiki/blob/master/Topological-Naming.md) addressing the issue.

**Key insight:** phkahler explicitly links the need for topological naming to chamfer/fillet. The core problem is: **how do you stably identify which edge to chamfer/fillet as the model changes?** This is the same "Topological Naming Problem" that has plagued FreeCAD for years. phkahler is aware of it and considers it a prerequisite (or at least a serious consideration) for any chamfer/fillet implementation.

**Milestone "Breaking Changes":** Fixing topological naming would require breaking changes to existing .slvs files. This is a serious obstacle — fixing it properly could break backward compatibility.

---

### #1291 — "Rounded corner cut (fillet) issue" (Open since Sep 6, 2022)
**URL:** https://github.com/solvespace/solvespace/issues/1291
**Author:** phkahler (core contributor)
**Labels:** NURBS

**Body (key excerpts):**
> It should be possible to create a fillet by using a difference operation as shown: [cube_cut.zip]

> When the corner point and P1 are coincident with the top cube edge the NURBS fail. Having just one of them on the cube edge is not enough to cause the problem.

> The simplest workaround for creating a fillet by cutting an arc is to place the corner point further away from the cube.

> This does not seem like a numerical issue but more like a logic issue in determining which curves and surfaces to keep during the boolean operation. It is also notable that there is no failure if we switch from difference to an intersection boolean... So somewhere in here: boolean.cpp, merge.cpp

**Key insight:** phkahler was experimenting with manual fillet creation (using a revolved arc + difference) and identified that when the arc profile's corner point is exactly on the face edge, the boolean fails. This is in `boolean.cpp` / `merge.cpp`. This bug **must be fixed** before even a manual fillet via boolean difference works reliably, let alone a proper built-in fillet group.

---

### #1305 — "Surface experiments" (Open since Oct 27, 2022)
**URL:** https://github.com/solvespace/solvespace/issues/1305
**Author:** BKLronin
**Labels:** NURBS

**Relevant context:** This is an exploratory issue about surface construction in SolveSpace. It's adjacent to the fillet work since fillet surfaces are NURBS surfaces.

---

## Pull Request Found

### #1501 — "WIP add functions for creating chamfers and fillets to the NURBS kernel" (Draft PR, Dec 10, 2024)
**URL:** https://github.com/solvespace/solvespace/pull/1501
**Author:** phkahler (core contributor)
**Status:** DRAFT (not ready)
**Branch:** phkahler:fillet → solvespace:master

**3 commits:**
1. "Comments showing the outline of a new edge modifier function for creating chamfers and fillets. The function is hooked in Group::Generate but does not modify the shell at this point so it is benign." (commit 5d5950f)
2. "add a hack to reliably identify one edge to modify. assumes a rectangle at the origin and extruded." (commit e921626)
3. "Create 2 new curves and one new surface. Not complete. Will crash." (commit 8c1f978)

**phkahler's initial comment (Dec 10, 2024):**
> Just an outline at this point (in the code). Hoping to attempt this over the holidays. Thought people might want to follow along or comment if this goes anywhere.

**ruevs's comment (Dec 10, 2024) — architecture proposal:**
> A chamfer/fillet is:
> - An edge modifier class/module/cpp_file goes over the selected edges and **duplicates them** (in reality each visible edge is already two coincident edges so the code could "just" move them) — this is specific to the fillet/chamfer tool.
> - A general purpose patch tool (with conditions as parameters — e.g. G1 continuity), which is a separate class, generates NURBS patches to close the holes between the edges created in the previous step.
>
> In the future the patch class can be used to implement generic patch functionality — e.g. close a "random" hole outlined by a contour of any edges (not necessarily straight) in 3d.

**phkahler's reply (Dec 10, 2024):**
> Curves are not duplicated, but there are two TrimBy, one for each surface that meets along a curve. These reference the same underlying curve but one is "backward". So I'll still need to generate 2 new curves, and move **both** Trims rather than creating a new one. Good news — the PWL points are not in the trims!
>
> Assuming all fillets meeting at a point are the same radius, the surface we want there is spherical. It's really hard to find anything (with google) on spherical NURBS patches, never mind triangular ones. But that's for later. Chamfers first and Fillets with 1 or 2 per vertex. I'm both excited and dreading this...

**phkahler's update (Aug 21, 2025):**
> Getting back to this soon. Here is a sketch to visualize how fillets meet at a corner. Drag the green lines around. [corner_arcs.zip]
>
> For equal radius fillets it forms a spherical triangle.

**phkahler's comment (Dec 28, 2025):**
> Haha brave is right. Anyone is welcome to attempt this, and I'm happy to answer questions on how the internals work. Such a person would become highly knowledgeable in the solvespace NURBS code. Unfortunately my attempt last year didn't pan out and I wasn't even able to make partial shell modifications without problems. It really takes a deep dive into it to make progress. **Next time I try I might limit myself to flat surfaces only just to get the basics down.**

**Key technical insights from PR #1501 discussion:**

1. **Hook-in point confirmed:** The implementation is hooked into `Group::Generate` (src/group.cpp).
2. **Curve duplication approach:** When chamfering/filleting an edge, you need to generate 2 new boundary curves (one per adjacent surface), then move the TrimBy references in both surfaces to point to these new curves instead of the original.
3. **Spherical corner patches are hard:** For vertices where 3+ fillets meet, you'd need a spherical NURBS triangular patch. No good resources exist.
4. **Pragmatic advice from phkahler:** Start with flat surfaces only (chamfer between two flat faces).
5. **Status as of Dec 2025:** phkahler's attempt "didn't pan out" and "wasn't even able to make partial shell modifications without problems."

---

## Related Linked Issues

### #1642 — "Eliminate duplicate entities from extrusions" (Open Dec 2, 2025)
**URL:** https://github.com/solvespace/solvespace/issues/1642
**Author:** phkahler
**Labels:** enhancement, internals

This issue shows phkahler is still actively working on the NURBS kernel and understanding edge/surface topology. The work done here (fixing duplicate entities in extrusions) is preparatory to chamfer/fillet work.

---

## Community Sentiment Analysis

### Feature Priority
- Chamfer/fillet appears in the official "commonly requested features" index (#1024)
- Multiple independent users have filed requests spanning 2017–2020
- The feature is labeled with [UI, internals, NURBS, enhancement] — cross-cutting
- 8 ❤️ reactions on PR #1501 from multiple community members including maintainer ruevs

### Technical Consensus
From the issue discussions, the community has converged on these technical points:
1. **Start with chamfer, not fillet** — chamfer (planar surface) is simpler than fillet (cylindrical surface)
2. **Start with flat face pairs** — corner vertices meeting at 90° are simplest case
3. **The edge representation challenge:** Each visible edge has TWO TrimBy entries (one per adjacent surface). Chamfer/fillet requires generating 2 new curves and updating both TrimBy entries.
4. **Spherical corner patches are the hardest part** — for 3+ edge vertices, all equal-radius fillets meeting at a point form a spherical triangle, which requires a spherical NURBS patch.
5. **Boolean approach is viable but buggy** — using revolve+difference to simulate a fillet is possible but fails when the arc profile touches the face boundary (boolean.cpp/merge.cpp bugs, #1291).

### What Has NOT Been Attempted
- No merged code exists in master
- No public fork has a working implementation
- The phkahler:fillet branch (PR #1501) has only 3 commits, last of which says "Will crash"
- Topological naming (#1236) has not been addressed and is a known blocker for stable edge selection

---

## Key Conclusions for Implementation Planning

1. **phkahler is the most knowledgeable person** about SolveSpace internals and has attempted this. He explicitly says:
   - Start with flat surfaces
   - The hardest part is modifying the shell topology (two TrimBy updates per edge)
   - Corner vertices with 3+ fillets require spherical NURBS patches (very hard)
   - He offered to help anyone else who attempts this

2. **The topological naming problem** (#1236) is a known blocker:
   - How do you stably identify which edge to chamfer after model changes?
   - This requires either a simple heuristic (edge index, which is fragile) or a proper topological naming system (complex, breaking change)
   - For an MVP, a heuristic approach (store the selected SCurve index at creation time) may be acceptable with the caveat that it breaks if edges are reordered

3. **The boolean pipeline has known bugs** (#1291):
   - Creating fillets via manual difference operations fails when the arc profile is tangent to face edges
   - A proper fillet implementation via direct topology injection avoids this issue (doesn't use the boolean pipeline)
   - But a chamfer implementation using a thin wedge solid + difference boolean WOULD hit this bug

4. **UI consensus from community:**
   - Users want to select edges and apply chamfer/fillet in one action
   - A single tool with a "style" parameter (chamfer vs. fillet) would be acceptable
   - The current tangent-arc approach for 2D fillets is adequate for 2D sketches; the need is specifically for **3D solid chamfer/fillet**

5. **Active development momentum (late 2025):**
   - phkahler is back working on related issues (#1642, #1631 arc entities from revolved points)
   - The fillet branch exists and the hook-in to Group::Generate is done
   - PR #1501 is draft but the scaffolding exists

---

## Raw Issue List (All Chamfer/Fillet Related)

| Issue # | Title | Status | Date | Author |
|---------|-------|--------|------|--------|
| #149 | make chamfers as easy as fillets | Open | Jan 2017 | wpwrak |
| #577 | Add chamfer and fillet tools | Closed | Mar 2020 | Timmmm |
| #1208 | NURBS solid fillet on cone cut fails | Open | Feb 2022 | ghost |
| #1236 | Improve Remap Functionality - Topological Naming | Open | May 2022 | phkahler |
| #1291 | Rounded corner cut (fillet) issue | Open | Sep 2022 | phkahler |
| #1305 | Surface experiments | Open | Oct 2022 | BKLronin |
| #1501 | WIP add functions for creating chamfers and fillets to the NURBS kernel | Draft PR | Dec 2024 | phkahler |

---

## Conclusion

The SolveSpace community has clearly wanted chamfer/fillet since at least 2017. The feature is officially listed as "commonly requested" by maintainer ruevs. The core contributor phkahler has attempted an implementation (PR #1501, Dec 2024), identified the key algorithmic challenges, and offered architectural guidance. His final advice: "Next time I try I might limit myself to flat surfaces only just to get the basics down." This suggests the MVP strategy should be:

1. **MVP: Chamfer between two flat faces** — new Group::CHAMFER with direct topology injection
2. **Extension 1: Fillet between two flat faces** — new Group::FILLET with cylindrical NURBS surface
3. **Extension 2: Curved face pairs** — general case (hard, may require canal surfaces)
4. **Extension 3: Vertex caps for 3+ edge meetings** — spherical patches (very hard)

The topological naming problem must be addressed at some level — at minimum, a documented limitation that edge selection is by index and may break if upstream geometry changes.
