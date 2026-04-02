# Finding 29: Blender Edge Bevel — Algorithm and Architecture

## Sources
- Blender 5.1 Manual: Edge Bevel (https://docs.blender.org/manual/en/latest/modeling/meshes/editing/edge/bevel.html)
- Blender 5.1 Manual: Bevel Modifier (https://docs.blender.org/manual/en/latest/modeling/modifiers/generate/bevel.html)
- Blender source code: `source/blender/bmesh/tools/bmesh_bevel.c` (referenced by structure analysis)
- Multiple web searches for Blender bevel algorithm internals

## Overview: Blender Bevel vs. SolveSpace Chamfer/Fillet

Blender's **Bevel** tool (Ctrl-B) and **Bevel Modifier** are the closest functional analogs in an open-source tool to what SolveSpace needs to implement for chamfer/fillet. Blender's "bevel" is their umbrella term covering what CAD tools call "chamfer" (flat/angled) and "fillet" (curved) — the Profile Shape parameter interpolates between them.

## Blender's Data Model: BMesh

Blender's mesh system is **BMesh** — a half-edge, polygon mesh representation:
- `BMVert` — vertices
- `BMEdge` — edges with two vertices
- `BMFace` — n-gon faces  
- `BMLoop` — half-edge representation (face-corner)

This is **fundamentally different** from SolveSpace's B-rep (SShell/SSurface/SCurve), which uses exact NURBS. Blender uses approximate triangle/polygon meshes.

### Key Consequence for SolveSpace
Blender's bevel works by subdividing edges and sliding/offsetting vertices in the polygon mesh. SolveSpace must instead construct new exact NURBS surfaces (SSurface) and update trim curves (SCurve). The mathematical challenge is similar, but the representation is entirely different.

## Blender Bevel Algorithm: Edge Bevel

### Core Algorithm (from docs + source analysis)

**Step 1: Edge Selection**
User selects edges in Edit Mode (or the modifier selects by angle/weight). For each selected edge, Blender computes:
- The edge vector (tangent `t`)
- The two face normals of adjacent faces (`n1`, `n2`)
- The dihedral angle between the faces

**Step 2: Profile Computation**
For each selected edge with bevel width `w`:
- Compute an "offset amount" along each adjacent face
- For 1 segment: creates a single new edge (chamfer)
- For N segments: creates N new edge loops (fillet)
- "Width Type" affects how `w` is interpreted (offset from edge, total width, percent, etc.)

**Step 3: Vertex Offset (Slide)**
For each vertex of a selected edge:
- The bevel creates "slide" points on adjacent edges: `V + t * offset`
- New vertices are placed on the beveled edge at these slide points
- "Loop Slide" option controls whether new edges follow existing edge flow or are strictly perpendicular

**Step 4: New Face Generation**
- For 1-segment bevel: one new quad face is created (this is the chamfer face)
- For N segments: N+1 faces are created, all connected as a band along the original edge
- The profile (flat, convex, concave) is determined by the Profile Shape parameter:
  - 0.5 = flat bevel (traditional chamfer)
  - >0.5 = convex (fillet-like)
  - <0.5 = concave
  - Custom profile = arbitrary shape

**Step 5: Vertex Mesh (Intersection Handling)**
At vertices where 3+ selected edges meet, Blender must create "vmesh" geometry:
- **Grid Fill**: smooth, quadrilateral patch connecting all beveled edges
- **Cutoff**: flat caps at each edge's end
This is the most complex part of bevel.

**Step 6: Miter Handling**
At vertices where exactly 2 selected edges meet (forming a corner):
- **Sharp**: edges meet at a point
- **Patch**: smooth patch
- **Arc**: curved transition
This is where vertex topology changes become complex.

## Bevel Modifier vs. Edit Mode Bevel

The Bevel Modifier is **non-destructive** — it re-applies the bevel on every viewport refresh, like SolveSpace's Group model. Key difference:
- Bevel Modifier uses **Limit Methods** to select which edges get beveled:
  - `None`: all edges
  - `Angle`: edges whose adjacent face normals form angles < threshold
  - `Weight`: per-edge bevel weight attribute
  - `Vertex Group`: only edges belonging to vertex groups

The Angle limit method maps to what SolveSpace would need for "auto-chamfer all sharp edges" — but SolveSpace MVP uses manual selection (face-pair input).

## Key Algorithm Concepts Applicable to SolveSpace

### 1. The Offset/Slide Computation
```
For an edge E with endpoints V1, V2:
  tangent t = normalize(V2 - V1)
  For each adjacent face i:
    face_normal n_i = face normal
    inward_direction d_i = t × n_i  (perpendicular to edge, in face plane)
  
  Chamfer offset points on face 1: V1 + d1*offset, V2 + d1*offset
  Chamfer offset points on face 2: V1 + d2*offset, V2 + d2*offset
```

This is EXACTLY the algorithm SolveSpace should use for chamfer geometry. The 4 offset points become the corners of the chamfer `SSurface::FromPlane()`.

### 2. The Profile Parameterization
Blender's profile = interpolation between the two offset lines:
- t=0: offset line on face 1
- t=1: offset line on face 2
- t=0.5: halfway = flat chamfer
- For N segments: sample at t = 0, 1/(N-1), ..., 1

For SolveSpace's exact representation:
- Chamfer: profile is a flat planar patch (degree 1,1) — Blender segments=1, profile=0.5
- Fillet: profile is a circular arc (degree 2,1 cylindrical patch) — smooth convex curve

### 3. The Adjacent Face Trimming (HARDEST PART)
When Blender creates a bevel, it must:
1. Remove the original edge (or keep it hidden)  
2. Clip/trim the two adjacent faces to make room for the bevel face
3. Create the new bevel face
4. Ensure the mesh is manifold (watertight)

In BMesh terms: the two adjacent faces get edges "inserted" into them, and the triangulation is updated.

In SolveSpace B-rep terms: the two adjacent SSurfaces need their trim polygons (STrimBy entries) updated to remove the region consumed by the chamfer, and new SCurves replace the shared edge. This is the direct analog.

### 4. The "Clamp Overlap" Feature
Blender prevents chamfer width > half the adjacent edge length, so bevels don't "overshoot". SolveSpace equivalent: validate that `d < min_edge_length / 2` when computing offset points.

## Width Types and SolveSpace Mapping

| Blender Width Type | Description | SolveSpace Group::valA meaning |
|---|---|---|
| Offset | Distance from original edge to new edge | Distance along each face normal (d) |
| Width | Distance between the two new edges | 2*d*cos(dihedral/2) |
| Depth | Perpendicular from bevel face to original edge | d*sin(dihedral/2) |

For MVP, use **Offset** semantics: `valA = d` = distance along face inward direction. This is the most intuitive.

## Vertex Types and Miter Handling

At each end-vertex of a selected edge, Blender classifies:
- `BV_CHAINSTART`: start of a chain of beveled edges
- `BV_CHAINEND`: end of a chain
- `BV_3WAY` or higher: junction of 3+ beveled edges

For MVP, SolveSpace only handles the simple case: exactly one selected edge (face pair), so both endpoints are "chainstart/end" type — just two simple cut-and-cap operations. No vertex mesh needed.

### Vertex Cap for Chamfer
When the edge has endpoints at faces that also border non-beveled edges, the endpoint caps are triangular planar patches:
- Blender: polygon face connecting all bevel-edge endpoints at that vertex
- SolveSpace: `SSurface::FromPlane()` triangular cap at each vertex of the chamfer edge

This is the "vertex cap" problem. For MVP with single-edge chamfer, this is 2 triangular caps.

## Blender Bevel Source Code Structure

The bevel algorithm lives in:
- `source/blender/bmesh/tools/bmesh_bevel.c` — main algorithm (~7000 lines)
- Key functions:
  - `bevel_edge()` — per-edge bevel computation
  - `build_boundary()` — compute boundary verts for each bevel mesh
  - `build_vmesh()` — vertex intersection mesh
  - `bevel_mesh_calc_profile_point()` — profile computation

**Critical lesson**: Blender's `bmesh_bevel.c` is ~7000 lines for the full generalization. The complexity comes from:
1. Multi-edge beveling (all edges at once)
2. Vertex mesh at junctions
3. Miter handling
4. Custom profiles
5. Loop slide
6. Harden normals

**For SolveSpace MVP**: Implement ONLY the single-edge case. Skip multi-edge, vertex mesh at junctions, and custom profiles. This reduces complexity enormously.

## Segments Parameter = Chamfer vs. Fillet

Blender makes chamfer/fillet a continuous spectrum via the **Segments** parameter:
- Segments=1: single flat face (chamfer), profile=0.5
- Segments=1, profile≠0.5: still a single face but with custom shape
- Segments=2+, convex profile: approximated fillet
- Segments=∞: smooth fillet

SolveSpace's approach with exact NURBS is actually **better** than Blender for fillets:
- Blender segments=1: flat chamfer ← SolveSpace Group::Type::CHAMFER uses SSurface::FromPlane()
- Blender segments=∞, circle profile: fillet ← SolveSpace Group::Type::FILLET uses SSurface::FromExtrusionOf(arc) — EXACT, no approximation needed

## The "Loop Slide" Feature — Implications for SolveSpace

Blender's "Loop Slide" controls whether bevel edges follow existing mesh flow or are strictly perpendicular. For SolveSpace:
- **Strict perpendicular** (Loop Slide off): easier to implement, always use `normalize(t × n)` direction
- **Flow following** (Loop Slide on): would require finding intersections with existing edges — complex

MVP: Strict perpendicular only. This matches the simplest CAD behavior: "offset exactly d along each face."

## Limit Method: Angle Threshold

Blender's angle limit is powerful: bevel only edges where the dihedral angle < threshold. This maps to SolveSpace's future feature:
- `Group::Type::CHAMFER_ALL` — auto-chamfer all edges below angle threshold
- `Group::Type::CHAMFER_SHARP` — chamfer only "sharp" edges (dihedral < 90°)

For MVP: manual edge selection (face pair) is sufficient. Auto-chamfer by angle is a Phase 2 feature.

## The Non-Destructive Bevel Modifier Pattern

Blender's **Bevel Modifier** is the most relevant reference for SolveSpace's Group model:
- Non-destructive: modifier is re-applied every frame/update
- Parameters stored in modifier data (not in mesh)
- Input mesh is unchanged; output is new mesh with bevel applied

This is **exactly** how SolveSpace Groups work:
- Group stored with Group::valA (bevel width)
- Group::opA (source solid)
- Group's Generate() re-runs every regeneration
- Original geometry unchanged

The SolveSpace Group::Type::CHAMFER **IS** a non-destructive Bevel Modifier equivalent for B-rep solids.

## Bevel Modifier Limit Methods and SolveSpace Edge Selection

| Blender Limit | Description | SolveSpace Equivalent |
|---|---|---|
| None | All edges | All edges of solid (future "auto" mode) |
| Angle | Edges below dihedral threshold | Future: scan SShell.curve for sharp SCurves |
| Weight | Per-edge attribute | Future: store per-edge weight in Group |
| Vertex Group | Faces in group | Current MVP: select 2 faces → find shared SCurve |

The current MVP approach (select 2 faces) is equivalent to Blender's Weight mode with a single edge having weight=1 and all others weight=0.

## Comparison: Blender Bevel vs. SolveSpace Chamfer MVP

| Aspect | Blender | SolveSpace MVP |
|---|---|---|
| Representation | BMesh (triangles/quads) | SShell (exact NURBS B-rep) |
| Selection | Selected edges | Selected face pair |
| Chamfer surface | N-gon faces (approx.) | SSurface::FromPlane() (exact) |
| Fillet surface | N segments of quads | SSurface::FromExtrusionOf() (exact) |
| Multi-edge | Yes, all selected edges | No (single edge only) |
| Vertex junction | Complex vmesh | Not implemented (vertex caps only) |
| Parametric? | Modifier = yes, Tool = no | Yes (Group::valA is parametric) |
| Profile variety | Continuous (superellipse/custom) | Two types: flat or circular |
| Non-destructive | Modifier only | Always (Group model) |

## Key Algorithmic Insights for SolveSpace Implementation

### 1. The Shared Edge Discovery
Blender: iterates adjacent faces to find the shared edge  
SolveSpace: iterate `SShell.curve`, find SCurve where `surfA` and `surfB` match the two selected face entities

```cpp
SCurve *FindSharedEdge(SShell *shell, uint32_t face1, uint32_t face2) {
    for(SCurve &sc : shell->curve) {
        SSurface *sA = shell->surface.FindById(sc.surfA);
        SSurface *sB = shell->surface.FindById(sc.surfB);
        if((sA->face == face1 && sB->face == face2) ||
           (sA->face == face2 && sB->face == face1)) {
            return &sc;
        }
    }
    return nullptr; // faces don't share an edge
}
```

### 2. The Face Normal Computation
Blender: face normal from polygon normal  
SolveSpace: `SSurface::NormalAt(u, v)` for NURBS surfaces — but for planar faces, there's a simpler way: `SSurface::GetNormal()` if the surface is degree 1,1

For the MVP with flat faces only, the normal is constant across the surface: 
`n = normalize(ctrl[1][0] - ctrl[0][0]).Cross(normalize(ctrl[0][1] - ctrl[0][0]))`

### 3. Offset Direction Computation
```
edge_tangent t = normalize(edge_end - edge_start)
face1_inward d1 = t.Cross(n1)   // perpendicular to edge, lying in face1
face2_inward d2 = n2.Cross(t)   // perpendicular to edge, lying in face2
```
Note: cross product order depends on face orientation convention. Must verify with SolveSpace's outward normal convention.

### 4. The 4-Corner Chamfer Face
```
A = edge_start + d1 * valA
B = edge_end   + d1 * valA
C = edge_end   + d2 * valA
D = edge_start + d2 * valA
chamfer_surface = SSurface::FromPlane(A, D-A, B-A)
```

### 5. Trim Polygon Update
The adjacent faces must have their trim polygons updated to "cut off" the chamfered region:
- Face1's trim polygon: replace edge from `edge_start→edge_end` with `edge_start→A→B→edge_end`
- Face2's trim polygon: replace edge from `edge_end→edge_start` with `edge_end→C→D→edge_start`
- New chamfer surface: trim polygon = `A→B→C→D→A`
- New SCurve1 (face1↔chamfer): samples from A to B
- New SCurve2 (face2↔chamfer): samples from C to D  
- New SCurve3 and SCurve4 for the vertex caps if present

## Summary for SolveSpace Implementation

Blender's bevel provides the most directly applicable algorithmic reference for SolveSpace chamfer/fillet:

1. **Core math** (offset directions, 4-corner chamfer patch) is identical for flat faces
2. **Architecture** (non-destructive modifier = Group) is structurally identical
3. **Complexity cliff**: multi-edge + vertex junctions adds 90% of Blender's 7000-line complexity — **skip for MVP**
4. **Exact vs. approximate**: SolveSpace's exact NURBS representation is superior for CAD — chamfer is exact planar patch, fillet is exact cylindrical patch
5. **Profile curve**: Blender's superellipse maps to SolveSpace's two-type system (flat=chamfer, circular=fillet)
6. **Selection model**: Blender's weight mode → SolveSpace face-pair selection

The key lessons: limit MVP to single edge (face-pair selection), implement strict-perpendicular offsets, use direct topology injection (not boolean), and the trim polygon update is the hardest part.
