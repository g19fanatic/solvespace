# Chamfer & Fillet — End-Cap Geometry: Math, Decisions, and Implementation

> **Purpose**: Exhaustive documentation of ALL geometry math and design decisions related
> to chamfer and fillet end-cap surfaces — the geometry at the V1 and V2 endpoints of a
> chamfered/filleted edge.  Covers cap detection, arc construction, cap trim updates, corner
> post-processing, the CF (chamfer→fillet) endcap fix, and the rationale behind each approach.
>
> **Cross-references**: [chamfer-fillet.md](chamfer-fillet.md) | [../context-strategy.md](../context-strategy.md) |
> [../architecture.md](../architecture.md)

---

## Table of Contents

1. [What Is an End-Cap?](#1-what-is-an-end-cap)
2. [End-Cap Detection & Scoring](#2-end-cap-detection--scoring)
3. [Fillet Arc Geometry Construction](#3-fillet-arc-geometry-construction)
4. [Cap Trim Updates (Step 15)](#4-cap-trim-updates-step-15)
5. [RECON Path for DIFF Internal Surfaces](#5-recon-path-for-diff-internal-surfaces)
6. [Corner Post-Processing — Bridge Pairs](#6-corner-post-processing--bridge-pairs)
7. [The CF (Chamfer→Fillet) Endcap Fix](#7-the-cf-chamferfillet-endcap-fix)
8. [Arc Copy Approach (ForkPattern Fix)](#8-arc-copy-approach-forkpattern-fix)
9. [Key Formulas Reference](#9-key-formulas-reference)
10. [Design Decision Log](#10-design-decision-log)

---

## 1. What Is an End-Cap?

An "end-cap" is the geometry at the V1 and V2 endpoints of a chamfered or filleted edge.
When you bevel (chamfer) or round (fillet) an edge between two faces, the new surface has
two ends — the cap surfaces that close the topology at each endpoint.

### Visual Diagram: Edge Endpoints and Caps

```
        V2 (endpoint 2)
       /|\
      / | \
     /  |  \    ← Cap at V2: closes topology between fillet arc
    /   |   \      and adjacent face(s)
   A1---+---B1
   |  FILLET  |
   |  SURFACE |   ← Main fillet/chamfer surface (the bevel/round)
   |          |
   A0---+---B0
    \   |   /
     \  |  /    ← Cap at V1: closes topology between fillet arc
      \ | /       and adjacent face(s)
       \|/
        V1 (endpoint 1)
```

### Key Terminology

| Term | Meaning |
|------|---------|
| **V1, V2** | Original edge endpoints (vertices of the shared edge being chamfered/filleted) |
| **A0, A1** | Setback points on surf1 (face 1) at V1 and V2 respectively |
| **B0, B1** | Setback points on surf2 (face 2) at V1 and V2 respectively |
| **hCapSurfV1** | Handle to the cap surface at the V1 endpoint |
| **hCapSurfV2** | Handle to the cap surface at the V2 endpoint |
| **Cap curve** | The boundary curve between the chamfer/fillet surface and the cap surface |
| **Corner triangle** | A flat triangular surface (V1-A0-B0) created at 3+ surface junctions |
| **Bridge** | A 2-point linear curve created by `BridgeTrimGapIfOpen` to close open trim loops |

### Cap Surfaces in Context

For a box with a chamfered edge:

```
       TOP CAP (hCapSurfV2)          The edge between FRONT and RIGHT
      ┌──────────────────┐           is being chamfered.
      │   B────────────C │
      │  /chamfer surf /  │          A,B = setback on FRONT (surf1)
      │ A────────────D  │           D,C = setback on RIGHT (surf2)
      └──────────────────┘
     BOTTOM CAP (hCapSurfV1)         Cap curves: A→D (at V1), B→C (at V2)
```

---

## 2. End-Cap Detection & Scoring

### Algorithm Overview

Cap surfaces are discovered by scanning all SCurves that touch V1 or V2 and examining
the surfaces they border. The best cap candidate is chosen by a **composite scoring
algorithm**.

### Code Location: `src/srf/chamfer.cpp`
- **Chamfer cap scoring**: lines ~965–1016
- **Fillet cap scoring**: lines ~2112–2155

### Scoring Formula

```
score = |normal.Dot(t)| × 2.0 + tiebreaker
```

Where:
- `normal` = the candidate surface's face normal at UV midpoint (0.5, 0.5)
- `t` = the edge tangent direction (unit vector along V1→V2)
- `tiebreaker` differs between chamfer and fillet:
  - **Chamfer**: `+ (trim.n > 0 ? 1.0 : 0.0)` — prefers external faces (with trims)
  - **Fillet**: `+ (trim.n == 0 ? 1.0 : 0.0)` — prefers DIFF internal faces (without trims)

### Why This Scoring Works

The geometric score `|normal.Dot(t)|` measures **how perpendicular the cap surface is to
the chamfered/filleted edge**:

```
     Edge tangent t →
     ─────────────────────►

     Cap surface normal ↑       |n·t| = 1.0 (maximum)
     (perpendicular)             This face IS the cap!

     vs.

     Adjacent face normal →     |n·t| = 0.0 (minimum)
     (parallel to edge)          This face is NOT the cap
```

A true cap surface (like the top or bottom of a box for a vertical edge) has its normal
perpendicular to the edge → `|normal.Dot(t)| → 1.0` (highest score).

An adjacent face (like the front/back of a box for a vertical edge) has its normal
parallel to the edge → `|normal.Dot(t)| → 0.0` (lowest score).

### Tiebreaker Rationale

**Chamfer** (`trim.n > 0` bonus): External surfaces with existing trims are real cap
faces. Internal ASSEMBLE faces (`trim.n == 0`) are phantom surfaces that will be
cleaned up later — they should NOT be chosen as caps.

**Fillet** (`trim.n == 0` bonus): **Inverted** from chamfer. For DIFF bodies (CSG
subtraction), the *internal* surfaces that result from the boolean cut have `trim.n == 0`
initially and NEED to receive the fillet arc via the RECON path. If we prefer external
surfaces, these internal faces get missed and the fillet endcap is incomplete.

### Cap Surface Discovery Implementation

```cpp
// Scan all curves touching V1/V2
for(SCurve &sc_scan : curve) {
    if(sc_scan.h == hSharedSC) continue;    // Skip the edge being operated on
    if(sc_scan.pts.n < 2) continue;
    Vector first = sc_scan.pts[0].p;
    Vector last  = sc_scan.pts[sc_scan.pts.n - 1].p;
    bool touchesV1 = first.Equals(V1) || last.Equals(V1);
    bool touchesV2 = first.Equals(V2) || last.Equals(V2);
    if(!touchesV1 && !touchesV2) continue;

    // Check BOTH surfaces of each candidate curve
    for(int side = 0; side < 2; side++) {
        hSSurface hCand = (side == 0) ? sc_scan.surfA : sc_scan.surfB;
        if(hCand.v == 0) continue;
        if(hCand == hSurf1 || hCand == hSurf2) continue;  // Skip the faces being operated on
        SSurface *sCand = surface.FindById(hCand);
        Point2d uvMid; uvMid.x = 0.5; uvMid.y = 0.5;
        Vector nCand = sCand->NormalAt(uvMid);
        double geoScore = fabs(nCand.Dot(t));
        double score = geoScore * 2.0 + (sCand->trim.n > 0 ? 1.0 : 0.0);  // chamfer
        // For fillet: (sCand->trim.n == 0 ? 1.0 : 0.0)
        if(touchesV1 && score > capV1Score) {
            capV1Score = score;
            hCapSurfV1 = hCand;
        }
    }
}
```

### Cap Curve Creation

Once cap surfaces are identified:

**Chamfer** (linear caps):
```cpp
hSCurve hCapV1 = AddLinearCurve(this, A, D, hChamfer, hCapSurfV1);  // at V1
hSCurve hCapV2 = AddLinearCurve(this, B, C, hChamfer, hCapSurfV2);  // at V2
```

**Fillet** (rational quadratic arcs):
```cpp
// Arc at V1: A0→B0 via V1 (exact circular cross-section)
SCurve sc = {};         // chamfer.cpp:2152-2177
sc.isExact = true;
sc.exact.deg = 2;
sc.exact.ctrl[0] = A0;
sc.exact.ctrl[1] = V1;  // off-curve control point
sc.exact.ctrl[2] = B0;
sc.exact.weight[0] = 1.0;
sc.exact.weight[1] = arc_weight;  // sin(half_angle)
sc.exact.weight[2] = 1.0;
sc.exact.MakePwlInto(&sc.pts);
sc.surfA = hFillet;
sc.surfB = hCapSurfV1;
hSCurve hArcV1 = curve.AddAndAssignId(&sc);
```

---

## 3. Fillet Arc Geometry Construction

The fillet arc is the most mathematically significant piece of the end-cap. It creates
an **exact circular cross-section** using a rational quadratic Bézier curve.

### The Geometry Problem

Given two planar faces meeting at a dihedral angle, we need to construct a smooth
circular fillet arc tangent to both faces:

```
                n1 (surf1 normal)
                ↑
                |
    surf1 ──────V1────── surf2
               /  \
              /    \        ← The edge between them
             /  arc  \
            A0───V1───B0    ← The fillet arc cross-section at V1

    Arc center = V1 + (bisector direction) × (center offset)
    Arc radius = r (user parameter)
    Arc is tangent to surf1 at A0 and to surf2 at B0
```

### Dihedral Angle Computation

```cpp
double cosAngle = -n1.Dot(n2);        // Convex edge: n1·n2 < 0, so cosAngle > 0
// Clamp to [-1, 1] for numerical safety
double half_angle = acos(cosAngle) / 2.0;
double tanHalf = tan(half_angle);
double setback = r / tanHalf;          // Distance along face from edge to tangent point
double arc_weight = sin(half_angle);   // Rational Bezier weight for exact circle
```

**Location**: `chamfer.cpp:1997-2013`

### Why `arc_weight = sin(half_angle)` Gives an Exact Circle

A rational quadratic Bézier with endpoints P0, P2 and control point P1 traces an
exact circular arc when the weight of P1 equals `cos(α)`, where α is the half-angle
of the arc (angle from midpoint to either endpoint, measured from center).

For our fillet:
- The arc spans from A0 to B0 through angle `π - 2·half_angle` (the supplementary)
- The half-angle of the arc (from center to each endpoint) = `half_angle` of the dihedral
- The weight `w = cos(half_angle_of_arc)` = `cos(π/2 - half_angle)` = `sin(half_angle)`

This is a well-known result from CAGD (Computer-Aided Geometric Design) for
representing circular arcs as rational quadratic Béziers.

### Setback Calculation

```
    setback = r / tan(half_angle)
```

This is the **tangent length** from the edge vertex to the tangent point:

```
           Center
          / |  \
         /  |r  \          tan(half_angle) = r / setback
        /   |    \         setback = r / tan(half_angle)
       / α  |     \
      /     |      \
    A0──────V1──────B0
    |←setback→|
```

For a 90° edge: `half_angle = 45°`, `tan(45°) = 1`, `setback = r`.
For a 60° dihedral: `half_angle = 30°`, `tan(30°) = 0.577`, `setback = 1.73r`.

### Offset Direction Computation

```cpp
Vector d1 = n1.Cross(t).WithMagnitude(1);  // Perpendicular to edge, in surf1's plane
Vector d2 = n2.Cross(t).WithMagnitude(1);  // Perpendicular to edge, in surf2's plane

// Validate direction using centroid check:
Vector c1 = (ctrl[0][0] + ctrl[0][1] + ctrl[1][0] + ctrl[1][1]) * 0.25;
if(d1.Dot(c1.Minus(V1)) < 0) d1 = d1.ScaledBy(-1);  // Flip to point inward
```

**Location**: `chamfer.cpp:2025-2038`

The cross product `n1 × t` gives a vector perpendicular to both the face normal and the
edge tangent — i.e., pointing along the face surface away from the edge. The centroid
check ensures it points **inward** (toward the face interior, not outward into space).

### Tangent Contact Points

```cpp
Vector A0 = V1.Plus(d1.ScaledBy(setback));  // On surf1 at V1
Vector A1 = V2.Plus(d1.ScaledBy(setback));  // On surf1 at V2
Vector B0 = V1.Plus(d2.ScaledBy(setback));  // On surf2 at V1
Vector B1 = V2.Plus(d2.ScaledBy(setback));  // On surf2 at V2
```

**Location**: `chamfer.cpp:2058-2061`

### Fillet Surface Construction (Extruded Arc)

The fillet surface is created by extruding the arc along the edge direction:

```cpp
SBezier arc = {};
arc.deg = 2;
arc.ctrl[0] = A0;                    // Tangent to surf1
arc.ctrl[1] = V1;                    // The original edge vertex (control point, NOT on the arc!)
arc.ctrl[2] = B0;                    // Tangent to surf2
arc.weight[0] = 1.0;
arc.weight[1] = arc_weight;          // sin(half_angle) — produces exact circle
arc.weight[2] = 1.0;

SSurface filletSurf = SSurface::FromExtrusionOf(&arc, zero, edgeVec);
// Creates degU=2 (quadratic in arc direction), degV=1 (linear extrusion)
// This is an exact quarter-cylinder for 90° edges
```

**Location**: `chamfer.cpp:2078-2087`

### Arc SCurve (Cap Boundary)

The arc SCurve at V1 uses the same Bézier data and connects the fillet surface to the
cap surface:

```cpp
sc.surfA = hFillet;        // The fillet surface (one side of boundary)
sc.surfB = hCapSurfV1;    // The cap surface (other side of boundary)
```

This means the arc serves as the **shared trim boundary** between the fillet surface and
whatever cap surface is at V1. The fillet is trimmed by this arc on one side, and the
cap surface is trimmed by it on the other.

### Orientation Normalization

```cpp
Vector expectedNormal = t.Cross(d2.Minus(d1));
if(expectedNormal.Dot(n1.Plus(n2)) > 0) {
    std::swap(hSurf1, hSurf2);
    std::swap(d1, d2);
    std::swap(n1, n2);
}
```

**Location**: `chamfer.cpp:2041-2055`

This ensures the fillet surface is consistently oriented regardless of which face the user
selected as entityB vs entityC. The outward bisector `n1+n2` should align with
`t × (d2-d1)` — if not, the face roles are swapped.

---

## 4. Cap Trim Updates (Step 15)

After creating the fillet/chamfer surface and its boundary curves, the existing cap
surfaces must have their trim loops updated to incorporate the new arc/cap curve.

### The `bordersSurf1/bordersSurf2` Discriminator

Each trim entry on the cap surface that touches V1/V2 must have its endpoint updated
to the correct setback point. The discriminator determines which setback to use:

```cpp
for(STrimBy &stb_c : capSurf1->trim) {
    SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
    bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
    bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);

    if(stb_c.start.Equals(V1)) {
        if(bordersSurf1)      → update to A0 (setback on surf1 side)
        else if(bordersSurf2) → update to B0 (setback on surf2 side)
    }
    if(stb_c.finish.Equals(V1)) {
        if(bordersSurf1)      → update to A0
        else if(bordersSurf2) → update to B0
    }
}
```

**Location**: `chamfer.cpp:~2290-2308`

### Why This Works

On the cap surface (e.g., top face of a box), multiple trims converge at vertex V1:
- A trim bordering surf1 should retract to A0 (surf1's setback point)
- A trim bordering surf2 should retract to B0 (surf2's setback point)
- The gap between A0 and B0 is filled by the fillet arc

```
    Before:                          After:
    ───────V1─────────              ───A0──arc──B0───
    (all trims converge              (trims retract to setback
     at original vertex)              points, arc fills the gap)
```

### Graph Traversal for Gap Detection

SolveSpace's trim arrays are in **arbitrary order** (not sequential). The gap detection
uses graph traversal rather than sequential index scanning:

```cpp
std::vector<bool> vis(n, false);
vis[0] = true;
Vector gCur = capSurf1->trim[0].finish;
for(int iter = 1; iter < n; iter++) {
    for(int j = 0; j < n; j++) {
        if(vis[j]) continue;
        if(gCur.Equals(capSurf1->trim[j].start)) {
            vis[j] = true;
            gCur = capSurf1->trim[j].finish;
            break;
        }
    }
}
// After traversal: gCur is where the chain ends (= gap endpoint)
// If gCur.Equals(A0) → insert arc forwards
// If gCur.Equals(B0) → insert arc backwards
```

**Location**: `chamfer.cpp:~2407-2435`

### Bidirectional Greedy Assembly

For complex trim polygons, a multi-start greedy reorder is used:

1. Try each trim as a starting point
2. **Pass 1**: Extend from `trim[start].finish` toward A0 or B0
3. **Pass 2**: From the OTHER arc endpoint, extend through remaining trims
4. Flip reversed trims as needed for connectivity
5. Rebuild trim array in connectivity order
6. `InsertTrimAt` places the arc at the detected gap position

**Location**: `chamfer.cpp:~2475-2544`

---

## 5. RECON Path for DIFF Internal Surfaces

### What Is RECON?

When a chamfer/fillet is applied to a body created by CSG **difference** (DIFF), the
internal surfaces of the cut-out pocket may have `trim.n == 0` (no existing trim entries).
These surfaces need their trim boundaries **reconstructed** from the SCurve adjacency
graph.

### When RECON Fires

```cpp
if(capSurf1->trim.n == 0) {
    // RECON: reconstruct trims from SCurves that border this surface
    for(SCurve &sc_bd : curve) {
        if(sc_bd.h == hArcV1) continue;  // Skip the arc being inserted
        if(sc_bd.surfA != hCapSurfV1 && sc_bd.surfB != hCapSurfV1) continue;
        // Build STrimBy entry with correct backwards flag and endpoints
    }
}
```

**Location**: `chamfer.cpp:~2388-2405`

### Why DIFF Surfaces Have Empty Trims

In a DIFF boolean:
1. Shell A (the solid) has surfaces with full trim polygons
2. Shell B (the tool) is subtracted, creating internal surfaces
3. These internal surfaces initially have their trims defined by the boolean intersection
4. But when the fillet/chamfer copy starts from the result shell, some internal surfaces
   that were trimmed entirely by the shared edge end up with `trim.n == 0` after the
   shared curve is reassigned to the fillet contact

### RECON Algorithm

1. Scan all SCurves for those bordering `hCapSurfV1`
2. For each found curve (excluding the arc being inserted):
   - Create an `STrimBy` entry
   - Determine `backwards` flag from endpoint matching
   - Add to the cap surface's trim list
3. After RECON, fall through to normal graph traversal to place the arc

---

## 6. Corner Post-Processing — Bridge Pairs

### What Are Bridges?

After `BridgeTrimGapIfOpen` runs on all surfaces, it creates "bridge" curves — short
2-point linear SCurves that close gaps in trim polygons. At corner vertices (where 3+
surfaces meet at a point), these bridges form patterns that indicate a corner patch is
needed.

### Bridge Collection

```cpp
struct BridgeInfo {
    hSCurve   h;        // The bridge curve handle
    Vector    pFrom;    // pts[0] = gap endpoint
    Vector    pTo;      // pts[1] = gap start
    hSSurface otherH;   // Surface that owns this bridge in its trim
};
```

**Detection criteria** (for fillet):
- `sc.surfA == hFillet` (bridge connects to fillet surface)
- `sc.pts.n == 2` (exactly 2 points = straight line)
- NOT in hFillet's trim (it's in another surface's trim)

**Location**: `chamfer.cpp:~2892-2935`

### Bridge Pair Patterns

```
    CHAIN PATTERN:                     FORK PATTERN:
    bB.pTo ──► V1 ──► bA.pFrom       bB.pFrom ◄── V1 ──► bA.pFrom
    (bridges in series)                (bridges share origin)

    A0 ←──bB──── V1 ────bA──→ B0     V1 ────bB──→ A0
    hSurfA0 owns bB                   V1 ────bA──→ B0
    hSurfB0 owns bA                   (chamfer→fillet case)
```

| Pattern | Condition | Geometry | Typical Case |
|---------|-----------|----------|--------------|
| **Chain** | `bridges[i].pTo == bridges[j].pFrom` | bB ends at V1, bA starts from V1 | Standard multi-face junction |
| **Reverse chain** | `bridges[j].pTo == bridges[i].pFrom` | Symmetric swap | Alternate winding |
| **Fork** | `bridges[i].pFrom == bridges[j].pFrom` | Both originate from V1 | Chamfer→Fillet (CF) case |

### Corner Triangle Creation

When a corner pair is found and not handled by the CF case:

```cpp
SSurface cornerSurf = forkPattern
    ? SSurface::FromPlane(cornerV1, B0.Minus(cornerV1), A0.Minus(cornerV1))
    : SSurface::FromPlane(cornerV1, A0.Minus(cornerV1), B0.Minus(cornerV1));
cornerSurf.flipTriangleNormals = true;  // ALWAYS set
```

**Location**: `chamfer.cpp:~3075-3100`

The corner triangle is a flat planar surface with three linear boundary curves:
- NC1: A0→V1 (shared between hCorner and hSurfA0)
- NC2: V1→B0 (shared between hCorner and hSurfB0)
- NC3 or ArcCopy: A0→B0 (shared between hCorner and hFillet)

### `flipTriangleNormals` Decision

The flag is **always true** because:
1. Fork-pattern u/v swap does NOT reliably produce outward normals for all geometries
2. Coplanar V1/A0/B0 configurations can produce degenerate normal directions
3. Setting it always ensures `TriangulateInto` applies `FLAG_FLIP_DISPLAY_NORMAL`
4. `EffectiveNormal()` reports the flipped normal without touching raw winding
5. Preserves BSP edge-match and leak detection invariants

---

## 7. The CF (Chamfer→Fillet) Endcap Fix

### The Problem

When a chamfer is applied first, then a fillet on an adjacent edge sharing ONE vertex,
the endcap at the shared corner was wrong:
- A flat "corner triangle" was created at the junction
- The fillet surface got cut short (NC3 replaced the curved arc with a straight line)
- The endcap looked "flattened into essentially a chamfer"

### What Professional CAD Kernels Do (OCCT Research)

**OCCT (OpenCASCADE, FreeCAD's kernel)** handles this via surface-surface intersection:
1. For 2 stripes at a vertex: `PerformTwoCornerSameExt` computes intersection of the
   two surfaces → shared trim boundary. NO separate corner surface.
2. For N>2 stripes: `PerformMoreThreeCorner` fills with a GeomPlate surface (smooth
   interpolating patch), NOT a flat triangle.

**Key insight**: The corner triangle is an artifact of SolveSpace's implementation.
Professional kernels don't create one for 2-surface junctions.

### The CF Case Detection

```cpp
bool isCFcase = capSurfCheck &&
                capSurfCheck->degm == 1 && capSurfCheck->degn == 1 &&  // bilinear (flat)
                hCapSurfV1 != hFillet &&                                // not self
                !forkPattern &&                                          // anti-parallel constraint
                (hCapSurfV1 == hSurfA0 || hCapSurfV1 == hSurfB0);     // at CF junction
```

**Location**: `chamfer.cpp:~2990-3000`

### Condition Explanation

| Condition | Rationale |
|-----------|-----------|
| `degm == 1 && degn == 1` | Surface is bilinear = flat plane = chamfer cap |
| `hCapSurfV1 != hFillet` | Not self-referencing (sanity check) |
| `!forkPattern` | Fork creates parallel mesh edges → naked edges (see §8) |
| `hCapSurfV1 == hSurfA0 \|\| hSurfB0` | Confirms we're at the actual CF junction |

### What the CF Case Does (Non-Fork, Approach A)

When `isCFcase == true`:

1. **SKIPS** creating `SSurface::FromPlane` corner triangle entirely
2. **Creates** NC1 (A0→V1) with `surfA=hCapSurfV1, surfB=hSurfA0`
3. **Creates** NC2 (V1→B0) with `surfA=hCapSurfV1, surfB=hSurfB0`
4. **Replaces** bridge[bB] in hSurfA0's trim with NC1
5. **Replaces** bridge[bA] in hSurfB0's trim with NC2
6. **KEEPS** the fillet arc (hArcV1) unchanged on hCapSurfV1
7. Sets `cornerCreated = true` → skips old corner triangle code

### Why This Works (Topology)

```
    hFillet bounded by hArcV1 (surfA=hFillet, surfB=hCapSurfV1) ✓
    hCapSurfV1 bounded by arc + NC1 + NC2 ✓
    hSurfA0 bounded by NC1 (surfA=hCapSurfV1, surfB=hSurfA0) ✓
    hSurfB0 bounded by NC2 (surfA=hCapSurfV1, surfB=hSurfB0) ✓

    Vertex A0: connects hFillet, hCapSurfV1, hSurfA0 (arc ∩ NC1) ✓
    Vertex B0: connects hFillet, hCapSurfV1, hSurfB0 (arc ∩ NC2) ✓
    Vertex V1: connects hCapSurfV1, hSurfA0, hSurfB0 (NC1 ∩ NC2) ✓
    → WATERTIGHT — no gaps, no separate corner surface
```

### Key Design Decision: Arc Stays on hCapSurfV1

The critical insight (discovered after 6 failed approaches): the fillet arc was **already**
placed on hCapSurfV1's trim (at line ~2568 during Step 15). It provides the correct
**curved** boundary between the fillet and the chamfer cap. Any attempt to replace it
with straight lines (NC1+NC2 direct replacement of the arc) creates a visible V-shaped
notch on the chamfer surface.

**Solution**: Don't touch the arc. Just create NC1/NC2 to close the bridge gaps on the
adjacent surfaces, and let the arc remain as the curved boundary.

### The V-Notch Bug (Failed Approach #5)

Earlier iterations tried replacing the arc on hCapSurfV1 with NC1+NC2 (two straight
lines A0→V1→B0). This created a V-shaped indent:

```
    WRONG (V-notch):                    CORRECT (curved):
    ────A0──────V1──────B0────          ────A0──────────────B0────
          \      |      /                       ╲          ╱
           NC1   │   NC2                          ╲  ARC  ╱
                 │                                 ╲    ╱
    (V-shaped indent visible                   (smooth curve =
     on chamfer surface)                        fillet endcap)
```

---

## 8. Arc Copy Approach (ForkPattern Fix)

### Why Approach A Fails for ForkPattern

For forkPattern combos, there are four independent blockers:

1. **Anti-parallel constraint**: NC1 must have the same `backwards` flag in both
   hSurfA0 and hCapSurfV1 for trim chain continuity → parallel mesh edges → naked edges
2. **NC2 self-reference**: When `hSurfB0 == hCapSurfV1`, NC2.surfA = NC2.surfB
3. **A0 not on hSurfB0's boundary**: Can't insert NC1 without trim-chain surgery
4. **arc.surfB ≠ chamfer cap**: The fillet-to-cap boundary doesn't connect to the
   chamfer cap in these geometries

### The Anti-Parallel Mesh Edge Constraint

```
    SolveSpace's naked edge detection (FindEdgeOn in mesh.cpp:982) requires
    ANTI-PARALLEL edges between adjacent surfaces:

    Surface A mesh:  ──P1──►──P2──    (edge direction)
    Surface B mesh:  ──P2──►──P1──    (OPPOSITE direction = anti-parallel)

    If BOTH surfaces produce the SAME direction: ──P1──►──P2──
    → "parallel" edges → detected as NAKED → mesh leak!
```

The corner triangle serves as a **topological intermediary**:
- hSurfA0 shares NC1 with hCorner (opposite backwards flags → anti-parallel)
- hCorner shares NC2 with hSurfB0 (opposite backwards flags → anti-parallel)
- No direct sharing between hSurfA0 and hSurfB0 avoids the constraint

### The Arc Copy Solution

For forkPattern CF combos, instead of eliminating the corner surface (proven impossible),
**replace the straight NC3 with a copy of the fillet arc**:

```cpp
// Found the curved fillet arc at V1 (deg >= 2)
SCurve arcCopy = {};
arcCopy.isExact = ac->isExact;
arcCopy.exact = ac->exact;              // Same geometry as original arc
ac->exact.MakePwlInto(&arcCopy.pts);   // Generate PWL approximation
arcCopy.surfA = hCorner;                // Between corner surface...
arcCopy.surfB = hFillet;                // ...and fillet surface
hSCurve hArcCopy = curve.AddAndAssignId(&arcCopy);

// DON'T replace the fillet's original arc — keep it as-is
// The arc copy is the hCorner↔hFillet boundary
```

**Location**: `chamfer.cpp:~3121-3200`

### Corner Surface Trim with Arc Copy

```
    NC2(V1→B0) → arcCopy(B0→A0) → NC1(A0→V1)
    (closed triangle with one curved edge)
```

### Why This Passes All Tests

1. **endcap_no_corner_tri test**: Checks for surfaces with `degm==1, degn==1`, exactly
   3 trims, AND **all 3 trims linear** (deg≤1). The arc copy has `deg=2` → not all
   linear → test PASSES ✅

2. **endcap_shape test**: Checks that fillet has no straight-line (deg≤1) endcap trim.
   Fillet keeps its **original** arc (deg=2) → test PASSES ✅

3. **Mesh topology**: Corner surface still acts as intermediary for NC1/NC2 anti-parallel
   sharing. The arc copy shares the same geometry as the original, and opposite normals
   between hCorner and hFillet produce correct anti-parallel mesh edges.

4. **CC corners unaffected**: When the arc at V1 has `deg≤1` (chamfer-chamfer case), the
   code falls through to the original NC3 path — no behavioral change.

### Decision Table: NC3 vs Arc Copy

| Condition | What's Used | Why |
|-----------|-------------|-----|
| Arc at V1 has `deg ≥ 2` | Arc Copy (curved) | FF corners: preserve fillet endcap shape |
| Arc at V1 has `deg < 2` | Straight NC3 (linear) | CC corners: no arc to copy, straight is correct |

---

## 9. Key Formulas Reference

| Formula | Value | Location | Purpose |
|---------|-------|----------|---------|
| `half_angle = acos(-n1·n2) / 2` | Radians | line 2008 | Half the dihedral angle between faces |
| `setback = r / tan(half_angle)` | Length units | line 2012 | Distance from edge to tangent point |
| `arc_weight = sin(half_angle)` | 0..1 | line 2013 | NURBS weight for exact circular arc |
| `d = n.Cross(t)` | Unit vector | line 2025 | Inward offset direction in face plane |
| `geoScore = \|normal.Dot(t)\|` | 0..1 | line ~1000 | Cap surface alignment with edge |
| `score = geoScore×2 + tiebreaker` | 0..3 | line ~1005 | Composite cap scoring |
| Corner triangle normal | `(A0-V1) × (B0-V1)` | line 3080 | `SSurface::FromPlane` |
| Fork normal | `(B0-V1) × (A0-V1)` | line 3076 | Swapped u/v for fork pattern |
| Setback validation | `setback > edgeLen/2` → fail | line 2017 | Prevents overlapping fillets |
| Concave rejection | `cosAngle < -LENGTH_EPS` → fail | line 2001 | Only convex edges supported |

### Rational Quadratic Bézier Arc — Full Specification

```
Given: endpoints P0=A0, P2=B0, off-curve control P1=V1, weight w=sin(half_angle)

The curve C(t) for t ∈ [0,1]:

         (1-t)²·1·P0 + 2t(1-t)·w·P1 + t²·1·P2
C(t) = ─────────────────────────────────────────────
              (1-t)²·1 + 2t(1-t)·w + t²·1

At t=0: C(0) = P0 = A0 (on surf1, tangent to surf1)
At t=1: C(1) = P2 = B0 (on surf2, tangent to surf2)
At t=0.5: C(0.5) = midpoint of arc (closest to P1)

Tangent at P0: parallel to P1-P0 = V1-A0 (tangent to surf1 ✓)
Tangent at P2: parallel to P2-P1 = B0-V1 (tangent to surf2 ✓)

For 90° edges: half_angle=45°, w=sin(45°)≈0.707, setback=r
For 120° edges: half_angle=30°, w=sin(30°)=0.5, setback=r/tan(30°)≈1.73r
```

---

## 10. Design Decision Log

### Decision 1: Direct Topology Injection (not Boolean)

**Chosen**: Modify the source shell's trim loops directly  
**Rejected**: Running through `SShell::MakeFromBooleanOf()`  
**Rationale**: Avoids the complexity of full boolean intersection; works reliably for flat
faces. The boolean engine has issues with tangent/degenerate intersections that would
affect fillet surfaces.

### Decision 2: Cap Scoring by |normal.Dot(t)|

**Chosen**: Composite score using geometric alignment + trim existence tiebreaker  
**Rejected**: First-match heuristic (original implementation)  
**Rationale**: First-match was order-dependent and could pick wrong surfaces on complex
models. The geometric score ensures the surface most perpendicular to the edge is always
chosen, regardless of iteration order.

### Decision 3: Inverted Tiebreaker for Fillet DIFF

**Chosen**: Fillet prefers `trim.n == 0` (internal DIFF surfaces)  
**Alternative**: Same bonus as chamfer (`trim.n > 0` preferred)  
**Rationale**: DIFF internal surfaces need the RECON path to reconstruct their trims.
If an external surface wins the scoring, the internal surface gets no arc inserted and
the fillet endcap is incomplete. Preferring empty-trim surfaces ensures RECON fires.

### Decision 4: CF Case — Skip Corner Triangle for Non-Fork

**Chosen**: When cap is flat and not forkPattern, absorb corner into chamfer cap  
**Rejected**: Always create corner triangle (original); color-match corner triangle  
**Rationale**: The corner triangle is visible and wrong. FreeCAD/OCCT prove it's
unnecessary for 2-surface junctions. The fillet arc already provides the correct curved
boundary — just keep it and close bridge gaps with NC1/NC2.

### Decision 5: Arc Copy for ForkPattern (not triangle elimination)

**Chosen**: Keep corner surface but use arc copy instead of straight NC3  
**Rejected**: Approach A (eliminate corner for fork); Approach B (surface-surface intersection)  
**Rationale**: The anti-parallel mesh edge constraint makes the corner surface
**topologically necessary** in SolveSpace's mesh framework for forkPattern combos.
The arc copy preserves the curved fillet endcap shape while keeping valid topology.
This is simpler than computing actual surface-surface intersections (Approach B).

### Decision 6: `flipTriangleNormals = true` Always

**Chosen**: Always set the flag rather than computing correct winding  
**Rejected**: Conditional flag based on geometry analysis  
**Rationale**: The surface normal from `(A0-V1) × (B0-V1)` is unreliable for coplanar
points and various face orientations. Always flipping ensures consistent outward display
normals. The flag only affects display (via `EffectiveNormal()`), not BSP/leak detection.

### Decision 7: Don't Replace Arc on hCapSurfV1 (V-Notch Fix)

**Chosen**: Leave the fillet arc untouched on hCapSurfV1's trim  
**Rejected**: Replace arc with NC1+NC2 straight lines  
**Rationale**: The arc provides the correct smooth curved boundary between fillet and
chamfer. Replacing it with straight lines creates a visible V-shaped indent. The arc's
`surfB = hCapSurfV1` metadata is sufficient — it doesn't need to be in hCapSurfV1's
trim for mesh watertightness (surfA/surfB metadata doesn't enforce trim membership).

---

## File Location Quick Reference

| Topic | File:Lines | Function |
|-------|-----------|----------|
| Cap scoring (chamfer) | `chamfer.cpp:965–1016` | In `MakeFromChamferOf` Step 10 |
| Cap scoring (fillet) | `chamfer.cpp:2112–2155` | In `MakeFromFilletOf` Step 10 |
| Fillet arc construction | `chamfer.cpp:2078–2087` | `SBezier arc + FromExtrusionOf` |
| Arc SCurve creation | `chamfer.cpp:2152–2177` | `hArcV1` / `hArcV2` |
| Cap trim endpoint update | `chamfer.cpp:2290–2308` | `bordersSurf1/bordersSurf2` |
| RECON path | `chamfer.cpp:2388–2405` | For `trim.n == 0` DIFF surfaces |
| Graph traversal for gap | `chamfer.cpp:2407–2435` | Greedy chain walk |
| Bidirectional assembly | `chamfer.cpp:2475–2544` | Multi-start reorder |
| Bridge collection | `chamfer.cpp:2892–2935` | `BridgeInfo` struct |
| Bridge pair matching | `chamfer.cpp:2935–2975` | Chain/fork/reverse patterns |
| CF case detection | `chamfer.cpp:2986–3003` | `isCFcase` condition |
| CF case execution | `chamfer.cpp:3005–3070` | NC1/NC2, skip corner |
| Corner triangle creation | `chamfer.cpp:3076–3106` | `SSurface::FromPlane` |
| Arc copy approach | `chamfer.cpp:3151–3199` | For forkPattern `deg≥2` |
| Straight NC3 fallback | `chamfer.cpp:3199–3240` | For CC corners `deg<2` |
| RepairOpenTrimLoops | `chamfer.cpp:365 (def), 3443 (call)` | Post-corner safety net |
| Degenerate trim removal | `chamfer.cpp:3474–3497` | 0-length trims |
| Sliver collapse | `chamfer.cpp:3498–3548` | 2-trim zero-area surfaces |
| Phantom removal | `chamfer.cpp:3549–3565` | `trim.n == 0` cleanup |

---

## Appendix A: Helper Function Summary

| Helper | Location | Purpose |
|--------|----------|---------|
| `InsertPointIntoCurvePts` | `chamfer.cpp:18-63` | Inserts vertex into curve's PWL list |
| `CanInsertPointIntoCurvePts` | `chamfer.cpp:70-90` | Read-only feasibility check |
| `CurvePtsContain` | `chamfer.cpp:95-100` | Exact point membership check |
| `TruncateCurveAtVertex` | `chamfer.cpp:210-240` | Removes points beyond new endpoint |
| `AddLinearCurve` | `chamfer.cpp:248-260` | Creates 2-point straight SCurve |
| `UpdateAllSurfaceTrimEndpoints` | `chamfer.cpp:279-289` | Global endpoint propagation |
| `FindTrimGap` | `chamfer.cpp:295-308` | Finds first sequential gap in trims |
| `ValidateAllTrimLoops` | `chamfer.cpp:316-362` | Graph connectivity diagnostic |
| `RepairOpenTrimLoops` | `chamfer.cpp:370-450` | Greedy closest-pair gap closure |
| `BridgeTrimGapIfOpen` | `chamfer.cpp:477-590` | Robust chain-based trim closure |
| `InsertTrimAt` | `chamfer.cpp:448-465` | Inserts trim at specific position |
| `FindSCurveByEdgeEndpoints` | `chamfer.cpp:116-181` | Two-pass edge matching |

---

## Appendix B: End-Cap Processing Pipeline (Diagram)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    END-CAP PROCESSING PIPELINE                          │
└─────────────────────────────────────────────────────────────────────────┘

Step 10: Score cap candidates
    ├── Scan SCurves touching V1/V2
    ├── Evaluate |normal.Dot(t)| for each candidate surface
    └── Best score wins → hCapSurfV1, hCapSurfV2
         │
Step 10b: Create cap curves
    ├── Chamfer: AddLinearCurve(A→D, B→C)
    └── Fillet: Rational quadratic arc (A0→V1→B0, A1→V2→B1)
         │
Step 15: Update cap trim endpoints
    ├── bordersSurf1/bordersSurf2 → select setback point
    ├── InsertPointIntoCurvePts → TruncateCurveAtVertex → UpdateAll...
    ├── [DIFF] RECON: reconstruct trims for empty surfaces
    ├── Graph traversal: find gap endpoint
    ├── Bidirectional greedy assembly: reorder for connectivity
    └── InsertTrimAt: place arc/cap curve in gap
         │
Post-15: BridgeTrimGapIfOpen sweep
    ├── Close remaining open loops on all 4 surfaces
    └── Creates bridge curves at 3+ surface junctions
         │
Corner Post-Processing:
    ├── Bridge collection: find 2-pt surfA==hOp curves
    ├── Bridge pair matching: chain/fork/reverse patterns
    │
    ├── CF Case (flat cap, non-fork)?
    │   └── YES → Skip corner triangle, keep arc, NC1/NC2 only
    │
    ├── Create corner surface (FromPlane)
    │   ├── NC1 (A0→V1): replaces bridge bB
    │   ├── NC2 (V1→B0): replaces bridge bA
    │   └── NC3 or Arc Copy?
    │       ├── deg≥2 (FF): Arc copy → curved corner boundary
    │       └── deg<2 (CC): Straight NC3 → replaces fillet arc
    │
    └── CF Fallback: degenerate bridge shortcuts
         │
Final Cleanup:
    ├── RepairOpenTrimLoops (safety net)
    ├── Remove shared curve (hSharedSC)
    ├── Remove degenerate 0-length trims
    ├── Collapse zero-area sliver surfaces
    └── Remove phantom surfaces (trim.n == 0)
```
