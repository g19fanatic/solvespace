# Chamfer and Fillet Concepts in 2D/3D CAD

## Sources
- Wikipedia: Chamfer (https://en.wikipedia.org/wiki/Chamfer)
- Wikipedia: Fillet (mechanics) (https://en.wikipedia.org/wiki/Fillet_(mechanics))
- Wikipedia: B-spline (https://en.wikipedia.org/wiki/B-spline)
- Wikipedia: Boundary Representation (search result)

---

## 1. Definitions

### Chamfer
A **chamfer** (/ˈ(t)ʃæmfər/) is a **transitional edge between two faces of an object**. 
- Sometimes defined as a form of bevel
- Most commonly created at a **45° angle** between two adjoining right-angled faces
- Can be created at any angle (e.g., 20° for seals, 30°, 60°)
- In machining: specifically refers to a slope cut at any right-angled edge of a workpiece
- Terminology: "chamfer" and "bevel" are often used interchangeably, but in machining "chamfer" is the specific term

**Types of chamfer:**
- **Standard chamfer**: Equal-leg chamfer at 45° (equal setback on both adjacent surfaces)
- **Unequal chamfer**: Different distances from the edge on each side (e.g., 2mm × 5mm at angle != 45°)
- **Asymmetric chamfer**: One leg much longer than the other
- **2D sketch chamfer**: At a corner between two line segments, creates a new short line segment that cuts the corner
- **3D edge chamfer**: At a 3D solid edge (where two faces meet), creates a new flat planar face (a flat strip along the edge)
- **Vertex/corner chamfer**: At a 3D solid vertex (where three or more edges meet), creates a small triangular or polygonal face

### Fillet
A **fillet** (/ˈfɪlɪt/, "fill it") is a **rounding of an interior or exterior corner of a part**.
- Contrasts with a chamfer (which is a flat angled cut)
- Interior corner fillet = concave rounded surface (stress reduction)
- Exterior corner fillet = convex rounded surface (also called a "round" or "radius")
- A broad radius is known as a "bullnose"

**Types of fillet:**
- **2D sketch fillet**: At a corner between two line segments, inserts an arc tangent to both lines
- **3D edge fillet**: At a 3D solid edge, replaces the sharp edge with a cylindrical or toroidal surface of given radius
- **Variable radius fillet**: The radius varies along the length of the edge
- **Full round fillet**: A face is replaced entirely by a cylindrical surface (when face width = 2 × radius)
- **Chamfer fillet blend**: A transition from a chamfer to a fillet

---

## 2. CAD Terminology by Software

Per Wikipedia (Fillet article):
- **Autodesk Inventor, AutoCAD, Rhino3D, CATIA, FreeCAD, SolidWorks**: Call both concave and convex rounded edges "fillets"; call angled cuts "chamfers"
- **CADKEY, Unigraphics (NX)**: Call rounded edges "blends"
- **PTC Creo/Pro-Engineer**: Calls rounded edges simply "rounds"

---

## 3. Mathematical Representations

### Chamfer Mathematics

#### 2D Sketch Chamfer
Given two line segments meeting at a corner point P:
- Line segment A: from P₀ to P (direction vector d_A)
- Line segment B: from P to P₁ (direction vector d_B)
- Chamfer distance d (equal-leg) or d₁, d₂ (unequal-leg)

**Algorithm:**
1. Find offset point on line A: P_A = P - d × normalize(d_A)
2. Find offset point on line B: P_B = P + d × normalize(d_B)
3. Trim line A to end at P_A
4. Trim line B to start at P_B
5. Create new chamfer segment from P_A to P_B

**Result**: A new line segment P_A–P_B replaces the sharp corner. The chamfer line lies on a plane passing through both offset points.

#### 3D Edge Chamfer
Given two planar faces F1 and F2 meeting at edge E:
- Each face has a surface normal n1, n2
- The dihedral angle θ = π - arccos(n1 · n2) (interior angle of the solid)
- Chamfer setback distance d (equal-leg) means the chamfer face begins at distance d from the edge on each adjacent face

**The chamfer surface is a flat planar strip** along the edge. The chamfer plane:
- Makes an angle of (π - θ)/2 with each adjacent face (for equal-leg chamfer)
- Is bounded by two lines parallel to the edge (the offset lines on each face)
- At vertices where multiple edges meet, requires special "corner" geometry to fill in triangular or polygonal corner patches

**NURBS representation of chamfer surface:**
A chamfer strip is a **ruled surface** — it can be exactly represented as a degree-1 NURBS patch:
- Parameter u along the edge direction
- Parameter v across the strip (linear between the two offset curves)
- The chamfer is a planar rectangular patch for straight edges
- For curved edges, the chamfer becomes a general ruled surface (not necessarily planar)

### Fillet Mathematics

#### 2D Sketch Fillet
Given two line segments meeting at corner P:
- Fillet radius r
- The inscribed circle of radius r is tangent to both line segments
- The tangent points T1, T2 define where the fillet arc meets the lines

**Algorithm:**
1. Find the angle bisector of the two lines at P
2. Find the center C of the inscribed circle along the bisector:
   - Distance from P to C = r / sin(α/2), where α = interior angle at P
3. Find tangent points:
   - T1 = foot of perpendicular from C to line A
   - T2 = foot of perpendicular from C to line B
4. Trim line A to end at T1, trim line B to start at T2
5. Create arc from T1 to T2 centered at C with radius r

**Result**: An arc (ARC_OF_CIRCLE) replaces the sharp corner, tangent to both original lines.

#### 3D Edge Fillet
A 3D edge fillet creates a **smooth surface between two adjacent faces** of a solid.

**Rolling ball algorithm** (standard approach):
- Conceptually: roll a ball of radius r along the edge, keeping it tangent to both adjacent faces simultaneously
- The center of the ball traces a curve called the **spine** (or medial axis)
- The fillet surface is a **canal surface**: the envelope of all positions of the sphere

**Mathematically:**
- For two planar faces (F1 and F2) meeting at angle θ:
  - The center of the rolling ball lies at distance r/sin(θ/2) from the edge
  - The spine is a straight line parallel to the edge (shifted toward interior)
  - The resulting surface is a **cylindrical surface** (partial cylinder) of radius r
  - The cylinder axis = the edge direction
  
- For two curved faces:
  - The spine becomes a curve (not a straight line)
  - The resulting surface is a general **canal surface** = envelope of moving sphere

**NURBS representation of fillet surface:**
- **Cylinder** (most common case): Can be represented exactly as a NURBS patch
  - A cylindrical surface of radius r is a degree-2 NURBS patch with weights 1 and sqrt(2)/2
  - The exact NURBS representation uses 9 control points for a quarter cylinder
  - This is the standard result when two planar faces meet at 90°
- **General canal surface**: Requires NURBS approximation (not exact)
- **Torus** (for edge connecting a cylindrical face): Can be represented exactly as NURBS

---

## 4. B-rep (Boundary Representation) Context

Solids in B-rep (used by SolveSpace) consist of:
- **Vertices**: Points where 3+ edges meet
- **Edges**: Curves where 2 faces meet
- **Faces**: Surfaces bounded by edges
- **Half-edges**: Directed edges with orientation (used in many B-rep kernels)

In B-rep context:
- A **chamfer operation** adds new faces (chamfer strips) and modifies adjacent faces
- A **fillet operation** adds new faces (cylindrical/canal surfaces) and modifies adjacent faces

The result in both cases is a topologically different solid (more faces, more edges, more vertices).

---

## 5. Types in Context of 2D Sketch vs 3D Solid

### 2D Sketch Operations
| Operation | Input | Output | Geometry |
|-----------|-------|--------|----------|
| Sketch Chamfer | Two line segments at a corner | New line segment + trimmed lines | LINE_SEGMENT |
| Sketch Fillet | Two line segments at a corner | Arc + trimmed lines | ARC_OF_CIRCLE |

### 3D Solid Operations
| Operation | Input | Output | Geometry |
|-----------|-------|--------|----------|
| 3D Edge Chamfer | Edge of solid + distance | New flat face + modified adjacent faces | Ruled NURBS surface |
| 3D Edge Fillet | Edge of solid + radius | New cylindrical face + modified adjacent faces | Cylindrical NURBS surface |

---

## 6. Engineering Uses

### Chamfer Uses
1. **Ease assembly**: Chamfered holes and shaft ends make insertion easier (bolts, pins, shafts)
2. **Safety**: Remove sharp edges that cause cuts/injuries
3. **Stress reduction**: Less effective than fillets for stress concentration
4. **Manufacturing**: Easier to inspect than radius; required for interference fits
5. **Aesthetic**: Decorative effect in architecture, furniture
6. **Corrosion prevention**: Eliminates thin sharp edges susceptible to rust

### Fillet Uses  
1. **Stress concentration reduction**: Primary use in load-bearing parts; distributes stress over broader area making parts more durable
2. **Aerodynamics**: Reduces interference drag at aircraft component junctions (wing-fuselage)
3. **Manufacturing**: Concave fillets allow round-tipped end mills
4. **Safety**: Eliminates sharp edges
5. **Aesthetics**: Smooth flowing appearance

---

## 7. Key Properties and Constraints

### Chamfer Parameters
- **Equal-leg chamfer**: Single parameter d (the setback distance on both sides)
- **Unequal-leg chamfer**: Two parameters d1 (setback on face 1) and d2 (setback on face 2)
- **By angle and distance**: Distance d and angle α (e.g., 1mm × 45°, or 2mm × 30°)

### Fillet Parameters
- **Constant radius fillet**: Single parameter r
- **Variable radius fillet**: r(t) function along edge curve parameter t
- **Start/end radius**: r_start and r_end (linearly interpolated)

---

## 8. CAD-Specific Notes for SolveSpace

SolveSpace uses B-rep with NURBS surfaces (SSurface/SShell). In this context:

### 2D Sketch Fillet (simplest case)
- Already partially supported via ARC_OF_CIRCLE entities
- Would need: detect corner between two sketch entities, insert arc tangent to both
- Math: inscribed circle algorithm

### 3D Edge Chamfer
- Requires identifying edges of SShell (SSurface boundary curves, SCurves)
- Creates a new planar SSurface (chamfer face)
- Boolean: union the solid with the chamfer face geometry (or difference with chamfer prism)
- Actually implemented as: trim adjacent faces and add new chamfer face

### 3D Edge Fillet
- Requires identifying edges of SShell
- Creates new cylindrical SSurface (for planar face-pair) or canal SSurface (general)
- NURBS cylinder: degree 2 × degree 1, 9 control points for 90° arc
- Boolean: complex — trim adjacent faces and add fillet surface(s)

---

## Summary of Key Findings

1. **Chamfer** = flat angled cut. In 3D = new flat strip face along an edge. Parametrized by distance(s) or angle+distance.

2. **Fillet** = curved blend. In 3D = cylindrical/toroidal/canal surface along an edge. Parametrized by radius.

3. **Both operations are topological changes**: They add new faces, edges, and vertices to a B-rep solid.

4. **Chamfer is mathematically simpler**: Results in a planar (degree 1) surface, exact NURBS.

5. **Fillet results in quadratic surfaces**: Cylinder or torus, representable as degree-2 NURBS with rational weights (√2/2 for circular arcs).

6. **2D sketch operations are fundamentally different from 3D solid operations**: Different algorithms, different complexity, different SolveSpace integration points.

7. **The most common fillet surface (90° edge between two flat faces)** is a quarter-cylinder, which SolveSpace's SSurface system already supports for extrusion operations.
