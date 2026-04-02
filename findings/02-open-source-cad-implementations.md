# Open-Source CAD Chamfer/Fillet Implementations

## Research Question
How do other open-source CAD tools implement chamfers/fillets? (FreeCAD, OpenSCAD, LibreCAD, BRL-CAD)

## Sources Examined
- FreeCAD source code: `src/Mod/Part/App/FeatureFillet.cpp`
- FreeCAD source code: `src/Mod/Part/App/FeatureChamfer.cpp`
- FreeCAD source code: `src/Mod/PartDesign/App/FeatureFillet.cpp`
- FreeCAD wiki: https://wiki.freecad.org/Part_Fillet
- FreeCAD wiki: https://wiki.freecad.org/Part_Chamfer

---

## 1. FreeCAD — The Most Complete Open-Source Implementation

FreeCAD has **two distinct implementations** of chamfer/fillet: one in the Part workbench and another in PartDesign. This distinction is architecturally significant.

### 1a. FreeCAD Part Module (Part::Fillet and Part::Chamfer)

**Architecture**: Both inherit from a common `Part::FilletBase` base class.

**Key Data Model (`FilletBase`)**:
- `Base` property: link to the input shape (`App::DocumentObject*`)
- `Edges` property: vector of `Part::FilletElement` — each element stores:
  - `radius1` (double) — fillet radius or chamfer start length
  - `radius2` (double) — chamfer end length (asymmetric chamfer)
- `EdgeLinks` property: topological edge references (using FreeCAD's toponaming system)

**FreeCAD Part Fillet `execute()` function (complete algorithm)**:
```cpp
App::DocumentObjectExecReturn* Fillet::execute() {
    // 1. Get the base shape via Feature::getTopoShape()
    TopoShape baseTopoShape = Feature::getTopoShape(link, ...);
    auto baseShape = baseTopoShape.getShape();  // TopoDS_Shape (OpenCASCADE)

    // 2. Create the OpenCASCADE fillet builder
    BRepFilletAPI_MakeFillet mkFillet(baseShape);

    // 3. Build edge index map (indexed by integer ID for stability post-toponam)
    TopTools_IndexedMapOfShape mapOfEdges;
    TopExp::MapShapes(baseShape, TopAbs_EDGE, mapOfEdges);

    // 4. For each edge in the Edges property:
    for (const auto& info : edges) {
        // Resolve edge by topological name -> integer index
        auto id = Data::MappedName(ref.c_str()).toIndexedName().getIndex();
        const TopoDS_Edge& edge = TopoDS::Edge(mapOfEdges.FindKey(id));

        // Apply fillet with two radii (allows variable-radius fillet)
        double radius1 = info.radius1;
        double radius2 = info.radius2;
        mkFillet.Add(radius1, radius2, TopoDS::Edge(edge));
    }

    // 5. Build the shape
    TopoDS_Shape shape = mkFillet.Shape();

    // 6. Store result with topology mapping for downstream references
    this->Shape.setValue(res.makeElementShape(mkFillet, baseTopoShape, 
                                              Part::OpCodes::Fillet));
}
```

**FreeCAD Part Chamfer `execute()` function (key difference from fillet)**:
```cpp
App::DocumentObjectExecReturn* Chamfer::execute() {
    // SAME setup as Fillet, but:
    
    // 1. Need EDGE-to-FACE mapping for chamfer (OCC requirement)
    TopTools_IndexedDataMapOfShapeListOfShape mapEdgeFace;
    TopExp::MapShapesAndAncestors(baseShape, TopAbs_EDGE, TopAbs_FACE, mapEdgeFace);
    
    // 2. Use BRepFilletAPI_MakeChamfer (not MakeFillet)
    BRepFilletAPI_MakeChamfer mkChamfer(baseShape);
    
    // 3. For each edge, need to also provide an adjacent face
    const TopoDS_Face& face = TopoDS::Face(mapEdgeFace.FindFromKey(edge).First());
    mkChamfer.Add(radius1, radius2, TopoDS::Edge(edge), face);
    
    // 4. Store with Chamfer opcode
    this->Shape.setValue(res.makeElementShape(mkChamfer, baseTopoShape, 
                                              Part::OpCodes::Chamfer));
}
```

**Critical Insight**: FreeCAD Part's chamfer needs an adjacent **face reference** for the chamfer API, whereas fillet does not. This is an OpenCASCADE API difference: `BRepFilletAPI_MakeChamfer::Add()` requires `(d1, d2, edge, face)` while `BRepFilletAPI_MakeFillet::Add()` takes `(r1, r2, edge)`.

**What OpenCASCADE (OCC) does under the hood**:
- Both `BRepFilletAPI_MakeFillet` and `BRepFilletAPI_MakeChamfer` are OCC kernel operations
- OCC handles all the geometry: computing the fillet surface (cylindrical/toroidal), trimming adjacent faces, generating the new B-rep topology
- FreeCAD Part fillet/chamfer is essentially **a thin wrapper** over OCC's kernel algorithms
- FreeCAD does NOT implement the geometry itself — it delegates entirely to OCC

### 1b. FreeCAD PartDesign Fillet

The PartDesign fillet is fundamentally different from Part fillet:

**Architecture**: Inherits from `PartDesign::DressUp` (not `Part::FilletBase`)

**Properties**:
```cpp
ADD_PROPERTY_TYPE(Radius, (1.0), "Fillet", App::Prop_None, "Fillet radius.");
ADD_PROPERTY_TYPE(UseAllEdges, (false), "Fillet", App::Prop_None, 
    "Fillet all edges if true...");
```

**Key differences from Part Fillet**:
1. **Single radius** (not per-edge): One `Radius` property applies to all selected edges
2. **UseAllEdges option**: Can automatically fillet ALL edges of the base shape
3. **DressUp pattern**: Takes the "most recent active Body feature" as input (parametric chain)
4. **getContinuousEdges()**: Smart edge grouping — selects edge chains that flow naturally
5. **Validity check**: Uses `BRepAlgo::IsValid()` to verify result
6. **Shape refinement**: `refineShapeIfActive()` cleans up artifacts
7. **Single solid rule**: Ensures result stays a single solid (required by PartDesign workflow)

**PartDesign Fillet execute() algorithm**:
```cpp
App::DocumentObjectExecReturn* Fillet::execute() {
    // 1. Get base shape from DressUp chain
    Part::TopoShape baseShape = getBaseTopoShape();
    
    // 2. Get edges: either all or selected continuous chains
    auto edges = UseAllEdges.getValue() 
        ? baseShape.getSubTopoShapes(TopAbs_EDGE)
        : getContinuousEdges(baseShape);
    
    // 3. Apply fillet via makeElementFillet (not direct OCC call)
    TopoShape shape(0);
    shape.makeElementFillet(baseShape, edges, Radius.getValue(), Radius.getValue());
    
    // 4. Validate, refine, store
    this->rawShape = shape;          // store before refinement
    shape = refineShapeIfActive(shape);
    this->Shape.setValue(shape);
}
```

### 1c. FreeCAD User-Facing Workflow

**Part Fillet/Chamfer workflow**:
1. User selects one or more edges in 3D view
2. Invokes Part → Fillet (or Chamfer) menu
3. Task panel opens: "Fillet Edges"
4. User picks which specific edges get filleted (each with own radius)
5. Press OK → `execute()` runs

**PartDesign Fillet/Chamfer workflow**:
1. Must be inside an active Body
2. Select the feature to dress up
3. Set single radius (applies uniformly)
4. Optionally: "Use all edges" toggle

**Chamfer options in FreeCAD**:
- Equal-leg: `d1 == d2` (symmetric chamfer at 45°)
- Two-leg: `d1 != d2` (asymmetric chamfer, custom angle)
- Select by edge or by face (face selection auto-selects all bordering edges)

---

## 2. OpenSCAD

OpenSCAD does **NOT have native chamfer/fillet** in its standard operations. It is a CSG-based tool, not a B-rep tool, so there is no edge-detection concept.

**Workarounds in OpenSCAD**:
- **Minkowski sum**: `minkowski() { cube(...); sphere(r=fillet_r); }` — adds a rounded offset to all edges, but this is computationally expensive (O(n²)) and applies to ALL edges uniformly
- **Hull trick**: For simple cases, use `hull()` on translated primitives
- **2D fillet**: Round off 2D polygon corners before extruding: offset → offset(-r) → linear_extrude
- **Third-party libraries**: e.g., BOSL2 library has `fillet()`, `chamfer_edge()` helpers that use boolean CSG subtractions/unions

**OpenSCAD chamfer approach** (manual):
```scad
// Cut a chamfer at a vertical edge by subtracting a rotated prism
module chamfer_edge(len, chamfer_size) {
    rotate([0, 0, 45]) cube([chamfer_size * sqrt(2), chamfer_size * sqrt(2), len], 
                             center=true);
}
```

**Key Insight**: OpenSCAD's CSG approach means it cannot introspect B-rep topology. There are no "edges" as first-class objects. This is fundamentally different from SolveSpace and FreeCAD.

---

## 3. LibreCAD (2D CAD)

LibreCAD is a **2D CAD tool** — chamfer and fillet are 2D sketch operations.

**LibreCAD fillet**:
- Menu: Modify → Fillet
- Algorithm: Given two intersecting lines (or line and arc), compute an inscribed arc tangent to both
- Parameters: Single radius value
- Implementation: Classical 2D geometry — find the center of the fillet arc by offsetting both lines by radius r, intersecting the offset lines, then drawing the arc

**LibreCAD chamfer**:
- Menu: Modify → Chamfer
- Two modes: (a) equal lengths, (b) two different lengths, (c) angle+length
- Algorithm: Trim both lines to a point at distance d along each, connect with a new line segment

**LibreCAD source reference**: `src/lib/actions/rs_actionmodifyfillet.cpp` and `rs_actionmodifychamfer.cpp`

The approach is:
1. User clicks first line, then second line
2. System detects intersection point
3. For fillet: Trim both lines to the tangent points, insert ARC entity
4. For chamfer: Trim both lines, insert LINE entity

**Key Insight**: LibreCAD's 2D approach is the **analogue of what SolveSpace would do for sketch-level fillet/chamfer**. This is simpler than the 3D problem.

---

## 4. BRL-CAD

BRL-CAD is a US Army-developed solid modeling system using a **CSG tree representation**.

**BRL-CAD fillet/chamfer approach**:
- No native "fillet edge" operation in the traditional B-rep sense
- Uses CSG: subtract a cylinder or torus at an edge to round it
- `r` command creates rounded corners via explicit geometric primitives
- Advanced users create macro scripts that approximate filleted edges using small cylinders

**Key Insight**: BRL-CAD's CSG approach, like OpenSCAD, means fillets are approximations via shape operations, not topological edge operations.

---

## 5. Comparison Table

| Tool | Approach | Edge Selection | Geometry Backend | Native Support |
|------|----------|---------------|-----------------|----------------|
| FreeCAD Part | B-rep DressUp | Per-edge reference | OpenCASCADE | ✅ Full |
| FreeCAD PartDesign | B-rep DressUp | Uniform radius | OpenCASCADE | ✅ Full |
| OpenSCAD | CSG | None (no edges) | CGAL | ❌ Workarounds only |
| LibreCAD | 2D sketch | Line pair selection | Custom 2D | ✅ 2D only |
| BRL-CAD | CSG | None (no edges) | Custom | ❌ Workarounds only |
| SolveSpace | B-rep+CSG mesh | ??? (not yet) | Custom NURBS | ❌ Not implemented |

---

## 6. Key Lessons for SolveSpace Implementation

### What FreeCAD teaches us:

1. **Two-level architecture**: Part-level fillet (shape modification) vs. PartDesign-level (history-based chain). SolveSpace uses a Group/history model more like PartDesign.

2. **Edge identity is critical**: FreeCAD invested heavily in the "Toponaming" project (2024) to fix stable edge references. SolveSpace will need a similar edge identity solution — edges in an SShell change when the upstream geometry changes.

3. **OpenCASCADE does the geometry**: FreeCAD does not compute fillet curves itself — OCC's `BRepFilletAPI_MakeFillet` does everything. SolveSpace **does not have OCC** and has its own custom NURBS kernel, so SolveSpace would need to implement the geometry math itself OR integrate OCC as a dependency.

4. **Data model**: FreeCAD stores per-edge parameters (radius1, radius2) with edge references. SolveSpace would need similar data structures.

5. **Chamfer requires face reference**: The OCC chamfer API requires knowing which face the chamfer "leans toward." This means edge selection must capture face adjacency information.

6. **PartDesign DressUp pattern**: The "DressUp" concept — take a solid, modify its edges, return a new solid — is exactly what SolveSpace's Group model implements. A new `Group::Type::FILLET` would be a DressUp group.

### What LibreCAD teaches us for the 2D case:

1. The 2D fillet (inscribed arc) is much simpler than 3D
2. It requires: two intersecting entities (lines/arcs), a radius, and produces an arc + trimmed lines
3. SolveSpace's sketch mode should support this as a sketch operation (not a 3D group)

### What OpenSCAD/BRL-CAD teach us:

1. Without a B-rep kernel, native fillet is impossible — CSG tools hack around it
2. SolveSpace IS a B-rep tool (SShell = B-rep), so it CAN support real fillets
3. But SolveSpace's geometry generation pipeline must be extended to support the math

---

## 7. Summary of Open-Source Approaches

The spectrum of approaches:
- **Full B-rep + dedicated geometry kernel (OCC)**: FreeCAD — most powerful, delegates geometry to OCC
- **Full B-rep + custom kernel**: What SolveSpace would need — must implement its own fillet/chamfer surface generation
- **CSG with workarounds**: OpenSCAD, BRL-CAD — no real fillets
- **2D sketch tool**: LibreCAD — simple but limited scope

**SolveSpace's situation**: It has a full B-rep system (SShell/SSurface) but no OCC dependency. It must implement the geometry itself. This is significantly harder than FreeCAD's approach, which is essentially "delegate to OCC." SolveSpace must generate the fillet/chamfer surfaces (cylindrical NURBS patches), trim existing faces, and maintain topology consistency.

The key insight from FreeCAD PartDesign Fillet: the "DressUp" pattern fits naturally with SolveSpace's Group model. A `Group::Type::FILLET` would:
1. Reference a previous Group's SShell
2. Store edge selections and radius
3. In `Generate()`, compute the modified SShell with fillet surfaces

This is architecturally very similar to how `Group::Type::EXTRUDE` takes a 2D sketch group and produces a 3D solid.
