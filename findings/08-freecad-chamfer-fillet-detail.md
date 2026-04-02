# Finding 08: FreeCAD Specific Chamfer/Fillet Implementation Details

## Overview

FreeCAD implements chamfer and fillet as "DressUp" features — operations that modify
an existing solid by processing its edges. There are **two separate module implementations**:
- **Part module** (`src/Mod/Part/`): Standalone chamfer/fillet that operates on any Part solid
- **PartDesign module** (`src/Mod/PartDesign/`): History-based parametric chamfer/fillet that
  chains in a feature tree

Both ultimately delegate ALL geometry computation to **OpenCASCADE** via `BRepFilletAPI_MakeFillet`
and `BRepFilletAPI_MakeChamfer`. This is the fundamental difference from SolveSpace which must
implement this math itself.

---

## Class Hierarchy

### Part Module

```
Part::Feature (base)
  └── Part::FilletBase
        ├── Part::Fillet     (BRepFilletAPI_MakeFillet)
        └── Part::Chamfer    (BRepFilletAPI_MakeChamfer)
```

**FilletBase** (shared base class) holds:
- `Base`: PropertyLink — reference to the source solid
- `Edges`: PropertyFilletEdges — list of `{edgeIndex, radius1, radius2}` structs
- `EdgeLinks`: PropertyLinkSubList — toponaming-aware edge references with shadow subs

### PartDesign Module

```
PartDesign::DressUp (inherits PartDesign::FeatureAddSub)
  ├── PartDesign::Fillet
  └── PartDesign::Chamfer
```

**DressUp** base class holds:
- `Base`: PropertyLinkSub — the base feature + selected sub-elements (edges)
- `SupportTransform`: PropertyBool
- `positionByBaseFeature()`: updates placement from the base feature
- `getContinuousEdges()`: extracts C0-continuous edge chains from selected faces

---

## Part::Chamfer Execute() — Detailed Walkthrough

Source: `src/Mod/Part/App/FeatureChamfer.cpp`

```cpp
App::DocumentObjectExecReturn* Chamfer::execute() {
    // 1. Get the base solid
    TopoShape baseTopoShape = Feature::getTopoShape(link, ...);
    const auto& baseShape = baseTopoShape.getShape();

    // 2. Create OCC chamfer builder on the base solid
    BRepFilletAPI_MakeChamfer mkChamfer(baseShape);

    // 3. Build edge-to-face map (CHAMFER NEEDS A FACE REFERENCE, fillet does not)
    TopTools_IndexedDataMapOfShapeListOfShape mapEdgeFace;
    TopExp::MapShapesAndAncestors(baseShape, TopAbs_EDGE, TopAbs_FACE, mapEdgeFace);

    // 4. Build edge index map
    TopTools_IndexedMapOfShape mapOfEdges;
    TopExp::MapShapes(baseShape, TopAbs_EDGE, mapOfEdges);

    // 5. For each selected edge:
    for (const auto& info : edges) {
        // Resolve edge via toponaming (MappedName -> IndexedName -> index)
        auto id = Data::MappedName(ref.c_str()).toIndexedName().getIndex();
        const TopoDS_Edge& edge = TopoDS::Edge(mapOfEdges.FindKey(id));

        // Get the first adjacent face (chamfer requires face reference)
        const TopoDS_Face& face = TopoDS::Face(mapEdgeFace.FindFromKey(edge).First());

        // Add edge to chamfer builder with two radii (equal for symmetric chamfer)
        mkChamfer.Add(radius1, radius2, TopoDS::Edge(edge), face);
    }

    // 6. Execute OCC chamfer algorithm
    TopoDS_Shape shape = mkChamfer.Shape();

    // 7. Store result with element mapping (for toponaming continuity)
    this->Shape.setValue(res.makeElementShape(mkChamfer, baseTopoShape, Part::OpCodes::Chamfer));

    return Part::FilletBase::execute();  // Validates and stores
}
```

**Key insight for chamfer**: OCC's `BRepFilletAPI_MakeChamfer::Add()` signature requires:
```cpp
void Add(double d1, double d2, const TopoDS_Edge& E, const TopoDS_Face& F);
```
The face reference tells OCC which face the `d1` distance is measured from. This means
chamfer is inherently directional: you specify which adjacent face gets `d1` vs `d2` offset.
FreeCAD has a `FlipDirection` property to swap this.

---

## Part::Fillet Execute() — Detailed Walkthrough

Source: `src/Mod/Part/App/FeatureFillet.cpp`

```cpp
App::DocumentObjectExecReturn* Fillet::execute() {
    // 1. Get base solid
    auto baseShape = baseTopoShape.getShape();

    // 2. Create OCC fillet builder
    BRepFilletAPI_MakeFillet mkFillet(baseShape);

    // 3. Build edge index map (no face map needed for fillet!)
    TopTools_IndexedMapOfShape mapOfEdges;
    TopExp::MapShapes(baseShape, TopAbs_EDGE, mapOfEdges);

    // 4. For each selected edge:
    for (const auto& info : edges) {
        auto id = Data::MappedName(ref.c_str()).toIndexedName().getIndex();
        const TopoDS_Edge& edge = TopoDS::Edge(mapOfEdges.FindKey(id));

        // Fillet only needs the edge + radii (no face reference!)
        mkFillet.Add(radius1, radius2, TopoDS::Edge(edge));
    }

    // 5. Execute OCC fillet algorithm
    TopoDS_Shape shape = mkFillet.Shape();

    // 6. Store with toponaming
    this->Shape.setValue(res.makeElementShape(mkFillet, baseTopoShape, Part::OpCodes::Fillet));

    return Part::FilletBase::execute();
}
```

**Key difference from chamfer**: `mkFillet.Add()` only needs the edge, not a face reference.
The fillet is rotationally symmetric and doesn't require a face to define orientation.

---

## PartDesign::Chamfer — Additional Features

Source: `src/Mod/PartDesign/App/FeatureChamfer.cpp`

PartDesign::Chamfer adds more sophistication on top:

### Chamfer Types

```cpp
const char* ChamferTypeEnums[] = {
    "Equal distance",      // case 0: symmetric, single size param
    "Two distances",       // case 1: asymmetric, size + size2
    "Distance and Angle",  // case 2: distance + angle (0-180°)
    nullptr
};
```

Properties:
- `ChamferType`: enum (0/1/2)
- `Size`: primary distance (always present)
- `Size2`: secondary distance (active only for type 1)
- `Angle`: chamfer angle (active only for type 2, default 45°)
- `FlipDirection`: bool — swaps which face gets d1 vs d2
- `UseAllEdges`: bool — chamfer ALL edges instead of selected ones

The `updateProperties()` method dynamically enables/disables Size2 and Angle UI fields
based on the active ChamferType enum.

The execute() delegates to `shape.makeElementChamfer(...)`:
```cpp
shape.makeElementChamfer(
    TopShape, edges,
    static_cast<Part::ChamferType>(chamferType),
    size, size2, nullptr,
    flipDirection ? Part::Flip::flip : Part::Flip::none
);
```

### Backward Compatibility / Migration

There's explicit migration code for files saved before FreeCAD v1.0 where the
`FlipDirection` semantics changed for types 1 and 2:

```cpp
void Chamfer::migrateFlippedProperties(const Base::XMLReader& reader) {
    if (!requiresSizeSwapping(reader)) return;
    // Flip direction is inverted to maintain same visual result
    FlipDirection.setValue(!FlipDirection.getValue());
}
```

This is exactly the kind of backward-compatibility issue SolveSpace would face
when evolving chamfer parameters.

---

## PartDesign::Fillet — Additional Features

Source: `src/Mod/PartDesign/App/FeatureFillet.cpp`

Properties:
- `Radius`: single constant radius (>0 required)
- `UseAllEdges`: bool — fillet all edges in the solid if true

Uses `shape.makeElementFillet(baseShape, edges, radius, radius)` — note both
`radius1` and `radius2` are the same value (constant radius, no variable fillet).

Has `getContinuousEdges()` call which expands selected faces to include all C0-continuous
edge chains — this is the DressUp base class method that finds tangentially adjacent edges.

The fillet also has a validity check using `BRepAlgo::IsValid()` and tolerance fixing via
`ShapeFix_ShapeTolerance`.

---

## PartDesign::DressUp Base Class

Source: `src/Mod/PartDesign/App/FeatureDressUp.h`

```cpp
class DressUp : public PartDesign::FeatureAddSub {
    App::PropertyLinkSub Base;       // Source feature + edge sub-elements
    App::PropertyBool SupportTransform;

    void positionByBaseFeature();    // Keep placement in sync
    Part::Feature* getBaseObject() const override;

    // Edge extraction helpers:
    void getContinuousEdges(Part::TopoShape, std::vector<TopoDS_Edge>&);
    void getContinuousEdges(Part::TopoShape, std::vector<TopoDS_Edge>&,
                            std::vector<TopoDS_Face>&);  // also returns faces
    std::vector<Part::TopoShape> getContinuousEdges(const TopoShape& shape);
    std::vector<Part::TopoShape> getFaces(const TopoShape& shape);

    void getAddSubShape(Part::TopoShape& addShape, Part::TopoShape& subShape) override;
    void updatePreviewShape() override;
};
```

**The DressUp pattern is key**: The base class stores the reference to the previous solid
and the user's edge selection. The subclass (Chamfer/Fillet) adds only the geometric
parameters. This is exactly analogous to SolveSpace's Group model where `opA` points to
the source group and `valA/valB` store the parameters.

---

## Toponaming Problem — Critical Learning

A major note in both FeatureChamfer.cpp and FeatureFillet.cpp is the "Toponaming project
March 2024" comment:

```cpp
// Toponaming project March 2024: Replaced this code because it wouldn't work:
// TopoDS_Shape edge;
// try {
//     edge = baseTopoShape.getSubShape(ref.c_str());
// }catch(...){}
auto id = Data::MappedName(ref.c_str()).toIndexedName().getIndex();
const TopoDS_Edge& edge = TopoDS::Edge(mapOfEdges.FindKey(id));
```

This shows that FreeCAD spent massive effort (years and multiple full-time developers)
to fix stable edge references. The old approach of "name edges by position" was completely
unreliable. The new approach uses a `MappedName` system that assigns stable persistent IDs.

**For SolveSpace**: This is the #1 risk. Without OpenCASCADE's built-in toponaming,
SolveSpace needs its own approach to stably reference edges across regeneration cycles.
The SolveSpace REMAP mechanism (REMAP_* constants in sketch.h) is the existing hook but
it's not yet used for edges within faces.

---

## Edge Selection in FreeCAD vs SolveSpace

### FreeCAD
- Edges stored as `PropertyFilletEdges` — a list of `{edgeIndex, r1, r2}` structs
- Edge index is an integer position in the topology, FRAGILE without toponaming
- Fixed in 2024 via MappedName/IndexedName system
- UI: 3D viewport click directly selects edge by clicking on it
- "Edge N" naming convention ("Edge1", "Edge2", etc.)

### SolveSpace (proposed equivalent)
- Edges would be selected by clicking on `SEdge`/`SCurve` objects in the 3D viewport
- Each `SCurve` has a unique `hSCurve` handle (similar to entity handles)
- Store selected edge references in the Group struct as `hEntity` or `hSCurve` list
- The REMAP mechanism needs to map old SCurve handles to new ones across regeneration

---

## Key Contrasts: FreeCAD vs SolveSpace

| Aspect | FreeCAD | SolveSpace |
|--------|---------|------------|
| Geometry engine | OpenCASCADE (full BREP kernel) | Custom SShell/SSurface/SCurve |
| Chamfer math | OCC `BRepFilletAPI_MakeChamfer` | Must implement from scratch |
| Fillet math | OCC `BRepFilletAPI_MakeFillet` | Must implement from scratch |
| Edge naming | MappedName → IndexedName (stable) | REMAP constants (partially stable) |
| Integration pattern | DressUp (reference + parameters) | Group (opA + valA/valB) |
| Chamfer types | 3 modes (equal/two dist/dist+angle) | MVP: equal distance only |
| Fillet types | Constant radius | MVP: constant radius only |
| UseAllEdges | Yes, as a property | Not needed for MVP |

---

## What FreeCAD Teaches SolveSpace

### 1. Chamfer requires a face reference; fillet does not
- For chamfer: need to know WHICH adjacent face d1 is measured from
- For fillet: rotationally symmetric, face not needed
- In SolveSpace: for chamfer, store `hEntity faceRef` alongside the edge selection

### 2. The "DressUp" data model is directly transferable
```
FreeCAD DressUp:               SolveSpace analog:
  Base: PropertyLinkSub    →     opA: hGroup (source group)
  Edges: PropertyFilletEdges →   chamferEdges: std::vector<EdgeRef>
  Size: PropertyQuantity   →     valA: double (distance/radius)
  FlipDirection: bool      →     extraParam: int (optional)
```

### 3. Validation is critical
FreeCAD checks both `BRepAlgo::IsValid()` and applies `ShapeFix_ShapeTolerance`.
SolveSpace equivalent: verify watertightness of the resulting SShell (all SCurves
referenced exactly twice).

### 4. Error messages suggest common failure modes
```
"Fillet operation failed. The selected edges may contain geometry that 
cannot be filleted together. Try filleting edges individually or with 
a smaller radius."
```
This reveals: fillets can fail when radius > edge length, or when two adjacent
fillet surfaces would overlap. SolveSpace must handle these cases gracefully.

### 5. "UseAllEdges" is a useful user convenience
FreeCAD allows selecting ALL edges at once. SolveSpace MVP can start with
single-edge selection and add "all edges" as a later enhancement.

---

## OpenCASCADE API Reference (for SolveSpace's math reference)

While SolveSpace won't use OCC, understanding what OCC's API computes helps
define what SolveSpace must implement:

**`BRepFilletAPI_MakeChamfer`** (from OCC documentation):
- Takes a base solid
- For each edge: computes the intersection of two offset planes (one per adjacent face)
  at distances d1 and d2 from the edge
- Inserts a new planar face between the two offset intersection curves
- Trims the original adjacent faces at the offset curves

**`BRepFilletAPI_MakeFillet`** (from OCC documentation):
- Takes a base solid
- For each edge: rolls a sphere of radius r along the edge
- The sphere's center traces a "spine" curve
- The swept sphere creates a canal surface (degenerate case: cylinder for planar faces)
- For planar-planar edges at any angle: exact rational NURBS degree (2,1) representation
- Trims the original adjacent faces at the tangent lines

---

## Summary: What FreeCAD's Implementation Tells Us About SolveSpace's Task

1. **Geometry math is the hard part**: FreeCAD effectively "cheated" by using OCC.
   SolveSpace must implement the actual plane intersection (chamfer) or rolling ball
   (fillet) algorithms itself.

2. **Edge stability is the second-hardest part**: FreeCAD's 2024 Toponaming overhaul
   shows this is a multi-year engineering challenge. SolveSpace needs a simpler
   approach (maybe face-pair identification rather than per-edge index).

3. **The data model is straightforward**: DressUp = Group is a clean mapping.
   The Group's `opA` field for source, `valA/valB` for parameters translates directly.

4. **Chamfer types beyond "equal distance" can be deferred**: MVP is equal-distance chamfer.

5. **Watertightness validation is essential**: FreeCAD uses `BRepAlgo::IsValid()`.
   SolveSpace needs equivalent SCurve topology validation.

6. **The UI workflow is**: select edges in 3D viewport → activate chamfer/fillet command →
   enter parameter → confirm. This matches SolveSpace's existing group creation workflow.

---

## Source Files Examined

- `src/Mod/Part/App/FeatureChamfer.cpp` — Part::Chamfer implementation
- `src/Mod/Part/App/FeatureFillet.cpp` — Part::Fillet implementation
- `src/Mod/Part/App/FeatureChamfer.h` — Part::Chamfer class declaration
- `src/Mod/Part/App/FeatureFillet.h` — Part::Fillet class declaration
- `src/Mod/PartDesign/App/FeatureChamfer.cpp` — PartDesign::Chamfer with ChamferType
- `src/Mod/PartDesign/App/FeatureFillet.cpp` — PartDesign::Fillet with UseAllEdges
- `src/Mod/PartDesign/App/FeatureDressUp.h` — DressUp base class interface
