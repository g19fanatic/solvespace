# OCCT Research Findings: How OpenCASCADE Handles Fillet/Chamfer Corner Junctions

## Source Files Analyzed

1. **ChFi3d_Builder.hxx** — Main class header (56KB)
2. **ChFi3d_Builder_6.cxx** — CompleteData/ComputeData implementations (89KB)
3. **ChFi3d_Builder_CnCrn.cxx** — Corner handling: `PerformMoreThreeCorner` (134KB)
4. **ChFi3d_Builder_C1.cxx** — `PerformOneCorner` (177KB)
5. **ChFi3d_Builder_C2.cxx** — `PerformTwoCorner` (29KB)
6. **ChFi3d_FilBuilder_C3.cxx** — Fillet-specific 3-corner (43KB)
7. **ChFi3d_ChBuilder_C3.cxx** — Chamfer-specific 3-corner (39KB)

## Key Architecture: OCCT Corner Handling Hierarchy

OCCT handles corners (vertices where fillet/chamfer stripes meet) with a hierarchy of methods:

| Method | Stripes at Vertex | Description |
|--------|-------------------|-------------|
| `PerformOneCorner` | 1 | Extends single stripe to vertex |
| `PerformTwoCorner` | 2 | Two stripes meeting: uses intersection |
| `PerformThreeCorner` | 3 | Three stripes meeting at vertex |
| `PerformMoreThreeCorner` | N≥3 | General N-corner: uses **GeomPlate** |
| `PerformTwoCornerbyInter` | 2 (same extension) | Intersection of two stripe surfaces |

## Critical Finding: `PerformTwoCornerSameExt` — Surface-Surface Intersection

Found in `ChFi3d_Builder_CnCrn.cxx` (line ~1680 in the fetched source):

```cpp
static void PerformTwoCornerSameExt(TopOpeBRepDS_DataStructure& DStr,
                                     const Handle(ChFiDS_Stripe)& stripe1,
                                     const int index1, const int sens1,
                                     const Handle(ChFiDS_Stripe)& stripe2,
                                     const int index2, const int sens2,
                                     bool& trouve)
{
    // ...
    Handle(Geom_Curve) cint;
    Handle(Geom2d_Curve) C2dint1, C2dint2;
    
    // Get the two stripe surfaces
    Handle(GeomAdaptor_Surface) HS1 = ChFi3d_BoundSurf(DStr, Fd1, 1, 2);
    Handle(GeomAdaptor_Surface) HS2 = ChFi3d_BoundSurf(DStr, Fd2, 1, 2);
    
    // COMPUTE SURFACE-SURFACE INTERSECTION
    trouve = false;
    if (ChFi3d_ComputeCurves(HS1, HS2, Pardeb, Parfin, cint, C2dint1, C2dint2, 1.e-4, 1.e-5, tol))
    {
        // Verify intersection endpoints match CommonPoints
        cint->D0(cint->FirstParameter(), P1);
        cint->D0(cint->LastParameter(), P2);
        trouve = ((Com11.Point().Distance(P1) < 1.e-4 || ...)
                 && (Com12.Point().Distance(P1) < 1.e-4 || ...));
    }
    
    if (trouve) {
        // Store intersection curve as shared boundary between surfaces
        TopOpeBRepDS_Curve tcurv3d(cint, tol);
        indcurve = DStr.AddCurve(tcurv3d);
        
        // Add curve as interference on BOTH surfaces
        Interfc = ChFi3d_FilCurveInDS(indcurve, indic1, C2dint1, orpcurve);
        DStr.ChangeSurfaceInterferences(indic1).Append(Interfc);
        
        Interfc = ChFi3d_FilCurveInDS(indcurve, indic2, C2dint2, orpcurve);
        DStr.ChangeSurfaceInterferences(indic2).Append(Interfc);
    }
}
```

**THIS IS THE KEY**: OCCT computes the **surface-surface intersection** between two fillet/chamfer stripes, producing:
- A 3D intersection curve (`cint`)
- 2D parametric curves on each surface (`C2dint1`, `C2dint2`)
- The intersection curve is stored as a **shared boundary** between both surfaces

**NO separate corner surface is created.**

## Critical Finding: `PerformMoreThreeCorner` — GeomPlate for N-corners

When there are more than 2 stripes at a vertex (N-corner), OCCT uses **GeomPlate** surface interpolation:

```cpp
void ChFi3d_Builder::PerformMoreThreeCorner(const int Jndex, const int nconges)
{
    // 1. Parse all edges and stripes around vertex
    // 2. For each pair of adjacent stripes:
    //    - Try ChFi3d_SearchFD for intersection
    //    - If no intersection: compute connecting curve (batten/projection/straight line)
    // 3. Build GeomPlate_BuildPlateSurface with all boundary curves as constraints
    // 4. Approximate the plate surface with GeomPlate_MakeApprox
    // 5. Store plate surface in DS with curve interferences on all adjacent surfaces
    
    GeomPlate_BuildPlateSurface PSurf(degree, nbcurvpnt, nbiter, tol2d, tolapp3d, angular);
    
    // Add curve constraints from stripes
    for (ic = 0; ic < nedge; ic++) {
        if (!sharp.Value(ic)) {
            // Compute curve on stripe surface
            Handle(GeomPlate_CurveConstraint) Cont = 
                new GeomPlate_CurveConstraint(HCons, Order, ...);
            PSurf.Add(Cont);
        }
    }
    
    // Add connecting curves between non-intersecting pairs
    // (battens, projections, or straight lines)
    
    PSurf.Perform();  // Solve the plate surface
    
    // Approximate with BSpline
    GeomPlate_MakeApprox Mapp(gpPlate, critere, tolapp, ...);
    Handle(Geom_BSplineSurface) Surf(Mapp.Surface());
}
```

**Key insight**: When a smooth (GeomPlate) corner surface IS created, it's a **smooth interpolating surface** (not a flat triangle). It's bounded by the actual curves from the adjacent fillet/chamfer surfaces.

## Critical Finding: When Two Stripes Meet (The Relevant Case for SolveSpace)

In the case of **chamfer + fillet meeting at a shared vertex** (which is the SolveSpace bug):

1. **OCCT identifies this as a 2-stripe corner** (one chamfer stripe, one fillet stripe)
2. **Calls `PerformTwoCornerbyInter`** or `PerformTwoCornerSameExt`
3. **Computes the surface-surface intersection** of the fillet surface and chamfer surface
4. **The intersection curve becomes the shared trim boundary**
5. **NO separate corner surface is created**

For the specific geometry:
- Fillet surface = cylindrical surface (rolling ball)
- Chamfer surface = flat planar surface
- Intersection of cylinder ∩ plane = **elliptical arc** (a conic section)

## OCCT's `ChFi3d_ComputeCurves` Function

This is the workhorse that computes surface-surface intersection:
- Takes two surfaces (HS1, HS2)
- Takes parameter bounds (Pardeb, Parfin) as starting/ending 2D points on each surface
- Returns:
  - `cint` — 3D intersection curve
  - `C2dint1` — 2D curve on surface 1 (parametric representation)
  - `C2dint2` — 2D curve on surface 2 (parametric representation)
  - `tol` — achieved tolerance

## Summary: How OCCT Avoids the Corner Triangle

| Step | OCCT Approach | SolveSpace (Current, Wrong) |
|------|---------------|----------------------------|
| 1 | Identifies stripes meeting at vertex | Identifies bridges at shared V1 |
| 2 | Computes surface-surface intersection | Creates flat FromPlane surface |
| 3 | Intersection curve = shared trim | NC3 linear curve replaces fillet arc |
| 4 | Both surfaces trimmed by intersection | Corner triangle + NC1/NC2/NC3 |
| 5 | No separate corner surface | Separate visible flat surface |

## Implications for SolveSpace Fix

The OCCT approach confirms:
1. **The corner triangle IS NOT topologically necessary** — OCCT doesn't create one for 2-stripe corners
2. **Surface-surface intersection is the correct approach** — compute fillet cylinder ∩ chamfer plane
3. **The intersection curve (elliptical arc) is the shared trim boundary**
4. **Both surfaces (fillet and chamfer) are trimmed by this intersection curve**
5. **For N>2 corners, a smooth plate surface (NOT flat triangle) can be used if needed**

## Alternative Simpler Approach (for SolveSpace)

Given SolveSpace's architecture where the fillet arc `hArcV1` already has `surfB = hCapSurfV1` (meaning the fillet arc already references the chamfer surface as its boundary partner), the simplest approach may be:

**APPROACH A: Skip the corner triangle entirely.**
- The fillet's arc (hArcV1) already serves as the boundary between fillet and chamfer surfaces
- `hArcV1.surfB = hCapSurfV1` means the arc ALREADY defines where fillet meets chamfer
- Don't create SSurface::FromPlane, don't create NC3, don't replace the arc
- Just keep NC1 and NC2 (which connect to the adjacent cap faces)
- The fillet surface keeps its arc, the chamfer surface is trimmed by it

If APPROACH A leaves topological gaps:

**APPROACH B: Compute the actual intersection.**
- Intersect the fillet cylinder surface with the chamfer plane
- This gives an elliptical arc trim curve
- Use this as the shared boundary between fillet and chamfer surfaces
- Similar to OCCT's `ChFi3d_ComputeCurves` approach
