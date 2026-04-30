//-----------------------------------------------------------------------------
// Geometry verification tests for chamfer/fillet (stale vertices, setback
// points, curve endpoints, surface associations, curve counts).
//-----------------------------------------------------------------------------
#include "helpers.h"


//-----------------------------------------------------------------------------
// Task 11: Stale-vertex regression tests.
//
// After a chamfer or fillet, the original shared-edge endpoint vertices V1 and
// V2 must NOT appear as terminal points (pts[0] or pts[n-1]) of any SCurve in
// the result shell.  Before the TruncateCurveAtVertex fix, InsertPointIntoCurvePts
// inserted the new setback point but left the old corner vertex at the tail of
// the pts array, causing ghost "green dots" / spurious display vertices.
//
// For the 20x20x20 box, face[0] (front, Y=0) and face[1] (right, X=20) share
// the vertical edge at X=20, Y=0 whose endpoints are:
//   V1 = (20,  0,  0)
//   V2 = (20,  0, 20)
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_no_stale_vertices) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity edge = FindEdgeBetweenFaces(extrudeH, face1, face2);
    CHECK_TRUE(edge.v != 0);
    if(!edge.v) return;

    hGroup chamferH = AddChamferGroupByEdge(extrudeH, edge, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // The original shared-edge endpoints before the chamfer.
    // After the chamfer these must NOT appear as terminal points of any curve.
    Vector V1 = Vector::From(20,  0,  0);
    Vector V2 = Vector::From(20,  0, 20);

    for(auto &sc : g->runningShell.curve) {
        if(sc.pts.n == 0) continue;
        Vector start = sc.pts[0].p;
        Vector end   = sc.pts[sc.pts.n - 1].p;
        CHECK_FALSE(start.Equals(V1));
        CHECK_FALSE(end.Equals(V1));
        CHECK_FALSE(start.Equals(V2));
        CHECK_FALSE(end.Equals(V2));
    }
}

//-----------------------------------------------------------------------------
// Task 12 — New TDD tests for the reported behavior issues.
// These tests check:
//   (a) Setback points A,B,D,C exist as terminal points of SCurves after chamfer
//   (b) Terminal SCurvePts at setback points have vertex=true
//   (c) No STrimBy.start/finish in any surface still references V1 or V2
//   (d) The 4 new chamfer boundary curves each have the chamfer surface as one side
//   (e) Setback points exist as terminals after fillet
//-----------------------------------------------------------------------------

// (a) After chamfer dist=2.0, setback points A=(18,0,0), B=(18,0,20),
//     D=(20,2,0), C=(20,2,20) each appear as the first or last pts[].p
//     of at least one SCurve in g->runningShell.curve.
TEST_CASE(chamfer_setback_points_are_curve_endpoints) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity edge = FindEdgeBetweenFaces(extrudeH, face1, face2);
    CHECK_TRUE(edge.v != 0);
    if(!edge.v) return;

    hGroup chamferH = AddChamferGroupByEdge(extrudeH, edge, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // The CreateBoxExtrude() box creates edges that go from top (Z=80) to
    // bottom (Z=0). The shared edge between face[0] (front Y=0) and face[1]
    // (right X=20) runs from V1=(20,0,80) [top] to V2=(20,0,0) [bottom].
    // After orientation normalization (SWAP occurs for this geometry):
    //   d1=(0,1,0) [inward of right face], d2=(-1,0,0) [inward of front face]
    // With dist=2.0, setback points are:
    Vector P1 = Vector::From(20, 2, 80);  // V1 setback on right face
    Vector P2 = Vector::From(20, 2,  0);  // V2 setback on right face
    Vector P3 = Vector::From(18, 0, 80);  // V1 setback on front face
    Vector P4 = Vector::From(18, 0,  0);  // V2 setback on front face

    bool foundP1 = false, foundP2 = false, foundP3 = false, foundP4 = false;
    for(auto &sc : g->runningShell.curve) {
        if(sc.pts.n < 2) continue;
        Vector s = sc.pts[0].p;
        Vector e = sc.pts[sc.pts.n - 1].p;
        if(s.Equals(P1) || e.Equals(P1)) foundP1 = true;
        if(s.Equals(P2) || e.Equals(P2)) foundP2 = true;
        if(s.Equals(P3) || e.Equals(P3)) foundP3 = true;
        if(s.Equals(P4) || e.Equals(P4)) foundP4 = true;
    }

    CHECK_TRUE(foundP1);   // (20,2,80) is a curve terminal
    CHECK_TRUE(foundP2);   // (20,2,0) is a curve terminal
    CHECK_TRUE(foundP3);   // (18,0,80) is a curve terminal
    CHECK_TRUE(foundP4);   // (18,0,0) is a curve terminal
}

// (b) After chamfer, for every SCurve whose first or last pts[].p equals
//     one of A/B/D/C, the SCurvePt.vertex flag must be true.
//     (MakePwlInto sets vertex=true for i==0 and i==lv.n-1, so this should pass.)
TEST_CASE(chamfer_setback_points_have_vertex_flag) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity edge = FindEdgeBetweenFaces(extrudeH, face1, face2);
    CHECK_TRUE(edge.v != 0);
    if(!edge.v) return;

    hGroup chamferH = AddChamferGroupByEdge(extrudeH, edge, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Actual setback points (Z=80 for top, Z=0 for bottom in this test geometry)
    Vector P1 = Vector::From(20, 2, 80);  // top setback on right face
    Vector P2 = Vector::From(20, 2,  0);  // bottom setback on right face
    Vector P3 = Vector::From(18, 0, 80);  // top setback on front face
    Vector P4 = Vector::From(18, 0,  0);  // bottom setback on front face

    for(auto &sc : g->runningShell.curve) {
        if(sc.pts.n < 2) continue;
        bool firstIsSetback = sc.pts[0].p.Equals(P1) || sc.pts[0].p.Equals(P2) ||
                              sc.pts[0].p.Equals(P3) || sc.pts[0].p.Equals(P4);
        bool lastIsSetback  = sc.pts[sc.pts.n-1].p.Equals(P1) ||
                              sc.pts[sc.pts.n-1].p.Equals(P2) ||
                              sc.pts[sc.pts.n-1].p.Equals(P3) ||
                              sc.pts[sc.pts.n-1].p.Equals(P4);
        if(firstIsSetback) {
            CHECK_TRUE(sc.pts[0].vertex);
        }
        if(lastIsSetback) {
            CHECK_TRUE(sc.pts[sc.pts.n-1].vertex);
        }
    }
}

// (c) After chamfer, no STrimBy.start or STrimBy.finish in any surface
//     of g->runningShell should equal V1=(20,0,0) or V2=(20,0,20).
//     The TruncateCurveAtVertex + STrimBy update code in Steps 12/13/15
//     should replace all such references with the setback points.
TEST_CASE(chamfer_neighboring_curves_end_at_setback) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity edge = FindEdgeBetweenFaces(extrudeH, face1, face2);
    CHECK_TRUE(edge.v != 0);
    if(!edge.v) return;

    hGroup chamferH = AddChamferGroupByEdge(extrudeH, edge, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Original shared-edge endpoints — must not appear in any trim boundary.
    Vector V1 = Vector::From(20, 0,  0);
    Vector V2 = Vector::From(20, 0, 20);

    for(auto &ss : g->runningShell.surface) {
        for(auto &stb : ss.trim) {
            CHECK_FALSE(stb.start.Equals(V1));
            CHECK_FALSE(stb.start.Equals(V2));
            CHECK_FALSE(stb.finish.Equals(V1));
            CHECK_FALSE(stb.finish.Equals(V2));
        }
    }
}

// (d) After chamfer, the 4 new boundary curves A->B, D->C, A->D, B->C
//     must each have the chamfer surface (found via REMAP_CHAMFER_FACE)
//     as one of their surfA/surfB associations.
TEST_CASE(chamfer_new_curves_have_correct_surface_associations) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity edge = FindEdgeBetweenFaces(extrudeH, face1, face2);
    CHECK_TRUE(edge.v != 0);
    if(!edge.v) return;

    hGroup chamferH = AddChamferGroupByEdge(extrudeH, edge, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Find the chamfer surface handle via its face entity.
    hEntity chamferFaceH = g->Remap(g->predef.entityB, Group::REMAP_CHAMFER_FACE);
    hSSurface hChamfer = { 0 };
    for(auto &ss : g->runningShell.surface) {
        if(ss.face == chamferFaceH.v) { hChamfer = ss.h; break; }
    }
    CHECK_TRUE(hChamfer.v != 0);
    if(hChamfer.v == 0) return;

    // Actual setback points for this test geometry:
    // V1=(20,0,80) top, V2=(20,0,0) bottom, swap → d1=(0,1,0), d2=(-1,0,0)
    // P1=(20,2,80), P2=(20,2,0) on right face; P3=(18,0,80), P4=(18,0,0) on front face
    Vector P1 = Vector::From(20, 2, 80);  // top setback on right face
    Vector P2 = Vector::From(20, 2,  0);  // bottom setback on right face
    Vector P3 = Vector::From(18, 0, 80);  // top setback on front face
    Vector P4 = Vector::From(18, 0,  0);  // bottom setback on front face

    bool curveABok = false, curveDCok = false, curveADok = false, curveBCok = false;
    for(auto &sc : g->runningShell.curve) {
        if(sc.pts.n < 2) continue;
        Vector s = sc.pts[0].p;
        Vector e = sc.pts[sc.pts.n - 1].p;
        // Check all 4 expected chamfer boundary curves with the correct endpoints
        bool hasPair13 = (s.Equals(P1) && e.Equals(P3)) || (s.Equals(P3) && e.Equals(P1));
        bool hasPair24 = (s.Equals(P2) && e.Equals(P4)) || (s.Equals(P4) && e.Equals(P2));
        bool hasPair12 = (s.Equals(P1) && e.Equals(P2)) || (s.Equals(P2) && e.Equals(P1));
        bool hasPair34 = (s.Equals(P3) && e.Equals(P4)) || (s.Equals(P4) && e.Equals(P3));
        bool hasChamfer = (sc.surfA == hChamfer || sc.surfB == hChamfer);
        if(hasPair13 && hasChamfer) curveABok = true;  // top cap: P1-P3
        if(hasPair24 && hasChamfer) curveDCok = true;  // bottom cap: P2-P4
        if(hasPair12 && hasChamfer) curveADok = true;  // right side: P1-P2
        if(hasPair34 && hasChamfer) curveBCok = true;  // front side: P3-P4
    }

    CHECK_TRUE(curveABok);   // cap curve at top (P1-P3) has chamfer as one surface
    CHECK_TRUE(curveDCok);   // cap curve at bottom (P2-P4) has chamfer as one surface
    CHECK_TRUE(curveADok);   // right face curve (P1-P2) has chamfer as one surface
    CHECK_TRUE(curveBCok);   // front face curve (P3-P4) has chamfer as one surface
}

// (e) After fillet with radius=2.0, the setback points A0=(18,0,0),
//     A1=(18,0,20), B0=(20,2,0), B1=(20,2,20) must each appear as
//     the first or last pts[].p of at least one SCurve.
//     setback = r / tan(45°) = 2.0 (for 90-degree edge).
TEST_CASE(fillet_setback_points_are_curve_endpoints) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity edge = FindEdgeBetweenFaces(extrudeH, face1, face2);
    CHECK_TRUE(edge.v != 0);
    if(!edge.v) return;

    hGroup filletH = AddFilletGroupByEdge(extrudeH, edge, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // setback = 2.0/tan(45°) = 2.0
    // Box height is 80 (V1 at top Z=80, V2 at bottom Z=0).
    // After orientation swap: d1=(0,1,0) [right face], d2=(-1,0,0) [front face]
    Vector A0 = Vector::From(18, 0,  0);   // V2 tangent on front face (bottom)
    Vector A1 = Vector::From(18, 0, 80);   // V1 tangent on front face (top)
    Vector B0 = Vector::From(20, 2,  0);   // V2 tangent on right face (bottom)
    Vector B1 = Vector::From(20, 2, 80);   // V1 tangent on right face (top)

    bool foundA0 = false, foundA1 = false, foundB0 = false, foundB1 = false;
    for(auto &sc : g->runningShell.curve) {
        if(sc.pts.n < 2) continue;
        Vector s = sc.pts[0].p;
        Vector e = sc.pts[sc.pts.n - 1].p;
        if(s.Equals(A0) || e.Equals(A0)) foundA0 = true;
        if(s.Equals(A1) || e.Equals(A1)) foundA1 = true;
        if(s.Equals(B0) || e.Equals(B0)) foundB0 = true;
        if(s.Equals(B1) || e.Equals(B1)) foundB1 = true;
    }

    CHECK_TRUE(foundA0);
    CHECK_TRUE(foundA1);
    CHECK_TRUE(foundB0);
    CHECK_TRUE(foundB1);
}

TEST_CASE(fillet_no_stale_vertices) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity edge = FindEdgeBetweenFaces(extrudeH, face1, face2);
    CHECK_TRUE(edge.v != 0);
    if(!edge.v) return;

    hGroup filletH = AddFilletGroupByEdge(extrudeH, edge, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // The original shared-edge endpoints before the fillet.
    // After the fillet these must NOT appear as terminal points of any curve.
    Vector V1 = Vector::From(20,  0,  0);
    Vector V2 = Vector::From(20,  0, 20);

    for(auto &sc : g->runningShell.curve) {
        if(sc.pts.n == 0) continue;
        Vector start = sc.pts[0].p;
        Vector end   = sc.pts[sc.pts.n - 1].p;
        CHECK_FALSE(start.Equals(V1));
        CHECK_FALSE(end.Equals(V1));
        CHECK_FALSE(start.Equals(V2));
        CHECK_FALSE(end.Equals(V2));
    }
}

//-----------------------------------------------------------------------------
// Task 13: Curve count tests.
//
// A rectangular box has exactly 12 edges (SCurves):
//   4 top edges + 4 bottom edges + 4 vertical side edges = 12 total.
//
// After chamfer on one edge:
//   - 1 shared SCurve removed
//   - 4 new SCurves added (hCurve1 A->B, hCurve2 D->C, hCapV1 A->D, hCapV2 B->C)
//   = 12 - 1 + 4 = 15 SCurves in the running shell.
//
// After fillet on one edge: same count (contact1, contact2, arcV1, arcV2 = 4 new).
//   = 12 - 1 + 4 = 15 SCurves.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_curve_count_correct) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    // A plain box has exactly 12 edges (4 top + 4 bottom + 4 vertical sides).
    CHECK_TRUE(eg->runningShell.curve.n == 12);

    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity edge = FindEdgeBetweenFaces(extrudeH, face1, face2);
    CHECK_TRUE(edge.v != 0);
    if(!edge.v) return;

    hGroup chamferH = AddChamferGroupByEdge(extrudeH, edge, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // After chamfer: 12 - 1 shared + 4 new (A->B, D->C, A->D, B->C) = 15
    CHECK_TRUE(g->runningShell.curve.n == 15);
}

TEST_CASE(fillet_curve_count_correct) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    // A plain box has exactly 12 edges.
    CHECK_TRUE(eg->runningShell.curve.n == 12);

    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity edge = FindEdgeBetweenFaces(extrudeH, face1, face2);
    CHECK_TRUE(edge.v != 0);
    if(!edge.v) return;

    hGroup filletH = AddFilletGroupByEdge(extrudeH, edge, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // After fillet: 12 - 1 shared + 4 new (contact1, contact2, arcV1, arcV2) = 15
    CHECK_TRUE(g->runningShell.curve.n == 15);
}
