//-----------------------------------------------------------------------------
// Cap/backface tests
//-----------------------------------------------------------------------------
#include "helpers.h"

//-----------------------------------------------------------------------------
// Side-face + top-cap chamfer test: chamfer(side FACE_XPROD, top FACE_NORMAL_PT)
// must NOT produce back-facing triangles.
//
// The user reports that chamfer(extruded side face + bottom cap) works but
// chamfer(extruded side face + top cap) is wrong (inverted geometry).
// The orientation normalization condition at chamfer.cpp:249 fires incorrectly
// for the top-cap horizontal edge, causing the chamfer face to be inverted.
//
// This test SHOULD FAIL before the fix (Task 2) is applied.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_side_top_cap_no_backface) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    // face1 is a FACE_XPROD (side face: front Y=0).
    // Find the top cap FACE_NORMAL_PT.
    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);  // Must find a FACE_NORMAL_PT at high Z
    if(topCap.v == 0) return;

    hGroup chamferH = AddChamferGroup(extrudeH, face1, topCap, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Check no back-facing triangles. Box center = (10,10,40) for 20x20x80 box.
    g->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 40);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g->displayMesh.l.n; ti++) {
        STriangle *tr = &g->displayMesh.l[ti];
        Vector normal = tr->EffectiveNormal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {  // slightly relaxed threshold
            anyBackFacing = true;
            break;
        }
    }
    CHECK_FALSE(anyBackFacing);
}

//-----------------------------------------------------------------------------
// Side-face + top-cap fillet test.
// Same scenario as chamfer_side_top_cap_no_backface but for fillet.
// This test SHOULD FAIL before the fix (Task 2) is applied.
//-----------------------------------------------------------------------------
TEST_CASE(fillet_side_top_cap_no_backface) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;

    hGroup filletH = AddFilletGroup(extrudeH, face1, topCap, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    g->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 40);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g->displayMesh.l.n; ti++) {
        STriangle *tr = &g->displayMesh.l[ti];
        Vector normal = tr->EffectiveNormal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {
            anyBackFacing = true;
            break;
        }
    }
    CHECK_FALSE(anyBackFacing);
}

TEST_CASE(chamfer_cap_edges_hidden) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    Vector V1 = Vector::From(20, 0, 0);
    Vector V2 = Vector::From(20, 0, 80);

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // After chamfer: ALL LINE_SEGMENT entities from the extrude group that
    // have V1 or V2 as an endpoint should be forceHidden.
    // This includes cap edges like (0,0,0)->(20,0,0) and (20,0,0)->(20,20,0).
    int capEdgesFound = 0;
    int capEdgesHidden = 0;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) continue;
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        // Any edge touching V1 or V2 should be hidden
        if(p0.Equals(V1) || p0.Equals(V2) || p1.Equals(V1) || p1.Equals(V2)) {
            capEdgesFound++;
            if(e.forceHidden) capEdgesHidden++;
        }
    }
    // There should be 5 edges touching V1 or V2:
    //   - 2 cap edges at V1 (from bottom face)
    //   - 2 cap edges at V2 (from top face)
    //   - 1 side edge V1->V2 (already fixed in previous session)
    // All should be hidden.
    CHECK_TRUE(capEdgesFound >= 4);   // at least the 4 cap edges
    CHECK_TRUE(capEdgesHidden == capEdgesFound);  // all should be hidden
}

TEST_CASE(fillet_cap_edges_hidden) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    Vector V1 = Vector::From(20, 0, 0);
    Vector V2 = Vector::From(20, 0, 80);

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    int capEdgesFound = 0;
    int capEdgesHidden = 0;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) continue;
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(V1) || p0.Equals(V2) || p1.Equals(V1) || p1.Equals(V2)) {
            capEdgesFound++;
            if(e.forceHidden) capEdgesHidden++;
        }
    }
    CHECK_TRUE(capEdgesFound >= 4);
    CHECK_TRUE(capEdgesHidden == capEdgesFound);
}

TEST_CASE(fillet_original_edge_hidden) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    Vector V1 = Vector::From(20, 0, 0);
    // Box height is 80: extrusion with valA=20 on the default workplane gives Z=80 at top.
    Vector V2 = Vector::From(20, 0, 80);

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    bool foundEdge = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) continue;
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if((p0.Equals(V1) && p1.Equals(V2)) || (p0.Equals(V2) && p1.Equals(V1))) {
            CHECK_TRUE(e.forceHidden);
            foundEdge = true;
        }
    }
    CHECK_TRUE(foundEdge);
}

//-----------------------------------------------------------------------------
// Fix plan item 1 (TDD): chamfer_face1_face2_cap_no_backface
//
// Scenario: box -> chamfer(face1 + face2, dist=2.0)
// where face1 = front side face (Y=0), face2 = right side face (X=20).
// These share the VERTICAL edge at X=20, Y=0 from Z=0 to Z=80.
//
// The cap surfaces for this chamfer are:
//   hCapSurfV1 = bottomCap (Z=0) for the bottom endpoint V1=(20,0,0)
//   hCapSurfV2 = topCap   (Z=80) for the top    endpoint V2=(20,0,80)
//
// After the chamfer, cap surface trim is updated in Step 15:
//   bottomCap gets cap curve A->D inserted (A=(18,0,0), D=(20,2,0))
//   topCap    gets cap curve B->C inserted (B=(18,0,80), C=(20,2,80))
//
// If the direction (capV1Backwards/capV2Backwards) is WRONG, the resulting
// triangulation of the corner triangle on topCap/bottomCap is BACKFACING.
// This produces the "red triangle" visible in the SolveSpace viewport.
//
// TDD: FAIL before fix (backfacing corner triangle at V1 on bottomCap or
//      V2 on topCap due to wrong cap curve insertion direction in Step 15).
// TDD: PASS after fix (correct direction: corner triangle faces outward).
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_face1_face2_cap_no_backface) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    // Single chamfer on the two vertical side faces (the shared vertical edge).
    // face1 = front face (Y=0), face2 = right face (X=20).
    // The cap surfaces are topCap (Z=80) and bottomCap (Z=0).
    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    if(g == nullptr) return;
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Check no back-facing triangles in the chamfer group's display mesh.
    // Box center = (10,10,40) for 20x20x80 box.
    // Any triangle whose outward normal points TOWARD the box center is backfacing.
    g->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 40);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g->displayMesh.l.n; ti++) {
        STriangle *tr = &g->displayMesh.l[ti];
        Vector normal = tr->EffectiveNormal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {
            anyBackFacing = true;
            break;
        }
    }
    // TDD: FAIL before fix (backfacing corner triangle from wrong Step 15 direction).
    // TDD: PASS after fix.
    CHECK_FALSE(anyBackFacing);
}

//-----------------------------------------------------------------------------
// Fix plan item 2 (TDD): fillet_face1_face2_cap_no_backface
//
// Scenario: box -> fillet(face1 + face2, radius=2.0)
// where face1 = front side face (Y=0), face2 = right side face (X=20).
// These share the VERTICAL edge at X=20, Y=0 from Z=0 to Z=80.
//
// The cap surfaces for this fillet are:
//   hCapSurfV1 = bottomCap (Z=0) for the bottom endpoint V1=(20,0,0)
//   hCapSurfV2 = topCap   (Z=80) for the top    endpoint V2=(20,0,80)
//
// After the fillet, the arc ends at V1 and V2 connect to the cap surfaces.
// Step 15 in MakeFromFilletOf inserts arc endpoints into the cap surface trim.
//
// If the direction (arcV1Backwards/arcV2Backwards) is WRONG in the fillet
// algorithm, the corner triangle on topCap/bottomCap is BACKFACING.
// This produces the "red triangle" visible in the SolveSpace viewport for fillets.
//
// TDD: expected FAIL before fix (backfacing corner triangle from wrong direction).
// TDD: PASS after fix (correct direction: corner triangle faces outward).
//-----------------------------------------------------------------------------
TEST_CASE(fillet_face1_face2_cap_no_backface) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    // Single fillet on the two vertical side faces (the shared vertical edge).
    // face1 = front face (Y=0), face2 = right face (X=20).
    // The cap surfaces are topCap (Z=80) and bottomCap (Z=0).
    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    if(g == nullptr) return;
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Check no back-facing triangles in the fillet group's display mesh.
    // Box center = (10,10,40) for 20x20x80 box.
    // Any triangle whose outward normal points TOWARD the box center is backfacing.
    g->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 40);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g->displayMesh.l.n; ti++) {
        STriangle *tr = &g->displayMesh.l[ti];
        Vector normal = tr->EffectiveNormal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {
            anyBackFacing = true;
            break;
        }
    }
    // TDD: expected FAIL before fix (backfacing corner from wrong Step 15 direction).
    // TDD: PASS after fix.
    CHECK_FALSE(anyBackFacing);
}

//-----------------------------------------------------------------------------
// Fix plan item 9: chamfer_adjacent_vertical_no_backface
//
// Scenario: box -> chamfer1(face1 + face2, dist=2.0)
//               -> chamfer2(face2 + face3, dist=2.0)
// where face1 = front side face (Y=0), face2 = right side face (X=20),
//       face3 = back side face (Y=20).
//
// face1 and face2 share the VERTICAL edge at X=20, Y=0 (from Z=0 to Z=80).
// face2 and face3 share the VERTICAL edge at X=20, Y=20 (from Z=0 to Z=80).
//
// Both chamfers modify face2 (the right face) from opposite sides.
// Both chamfers also modify topCap (Z=80) and bottomCap (Z=0) at DIFFERENT corners:
//   - chamfer1 modifies topCap at corner (20,0,80) and bottomCap at corner (20,0,0)
//   - chamfer2 modifies topCap at corner (20,20,80) and bottomCap at corner (20,20,0)
//
// This is the EXACT scenario from the screenshot (two adjacent VERTICAL-edge chamfers
// on the same face, sharing topCap/bottomCap but at different corners).
//
// After chamfer2, check no backfacing triangles in chamfer2's displayMesh.
// Regression guard: ensure both chamfers produce correct geometry.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_adjacent_vertical_no_backface) {
    hGroup extrudeH = CreateBoxExtrude();

    // Find three consecutive FACE_XPROD entities (fA, fB, fC):
    //   fA = front face (Y=0), from line (0,0,0)->(20,0,0)
    //   fB = right face (X=20), from line (20,0,0)->(20,20,0)
    //   fC = back face (Y=20), from line (20,20,0)->(0,20,0)
    hEntity fA = {}, fB = {}, fC = {};
    int nFound = 0;
    for(auto &e : SK.entity) {
        if(e.group != extrudeH) continue;
        if(e.type != Entity::Type::FACE_XPROD) continue;
        if(nFound == 0) fA = e.h;
        else if(nFound == 1) fB = e.h;
        else if(nFound == 2) fC = e.h;
        nFound++;
        if(nFound >= 3) break;
    }
    CHECK_TRUE(nFound >= 3);
    if(nFound < 3) return;

    // chamfer1: fA (front Y=0) + fB (right X=20) — vertical edge at X=20, Y=0.
    hGroup chamfer1H = AddChamferGroup(extrudeH, fA, fB, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    if(g1 == nullptr) return;
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // chamfer2: fB (right X=20) + fC (back Y=20) — vertical edge at X=20, Y=20.
    // opA = chamfer1H (chamfer2 builds on the result of chamfer1).
    hGroup chamfer2H = AddChamferGroup(chamfer1H, fB, fC, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    if(g2 == nullptr) return;
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check no backfacing triangles in chamfer2's display mesh.
    // Box center = (10,10,40) for 20x20x80 box.
    // Any triangle whose geometric normal points TOWARD the box center is backfacing.
    g2->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 40);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g2->displayMesh.l.n; ti++) {
        STriangle *tr = &g2->displayMesh.l[ti];
        Vector normal = tr->EffectiveNormal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {
            anyBackFacing = true;
            break;
        }
    }
    // Regression guard: two adjacent vertical chamfers on same right face (X=20).
    CHECK_FALSE(anyBackFacing);

    // Also check chamfer1's display mesh for regressions.
    g1->GenerateDisplayItems();
    anyBackFacing = false;
    for(int ti = 0; ti < g1->displayMesh.l.n; ti++) {
        STriangle *tr = &g1->displayMesh.l[ti];
        Vector normal = tr->EffectiveNormal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {
            anyBackFacing = true;
            break;
        }
    }
    CHECK_FALSE(anyBackFacing);
}

//-----------------------------------------------------------------------------
// TDD test: fillet on diff cutout wall faces must not lose the endcap geometry.
//
// Scenario: CreateBoxWithCutoutFromBottom(5, 5, 10, 10, 10)
//   * Base box: 20x20x80 (valA=20, 4x scale)
//   * Pocket: 10x10 centered cutout from (5,5), extruded as DIFF from z=0.
//     SolveSpace DIFFERENCE creates a through-hole: pocket walls z=0 to z=80.
//     The inner bottom cap (capSurf2) is at z=0, INSIDE the pocket footprint.
//
// Fillet: two adjacent wall faces (FACE_XPROD) of the cutout pocket.
//   The shared vertical edge runs from z=0 to z=80 (full box height = through-hole).
//   capSurf2 (inner bottom cap) is at z=0, INSIDE the pocket footprint 5..15 x 5..15.
//   capSurf2 initially has trim.n == 0. The fix reconstructs its trim loop and
//   inserts the fillet arc to make it fully triangulated.
//
// Before fix (with `if(n > 0)` guard in Step 15 of MakeFromFilletOf):
//   capSurf2->trim.n == 0, the guard skips the arc insertion.
//   The inner cap trim loop is incomplete => inner cap has NO triangles.
//   No triangles inside pocket footprint at z=0 => TEST FAILS (TDD RED).
//
// After fix:
//   The arc at z=0 is added to capSurf2's trim loop (via RECON + traversal).
//   capSurf2 is fully triangulated => triangles appear inside pocket footprint at z=0.
//   TEST PASSES (TDD GREEN).
//-----------------------------------------------------------------------------
TEST_CASE(fillet_diff_endcap_has_triangles) {
    // Create box + CENTERED pocket (not corner) from bottom.
    // Using a centered pocket at (5,5) ensures ALL 4 pocket walls are fully
    // inside the box after the DIFFERENCE boolean. Corner pockets (0,0) cause
    // the first FACE_XPROD entity to be the y=0 boundary face which coincides
    // with the outer box wall and is removed by the boolean — making it
    // unfindable by face handle lookup → hSurf1.v==0 → booleanFailed=true.
    //
    // Box: 0..20 x 0..20 (base), extruded to Z=80 (valA=20 with 4x scale).
    // Pocket: 5..15 x 5..15, extrude DIFF from z=0. SolveSpace's DIFFERENCE boolean
    // creates a through-hole (pocket walls z=0..z=80). The inner bottom cap (capSurf2)
    // is the flat surface at z=0 INSIDE the pocket footprint (x=5..15, y=5..15).
    // The outer box bottom (surface 2) has a hole at the pocket footprint — no triangles
    // there. Only capSurf2 (after fix) contributes triangles at z=0 inside the pocket.
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(5.0, 5.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);

    // Find two adjacent wall faces (FACE_XPROD) of the pocket's cutExtrude group.
    // With a centered pocket, all 4 FACE_XPROD entities exist in the result shell.
    // FindTwoAdjacentFaces returns the first two (face[0]=y=5 wall, face[1]=x=15 wall),
    // which ARE adjacent and share the corner vertical edge.
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    // Apply fillet on the shared vertical edge of face1 and face2 (radius=1.0).
    // The edge runs along the pocket corner from z=0 to z=80 (full box height).
    hGroup filletH = AddFilletGroup(bwc.cutExtrude, face1, face2, 1.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    if(g == nullptr) return;

    // The fillet must not fail.
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Generate display mesh for the fillet group.
    g->GenerateDisplayItems();

    // Count triangles where ALL three vertices lie within 2.0 units of z=0.0
    // AND inside the pocket footprint (x in [5,15], y in [5,15]).
    //
    // The outer box bottom (surface 2) at z=0 has a HOLE in the pocket footprint —
    // no triangles there. The inner bottom cap (capSurf2) at z=0 INSIDE the pocket
    // footprint only gets triangulated after the fix (RECON + arc insertion).
    //
    // Before fix: capSurf2->trim.n == 0, arc NOT inserted => no triangles inside pocket
    //             at z=0 => TEST FAILS (TDD RED).
    //
    // After fix: RECON reconstructs capSurf2's trim, arc IS inserted => capSurf2 is
    //            fully triangulated => triangles appear inside pocket at z=0.
    //            => TEST PASSES (TDD GREEN).
    int endcapTriCount = 0;
    const double endcapZ = 0.0;
    const double tol = 2.0;
    // Pocket footprint bounds (centroid-based: pocket center (10,10), half-size 5).
    // Use a slightly relaxed inner region so surface 8 triangles qualify.
    // Surface 8 polygon: (5,5)-(5,15)-(15,15)-(15,6-arc-(14,5).
    // A triangle centroid at (10,10) has centroid inside; require centroid inside 5..15.
    const double pocketX0 = 5.0, pocketX1 = 15.0;
    const double pocketY0 = 5.0, pocketY1 = 15.0;
    for(int ti = 0; ti < g->displayMesh.l.n; ti++) {
        STriangle *tr = &g->displayMesh.l[ti];
        if(fabs(tr->a.z - endcapZ) < tol &&
           fabs(tr->b.z - endcapZ) < tol &&
           fabs(tr->c.z - endcapZ) < tol) {
            // Only count triangles inside the pocket footprint.
            // The outer box bottom has a hole there; only capSurf2 (inner cap)
            // contributes triangles in this region after the fix.
            // Surface 8 polygon has boundary vertices exactly AT x=5,y=5,x=15,y=15.
            // Use centroid check to avoid strict > failing on boundary vertices.
            auto inPocket = [&](const Vector &v) {
                return v.x >= pocketX0 - LENGTH_EPS && v.x <= pocketX1 + LENGTH_EPS &&
                       v.y >= pocketY0 - LENGTH_EPS && v.y <= pocketY1 + LENGTH_EPS;
            };
            if(inPocket(tr->a) && inPocket(tr->b) && inPocket(tr->c)) {
                endcapTriCount++;
            }
        }
    }
    // TDD: FAIL before fix (endcap has 0 triangles -- trim loop is incomplete).
    // TDD: PASS after fix (endcap is properly triangulated with its arc curve).

    // DIAGNOSTIC: Print total triangle count and Z range
    {
        int totalTri = g->displayMesh.l.n;
        double minZ = 1e9, maxZ = -1e9;
        for(int ti = 0; ti < totalTri; ti++) {
            STriangle *tr = &g->displayMesh.l[ti];
            double zvals[3] = {tr->a.z, tr->b.z, tr->c.z};
            for(int k = 0; k < 3; k++) {
                if(zvals[k] < minZ) minZ = zvals[k];
                if(zvals[k] > maxZ) maxZ = zvals[k];
            }
        }
        dbp("DIAG: totalTri=%d minZ=%.2f maxZ=%.2f endcapZ=%.2f endcapTriCount=%d",
            totalTri, (totalTri > 0 ? minZ : 0.0), (totalTri > 0 ? maxZ : 0.0),
            endcapZ, endcapTriCount);
    }

    // Extra debug: print all z=0 triangles and their x,y positions
    for(int ti = 0; ti < g->displayMesh.l.n; ti++) {
        STriangle *tr = &g->displayMesh.l[ti];
        if(fabs(tr->a.z) < 2.0 && fabs(tr->b.z) < 2.0 && fabs(tr->c.z) < 2.0) {
            dbp("TRI_Z0: a=(%.2f,%.2f) b=(%.2f,%.2f) c=(%.2f,%.2f)",
                tr->a.x, tr->a.y, tr->b.x, tr->b.y, tr->c.x, tr->c.y);
        }
    }
    CHECK_TRUE(endcapTriCount > 0);
}
