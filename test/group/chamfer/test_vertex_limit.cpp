//-----------------------------------------------------------------------------
// Tests for the chamfer/fillet vertex limit: max 2 operations per vertex.
//
// The generate-time validation in groupmesh.cpp sets booleanFailed = true
// when a 3rd chamfer or fillet targets an edge sharing a vertex already
// touched by 2 other chamfer/fillet groups.
//
// These tests exercise the generate-time path (AddChamferGroupByEdge /
// AddFilletGroupByEdge), which bypasses the UI command handler but
// triggers GenerateAll() -> GenerateShellAndMesh() where the check lives.
//-----------------------------------------------------------------------------
#include "helpers.h"

//=============================================================================
// Test 1: Two chamfers at the same vertex should succeed.
// This is the already-working 2-adjacent-chamfer case.
//=============================================================================
TEST_CASE(vertex_limit_two_chamfers_allowed) {
    hGroup extrudeH = CreateBoxExtrude();

    // Pick the front-top-left corner: front(0,-1,0), top, left(-1,0,0)
    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // Chamfer 1: front-top edge
    hEntity edge1 = FindEdgeBetweenFaces(extrudeH, frontFace, topCap);
    CHECK_TRUE(edge1.v != 0);
    hGroup ch1 = AddChamferGroupByEdge(extrudeH, edge1, 2.0);
    CHECK_FALSE(SK.GetGroup(ch1)->booleanFailed);

    // Chamfer 2: front-left edge (shares the front-top-left vertex with chamfer 1)
    hEntity edge2 = FindEdgeBetweenFaces(extrudeH, frontFace, leftFace);
    CHECK_TRUE(edge2.v != 0);
    hGroup ch2 = AddChamferGroupByEdge(ch1, edge2, 2.0);
    CHECK_FALSE(SK.GetGroup(ch2)->booleanFailed);
}

//=============================================================================
// Test 2: Three chamfers at the same vertex should trigger booleanFailed
// on the 3rd group.
//=============================================================================
TEST_CASE(vertex_limit_third_chamfer_blocked) {
    hGroup extrudeH = CreateBoxExtrude();

    // Front-top-left corner: 3 faces meet
    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // Chamfer 1: front-top edge
    hEntity edge1 = FindEdgeBetweenFaces(extrudeH, frontFace, topCap);
    CHECK_TRUE(edge1.v != 0);
    hGroup ch1 = AddChamferGroupByEdge(extrudeH, edge1, 2.0);
    CHECK_FALSE(SK.GetGroup(ch1)->booleanFailed);

    // Chamfer 2: front-left edge
    hEntity edge2 = FindEdgeBetweenFaces(extrudeH, frontFace, leftFace);
    CHECK_TRUE(edge2.v != 0);
    hGroup ch2 = AddChamferGroupByEdge(ch1, edge2, 2.0);
    CHECK_FALSE(SK.GetGroup(ch2)->booleanFailed);

    // Chamfer 3: top-left edge (the third edge at the same vertex)
    hEntity edge3 = FindEdgeBetweenFaces(extrudeH, topCap, leftFace);
    CHECK_TRUE(edge3.v != 0);
    hGroup ch3 = AddChamferGroupByEdge(ch2, edge3, 2.0);
    // The 3rd chamfer at the same corner should be blocked
    CHECK_TRUE(SK.GetGroup(ch3)->booleanFailed);
}

//=============================================================================
// Test 3: Three fillets at the same vertex should trigger booleanFailed
// on the 3rd group.
//=============================================================================
TEST_CASE(vertex_limit_third_fillet_blocked) {
    hGroup extrudeH = CreateBoxExtrude();

    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // Fillet 1: front-top edge
    hEntity edge1 = FindEdgeBetweenFaces(extrudeH, frontFace, topCap);
    CHECK_TRUE(edge1.v != 0);
    hGroup fl1 = AddFilletGroupByEdge(extrudeH, edge1, 2.0);
    CHECK_FALSE(SK.GetGroup(fl1)->booleanFailed);

    // Fillet 2: front-left edge
    hEntity edge2 = FindEdgeBetweenFaces(extrudeH, frontFace, leftFace);
    CHECK_TRUE(edge2.v != 0);
    hGroup fl2 = AddFilletGroupByEdge(fl1, edge2, 2.0);
    CHECK_FALSE(SK.GetGroup(fl2)->booleanFailed);

    // Fillet 3: top-left edge
    hEntity edge3 = FindEdgeBetweenFaces(extrudeH, topCap, leftFace);
    CHECK_TRUE(edge3.v != 0);
    hGroup fl3 = AddFilletGroupByEdge(fl2, edge3, 2.0);
    CHECK_TRUE(SK.GetGroup(fl3)->booleanFailed);
}

//=============================================================================
// Test 4: Mixed chamfer + fillet: 2 chamfers + 1 fillet at the same vertex.
// The fillet (3rd op) should trigger booleanFailed.
//=============================================================================
TEST_CASE(vertex_limit_mixed_third_blocked) {
    hGroup extrudeH = CreateBoxExtrude();

    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // Chamfer 1: front-top edge
    hEntity edge1 = FindEdgeBetweenFaces(extrudeH, frontFace, topCap);
    CHECK_TRUE(edge1.v != 0);
    hGroup ch1 = AddChamferGroupByEdge(extrudeH, edge1, 2.0);
    CHECK_FALSE(SK.GetGroup(ch1)->booleanFailed);

    // Chamfer 2: front-left edge
    hEntity edge2 = FindEdgeBetweenFaces(extrudeH, frontFace, leftFace);
    CHECK_TRUE(edge2.v != 0);
    hGroup ch2 = AddChamferGroupByEdge(ch1, edge2, 2.0);
    CHECK_FALSE(SK.GetGroup(ch2)->booleanFailed);

    // Fillet 3: top-left edge (the third op at the same corner, but a fillet)
    hEntity edge3 = FindEdgeBetweenFaces(extrudeH, topCap, leftFace);
    CHECK_TRUE(edge3.v != 0);
    hGroup fl3 = AddFilletGroupByEdge(ch2, edge3, 2.0);
    CHECK_TRUE(SK.GetGroup(fl3)->booleanFailed);
}

//=============================================================================
// Test 5: Two chamfers at different vertices should both succeed.
// Chamfer the front-top edge at the (0,0,20) corner and the
// back-bottom edge at the (20,20,0) corner. These vertices are
// completely separate, so no limit should apply.
//=============================================================================
TEST_CASE(vertex_limit_different_vertices_allowed) {
    hGroup extrudeH = CreateBoxExtrude();

    // Front face + top cap edge → touches vertex (0,0,20)
    hEntity frontFace  = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap     = FindCapFace(extrudeH, /*wantTop=*/true);
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);

    hEntity edge1 = FindEdgeBetweenFaces(extrudeH, frontFace, topCap);
    CHECK_TRUE(edge1.v != 0);
    hGroup ch1 = AddChamferGroupByEdge(extrudeH, edge1, 2.0);
    CHECK_FALSE(SK.GetGroup(ch1)->booleanFailed);

    // Back face + bottom cap edge → touches vertex (20,20,0)
    hEntity backFace   = FindFaceByNormal(extrudeH, Vector::From(0, 1, 0));
    hEntity bottomCap  = FindCapFace(extrudeH, /*wantTop=*/false);
    CHECK_TRUE(backFace.v != 0);
    CHECK_TRUE(bottomCap.v != 0);

    hEntity edge2 = FindEdgeBetweenFaces(extrudeH, backFace, bottomCap);
    CHECK_TRUE(edge2.v != 0);
    hGroup ch2 = AddChamferGroupByEdge(ch1, edge2, 2.0);
    CHECK_FALSE(SK.GetGroup(ch2)->booleanFailed);
}

//=============================================================================
// Test 6: CountChamferFilletsAtVertex helper function direct test.
// Create 2 chamfer groups, then verify the count at the shared vertex
// returns 2, and at a far-away vertex returns 0.
//=============================================================================
TEST_CASE(vertex_limit_count_helper_correct) {
    hGroup extrudeH = CreateBoxExtrude();

    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // Chamfer 1: front-top edge
    hEntity edge1 = FindEdgeBetweenFaces(extrudeH, frontFace, topCap);
    CHECK_TRUE(edge1.v != 0);
    hGroup ch1 = AddChamferGroupByEdge(extrudeH, edge1, 2.0);
    CHECK_FALSE(SK.GetGroup(ch1)->booleanFailed);

    // Chamfer 2: front-left edge
    hEntity edge2 = FindEdgeBetweenFaces(extrudeH, frontFace, leftFace);
    CHECK_TRUE(edge2.v != 0);
    hGroup ch2 = AddChamferGroupByEdge(ch1, edge2, 2.0);
    CHECK_FALSE(SK.GetGroup(ch2)->booleanFailed);

    // The shared vertex of front-top and front-left edges is (0,0,20).
    // Both edges touch this point. CountChamferFilletsAtVertex should return 2.
    Vector sharedVertex = Vector::From(0, 0, 20);
    int count = Group::CountChamferFilletsAtVertex(sharedVertex);
    CHECK_TRUE(count == 2);

    // A far-away vertex should have count 0.
    Vector farVertex = Vector::From(20, 20, 0);
    int farCount = Group::CountChamferFilletsAtVertex(farVertex);
    CHECK_TRUE(farCount == 0);
}
