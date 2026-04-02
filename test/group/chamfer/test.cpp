//-----------------------------------------------------------------------------
// Tests for Group::Type::CHAMFER and Group::Type::FILLET operations.
// These tests programmatically create a box (via DRAWING_WORKPLANE + EXTRUDE)
// and then apply chamfer/fillet groups to two adjacent faces.
//
// Copyright 2025 solvespace contributors.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "harness.h"

namespace {

using namespace SolveSpace;

//-----------------------------------------------------------------------------
// Helper: Create a 20x20x20 box extrusion.
//
// After SS.Init(), the sketch already contains:
//   - Group h={1}: DRAWING_3D (references), order=0
//   - Group h={2}: DRAWING_WORKPLANE (XY plane sketch group), order=1
//     with activeWorkplane = h.entity(0) = {0x00020000}
//
// We add 4 LINE_SEGMENT requests to group {2}, force their points to form a
// 20x20 square, then add an EXTRUDE group (order=2).
//
// Returns the extrude group handle.
//-----------------------------------------------------------------------------
static hGroup CreateBoxExtrude() {
    // After SS.Init(), group #2 is the DRAWING_WORKPLANE (order=1).
    // SS.GW.activeGroup is set by GW.Init() to the last group = {2}.
    hGroup sketchGroupH = SS.GW.activeGroup;  // {2}
    hEntity workplaneH = SS.GW.ActiveWorkplane();  // {0x00020000}

    // Create 4 line segment requests in the sketch group.
    // Each LINE_SEGMENT in a workplane generates:
    //   entity(0) = LINE_SEGMENT entity
    //   entity(1) = start POINT_IN_2D
    //   entity(2) = end POINT_IN_2D
    hRequest rh[4];
    for(int i = 0; i < 4; i++) {
        Request r = {};
        r.type = Request::Type::LINE_SEGMENT;
        r.group = sketchGroupH;
        r.workplane = workplaneH;
        r.construction = false;
        SK.request.AddAndAssignId(&r);
        rh[i] = r.h;
    }

    // Regenerate so entities exist.
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    // Force the 4 corners of a 20x20 square in the XY workplane.
    // Line 0: (0,0,0) -> (20,0,0)
    // Line 1: (20,0,0) -> (20,20,0)
    // Line 2: (20,20,0) -> (0,20,0)
    // Line 3: (0,20,0) -> (0,0,0)
    Vector corners[4] = {
        Vector::From(  0,  0, 0),
        Vector::From( 20,  0, 0),
        Vector::From( 20, 20, 0),
        Vector::From(  0, 20, 0),
    };
    for(int i = 0; i < 4; i++) {
        Entity *startPt = SK.GetEntity(rh[i].entity(1));
        Entity *endPt   = SK.GetEntity(rh[i].entity(2));
        startPt->PointForceTo(corners[i]);
        endPt->PointForceTo(corners[(i + 1) % 4]);
    }

    // Coincident constraints to close the loop.
    for(int i = 0; i < 4; i++) {
        Constraint c = {};
        c.type = Constraint::Type::POINTS_COINCIDENT;
        c.group = sketchGroupH;
        c.workplane = workplaneH;
        c.ptA = rh[i].entity(2);         // end of line i
        c.ptB = rh[(i + 1) % 4].entity(1); // start of line (i+1)
        SK.constraint.AddAndAssignId(&c);
    }

    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    // Create the extrude group (order=2).
    Group eg = {};
    eg.type = Group::Type::EXTRUDE;
    eg.opA = sketchGroupH;
    eg.predef.entityB = workplaneH;
    eg.subtype = Group::Subtype::ONE_SIDED;
    eg.valA = 20.0;
    eg.name = "extrude";
    eg.visible = true;
    eg.color = RGBi(100, 100, 100);
    eg.scale = 1;
    eg.order = 2;
    SK.group.AddAndAssignId(&eg);
    SK.groupOrder.Add(&eg.h);
    SS.GW.activeGroup = eg.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    return eg.h;
}

//-----------------------------------------------------------------------------
// Helper: Find two FACE_XPROD entities belonging to the given extrude group.
// Returns true if found, and sets face1/face2 to the first two FACE_XPROD
// entity handles in that group.
//-----------------------------------------------------------------------------
static bool FindTwoAdjacentFaces(hGroup extrudeGroupH,
                                  hEntity *face1, hEntity *face2) {
    int found = 0;
    for(auto &e : SK.entity) {
        if(e.group != extrudeGroupH) continue;
        if(e.type != Entity::Type::FACE_XPROD) continue;
        if(found == 0) *face1 = e.h;
        else if(found == 1) *face2 = e.h;
        found++;
        if(found >= 2) return true;
    }
    return (found >= 2);
}

//-----------------------------------------------------------------------------
// Helper: Add a CHAMFER group on top of the extrude group.
// Returns the chamfer group handle.
//-----------------------------------------------------------------------------
static hGroup AddChamferGroup(hGroup extrudeGroupH,
                               hEntity face1, hEntity face2,
                               double dist) {
    Group g = {};
    g.type = Group::Type::CHAMFER;
    g.opA = extrudeGroupH;
    g.predef.entityB = face1;
    g.predef.entityC = face2;
    g.valA = dist;
    g.meshCombine = Group::CombineAs::ASSEMBLE;
    g.name = "test-chamfer";
    g.visible = true;
    g.color = RGBi(100, 100, 100);
    g.scale = 1;
    g.order = 3;
    SK.group.AddAndAssignId(&g);
    SK.groupOrder.Add(&g.h);
    SS.GW.activeGroup = g.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    return g.h;
}

//-----------------------------------------------------------------------------
// Helper: Add a FILLET group on top of the extrude group.
// Returns the fillet group handle.
//-----------------------------------------------------------------------------
static hGroup AddFilletGroup(hGroup extrudeGroupH,
                              hEntity face1, hEntity face2,
                              double radius) {
    Group g = {};
    g.type = Group::Type::FILLET;
    g.opA = extrudeGroupH;
    g.predef.entityB = face1;
    g.predef.entityC = face2;
    g.valA = radius;
    g.meshCombine = Group::CombineAs::ASSEMBLE;
    g.name = "test-fillet";
    g.visible = true;
    g.color = RGBi(100, 100, 100);
    g.scale = 1;
    g.order = 3;
    SK.group.AddAndAssignId(&g);
    SK.groupOrder.Add(&g.h);
    SS.GW.activeGroup = g.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    return g.h;
}

} // anonymous namespace

//-----------------------------------------------------------------------------
// Task 1: Basic skeleton - just one test to verify compilation.
// Verify that a box can be created and extruded without crash.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_basic_no_crash) {
    SS.NewFile();
    // After SS.Init(), SS.NewFile() has been called. We call it again to
    // get a fresh sketch (the harness called SS.Init() which includes
    // NewFile()+AfterNewFile()+GW.Init()).
    // The harness already called SS.Init() for us; just build the geometry.
    hGroup extrudeH = CreateBoxExtrude();

    // Verify extrude group exists and has a mesh.
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);
    CHECK_TRUE(eg->runningShell.surface.n > 0 || eg->thisMesh.l.n > 0 ||
               eg->displayMesh.l.n > 0);
}

//-----------------------------------------------------------------------------
// Task 2: Verify that FACE_XPROD entities are generated by the extrude group.
//-----------------------------------------------------------------------------
TEST_CASE(extrude_has_faces) {
    hGroup extrudeH = CreateBoxExtrude();

    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    CHECK_TRUE(face1.v != 0);
    CHECK_TRUE(face2.v != 0);
}

//-----------------------------------------------------------------------------
// Task 3: Chamfer phase 1 tests
//-----------------------------------------------------------------------------

TEST_CASE(chamfer_basic_no_boolean_fail) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
}

TEST_CASE(chamfer_basic_has_mesh) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    g->GenerateDisplayItems();
    CHECK_TRUE(g->displayMesh.l.n > 0);
}

//-----------------------------------------------------------------------------
// Task 4: Fillet phase 1 tests
//-----------------------------------------------------------------------------

TEST_CASE(fillet_basic_no_boolean_fail) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
}

TEST_CASE(fillet_basic_has_mesh) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    g->GenerateDisplayItems();
    CHECK_TRUE(g->displayMesh.l.n > 0);
}

//-----------------------------------------------------------------------------
// Task 5: Chamfer deep tests — self-intersection and shell face count
//-----------------------------------------------------------------------------

TEST_CASE(chamfer_no_self_intersection) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    if(g->booleanFailed) return;

    g->GenerateDisplayItems();
    SMesh *m = &g->displayMesh;
    SEdgeList el = {};
    bool inters, leaks;
    SKdNode::From(m)->MakeCertainEdgesInto(&el,
        EdgeKind::SELF_INTER, /*coplanarIsInter=*/false, &inters, &leaks);
    el.Clear();
    CHECK_FALSE(inters);
}

TEST_CASE(chamfer_shell_has_surfaces) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    if(g->booleanFailed) return;

    // A chamfered box should have more surfaces than a plain box (6).
    CHECK_TRUE(g->runningShell.surface.n > 6);
}

//-----------------------------------------------------------------------------
// Task 6: Fillet deep tests — self-intersection and shell face count
//-----------------------------------------------------------------------------

TEST_CASE(fillet_no_self_intersection) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    if(g->booleanFailed) return;

    g->GenerateDisplayItems();
    SMesh *m = &g->displayMesh;
    SEdgeList el = {};
    bool inters, leaks;
    SKdNode::From(m)->MakeCertainEdgesInto(&el,
        EdgeKind::SELF_INTER, /*coplanarIsInter=*/false, &inters, &leaks);
    el.Clear();
    CHECK_FALSE(inters);
}

TEST_CASE(fillet_shell_has_surfaces) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    if(g->booleanFailed) return;

    // A filleted box should have more surfaces than a plain box (6).
    CHECK_TRUE(g->runningShell.surface.n > 6);
}

//-----------------------------------------------------------------------------
// Task 7: Edge case tests
//-----------------------------------------------------------------------------

TEST_CASE(chamfer_edge_case_small_dist) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    // Very small chamfer distance — should not crash.
    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 0.01);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    // Pass as long as no crash; boolean may or may not fail for small distances.
}

TEST_CASE(chamfer_edge_case_large_dist) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    // Large chamfer distance (nearly the full face size) — should not crash,
    // but booleanFailed may be true.
    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 9.9);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    // Either succeeds or fails gracefully (booleanFailed=true), not a crash.
    // Just verifying no segfault/assertion.
}

//-----------------------------------------------------------------------------
// Task 8: Chaining test — two chamfers sharing the same face (entityB).
//
// This is a direct regression test for the ASSEMBLE duplication bug.
// The scenario matches the user's .slvs file (Group 5 and Group 6 both have
// entityB = left face; Group 6 was the one that failed):
//   - Group N   (chamfer1): opA=extrude, entityB=face[0], entityC=face[1]
//   - Group N+1 (chamfer2): opA=chamfer1, entityB=face[0], entityC=face[3]
//
// Both chamfers share face[0] as entityB. chamfer1 processes the front-right
// edge; chamfer2 processes the front-left edge. Before the fix, chamfer2 would
// set booleanFailed=true because ASSEMBLE mode duplicated all geometry, making
// the face lookup find wrong (duplicate) copies.
//
// For a 20x20x20 box extruded from a 4-line square, there are exactly 4
// FACE_XPROD entities (one per LINE_SEGMENT, one per side face).
//   face[0]=front, face[1]=right, face[2]=back(opposite!), face[3]=left.
// face[0] is adjacent to face[1] (front-right) and face[3] (front-left).
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_chaining_no_boolean_fail) {
    hGroup extrudeH = CreateBoxExtrude();

    // Collect all 4 FACE_XPROD entities from the extrude group.
    // The box has 4 side faces (front, right, back, left) as FACE_XPROD,
    // and top/bottom as FACE_NORMAL_PT (different type, not collected here).
    hEntity faces[4] = {};
    int found = 0;
    for(auto &e : SK.entity) {
        if(e.group != extrudeH) continue;
        if(e.type != Entity::Type::FACE_XPROD) continue;
        if(found < 4) faces[found] = e.h;
        found++;
        if(found >= 4) break;
    }
    CHECK_TRUE(found >= 4);

    // face[0] (front) is adjacent to face[1] (right) and face[3] (left).
    // face[2] (back) is OPPOSITE to face[0] — skipped.
    hEntity sharedFace      = faces[0];  // shared entityB for both chamfers
    hEntity chamfer1Partner = faces[1];  // chamfer1: front-right edge
    hEntity chamfer2Partner = faces[3];  // chamfer2: front-left edge (different edge!)

    // Step 1: First chamfer on (sharedFace, chamfer1Partner) — opA = extrude.
    // This processes the front-right edge. Uses existing helper for brevity.
    hGroup chamfer1H = AddChamferGroup(extrudeH, sharedFace, chamfer1Partner, 2.0);
    Group *c1g = SK.GetGroup(chamfer1H);
    CHECK_TRUE(c1g != nullptr);
    // The first chamfer must succeed (same geometry as chamfer_basic_no_boolean_fail).
    CHECK_FALSE(c1g->booleanFailed);
    if(c1g->booleanFailed) return;

    // Step 2: Second chamfer on (sharedFace, chamfer2Partner) — opA = chamfer1H.
    // sharedFace (face[0]) is the SAME face as chamfer1's entityB — this is the
    // EXACT scenario that triggered the bug: second chamfer on the same face.
    // chamfer2Partner (face[3]=left) is a DIFFERENT adjacent face, unused so far.
    Group g2 = {};
    g2.type = Group::Type::CHAMFER;
    g2.opA = chamfer1H;
    g2.predef.entityB = sharedFace;   // SAME face[0] as chamfer1's entityB
    g2.predef.entityC = chamfer2Partner;  // different adjacent face
    g2.valA = 2.0;
    g2.meshCombine = Group::CombineAs::ASSEMBLE;
    g2.name = "test-chamfer-chain2";
    g2.visible = true;
    g2.color = RGBi(100, 100, 100);
    g2.scale = 1;
    g2.order = 4;
    SK.group.AddAndAssignId(&g2);
    SK.groupOrder.Add(&g2.h);
    SS.GW.activeGroup = g2.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    hGroup chamfer2H = g2.h;

    Group *c2g = SK.GetGroup(chamfer2H);
    CHECK_TRUE(c2g != nullptr);
    // KEY REGRESSION CHECK: the second chamfer on the SAME face must NOT fail.
    // Before the fix, booleanFailed=true here due to ASSEMBLE mode duplicating geometry.
    CHECK_FALSE(c2g->booleanFailed);

    // The final running shell should have more surfaces than a plain box (6 surfaces).
    if(!c2g->booleanFailed) {
        CHECK_TRUE(c2g->runningShell.surface.n > 6);
    }
}
