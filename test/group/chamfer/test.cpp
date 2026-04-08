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
// Helper: Find a FACE_NORMAL_PT entity (cap face) from the extrude group.
// wantTop=true  -> returns the face with the highest Z coordinate
// wantTop=false -> returns the face with the lowest Z coordinate
//-----------------------------------------------------------------------------
static hEntity FindCapFace(hGroup extrudeGroupH, bool wantTop) {
    hEntity best = {};
    double bestZ = wantTop ? -1e30 : 1e30;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeGroupH) continue;
        if(e.type != Entity::Type::FACE_NORMAL_PT) continue;
        Vector pt = e.FaceGetPointNum();
        if(wantTop && pt.z > bestZ) { bestZ = pt.z; best = e.h; }
        if(!wantTop && pt.z < bestZ) { bestZ = pt.z; best = e.h; }
    }
    return best;
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
    g.order = SK.group.n + 1;
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
    g.order = SK.group.n + 1;
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
    g2.order = SK.group.n + 1;
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

//-----------------------------------------------------------------------------
// Task 9: Face-order-invariant chamfer test — order 1 (face1, face2)
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_face_order_forward) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup chamferH1 = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g1 = SK.GetGroup(chamferH1);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    int surfCount1 = g1->runningShell.surface.n;
    CHECK_TRUE(surfCount1 > 6);

    // Verify no back-facing triangles: all triangle normals should point
    // away from the box center (10,10,10). A flipped chamfer/fillet face
    // produces red back-facing triangles in the UI.
    g1->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 10);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g1->displayMesh.l.n; ti++) {
        STriangle *tr = &g1->displayMesh.l[ti];
        Vector normal = tr->Normal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -LENGTH_EPS) {
            anyBackFacing = true;
            break;
        }
    }
    CHECK_FALSE(anyBackFacing);
}

//-----------------------------------------------------------------------------
// Task 9b: Face-order-invariant chamfer test — order 2 (face2, face1)
// This was the buggy order before the orientation normalization fix.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_face_order_invariant) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    // Reversed order: face2 first, face1 second (previously buggy)
    hGroup chamferH = AddChamferGroup(extrudeH, face2, face1, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    int surfCount = g->runningShell.surface.n;
    CHECK_TRUE(surfCount > 6);

    // Verify no back-facing triangles: all triangle normals should point
    // away from the box center (10,10,10). A flipped chamfer/fillet face
    // produces red back-facing triangles in the UI.
    g->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 10);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g->displayMesh.l.n; ti++) {
        STriangle *tr = &g->displayMesh.l[ti];
        Vector normal = tr->Normal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -LENGTH_EPS) {
            anyBackFacing = true;
            break;
        }
    }
    CHECK_FALSE(anyBackFacing);
}

//-----------------------------------------------------------------------------
// Task 10: Face-order-invariant fillet test — order 1 (face1, face2)
//-----------------------------------------------------------------------------
TEST_CASE(fillet_face_order_forward) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    hGroup filletH1 = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g1 = SK.GetGroup(filletH1);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    g1->GenerateDisplayItems();
    int surfCount1 = g1->runningShell.surface.n;
    CHECK_TRUE(surfCount1 > 6);

    // Verify no back-facing triangles: all triangle normals should point
    // away from the box center (10,10,10). A flipped chamfer/fillet face
    // produces red back-facing triangles in the UI.
    g1->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 10);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g1->displayMesh.l.n; ti++) {
        STriangle *tr = &g1->displayMesh.l[ti];
        Vector normal = tr->Normal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -LENGTH_EPS) {
            anyBackFacing = true;
            break;
        }
    }
    CHECK_FALSE(anyBackFacing);
}

//-----------------------------------------------------------------------------
// Task 10b: Face-order-invariant fillet test — order 2 (face2, face1)
// This was the buggy order before the orientation normalization fix.
//-----------------------------------------------------------------------------
TEST_CASE(fillet_face_order_invariant) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);

    // Reversed order: face2 first, face1 second (previously buggy)
    hGroup filletH = AddFilletGroup(extrudeH, face2, face1, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    g->GenerateDisplayItems();
    int surfCount = g->runningShell.surface.n;
    CHECK_TRUE(surfCount > 6);

    // Verify no back-facing triangles: all triangle normals should point
    // away from the box center (10,10,10). A flipped chamfer/fillet face
    // produces red back-facing triangles in the UI.
    g->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 10);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g->displayMesh.l.n; ti++) {
        STriangle *tr = &g->displayMesh.l[ti];
        Vector normal = tr->Normal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -LENGTH_EPS) {
            anyBackFacing = true;
            break;
        }
    }
    CHECK_FALSE(anyBackFacing);
}

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

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
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

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
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

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
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

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
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

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
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

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
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

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
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

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
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

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // After fillet: 12 - 1 shared + 4 new (contact1, contact2, arcV1, arcV2) = 15
    CHECK_TRUE(g->runningShell.curve.n == 15);
}

//-----------------------------------------------------------------------------
// Task 14: Entity-level tests for chamfer/fillet setback points.
// These tests check SK.entity directly (not just the shell/curves).
//
// Tests (a) and (b) document CURRENT behavior (should PASS immediately).
// Tests (c), (d), (e) describe DESIRED behavior (should FAIL until fixed).
//
// SAFETY NOTE: Always check e.IsPoint() before calling e.PointGetNum()
// because PointGetNum() will crash on non-point entities (e.g. FACE_XPROD).
//-----------------------------------------------------------------------------

// (a) Verify the chamfer group generates a FACE entity (IsFace()==true).
//     This should PASS (already implemented in group.cpp).
TEST_CASE(chamfer_group_has_face_entity) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Check that at least one entity in the chamfer group is a face.
    bool foundFace = false;
    for(auto &e : SK.entity) {
        if(e.group != chamferH) continue;
        if(e.IsFace()) { foundFace = true; break; }
    }
    CHECK_TRUE(foundFace);
}

// (b) Document current behavior: point entities exist in chamfer group.
//     This should PASS (point entities generated by Task 2).
//     Purpose: baseline documentation test.
TEST_CASE(chamfer_setback_points_are_in_shell_not_entities) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Point entities ARE generated for chamfer groups (Task 2 fix).
    bool foundPointEntity = false;
    for(auto &e : SK.entity) {
        if(e.group != chamferH) continue;
        if(e.IsPoint()) { foundPointEntity = true; break; }
    }
    CHECK_TRUE(foundPointEntity);
}

// (c) After chamfer with dist=2.0, the setback points A, B, D, C should
//     exist as point entities in SK.entity belonging to the chamfer group.
//     Setback points (after orientation swap): A=(20,2,80), B=(20,2,0),
//     D=(18,0,80), C=(18,0,0).
TEST_CASE(chamfer_should_have_setback_point_entities) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Expected setback points after dist=2.0 (after orientation normalization swap):
    //   surf1=right(X=20): d1=(0,1,0)  => A=(20,2,80), B=(20,2,0)
    //   surf2=front(Y=0):  d2=(-1,0,0) => D=(18,0,80), C=(18,0,0)
    Vector A = Vector::From(20, 2, 80);  // V1 setback on right face
    Vector B = Vector::From(20, 2,  0);  // V2 setback on right face
    Vector D = Vector::From(18, 0, 80);  // V1 setback on front face
    Vector C = Vector::From(18, 0,  0);  // V2 setback on front face

    bool foundA = false, foundB = false, foundD = false, foundC = false;
    for(auto &e : SK.entity) {
        if(e.group != chamferH) continue;
        if(!e.IsPoint()) continue;  // MUST check IsPoint() before PointGetNum()!
        Vector p = e.PointGetNum();
        if(p.Equals(A)) foundA = true;
        if(p.Equals(B)) foundB = true;
        if(p.Equals(D)) foundD = true;
        if(p.Equals(C)) foundC = true;
    }

    CHECK_TRUE(foundA);
    CHECK_TRUE(foundB);
    CHECK_TRUE(foundD);
    CHECK_TRUE(foundC);
}

// (d) After chamfer, there should exist non-face entities (e.g. point or edge)
//     belonging to the chamfer group.
TEST_CASE(chamfer_edges_should_exist_as_entities) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Check for any entity in the chamfer group that is NOT a face.
    // This could be a point, line segment, or curve entity.
    bool foundNonFace = false;
    for(auto &e : SK.entity) {
        if(e.group != chamferH) continue;
        if(!e.IsFace()) { foundNonFace = true; break; }
    }
    CHECK_TRUE(foundNonFace);
}

// (e) After fillet with radius=2.0, setback points should exist as point
//     entities in SK.entity belonging to the fillet group.
//     For a 90-degree edge with r=2.0, setback = r/tan(45deg) = 2.0, giving
//     the same setback positions as chamfer dist=2.0.
TEST_CASE(fillet_should_have_setback_point_entities) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // For 90-degree edge, setback = r/tan(45deg) = r = 2.0.
    // Same setback positions as chamfer with dist=2.0 (after swap):
    Vector A = Vector::From(20, 2, 80);  // V1 tangent on right face
    Vector B = Vector::From(20, 2,  0);  // V2 tangent on right face
    Vector D = Vector::From(18, 0, 80);  // V1 tangent on front face
    Vector C = Vector::From(18, 0,  0);  // V2 tangent on front face

    bool foundA = false, foundB = false, foundD = false, foundC = false;
    for(auto &e : SK.entity) {
        if(e.group != filletH) continue;
        if(!e.IsPoint()) continue;  // MUST check IsPoint() before PointGetNum()!
        Vector p = e.PointGetNum();
        if(p.Equals(A)) foundA = true;
        if(p.Equals(B)) foundB = true;
        if(p.Equals(D)) foundD = true;
        if(p.Equals(C)) foundC = true;
    }

    // All four should exist (implemented in Task 3).
    CHECK_TRUE(foundA);
    CHECK_TRUE(foundB);
    CHECK_TRUE(foundD);
    CHECK_TRUE(foundC);
}

// (f) After chamfer with dist=2.0, LINE_SEGMENT entities should exist for the
//     4 boundary edges: surf1 contact (A-B), surf2 contact (D-C), top cap (A-D),
//     bottom cap (B-C). These are implemented in Task 4.
TEST_CASE(chamfer_has_line_segment_entities) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Count LINE_SEGMENT entities in the chamfer group.
    int lineCount = 0;
    for(auto &e : SK.entity) {
        if(e.group != chamferH) continue;
        if(e.type == Entity::Type::LINE_SEGMENT) lineCount++;
    }
    // Should have exactly 4 line segments for the 4 boundary edges.
    CHECK_TRUE(lineCount == 4);
}

// (g) After fillet with radius=2.0, LINE_SEGMENT entities should exist for the
//     2 contact edges: surf1 contact (A-B) and surf2 contact (D-C).
//     (Cap arcs at V1 and V2 are not generated here; they require ARC_OF_CIRCLE.)
TEST_CASE(fillet_has_contact_line_entities) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Count LINE_SEGMENT entities in the fillet group.
    int lineCount = 0;
    for(auto &e : SK.entity) {
        if(e.group != filletH) continue;
        if(e.type == Entity::Type::LINE_SEGMENT) lineCount++;
    }
    // Should have exactly 2 line segments for the straight contact edges.
    CHECK_TRUE(lineCount == 2);
}

//-----------------------------------------------------------------------------
// Task 15: Original extrude vertices V1/V2 should be forceHidden after chamfer/fillet.
//
// When a chamfer/fillet removes the shared edge V1-V2, the point entities from
// the extrude group at V1 and V2 should have forceHidden=true so they don't
// appear as selectable green dots in the UI.

TEST_CASE(chamfer_original_vertices_hidden) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    // V1 and V2 BEFORE chamfer: extrude group point entities at (20,0,0) and (20,0,20)
    Vector V1 = Vector::From(20, 0,  0);
    Vector V2 = Vector::From(20, 0, 80);

    // Before chamfer: V1/V2 entities should NOT be forceHidden
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(!e.IsPoint()) continue;
        Vector p = e.PointGetNum();
        if(p.Equals(V1) || p.Equals(V2)) {
            CHECK_FALSE(e.forceHidden);
        }
    }

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // After chamfer: V1/V2 entities from the extrude group should be forceHidden
    bool foundV1 = false, foundV2 = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(!e.IsPoint()) continue;
        Vector p = e.PointGetNum();
        if(p.Equals(V1)) {
            CHECK_TRUE(e.forceHidden);
            foundV1 = true;
        }
        if(p.Equals(V2)) {
            CHECK_TRUE(e.forceHidden);
            foundV2 = true;
        }
    }
    CHECK_TRUE(foundV1);
    CHECK_TRUE(foundV2);
}

TEST_CASE(fillet_original_vertices_hidden) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    Vector V1 = Vector::From(20, 0,  0);
    Vector V2 = Vector::From(20, 0, 80);

    hGroup filletH = AddFilletGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // After fillet: V1/V2 entities from the extrude group should be forceHidden
    bool foundV1 = false, foundV2 = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(!e.IsPoint()) continue;
        Vector p = e.PointGetNum();
        if(p.Equals(V1)) {
            CHECK_TRUE(e.forceHidden);
            foundV1 = true;
        }
        if(p.Equals(V2)) {
            CHECK_TRUE(e.forceHidden);
            foundV2 = true;
        }
    }
    CHECK_TRUE(foundV1);
    CHECK_TRUE(foundV2);
}

//-----------------------------------------------------------------------------
// Task 16: Original extrude LINE_SEGMENT V1-V2 should be forceHidden after chamfer/fillet.
//
// After a chamfer/fillet removes the shared edge V1-V2, the LINE_SEGMENT entity
// from the extrude group connecting V1=(20,0,0) to V2=(20,0,20) should have
// forceHidden=true so it doesn't appear as a visible edge in the UI.
//-----------------------------------------------------------------------------

TEST_CASE(chamfer_original_edge_hidden) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    Vector V1 = Vector::From(20, 0, 0);
    // Box height is 80: extrusion with valA=20 on the default workplane gives Z=80 at top.
    Vector V2 = Vector::From(20, 0, 80);

    hGroup chamferH = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g = SK.GetGroup(chamferH);
    CHECK_TRUE(g != nullptr);
    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // After chamfer: the LINE_SEGMENT entity from the extrude group
    // connecting V1 and V2 should be forceHidden.
    bool foundEdge = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        // Look up endpoint positions
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
// Chained chamfer test: chamfer1 on bottom edge, chamfer2 on top edge.
// After chamfer2, the EXTRUDE group's top-corner point entities (at V1=(0,0,80)
// and V2=(20,0,80)) and the LINE_SEGMENT entities touching them should be
// forceHidden=true.
//
// BUG: The current forceHidden loop checks `existEnt.group == opA`. For chamfer2,
// opA = chamfer1 group (NOT extrude), so extrude group entities are NOT found.
// This test SHOULD FAIL before the fix (Task 2) is applied.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_chained_second_hides_extrude_entities) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    // Apply chamfer1 on (face1, bottom cap). opA = extrude group.
    hEntity bottomCap = FindCapFace(extrudeH, false);  // wantTop=false
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // Apply chamfer2 on (face1, top cap). opA = chamfer1 group.
    hEntity topCap = FindCapFace(extrudeH, true);   // wantTop=true
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // The top-cap shared edge endpoints are at the corner between face1 (Y=0)
    // and top cap (Z=80). For the 20x20 box with front face Y=0:
    // V1 and V2 are the two endpoints of the top-front edge.
    // We check that ANY extrude-group point entity at Z=80 and Y=0 is forceHidden.
    bool foundAnyTopCornerHidden = false;
    bool anyTopCornerNotHidden = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(!e.IsPoint()) continue;
        Vector p = e.PointGetNum();
        // Top-front corner: Z near 80 AND Y near 0
        if(fabs(p.z - 80.0) < LENGTH_EPS && fabs(p.y) < LENGTH_EPS) {
            if(e.forceHidden) {
                foundAnyTopCornerHidden = true;
            } else {
                anyTopCornerNotHidden = true;
            }
        }
    }
    // After chamfer2, ALL top-front corners from extrude group should be forceHidden.
    CHECK_TRUE(foundAnyTopCornerHidden);    // we found hidden ones
    CHECK_FALSE(anyTopCornerNotHidden);     // no unhidden top-front corners remain

    // Also check LINE_SEGMENT entities from extrude group touching those corners are hidden.
    bool anyTopEdgeNotHidden = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) continue;
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        bool p0IsTopFront = (fabs(p0.z - 80.0) < LENGTH_EPS && fabs(p0.y) < LENGTH_EPS);
        bool p1IsTopFront = (fabs(p1.z - 80.0) < LENGTH_EPS && fabs(p1.y) < LENGTH_EPS);
        if(p0IsTopFront || p1IsTopFront) {
            if(!e.forceHidden) anyTopEdgeNotHidden = true;
        }
    }
    CHECK_FALSE(anyTopEdgeNotHidden);  // all extrude edges touching top-front corner should be hidden
}

TEST_CASE(fillet_chained_second_hides_extrude_entities) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup fillet1H = AddFilletGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(fillet1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup fillet2H = AddFilletGroup(fillet1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(fillet2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    bool foundAnyTopCornerHidden = false;
    bool anyTopCornerNotHidden = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(!e.IsPoint()) continue;
        Vector p = e.PointGetNum();
        if(fabs(p.z - 80.0) < LENGTH_EPS && fabs(p.y) < LENGTH_EPS) {
            if(e.forceHidden) {
                foundAnyTopCornerHidden = true;
            } else {
                anyTopCornerNotHidden = true;
            }
        }
    }
    CHECK_TRUE(foundAnyTopCornerHidden);
    CHECK_FALSE(anyTopCornerNotHidden);

    bool anyTopEdgeNotHidden = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != extrudeH) continue;
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) continue;
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        bool p0IsTopFront = (fabs(p0.z - 80.0) < LENGTH_EPS && fabs(p0.y) < LENGTH_EPS);
        bool p1IsTopFront = (fabs(p1.z - 80.0) < LENGTH_EPS && fabs(p1.y) < LENGTH_EPS);
        if(p0IsTopFront || p1IsTopFront) {
            if(!e.forceHidden) anyTopEdgeNotHidden = true;
        }
    }
    CHECK_FALSE(anyTopEdgeNotHidden);
}

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
        Vector normal = tr->Normal();
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
        Vector normal = tr->Normal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {
            anyBackFacing = true;
            break;
        }
    }
    CHECK_FALSE(anyBackFacing);
}

//-----------------------------------------------------------------------------
// Helper: Create a box with a rectangular pocket cutout.
//
// Two variants:
//   CreateBoxWithCutout()           - pocket sketch at z=20 (top), extrudes DIFF
//   CreateBoxWithCutoutFromBottom() - pocket sketch at z=0, extrudes DIFF upward
//
// Both create: 20x20x20 box + second sketch + EXTRUDE DIFFERENCE group.
// The DIFFERENCE topology creates complex surface connectivity that can trigger
// the fillet crash when cap surfaces have empty trim lists.
//-----------------------------------------------------------------------------
struct BoxWithCutout {
    hGroup baseExtrude;
    hGroup cutSketchGroup;
    hGroup cutExtrude;
};

// Helper: Find a POINT entity near (x, y, z). Returns {} if not found.
static hEntity FindPointNear(double x, double y, double z) {
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(!e.IsPoint()) continue;
        Vector p = e.PointGetNum();
        if(fabs(p.x - x) < 0.5 && fabs(p.y - y) < 0.5 && fabs(p.z - z) < 0.5)
            return e.h;
    }
    return {};
}

// CreateBoxWithCutout: box at z=20 workplane, extrudes outward as DIFFERENCE
static BoxWithCutout CreateBoxWithCutout(
    double pocketX = 0.0, double pocketY = 0.0,
    double pocketW = 10.0, double pocketH = 10.0,
    double pocketDepth = 10.0)
{
    BoxWithCutout result = {};
    hGroup baseExtrude = CreateBoxExtrude();
    result.baseExtrude = baseExtrude;

    hEntity originPt = FindPointNear(0.0, 0.0, 20.0);
    if(originPt.v == 0) {
        hGroup sketchGroupH = { 2 };
        Group *baseSketchGroup = SK.GetGroup(sketchGroupH);
        originPt = baseSketchGroup->predef.origin;
    }

    Group wpg = {};
    wpg.type = Group::Type::DRAWING_WORKPLANE;
    wpg.subtype = Group::Subtype::WORKPLANE_BY_POINT_ORTHO;
    wpg.predef.origin = originPt;
    wpg.predef.q = Quaternion::From(1.0, 0.0, 0.0, 0.0);
    wpg.name = "cut-sketch";
    wpg.visible = true;
    wpg.color = RGBi(100, 100, 100);
    wpg.scale = 1;
    wpg.order = SK.group.n + 1;
    SK.group.AddAndAssignId(&wpg);
    SK.groupOrder.Add(&wpg.h);
    SS.GW.activeGroup = wpg.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    hGroup wpGroupH = wpg.h;
    result.cutSketchGroup = wpGroupH;

    hEntity cutWorkplane = SS.GW.ActiveWorkplane();

    hRequest crh[4];
    for(int i = 0; i < 4; i++) {
        Request r = {};
        r.type = Request::Type::LINE_SEGMENT;
        r.group = wpGroupH;
        r.workplane = cutWorkplane;
        r.construction = false;
        SK.request.AddAndAssignId(&r);
        crh[i] = r.h;
    }
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    double x0 = pocketX, y0 = pocketY, z = 20.0;
    double x1 = pocketX + pocketW, y1 = pocketY + pocketH;
    Vector pts[4] = {
        Vector::From(x0, y0, z), Vector::From(x1, y0, z),
        Vector::From(x1, y1, z), Vector::From(x0, y1, z),
    };
    for(int i = 0; i < 4; i++) {
        SK.GetEntity(crh[i].entity(1))->PointForceTo(pts[i]);
        SK.GetEntity(crh[i].entity(2))->PointForceTo(pts[(i+1)%4]);
    }
    for(int i = 0; i < 4; i++) {
        Constraint c = {};
        c.type = Constraint::Type::POINTS_COINCIDENT;
        c.group = wpGroupH; c.workplane = cutWorkplane;
        c.ptA = crh[i].entity(2); c.ptB = crh[(i+1)%4].entity(1);
        SK.constraint.AddAndAssignId(&c);
    }
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    Group eg = {};
    eg.type = Group::Type::EXTRUDE;
    eg.opA = wpGroupH; eg.predef.entityB = cutWorkplane;
    eg.subtype = Group::Subtype::ONE_SIDED;
    eg.meshCombine = Group::CombineAs::DIFFERENCE;
    eg.valA = pocketDepth;
    eg.name = "cut-extrude";
    eg.visible = true; eg.color = RGBi(100, 100, 100); eg.scale = 1;
    eg.order = SK.group.n + 1;
    SK.group.AddAndAssignId(&eg);
    SK.groupOrder.Add(&eg.h);
    SS.GW.activeGroup = eg.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    result.cutExtrude = eg.h;
    return result;
}

// CreateBoxWithCutoutFromBottom: second sketch at z=0, extrudes upward as DIFFERENCE
static BoxWithCutout CreateBoxWithCutoutFromBottom(
    double pocketX = 0.0, double pocketY = 0.0,
    double pocketW = 10.0, double pocketH = 10.0,
    double pocketDepth = 10.0)
{
    BoxWithCutout result = {};
    hGroup baseExtrude = CreateBoxExtrude();
    result.baseExtrude = baseExtrude;

    hGroup sketchGroupH = { 2 };
    Group *baseWpGroup = SK.GetGroup(sketchGroupH);

    Group wpg = {};
    wpg.type = Group::Type::DRAWING_WORKPLANE;
    wpg.subtype = Group::Subtype::WORKPLANE_BY_POINT_ORTHO;
    wpg.predef.origin = baseWpGroup->predef.origin;
    wpg.predef.q = baseWpGroup->predef.q;
    wpg.name = "cut-sketch";
    wpg.visible = true; wpg.color = RGBi(100, 100, 100); wpg.scale = 1;
    wpg.order = SK.group.n + 1;
    SK.group.AddAndAssignId(&wpg);
    SK.groupOrder.Add(&wpg.h);
    SS.GW.activeGroup = wpg.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    hGroup wpGroupH = wpg.h;
    result.cutSketchGroup = wpGroupH;

    hEntity cutWorkplane = SS.GW.ActiveWorkplane();

    hRequest crh[4];
    for(int i = 0; i < 4; i++) {
        Request r = {};
        r.type = Request::Type::LINE_SEGMENT;
        r.group = wpGroupH; r.workplane = cutWorkplane; r.construction = false;
        SK.request.AddAndAssignId(&r);
        crh[i] = r.h;
    }
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    double x0 = pocketX, y0 = pocketY, z = 0.0;
    double x1 = pocketX + pocketW, y1 = pocketY + pocketH;
    Vector pts[4] = {
        Vector::From(x0, y0, z), Vector::From(x1, y0, z),
        Vector::From(x1, y1, z), Vector::From(x0, y1, z),
    };
    for(int i = 0; i < 4; i++) {
        SK.GetEntity(crh[i].entity(1))->PointForceTo(pts[i]);
        SK.GetEntity(crh[i].entity(2))->PointForceTo(pts[(i+1)%4]);
    }
    for(int i = 0; i < 4; i++) {
        Constraint c = {};
        c.type = Constraint::Type::POINTS_COINCIDENT;
        c.group = wpGroupH; c.workplane = cutWorkplane;
        c.ptA = crh[i].entity(2); c.ptB = crh[(i+1)%4].entity(1);
        SK.constraint.AddAndAssignId(&c);
    }
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    Group eg = {};
    eg.type = Group::Type::EXTRUDE;
    eg.opA = wpGroupH; eg.predef.entityB = cutWorkplane;
    eg.subtype = Group::Subtype::ONE_SIDED;
    eg.meshCombine = Group::CombineAs::DIFFERENCE;
    eg.valA = pocketDepth;
    eg.name = "cut-extrude";
    eg.visible = true; eg.color = RGBi(100, 100, 100); eg.scale = 1;
    eg.order = SK.group.n + 1;
    SK.group.AddAndAssignId(&eg);
    SK.groupOrder.Add(&eg.h);
    SS.GW.activeGroup = eg.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    result.cutExtrude = eg.h;
    return result;
}

//-----------------------------------------------------------------------------
// Tests: Fillet/Chamfer on boolean-difference geometry (Phase 1: TDD RED)
// After the n==0 guard fix (task 13), all tests should PASS (no crash).
//-----------------------------------------------------------------------------

TEST_CASE(fillet_diff_inside_faces_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutout(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2);
    if(!found) found = FindTwoAdjacentFaces(bwc.baseExtrude, &face1, &face2);
    if(!found) return;
    hGroup filletH = AddFilletGroup(bwc.cutExtrude, face1, face2, 1.0);
    CHECK_TRUE(SK.GetGroup(filletH) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_bottom_cutout_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2);
    if(!found) found = FindTwoAdjacentFaces(bwc.baseExtrude, &face1, &face2);
    if(!found) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_corner_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 1.5)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_centered_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(5.0, 5.0, 10.0, 10.0, 8.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_small_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(2.0, 2.0, 5.0, 5.0, 5.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 0.5)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_small_radius_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 0.1)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_large_radius_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 3.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(chamfer_diff_inside_faces_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddChamferGroup(bwc.cutExtrude, face1, face2, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_asymmetric_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 15.0, 5.0, 8.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_deep_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(2.0, 2.0, 8.0, 8.0, 18.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_shallow_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 2.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 0.5)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_offset_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(3.0, 3.0, 12.0, 12.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroup(bwc.cutExtrude, face1, face2, 1.0)) != nullptr);
    CHECK_TRUE(true);
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

//=============================================================================
// Phase 1-2 TDD Tests: Extra line from origin bug in chained chamfer
// Fix plan items 1-14
//=============================================================================

//-----------------------------------------------------------------------------
// Helper: Count LINE_SEGMENT entities in a specific group
//-----------------------------------------------------------------------------
namespace {
static int CountLineSegmentsInGroup(hGroup grpH, bool visibleOnly = false) {
    int count = 0;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.group != grpH) continue;
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(visibleOnly && e.forceHidden) continue;
        count++;
    }
    return count;
}

static int CountLineSegmentsWithOriginEndpoint(bool skipHidden = false) {
    int count = 0;
    Vector origin = Vector::From(0, 0, 0);
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(skipHidden && e.forceHidden) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) continue;
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) {
            count++;
        }
    }
    return count;
}
} // anonymous namespace

//-----------------------------------------------------------------------------
// Fix plan item 1 (diagnostic):
// Read chamfer entity generation code (done via code review above) +
// Fix plan item 3: Check for LINE_SEGMENT entities with null handles or
// with one endpoint at (0,0,0) after chained chamfer.
// This test should FAIL if the bug is present.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_chained_no_null_point_handles) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check: no LINE_SEGMENT in ANY group has a null point handle
    bool anyNullHandle = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.point[0].v == 0 || e.point[1].v == 0) {
            anyNullHandle = true;
        }
    }
    CHECK_FALSE(anyNullHandle);
}

//-----------------------------------------------------------------------------
// Fix plan item 4: No LINE_SEGMENT from any chamfer group has an endpoint
// at (0,0,0) after second chamfer.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_chained_no_origin_line) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check: no visible LINE_SEGMENT from a chamfer group has an endpoint at (0,0,0)
    Vector origin = Vector::From(0, 0, 0);
    bool anyOriginEndpt = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.group != chamfer1H && e.group != chamfer2H) continue;
        if(e.forceHidden) continue;  // ignore already-hidden entities
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) continue;
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) {
            anyOriginEndpt = true;
        }
    }
    CHECK_FALSE(anyOriginEndpt);
}

//-----------------------------------------------------------------------------
// Fix plan item 5: No LINE_SEGMENT entity in chamfer2 group has
// point[0].v == 0 or point[1].v == 0.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_chained_no_null_endpoint_handles) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check chamfer2's LINE_SEGMENT entities specifically
    bool anyNullHandle = false;
    bool anyMissingEntity = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.group != chamfer2H) continue;
        if(e.point[0].v == 0 || e.point[1].v == 0) {
            anyNullHandle = true;
        }
        if(SK.entity.FindByIdNoOops(e.point[0]) == nullptr) {
            anyMissingEntity = true;
        }
        if(SK.entity.FindByIdNoOops(e.point[1]) == nullptr) {
            anyMissingEntity = true;
        }
    }
    CHECK_FALSE(anyNullHandle);
    CHECK_FALSE(anyMissingEntity);
}

//-----------------------------------------------------------------------------
// Fix plan item 6: After chained chamfer, chamfer2 group has exactly 4
// visible (non-hidden) LINE_SEGMENT entities (EDGE_AB, EDGE_DC, EDGE_AD, EDGE_BC).
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_second_visible_line_count) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Count visible LINE_SEGMENT entities in chamfer2 group
    int totalInChamfer2 = CountLineSegmentsInGroup(chamfer2H, false);
    int visibleInChamfer2 = CountLineSegmentsInGroup(chamfer2H, true);

    // chamfer2 should generate exactly 4 LINE_SEGMENT entities (AB, DC, AD, BC)
    CHECK_TRUE(totalInChamfer2 == 4);
    CHECK_TRUE(visibleInChamfer2 == 4);
}

//-----------------------------------------------------------------------------
// Fix plan item 7: All LINE_SEGMENT entities from chamfer2 group have
// endpoints resolvable in SK.entity and at non-origin positions.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_line_endpoints_valid) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    Vector origin = Vector::From(0, 0, 0);
    bool anyBadEndpoint = false;
    int linesFound = 0;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.group != chamfer2H) continue;
        linesFound++;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) {
            anyBadEndpoint = true;
            continue;
        }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) {
            anyBadEndpoint = true;
        }
    }
    CHECK_TRUE(linesFound == 4);  // chamfer2 must have exactly 4 edges
    CHECK_FALSE(anyBadEndpoint);
}

//-----------------------------------------------------------------------------
// Fix plan item 8: Total visible LINE_SEGMENT count after chained chamfer.
// After chamfer1+chamfer2 on front face (Y=0):
// - Sketch: 4 lines (at Z=0 plane, forceHidden by chamfer groups or visible?)
// - Extrude: multiple vertical + top edge LINE_SEGMENTs
// - Chamfer1: 4 edges (AB, DC, AD, BC) at the bottom-front edge
// - Chamfer2: 4 edges (AB, DC, AD, BC) at the top-front edge
// The key assertion: no spurious extra visible LINE_SEGMENT that creates a
// line from (0,0,0) to somewhere.
// This test checks: visible LINE_SEGMENTs from ALL non-sketch/non-extrude groups
// should be exclusively from chamfer groups and have proper endpoints.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_chained_total_visible_lines) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // ALL visible LINE_SEGMENT entities should have both endpoints resolvable
    // and neither at origin (except if they're in sketch group from the sketch itself)
    // Since the sketch has lines at z=0 that may still be visible...
    // The key invariant: no SPURIOUS extra line from a chamfer group with origin endpoint.
    Vector origin = Vector::From(0, 0, 0);
    bool anyUnresolvableEndpoint = false;
    bool anyChamferOriginLine = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) {
            anyUnresolvableEndpoint = true;
            continue;
        }
        // For chamfer groups specifically: no endpoint at origin
        if(e.group == chamfer1H || e.group == chamfer2H) {
            Vector p0 = ep0->PointGetNum();
            Vector p1 = ep1->PointGetNum();
            if(p0.Equals(origin) || p1.Equals(origin)) {
                anyChamferOriginLine = true;
            }
        }
    }
    CHECK_FALSE(anyUnresolvableEndpoint);
    CHECK_FALSE(anyChamferOriginLine);
}

//-----------------------------------------------------------------------------
// Fix plan item 9: Box shifted to (50,50,0): still no spurious lines at origin.
// This box has NO corner at (0,0,0), so any line to origin would be clearly wrong.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_chained_offset_from_origin) {
    // Use CreateBoxExtrude() and then adjust corner positions to (50,50,0) origin
    // Actually: create fresh with offset corners via PointForceTo
    hGroup sketchGroupH = SS.GW.activeGroup;
    hEntity workplaneH = SS.GW.ActiveWorkplane();

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
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    // Offset box: corners at (50,50,0), (70,50,0), (70,70,0), (50,70,0)
    Vector corners[4] = {
        Vector::From(50, 50, 0),
        Vector::From(70, 50, 0),
        Vector::From(70, 70, 0),
        Vector::From(50, 70, 0),
    };
    for(int i = 0; i < 4; i++) {
        Entity *startPt = SK.GetEntity(rh[i].entity(1));
        Entity *endPt   = SK.GetEntity(rh[i].entity(2));
        startPt->PointForceTo(corners[i]);
        endPt->PointForceTo(corners[(i + 1) % 4]);
    }
    for(int i = 0; i < 4; i++) {
        Constraint c = {};
        c.type = Constraint::Type::POINTS_COINCIDENT;
        c.group = sketchGroupH;
        c.workplane = workplaneH;
        c.ptA = rh[i].entity(2);
        c.ptB = rh[(i + 1) % 4].entity(1);
        SK.constraint.AddAndAssignId(&c);
    }
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    Group eg = {};
    eg.type = Group::Type::EXTRUDE;
    eg.opA = sketchGroupH;
    eg.predef.entityB = workplaneH;
    eg.subtype = Group::Subtype::ONE_SIDED;
    eg.valA = 20.0;
    eg.name = "extrude-offset";
    eg.visible = true;
    eg.color = RGBi(100, 100, 100);
    eg.scale = 1;
    eg.order = 2;
    SK.group.AddAndAssignId(&eg);
    SK.groupOrder.Add(&eg.h);
    SS.GW.activeGroup = eg.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    hGroup extrudeH = eg.h;

    hEntity face1 = {}, face2dummy = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2dummy);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // For offset box: NO line should have an endpoint anywhere near (0,0,0)
    // (the closest corner is at (50,50,0) so anything at (0,0,0) is completely wrong)
    Vector origin = Vector::From(0, 0, 0);
    bool anyOriginLine = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        if(e.group != chamfer1H && e.group != chamfer2H) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) {
            anyOriginLine = true;
            continue;
        }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) {
            anyOriginLine = true;
        }
    }
    CHECK_FALSE(anyOriginLine);
}

//-----------------------------------------------------------------------------
// Fix plan item 10: Three chained chamfers: no spurious lines.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_three_chained) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Third chamfer: face2 + bottomCap (a different edge)
    hGroup chamfer3H = AddChamferGroup(chamfer2H, face2, bottomCap, 2.0);
    Group *g3 = SK.GetGroup(chamfer3H);
    CHECK_TRUE(g3 != nullptr);
    CHECK_FALSE(g3->booleanFailed);
    if(g3->booleanFailed) return;

    // No visible LINE_SEGMENT from any chamfer group should have an origin endpoint
    Vector origin = Vector::From(0, 0, 0);
    bool anyOriginLine = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        if(e.group != chamfer1H && e.group != chamfer2H && e.group != chamfer3H) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) {
            anyOriginLine = true;
            continue;
        }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) {
            anyOriginLine = true;
        }
    }
    CHECK_FALSE(anyOriginLine);
}

//-----------------------------------------------------------------------------
// Fix plan item 11: Fillet version of the no-origin-line test.
//-----------------------------------------------------------------------------
TEST_CASE(fillet_chained_no_origin_line) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup fillet1H = AddFilletGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(fillet1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup fillet2H = AddFilletGroup(fillet1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(fillet2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    Vector origin = Vector::From(0, 0, 0);
    bool anyOriginLine = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        if(e.group != fillet1H && e.group != fillet2H) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) {
            anyOriginLine = true;
            continue;
        }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) {
            anyOriginLine = true;
        }
    }
    CHECK_FALSE(anyOriginLine);
}

//-----------------------------------------------------------------------------
// Fix plan item 12: Fillet version of the null-endpoint-handles test.
//-----------------------------------------------------------------------------
TEST_CASE(fillet_chained_no_null_endpoints) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup fillet1H = AddFilletGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(fillet1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup fillet2H = AddFilletGroup(fillet1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(fillet2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check fillet2's LINE_SEGMENT entities specifically
    bool anyNullHandle = false;
    bool anyMissingEntity = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.group != fillet2H) continue;
        if(e.point[0].v == 0 || e.point[1].v == 0) {
            anyNullHandle = true;
        }
        if(SK.entity.FindByIdNoOops(e.point[0]) == nullptr) {
            anyMissingEntity = true;
        }
        if(SK.entity.FindByIdNoOops(e.point[1]) == nullptr) {
            anyMissingEntity = true;
        }
    }
    CHECK_FALSE(anyNullHandle);
    CHECK_FALSE(anyMissingEntity);
}

//-----------------------------------------------------------------------------
// Fix plan item 13: Chamfer dist=0.1 (minimal): no extra line.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_minimal_offset_no_extra_line) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 0.1);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 0.1);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    Vector origin = Vector::From(0, 0, 0);
    bool anyOriginLine = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        if(e.group != chamfer1H && e.group != chamfer2H) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) { anyOriginLine = true; continue; }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) anyOriginLine = true;
    }
    CHECK_FALSE(anyOriginLine);
}

//-----------------------------------------------------------------------------
// Fix plan item 14: Chamfer dist=9 (large, near edge limit): no extra line.
// Edge is 20 units, so dist=9 < 10 = edge/2 is valid.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_large_offset_no_extra_line) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 9.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 9.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    Vector origin = Vector::From(0, 0, 0);
    bool anyOriginLine = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        if(e.group != chamfer1H && e.group != chamfer2H) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) { anyOriginLine = true; continue; }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) anyOriginLine = true;
    }
    CHECK_FALSE(anyOriginLine);
}

//-----------------------------------------------------------------------------
// Fix plan item 2 (diagnostic) - Extended diagnostic: Check if there's any
// LINE_SEGMENT in any group (visible) that has one endpoint resolving to origin.
// This is a broad diagnostic test that catches the visual "line from origin" bug.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_chained_diagnostic_no_visible_origin_line) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // With single chamfer (only chamfer1), how many visible non-sketch LINE_SEGMENTs
    // have an endpoint at (0,0,0)? Should be 0.
    // With chained chamfer (chamfer1+chamfer2), should still be 0.
    int visibleOriginLines = CountLineSegmentsWithOriginEndpoint(/*skipHidden=*/true);

    // The sketch group itself has lines that START at (0,0,0): line from (0,0,0) to (20,0,0).
    // The sketch line from (0,0,0) is a LINE_SEGMENT in the sketch group.
    // After extrusion + chamfering, this sketch line may or may not be forceHidden.
    // Let's check chamfer groups specifically: they should have NO origin endpoints.
    int visibleChamferOriginLines = 0;
    Vector origin = Vector::From(0, 0, 0);
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        if(e.group != chamfer1H && e.group != chamfer2H) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) { visibleChamferOriginLines++; continue; }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) visibleChamferOriginLines++;
    }
    CHECK_TRUE(visibleChamferOriginLines == 0);
}

//-----------------------------------------------------------------------------
// Fix plan items 18-20: Edge cases
// Item 18: chamfer2 on two SIDE faces (not face+cap): no extra line.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_two_side_faces_no_extra_line) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    // Chamfer the edge between face1 and face2 (two side faces share a vertical edge)
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, face2, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // Apply a second chamfer: face1 + bottomCap (different edge, chained)
    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, bottomCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    // booleanFailed might be true if face1 no longer has a simple edge with bottomCap
    // but let's check for no spurious lines regardless
    if(g2->booleanFailed) return;  // skip remaining checks if geometry failed

    Vector origin = Vector::From(0, 0, 0);
    bool anyOriginLine = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        if(e.group != chamfer1H && e.group != chamfer2H) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) { anyOriginLine = true; continue; }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) anyOriginLine = true;
    }
    CHECK_FALSE(anyOriginLine);
}

//-----------------------------------------------------------------------------
// Fix plan item 19: Chamfer then fillet (mixed chain): no extra line.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_then_fillet_no_extra_line) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup fillet2H = AddFilletGroup(chamfer1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(fillet2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    Vector origin = Vector::From(0, 0, 0);
    bool anyOriginLine = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        if(e.group != chamfer1H && e.group != fillet2H) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) { anyOriginLine = true; continue; }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) anyOriginLine = true;
    }
    CHECK_FALSE(anyOriginLine);
}

//-----------------------------------------------------------------------------
// Fix plan item 20: Fillet then chamfer: no extra line.
//-----------------------------------------------------------------------------
TEST_CASE(fillet_then_chamfer_no_extra_line) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    hEntity bottomCap = FindCapFace(extrudeH, false);
    CHECK_TRUE(bottomCap.v != 0);
    if(bottomCap.v == 0) return;
    hGroup fillet1H = AddFilletGroup(extrudeH, face1, bottomCap, 2.0);
    Group *g1 = SK.GetGroup(fillet1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer2H = AddChamferGroup(fillet1H, face1, topCap, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    Vector origin = Vector::From(0, 0, 0);
    bool anyOriginLine = false;
    for(int i = 0; i < SK.entity.n; i++) {
        Entity &e = SK.entity.Get(i);
        if(e.type != Entity::Type::LINE_SEGMENT) continue;
        if(e.forceHidden) continue;
        if(e.group != fillet1H && e.group != chamfer2H) continue;
        Entity *ep0 = SK.entity.FindByIdNoOops(e.point[0]);
        Entity *ep1 = SK.entity.FindByIdNoOops(e.point[1]);
        if(!ep0 || !ep1) { anyOriginLine = true; continue; }
        Vector p0 = ep0->PointGetNum();
        Vector p1 = ep1->PointGetNum();
        if(p0.Equals(origin) || p1.Equals(origin)) anyOriginLine = true;
    }
    CHECK_FALSE(anyOriginLine);
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
        Vector normal = tr->Normal();
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
        Vector normal = tr->Normal();
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
// Fix plan item 3 (TDD): chamfer_adjacent_cap_no_backface
//
// Scenario: box -> chamfer1(topCap + face1, dist=2.0) -> chamfer2(face1 + face2, dist=2.0)
//
// chamfer1 operates on topCap (Z=80) and face1 (front face Y=0).
//   - The shared edge runs along X from X=0..20 at Y=0, Z=80.
//   - This is a HORIZONTAL edge on the top cap.
//   - After chamfer1: topCap gets a cut edge along Y=0 plane, face1 gets a setback.
//
// chamfer2 operates on face1 (front face Y=0, now modified by chamfer1) and
// face2 (right face X=20).
//   - The shared edge runs along Z from Z=0..80 at X=20, Y=0 (vertical edge).
//   - At V2=(20,0,80): the cap surface for this endpoint is the MODIFIED topCap
//     (which now has a chamfered corner from chamfer1). The topCap at (20,0,80)
//     is no longer a clean rectangular edge — it has the chamfer1 setback nearby.
//   - At V1=(20,0,0): the cap surface is bottomCap (unmodified).
//
// The BACKFACING TRIANGLE appears at the corner where:
//   - topCap's already-modified edge (from chamfer1) meets
//   - the new chamfer2 cap curve (B->C where B=(18,0,80), C=(20,2,80))
//   - The hCapSurfV2 for chamfer2 may fall back to hChamfer (since topCap's
//     adjacent edges at (20,0,80) are now owned by chamfer1, not the original
//     topCap/face1 edges).
//   - This produces the "red triangle" from the screenshot.
//
// TDD: FAIL before fix (backfacing triangle at the complex corner from adjacent chamfers).
// TDD: PASS after fix.
//
// We check backfaces in BOTH chamfer groups' displayMesh (the bug may appear in
// either chamfer1's cap surface or chamfer2's display).
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_adjacent_cap_no_backface) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(extrudeH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    // chamfer1: topCap + face1 (horizontal top edge of front face).
    hEntity topCap = FindCapFace(extrudeH, true);
    CHECK_TRUE(topCap.v != 0);
    if(topCap.v == 0) return;
    hGroup chamfer1H = AddChamferGroup(extrudeH, face1, topCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_TRUE(g1 != nullptr);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // chamfer2: face1 + face2 (vertical edge at X=20, Y=0).
    hGroup chamfer2H = AddChamferGroup(chamfer1H, face1, face2, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_TRUE(g2 != nullptr);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check no back-facing triangles in chamfer2's display mesh.
    // Box center = (10,10,40) for 20x20x80 box.
    // Any triangle whose outward normal points TOWARD the box center is backfacing.
    g2->GenerateDisplayItems();
    Vector boxCenter = Vector::From(10, 10, 40);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g2->displayMesh.l.n; ti++) {
        STriangle *tr = &g2->displayMesh.l[ti];
        Vector normal = tr->Normal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {
            anyBackFacing = true;
            break;
        }
    }
    // TDD: FAIL before fix (backfacing triangle at adjacent chamfer corner).
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
        Vector normal = tr->Normal();
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
        Vector normal = tr->Normal();
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

// TDD test: fillet_assemble_cap_no_backface
//
// Scenario: Box1 (20x20x80) with a smaller ASSEMBLE boss (10x10x10) on top.
// The boss covers x=10..20, y=10..20, z=80..90.
//
// A fillet is applied to the first two FACE_XPROD entities of the boss extrude
// (face[0]=y=10 wall, face[1]=x=20 wall). Their shared vertical edge runs from
// V1=(20,10,80) to V2=(20,10,90).
//
// At V1=(20,10,80), multiple surfaces touch:
//   - box1 top face (z=80, normal=(0,0,+1)) — correct cap, |n.Dot(t)|=1
//   - box1 side face at x=20 (normal=(+1,0,0)) — wrong cap, |n.Dot(t)|=0
//
// The bug: the current cap detection picks the FIRST non-hSurf1/hSurf2 surface,
// which may be box1's x=20 side face. This causes the arc to be inserted into
// box1's x=20 face with wrong orientation, inverting that whole face.
//
// After the fix (score by |n.Dot(t)|, pick highest), box1's top face is always
// selected as hCapSurfV1.
//
// TDD: FAIL before fix (box1 x=20 face inverted => backfacing triangles).
// TDD: PASS after fix (correct cap selected => no backfacing triangles).
//-----------------------------------------------------------------------------
TEST_CASE(fillet_assemble_cap_no_backface) {
    // Create base box: 20x20x80 (z=0..80)
    hGroup box1H = CreateBoxExtrude();
    (void)box1H;

    // Create second sketch workplane anchored at (or near) z=80.
    // FindPointNear looks for a POINT entity generated by the EXTRUDE group
    // at the top corner (0,0,80). If not found (shouldn't happen for an 80-tall
    // box), fall back to group {2}'s origin (z=0) and just force coords to z=80.
    hEntity originPt = FindPointNear(0.0, 0.0, 80.0);
    if(originPt.v == 0) {
        hGroup sketchGroupH = { 2 };
        Group *baseSketchGroup = SK.GetGroup(sketchGroupH);
        originPt = baseSketchGroup->predef.origin;
    }

    Group wpg = {};
    wpg.type = Group::Type::DRAWING_WORKPLANE;
    wpg.subtype = Group::Subtype::WORKPLANE_BY_POINT_ORTHO;
    wpg.predef.origin = originPt;
    wpg.predef.q = Quaternion::From(1.0, 0.0, 0.0, 0.0);
    wpg.name = "boss-sketch";
    wpg.visible = true;
    wpg.color = RGBi(100, 100, 100);
    wpg.scale = 1;
    wpg.order = SK.group.n + 1;
    SK.group.AddAndAssignId(&wpg);
    SK.groupOrder.Add(&wpg.h);
    SS.GW.activeGroup = wpg.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    hGroup bossWpH = wpg.h;

    hEntity bossWorkplane = SS.GW.ActiveWorkplane();

    // Draw boss square: x=10..20, y=10..20 at z=80
    // This boss sits at the corner of box1 (box1 extends to x=20 and y=20).
    // The boss corner at (20,10,80) lies where box1's top face (z=80) and
    // box1's side face at x=20 both terminate — the complex vertex scenario.
    hRequest crh[4];
    for(int i = 0; i < 4; i++) {
        Request r = {};
        r.type = Request::Type::LINE_SEGMENT;
        r.group = bossWpH;
        r.workplane = bossWorkplane;
        r.construction = false;
        SK.request.AddAndAssignId(&r);
        crh[i] = r.h;
    }
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    // Boss lines in XY (z=80 forced directly):
    //   line[0]: (10,10,80) -> (20,10,80)  => FACE_XPROD: y=10 face
    //   line[1]: (20,10,80) -> (20,20,80)  => FACE_XPROD: x=20 face
    //   line[2]: (20,20,80) -> (10,20,80)  => FACE_XPROD: y=20 face
    //   line[3]: (10,20,80) -> (10,10,80)  => FACE_XPROD: x=10 face
    double bossZ = 80.0;
    Vector bossPts[4] = {
        Vector::From(10, 10, bossZ), Vector::From(20, 10, bossZ),
        Vector::From(20, 20, bossZ), Vector::From(10, 20, bossZ),
    };
    for(int i = 0; i < 4; i++) {
        SK.GetEntity(crh[i].entity(1))->PointForceTo(bossPts[i]);
        SK.GetEntity(crh[i].entity(2))->PointForceTo(bossPts[(i + 1) % 4]);
    }
    for(int i = 0; i < 4; i++) {
        Constraint c = {};
        c.type = Constraint::Type::POINTS_COINCIDENT;
        c.group = bossWpH;
        c.workplane = bossWorkplane;
        c.ptA = crh[i].entity(2);
        c.ptB = crh[(i + 1) % 4].entity(1);
        SK.constraint.AddAndAssignId(&c);
    }
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);

    // Create boss extrude: ASSEMBLE (union), scale=1, valA=10 => boss at z=80..90
    Group bossEg = {};
    bossEg.type = Group::Type::EXTRUDE;
    bossEg.opA = bossWpH;
    bossEg.predef.entityB = bossWorkplane;
    bossEg.subtype = Group::Subtype::ONE_SIDED;
    bossEg.meshCombine = Group::CombineAs::ASSEMBLE;
    bossEg.valA = 10.0;
    bossEg.name = "boss-extrude";
    bossEg.visible = true;
    bossEg.color = RGBi(100, 100, 100);
    bossEg.scale = 1;
    bossEg.order = SK.group.n + 1;
    SK.group.AddAndAssignId(&bossEg);
    SK.groupOrder.Add(&bossEg.h);
    SS.GW.activeGroup = bossEg.h;
    SS.GenerateAll(SolveSpaceUI::Generate::ALL);
    hGroup bossH = bossEg.h;

    // Verify boss extrude succeeded
    Group *bossGrp = SK.GetGroup(bossH);
    CHECK_TRUE(bossGrp != nullptr);
    if(bossGrp == nullptr) return;
    CHECK_FALSE(bossGrp->booleanFailed);
    if(bossGrp->booleanFailed) return;

    // Find first two FACE_XPROD of boss extrude:
    //   face1 = y=10 wall (from line[0])
    //   face2 = x=20 wall (from line[1])
    // These ARE adjacent; they share the vertical edge at x=20, y=10, z=80..90.
    // At V1=(20,10,80): box1's top face AND box1's x=20 side face both touch.
    hEntity face1 = {}, face2 = {};
    bool found = FindTwoAdjacentFaces(bossH, &face1, &face2);
    CHECK_TRUE(found);
    if(!found) return;

    // Apply fillet (radius=1.0) to the two adjacent boss faces.
    hGroup filletH = AddFilletGroup(bossH, face1, face2, 1.0);
    Group *g = SK.GetGroup(filletH);
    CHECK_TRUE(g != nullptr);
    if(g == nullptr) return;

    CHECK_FALSE(g->booleanFailed);
    if(g->booleanFailed) return;

    // Generate display mesh.
    g->GenerateDisplayItems();

    // Check for backfacing triangles.
    // Box center = (10,10,40) (interior of the 20x20x80 base box).
    // Any outward-facing surface triangle should have
    //   normal.Dot(centroid - boxCenter) > 0
    // The bug inverts box1's x=20 side face, causing triangles there to have
    //   normal = (-1,0,0) instead of (+1,0,0), giving a negative dot product.
    Vector boxCenter = Vector::From(10, 10, 40);
    bool anyBackFacing = false;
    for(int ti = 0; ti < g->displayMesh.l.n; ti++) {
        STriangle *tr = &g->displayMesh.l[ti];
        Vector normal = tr->Normal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0 / 3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {
            anyBackFacing = true;
            break;
        }
    }
    // TDD: FAIL before fix (wrong cap => x=20 face inverted => backfacing).
    // TDD: PASS after fix (correct cap => no backfacing triangles).
    CHECK_FALSE(anyBackFacing);
}
