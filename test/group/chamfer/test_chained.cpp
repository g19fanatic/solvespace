//-----------------------------------------------------------------------------
// Chained operation tests
//-----------------------------------------------------------------------------
#include "helpers.h"

// Phase 1-2 TDD Tests: Extra line from origin bug in chained chamfer
// Fix plan items 1-14
//=============================================================================

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

