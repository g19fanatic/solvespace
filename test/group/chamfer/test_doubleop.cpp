//-----------------------------------------------------------------------------
// Double operation and parametric tests
//-----------------------------------------------------------------------------
#include "helpers.h"

//-----------------------------------------------------------------------------
// TDD test: fillet_double_fillet_no_naked_edges
//
// Fillet 1: front face (0,-1,0) + top cap → fillets top-front horizontal edge
// Fillet 2: front face (0,-1,0) + left face (-1,0,0) → fillets front-left vertical edge
//
// Before fix: 14 naked edges in display mesh (leaks=true).
// After fix:  mesh is watertight (leaks=false).
//-----------------------------------------------------------------------------
TEST_CASE(fillet_double_fillet_no_naked_edges) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    hGroup fillet1H = AddFilletGroup(extrudeH, frontFace, topCap, 2.0);
    CHECK_FALSE(SK.GetGroup(fillet1H)->booleanFailed);

    hGroup fillet2H = AddFilletGroup(fillet1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(fillet2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    g2->GenerateDisplayItems();
    SMesh *m = &g2->displayMesh;
    SKdNode *root = SKdNode::From(m);
    SEdgeList el = {};
    bool inters, leaks;
    root->MakeCertainEdgesInto(&el,
        EdgeKind::NAKED_OR_SELF_INTER, /*coplanarIsInter=*/true,
        &inters, &leaks);
    el.Clear();
    CHECK_FALSE(leaks);
}

//-----------------------------------------------------------------------------
// TDD test: chamfer_double_chamfer_no_naked_edges
//
// Chamfer 1: front face (0,-1,0) + top cap → chamfers top-front horizontal edge
// Chamfer 2: front face (0,-1,0) + left face (-1,0,0) → chamfers front-left vertical edge
//
// Before fix: chamfer2 mesh has naked edges (leaks=true).
// After fix:  mesh is watertight (leaks=false).
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_double_chamfer_no_naked_edges) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    hGroup chamfer1H = AddChamferGroup(extrudeH, frontFace, topCap, 2.0);
    CHECK_FALSE(SK.GetGroup(chamfer1H)->booleanFailed);

    hGroup chamfer2H = AddChamferGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    g2->GenerateDisplayItems();
    SMesh *m = &g2->displayMesh;
    SKdNode *root = SKdNode::From(m);
    SEdgeList el = {};
    bool inters, leaks;
    root->MakeCertainEdgesInto(&el,
        EdgeKind::NAKED_OR_SELF_INTER, /*coplanarIsInter=*/true,
        &inters, &leaks);
    el.Clear();
    CHECK_FALSE(leaks);  // No naked edges -- mesh must be watertight
}

//-----------------------------------------------------------------------------
// TDD test: mixed_chamfer_fillet_no_naked_edges
//
// Chamfer 1: front face (0,-1,0) + top cap → chamfers top-front horizontal edge
// Fillet 2: front face (0,-1,0) + left face (-1,0,0) → fillets front-left vertical edge
//
// Before fix: fillet2 mesh has naked edges (leaks=true).
// After fix:  mesh is watertight (leaks=false).
//-----------------------------------------------------------------------------
TEST_CASE(mixed_chamfer_fillet_no_naked_edges) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // Apply chamfer first, then fillet (mixed — reverse order)
    hGroup chamfer1H = AddChamferGroup(extrudeH, frontFace, topCap, 2.0);
    CHECK_FALSE(SK.GetGroup(chamfer1H)->booleanFailed);

    hGroup fillet2H = AddFilletGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(fillet2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check the final mesh for naked edges (watertightness).
    // Before fix: leaks=true (naked edges); After fix: leaks=false.
    g2->GenerateDisplayItems();
    SMesh *m = &g2->displayMesh;
    SKdNode *root = SKdNode::From(m);
    SEdgeList el = {};
    bool inters, leaks;
    root->MakeCertainEdgesInto(&el,
        EdgeKind::NAKED_OR_SELF_INTER, /*coplanarIsInter=*/true,
        &inters, &leaks);
    el.Clear();
    CHECK_FALSE(leaks);  // No naked edges -- mesh must be watertight
}

//-----------------------------------------------------------------------------
// TDD test: mixed_fillet_chamfer_no_naked_edges
//
// Fillet 1: front face (0,-1,0) + top cap → fillets top-front horizontal edge
// Chamfer 2: front face (0,-1,0) + left face (-1,0,0) → chamfers front-left vertical edge
//
// Before fix: chamfer2 mesh has naked edges (leaks=true).
// After fix:  mesh is watertight (leaks=false).
//-----------------------------------------------------------------------------
TEST_CASE(mixed_fillet_chamfer_no_naked_edges) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // Apply fillet first, then chamfer (mixed)
    hGroup fillet1H = AddFilletGroup(extrudeH, frontFace, topCap, 2.0);
    CHECK_FALSE(SK.GetGroup(fillet1H)->booleanFailed);

    hGroup chamfer2H = AddChamferGroup(fillet1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check the final mesh for naked edges (watertightness).
    // Before fix: leaks=true (naked edges); After fix: leaks=false.
    g2->GenerateDisplayItems();
    SMesh *m = &g2->displayMesh;
    SKdNode *root = SKdNode::From(m);
    SEdgeList el = {};
    bool inters, leaks;
    root->MakeCertainEdgesInto(&el,
        EdgeKind::NAKED_OR_SELF_INTER, /*coplanarIsInter=*/true,
        &inters, &leaks);
    el.Clear();
    CHECK_FALSE(leaks);  // No naked edges -- mesh must be watertight
}

//=============================================================================
// Group A, Corner TFR: Front face shared, top + right partners
// Corner vertex: top-front-right = (20, 0, 80)
//=============================================================================

TEST_CASE(doubleop_front_top_right_FF) {
    RunDoubleOpTest(helper, FS_FRONT, FS_TOP, FS_RIGHT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_front_top_right_CC) {
    RunDoubleOpTest(helper, FS_FRONT, FS_TOP, FS_RIGHT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_front_top_right_CF) {
    RunDoubleOpTest(helper, FS_FRONT, FS_TOP, FS_RIGHT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_front_top_right_FC) {
    RunDoubleOpTest(helper, FS_FRONT, FS_TOP, FS_RIGHT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/true);
}

//=============================================================================
// Group A, Corner BFL: Front face shared, bottom + left partners
// Corner vertex: bottom-front-left = (0, 0, 0)
//=============================================================================

TEST_CASE(doubleop_front_bottom_left_FF) {
    RunDoubleOpTest(helper, FS_FRONT, FS_BOTTOM, FS_LEFT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_front_bottom_left_CC) {
    RunDoubleOpTest(helper, FS_FRONT, FS_BOTTOM, FS_LEFT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_front_bottom_left_CF) {
    RunDoubleOpTest(helper, FS_FRONT, FS_BOTTOM, FS_LEFT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_front_bottom_left_FC) {
    RunDoubleOpTest(helper, FS_FRONT, FS_BOTTOM, FS_LEFT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_front_bottom_right_FF) {
    RunDoubleOpTest(helper, FS_FRONT, FS_BOTTOM, FS_RIGHT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_front_bottom_right_CC) {
    RunDoubleOpTest(helper, FS_FRONT, FS_BOTTOM, FS_RIGHT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_front_bottom_right_CF) {
    RunDoubleOpTest(helper, FS_FRONT, FS_BOTTOM, FS_RIGHT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_front_bottom_right_FC) {
    RunDoubleOpTest(helper, FS_FRONT, FS_BOTTOM, FS_RIGHT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_left_top_front_FF) {
    RunDoubleOpTest(helper, FS_LEFT, FS_TOP, FS_FRONT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_left_top_front_CC) {
    RunDoubleOpTest(helper, FS_LEFT, FS_TOP, FS_FRONT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_left_top_front_CF) {
    RunDoubleOpTest(helper, FS_LEFT, FS_TOP, FS_FRONT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_left_top_front_FC) {
    RunDoubleOpTest(helper, FS_LEFT, FS_TOP, FS_FRONT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_left_top_back_FF) {
    RunDoubleOpTest(helper, FS_LEFT, FS_TOP, FS_BACK,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_left_top_back_CC) {
    RunDoubleOpTest(helper, FS_LEFT, FS_TOP, FS_BACK,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_left_top_back_CF) {
    RunDoubleOpTest(helper, FS_LEFT, FS_TOP, FS_BACK,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_left_top_back_FC) {
    RunDoubleOpTest(helper, FS_LEFT, FS_TOP, FS_BACK,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_left_bottom_front_FF) {
    RunDoubleOpTest(helper, FS_LEFT, FS_BOTTOM, FS_FRONT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_left_bottom_front_CC) {
    RunDoubleOpTest(helper, FS_LEFT, FS_BOTTOM, FS_FRONT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_left_bottom_front_CF) {
    RunDoubleOpTest(helper, FS_LEFT, FS_BOTTOM, FS_FRONT,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_left_bottom_front_FC) {
    RunDoubleOpTest(helper, FS_LEFT, FS_BOTTOM, FS_FRONT,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_left_bottom_back_FF) {
    RunDoubleOpTest(helper, FS_LEFT, FS_BOTTOM, FS_BACK,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_left_bottom_back_CC) {
    RunDoubleOpTest(helper, FS_LEFT, FS_BOTTOM, FS_BACK,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/true);
}

TEST_CASE(doubleop_left_bottom_back_CF) {
    RunDoubleOpTest(helper, FS_LEFT, FS_BOTTOM, FS_BACK,
                    /*op1_chamfer=*/true, /*op2_chamfer=*/false);
}

TEST_CASE(doubleop_left_bottom_back_FC) {
    RunDoubleOpTest(helper, FS_LEFT, FS_BOTTOM, FS_BACK,
                    /*op1_chamfer=*/false, /*op2_chamfer=*/true);
}

//-----------------------------------------------------------------------------
// Characterization test: SSurface::MakeTrimEdgesInto basic forward/backward
//-----------------------------------------------------------------------------
TEST_CASE(surface_MakeTrimEdgesInto_basic) {
    // Create a simple plane surface: origin at (0,0,0), u=(100,0,0), v=(0,100,0)
    SSurface srf = SSurface::FromPlane(
        Vector::From(0, 0, 0),
        Vector::From(100, 0, 0),
        Vector::From(0, 100, 0));

    // Create an SCurve with 5 points along the bottom edge: x=0..100, y=0, z=0
    SCurve sc = {};
    sc.h.v = 0x100;  // arbitrary handle
    for(int i = 0; i <= 4; i++) {
        SCurvePt scp = {};
        scp.p = Vector::From(i * 25.0, 0, 0);
        scp.vertex = (i == 0 || i == 4);
        sc.pts.Add(&scp);
    }

    // --- Test forward trim: start=(0,0,0) finish=(100,0,0) ---
    {
        STrimBy stb = {};
        stb.curve.v = sc.h.v;
        stb.backwards = false;
        stb.start  = Vector::From(0, 0, 0);
        stb.finish = Vector::From(100, 0, 0);

        SEdgeList sel = {};
        srf.MakeTrimEdgesInto(&sel, SSurface::MakeAs::XYZ, &sc, &stb);

        // Should produce 4 edges: (0,0,0)→(25,0,0), (25,0,0)→(50,0,0),
        // (50,0,0)→(75,0,0), (75,0,0)→(100,0,0)
        CHECK_TRUE(sel.l.n == 4);

        // Verify first edge starts at (0,0,0)
        CHECK_TRUE(sel.l[0].a.Equals(Vector::From(0, 0, 0)));
        CHECK_TRUE(sel.l[0].b.Equals(Vector::From(25, 0, 0)));

        // Verify last edge ends at (100,0,0)
        CHECK_TRUE(sel.l[3].a.Equals(Vector::From(75, 0, 0)));
        CHECK_TRUE(sel.l[3].b.Equals(Vector::From(100, 0, 0)));

        // Verify auxA is set to curve handle value
        CHECK_TRUE(sel.l[0].auxA == (int)sc.h.v);
        // Verify auxB is 0 for forward trim (backwards=false → cast to int 0)
        CHECK_TRUE(sel.l[0].auxB == 0);

        sel.Clear();
    }

    // --- Test backward trim: start=(100,0,0) finish=(0,0,0) ---
    {
        STrimBy stb = {};
        stb.curve.v = sc.h.v;
        stb.backwards = true;
        stb.start  = Vector::From(100, 0, 0);
        stb.finish = Vector::From(0, 0, 0);

        SEdgeList sel = {};
        srf.MakeTrimEdgesInto(&sel, SSurface::MakeAs::XYZ, &sc, &stb);

        // Should produce 4 edges in reverse direction:
        // (100,0,0)→(75,0,0), (75,0,0)→(50,0,0), etc.
        CHECK_TRUE(sel.l.n == 4);

        // Verify first edge starts at (100,0,0) — backward iteration
        CHECK_TRUE(sel.l[0].a.Equals(Vector::From(100, 0, 0)));
        CHECK_TRUE(sel.l[0].b.Equals(Vector::From(75, 0, 0)));

        // Verify last edge ends at (0,0,0)
        CHECK_TRUE(sel.l[3].a.Equals(Vector::From(25, 0, 0)));
        CHECK_TRUE(sel.l[3].b.Equals(Vector::From(0, 0, 0)));

        // Verify auxB is 1 for backward trim (backwards=true → cast to int 1)
        CHECK_TRUE(sel.l[0].auxB == 1);

        sel.Clear();
    }

    // Cleanup
    sc.pts.Clear();
    srf.Clear();
}
