//-----------------------------------------------------------------------------
// Specific geometry verification tests for chamfer/fillet.
//-----------------------------------------------------------------------------
#include "helpers.h"

//-----------------------------------------------------------------------------
// Session 13, Fix plan item 10 — RED-PHASE test: chamfer cap extent correct.
//
// Same scenario as chamfer_cap_extent_diagnostic_left_top_back_CC but asserts
// the CORRECT cap2 extent (Z max = 80) rather than the current wrong one (78).
//
// RED state (current code): cap2 Z max = 78 → CHECK fails (|78-80|=2 > 0.5).
// GREEN state (after fix):  cap2 Z max = 80 → CHECK passes.
//
// Also verifies surface count: after fix the corner-triangle surface should
// be gone or reduced, so total surfaces ≤ 8 (was 9 with corner triangle).
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_cap_extent_correct_left_top_back_CC) {
    // --- Setup: 20×20×80 box ---
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity leftFace = GetFace(extrudeH, FS_LEFT);
    hEntity topFace  = GetFace(extrudeH, FS_TOP);
    hEntity backFace = GetFace(extrudeH, FS_BACK);
    CHECK_TRUE(leftFace.v != 0);
    CHECK_TRUE(topFace.v != 0);
    CHECK_TRUE(backFace.v != 0);

    // --- Chamfer 1: LEFT + TOP, dist=2 ---
    hGroup op1H = AddChamferGroup(extrudeH, leftFace, topFace, 2.0);
    CHECK_FALSE(SK.GetGroup(op1H)->booleanFailed);

    // --- Chamfer 2: LEFT + BACK, dist=2 ---
    hGroup op2H = AddChamferGroup(op1H, leftFace, backFace, 2.0);
    Group *g2 = SK.GetGroup(op2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // --- Find cap2: flat surface with normal ≈ (1,-1,0)/√2 ---
    SSurface *cap2 = nullptr;
    for(auto &ss : g2->runningShell.surface) {
        if(ss.degm != 1 || ss.degn != 1) continue;
        Vector n = ss.NormalAt(0.5, 0.5).WithMagnitude(1.0);
        if(fabs(n.x) > 0.3 && fabs(n.y) > 0.3 && fabs(n.z) < 0.3) {
            cap2 = &ss;
        }
    }
    CHECK_TRUE(cap2 != nullptr);
    if(!cap2) return;

    // --- Compute Z extent of cap2 ---
    Vector A = cap2->ctrl[0][0];
    Vector B = cap2->ctrl[0][1];
    Vector D = cap2->ctrl[1][0];
    Vector C = cap2->ctrl[1][1];

    double zMin = fmin(fmin(A.z, B.z), fmin(C.z, D.z));
    double zMax = fmax(fmax(A.z, B.z), fmax(C.z, D.z));
    double edgeExtent = zMax - zMin;

    dbp("RED_CAP2_EXTENT: Z range = [%.2f, %.2f], extent = %.2f", zMin, zMax, edgeExtent);

    // --- KEY ASSERTION: cap2 Z max must be ~80.0 (full edge extent) ---
    // On current (broken) code: zMax ≈ 78.0 → FAILS (|78-80| = 2 > 0.5)
    // After fix: zMax ≈ 80.0 → PASSES
    CHECK_TRUE(fabs(zMax - 80.0) < 0.5);

    // --- KEY ASSERTION: edge extent must be ~80 (full box height) ---
    // On current code: edgeExtent ≈ 78 → FAILS
    // After fix: edgeExtent ≈ 80 → PASSES
    CHECK_TRUE(fabs(edgeExtent - 80.0) < 0.5);

    // --- Surface count: with correct extent, corner triangle may be gone ---
    // Current: 9 (6 box + 2 caps + 1 corner triangle)
    // After fix: ≤ 8 expected (corner triangle eliminated or absorbed)
    // NOTE: This assertion may need adjustment; the primary assertions above
    // are the definitive ones for the cap extent bug.
    int surfCount = g2->runningShell.surface.n;
    dbp("RED_CAP2_EXTENT: surface count = %d (expected <= 8 after fix)", surfCount);
    CHECK_TRUE(surfCount <= 8);
}

//-----------------------------------------------------------------------------
// chamfer_geometry_verification_left_top_back_CC
//
// Verifies all 3 user-identified geometry requirements for the adjacent-chamfer
// fix (Session 13 reference screenshots):
//
//   (a) Cap1 and Cap2 share a common edge (the "blue diagonal line")
//       → An SCurve exists in the shell whose surfA/surfB pair references both
//         cap surfaces.
//   (b) Cap2's extent reaches the full edge (Z max ≈ 80 for this box)
//       → Cap2 ctrl points span Z=[0,80].
//   (c) No extra surfaces between the two caps (the "dark triangle" gap is gone)
//       → surfCount == 8 (6 box faces + 2 chamfer caps, no corner triangle).
//
// Additionally verifies the shared edge endpoints lie on the expected diagonal
// intersection line of the two cap planes.
//
// Geometry: 20×20×80 box, Chamfer1(LEFT+TOP, dist=2), Chamfer2(LEFT+BACK, dist=2)
//   LEFT=X=20, TOP=Z=80, BACK=Y=0
//   Cap1 plane: x + z = 98  (normal ≈ (1,0,1)/√2)
//   Cap2 plane: x - y = 18  (normal ≈ (1,-1,0)/√2)
//   Intersection line: (t, t-18, 98-t) for t ∈ [18,20]
//     from (18, 0, 80) to (20, 2, 78)
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_geometry_verification_left_top_back_CC) {
    // --- Setup: 20×20×80 box ---
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity leftFace = GetFace(extrudeH, FS_LEFT);
    hEntity topFace  = GetFace(extrudeH, FS_TOP);
    hEntity backFace = GetFace(extrudeH, FS_BACK);
    CHECK_TRUE(leftFace.v != 0);
    CHECK_TRUE(topFace.v != 0);
    CHECK_TRUE(backFace.v != 0);

    // --- Chamfer 1: LEFT + TOP, dist=2 ---
    hGroup op1H = AddChamferGroup(extrudeH, leftFace, topFace, 2.0);
    CHECK_FALSE(SK.GetGroup(op1H)->booleanFailed);

    // --- Chamfer 2: LEFT + BACK, dist=2 ---
    hGroup op2H = AddChamferGroup(op1H, leftFace, backFace, 2.0);
    Group *g2 = SK.GetGroup(op2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // --- Find cap1: flat surface with normal ≈ (1,0,1)/√2 ---
    hSSurface hCap1 = { 0 };
    SSurface *cap1 = nullptr;
    for(auto &ss : g2->runningShell.surface) {
        if(ss.degm != 1 || ss.degn != 1) continue;
        Vector n = ss.NormalAt(0.5, 0.5).WithMagnitude(1.0);
        if(fabs(n.x) > 0.3 && fabs(n.z) > 0.3 && fabs(n.y) < 0.3) {
            cap1 = &ss;
            hCap1 = ss.h;
        }
    }
    CHECK_TRUE(cap1 != nullptr);
    if(!cap1) return;

    // --- Find cap2: flat surface with normal ≈ (1,-1,0)/√2 ---
    hSSurface hCap2 = { 0 };
    SSurface *cap2 = nullptr;
    for(auto &ss : g2->runningShell.surface) {
        if(ss.degm != 1 || ss.degn != 1) continue;
        Vector n = ss.NormalAt(0.5, 0.5).WithMagnitude(1.0);
        if(fabs(n.x) > 0.3 && fabs(n.y) > 0.3 && fabs(n.z) < 0.3) {
            cap2 = &ss;
            hCap2 = ss.h;
        }
    }
    CHECK_TRUE(cap2 != nullptr);
    if(!cap2) return;

    dbp("GEOM_VERIFY: cap1 h=%08x, cap2 h=%08x", hCap1.v, hCap2.v);
    dbp("GEOM_VERIFY: cap1 normal=(%.4f,%.4f,%.4f)",
        cap1->NormalAt(0.5,0.5).WithMagnitude(1.0).x,
        cap1->NormalAt(0.5,0.5).WithMagnitude(1.0).y,
        cap1->NormalAt(0.5,0.5).WithMagnitude(1.0).z);
    dbp("GEOM_VERIFY: cap2 normal=(%.4f,%.4f,%.4f)",
        cap2->NormalAt(0.5,0.5).WithMagnitude(1.0).x,
        cap2->NormalAt(0.5,0.5).WithMagnitude(1.0).y,
        cap2->NormalAt(0.5,0.5).WithMagnitude(1.0).z);

    // =====================================================================
    // CONDITION (a): Cap1 and Cap2 share a common edge (SCurve)
    // =====================================================================
    bool sharedEdgeFound = false;
    Vector sharedEdgeStart = Vector::From(0, 0, 0);
    Vector sharedEdgeEnd   = Vector::From(0, 0, 0);
    for(auto &sc : g2->runningShell.curve) {
        if(sc.pts.n < 2) continue;
        bool hasCap1 = (sc.surfA == hCap1 || sc.surfB == hCap1);
        bool hasCap2 = (sc.surfA == hCap2 || sc.surfB == hCap2);
        if(hasCap1 && hasCap2) {
            sharedEdgeFound = true;
            sharedEdgeStart = sc.pts[0].p;
            sharedEdgeEnd   = sc.pts[sc.pts.n - 1].p;
            dbp("GEOM_VERIFY: shared edge found! start=(%.2f,%.2f,%.2f) end=(%.2f,%.2f,%.2f)",
                sharedEdgeStart.x, sharedEdgeStart.y, sharedEdgeStart.z,
                sharedEdgeEnd.x,   sharedEdgeEnd.y,   sharedEdgeEnd.z);
            break;
        }
    }
    CHECK_TRUE(sharedEdgeFound);

    // Verify the shared edge endpoints lie on the expected diagonal.
    // Expected diagonal: from (18, 0, 80) to (20, 2, 78)
    // (Or reversed depending on curve orientation)
    if(sharedEdgeFound) {
        Vector expA = Vector::From(18.0, 0.0, 80.0);
        Vector expB = Vector::From(20.0, 2.0, 78.0);
        double dSA = sharedEdgeStart.Minus(expA).Magnitude();
        double dSB = sharedEdgeStart.Minus(expB).Magnitude();
        double dEA = sharedEdgeEnd.Minus(expA).Magnitude();
        double dEB = sharedEdgeEnd.Minus(expB).Magnitude();
        // Either (start≈expA, end≈expB) or (start≈expB, end≈expA)
        bool match1 = (dSA < 1.0 && dEB < 1.0);
        bool match2 = (dSB < 1.0 && dEA < 1.0);
        dbp("GEOM_VERIFY: diagonal endpoint distances: dSA=%.2f dSB=%.2f dEA=%.2f dEB=%.2f",
            dSA, dSB, dEA, dEB);
        CHECK_TRUE(match1 || match2);
    }

    // =====================================================================
    // CONDITION (b): Cap2 Z range extends to full edge (Z max ≈ 80)
    // =====================================================================
    Vector A2 = cap2->ctrl[0][0];
    Vector B2 = cap2->ctrl[0][1];
    Vector D2 = cap2->ctrl[1][0];
    Vector C2 = cap2->ctrl[1][1];
    double zMax2 = fmax(fmax(A2.z, B2.z), fmax(C2.z, D2.z));
    double zMin2 = fmin(fmin(A2.z, B2.z), fmin(C2.z, D2.z));
    dbp("GEOM_VERIFY: cap2 Z range = [%.2f, %.2f]", zMin2, zMax2);
    CHECK_TRUE(fabs(zMax2 - 80.0) < 0.5);
    CHECK_TRUE(fabs(zMin2 - 0.0) < 0.5);

    // =====================================================================
    // CONDITION (c): No extra surfaces between the two caps
    //   surfCount == 8 (6 box + 2 caps, no corner triangle)
    // =====================================================================
    int surfCount = g2->runningShell.surface.n;
    dbp("GEOM_VERIFY: surface count = %d (expected 8)", surfCount);
    CHECK_TRUE(surfCount == 8);
}

//-----------------------------------------------------------------------------
// chamfer_adjacent_corner_display_excluded
//
// RED-PHASE test for display-level exclusion of the corner surface.
//
// The corner surface is kept in the B-rep for topological integrity and its
// triangles remain in the displayMesh for watertightness (naked-edge checks).
// However, each triangle is tagged with FLAG_DISPLAY_HIDDEN so rendering
// consumers can skip them.  This test verifies that every corner-surface
// triangle near the corner carries FLAG_DISPLAY_HIDDEN, i.e. there are ZERO
// "visible flipped" triangles (FLAG_FLIP_DISPLAY_NORMAL set without
// FLAG_DISPLAY_HIDDEN).
//
// Detection: count displayMesh triangles near the corner that carry
// FLAG_FLIP_DISPLAY_NORMAL but NOT FLAG_DISPLAY_HIDDEN.  These would be
// visible corner-surface triangles.  After excludeFromDisplay, every such
// triangle also has FLAG_DISPLAY_HIDDEN → visibleFlippedCount == 0 (GREEN).
//
// Setup: same as chamfer_adjacent_corner_surface_eliminated.
//   Box 20×20×80 → Chamfer1(front+top, d=2) → Chamfer2(front+left, d=2)
//   Corner at (20,20,78).
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_adjacent_corner_display_excluded) {
    hGroup extrudeH = CreateBoxExtrude();

    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // Chamfer 1: front face + top cap (dist=2.0)
    hGroup chamfer1H = AddChamferGroup(extrudeH, frontFace, topCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // Chamfer 2: front face + left face (dist=2.0) — shares corner with chamfer1
    hGroup chamfer2H = AddChamferGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Generate display mesh
    g2->GenerateDisplayItems();
    SMesh *dm = &g2->displayMesh;
    CHECK_TRUE(dm->l.n > 0);

    // Count "visible flipped" triangles near the corner: those with
    // FLAG_FLIP_DISPLAY_NORMAL set but NOT FLAG_DISPLAY_HIDDEN.
    // After excludeFromDisplay, every corner-surface triangle also carries
    // FLAG_DISPLAY_HIDDEN, so visibleFlippedCount == 0.
    Vector corner = Vector::From(20, 20, 78);
    double searchR = 3.0;
    int visibleFlippedCount = 0;
    for(int i = 0; i < dm->l.n; i++) {
        STriangle &t = dm->l[i];
        Vector mid = t.a.Plus(t.b).Plus(t.c).ScaledBy(1.0/3.0);
        if(mid.Minus(corner).Magnitude() > searchR) continue;
        if((t.flags & STriangle::FLAG_FLIP_DISPLAY_NORMAL) &&
           !(t.flags & STriangle::FLAG_DISPLAY_HIDDEN)) {
            visibleFlippedCount++;
        }
    }

    dbp("DIAG chamfer_adjacent_corner_display_excluded: visibleFlippedCount=%d",
        visibleFlippedCount);

    // After display-level exclusion: all corner-surface triangles carry
    // FLAG_DISPLAY_HIDDEN, so no "visible flipped" triangles remain.
    // RED: without excludeFromDisplay, visibleFlippedCount > 0.
    // GREEN: with excludeFromDisplay, visibleFlippedCount == 0.
    CHECK_TRUE(visibleFlippedCount == 0);
}

//-----------------------------------------------------------------------------
// Diagnostic test: fillet_adjacent_corner_flat_triangle_diagnostic
//
// PURPOSE: Investigate whether the fillet-fillet adjacent scenario has the same
// flat corner triangle artifact as the chamfer-chamfer adjacent scenario.
//
// SCENARIO (same geometry as fillet_double_fillet_no_naked_edges):
//   Box 20×20×80 → Fillet1(front+top, r=2) → Fillet2(front+left, r=2)
//   Shared corner setback at (0,20,78) — fillet1 endpoint on the vertical edge.
//
// The fillet corner-synthesis code (MakeFromFilletOf, chamfer.cpp:2305) uses
// SSurface::FromPlane() — the SAME flat surface as the chamfer path. For
// adjacent fillets, V1=(0,20,78), A0=(2,20,78), B0=(0,18,78) are all at z=78,
// so the corner surface is a flat triangle with pure -Z normal — identical
// artifact as the chamfer-chamfer case.
//
// This test documents whether the fillet case has this artifact and whether
// the mesh remains watertight.
//-----------------------------------------------------------------------------
TEST_CASE(fillet_adjacent_corner_flat_triangle_diagnostic) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // --- Fillet 1: face@Y=20 + top cap (radius=2.0) ---
    hGroup fillet1H = AddFilletGroup(extrudeH, frontFace, topCap, 2.0);
    Group *g1 = SK.GetGroup(fillet1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // --- Fillet 2: face@Y=20 + face@X=0 (radius=2.0) ---
    hGroup fillet2H = AddFilletGroup(fillet1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(fillet2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    g2->GenerateDisplayItems();
    SMesh *dm = &g2->displayMesh;
    CHECK_TRUE(dm->l.n > 0);

    // --- Watertightness check ---
    SKdNode *root = SKdNode::From(dm);
    SEdgeList el = {};
    bool inters = false, leaks = false;
    root->MakeCertainEdgesInto(&el,
        EdgeKind::NAKED_OR_SELF_INTER, /*coplanarIsInter=*/true,
        &inters, &leaks);
    el.Clear();
    CHECK_FALSE(leaks);
    CHECK_FALSE(inters);

    // --- Corner triangle scan ---
    // The shared corner setback: fillet1 placed (0,20,78) on the back-left
    // vertical edge. Use same coordinate as existing fillet-adjacent tests.
    Vector corner = Vector::From(0, 20, 78);
    double searchR = 3.0;

    int nearCount = 0;
    int flatMinusZCount = 0;
    for(int i = 0; i < dm->l.n; i++) {
        STriangle t = dm->l[i];
        Vector centroid = t.a.Plus(t.b).Plus(t.c).ScaledBy(1.0/3.0);
        bool near = (centroid.Minus(corner).Magnitude() <= searchR) ||
                    (t.a.Minus(corner).Magnitude() <= searchR) ||
                    (t.b.Minus(corner).Magnitude() <= searchR) ||
                    (t.c.Minus(corner).Magnitude() <= searchR);
        if(!near) continue;
        nearCount++;

        Vector n = t.Normal();
        if(n.z < -0.1 && fabs(n.x) < 0.1 && fabs(n.y) < 0.1) {
            flatMinusZCount++;
            dbp("FILLET_DIAG: flat -Z triangle TRI[%d] face=0x%08x "
                "normal=(%.4f,%.4f,%.4f) verts=(%.1f,%.1f,%.1f)(%.1f,%.1f,%.1f)(%.1f,%.1f,%.1f)",
                i, t.meta.face,
                n.x, n.y, n.z,
                t.a.x, t.a.y, t.a.z,
                t.b.x, t.b.y, t.b.z,
                t.c.x, t.c.y, t.c.z);
            CHECK_TRUE(t.meta.face != 0);
        }
    }

    dbp("FILLET_DIAG: total dm triangles=%d near(0,20,78) r=3.0=%d flatMinusZ=%d",
        dm->l.n, nearCount, flatMinusZCount);

    // FINDING: fillet-fillet adjacent does NOT have the flat corner triangle
    // artifact. Despite the fillet corner-synthesis code (chamfer.cpp:2305)
    // also using FromPlane(), the fillet case produces 0 flat -Z triangles.
    // This is chamfer-specific — likely because the fillet arc connectivity
    // at the corner prevents the flat corner surface from being needed or
    // the bridge pattern doesn't match in the fillet-fillet case.
    CHECK_TRUE(nearCount >= 5);
    // VERIFIED: 0 flat -Z triangles in fillet-fillet adjacent case.
    // The artifact is chamfer-specific, not present in the fillet path.
    CHECK_TRUE(flatMinusZCount == 0);
}

// Test: asymmetric chamfer distances (dist1=4, dist2=1) — no self-intersections/leaks
// The diagonal trim produces geometrically correct cap even when distances differ
// (cap2 extends to where it meets cap1's plane, not to full edge Z=80).
TEST_CASE(chamfer_asymmetric_dist_large_small_CC) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity leftFace = GetFace(extrudeH, FS_LEFT);
    hEntity topFace  = GetFace(extrudeH, FS_TOP);
    hEntity backFace = GetFace(extrudeH, FS_BACK);

    hGroup op1H = AddChamferGroup(extrudeH, leftFace, topFace, 4.0);
    CHECK_FALSE(SK.GetGroup(op1H)->booleanFailed);

    hGroup op2H = AddChamferGroup(op1H, leftFace, backFace, 1.0);
    Group *g2 = SK.GetGroup(op2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check topology: no self-intersections, no naked edges
    g2->GenerateDisplayItems();
    SMesh *m = &g2->displayMesh;
    SEdgeList el = {};
    bool inters, leaks;
    SKdNode::From(m)->MakeCertainEdgesInto(&el,
        EdgeKind::SELF_INTER, /*coplanarIsInter=*/false, &inters, &leaks);
    el.Clear();
    dbp("ASYM_LARGE_SMALL: inters=%s, leaks=%s",
        inters ? "true" : "false", leaks ? "true" : "false");
    CHECK_FALSE(inters);
    CHECK_FALSE(leaks);
}

//-----------------------------------------------------------------------------
// RED-PHASE TDD test: chamfer_adjacent_entity_endpoints_match_shell
//
// BUG: Entity generation in group.cpp computes chamfer setback points A, B, C, D
// from V1/V2 read from opA's runningShell WITHOUT performing Step 5.5 (V extension)
// or Step 8.5 (diagonal trim) that MakeFromChamferOf performs. For adjacent chamfers,
// the entity POINT_N_COPY positions are at the WRONG (truncated) positions while
// the B-rep cap surface has the CORRECT (extended+trimmed) positions.
//
// SCENARIO: LEFT+TOP+BACK CC double chamfer
//   Box 20×20×80
//   Chamfer1: LEFT + TOP, dist=2 → chamfers top-left horizontal edge
//   Chamfer2: LEFT + BACK, dist=2 → chamfers left-back vertical edge
//
// After chamfer1, the left-back vertical edge V1 is truncated from Z=80 to Z=78.
// MakeFromChamferOf (B-rep) extends V1 back to Z=80 via Step 5.5, then diagonal-
// trims via Step 8.5. But the entity generation code reads V1 at Z=78 and computes
// entity points from that → entity positions ≠ B-rep cap trim corner positions.
//
// ASSERTIONS:
//   1. chamfer2 group has exactly 4 POINT_N_COPY entities (A, B, D, C)
//   2. chamfer2's cap surface exists via REMAP_CHAMFER_FACE
//   3. The cap surface has exactly 4 trim entries with 4 unique corner vertices
//   4. EVERY entity point matches SOME trim corner vertex (within LENGTH_EPS)
//
// RED: On current code, assertion (4) FAILS because at least one entity point
//      is computed from the truncated V1 (Z=78) while the cap trim corner is
//      at the extended position (Z=80 or diagonal-trimmed).
// GREEN: After fixing entity generation to replicate Step 5.5/8.5 or read
//        from the group's own runningShell, all entity points match.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_adjacent_entity_endpoints_match_shell) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity leftFace = GetFace(extrudeH, FS_LEFT);
    hEntity topFace  = GetFace(extrudeH, FS_TOP);
    hEntity backFace = GetFace(extrudeH, FS_BACK);
    CHECK_TRUE(leftFace.v != 0);
    CHECK_TRUE(topFace.v != 0);
    CHECK_TRUE(backFace.v != 0);

    // --- Chamfer 1: LEFT + TOP, dist=2 ---
    hGroup op1H = AddChamferGroup(extrudeH, leftFace, topFace, 2.0);
    CHECK_FALSE(SK.GetGroup(op1H)->booleanFailed);

    // --- Chamfer 2: LEFT + BACK, dist=2 ---
    hGroup op2H = AddChamferGroup(op1H, leftFace, backFace, 2.0);
    Group *g2 = SK.GetGroup(op2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // --- Step 1: Collect chamfer2's 4 POINT_N_COPY entity positions ---
    std::vector<Vector> entityPts;
    for(auto &e : SK.entity) {
        if(e.group != op2H) continue;
        if(!e.IsPoint()) continue;
        entityPts.push_back(e.PointGetNum());
    }
    dbp("DIAG chamfer_adjacent_entity_endpoints_match_shell: entityPts.size=%d (expected 4)",
        (int)entityPts.size());
    CHECK_TRUE(entityPts.size() == 4);
    if(entityPts.size() != 4) return;

    // --- Step 2: Find chamfer2's cap surface via REMAP_CHAMFER_FACE ---
    hEntity chamferFaceH2 = g2->Remap(g2->predef.entityB, Group::REMAP_CHAMFER_FACE);
    SSurface *capSurf = nullptr;
    for(auto &ss : g2->runningShell.surface) {
        if(ss.face == chamferFaceH2.v) { capSurf = &ss; break; }
    }
    CHECK_TRUE(capSurf != nullptr);
    if(!capSurf) return;

    // --- Step 3: Extract unique trim corner vertices from the cap surface ---
    int nTrims = capSurf->trim.n;
    dbp("DIAG: chamfer2 cap nTrims=%d", nTrims);
    CHECK_TRUE(nTrims >= 4);

    std::vector<Vector> trimCorners;
    for(int i = 0; i < nTrims; i++) {
        STrimBy &stb = capSurf->trim.Get(i);
        auto addUnique = [&](Vector v) {
            for(const Vector &u : trimCorners) {
                if(u.Equals(v)) return;
            }
            trimCorners.push_back(v);
        };
        addUnique(stb.start);
        addUnique(stb.finish);
    }
    dbp("DIAG: chamfer2 cap unique trim corners=%d (expected 4)", (int)trimCorners.size());
    CHECK_TRUE(trimCorners.size() == 4);

    // --- Step 4: Verify EVERY entity point matches SOME trim corner ---
    // This is the KEY assertion that FAILS on current code (RED).
    int matchCount = 0;
    for(const Vector &ep : entityPts) {
        bool matched = false;
        for(const Vector &tc : trimCorners) {
            if(ep.Equals(tc)) { matched = true; break; }
        }
        dbp("DIAG: entity pt (%.4f,%.4f,%.4f) matched=%s",
            ep.x, ep.y, ep.z, matched ? "YES" : "NO");
        if(matched) matchCount++;
    }
    dbp("DIAG: matchCount=%d / %d", matchCount, (int)entityPts.size());

    // ALL 4 entity points must match a trim corner.
    // RED: at least one entity point is at the truncated V1 position (Z=78)
    // while the corresponding trim corner is at the extended position.
    CHECK_TRUE(matchCount == (int)entityPts.size());
}

// Test: asymmetric chamfer distances (dist1=1, dist2=4) — no self-intersections/leaks
// When dist2 > dist1, the revert guard in Step 8.5 prevents the V extension from
// creating an oversized cap that physically overlaps the adjacent cap. The cap falls
// back to the truncated behavior (corner triangle fills the gap).
TEST_CASE(chamfer_asymmetric_dist_small_large_CC) {
    hGroup extrudeH = CreateBoxExtrude();
    hEntity leftFace = GetFace(extrudeH, FS_LEFT);
    hEntity topFace  = GetFace(extrudeH, FS_TOP);
    hEntity backFace = GetFace(extrudeH, FS_BACK);

    hGroup op1H = AddChamferGroup(extrudeH, leftFace, topFace, 1.0);
    CHECK_FALSE(SK.GetGroup(op1H)->booleanFailed);

    hGroup op2H = AddChamferGroup(op1H, leftFace, backFace, 4.0);
    Group *g2 = SK.GetGroup(op2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Check topology: no self-intersections, no naked edges
    g2->GenerateDisplayItems();
    SMesh *m = &g2->displayMesh;
    SEdgeList el = {};
    bool inters, leaks;
    SKdNode::From(m)->MakeCertainEdgesInto(&el,
        EdgeKind::SELF_INTER, /*coplanarIsInter=*/false, &inters, &leaks);
    el.Clear();
    dbp("ASYM_SMALL_LARGE: inters=%s, leaks=%s",
        inters ? "true" : "false", leaks ? "true" : "false");
    CHECK_FALSE(inters);
    CHECK_FALSE(leaks);
}
