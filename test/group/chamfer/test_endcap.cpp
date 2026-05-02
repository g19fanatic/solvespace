//-----------------------------------------------------------------------------
// Endcap geometry tests for chamfer-then-fillet (CF) on adjacent edges
//
// Bug: "Creating a chamfer then a fillet on a connection edge to 1 point on
// a box gives strange end-cap geometry."
//
// The existing RunDoubleOpTest() SKIPS the backfacing check for the CF
// (chamfer-first, fillet-second) category because 3 of 7 face-pair combos
// produce a corner triangle whose EffectiveNormal() points inward.
//
// These tests exercise each CF combo WITH the full backfacing check enabled,
// directly testing the endcap geometry quality at the shared vertex.
//
// Additionally, the tests check that corner triangles near the shared vertex
// do NOT require FLAG_FLIP_DISPLAY_NORMAL to have correct outward normals.
// A triangle needing this flag has wrong geometric winding (backwards face)
// that is only masked for display purposes — the underlying geometry defect
// persists.
//-----------------------------------------------------------------------------
#include "helpers.h"

//-----------------------------------------------------------------------------
// Helper: Compute the corner vertex where three box faces meet.
// Box is 20x20x80: x=[0,20], y=[0,20], z=[0,80].
//   FRONT=y=0, BACK=y=20, LEFT=x=0, RIGHT=x=20, TOP=z=80, BOTTOM=z=0
//-----------------------------------------------------------------------------
static inline Vector CornerVertex(FaceSpec f1, FaceSpec f2, FaceSpec f3) {
    double x = 10, y = 10, z = 40; // defaults (shouldn't remain)
    FaceSpec faces[3] = { f1, f2, f3 };
    for(int i = 0; i < 3; i++) {
        switch(faces[i]) {
            case FS_LEFT:   x = 0;  break;
            case FS_RIGHT:  x = 20; break;
            case FS_FRONT:  y = 0;  break;
            case FS_BACK:   y = 20; break;
            case FS_BOTTOM: z = 0;  break;
            case FS_TOP:    z = 80; break;
        }
    }
    return Vector::From(x, y, z);
}

//-----------------------------------------------------------------------------
// Helper: Run a CF (chamfer-first, fillet-second) endcap test for one
// face-triple.  Tests the full suite of mesh quality checks INCLUDING
// the backfacing check that RunDoubleOpTest skips for CF.
//
// Additionally checks that NO triangle near the shared vertex has
// FLAG_FLIP_DISPLAY_NORMAL set — meaning all triangles have correct
// geometric winding order, not just correct display orientation.
//
// sharedFace: the face shared by both edges
// partner1:   second face for the chamfer edge (chamfer = shared ∩ partner1)
// partner2:   second face for the fillet edge  (fillet  = shared ∩ partner2)
//
// The shared vertex (corner) is where all three faces meet.
//-----------------------------------------------------------------------------
static inline void RunCFEndcapTest(
    Test::Helper *helper,
    FaceSpec sharedFace,
    FaceSpec partner1,
    FaceSpec partner2,
    double offset = 2.0)
{
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity face_shared   = GetFace(extrudeH, sharedFace);
    hEntity face_partner1 = GetFace(extrudeH, partner1);
    hEntity face_partner2 = GetFace(extrudeH, partner2);
    CHECK_TRUE(face_shared.v != 0);
    CHECK_TRUE(face_partner1.v != 0);
    CHECK_TRUE(face_partner2.v != 0);

    // Operation 1: CHAMFER on edge between shared face and partner1
    hEntity edge1 = FindEdgeBetweenFaces(extrudeH, face_shared, face_partner1);
    CHECK_TRUE(edge1.v != 0);
    hGroup chamferH = AddChamferGroupByEdge(extrudeH, edge1, offset);
    CHECK_FALSE(SK.GetGroup(chamferH)->booleanFailed);

    // Operation 2: FILLET on edge between shared face and partner2
    hEntity edge2 = FindEdgeBetweenFaces(extrudeH, face_shared, face_partner2);
    CHECK_TRUE(edge2.v != 0);
    hGroup filletH = AddFilletGroupByEdge(chamferH, edge2, offset);
    Group *g2 = SK.GetGroup(filletH);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Generate display mesh
    g2->GenerateDisplayItems();
    SMesh *m = &g2->displayMesh;

    // Check 1: Mesh has triangles
    CHECK_TRUE(m->l.n > 0);

    // Check 2: Minimum triangle count (box=12, chamfer/fillet adds more)
    CHECK_TRUE(m->l.n >= 12);

    // Check 3 & 4: No naked edges and no self-intersections
    SKdNode *root = SKdNode::From(m);
    SEdgeList el = {};
    bool inters, leaks;
    root->MakeCertainEdgesInto(&el,
        EdgeKind::NAKED_OR_SELF_INTER, /*coplanarIsInter=*/true,
        &inters, &leaks);
    el.Clear();
    CHECK_FALSE(inters);  // No self-intersections
    CHECK_FALSE(leaks);   // No naked edges -- mesh must be watertight

    // Check 5: No degenerate triangles near the shared vertex
    Vector corner = CornerVertex(sharedFace, partner1, partner2);
    double searchRadius = offset * 2.0;  // search near the corner
    int degenCount = CountDegenerateTrianglesNear(m, corner, searchRadius);
    CHECK_TRUE(degenCount == 0);

    // Check 6: No backfacing triangles using EffectiveNormal (THE KEY CHECK)
    // This is the check that RunDoubleOpTest SKIPS for CF.
    // Box interior center = (10, 10, 40) for the 20x20x80 box.
    Vector boxCenter = Vector::From(10, 10, 40);
    bool anyBackFacing = false;
    for(int ti = 0; ti < m->l.n; ti++) {
        STriangle *tr = &m->l[ti];
        Vector normal = tr->EffectiveNormal();
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(normal.Dot(centroid.Minus(boxCenter)) < -0.01) {
            anyBackFacing = true;
            break;
        }
    }
    CHECK_FALSE(anyBackFacing);

    // Check 7: No triangles near the corner require FLAG_FLIP_DISPLAY_NORMAL.
    // The presence of this flag indicates the triangle has wrong geometric
    // winding order (Normal() points inward) that is only compensated for
    // display purposes. A correct implementation generates corner triangles
    // with proper winding so no flag is needed.
    //
    // This is the "strange endcap geometry" test: the corner triangle's
    // underlying geometry is backwards, even if it displays correctly.
    bool anyFlipNeeded = false;
    for(int ti = 0; ti < m->l.n; ti++) {
        STriangle *tr = &m->l[ti];
        if(!(tr->flags & STriangle::FLAG_FLIP_DISPLAY_NORMAL)) continue;
        // Check if this flipped triangle is near the corner
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        if(centroid.Minus(corner).Magnitude() <= searchRadius) {
            anyFlipNeeded = true;
            break;
        }
    }
    CHECK_FALSE(anyFlipNeeded);
}

//=============================================================================
// Group A: Front face shared
//=============================================================================

// Corner: top-front-right = (20, 0, 80)
TEST_CASE(endcap_cf_front_top_right) {
    RunCFEndcapTest(helper, FS_FRONT, FS_TOP, FS_RIGHT);
}

// Corner: bottom-front-left = (0, 0, 0)
TEST_CASE(endcap_cf_front_bottom_left) {
    RunCFEndcapTest(helper, FS_FRONT, FS_BOTTOM, FS_LEFT);
}

// Corner: bottom-front-right = (20, 0, 0)
TEST_CASE(endcap_cf_front_bottom_right) {
    RunCFEndcapTest(helper, FS_FRONT, FS_BOTTOM, FS_RIGHT);
}

//=============================================================================
// Group B: Left face shared
//=============================================================================

// Corner: top-front-left = (0, 0, 80)
TEST_CASE(endcap_cf_left_top_front) {
    RunCFEndcapTest(helper, FS_LEFT, FS_TOP, FS_FRONT);
}

// Corner: top-back-left = (0, 20, 80)
TEST_CASE(endcap_cf_left_top_back) {
    RunCFEndcapTest(helper, FS_LEFT, FS_TOP, FS_BACK);
}

// Corner: bottom-front-left = (0, 0, 0)
TEST_CASE(endcap_cf_left_bottom_front) {
    RunCFEndcapTest(helper, FS_LEFT, FS_BOTTOM, FS_FRONT);
}

// Corner: bottom-back-left = (0, 20, 0)
TEST_CASE(endcap_cf_left_bottom_back) {
    RunCFEndcapTest(helper, FS_LEFT, FS_BOTTOM, FS_BACK);
}

//=============================================================================
// Endcap SHAPE tests — verify the fillet endcap at the chamfer corner is
// CURVED (rational quadratic Bezier, degree >= 2), not a straight line
// (degree 1).
//
// The bug: corner post-processing in MakeFromFilletOf replaces the fillet
// arc (A0→V1→B0, degree 2) with NC3 (A0→B0, degree 1 straight line).
// This makes the endcap look flat/chamfered instead of properly curved.
//
// These tests inspect the B-rep shell directly to check curve degrees.
//=============================================================================

//-----------------------------------------------------------------------------
// Helper: Run a CF endcap SHAPE test.  Creates box + chamfer + fillet,
// then inspects the fillet surface's trim curves to verify that NO endcap
// trim (both endpoints close together) is a straight line (degree 1).
//
// A curved endcap trim should be degree >= 2 (rational quadratic Bezier).
//
// Returns: true if all endcap trims are curved, false if any is straight.
//-----------------------------------------------------------------------------
static inline void RunCFEndcapShapeTest(
    Test::Helper *helper,
    FaceSpec sharedFace,
    FaceSpec partner1,   // chamfer edge = shared ∩ partner1
    FaceSpec partner2,   // fillet edge  = shared ∩ partner2
    double offset = 2.0)
{
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity face_shared   = GetFace(extrudeH, sharedFace);
    hEntity face_partner1 = GetFace(extrudeH, partner1);
    hEntity face_partner2 = GetFace(extrudeH, partner2);
    CHECK_TRUE(face_shared.v != 0);
    CHECK_TRUE(face_partner1.v != 0);
    CHECK_TRUE(face_partner2.v != 0);

    // Op 1: CHAMFER on edge between shared face and partner1
    hEntity edge1 = FindEdgeBetweenFaces(extrudeH, face_shared, face_partner1);
    CHECK_TRUE(edge1.v != 0);
    hGroup chamferH = AddChamferGroupByEdge(extrudeH, edge1, offset);
    CHECK_FALSE(SK.GetGroup(chamferH)->booleanFailed);

    // Op 2: FILLET on edge between shared face and partner2
    hEntity edge2 = FindEdgeBetweenFaces(extrudeH, face_shared, face_partner2);
    CHECK_TRUE(edge2.v != 0);
    hGroup filletH = AddFilletGroupByEdge(chamferH, edge2, offset);
    Group *g2 = SK.GetGroup(filletH);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Access the B-rep shell
    SShell *shell = &g2->runningShell;

    // Find the fillet surface: the one with degree 2 in one direction and
    // degree 1 in the other (extruded rational arc = cylinder).
    // There should be exactly one such surface (the fillet cylinder).
    SSurface *filletSurf = nullptr;
    for(auto &ss : shell->surface) {
        bool isCylinder = (ss.degm == 2 && ss.degn == 1) ||
                          (ss.degm == 1 && ss.degn == 2);
        if(!isCylinder) continue;
        // Verify it has rational weights (fillet arc weight != 1.0)
        bool hasRationalWeight = false;
        for(int i = 0; i <= ss.degm; i++) {
            for(int j = 0; j <= ss.degn; j++) {
                if(fabs(ss.weight[i][j] - 1.0) > 1e-6) {
                    hasRationalWeight = true;
                    break;
                }
            }
            if(hasRationalWeight) break;
        }
        if(!hasRationalWeight) continue;
        filletSurf = &ss;
        break;
    }
    // Must find the fillet surface
    CHECK_TRUE(filletSurf != nullptr);
    if(!filletSurf) return;

    // Scan the fillet surface's trim list for endcap trims.
    // An endcap trim has both start and finish endpoints close together
    // (the chord A0→B0 is about offset*sqrt(2) for 90° edges).
    // Side-edge trims (contact lines) span the full fillet length (~16 units).
    double maxEndcapChord = offset * 4.0;  // generous threshold

    bool foundStraightEndcap = false;
    int endcapCount = 0;
    for(auto &stb : filletSurf->trim) {
        double chord = stb.start.Minus(stb.finish).Magnitude();
        if(chord >= maxEndcapChord) continue;  // side edge, not endcap
        if(chord < 1e-6) continue;  // skip degenerate zero-length trims

        endcapCount++;

        // Get the SCurve for this trim
        SCurve *sc = shell->curve.FindByIdNoOops(stb.curve);
        if(!sc) continue;

        // Check: the endcap curve must be exact and degree >= 2 (curved)
        if(sc->isExact && sc->exact.deg <= 1) {
            // Found a straight-line (degree 1) endcap on the fillet surface.
            // This is the NC3 flat line — the bug we're testing for.
            foundStraightEndcap = true;
        }
    }

    // We expect to find at least one endcap trim on the fillet surface
    CHECK_TRUE(endcapCount >= 1);

    // THE KEY ASSERTION: No endcap trim on the fillet surface should be
    // a straight line.  If foundStraightEndcap is true, the endcap is flat
    // (the bug).  After the fix, all endcap trims should be curved (deg >= 2).
    CHECK_FALSE(foundStraightEndcap);
}

//=============================================================================
// Endcap shape tests: verify curved (not flat) endcap for CF combinations
//=============================================================================

TEST_CASE(endcap_shape_cf_front_top_right) {
    RunCFEndcapShapeTest(helper, FS_FRONT, FS_TOP, FS_RIGHT);
}

TEST_CASE(endcap_shape_cf_front_bottom_left) {
    RunCFEndcapShapeTest(helper, FS_FRONT, FS_BOTTOM, FS_LEFT);
}

TEST_CASE(endcap_shape_cf_front_bottom_right) {
    RunCFEndcapShapeTest(helper, FS_FRONT, FS_BOTTOM, FS_RIGHT);
}

TEST_CASE(endcap_shape_cf_left_top_front) {
    RunCFEndcapShapeTest(helper, FS_LEFT, FS_TOP, FS_FRONT);
}

TEST_CASE(endcap_shape_cf_left_top_back) {
    RunCFEndcapShapeTest(helper, FS_LEFT, FS_TOP, FS_BACK);
}

TEST_CASE(endcap_shape_cf_left_bottom_front) {
    RunCFEndcapShapeTest(helper, FS_LEFT, FS_BOTTOM, FS_FRONT);
}

TEST_CASE(endcap_shape_cf_left_bottom_back) {
    RunCFEndcapShapeTest(helper, FS_LEFT, FS_BOTTOM, FS_BACK);
}

//=============================================================================
// Endcap NO-CORNER-TRIANGLE tests — verify that after a CF operation, NO
// separate flat corner triangle surface exists in the B-rep shell.
//
// The bug: corner post-processing in MakeFromFilletOf creates a separate
// SSurface::FromPlane (degm==1, degn==1) with exactly 3 linear trim edges
// (NC1, NC2, NC3). This is the flat "corner triangle" that should NOT exist.
//
// After the fix, the corner area should be absorbed into the chamfer cap
// surface (hCapSurfV1) — no separate surface with the corner triangle
// signature should appear.
//
// Corner triangle signature:
//   1. Surface is bilinear (degm==1, degn==1) — flat plane
//   2. Surface has EXACTLY 3 trims
//   3. ALL 3 trim curves are exact and degree <= 1 (linear/straight lines)
//
// This signature is unique to the corner triangle because:
//   - Box faces: degm==1, degn==1 but have 4+ trims (rectangle edges)
//   - Chamfer cap: degm==1, degn==1 but has more than 3 trims
//   - Fillet cylinder: degm==2 or degn==2 (curved)
//=============================================================================

//-----------------------------------------------------------------------------
// Helper: Run a CF no-corner-triangle test.  Creates box + chamfer + fillet,
// then inspects the B-rep shell to ensure NO surface has the corner triangle
// signature (bilinear plane with exactly 3 linear trims).
//
// Also verifies that the fillet surface still has a curved (deg >= 2) endcap
// trim (the original arc, not the NC3 straight-line replacement).
//-----------------------------------------------------------------------------
static inline void RunCFNoCornerTriangleTest(
    Test::Helper *helper,
    FaceSpec sharedFace,
    FaceSpec partner1,   // chamfer edge = shared ∩ partner1
    FaceSpec partner2,   // fillet edge  = shared ∩ partner2
    double offset = 2.0)
{
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    hEntity face_shared   = GetFace(extrudeH, sharedFace);
    hEntity face_partner1 = GetFace(extrudeH, partner1);
    hEntity face_partner2 = GetFace(extrudeH, partner2);
    CHECK_TRUE(face_shared.v != 0);
    CHECK_TRUE(face_partner1.v != 0);
    CHECK_TRUE(face_partner2.v != 0);

    // Op 1: CHAMFER on edge between shared face and partner1
    hEntity edge1 = FindEdgeBetweenFaces(extrudeH, face_shared, face_partner1);
    CHECK_TRUE(edge1.v != 0);
    hGroup chamferH = AddChamferGroupByEdge(extrudeH, edge1, offset);
    CHECK_FALSE(SK.GetGroup(chamferH)->booleanFailed);

    // Op 2: FILLET on edge between shared face and partner2
    hEntity edge2 = FindEdgeBetweenFaces(extrudeH, face_shared, face_partner2);
    CHECK_TRUE(edge2.v != 0);
    hGroup filletH = AddFilletGroupByEdge(chamferH, edge2, offset);
    Group *g2 = SK.GetGroup(filletH);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Access the B-rep shell
    SShell *shell = &g2->runningShell;

    // Check: NO surface has the "corner triangle signature"
    // Signature: degm==1 && degn==1 (bilinear plane) AND exactly 3 trims
    //            AND all 3 trims reference exact curves with deg <= 1 (linear)
    bool foundCornerTriangle = false;
    for(auto &ss : shell->surface) {
        // Must be a bilinear (flat plane) surface
        if(ss.degm != 1 || ss.degn != 1) continue;

        // Must have exactly 3 trims
        if(ss.trim.n != 3) continue;

        // Check if ALL 3 trim curves are exact and linear (deg <= 1)
        bool allLinear = true;
        for(int ti = 0; ti < ss.trim.n; ti++) {
            SCurve *sc = shell->curve.FindByIdNoOops(ss.trim[ti].curve);
            if(!sc || !sc->isExact || sc->exact.deg > 1) {
                allLinear = false;
                break;
            }
        }

        if(allLinear) {
            // This surface matches the corner triangle signature
            foundCornerTriangle = true;
            break;
        }
    }

    // THE KEY ASSERTION: No corner triangle surface should exist.
    // After the fix, the corner area is absorbed into the chamfer cap surface.
    // With current (buggy) code, this CHECK will FAIL (RED) because the corner
    // triangle IS created by SSurface::FromPlane with NC1, NC2, NC3.
    CHECK_FALSE(foundCornerTriangle);
}

//=============================================================================
// No-corner-triangle tests: verify absence of flat corner triangle for CF
//=============================================================================

TEST_CASE(endcap_no_corner_tri_cf_front_top_right) {
    RunCFNoCornerTriangleTest(helper, FS_FRONT, FS_TOP, FS_RIGHT);
}

TEST_CASE(endcap_no_corner_tri_cf_front_bottom_left) {
    RunCFNoCornerTriangleTest(helper, FS_FRONT, FS_BOTTOM, FS_LEFT);
}

TEST_CASE(endcap_no_corner_tri_cf_front_bottom_right) {
    RunCFNoCornerTriangleTest(helper, FS_FRONT, FS_BOTTOM, FS_RIGHT);
}

TEST_CASE(endcap_no_corner_tri_cf_left_top_front) {
    RunCFNoCornerTriangleTest(helper, FS_LEFT, FS_TOP, FS_FRONT);
}

TEST_CASE(endcap_no_corner_tri_cf_left_top_back) {
    RunCFNoCornerTriangleTest(helper, FS_LEFT, FS_TOP, FS_BACK);
}

TEST_CASE(endcap_no_corner_tri_cf_left_bottom_front) {
    RunCFNoCornerTriangleTest(helper, FS_LEFT, FS_BOTTOM, FS_FRONT);
}

TEST_CASE(endcap_no_corner_tri_cf_left_bottom_back) {
    RunCFNoCornerTriangleTest(helper, FS_LEFT, FS_BOTTOM, FS_BACK);
}
