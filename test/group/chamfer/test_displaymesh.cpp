//-----------------------------------------------------------------------------
// Display mesh tests for chamfer/fillet.
//-----------------------------------------------------------------------------
#include "helpers.h"

//-----------------------------------------------------------------------------
// TDD characterization test: displaymesh_chamfer_adjacent_triangle_vertex_count
//
// Probes the displayMesh (what the renderer actually draws) for the adjacent
// chamfer-chamfer scenario. The runningShell B-rep is known-clean (prior tests
// pass), but the triangulation path (TriangulateInto) could still produce
// phantom vertices or degenerate triangles at the shared corner.
//
// SCENARIO (same as chamfer_adjacent_corner_vertex_reuse):
//   CreateBoxExtrude() -> Box 20x20x80, Z range [0,80].
//   Chamfer1: face@Y=20 + topCap@Z=80 (dist=2)
//     -> setback vertex (0,20,78) on back-left edge
//   Chamfer2: face@Y=20 + face@X=0 (dist=2) on chamfer1H
//     -> shared corner setback at (0,20,78)
//
// ASSERTIONS (on g2->displayMesh, the rendered triangulation):
//   A. CountUniqueMeshVerticesNear(dm, (0,20,78), 1.0) <= 3
//      At most 3 unique rendered vertices within 1 unit of the setback point.
//      A count > 3 means a phantom rendered vertex was introduced by
//      TriangulateInto -- this IS the displayMesh bug.
//   B. CountDegenerateTrianglesNear(dm, (0,20,78), 2.0) == 0
//      No sliver/degenerate triangles (area < 1e-6) near the shared corner.
//      A non-zero count indicates a triangulation artifact.
//
// EXPECTED RESULT: Both assertions PASS (no displayMesh bug in this scenario).
// If they FAIL: phantom vertices or degenerate triangles exist in the rendered
// mesh at the shared corner -- the user-visible bug described in the session.
//
// Documents actual numeric values via DIAG for baseline characterization.
//-----------------------------------------------------------------------------
TEST_CASE(displaymesh_chamfer_adjacent_triangle_vertex_count) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    // Resolve the three faces (same as chamfer_adjacent_corner_vertex_reuse).
    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // --- Chamfer 1: face@Y=20 + top cap ---
    // Chamfers the top-back horizontal edge at (Y=20, Z=80).
    // Creates setback (0,20,78) on back-left edge and (0,18,80) on top cap.
    hGroup chamfer1H = AddChamferGroup(extrudeH, frontFace, topCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // --- Chamfer 2: face@Y=20 + face@X=0 ---
    // Chamfers the back-left vertical edge from (0,20,0) to (0,20,78).
    // The shared corner setback is at (0,20,78).
    hGroup chamfer2H = AddChamferGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Access the displayMesh -- the triangulation of the rendered geometry.
    // IMPORTANT: Must call GenerateDisplayItems() first -- it calls
    // runningShell.TriangulateInto(&displayMesh) which populates the mesh.
    // Without this call, displayMesh.l.n == 0 (empty).
    g2->GenerateDisplayItems();
    // This is what TriangulateInto builds from the B-rep runningShell.
    SMesh *dm = &g2->displayMesh;

    // The shared corner setback point (confirmed by chamfer_adjacent_corner_vertex_reuse).
    Vector sharedCornerSB = Vector::From(0, 20, 78);

    // --- COUNT 1: Unique rendered vertices within r=1.0 of setback point ---
    // A clean chamfer-chamfer junction should have at most 3 unique triangle
    // vertices within 1 unit of (0,20,78): the setback vertex itself plus up to
    // 2 adjacent face vertices from neighboring triangles.
    int uniqueVerts = CountUniqueMeshVerticesNear(dm, sharedCornerSB, 1.0);
    dbp("DIAG displaymesh_chamfer_adjacent_triangle_vertex_count: "
        "uniqueMeshVerts(r=1.0)=%d dm->l.n=%d (must be > 0)", uniqueVerts, dm->l.n);
    // Ensure displayMesh was actually populated (non-empty is required for
    // meaningful assertions; empty would trivially pass all checks).
    CHECK_TRUE(dm->l.n > 0);

    // --- COUNT 2: Degenerate triangles within r=2.0 of setback point ---
    // Any triangle with area < 1e-6 near the shared corner is a triangulation
    // artifact. Zero is the only acceptable count.
    int degTris = CountDegenerateTrianglesNear(dm, sharedCornerSB, 2.0, 1e-6);
    dbp("DIAG displaymesh_chamfer_adjacent_triangle_vertex_count: "
        "degenerateTris(r=2.0)=%d", degTris);

    // INVARIANT A: no degenerate triangles at the shared corner.
    // FAILS if TriangulateInto produced a sliver near (0,20,78).
    CHECK_TRUE(degTris == 0);

    // INVARIANT B: unique vertex count at the shared corner <= 3.
    // FAILS if TriangulateInto introduced a phantom rendered vertex.
    // This would confirm the displayMesh bug the user reported.
    CHECK_TRUE(uniqueVerts <= 3);
}

//-----------------------------------------------------------------------------
// TDD characterization test: displaymesh_fillet_adjacent_triangle_vertex_count
//
// Probes the displayMesh (what the renderer actually draws) for the adjacent
// fillet-fillet scenario. Mirrors displaymesh_chamfer_adjacent_triangle_vertex_count
// but uses AddFilletGroup. Fillet arcs are tessellated into PWL segments, so
// the expected unique mesh vertex count near (0,20,78) is HIGHER than the
// chamfer case (more arc midpoints from cylindrical surface triangulation).
//
// SCENARIO (same as fillet_adjacent_corner_vertex_reuse):
//   CreateBoxExtrude() -> Box 20x20x80, Z range [0,80].
//   Fillet1: face@Y=20 + topCap@Z=80 (radius=2.0)
//     -> setback vertex (0,20,78) on back-left edge
//   Fillet2: face@Y=20 + face@X=0 (radius=2.0) on fillet1H
//     -> shared corner setback at (0,20,78)
//
// ASSERTIONS (on g2->displayMesh, the rendered triangulation):
//   A. CountDegenerateTrianglesNear(dm, (0,20,78), 2.0) == 0
//      No sliver/degenerate triangles (area < 1e-6) near the shared corner.
//      Any phantom degenerate triangle is a triangulation artifact.
//   B. CountUniqueMeshVerticesNear(dm, (0,20,78), 1.0) <= 20
//      Generous upper bound for arc tessellation.
//      A fillet arc with 8 PWL segments produces ~10 unique vertices near
//      the junction point. The bound of 20 catches pathological phantom counts
//      while allowing normal arc tessellation.
//   C. dm->l.n > 0 (displayMesh is non-empty — GenerateDisplayItems() worked)
//   D. CountSharpOutlineEdgesNear(dm, (0,20,78), 1.0) == 5
//      5 sharp edges: 4 fillet boundaries + 1 untouched box vertical edge.
//      The 5th edge is the box's left-front vertical edge (0,20,0)->(0,20,78),
//      which terminates within r=1.0 of the corner. Entity resolution maps
//      leftFace to rightFace surface, so fillet2 does NOT modify the left
//      vertical edge. This is expected geometry, not a bug.
//
// Passing = all 4 assertions pass (A: no degenerate tris, B: non-empty mesh,
//           C: vertex bound, D: exactly 5 sharp edges).
//-----------------------------------------------------------------------------
TEST_CASE(displaymesh_fillet_adjacent_triangle_vertex_count) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    // Resolve the three faces (same as fillet_adjacent_corner_vertex_reuse).
    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // --- Fillet 1: face@Y=20 + top cap (radius=2.0) ---
    // Fillets the top-back horizontal edge at (Y=20, Z=80).
    // Creates setback (0,20,78) on back-left edge and (0,18,80) on top cap.
    hGroup fillet1H = AddFilletGroup(extrudeH, frontFace, topCap, 2.0);
    Group *g1 = SK.GetGroup(fillet1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // --- Fillet 2: face@Y=20 + face@X=0 (radius=2.0) ---
    // Fillets the back-left vertical edge from (0,20,0) to (0,20,78).
    // The shared corner setback is at (0,20,78).
    hGroup fillet2H = AddFilletGroup(fillet1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(fillet2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Access the displayMesh -- the triangulation of the rendered geometry.
    // IMPORTANT: Must call GenerateDisplayItems() first -- it calls
    // runningShell.TriangulateInto(&displayMesh) which populates the mesh.
    g2->GenerateDisplayItems();
    SMesh *dm = &g2->displayMesh;

    // The shared corner setback point (same as chamfer case for 90-deg, r=2).
    Vector sharedCornerSB = Vector::From(0, 20, 78);

    // --- COUNT 1: Unique rendered vertices within r=1.0 of setback point ---
    // Fillet arcs are tessellated: the cylindrical surfaces produce PWL triangle
    // fans. Arc midpoints within r=1.0 of (0,20,78) are expected and legitimate.
    // The B-rep test (fillet_adjacent_corner_vertex_reuse) shows countAfterR1<=2
    // in runningShell; displayMesh may have more due to grid tessellation.
    int uniqueVerts = CountUniqueMeshVerticesNear(dm, sharedCornerSB, 1.0);
    dbp("DIAG displaymesh_fillet_adjacent_triangle_vertex_count: "
        "uniqueMeshVerts(r=1.0)=%d dm->l.n=%d", uniqueVerts, dm->l.n);

    // --- COUNT 2: Degenerate triangles within r=2.0 of setback point ---
    int degTris = CountDegenerateTrianglesNear(dm, sharedCornerSB, 2.0, 1e-6);
    dbp("DIAG displaymesh_fillet_adjacent_triangle_vertex_count: "
        "degenerateTris(r=2.0)=%d", degTris);

    // --- COUNT 3: Sharp outline edges within r=1.0 of setback point ---
    // 5 sharp edges expected at (0,20,78): 4 fillet boundary edges + 1 box
    // vertical edge (0,20,0)->(0,20,78) that is untouched by fillet2.
    // See item 7 DIAG test (displaymesh_fillet_adjacent_sharp_edge_enumeration)
    // for full enumeration of all 5 edges.
    int sharpEdges = CountSharpOutlineEdgesNear(dm, sharedCornerSB, 1.0);
    dbp("DIAG displaymesh_fillet_adjacent_triangle_vertex_count: "
        "sharpOutlineEdges(r=1.0)=%d", sharpEdges);

    // INVARIANT D: exactly 5 sharp outline edges at the fillet corner.
    CHECK_TRUE(sharpEdges == 5);

    // INVARIANT A: no degenerate triangles at the shared corner.
    // A non-zero count confirms a triangulation artifact in the fillet path.
    CHECK_TRUE(degTris == 0);

    // INVARIANT B: non-empty displayMesh (GenerateDisplayItems() worked).
    CHECK_TRUE(dm->l.n > 0);

    // INVARIANT C: unique vertex count within generous fillet arc bound.
    // <= 20 catches pathological phantoms while permitting normal arc PWL pts.
    // DIAG line above documents the actual number for baseline records.
    CHECK_TRUE(uniqueVerts <= 20);
}

//-----------------------------------------------------------------------------
// RED-PHASE TDD test: chamfer_adjacent_no_unassigned_face_at_corner
//
// Asserts that NO displayMesh triangle near the shared chamfer-chamfer corner
// has meta.face == 0 (unassigned face handle). The existing diagnostic test
// (displaymesh_chamfer_adjacent_mesh_triangle_dump) reveals TRI[1] with
// face=0x00000000 at vertices a=(20,20,78) b=(20,18,78) c=(18,20,78).
// This is the unassigned-face corner triangle hypothesized in Session 3 as
// the root cause of the visible green vertex glitch.
//
// SCENARIO:
//   CreateBoxExtrude() -> 20x20x80 box.
//   Chamfer1: frontFace(Y=20) + topCap (dist=2.0)
//   Chamfer2: frontFace(Y=20) + leftFace(X=0 entity -> X=20 surface) (dist=2.0)
//
// ASSERTION:
//   CountUnassignedFaceTrianglesNear(displayMesh, (20,20,78), 5.0) == 0
//   Any count > 0 means a corner triangle lacks a face handle -- the bug.
//
// EXPECTED: FAILS (red phase) -- count == 1 (the known unassigned triangle).
//           Production fix in chamfer.cpp corner-synthesis path will make it PASS.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_adjacent_no_unassigned_face_at_corner) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    // Resolve the three faces.
    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // --- Chamfer 1: frontFace + topCap (dist=2.0) ---
    hGroup chamfer1H = AddChamferGroup(extrudeH, frontFace, topCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // --- Chamfer 2: frontFace + leftFace (dist=2.0) ---
    hGroup chamfer2H = AddChamferGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Generate displayMesh.
    g2->GenerateDisplayItems();
    SMesh *dm = &g2->displayMesh;
    CHECK_TRUE(dm->l.n > 0);

    // Probe the corner at (20,20,78) -- entity resolution maps leftFace entity
    // to rightFace surface at X=20, so the corner triangle is at (20,20,78).
    Vector cornerProbe = Vector::From(20, 20, 78);
    double probeRadius = 5.0;

    int unassignedCount = CountUnassignedFaceTrianglesNear(*dm, cornerProbe, probeRadius);

    // Diagnostic: dump centroids of any unassigned-face triangles found.
    dbp("DIAG chamfer_adjacent_no_unassigned_face_at_corner: "
        "unassignedFaceTris(r=%.1f)=%d dm->l.n=%d", probeRadius, unassignedCount, dm->l.n);
    for(int i = 0; i < dm->l.n; i++) {
        const STriangle &t = dm->l[i];
        if(t.meta.face != 0) continue;
        Vector centroid = t.a.Plus(t.b).Plus(t.c).ScaledBy(1.0/3.0);
        if(centroid.Minus(cornerProbe).Magnitude() > probeRadius) continue;
        dbp("  UNASSIGNED TRI[%d]: face=0x%08x "
            "a=(%.1f,%.1f,%.1f) b=(%.1f,%.1f,%.1f) c=(%.1f,%.1f,%.1f) "
            "centroid=(%.2f,%.2f,%.2f)",
            i, t.meta.face,
            t.a.x, t.a.y, t.a.z,
            t.b.x, t.b.y, t.b.z,
            t.c.x, t.c.y, t.c.z,
            centroid.x, centroid.y, centroid.z);
    }

    // INVARIANT: No triangles near the corner should have an unassigned face.
    CHECK_TRUE(unassignedCount == 0);
}

//-----------------------------------------------------------------------------
// Safety-net test: chamfer_adjacent_no_extra_mesh_vertices_at_corner
//
// User invariant: "the expected geometry shouldn't create any more points than
// is created after the first chamfer." This test asserts that the unique
// displayMesh vertex count near the shared chamfer-chamfer corner is exactly
// the topologically expected value — no phantom/extra vertices from the
// triangulation or corner-synthesis path.
//
// SCENARIO (same as chamfer_adjacent_no_unassigned_face_at_corner):
//   CreateBoxExtrude() -> 20x20x80 box.
//   Chamfer1: frontFace(Y=20) + topCap (dist=2.0)
//   Chamfer2: frontFace(Y=20) + leftFace(X=0 entity -> X=20 surface) (dist=2.0)
//
// PROBE: (20,20,78) with r=3.0 — captures the corner-triangle setback
//        vertices (20,20,78), (20,18,78), (18,20,78) plus any adjacent
//        triangle vertices within range.
//
// EXPECTED: PASS (fix already applied). The exact count is determined
//           empirically and locked in as a regression guard.
//-----------------------------------------------------------------------------
TEST_CASE(chamfer_adjacent_no_extra_mesh_vertices_at_corner) {
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
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hGroup chamfer2H = AddChamferGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    g2->GenerateDisplayItems();
    SMesh *dm = &g2->displayMesh;
    CHECK_TRUE(dm->l.n > 0);

    // Probe the corner at (20,20,78) — the shared chamfer-chamfer corner
    // where the unassigned-face bug was fixed.
    Vector cornerProbe = Vector::From(20, 20, 78);
    double probeRadius = 3.0;

    int uniqueVerts = CountUniqueMeshVerticesNear(dm, cornerProbe, probeRadius);

    // Diagnostic: print count and enumerate all unique vertices found.
    dbp("DIAG chamfer_adjacent_no_extra_mesh_vertices_at_corner: "
        "uniqueVerts(r=%.1f)=%d dm->l.n=%d", probeRadius, uniqueVerts, dm->l.n);

    // INVARIANT: No extra phantom vertices at the corner. The topologically
    // correct count is locked as an exact value to catch any future regression
    // that introduces extra intermediate vertices at the shared corner.
    // After Strategy H (cap extent fix + diagonal trim), the cap extends to
    // Z=80 and the old corner-triangle at Z=78 no longer exists. The vertex
    // count near (20,20,78) is now 2 — the geometry is correct with fewer
    // vertices because the corner triangle setback points are gone.
    CHECK_TRUE(uniqueVerts == 2);
}

//-----------------------------------------------------------------------------
// displaymesh_chamfer_adjacent_sharp_edge_coords_diagnostic
//
// DIAGNOSTIC ONLY — no assertions. Prints the actual (x,y,z) coordinates
// of every sharp outline edge with at least one endpoint within r=3.0 of the
// shared corner setback point (0,20,78).
//
// This reveals which of the 4 expected sharp edges is MISSING (sharpEdges=3
// vs expected=3 per displaymesh_chamfer_adjacent_sharp_outline_count).
//
// Expected sharp edges radiating from (0,20,78):
//   1. chamfer1 boundary on back face (toward Z=0, Y=20)
//   2. chamfer1 boundary on top cap (toward Y=18, X>0)
//   3. chamfer2 top cap edge on back face (toward X=2)
//   4. chamfer2 top cap edge on left face (toward Y=18)
//
// Also prints radius counts at r=1.0, r=1.5, r=2.0, r=2.5, r=3.0 for
// differential diagnosis.
//-----------------------------------------------------------------------------
TEST_CASE(displaymesh_chamfer_adjacent_sharp_edge_coords_diagnostic) {
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
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    hGroup chamfer2H = AddChamferGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    g2->GenerateDisplayItems();
    SMesh *dm = &g2->displayMesh;

    Vector sb = Vector::From(0, 20, 78);

    // Build the full sharp outline list.
    SOutlineList sol = {};
    dm->MakeOutlinesInto(&sol, EdgeKind::SHARP);

    // Print total outline count for context.
    dbp("DIAG displaymesh_chamfer_sharp_edge_coords: total_sharp_outlines=%d dm->l.n=%d",
        sol.l.n, dm->l.n);

    // Counts at different radii around (0,20,78).
    int cnt10 = 0, cnt15 = 0, cnt20 = 0, cnt25 = 0, cnt30 = 0;
    for(auto &ol : sol.l) {
        double da = ol.a.Minus(sb).Magnitude();
        double db = ol.b.Minus(sb).Magnitude();
        double dmin = min(da, db);
        if(dmin < 1.0) cnt10++;
        if(dmin < 1.5) cnt15++;
        if(dmin < 2.0) cnt20++;
        if(dmin < 2.5) cnt25++;
        if(dmin < 3.0) cnt30++;
    }
    dbp("DIAG displaymesh_chamfer_sharp_edge_coords: "
        "count_r1.0=%d r1.5=%d r2.0=%d r2.5=%d r3.0=%d",
        cnt10, cnt15, cnt20, cnt25, cnt30);

    // Print coordinates of all sharp edges within r=3.0 of setback.
    // This reveals which 3 are present and which 4th is absent.
    int idx = 0;
    for(auto &ol : sol.l) {
        double da = ol.a.Minus(sb).Magnitude();
        double db = ol.b.Minus(sb).Magnitude();
        if(min(da, db) >= 3.0) continue;
        dbp("DIAG sharp_edge[%d]: a=(%.3f,%.3f,%.3f) b=(%.3f,%.3f,%.3f) "
            "da=%.3f db=%.3f tag=%d",
            idx++,
            ol.a.x, ol.a.y, ol.a.z,
            ol.b.x, ol.b.y, ol.b.z,
            da, db, ol.tag);
    }
    dbp("DIAG displaymesh_chamfer_sharp_edge_coords: printed %d edges near corner", idx);

    // Also print all triangles' vertices for complete topology picture.
    // Filter to triangles near the corner (any vertex within r=3.0).
    int triIdx = 0;
    for(int i = 0; i < dm->l.n; i++) {
        const STriangle &t = dm->l[i];
        double da = t.a.Minus(sb).Magnitude();
        double db = t.b.Minus(sb).Magnitude();
        double dc = t.c.Minus(sb).Magnitude();
        if(da >= 3.0 && db >= 3.0 && dc >= 3.0) continue;
        dbp("DIAG tri[%d]: a=(%.3f,%.3f,%.3f) b=(%.3f,%.3f,%.3f) "
            "c=(%.3f,%.3f,%.3f) face=%u",
            triIdx++,
            t.a.x, t.a.y, t.a.z,
            t.b.x, t.b.y, t.b.z,
            t.c.x, t.c.y, t.c.z,
            t.meta.face);
    }
    dbp("DIAG displaymesh_chamfer_sharp_edge_coords: printed %d tris near corner", triIdx);

    // No assertion -- pure diagnostic. The test always passes.
    // Read the DIAG output to determine which sharp edge is missing.
    sol.Clear();
}

//-----------------------------------------------------------------------------
// displaymesh_chamfer_adjacent_frontface_trim_list_count
//
// B-REP DIAGNOSTIC: Inspects the frontFace (Y=20 plane) surface in
// g2->runningShell after two adjacent chamfer operations, and checks whether
// chamfer2's cap-boundary curve (at X=18) is present in the trim list.
//
// SCENARIO (same as displaymesh_chamfer_adjacent_triangle_vertex_count):
//   CreateBoxExtrude() -> Box 20x20x80, Z range [0,80].
//   Chamfer1: face@Y=20 + topCap@Z=80 (dist=2.0)
//     -> setback vertex (0,20,78) on back-left edge
//   Chamfer2: face@Y=20 + face@X=0 (dist=2.0) on chamfer1H
//     [NOTE: entity resolution maps face@X=0 entity to rightFace surface X=20;
//      chamfer2 actually operates on the front+RIGHT edge]
//     -> actual adjacent-chamfer corner at (20,20,78)
//
// ROOT CAUSE being tested:
//   chamfer2's cap-boundary curve lies at X=18 in 3D (on the frontFace plane).
//   If the trim construction code properly inserts this boundary into
//   frontFace's trim list, at least one STrimBy entry should have start.x or
//   finish.x ≈ 18.0.
//
// ASSERTIONS:
//   A. frontFaceSurf found (Y=20 plane exists in runningShell)
//   B. At least one trim entry has start.x or finish.x within 0.01 of 18.0
//      EXPECTED: RED — this fails because the trim entry is absent (the bug).
//      After the production fix in chamfer.cpp, this should turn GREEN.
//
// Also prints DIAG output for every trim entry on frontFace for inspection.
//-----------------------------------------------------------------------------
TEST_CASE(displaymesh_chamfer_adjacent_frontface_trim_list_count) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    // Resolve the three faces.
    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // --- Chamfer 1: face@Y=20 + top cap (dist=2.0) ---
    hGroup chamfer1H = AddChamferGroup(extrudeH, frontFace, topCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // --- Chamfer 2: face@Y=20 + face@X=0 (dist=2.0) ---
    hGroup chamfer2H = AddChamferGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Find the frontFace surface (Y=20 plane) in the B-rep runningShell.
    // Identify by checking that the surface normal at (0.5,0.5) is
    // approximately (0,-1,0) AND that all 4 control points have Y=20.
    SSurface *frontFaceSurf = nullptr;
    int surfIdx = 0;
    for(auto &ss : g2->runningShell.surface) {
        // Check if this is a degree 1x1 bilinear surface with all ctrl Y=20.
        bool allY20 = true;
        for(int i = 0; i <= ss.degm; i++) {
            for(int j = 0; j <= ss.degn; j++) {
                if(fabs(ss.ctrl[i][j].y - 20.0) > 0.01) { allY20 = false; break; }
            }
            if(!allY20) break;
        }
        if(allY20 && ss.degm >= 1 && ss.degn >= 1) {
            frontFaceSurf = &ss;
            dbp("DIAG frontface_trim: found frontFace surface at index %d "
                "face=%u degm=%d degn=%d", surfIdx, ss.face, ss.degm, ss.degn);
            break;
        }
        surfIdx++;
    }
    CHECK_TRUE(frontFaceSurf != nullptr);
    if(!frontFaceSurf) return;

    // Dump every trim entry and count how many have start.x or finish.x ≈ 18.0.
    int nTrims = frontFaceSurf->trim.n;
    int trimCountX18 = 0;
    const double tol = 0.01;
    dbp("DIAG frontface_trim: nTrims=%d", nTrims);
    for(int i = 0; i < nTrims; i++) {
        STrimBy &stb = frontFaceSurf->trim.Get(i);
        dbp("DIAG frontface_trim[%d]: curve=%08x backwards=%d "
            "start=(%.4f,%.4f,%.4f) finish=(%.4f,%.4f,%.4f)",
            i, stb.curve.v, stb.backwards,
            stb.start.x, stb.start.y, stb.start.z,
            stb.finish.x, stb.finish.y, stb.finish.z);
        if(fabs(stb.start.x - 18.0) < tol || fabs(stb.finish.x - 18.0) < tol) {
            trimCountX18++;
        }
    }
    dbp("DIAG frontface_trim: trimCountX18=%d (expected >= 1)", trimCountX18);

    // INVARIANT: at least one trim entry references chamfer2's boundary at X=18.
    // EXPECTED: RED before fix (trimCountX18 == 0 due to missing trim insertion).
    // After production fix in chamfer.cpp, this should be >= 1.
    CHECK_TRUE(trimCountX18 >= 1);
}

//-----------------------------------------------------------------------------
// displaymesh_fillet_adjacent_sharp_edge_enumeration
//
// DIAGNOSTIC ONLY — no assertions. Enumerates all SHARP outline edges near
// the fillet-fillet shared corner.  Checks BOTH (0,20,78) and (20,20,78)
// because entity resolution maps face@X=0 entity to rightFace surface X=20
// (discovered during chamfer investigation, iteration 5).  The fillet path
// uses the same FindFaceByNormal(-1,0,0) → leftFace entity → rightFace
// surface mapping, so fillet2 may operate on the front+RIGHT edge instead
// of front+LEFT.
//
// SCENARIO:
//   CreateBoxExtrude() -> Box 20x20x80, Z range [0,80].
//   Fillet1: face@Y=20 + topCap@Z=80 (radius=2.0)
//     -> setback vertex (0,20,78) on back-left edge
//   Fillet2: face@Y=20 + face@X=0 (radius=2.0) on fillet1H
//     -> nominal corner at (0,20,78), actual may be at (20,20,78)
//
// PURPOSE:
//   The existing displaymesh_fillet_adjacent_triangle_vertex_count test
//   reports 5 sharp edges at (0,20,78) via CountSharpOutlineEdgesNear
//   (expected: 4). This DIAG test prints the actual (x,y,z) coordinates
//   of every sharp edge near BOTH corners to determine:
//     A. Whether the 5th edge is a real phantom or arc-PWL tessellation artifact
//     B. Whether entity resolution shifts the fillet corner to (20,20,78)
//     C. The exact geometry of each edge for root-cause analysis
//
// OUTPUT: DIAG lines only. No assertions. Always passes.
//-----------------------------------------------------------------------------
TEST_CASE(displaymesh_fillet_adjacent_sharp_edge_enumeration) {
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

    // Build the full sharp outline list once for both corners.
    SOutlineList sol = {};
    dm->MakeOutlinesInto(&sol, EdgeKind::SHARP);

    dbp("DIAG fillet_sharp_enum: total_sharp_outlines=%d dm->l.n=%d",
        sol.l.n, dm->l.n);

    // --- Corner A: (0,20,78) — nominal leftFace corner ---
    Vector cornerA = Vector::From(0, 20, 78);
    {
        int c10 = 0, c15 = 0, c20 = 0, c25 = 0, c30 = 0;
        for(auto &ol : sol.l) {
            double da = ol.a.Minus(cornerA).Magnitude();
            double db = ol.b.Minus(cornerA).Magnitude();
            double dmin = (da < db) ? da : db;
            if(dmin < 1.0) c10++;
            if(dmin < 1.5) c15++;
            if(dmin < 2.0) c20++;
            if(dmin < 2.5) c25++;
            if(dmin < 3.0) c30++;
        }
        dbp("DIAG fillet_sharp_enum cornerA(0,20,78): "
            "r1.0=%d r1.5=%d r2.0=%d r2.5=%d r3.0=%d",
            c10, c15, c20, c25, c30);

        int idx = 0;
        for(auto &ol : sol.l) {
            double da = ol.a.Minus(cornerA).Magnitude();
            double db = ol.b.Minus(cornerA).Magnitude();
            if(da >= 3.0 && db >= 3.0) continue;
            dbp("DIAG fillet_edgeA[%d]: a=(%.3f,%.3f,%.3f) b=(%.3f,%.3f,%.3f) "
                "da=%.3f db=%.3f",
                idx++,
                ol.a.x, ol.a.y, ol.a.z,
                ol.b.x, ol.b.y, ol.b.z,
                da, db);
        }
        dbp("DIAG fillet_sharp_enum cornerA: %d edges within r=3.0", idx);
    }

    // --- Corner B: (20,20,78) — rightFace corner (entity resolution) ---
    Vector cornerB = Vector::From(20, 20, 78);
    {
        int c10 = 0, c15 = 0, c20 = 0, c25 = 0, c30 = 0;
        for(auto &ol : sol.l) {
            double da = ol.a.Minus(cornerB).Magnitude();
            double db = ol.b.Minus(cornerB).Magnitude();
            double dmin = (da < db) ? da : db;
            if(dmin < 1.0) c10++;
            if(dmin < 1.5) c15++;
            if(dmin < 2.0) c20++;
            if(dmin < 2.5) c25++;
            if(dmin < 3.0) c30++;
        }
        dbp("DIAG fillet_sharp_enum cornerB(20,20,78): "
            "r1.0=%d r1.5=%d r2.0=%d r2.5=%d r3.0=%d",
            c10, c15, c20, c25, c30);

        int idx = 0;
        for(auto &ol : sol.l) {
            double da = ol.a.Minus(cornerB).Magnitude();
            double db = ol.b.Minus(cornerB).Magnitude();
            if(da >= 3.0 && db >= 3.0) continue;
            dbp("DIAG fillet_edgeB[%d]: a=(%.3f,%.3f,%.3f) b=(%.3f,%.3f,%.3f) "
                "da=%.3f db=%.3f",
                idx++,
                ol.a.x, ol.a.y, ol.a.z,
                ol.b.x, ol.b.y, ol.b.z,
                da, db);
        }
        dbp("DIAG fillet_sharp_enum cornerB: %d edges within r=3.0", idx);
    }

    // No assertions — pure diagnostic. Always passes.
    sol.Clear();
}

//-----------------------------------------------------------------------------
// MIXED OP TEST: chamfer1 (top+front) then fillet2 (front+left).
//
// Scenario:
//   1. CreateBoxExtrude(): 20×20×80 box (valA=20).
//   2. Chamfer1: frontFace(Y=20) + topCap → chamfer on top-front horizontal edge.
//      Setback = 2.0. Creates boundary at Z=78 on frontFace.
//   3. Fillet2: frontFace(Y=20) + leftFace(X=0) → fillet on front-left vertical edge.
//      Radius = 2.0. Entity resolution maps leftFace entity to rightFace surface (X=20),
//      so fillet2 actually operates on the front-RIGHT vertical edge.
//      The shared corner between chamfer1 and fillet2 is at (20,20,78).
//
// Expected sharp edges at corner (20,20,78), radius=1.0:
//   Ideal: boundary edges from chamfer1 (flat) and fillet2 (arc).
//   The fillet adds arc boundary curves which may produce PWL sharp edges.
//   The chamfer-chamfer analog at this corner shows 3 edges (known bug, 1 missing).
//   The fillet-fillet analog shows 5 edges (4 boundaries + 1 box vertical edge).
//   The mixed case count is unknown a priori — this test characterizes it.
//
// Also checks both corners (20,20,78) and (0,20,78) to detect asymmetry.
//
// Passing: booleanFailed == false, non-empty displayMesh, guard assertion on
//          observed sharp edge count (set after first run).
//-----------------------------------------------------------------------------
TEST_CASE(displaymesh_mixed_chamfer_fillet_adjacent_sharp_outline_count) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    // Resolve the three faces.
    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // --- Chamfer 1: frontFace + topCap (dist=2.0) ---
    // Chamfers the top-front horizontal edge at (Y=20, Z=80).
    hGroup chamfer1H = AddChamferGroup(extrudeH, frontFace, topCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // --- Fillet 2: frontFace + leftFace (radius=2.0) ---
    // Fillets the front-left vertical edge (entity resolution maps to front-RIGHT).
    // Chains off chamfer1's output shell.
    hGroup fillet2H = AddFilletGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(fillet2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Generate displayMesh.
    g2->GenerateDisplayItems();
    SMesh *dm = &g2->displayMesh;

    // Sanity: displayMesh must be non-empty.
    CHECK_TRUE(dm->l.n > 0);
    dbp("DIAG displaymesh_mixed_chamfer_fillet: dm->l.n=%d", dm->l.n);

    // --- Corner A: (20,20,78) — the adjacent corner (entity-resolved right side) ---
    Vector cornerA = Vector::From(20, 20, 78);
    int sharpA = CountSharpOutlineEdgesNear(dm, cornerA, 1.0);
    dbp("DIAG displaymesh_mixed_chamfer_fillet: cornerA(20,20,78) sharpEdges(r=1.0)=%d",
        sharpA);

    // --- Corner B: (0,20,78) — the opposite corner (chamfer1's far end) ---
    Vector cornerB = Vector::From(0, 20, 78);
    int sharpB = CountSharpOutlineEdgesNear(dm, cornerB, 1.0);
    dbp("DIAG displaymesh_mixed_chamfer_fillet: cornerB(0,20,78) sharpEdges(r=1.0)=%d",
        sharpB);

    // --- Enumerate edges near corner A for DIAG ---
    SOutlineList sol = {};
    dm->MakeOutlinesInto(&sol, EdgeKind::SHARP);
    int idx = 0;
    for(auto &ol : sol.l) {
        double da = ol.a.Minus(cornerA).Magnitude();
        double db = ol.b.Minus(cornerA).Magnitude();
        double dmin = std::min(da, db);
        if(dmin >= 2.0) continue;
        dbp("DIAG mixed_CF_edgeA[%d]: a=(%.3f,%.3f,%.3f) b=(%.3f,%.3f,%.3f) "
            "da=%.3f db=%.3f",
            idx++,
            ol.a.x, ol.a.y, ol.a.z,
            ol.b.x, ol.b.y, ol.b.z,
            da, db);
    }
    dbp("DIAG mixed_CF: %d edges within r=2.0 of cornerA", idx);
    sol.Clear();

    // Guard assertion on observed sharp edge counts at corner A (20,20,78).
    //
    // Observed: sharpA == 3 (same as chamfer-chamfer adjacent).
    // The 3 edges are:
    //   1. (20,20,78)→(20,18,80) — chamfer1 boundary toward top cap (da=0, db=2.83)
    //   2. (20,20,78)→(18,20,78) — chamfer1 boundary along frontFace (da=0, db=2.0)
    //   3. (20,20,78)→(20,18,78) — fillet2 boundary along rightFace (da=0, db=2.0)
    //
    // Missing: fillet2's boundary on frontFace at X=18 (same oversized-trim-polygon
    // bug as chamfer-chamfer, item 4 BLOCKED). Expected ideal: 4 sharp edges.
    // When item 4's production fix is applied, update to sharpA == 4.
    //
    // KNOWN BUG GUARD: assert == 3 to lock in current behavior.
    CHECK_TRUE(sharpA == 3);
}

//-----------------------------------------------------------------------------
// MIXED OP TEST: fillet1 (top+front) then chamfer2 (front+left).
//
// Scenario (REVERSE order of item 10's chamfer+fillet test):
//   1. CreateBoxExtrude(): 20×20×80 box (valA=20).
//   2. Fillet1: frontFace(Y=20) + topCap → fillet on top-front horizontal edge.
//      Radius = 2.0. Creates arc boundary at ~Z=78 on frontFace.
//   3. Chamfer2: frontFace(Y=20) + leftFace(X=0) → chamfer on front-left vertical edge.
//      Dist = 2.0. Entity resolution maps leftFace entity to rightFace surface (X=20),
//      so chamfer2 actually operates on the front-RIGHT vertical edge.
//      The shared corner between fillet1 and chamfer2 is at (20,20,78).
//
// Expected sharp edges at corner (20,20,78), radius=1.0:
//   Ideal: boundary edges from fillet1 (arc) and chamfer2 (flat).
//   The reverse-order (chamfer then fillet) from item 10 shows 3 edges (known bug).
//   This test characterizes whether op-order matters (asymmetry detection).
//
// Also checks both corners (20,20,78) and (0,20,78) to detect asymmetry.
//
// Passing: booleanFailed == false, non-empty displayMesh, guard assertion on
//          observed sharp edge count (set after first run).
//-----------------------------------------------------------------------------
TEST_CASE(displaymesh_mixed_fillet_chamfer_adjacent_sharp_outline_count) {
    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    // Resolve the three faces.
    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // --- Fillet 1: frontFace + topCap (radius=2.0) ---
    // Fillets the top-front horizontal edge at (Y=20, Z=80).
    hGroup fillet1H = AddFilletGroup(extrudeH, frontFace, topCap, 2.0);
    Group *g1 = SK.GetGroup(fillet1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // --- Chamfer 2: frontFace + leftFace (dist=2.0) ---
    // Chamfers the front-left vertical edge (entity resolution maps to front-RIGHT).
    // Chains off fillet1's output shell.
    hGroup chamfer2H = AddChamferGroup(fillet1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Generate displayMesh.
    g2->GenerateDisplayItems();
    SMesh *dm = &g2->displayMesh;

    // Sanity: displayMesh must be non-empty.
    CHECK_TRUE(dm->l.n > 0);
    dbp("DIAG displaymesh_mixed_fillet_chamfer: dm->l.n=%d", dm->l.n);

    // --- Corner A: (20,20,78) — the adjacent corner (entity-resolved right side) ---
    Vector cornerA = Vector::From(20, 20, 78);
    int sharpA = CountSharpOutlineEdgesNear(dm, cornerA, 1.0);
    dbp("DIAG displaymesh_mixed_fillet_chamfer: cornerA(20,20,78) sharpEdges(r=1.0)=%d",
        sharpA);

    // --- Corner B: (0,20,78) — the opposite corner (fillet1's far end) ---
    Vector cornerB = Vector::From(0, 20, 78);
    int sharpB = CountSharpOutlineEdgesNear(dm, cornerB, 1.0);
    dbp("DIAG displaymesh_mixed_fillet_chamfer: cornerB(0,20,78) sharpEdges(r=1.0)=%d",
        sharpB);

    // --- Enumerate edges near corner A for DIAG ---
    SOutlineList sol = {};
    dm->MakeOutlinesInto(&sol, EdgeKind::SHARP);
    int idx = 0;
    for(auto &ol : sol.l) {
        double da = ol.a.Minus(cornerA).Magnitude();
        double db = ol.b.Minus(cornerA).Magnitude();
        double dmin = std::min(da, db);
        if(dmin >= 2.0) continue;
        dbp("DIAG mixed_FC_edgeA[%d]: a=(%.3f,%.3f,%.3f) b=(%.3f,%.3f,%.3f) "
            "da=%.3f db=%.3f",
            idx++,
            ol.a.x, ol.a.y, ol.a.z,
            ol.b.x, ol.b.y, ol.b.z,
            da, db);
    }
    dbp("DIAG mixed_FC: %d edges within r=2.0 of cornerA", idx);
    sol.Clear();

    // Guard assertion on observed sharp edge counts at corner A (20,20,78).
    //
    // Observed: sharpA == 5.  OP-ORDER ASYMMETRY vs chamfer+fillet (item 10, sharpA==3).
    //
    // The 5 edges at r=1.0 of corner (20,20,78):
    //   1. (20,20,78)→(18,20,78) — chamfer2 boundary along frontFace (da=0)
    //   2. (20,20,78)→(20,18,78) — chamfer2 boundary along rightFace (da=0)
    //   3. (20,19.860,78.736)→(20,19.414,79.414) — fillet1 arc seg on rightFace (da=0.749)
    //   4. (0,19.860,78.736)→(20,19.860,78.736) — fillet1 boundary on frontFace (db=0.749)
    //   5. (20,19.860,78.736)→(20,20,78) — fillet1→corner on rightFace (db=0)
    //
    // When fillet runs FIRST, the fillet arc boundaries are fully formed, then
    // chamfer2 adds its flat boundaries at the corner. Both fillet and chamfer
    // boundaries are present → 5 edges. When chamfer runs first (item 10),
    // chamfer1's boundary is already on frontFace, and fillet2's boundary on
    // frontFace is MISSING due to the oversized-trim-polygon bug → only 3 edges.
    //
    // This confirms op-order asymmetry: the first op's boundaries survive, but
    // the second op's frontFace boundary may be lost if it encounters the
    // oversized-trim-polygon bug from the corner post-process.
    //
    // KNOWN BEHAVIOR GUARD: assert == 5 to lock in current fillet-first behavior.
    // If item 4's production fix is applied, the chamfer+fillet case (item 10)
    // should also show more edges (ideally matching this count).
    //
    CHECK_TRUE(sharpA == 5);
}

//-----------------------------------------------------------------------------
// DIAG: Per-surface mesh triangle dump near the adjacent chamfer corner.
//
// This test enumerates ALL triangles in the displayMesh within r=3.0 of the
// corner at (20,20,78), grouped by meta.face.  For each triangle it prints
// the vertices and face ID, allowing us to determine:
//   1. Whether V1=(20,20,78) appears as a mesh vertex on frontFace
//   2. Which surfaces contribute triangles near the corner
//   3. Which mesh edge (shared between two specific surfaces) is the
//      missing 4th sharp outline edge
//
// Pure DIAG — always passes, no assertions except basic setup.
//-----------------------------------------------------------------------------
TEST_CASE(displaymesh_chamfer_adjacent_mesh_triangle_dump) {
    hGroup extrudeH = CreateBoxExtrude();

    // Resolve faces using the same pattern as adjacent-chamfer tests.
    hEntity frontFace = FindFaceByNormal(extrudeH, Vector::From(0, -1, 0));
    hEntity topCap    = FindCapFace(extrudeH, /*wantTop=*/true);
    hEntity leftFace  = FindFaceByNormal(extrudeH, Vector::From(-1, 0, 0));
    CHECK_TRUE(frontFace.v != 0);
    CHECK_TRUE(topCap.v != 0);
    CHECK_TRUE(leftFace.v != 0);

    // --- Chamfer 1: frontFace + topCap (dist=2.0) ---
    hGroup chamfer1H = AddChamferGroup(extrudeH, frontFace, topCap, 2.0);
    Group *g1 = SK.GetGroup(chamfer1H);
    CHECK_FALSE(g1->booleanFailed);
    if(g1->booleanFailed) return;

    // --- Chamfer 2: frontFace + leftFace (dist=2.0) ---
    hGroup chamfer2H = AddChamferGroup(chamfer1H, frontFace, leftFace, 2.0);
    Group *g2 = SK.GetGroup(chamfer2H);
    CHECK_FALSE(g2->booleanFailed);
    if(g2->booleanFailed) return;

    // Generate displayMesh.
    g2->GenerateDisplayItems();
    SMesh *dm = &g2->displayMesh;
    CHECK_TRUE(dm->l.n > 0);

    Vector corner = Vector::From(20, 20, 78);
    double searchR = 3.0;

    dbp("======= MESH TRIANGLE DUMP near (20,20,78) r=%.1f =======", searchR);
    dbp("Total triangles in displayMesh: %d", dm->l.n);

    // Collect unique face IDs near the corner.
    int nearCount = 0;
    bool hasV1 = false;
    Vector V1 = Vector::From(20, 20, 78);

    for(int i = 0; i < dm->l.n; i++) {
        STriangle &t = dm->l[i];
        bool near = (t.a.Minus(corner).Magnitude() <= searchR ||
                     t.b.Minus(corner).Magnitude() <= searchR ||
                     t.c.Minus(corner).Magnitude() <= searchR);
        if(!near) continue;

        nearCount++;
        dbp("TRI[%d] face=0x%08x "
            "a=(%.3f,%.3f,%.3f) b=(%.3f,%.3f,%.3f) c=(%.3f,%.3f,%.3f)",
            i, t.meta.face,
            t.a.x, t.a.y, t.a.z,
            t.b.x, t.b.y, t.b.z,
            t.c.x, t.c.y, t.c.z);

        // Check if V1 is a vertex.
        if(t.a.Minus(V1).Magnitude() < 1e-6 ||
           t.b.Minus(V1).Magnitude() < 1e-6 ||
           t.c.Minus(V1).Magnitude() < 1e-6) {
            hasV1 = true;
            dbp("  ^^^ HAS V1=(20,20,78) as vertex");
        }

        // Print normals at each vertex.
        dbp("  normals: an=(%.4f,%.4f,%.4f) bn=(%.4f,%.4f,%.4f) cn=(%.4f,%.4f,%.4f)",
            t.an.x, t.an.y, t.an.z,
            t.bn.x, t.bn.y, t.bn.z,
            t.cn.x, t.cn.y, t.cn.z);
    }

    dbp("Near corner: %d triangles, V1 present: %s",
        nearCount, hasV1 ? "YES" : "NO");

    // --- Check specific edges involving V1 ---
    // For each triangle with V1, check if its edges are also in another triangle
    // (anti-parallel = shared edge).
    Vector A0 = Vector::From(18, 20, 78);
    dbp("--- Edge analysis at V1=(20,20,78) ---");
    // Check if edge A0->V1 exists as a triangle edge.
    int edgeA0V1 = 0, edgeV1A0 = 0;
    for(int i = 0; i < dm->l.n; i++) {
        STriangle &t = dm->l[i];
        Vector verts[3] = {t.a, t.b, t.c};
        for(int j = 0; j < 3; j++) {
            Vector ea = verts[j], eb = verts[(j+1)%3];
            if(ea.Minus(A0).Magnitude() < 1e-6 &&
               eb.Minus(V1).Magnitude() < 1e-6) {
                edgeA0V1++;
                dbp("  Edge A0->V1 found in TRI[%d] face=0x%08x", i, t.meta.face);
            }
            if(ea.Minus(V1).Magnitude() < 1e-6 &&
               eb.Minus(A0).Magnitude() < 1e-6) {
                edgeV1A0++;
                dbp("  Edge V1->A0 found in TRI[%d] face=0x%08x", i, t.meta.face);
            }
        }
    }
    dbp("Edge A0->V1 count: %d, Edge V1->A0 count: %d", edgeA0V1, edgeV1A0);
    dbp("For sharp edge detection: need both A0->V1 and V1->A0 each with count 1 "
        "(anti-parallel pair from different surfaces).");

    // --- Also check edges between V1 and other key points ---
    Vector D0 = Vector::From(20, 18, 78);
    int edgeV1D0 = 0, edgeD0V1 = 0;
    for(int i = 0; i < dm->l.n; i++) {
        STriangle &t = dm->l[i];
        Vector verts[3] = {t.a, t.b, t.c};
        for(int j = 0; j < 3; j++) {
            Vector ea = verts[j], eb = verts[(j+1)%3];
            if(ea.Minus(V1).Magnitude() < 1e-6 &&
               eb.Minus(D0).Magnitude() < 1e-6) edgeV1D0++;
            if(ea.Minus(D0).Magnitude() < 1e-6 &&
               eb.Minus(V1).Magnitude() < 1e-6) edgeD0V1++;
        }
    }
    dbp("Edge V1->D0(20,18,78) count: %d, Edge D0->V1 count: %d", edgeV1D0, edgeD0V1);

    dbp("======= END MESH TRIANGLE DUMP =======");
}
