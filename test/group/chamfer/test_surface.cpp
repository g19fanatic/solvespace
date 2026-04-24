//-----------------------------------------------------------------------------
// Surface and triangulation tests
//-----------------------------------------------------------------------------
#include "helpers.h"

//-----------------------------------------------------------------------------
// Characterization test: MakeEdgesInto gracefully skips orphaned trim entries
// (STrimBy referencing a curve handle not present in the shell).
//-----------------------------------------------------------------------------
TEST_CASE(surface_MakeEdgesInto_orphaned_trim) {
    // Create a plane surface: origin at (0,0,0), u=(100,0,0), v=(0,100,0)
    SSurface srf = SSurface::FromPlane(
        Vector::From(0, 0, 0),
        Vector::From(100, 0, 0),
        Vector::From(0, 100, 0));

    // Create an SShell with only 2 valid curves (bottom + right edges)
    SShell shell = {};

    // --- Curve 1: Bottom edge (0,0,0) -> (50,0,0) -> (100,0,0) ---
    {
        SCurve sc = {};
        sc.h.v = 1;
        SCurvePt p0 = {}; p0.p = Vector::From(0, 0, 0);   p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(50, 0, 0);   p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(100, 0, 0);  p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }

    // --- Curve 2: Right edge (100,0,0) -> (100,50,0) -> (100,100,0) ---
    {
        SCurve sc = {};
        sc.h.v = 2;
        SCurvePt p0 = {}; p0.p = Vector::From(100, 0, 0);   p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(100, 50, 0);  p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(100, 100, 0); p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }

    // --- Add STrimBy entries: 2 valid + 1 orphaned ---
    // Valid: Bottom edge, forward from (0,0,0) to (100,0,0)
    { STrimBy stb = {}; stb.curve.v = 1; stb.backwards = false;
      stb.start = Vector::From(0, 0, 0); stb.finish = Vector::From(100, 0, 0);
      srf.trim.Add(&stb); }
    // Orphaned: references curve handle 999 which does NOT exist in shell
    { STrimBy stb = {}; stb.curve.v = 999; stb.backwards = false;
      stb.start = Vector::From(100, 0, 0); stb.finish = Vector::From(100, 100, 0);
      srf.trim.Add(&stb); }
    // Valid: Right edge, forward from (100,0,0) to (100,100,0)
    { STrimBy stb = {}; stb.curve.v = 2; stb.backwards = false;
      stb.start = Vector::From(100, 0, 0); stb.finish = Vector::From(100, 100, 0);
      srf.trim.Add(&stb); }

    // --- Call MakeEdgesInto ---
    SEdgeList sel = {};
    srf.MakeEdgesInto(&shell, &sel, SSurface::MakeAs::XYZ);

    // Only the 2 valid curves should produce edges (2 edges each = 4 total)
    // The orphaned trim (curve.v=999) should be silently skipped
    CHECK_TRUE(sel.l.n == 4);

    // Cleanup
    sel.Clear();
    srf.trim.Clear();
    srf.Clear();
    shell.curve.Clear();
}

//-----------------------------------------------------------------------------
// Characterization test: SSurface::TriangulateInto on a curved surface (degm=2)
// Creates a degree-2 surface via FromExtrusionOf with a quadratic SBezier,
// adds a trim loop using on-surface boundary points, calls TriangulateInto,
// and verifies UvGridTriangulateInto produces valid triangles with curvature.
//-----------------------------------------------------------------------------
TEST_CASE(surface_TriangulateInto_curved) {
    // Create a quadratic (degree-2) SBezier: parabolic arch in XZ plane
    // ctrl[0]=(0,0,0), ctrl[1]=(50,0,25), ctrl[2]=(100,0,0)
    SBezier sb = {};
    sb.deg = 2;
    sb.ctrl[0] = Vector::From(0, 0, 0);
    sb.ctrl[1] = Vector::From(50, 0, 25);
    sb.ctrl[2] = Vector::From(100, 0, 0);
    sb.weight[0] = 1.0;
    sb.weight[1] = 1.0;
    sb.weight[2] = 1.0;

    // Extrude along Y axis: t0=(0,0,0), t1=(0,100,0)
    // Result: degm=2, degn=1 surface
    // ctrl[0][0]=(0,0,0),   ctrl[0][1]=(0,100,0)
    // ctrl[1][0]=(50,0,25), ctrl[1][1]=(50,100,25)
    // ctrl[2][0]=(100,0,0), ctrl[2][1]=(100,100,0)
    SSurface srf = SSurface::FromExtrusionOf(&sb, Vector::From(0, 0, 0),
                                                   Vector::From(0, 100, 0));
    srf.face = 99;
    srf.color = RgbaColor::From(200, 100, 50, 255);

    // Verify surface degree triggers UvGridTriangulateInto path
    CHECK_TRUE(srf.degm == 2);
    CHECK_TRUE(srf.degn == 1);

    // Create SShell with 4 boundary curves using on-surface points.
    // Surface corners:
    //   A = PointAt(0,0) = (0,0,0)
    //   B = PointAt(1,0) = (100,0,0)
    //   C = PointAt(1,1) = (100,100,0)
    //   D = PointAt(0,1) = (0,100,0)
    // Bottom edge at v=0 is the parabolic arch. Sample on-surface point:
    //   PointAt(0.5,0) = 0.25*(0,0,0)+0.5*(50,0,25)+0.25*(100,0,0) = (50,0,12.5)
    // Top edge at v=1: same arch shifted by Y:
    //   PointAt(0.5,1) = (50,100,12.5)
    // Left/right edges are linear (degn=1):
    //   PointAt(0,0.5) = (0,50,0),  PointAt(1,0.5) = (100,50,0)
    SShell shell = {};

    // --- Curve 1: Bottom edge v=0 (parabolic arch) ---
    {
        SCurve sc = {};
        sc.h.v = 1;
        SCurvePt p0 = {}; p0.p = Vector::From(0, 0, 0);       p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(50, 0, 12.5);   p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(100, 0, 0);     p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }
    // --- Curve 2: Right edge u=1 (straight line) ---
    {
        SCurve sc = {};
        sc.h.v = 2;
        SCurvePt p0 = {}; p0.p = Vector::From(100, 0, 0);     p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(100, 50, 0);    p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(100, 100, 0);   p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }
    // --- Curve 3: Top edge v=1 (parabolic arch at y=100) ---
    {
        SCurve sc = {};
        sc.h.v = 3;
        SCurvePt p0 = {}; p0.p = Vector::From(100, 100, 0);     p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(50, 100, 12.5);   p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(0, 100, 0);       p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }
    // --- Curve 4: Left edge u=0 (straight line) ---
    {
        SCurve sc = {};
        sc.h.v = 4;
        SCurvePt p0 = {}; p0.p = Vector::From(0, 100, 0);   p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(0, 50, 0);    p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(0, 0, 0);     p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }

    // --- Add STrimBy entries forming a closed loop ---
    { STrimBy stb = {}; stb.curve.v = 1; stb.backwards = false;
      stb.start = Vector::From(0, 0, 0); stb.finish = Vector::From(100, 0, 0);
      srf.trim.Add(&stb); }
    { STrimBy stb = {}; stb.curve.v = 2; stb.backwards = false;
      stb.start = Vector::From(100, 0, 0); stb.finish = Vector::From(100, 100, 0);
      srf.trim.Add(&stb); }
    { STrimBy stb = {}; stb.curve.v = 3; stb.backwards = false;
      stb.start = Vector::From(100, 100, 0); stb.finish = Vector::From(0, 100, 0);
      srf.trim.Add(&stb); }
    { STrimBy stb = {}; stb.curve.v = 4; stb.backwards = false;
      stb.start = Vector::From(0, 100, 0); stb.finish = Vector::From(0, 0, 0);
      srf.trim.Add(&stb); }

    // --- Call TriangulateInto ---
    SMesh sm = {};
    srf.TriangulateInto(&shell, &sm);

    // UvGridTriangulateInto should produce triangles for the curved surface.
    CHECK_TRUE(sm.l.n >= 2);

    // Verify all triangles are valid
    double totalArea = 0.0;
    bool hasElevatedVertex = false;
    for(int i = 0; i < sm.l.n; i++) {
        STriangle *st = &sm.l[i];

        // Check for non-degenerate triangle (positive area)
        Vector ab = st->b.Minus(st->a);
        Vector ac = st->c.Minus(st->a);
        double triArea = 0.5 * ab.Cross(ac).Magnitude();
        CHECK_TRUE(triArea > 1e-6);
        totalArea += triArea;

        // Check that vertices are within expected bounds (with tolerance)
        for(int v = 0; v < 3; v++) {
            CHECK_TRUE(st->vertices[v].x >= -1.0 && st->vertices[v].x <= 101.0);
            CHECK_TRUE(st->vertices[v].y >= -1.0 && st->vertices[v].y <= 101.0);
            // z should be in [0, ~12.5] for the parabolic arch
            CHECK_TRUE(st->vertices[v].z >= -1.0 && st->vertices[v].z <= 14.0);
            if(st->vertices[v].z > 1.0) hasElevatedVertex = true;
        }

        // Normals should be non-zero
        CHECK_TRUE(st->an.Magnitude() > 0.01);

        // Metadata should match what we set
        CHECK_TRUE(st->meta.face == 99);
    }

    // At least one vertex should have z > 0, confirming the curved surface
    // (the arch reaches z ~= 12.5 at the midpoint)
    CHECK_TRUE(hasElevatedVertex);

    // Total area should be substantial (covering 100x100 region with curvature)
    CHECK_TRUE(totalArea > 5000.0);

    // Cleanup
    sm.Clear();
    srf.trim.Clear();
    srf.Clear();
    shell.curve.Clear();
}

//-----------------------------------------------------------------------------
// Characterization test: SSurface::TriangulateInto on a plane surface
// Creates a plane surface with a complete rectangular trim loop (4 curves),
// calls TriangulateInto, and verifies it produces valid triangles covering
// the rectangle with correct 3D coordinates, normals, and metadata.
//-----------------------------------------------------------------------------
TEST_CASE(surface_TriangulateInto_plane) {
    // Create a plane surface: origin at (0,0,0), u=(100,0,0), v=(0,100,0)
    // PointAt(u,v) = (100*v, 100*u, 0)
    // 3D rectangle [0,100] x [0,100] in XY plane
    SSurface srf = SSurface::FromPlane(
        Vector::From(0, 0, 0),
        Vector::From(100, 0, 0),
        Vector::From(0, 100, 0));
    srf.face = 42;
    srf.color = RgbaColor::From(128, 64, 32, 255);

    // Create an SShell to hold the curves
    SShell shell = {};

    // --- Curve 1: Bottom edge (0,0,0) -> (50,0,0) -> (100,0,0) ---
    {
        SCurve sc = {};
        sc.h.v = 1;
        SCurvePt p0 = {}; p0.p = Vector::From(0, 0, 0);   p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(50, 0, 0);   p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(100, 0, 0);  p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }
    // --- Curve 2: Right edge (100,0,0) -> (100,50,0) -> (100,100,0) ---
    {
        SCurve sc = {};
        sc.h.v = 2;
        SCurvePt p0 = {}; p0.p = Vector::From(100, 0, 0);   p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(100, 50, 0);  p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(100, 100, 0); p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }
    // --- Curve 3: Top edge (100,100,0) -> (50,100,0) -> (0,100,0) ---
    {
        SCurve sc = {};
        sc.h.v = 3;
        SCurvePt p0 = {}; p0.p = Vector::From(100, 100, 0); p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(50, 100, 0);  p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(0, 100, 0);   p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }
    // --- Curve 4: Left edge (0,100,0) -> (0,50,0) -> (0,0,0) ---
    {
        SCurve sc = {};
        sc.h.v = 4;
        SCurvePt p0 = {}; p0.p = Vector::From(0, 100, 0); p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(0, 50, 0);  p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(0, 0, 0);   p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }

    // --- Add STrimBy entries ---
    { STrimBy stb = {}; stb.curve.v = 1; stb.backwards = false;
      stb.start = Vector::From(0, 0, 0); stb.finish = Vector::From(100, 0, 0);
      srf.trim.Add(&stb); }
    { STrimBy stb = {}; stb.curve.v = 2; stb.backwards = false;
      stb.start = Vector::From(100, 0, 0); stb.finish = Vector::From(100, 100, 0);
      srf.trim.Add(&stb); }
    { STrimBy stb = {}; stb.curve.v = 3; stb.backwards = false;
      stb.start = Vector::From(100, 100, 0); stb.finish = Vector::From(0, 100, 0);
      srf.trim.Add(&stb); }
    { STrimBy stb = {}; stb.curve.v = 4; stb.backwards = false;
      stb.start = Vector::From(0, 100, 0); stb.finish = Vector::From(0, 0, 0);
      srf.trim.Add(&stb); }

    // --- Call TriangulateInto ---
    SMesh sm = {};
    srf.TriangulateInto(&shell, &sm);

    // A rectangle should produce at least 2 triangles
    CHECK_TRUE(sm.l.n >= 2);

    // Verify all triangles: vertices on XY plane, within bounds, valid normals, correct meta
    double totalArea = 0.0;
    for(int i = 0; i < sm.l.n; i++) {
        STriangle *st = &sm.l[i];

        // All vertices should have z == 0 (on XY plane)
        CHECK_TRUE(fabs(st->a.z) < 0.01);
        CHECK_TRUE(fabs(st->b.z) < 0.01);
        CHECK_TRUE(fabs(st->c.z) < 0.01);

        // All vertices within [0, 100] x [0, 100] bounds (with tolerance)
        for(int v = 0; v < 3; v++) {
            CHECK_TRUE(st->vertices[v].x >= -0.01 && st->vertices[v].x <= 100.01);
            CHECK_TRUE(st->vertices[v].y >= -0.01 && st->vertices[v].y <= 100.01);
        }

        // Normals should be non-zero (plane normal)
        CHECK_TRUE(st->an.Magnitude() > 0.1);

        // Metadata should match what we set
        CHECK_TRUE(st->meta.face == 42);

        // Accumulate triangle area: 0.5 * |cross(b-a, c-a)|
        Vector ab = st->b.Minus(st->a);
        Vector ac = st->c.Minus(st->a);
        totalArea += 0.5 * ab.Cross(ac).Magnitude();
    }

    // Total area should equal the rectangle area (100 * 100 = 10000)
    CHECK_TRUE(fabs(totalArea - 10000.0) < 1.0);

    // Cleanup
    sm.Clear();
    srf.trim.Clear();
    srf.Clear();
    shell.curve.Clear();
}

//-----------------------------------------------------------------------------
// Characterization test: SSurface::MakeEdgesInto with UV mode on a plane surface
// Creates a plane surface with a complete rectangular trim loop (4 curves),
// calls MakeEdgesInto with MakeAs::UV, and verifies the edges form a closed
// polygon in UV space.
//-----------------------------------------------------------------------------
TEST_CASE(surface_MakeEdgesInto_plane_UV) {
    // Create a plane surface: origin at (0,0,0), u=(100,0,0), v=(0,100,0)
    // For this surface: PointAt(u,v) = (100*v, 100*u, 0)
    // So ClosestPointTo maps:
    //   (0,0,0)     -> UV(0, 0)
    //   (100,0,0)   -> UV(0, 1)
    //   (100,100,0) -> UV(1, 1)
    //   (0,100,0)   -> UV(1, 0)
    SSurface srf = SSurface::FromPlane(
        Vector::From(0, 0, 0),
        Vector::From(100, 0, 0),
        Vector::From(0, 100, 0));

    // Create an SShell to hold the curves
    SShell shell = {};

    // --- Curve 1: Bottom edge (0,0,0) -> (50,0,0) -> (100,0,0) ---
    {
        SCurve sc = {};
        sc.h.v = 1;
        SCurvePt p0 = {}; p0.p = Vector::From(0, 0, 0);   p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(50, 0, 0);   p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(100, 0, 0);  p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }

    // --- Curve 2: Right edge (100,0,0) -> (100,50,0) -> (100,100,0) ---
    {
        SCurve sc = {};
        sc.h.v = 2;
        SCurvePt p0 = {}; p0.p = Vector::From(100, 0, 0);   p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(100, 50, 0);  p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(100, 100, 0); p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }

    // --- Curve 3: Top edge (100,100,0) -> (50,100,0) -> (0,100,0) ---
    {
        SCurve sc = {};
        sc.h.v = 3;
        SCurvePt p0 = {}; p0.p = Vector::From(100, 100, 0); p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(50, 100, 0);  p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(0, 100, 0);   p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }

    // --- Curve 4: Left edge (0,100,0) -> (0,50,0) -> (0,0,0) ---
    {
        SCurve sc = {};
        sc.h.v = 4;
        SCurvePt p0 = {}; p0.p = Vector::From(0, 100, 0); p0.vertex = true;
        SCurvePt p1 = {}; p1.p = Vector::From(0, 50, 0);  p1.vertex = false;
        SCurvePt p2 = {}; p2.p = Vector::From(0, 0, 0);   p2.vertex = true;
        sc.pts.Add(&p0); sc.pts.Add(&p1); sc.pts.Add(&p2);
        shell.curve.Add(&sc);
    }

    // --- Add STrimBy entries to the surface's trim list ---
    // Bottom: forward from (0,0,0) to (100,0,0)
    { STrimBy stb = {}; stb.curve.v = 1; stb.backwards = false;
      stb.start = Vector::From(0, 0, 0); stb.finish = Vector::From(100, 0, 0);
      srf.trim.Add(&stb); }
    // Right: forward from (100,0,0) to (100,100,0)
    { STrimBy stb = {}; stb.curve.v = 2; stb.backwards = false;
      stb.start = Vector::From(100, 0, 0); stb.finish = Vector::From(100, 100, 0);
      srf.trim.Add(&stb); }
    // Top: forward from (100,100,0) to (0,100,0)
    { STrimBy stb = {}; stb.curve.v = 3; stb.backwards = false;
      stb.start = Vector::From(100, 100, 0); stb.finish = Vector::From(0, 100, 0);
      srf.trim.Add(&stb); }
    // Left: forward from (0,100,0) to (0,0,0)
    { STrimBy stb = {}; stb.curve.v = 4; stb.backwards = false;
      stb.start = Vector::From(0, 100, 0); stb.finish = Vector::From(0, 0, 0);
      srf.trim.Add(&stb); }

    // --- Call MakeEdgesInto with UV mode ---
    SEdgeList sel = {};
    srf.MakeEdgesInto(&shell, &sel, SSurface::MakeAs::UV);

    // Each curve has 3 points -> 2 edges per curve -> 8 edges total
    CHECK_TRUE(sel.l.n == 8);

    // Verify UV coordinates of key edges:
    // Bottom edge (curve 1): UV(0,0,0)->(0,0.5,0), UV(0,0.5,0)->(0,1,0)
    CHECK_TRUE(sel.l[0].a.Equals(Vector::From(0, 0, 0)));
    CHECK_TRUE(sel.l[0].b.Equals(Vector::From(0, 0.5, 0)));
    CHECK_TRUE(sel.l[1].a.Equals(Vector::From(0, 0.5, 0)));
    CHECK_TRUE(sel.l[1].b.Equals(Vector::From(0, 1, 0)));

    // Right edge (curve 2): UV(0,1,0)->(0.5,1,0), UV(0.5,1,0)->(1,1,0)
    CHECK_TRUE(sel.l[2].a.Equals(Vector::From(0, 1, 0)));
    CHECK_TRUE(sel.l[2].b.Equals(Vector::From(0.5, 1, 0)));
    CHECK_TRUE(sel.l[3].a.Equals(Vector::From(0.5, 1, 0)));
    CHECK_TRUE(sel.l[3].b.Equals(Vector::From(1, 1, 0)));

    // Top edge (curve 3): UV(1,1,0)->(1,0.5,0), UV(1,0.5,0)->(1,0,0)
    CHECK_TRUE(sel.l[4].a.Equals(Vector::From(1, 1, 0)));
    CHECK_TRUE(sel.l[4].b.Equals(Vector::From(1, 0.5, 0)));
    CHECK_TRUE(sel.l[5].a.Equals(Vector::From(1, 0.5, 0)));
    CHECK_TRUE(sel.l[5].b.Equals(Vector::From(1, 0, 0)));

    // Left edge (curve 4): UV(1,0,0)->(0.5,0,0), UV(0.5,0,0)->(0,0,0)
    CHECK_TRUE(sel.l[6].a.Equals(Vector::From(1, 0, 0)));
    CHECK_TRUE(sel.l[6].b.Equals(Vector::From(0.5, 0, 0)));
    CHECK_TRUE(sel.l[7].a.Equals(Vector::From(0.5, 0, 0)));
    CHECK_TRUE(sel.l[7].b.Equals(Vector::From(0, 0, 0)));

    // Verify the polygon is closed: assemble into SPolygon
    SPolygon poly = {};
    SEdge notClosedAt = {};
    bool assembled = sel.AssemblePolygon(&poly, &notClosedAt, /*keepDir=*/true);
    CHECK_TRUE(assembled);
    // Should have exactly 1 contour (the rectangular loop)
    CHECK_TRUE(poly.l.n == 1);

    // Cleanup
    poly.Clear();
    sel.Clear();
    srf.trim.Clear();
    srf.Clear();
    shell.curve.Clear();
}

//-----------------------------------------------------------------------------
// Characterization test: SSurface::MakeTrimEdgesInto with empty trim (start==finish)
//-----------------------------------------------------------------------------
TEST_CASE(surface_MakeTrimEdgesInto_empty_trim) {
    // Create a simple plane surface: origin at (0,0,0), u=(100,0,0), v=(0,100,0)
    SSurface srf = SSurface::FromPlane(
        Vector::From(0, 0, 0),
        Vector::From(100, 0, 0),
        Vector::From(0, 100, 0));

    // Create an SCurve with 5 points along the bottom edge: x=0..100, y=0, z=0
    SCurve sc = {};
    sc.h.v = 0x200;  // arbitrary handle
    for(int i = 0; i <= 4; i++) {
        SCurvePt scp = {};
        scp.p = Vector::From(i * 25.0, 0, 0);
        scp.vertex = (i == 0 || i == 4);
        sc.pts.Add(&scp);
    }

    // --- Case A: Forward, start == finish at interior point on curve ---
    // When start == finish, the function sets inCurve=true then immediately false
    // on the same point, so no edges are emitted.
    {
        STrimBy stb = {};
        stb.curve.v = sc.h.v;
        stb.backwards = false;
        stb.start  = Vector::From(50, 0, 0);  // point [2]
        stb.finish = Vector::From(50, 0, 0);  // same point

        SEdgeList sel = {};
        srf.MakeTrimEdgesInto(&sel, SSurface::MakeAs::XYZ, &sc, &stb);

        // Empty trim: no edges produced
        CHECK_TRUE(sel.l.n == 0);
        sel.Clear();
    }

    // --- Case B: Backward, start == finish at interior point on curve ---
    {
        STrimBy stb = {};
        stb.curve.v = sc.h.v;
        stb.backwards = true;
        stb.start  = Vector::From(50, 0, 0);
        stb.finish = Vector::From(50, 0, 0);

        SEdgeList sel = {};
        srf.MakeTrimEdgesInto(&sel, SSurface::MakeAs::XYZ, &sc, &stb);

        CHECK_TRUE(sel.l.n == 0);
        sel.Clear();
    }

    // --- Case C: Forward, start == finish at first point (0,0,0) ---
    {
        STrimBy stb = {};
        stb.curve.v = sc.h.v;
        stb.backwards = false;
        stb.start  = Vector::From(0, 0, 0);
        stb.finish = Vector::From(0, 0, 0);

        SEdgeList sel = {};
        srf.MakeTrimEdgesInto(&sel, SSurface::MakeAs::XYZ, &sc, &stb);

        CHECK_TRUE(sel.l.n == 0);
        sel.Clear();
    }

    // --- Case D: Forward, start == finish at a point NOT on the curve ---
    // Neither start nor finish match any curve point, so inCurve never activates.
    {
        STrimBy stb = {};
        stb.curve.v = sc.h.v;
        stb.backwards = false;
        stb.start  = Vector::From(999, 999, 0);
        stb.finish = Vector::From(999, 999, 0);

        SEdgeList sel = {};
        srf.MakeTrimEdgesInto(&sel, SSurface::MakeAs::XYZ, &sc, &stb);

        CHECK_TRUE(sel.l.n == 0);
        sel.Clear();
    }

    // Cleanup
    sc.pts.Clear();
    srf.Clear();
}
