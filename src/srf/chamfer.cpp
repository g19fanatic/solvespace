//-----------------------------------------------------------------------------
// Chamfer and fillet operations on NURBS shells. These work by direct
// topology injection: the source shell is copied, then surgically modified
// to add the chamfer or fillet surface in place of the shared edge between
// two selected faces.
//
// Copyright 2024 SolveSpace Contributors.
//-----------------------------------------------------------------------------
#include "solvespace.h"

namespace SolveSpace {

//-----------------------------------------------------------------------------
// Helper: insert point P into a curve's pts list at the segment that
// contains P. P must lie on one of the piecewise-linear segments.
// Returns true if the point was inserted (or already present).
//-----------------------------------------------------------------------------
static bool InsertPointIntoCurvePts(SCurve *sc, Vector P) {
    // Check if P is already in the list
    for(int i = 0; i < sc->pts.n; i++) {
        if(sc->pts[i].p.Equals(P)) return true;
    }

    // Find the segment that contains P
    for(int i = 0; i < sc->pts.n - 1; i++) {
        Vector a = sc->pts[i].p;
        Vector b = sc->pts[i+1].p;
        Vector ab = b.Minus(a);
        double len = ab.Magnitude();
        if(len < LENGTH_EPS) continue;

        Vector ap = P.Minus(a);
        double t = ap.Dot(ab) / (len * len);
        if(t < LENGTH_EPS/len || t > 1.0 - LENGTH_EPS/len) continue;

        // Check that P lies on the line (not just on the parametric extension)
        Vector closest = a.Plus(ab.ScaledBy(t));
        if(!closest.Equals(P)) continue;

        // Insert P after position i: rebuild pts
        List<SCurvePt> newPts = {};
        for(int j = 0; j <= i; j++) {
            newPts.Add(&sc->pts[j]);
        }
        SCurvePt newpt = {};
        newpt.p = P;
        newpt.vertex = true;
        newpt.tag = 0;
        newPts.Add(&newpt);
        for(int j = i+1; j < sc->pts.n; j++) {
            newPts.Add(&sc->pts[j]);
        }
        sc->pts.Clear();
        for(int j = 0; j < newPts.n; j++) {
            sc->pts.Add(&newPts[j]);
        }
        newPts.Clear();
        return true;
    }
    return false;
}

//-----------------------------------------------------------------------------
// Helper: add a new straight SCurve between two points, assigning surfA and
// surfB, and populating pts via MakePwlInto.
//-----------------------------------------------------------------------------
static hSCurve AddLinearCurve(SShell *shell, Vector from, Vector to,
                               hSSurface surfA, hSSurface surfB)
{
    SCurve sc = {};
    sc.isExact = true;
    sc.exact = SBezier::From(from, to);
    sc.exact.MakePwlInto(&sc.pts);
    sc.surfA = surfA;
    sc.surfB = surfB;
    return shell->curve.AddAndAssignId(&sc);
}

//-----------------------------------------------------------------------------
// SShell::MakeFromChamferOf
//
// Creates a new shell that is a copy of `src` with a chamfer applied at the
// edge shared between the two faces referenced by g->predef.entityB and
// g->predef.entityC.
//
// Algorithm (for flat faces only, MVP):
//   1. Copy source shell (preserving all IDs)
//   2. Find surf1, surf2 by face handle
//   3. Validate both are flat (DepartureFromCoplanar)
//   4. Find shared SCurve between surf1 and surf2
//   5. Get edge endpoints V1, V2
//   6. Compute face normals n1, n2 at edge midpoint
//   7. Compute inward offset directions d1, d2
//   8. Compute chamfer corners A(V1-face1), B(V2-face1), C(V2-face2), D(V1-face2)
//   9. Create chamfer SSurface
//  10. Create 4 SCurves (A-B, D-C, A-D cap, B-C cap)
//  11. Build chamfer trim polygon
//  12. Update surf1 trim: replace sharedSC -> hCurve1, update endpoint pts
//  13. Update surf2 trim: replace sharedSC -> hCurve2, update endpoint pts
//  14. Remove old shared SCurve
//-----------------------------------------------------------------------------
void SShell::MakeFromChamferOf(SShell *src, Group *g, double dist) {
    booleanFailed = false;

    // Step 1: copy source shell (preserves all surface/curve IDs)
    MakeFromCopyOf(src);

    // Step 2: find surf1 and surf2 by face handle
    hEntity entityB = g->predef.entityB;
    hEntity entityC = g->predef.entityC;

    hSSurface hSurf1 = { 0 }, hSurf2 = { 0 };
    for(SSurface &ss : surface) {
        if(ss.face == entityB.v) hSurf1 = ss.h;
        if(ss.face == entityC.v) hSurf2 = ss.h;
    }

    if(hSurf1.v == 0 || hSurf2.v == 0) {
        booleanFailed = true;
        return;
    }

    // Step 3: validate flat surfaces
    {
        SSurface *s1 = surface.FindById(hSurf1);
        SSurface *s2 = surface.FindById(hSurf2);
        double depart1 = s1->DepartureFromCoplanar();
        double depart2 = s2->DepartureFromCoplanar();
        if(depart1 > LENGTH_EPS ||
           depart2 > LENGTH_EPS) {
            booleanFailed = true;
            return;
        }
    }

    // Step 4: find shared SCurve between surf1 and surf2
    hSCurve hSharedSC = { 0 };
    for(SCurve &sc : curve) {
        if((sc.surfA == hSurf1 && sc.surfB == hSurf2) ||
           (sc.surfA == hSurf2 && sc.surfB == hSurf1)) {
            hSharedSC = sc.h;
            break;
        }
    }

    if(hSharedSC.v == 0) {
        booleanFailed = true;
        return;
    }

    // Step 5: get edge endpoints V1, V2
    SCurve *sharedSC = curve.FindById(hSharedSC);
    if(sharedSC->pts.n < 2) {
        booleanFailed = true;
        return;
    }
    Vector V1 = sharedSC->pts[0].p;
    Vector V2 = sharedSC->pts[sharedSC->pts.n - 1].p;
    Vector edgeVec = V2.Minus(V1);
    double edgeLen = edgeVec.Magnitude();

    if(edgeLen < LENGTH_EPS) {
        booleanFailed = true;
        return;
    }
    if(dist < LENGTH_EPS || dist > edgeLen / 2.0) {
        booleanFailed = true;
        return;
    }

    Vector t = edgeVec.WithMagnitude(1);  // unit tangent along edge

    // Step 6: compute face normals at edge midpoint
    Vector edgeMid = V1.Plus(V2).ScaledBy(0.5);
    SSurface *surf1 = surface.FindById(hSurf1);
    SSurface *surf2 = surface.FindById(hSurf2);

    Point2d uv1, uv2;
    surf1->ClosestPointTo(edgeMid, &uv1);
    surf2->ClosestPointTo(edgeMid, &uv2);
    Vector n1 = surf1->NormalAt(uv1);
    Vector n2 = surf2->NormalAt(uv2);
    if(n1.Magnitude() < LENGTH_EPS || n2.Magnitude() < LENGTH_EPS) {
        booleanFailed = true;
        return;
    }
    n1 = n1.WithMagnitude(1);
    n2 = n2.WithMagnitude(1);

    // Step 7: compute inward offset directions
    // d1 is the direction in surf1's plane, perpendicular to the edge,
    // pointing away from the edge toward the interior of surf1.
    // Convention: d = n.Cross(t) then flip if pointing wrong way.
    Vector d1 = n1.Cross(t).WithMagnitude(1);
    Vector d2 = n2.Cross(t).WithMagnitude(1);

    // Use centroid of the surface ctrl points to determine "inward" direction
    {
        Vector c1 = surf1->ctrl[0][0].Plus(surf1->ctrl[0][1])
                         .Plus(surf1->ctrl[1][0]).Plus(surf1->ctrl[1][1])
                         .ScaledBy(0.25);
        if(d1.Dot(c1.Minus(V1)) < 0) d1 = d1.ScaledBy(-1);

        Vector c2 = surf2->ctrl[0][0].Plus(surf2->ctrl[0][1])
                         .Plus(surf2->ctrl[1][0]).Plus(surf2->ctrl[1][1])
                         .ScaledBy(0.25);
        if(d2.Dot(c2.Minus(V1)) < 0) d2 = d2.ScaledBy(-1);
    }

    // Step 8: compute chamfer corners
    // A, B: setback points on surf1 (at V1 and V2 ends)
    // D, C: setback points on surf2 (at V1 and V2 ends)
    Vector A = V1.Plus(d1.ScaledBy(dist));
    Vector B = V2.Plus(d1.ScaledBy(dist));
    Vector D = V1.Plus(d2.ScaledBy(dist));
    Vector C = V2.Plus(d2.ScaledBy(dist));

    // Step 9: create chamfer surface
    // FromPlane(origin, u, v):
    //   ctrl[0][0] = origin     = A  (u=0,v=0)
    //   ctrl[0][1] = origin+u   = B  (u=1,v=0)
    //   ctrl[1][0] = origin+v   = D  (u=0,v=1)
    //   ctrl[1][1] = origin+u+v = C  (u=1,v=1)
    SSurface chamferSurf = SSurface::FromPlane(A, B.Minus(A), D.Minus(A));
    chamferSurf.color = surf1->color;
    hEntity faceH = g->Remap(g->predef.entityB, Group::REMAP_CHAMFER_FACE);
    chamferSurf.face = faceH.v;

    hSSurface hChamfer = surface.AddAndAssignId(&chamferSurf);

    // NOTE: AddAndAssignId may reallocate the surface list.
    // We must re-lookup pointers after any Add operation.
    // Save handles first, re-lookup as needed.

    // Step 10: create SCurves for chamfer edges
    // Re-lookup surf1, surf2 after potential reallocation
    surf1 = surface.FindById(hSurf1);
    surf2 = surface.FindById(hSurf2);
    sharedSC = curve.FindById(hSharedSC);

    // hCurve1: A->B (between chamfer surface and surf1)
    // Convention: surfA=hChamfer uses backwards=true, surfB=hSurf1 uses backwards=false
    hSCurve hCurve1 = AddLinearCurve(this, A, B, hChamfer, hSurf1);

    // hCurve2: D->C (between chamfer surface and surf2)
    hSCurve hCurve2 = AddLinearCurve(this, D, C, hChamfer, hSurf2);

    // Find cap surfaces at V1 and V2 endpoints
    hSSurface hCapSurfV1 = hChamfer; // fallback
    hSSurface hCapSurfV2 = hChamfer; // fallback
    for(SCurve &sc_scan : curve) {
        if(sc_scan.h == hSharedSC) continue;
        if(sc_scan.pts.n < 2) continue;
        Vector first = sc_scan.pts[0].p;
        Vector last = sc_scan.pts[sc_scan.pts.n - 1].p;
        bool touchesV1 = first.Equals(V1) || last.Equals(V1);
        bool touchesV2 = first.Equals(V2) || last.Equals(V2);
        if(touchesV1 && hCapSurfV1 == hChamfer) {
            if(sc_scan.surfA != hSurf1 && sc_scan.surfA != hSurf2 && sc_scan.surfA.v != 0)
                hCapSurfV1 = sc_scan.surfA;
            else if(sc_scan.surfB != hSurf1 && sc_scan.surfB != hSurf2 && sc_scan.surfB.v != 0)
                hCapSurfV1 = sc_scan.surfB;
        }
        if(touchesV2 && hCapSurfV2 == hChamfer) {
            if(sc_scan.surfA != hSurf1 && sc_scan.surfA != hSurf2 && sc_scan.surfA.v != 0)
                hCapSurfV2 = sc_scan.surfA;
            else if(sc_scan.surfB != hSurf1 && sc_scan.surfB != hSurf2 && sc_scan.surfB.v != 0)
                hCapSurfV2 = sc_scan.surfB;
        }
    }
    // hCapV1: A->D (cap at V1 end, exposed edge)
    hSCurve hCapV1 = AddLinearCurve(this, A, D, hChamfer, hCapSurfV1);

    // hCapV2: B->C (cap at V2 end, exposed edge)
    hSCurve hCapV2 = AddLinearCurve(this, B, C, hChamfer, hCapSurfV2);

    // Re-lookup after more curve additions
    surf1 = surface.FindById(hSurf1);
    surf2 = surface.FindById(hSurf2);
    sharedSC = curve.FindById(hSharedSC);
    SSurface *chamferSurfPtr = surface.FindById(hChamfer);

    // Step 11: build trim polygon for chamfer surface
    // The chamfer surface traversal in uv: A(0,0)->B(1,0)->C(1,1)->D(0,1)->A
    // TriangulateInto calls FlipNormal, so CW uv traversal gives outward normal (visible).
    // CW order in uv: A(0,0)->D(0,1)->C(1,1)->B(1,0)->A
    //   Matches fillet convention: contact1(fwd), arcV2(fwd), contact2(bkw), arcV1(bkw)
    //   => A->D: hCapV1 forward  (pts[0]=A, pts[n-1]=D) → UV (0,0)->(0,1)
    //   => D->C: hCurve2 forward (pts[0]=D, pts[n-1]=C) → UV (0,1)->(1,1)
    //   => C->B: hCapV2 backwards (pts[0]=B, pts[n-1]=C, reversed=C->B) → UV (1,1)->(1,0)
    //   => B->A: hCurve1 backwards (pts[0]=A, pts[n-1]=B, reversed=B->A) → UV (1,0)->(0,0)
    {
        STrimBy stb;
        // A->D: traverse hCapV1 forward (pts[0]=A, pts[n-1]=D) → UV (0,0)->(0,1)
        stb = STrimBy::EntireCurve(this, hCapV1, /*backwards=*/false);
        chamferSurfPtr->trim.Add(&stb);

        // D->C: traverse hCurve2 forward (pts[0]=D, pts[n-1]=C) → UV (0,1)->(1,1)
        stb = STrimBy::EntireCurve(this, hCurve2, /*backwards=*/false);
        chamferSurfPtr->trim.Add(&stb);

        // C->B: traverse hCapV2 backwards (pts[0]=B, pts[n-1]=C, backwards gives C->B) → UV (1,1)->(1,0)
        stb = STrimBy::EntireCurve(this, hCapV2, /*backwards=*/true);
        chamferSurfPtr->trim.Add(&stb);

        // B->A: traverse hCurve1 backwards (pts[0]=A, pts[n-1]=B, backwards gives B->A) → UV (1,0)->(0,0)
        stb = STrimBy::EntireCurve(this, hCurve1, /*backwards=*/true);
        chamferSurfPtr->trim.Add(&stb);
    }

    // Step 12: update surf1 trim
    // Find the STrimBy in surf1 that references sharedSC.
    // Replace it with a reference to hCurve1.
    // Also update endpoints of neighboring trim curves from V1->A and V2->B.
    {
        // Determine the direction of the shared curve as seen from surf1
        // by looking at start point of the existing STrimBy entry.
        bool v1AtStart = false;
        bool foundShared = false;
        for(STrimBy &stb : surf1->trim) {
            if(stb.curve == hSharedSC) {
                v1AtStart = stb.start.Equals(V1);
                foundShared = true;

                // Replace with hCurve1 (A->B forward, D is V1 side, B is V2 side)
                // hCurve1: surfA=hChamfer, surfB=hSurf1
                // Convention: surfB uses backwards=true
                // If old traversal was V1->V2 (forward), new is A->B (forward for surf1 as surfB)
                // surfB convention: backwards=true means traverse pts backwards.
                // Hmm, let me check: surfA=hChamfer, surfB=hSurf1.
                // The extrusion code shows: surfB gets backwards=true, surfA gets backwards=false.
                // But that's for the side surfaces. Let me use what makes the trim close.
                //
                // For surf1's trim polygon to close properly:
                // The neighbor ending at V1 should now end at A.
                // The trim entry we're replacing should match: if old was V1->V2, new is A->B.
                // For surf1 (surfB of hCurve1), convention is backwards=true.
                // backwards=true means: start=pts[n-1]=B, finish=pts[0]=A => traverse B->A.
                // That would give the edge B->A, which going backwards from the old V1->V2.
                // Actually the old V2->V1 (backwards) direction for surf1 when it's surfB...
                //
                // Let me just match the direction: if old was V1->V2 on surf1 (v1AtStart=true),
                // new should be A->B. hCurve1 forward is A->B, so backwards=false.
                // If old was V2->V1 on surf1 (v1AtStart=false), new should be B->A,
                // so backwards=true for hCurve1.
                //
                // This matches: backwards = !v1AtStart
                stb.curve = hCurve1;
                stb.backwards = !v1AtStart;
                if(v1AtStart) {
                    stb.start = A;
                    stb.finish = B;
                } else {
                    stb.start = B;
                    stb.finish = A;
                }
                break;
            }
        }
        if(!foundShared) {
            booleanFailed = true;
            return;
        }

        // Update neighboring trim endpoints: V1->A, V2->B in surf1
        for(STrimBy &stb_n : surf1->trim) {
            if(stb_n.curve == hCurve1) continue;  // skip the one we just replaced

            SCurve *nc = curve.FindByIdNoOops(stb_n.curve);

            if(stb_n.start.Equals(V1)) {
                InsertPointIntoCurvePts(nc, A);
                stb_n.start = A;
            } else if(stb_n.start.Equals(V2)) {
                InsertPointIntoCurvePts(nc, B);
                stb_n.start = B;
            }

            if(stb_n.finish.Equals(V1)) {
                InsertPointIntoCurvePts(nc, A);
                stb_n.finish = A;
            } else if(stb_n.finish.Equals(V2)) {
                InsertPointIntoCurvePts(nc, B);
                stb_n.finish = B;
            }
        }
    }

    // Re-lookup surf2 after potential modifications (none here, but for safety)
    surf2 = surface.FindById(hSurf2);

    // Step 13: update surf2 trim
    // Replace sharedSC reference with hCurve2.
    // Update endpoints: V1->D, V2->C in surf2.
    {
        bool v1AtStart2 = false;
        bool foundShared2 = false;
        for(STrimBy &stb : surf2->trim) {
            if(stb.curve == hSharedSC) {
                v1AtStart2 = stb.start.Equals(V1);
                foundShared2 = true;

                // hCurve2: surfA=hChamfer, surfB=hSurf2
                // Forward is D->C.
                // If old was V1->V2 (v1AtStart2=true), new should be D->C (forward): backwards=false
                // If old was V2->V1 (v1AtStart2=false), new should be C->D (backwards): backwards=true
                stb.curve = hCurve2;
                stb.backwards = !v1AtStart2;
                if(v1AtStart2) {
                    stb.start = D;
                    stb.finish = C;
                } else {
                    stb.start = C;
                    stb.finish = D;
                }
                break;
            }
        }
        if(!foundShared2) {
            booleanFailed = true;
            return;
        }

        // Update neighboring trim endpoints: V1->D, V2->C in surf2
        for(STrimBy &stb_n : surf2->trim) {
            if(stb_n.curve == hCurve2) continue;

            SCurve *nc = curve.FindByIdNoOops(stb_n.curve);

            if(stb_n.start.Equals(V1)) {
                InsertPointIntoCurvePts(nc, D);
                stb_n.start = D;
            } else if(stb_n.start.Equals(V2)) {
                InsertPointIntoCurvePts(nc, C);
                stb_n.start = C;
            }

            if(stb_n.finish.Equals(V1)) {
                InsertPointIntoCurvePts(nc, D);
                stb_n.finish = D;
            } else if(stb_n.finish.Equals(V2)) {
                InsertPointIntoCurvePts(nc, C);
                stb_n.finish = C;
            }
        }
    }

    // Step 15: update cap surface trims at V1 and V2
    // The cap surfaces (hCapSurfV1, hCapSurfV2) also have trims referencing the same
    // neighbor curves that were modified in Steps 12/13. We must update their STrimBy
    // start/finish endpoints and add the cap curve as a new trim entry.
    if(hCapSurfV1 != hChamfer) {
        SSurface *capSurf1 = surface.FindById(hCapSurfV1);
        for(STrimBy &stb_c : capSurf1->trim) {
            SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
            if(!nc) continue;
            bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
            bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
            if(stb_c.start.Equals(V1)) {
                if(bordersSurf1) { InsertPointIntoCurvePts(nc, A); stb_c.start = A; }
                else if(bordersSurf2) { InsertPointIntoCurvePts(nc, D); stb_c.start = D; }
            }
            if(stb_c.finish.Equals(V1)) {
                if(bordersSurf1) { InsertPointIntoCurvePts(nc, A); stb_c.finish = A; }
                else if(bordersSurf2) { InsertPointIntoCurvePts(nc, D); stb_c.finish = D; }
            }
        }
        // Detect gap direction: which endpoint (A or D) is the 'finish' of an existing trim?
        // That tells us where the trim loop arrives, so the cap curve must depart from there.
        bool capV1Backwards = false; // default: forward A->D
        for(const STrimBy &stb_check : capSurf1->trim) {
            if(stb_check.finish.Equals(D)) { capV1Backwards = true; break; }  // gap goes D->A
            if(stb_check.finish.Equals(A)) { capV1Backwards = false; break; } // gap goes A->D
        }
        STrimBy stbCap1 = STrimBy::EntireCurve(this, hCapV1, capV1Backwards);
        capSurf1->trim.Add(&stbCap1);
    }
    if(hCapSurfV2 != hChamfer) {
        SSurface *capSurf2 = surface.FindById(hCapSurfV2);
        for(STrimBy &stb_c : capSurf2->trim) {
            SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
            if(!nc) continue;
            bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
            bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
            if(stb_c.start.Equals(V2)) {
                if(bordersSurf1) { InsertPointIntoCurvePts(nc, B); stb_c.start = B; }
                else if(bordersSurf2) { InsertPointIntoCurvePts(nc, C); stb_c.start = C; }
            }
            if(stb_c.finish.Equals(V2)) {
                if(bordersSurf1) { InsertPointIntoCurvePts(nc, B); stb_c.finish = B; }
                else if(bordersSurf2) { InsertPointIntoCurvePts(nc, C); stb_c.finish = C; }
            }
        }
        // Detect gap direction for V2 end: which endpoint (B or C) is the 'finish' of an existing trim?
        bool capV2Backwards = false; // default: forward B->C
        for(const STrimBy &stb_check : capSurf2->trim) {
            if(stb_check.finish.Equals(C)) { capV2Backwards = true; break; }  // gap goes C->B
            if(stb_check.finish.Equals(B)) { capV2Backwards = false; break; } // gap goes B->C
        }
        STrimBy stbCap2 = STrimBy::EntireCurve(this, hCapV2, capV2Backwards);
        capSurf2->trim.Add(&stbCap2);
    }

    // Step 14: remove old shared SCurve
    // Safety: verify no surface trim still references hSharedSC before removing.
    for(SSurface &ss : surface) {
        for(STrimBy *stb = ss.trim.First(); stb; stb = ss.trim.NextAfter(stb)) {
            if(stb->curve == hSharedSC) {
                booleanFailed = true;
                return;
            }
        }
    }

    curve.RemoveById(hSharedSC);
}

//-----------------------------------------------------------------------------
// SShell::MakeFromFilletOf
//
// Creates a new shell with a cylindrical fillet applied at the edge shared
// between the two faces referenced by g->predef.entityB and entityC.
//
// The fillet surface is a rational quadratic Bezier arc extruded along the
// edge direction, creating an exact quarter-cylinder (for 90-degree edges).
//
// For flat faces only (MVP). setback = r / tan(half_angle),
// arc_weight = sin(half_angle) for exact circle.
//-----------------------------------------------------------------------------
void SShell::MakeFromFilletOf(SShell *src, Group *g, double r) {
    booleanFailed = false;

    // Step 1: copy source shell
    MakeFromCopyOf(src);

    // Step 2: find surf1 and surf2 by face handle
    hEntity entityB = g->predef.entityB;
    hEntity entityC = g->predef.entityC;

    hSSurface hSurf1 = { 0 }, hSurf2 = { 0 };
    for(SSurface &ss : surface) {
        if(ss.face == entityB.v) hSurf1 = ss.h;
        if(ss.face == entityC.v) hSurf2 = ss.h;
    }

    if(hSurf1.v == 0 || hSurf2.v == 0) {
        booleanFailed = true;
        return;
    }


    // Step 3: validate flat surfaces
    {
        SSurface *s1 = surface.FindById(hSurf1);
        SSurface *s2 = surface.FindById(hSurf2);
        if(s1->DepartureFromCoplanar() > LENGTH_EPS ||
           s2->DepartureFromCoplanar() > LENGTH_EPS) {
            booleanFailed = true;
            return;
        }
    }

    // Step 4: find shared SCurve
    hSCurve hSharedSC = { 0 };
    for(SCurve &sc : curve) {
        if((sc.surfA == hSurf1 && sc.surfB == hSurf2) ||
           (sc.surfA == hSurf2 && sc.surfB == hSurf1)) {
            hSharedSC = sc.h;
            break;
        }
    }

    if(hSharedSC.v == 0) {
        booleanFailed = true;
        return;
    }

    // Step 5: get edge endpoints
    SCurve *sharedSC = curve.FindById(hSharedSC);
    if(sharedSC->pts.n < 2) {
        booleanFailed = true;
        return;
    }
    Vector V1 = sharedSC->pts[0].p;
    Vector V2 = sharedSC->pts[sharedSC->pts.n - 1].p;
    Vector edgeVec = V2.Minus(V1);
    double edgeLen = edgeVec.Magnitude();

    if(edgeLen < LENGTH_EPS) {
        booleanFailed = true;
        return;
    }

    Vector t = edgeVec.WithMagnitude(1);

    // Step 6: compute normals
    Vector edgeMid = V1.Plus(V2).ScaledBy(0.5);
    SSurface *surf1 = surface.FindById(hSurf1);
    SSurface *surf2 = surface.FindById(hSurf2);

    Point2d uv1, uv2;
    surf1->ClosestPointTo(edgeMid, &uv1);
    surf2->ClosestPointTo(edgeMid, &uv2);
    Vector n1 = surf1->NormalAt(uv1);
    Vector n2 = surf2->NormalAt(uv2);
    if(n1.Magnitude() < LENGTH_EPS || n2.Magnitude() < LENGTH_EPS) {
        booleanFailed = true;
        return;
    }
    n1 = n1.WithMagnitude(1);
    n2 = n2.WithMagnitude(1);


    // Compute half-angle between the faces (for convex edge, n1.Dot(n2) < 0)
    double cosAngle = -n1.Dot(n2);
    if(cosAngle < -1.0) cosAngle = -1.0;
    if(cosAngle >  1.0) cosAngle =  1.0;

    // Reject concave edges: if n1·n2 > 0 then cosAngle < 0, meaning the
    // faces form a reflex (>180°) dihedral angle. Fillet is only supported
    // on convex edges.
    if(cosAngle < 0) {
        booleanFailed = true;
        return;
    }

    double half_angle = acos(cosAngle) / 2.0;
    double tanHalf = tan(half_angle);


    // Guard against degenerate cases (flat or concave edge)
    if(fabs(tanHalf) < 1e-10) {
        booleanFailed = true;
        return;
    }

    double setback = r / tanHalf;
    double arc_weight = sin(half_angle);


    if(r < LENGTH_EPS || setback > edgeLen / 2.0) {
        booleanFailed = true;
        return;
    }

    // Step 7: compute inward offset directions (same as chamfer)
    Vector d1 = n1.Cross(t).WithMagnitude(1);
    Vector d2 = n2.Cross(t).WithMagnitude(1);

    {
        Vector c1 = surf1->ctrl[0][0].Plus(surf1->ctrl[0][1])
                         .Plus(surf1->ctrl[1][0]).Plus(surf1->ctrl[1][1])
                         .ScaledBy(0.25);
        if(d1.Dot(c1.Minus(V1)) < 0) d1 = d1.ScaledBy(-1);


        Vector c2 = surf2->ctrl[0][0].Plus(surf2->ctrl[0][1])
                         .Plus(surf2->ctrl[1][0]).Plus(surf2->ctrl[1][1])
                         .ScaledBy(0.25);
        if(d2.Dot(c2.Minus(V1)) < 0) d2 = d2.ScaledBy(-1);

    }


    // Step 8: compute tangent contact points (setback along each face)
    // A0, A1: tangent contact points on surf1 at V1 and V2 ends
    // B0, B1: tangent contact points on surf2 at V1 and V2 ends
    Vector A0 = V1.Plus(d1.ScaledBy(setback));
    Vector A1 = V2.Plus(d1.ScaledBy(setback));
    Vector B0 = V1.Plus(d2.ScaledBy(setback));
    Vector B1 = V2.Plus(d2.ScaledBy(setback));

    // Step 9: create fillet surface
    // The fillet arc at V1 goes from A0 through V1 to B0 (quadratic rational Bezier)
    // Extruded along edge direction (V2-V1) to create the cylindrical surface.
    //
    // SSurface::FromExtrusionOf(sb, t0, t1):
    //   t0 = zero vector (start of extrusion at arc location)
    //   t1 = edgeVec (V2-V1), so the extruded surface goes from arc-at-V1 to arc-at-V2
    //
    // The arc SBezier:
    //   ctrl[0] = A0 (tangent to surf1), weight=1
    //   ctrl[1] = V1 (the edge vertex), weight=arc_weight
    //   ctrl[2] = B0 (tangent to surf2), weight=1

    SBezier arc = {};
    arc.deg = 2;
    arc.ctrl[0] = A0;
    arc.ctrl[1] = V1;
    arc.ctrl[2] = B0;
    arc.weight[0] = 1.0;
    arc.weight[1] = arc_weight;
    arc.weight[2] = 1.0;

    Vector zero = Vector::From(0, 0, 0);
    SSurface filletSurf = SSurface::FromExtrusionOf(&arc, zero, edgeVec);
    filletSurf.color = surf1->color;
    hEntity faceH = g->Remap(g->predef.entityB, Group::REMAP_FILLET_FACE);
    filletSurf.face = faceH.v;

    hSSurface hFillet = surface.AddAndAssignId(&filletSurf);

    // Re-lookup after reallocation
    surf1 = surface.FindById(hSurf1);
    surf2 = surface.FindById(hSurf2);
    sharedSC = curve.FindById(hSharedSC);

    // Step 10: create SCurves for fillet edges
    // The fillet surface has:
    //   - Two arc edges: arc-at-V1 (A0->B0 through V1) and arc-at-V2 (A1->B1 through V2)
    //     These are the seam lines (generated by FromExtrusionOf's trim lines)
    //   - Two straight edges along surf1 and surf2 contact lines

    // Contact line on surf1: A0->A1 (between fillet and surf1)
    hSCurve hContact1 = AddLinearCurve(this, A0, A1, hFillet, hSurf1);

    // Contact line on surf2: B0->B1 (between fillet and surf2)
    hSCurve hContact2 = AddLinearCurve(this, B0, B1, hFillet, hSurf2);

    // Find cap surfaces at V1 and V2 endpoints for fillet arcs
    hSSurface hCapSurfV1 = hFillet; // fallback
    hSSurface hCapSurfV2 = hFillet; // fallback
    for(SCurve &sc_scan : curve) {
        if(sc_scan.h == hSharedSC) continue;
        if(sc_scan.pts.n < 2) continue;
        Vector first = sc_scan.pts[0].p;
        Vector last = sc_scan.pts[sc_scan.pts.n - 1].p;
        bool touchesV1 = first.Equals(V1) || last.Equals(V1);
        bool touchesV2 = first.Equals(V2) || last.Equals(V2);
        if(touchesV1 && hCapSurfV1 == hFillet) {
            if(sc_scan.surfA != hSurf1 && sc_scan.surfA != hSurf2 && sc_scan.surfA.v != 0)
                hCapSurfV1 = sc_scan.surfA;
            else if(sc_scan.surfB != hSurf1 && sc_scan.surfB != hSurf2 && sc_scan.surfB.v != 0)
                hCapSurfV1 = sc_scan.surfB;
        }
        if(touchesV2 && hCapSurfV2 == hFillet) {
            if(sc_scan.surfA != hSurf1 && sc_scan.surfA != hSurf2 && sc_scan.surfA.v != 0)
                hCapSurfV2 = sc_scan.surfA;
            else if(sc_scan.surfB != hSurf1 && sc_scan.surfB != hSurf2 && sc_scan.surfB.v != 0)
                hCapSurfV2 = sc_scan.surfB;
        }
    }

    // Arc at V1: A0->B0 via V1 (cap, exposed)
    {
        SCurve sc = {};
        sc.isExact = true;
        sc.exact = arc;  // the arc from A0 through V1 to B0
        sc.exact.MakePwlInto(&sc.pts);
        sc.surfA = hFillet;
        sc.surfB = hCapSurfV1;
        hSCurve hArcV1 = curve.AddAndAssignId(&sc);

        // Arc at V2: A1->B1 via V2 (cap, exposed)
        SBezier arc2 = {};
        arc2.deg = 2;
        arc2.ctrl[0] = A1;
        arc2.ctrl[1] = V2;
        arc2.ctrl[2] = B1;
        arc2.weight[0] = 1.0;
        arc2.weight[1] = arc_weight;
        arc2.weight[2] = 1.0;

        SCurve sc2 = {};
        sc2.isExact = true;
        sc2.exact = arc2;
        sc2.exact.MakePwlInto(&sc2.pts);
        sc2.surfA = hFillet;
        sc2.surfB = hCapSurfV2;
        hSCurve hArcV2 = curve.AddAndAssignId(&sc2);

        // Re-lookup after additions
        surf1 = surface.FindById(hSurf1);
        surf2 = surface.FindById(hSurf2);
        sharedSC = curve.FindById(hSharedSC);
        SSurface *filletSurfPtr = surface.FindById(hFillet);

        // Step 11: build fillet trim polygon
        // A0->A1 (contact1 forward), A1->B1 via V2 (arcV2 forward),
        // B1->B0 (contact2 backwards), B0->A0 via V1 (arcV1 backwards)
        {
            STrimBy stb;
            stb = STrimBy::EntireCurve(this, hContact1, /*backwards=*/false);
            filletSurfPtr->trim.Add(&stb);

            stb = STrimBy::EntireCurve(this, hArcV2, /*backwards=*/false);
            filletSurfPtr->trim.Add(&stb);

            stb = STrimBy::EntireCurve(this, hContact2, /*backwards=*/true);
            filletSurfPtr->trim.Add(&stb);

            stb = STrimBy::EntireCurve(this, hArcV1, /*backwards=*/true);
            filletSurfPtr->trim.Add(&stb);
        }

        // Step 12: update surf1 trim (replace sharedSC -> hContact1)
        {
            bool v1AtStart = false;
            bool foundShared = false;
            for(STrimBy &stb : surf1->trim) {
                if(stb.curve == hSharedSC) {
                    v1AtStart = stb.start.Equals(V1);
                    foundShared = true;
                    stb.curve = hContact1;
                    stb.backwards = !v1AtStart;
                    if(v1AtStart) {
                        stb.start = A0;
                        stb.finish = A1;
                    } else {
                        stb.start = A1;
                        stb.finish = A0;
                    }
                    break;
                }
            }
            if(!foundShared) {
                booleanFailed = true;
                return;
            }

            for(STrimBy &stb_n : surf1->trim) {
                if(stb_n.curve == hContact1) continue;
                SCurve *nc = curve.FindByIdNoOops(stb_n.curve);
                if(!nc) continue;
                if(stb_n.start.Equals(V1)) {
                    InsertPointIntoCurvePts(nc, A0);
                    stb_n.start = A0;
                } else if(stb_n.start.Equals(V2)) {
                    InsertPointIntoCurvePts(nc, A1);
                    stb_n.start = A1;
                }
                if(stb_n.finish.Equals(V1)) {
                    InsertPointIntoCurvePts(nc, A0);
                    stb_n.finish = A0;
                } else if(stb_n.finish.Equals(V2)) {
                    InsertPointIntoCurvePts(nc, A1);
                    stb_n.finish = A1;
                }
            }
        }

        surf2 = surface.FindById(hSurf2);

        // Step 13: update surf2 trim (replace sharedSC -> hContact2)
        {
            bool v1AtStart2 = false;
            bool foundShared2 = false;
            for(STrimBy &stb : surf2->trim) {
                if(stb.curve == hSharedSC) {
                    v1AtStart2 = stb.start.Equals(V1);
                    foundShared2 = true;
                    stb.curve = hContact2;
                    stb.backwards = !v1AtStart2;
                    if(v1AtStart2) {
                        stb.start = B0;
                        stb.finish = B1;
                    } else {
                        stb.start = B1;
                        stb.finish = B0;
                    }
                    break;
                }
            }
            if(!foundShared2) {
                booleanFailed = true;
                return;
            }

            for(STrimBy &stb_n : surf2->trim) {
                if(stb_n.curve == hContact2) continue;
                SCurve *nc = curve.FindByIdNoOops(stb_n.curve);
                if(!nc) continue;
                if(stb_n.start.Equals(V1)) {
                    InsertPointIntoCurvePts(nc, B0);
                    stb_n.start = B0;
                } else if(stb_n.start.Equals(V2)) {
                    InsertPointIntoCurvePts(nc, B1);
                    stb_n.start = B1;
                }
                if(stb_n.finish.Equals(V1)) {
                    InsertPointIntoCurvePts(nc, B0);
                    stb_n.finish = B0;
                } else if(stb_n.finish.Equals(V2)) {
                    InsertPointIntoCurvePts(nc, B1);
                    stb_n.finish = B1;
                }
            }
        }

        // Step 14: remove old shared SCurve
        // Step 15: update cap surface trims at V1 and V2 for fillet
        // Same logic as chamfer Step 15, but using fillet contact points:
        //   hCapSurfV1 trims: V1->A0 (surf1-side) or V1->B0 (surf2-side); add hArcV1
        //   hCapSurfV2 trims: V2->A1 (surf1-side) or V2->B1 (surf2-side); add hArcV2
        if(hCapSurfV1 != hFillet) {
            SSurface *capSurf1 = surface.FindById(hCapSurfV1);
            for(STrimBy &stb_c : capSurf1->trim) {
                SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
                if(!nc) continue;
                bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
                bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
                if(stb_c.start.Equals(V1)) {
                    if(bordersSurf1) { InsertPointIntoCurvePts(nc, A0); stb_c.start = A0; }
                    else if(bordersSurf2) { InsertPointIntoCurvePts(nc, B0); stb_c.start = B0; }
                }
                if(stb_c.finish.Equals(V1)) {
                    if(bordersSurf1) { InsertPointIntoCurvePts(nc, A0); stb_c.finish = A0; }
                    else if(bordersSurf2) { InsertPointIntoCurvePts(nc, B0); stb_c.finish = B0; }
                }
            }
            // Detect gap direction for fillet V1 end: which endpoint (A0 or B0) is the 'finish'?
            bool arcV1Backwards = false; // default: forward A0->B0
            for(const STrimBy &stb_check : capSurf1->trim) {
                if(stb_check.finish.Equals(B0)) { arcV1Backwards = true; break; }  // gap goes B0->A0
                if(stb_check.finish.Equals(A0)) { arcV1Backwards = false; break; } // gap goes A0->B0
            }
            STrimBy stbArc1 = STrimBy::EntireCurve(this, hArcV1, arcV1Backwards);
            capSurf1->trim.Add(&stbArc1);
        }
        if(hCapSurfV2 != hFillet) {
            SSurface *capSurf2 = surface.FindById(hCapSurfV2);
            for(STrimBy &stb_c : capSurf2->trim) {
                SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
                if(!nc) continue;
                bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
                bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
                if(stb_c.start.Equals(V2)) {
                    if(bordersSurf1) { InsertPointIntoCurvePts(nc, A1); stb_c.start = A1; }
                    else if(bordersSurf2) { InsertPointIntoCurvePts(nc, B1); stb_c.start = B1; }
                }
                if(stb_c.finish.Equals(V2)) {
                    if(bordersSurf1) { InsertPointIntoCurvePts(nc, A1); stb_c.finish = A1; }
                    else if(bordersSurf2) { InsertPointIntoCurvePts(nc, B1); stb_c.finish = B1; }
                }
            }
            // Detect gap direction for fillet V2 end: which endpoint (A1 or B1) is the 'finish'?
            bool arcV2Backwards = false; // default: forward A1->B1
            for(const STrimBy &stb_check : capSurf2->trim) {
                if(stb_check.finish.Equals(B1)) { arcV2Backwards = true; break; }  // gap goes B1->A1
                if(stb_check.finish.Equals(A1)) { arcV2Backwards = false; break; } // gap goes A1->B1
            }
            STrimBy stbArc2 = STrimBy::EntireCurve(this, hArcV2, arcV2Backwards);
            capSurf2->trim.Add(&stbArc2);
        }

        // Safety: verify no surface trim still references hSharedSC before removing.
        bool hSharedStillReferenced = false;
        for(SSurface &ss : surface) {
            for(STrimBy *stb = ss.trim.First(); stb; stb = ss.trim.NextAfter(stb)) {
                if(stb->curve == hSharedSC) { hSharedStillReferenced = true; break; }
            }
            if(hSharedStillReferenced) break;
        }
        if(hSharedStillReferenced) { booleanFailed = true; return; }

        curve.RemoveById(hSharedSC);
    }
}

} // namespace SolveSpace
