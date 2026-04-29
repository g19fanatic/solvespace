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
// Helper: read-only check whether InsertPointIntoCurvePts would succeed.
// Returns true if P already exists in the curve or lies on one of its
// piecewise-linear segments. Does NOT modify the curve.
//-----------------------------------------------------------------------------
static bool CanInsertPointIntoCurvePts(SCurve *sc, Vector P) {
    for(int i = 0; i < sc->pts.n; i++) {
        if(sc->pts[i].p.Equals(P)) return true;
    }
    for(int i = 0; i < sc->pts.n - 1; i++) {
        Vector a = sc->pts[i].p;
        Vector b = sc->pts[i+1].p;
        Vector ab = b.Minus(a);
        double len = ab.Magnitude();
        if(len < LENGTH_EPS) continue;

        Vector ap = P.Minus(a);
        double t = ap.Dot(ab) / (len * len);
        if(t < LENGTH_EPS/len || t > 1.0 - LENGTH_EPS/len) continue;

        Vector closest = a.Plus(ab.ScaledBy(t));
        if(!closest.Equals(P)) continue;

        return true;
    }
    return false;
}

//-----------------------------------------------------------------------------
// Helper: check if a curve's pts array contains point P (exact match only).
//-----------------------------------------------------------------------------
static bool CurvePtsContain(SCurve *sc, Vector P) {
    for(int i = 0; i < sc->pts.n; i++) {
        if(sc->pts[i].p.Equals(P)) return true;
    }
    return false;
}

//-----------------------------------------------------------------------------
// Helper: truncate a curve's pts so it ends at `newEnd` instead of `oldEnd`.
// Called after InsertPointIntoCurvePts to remove the stale original corner
// vertex (V1 or V2) from the pts array of a neighboring curve.
//
// Precondition: newEnd is already in sc->pts (just inserted by InsertPointIntoCurvePts).
// Idempotent: if oldEnd is no longer at either terminal, does nothing.
//-----------------------------------------------------------------------------
static void TruncateCurveAtVertex(SCurve *sc, Vector oldEnd, Vector newEnd) {
    // Find newEnd in pts
    int newIdx = -1;
    for(int i = 0; i < sc->pts.n; i++) {
        if(sc->pts[i].p.Equals(newEnd)) { newIdx = i; break; }
    }
    if(newIdx < 0) return;

    List<SCurvePt> newPts = {};
    if(sc->pts[0].p.Equals(oldEnd)) {
        // oldEnd at start: keep pts[newIdx..n-1]
        for(int i = newIdx; i < sc->pts.n; i++) newPts.Add(&sc->pts[i]);
    } else if(sc->pts[sc->pts.n - 1].p.Equals(oldEnd)) {
        // oldEnd at end: keep pts[0..newIdx]
        for(int i = 0; i <= newIdx; i++) newPts.Add(&sc->pts[i]);
    } else {
        newPts.Clear();
        return;  // oldEnd not at either end — already truncated (idempotent), skip
    }
    sc->pts.Clear();
    for(int i = 0; i < newPts.n; i++) sc->pts.Add(&newPts[i]);
    newPts.Clear();
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
// Helper: remove the last entry from ss->trim.
//-----------------------------------------------------------------------------
static void RemoveTrimLast(SSurface *ss) {
    if(ss->trim.n == 0) return;
    List<STrimBy> tmp = {};
    for(int i = 0; i < ss->trim.n - 1; i++) tmp.Add(&ss->trim[i]);
    ss->trim.Clear();
    for(int i = 0; i < tmp.n; i++) ss->trim.Add(&tmp[i]);
    tmp.Clear();
}

//-----------------------------------------------------------------------------
// Helper: After TruncateCurveAtVertex modifies a curve's pts, update ALL
// surfaces' STrimBy entries that reference that curve and still have oldPt
// as their start or finish. This propagates endpoint changes to surfaces
// not directly processed in Steps 12/13.
//-----------------------------------------------------------------------------
static void UpdateAllSurfaceTrimEndpoints(SShell *shell, hSCurve hSC,
                                          Vector oldPt, Vector newPt) {
    for(SSurface &ss : shell->surface) {
        for(STrimBy &stb : ss.trim) {
            if(stb.curve != hSC) continue;
            if(stb.start.Equals(oldPt))  stb.start  = newPt;
            if(stb.finish.Equals(oldPt)) stb.finish = newPt;
        }
    }
}

//-----------------------------------------------------------------------------
// Helper: find the first gap in a trim polygon.
// Returns the index i such that trim[i].finish != trim[(i+1)%n].start.
// Sets *gapEnd = trim[i].finish, *gapStart = trim[(i+1)%n].start.
// Returns -1 if the polygon is closed (no gap found).
//-----------------------------------------------------------------------------
static int FindTrimGap(SSurface *ss, Vector *gapEnd, Vector *gapStart) {
    if(ss->trim.n < 2) return -1;
    for(int i = 0; i < ss->trim.n; i++) {
        const STrimBy &cur = ss->trim[i];
        const STrimBy &nxt = ss->trim[(i + 1) % ss->trim.n];
        if(!cur.finish.Equals(nxt.start)) {
            *gapEnd   = cur.finish;
            *gapStart = nxt.start;
            return i;
        }
    }
    return -1;
}

//-----------------------------------------------------------------------------
// Diagnostic: validate that ALL surfaces in a shell have closed trim loops.
// Uses GRAPH CONNECTIVITY (endpoint matching) instead of sequential ordering,
// because SolveSpace stores trim entries in ARBITRARY order — they are
// assembled into closed contours by AssemblePolygon at triangulation time.
//
// For each trim's finish endpoint, checks if ANY other trim's start matches.
// For each trim's start endpoint, checks if ANY other trim's finish matches.
// Unmatched endpoints indicate genuine topology breaks (open trim loops).
// Returns the total number of unmatched endpoints found across all surfaces.
// This is a read-only diagnostic — it does NOT modify any data.
//-----------------------------------------------------------------------------
static int ValidateAllTrimLoops(SShell *shell, const char *context) {
    int totalGaps = 0;
    for(SSurface &ss : shell->surface) {
        if(ss.trim.n < 2) continue;
        int nUnmatched = 0;
        // Check each trim's finish: does ANY other trim's start match it?
        for(int i = 0; i < ss.trim.n; i++) {
            Vector fin = ss.trim[i].finish;
            bool matched = false;
            for(int j = 0; j < ss.trim.n; j++) {
                if(j == i) continue;
                if(fin.Equals(ss.trim[j].start)) {
                    matched = true;
                    break;
                }
            }
            if(!matched) {
                if(nUnmatched == 0) {
                    dbp("TRIM-DIAG [%s] Surface h=%08x (%d trims): OPEN LOOP",
                        context, ss.h.v, ss.trim.n);
                }
                dbp("  trim[%d].finish=(%.4f,%.4f,%.4f) has no matching start",
                    i, fin.x, fin.y, fin.z);
                nUnmatched++;
            }
        }
        // Check each trim's start: does ANY other trim's finish match it?
        for(int i = 0; i < ss.trim.n; i++) {
            Vector st = ss.trim[i].start;
            bool matched = false;
            for(int j = 0; j < ss.trim.n; j++) {
                if(j == i) continue;
                if(st.Equals(ss.trim[j].finish)) {
                    matched = true;
                    break;
                }
            }
            if(!matched) {
                if(nUnmatched == 0) {
                    dbp("TRIM-DIAG [%s] Surface h=%08x (%d trims): OPEN LOOP",
                        context, ss.h.v, ss.trim.n);
                }
                dbp("  trim[%d].start=(%.4f,%.4f,%.4f) has no matching finish",
                    i, st.x, st.y, st.z);
                nUnmatched++;
            }
        }
        totalGaps += nUnmatched;
    }
    if(totalGaps == 0) {
        dbp("TRIM-DIAG [%s] All surfaces have closed trim loops", context);
    } else {
        dbp("TRIM-DIAG [%s] Found %d total unmatched endpoint(s)", context, totalGaps);
    }
    return totalGaps;
}

//-----------------------------------------------------------------------------
// Trim closure enforcement: detect unmatched endpoints via graph connectivity
// and repair them by creating linear bridge curves. This is a safety net that
// ensures every surface has a closed trim loop after corner synthesis.
// Uses a two-pass approach to avoid iterator invalidation:
//   Pass 1: collect all needed repairs (surface handle, gap finish, gap start)
//   Pass 2: apply repairs (create linear curves, add trims)
// Returns the number of repairs made.
//-----------------------------------------------------------------------------
static int RepairOpenTrimLoops(SShell *shell, const char *context) {
    // First run validation to detect any gaps
    int gapsBefore = ValidateAllTrimLoops(shell, context);
    if(gapsBefore == 0) return 0;

    // Pass 1: collect repair requests
    struct TrimRepair {
        hSSurface hSurf;
        Vector gapFinish;
        Vector gapStart;
    };
    std::vector<TrimRepair> repairs;

    for(SSurface &ss : shell->surface) {
        if(ss.trim.n < 2) continue;

        // Find unmatched finish endpoints
        std::vector<Vector> unmatchedFinish;
        for(int i = 0; i < ss.trim.n; i++) {
            Vector fin = ss.trim[i].finish;
            bool matched = false;
            for(int j = 0; j < ss.trim.n; j++) {
                if(j == i) continue;
                if(fin.Equals(ss.trim[j].start)) { matched = true; break; }
            }
            if(!matched) unmatchedFinish.push_back(fin);
        }

        // Find unmatched start endpoints
        std::vector<Vector> unmatchedStart;
        for(int i = 0; i < ss.trim.n; i++) {
            Vector st = ss.trim[i].start;
            bool matched = false;
            for(int j = 0; j < ss.trim.n; j++) {
                if(j == i) continue;
                if(st.Equals(ss.trim[j].finish)) { matched = true; break; }
            }
            if(!matched) unmatchedStart.push_back(st);
        }

        // Greedy closest-pair matching: pair each unmatched finish with nearest start
        std::vector<bool> usedStart(unmatchedStart.size(), false);
        for(const Vector &fin : unmatchedFinish) {
            double bestDist = 1e20;
            int bestSi = -1;
            for(size_t si = 0; si < unmatchedStart.size(); si++) {
                if(usedStart[si]) continue;
                double d = fin.Minus(unmatchedStart[si]).Magnitude();
                if(d < bestDist) { bestDist = d; bestSi = (int)si; }
            }
            if(bestSi < 0) continue;
            usedStart[bestSi] = true;

            TrimRepair tr;
            tr.hSurf = ss.h;
            tr.gapFinish = fin;
            tr.gapStart = unmatchedStart[bestSi];
            repairs.push_back(tr);

            dbp("TRIM-REPAIR [%s] Surface h=%08x: will bridge "
                "(%.4f,%.4f,%.4f)->(%.4f,%.4f,%.4f) dist=%.6f",
                context, ss.h.v,
                fin.x, fin.y, fin.z,
                unmatchedStart[bestSi].x, unmatchedStart[bestSi].y,
                unmatchedStart[bestSi].z, bestDist);
        }
    }

    // Pass 2: apply repairs
    int totalRepairs = 0;
    for(const TrimRepair &tr : repairs) {
        hSCurve hBridge = AddLinearCurve(shell, tr.gapFinish, tr.gapStart,
                                         tr.hSurf, tr.hSurf);
        SSurface *ssPtr = shell->surface.FindById(tr.hSurf);
        STrimBy stbBridge = STrimBy::EntireCurve(shell, hBridge, /*backwards=*/false);
        ssPtr->trim.Add(&stbBridge);
        totalRepairs++;
    }

    if(totalRepairs > 0) {
        dbp("TRIM-REPAIR [%s] Made %d repair(s)", context, totalRepairs);
        // Re-validate after repairs
        ValidateAllTrimLoops(shell, context);
    }
    return totalRepairs;
}

//-----------------------------------------------------------------------------
// Helper: insert newStb into ss->trim at position insertAfter+1, preserving
// all existing entries. Used to close a gap at an arbitrary trim position.
//-----------------------------------------------------------------------------
static void InsertTrimAt(SSurface *ss, int insertAfter, STrimBy *newStb) {
    List<STrimBy> tmp = {};
    for(int i = 0; i <= insertAfter; i++) tmp.Add(&ss->trim[i]);
    tmp.Add(newStb);
    for(int i = insertAfter + 1; i < ss->trim.n; i++) tmp.Add(&ss->trim[i]);
    ss->trim.Clear();
    for(int i = 0; i < tmp.n; i++) ss->trim.Add(&tmp[i]);
    tmp.Clear();
}

//-----------------------------------------------------------------------------
// Helper: bridge any open trim polygon gap in surface hSurf of shell, by
// adding a new linear SCurve from gapEnd to gapStart with the given surfA.
// The new curve is added to hSurf's trim at the gap position.
// Does nothing if the trim polygon is already closed.
//
// APPROACH:
// 1. Pre-pass: remove degenerate trims (start==finish).
// 2. Find a simple closed sub-chain of maximum size that has NO branch points
//    (no two trims sharing the same .start). This is the main face boundary.
//    Any trims not in this sub-chain are orphaned and are removed.
// 3. If the remaining polygon is open, add a bridge curve to close it.
//-----------------------------------------------------------------------------
static void BridgeTrimGapIfOpen(SShell *shell, hSSurface hSurf, hSSurface surfA) {
    SSurface *ss = shell->surface.FindById(hSurf);
    if(ss->trim.n < 2) return;
    // Pre-pass: remove degenerate trims (start==finish).
    {
        bool hasDegen = false;
        for(int i = 0; i < ss->trim.n; i++) {
            if(ss->trim[i].start.Equals(ss->trim[i].finish)) { hasDegen = true; break; }
        }
        if(hasDegen) {
            List<STrimBy> kept = {};
            for(int i = 0; i < ss->trim.n; i++) {
                if(!ss->trim[i].start.Equals(ss->trim[i].finish))
                    kept.Add(&ss->trim[i]);
            }
            ss->trim.Clear();
            for(int i = 0; i < kept.n; i++) ss->trim.Add(&kept[i]);
            kept.Clear();
            if(ss->trim.n < 2) return;
        }
    }
    // Find the largest simple closed sub-chain with no branch points.
    // Strategy: try each trim as chain start, follow the chain. Among all
    // chains that close back to their start (closed sub-chains), keep the
    // LARGEST one. Trims not in that chain are orphaned and removed.
    // For open chains (no closure found), keep the deepest chain and bridge.
    //
    // Branch detection: when two trims share the same .start, the polygon is
    // ambiguous. We pick the FIRST match for each step (greedy traversal).
    // The largest closed sub-chain is the correct main polygon.
    int  n            = ss->trim.n;
    int  bestStart    = -1;
    int  bestLastIdx  = -1;
    int  bestVisited  = -1;
    Vector bestGapEnd   = {};
    Vector bestGapStart = {};
    std::vector<bool> bestVis(n, false);
    bool foundClosed   = false;   // Did we find ANY closed sub-chain?
    int  bestClosedCnt = -1;      // Size of largest closed sub-chain found

    for(int s = 0; s < n; s++) {
        std::vector<bool> vis(n, false);
        vis[s] = true;
        Vector sPt  = ss->trim[s].start;
        Vector cur  = ss->trim[s].finish;
        int    last = s;
        int    cnt  = 1;
        bool   ok   = true;
        for(int iter = 1; iter < n; iter++) {
            // Check if we've closed the loop already
            if(cur.Equals(sPt)) break;
            bool found = false;
            for(int j = 0; j < n; j++) {
                if(vis[j]) continue;
                if(cur.Equals(ss->trim[j].start)) {
                    cur = ss->trim[j].finish;
                    last = j;
                    vis[j] = true;
                    found = true;
                    cnt++;
                    break;
                }
            }
            if(!found) { ok = false; break; }
        }
        bool closed = ok && cur.Equals(sPt);
        if(closed) {
            // Closed sub-chain of length cnt
            if(cnt > bestClosedCnt) {
                foundClosed    = true;
                bestClosedCnt  = cnt;
                bestVisited    = cnt;
                bestVis        = vis;
                bestStart      = s;
                bestLastIdx    = last;
                bestGapEnd     = cur;
                bestGapStart   = sPt;
            }
        } else if(!foundClosed && cnt > bestVisited) {
            // No closed chain found yet — keep the deepest open chain
            bestVisited  = cnt;
            bestStart    = s;
            bestLastIdx  = last;
            bestGapEnd   = cur;
            bestGapStart = sPt;
            bestVis      = vis;
        }
    }

    // Remove any trims not in the best chain (orphaned or sub-loop)
    ss = shell->surface.FindById(hSurf);
    n = ss->trim.n;
    if(bestVisited < n) {
        List<STrimBy> kept = {};
        int removed_before_last = 0;
        for(int i = 0; i < ss->trim.n; i++) {
            if(bestVis[i]) {
                kept.Add(&ss->trim[i]);
            } else {
                if(i < bestLastIdx) removed_before_last++;
            }
        }
        bestLastIdx -= removed_before_last;
        ss->trim.Clear();
        for(int i = 0; i < kept.n; i++) ss->trim.Add(&kept[i]);
        kept.Clear();
    }

    if(foundClosed) return;  // Polygon is closed (after orphan removal)
    if(bestStart < 0) return;

    // Open polygon: bridge the gap from gapEnd to gapStart
    Vector gapEnd   = bestGapEnd;
    Vector gapStart = bestGapStart;
    int    gapIdx   = bestLastIdx;
    ss = shell->surface.FindById(hSurf);
    hSCurve hBridge = AddLinearCurve(shell, gapEnd, gapStart, surfA, hSurf);
    ss = shell->surface.FindById(hSurf);
    STrimBy stbBridge = STrimBy::EntireCurve(shell, hBridge, false);
    InsertTrimAt(ss, gapIdx, &stbBridge);
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

    // Step 5.5: Early adjacency pre-scan — detect if V1 or V2 was truncated
    // by a prior chamfer, and extend back to the original corner position.
    // V1_orig/V2_orig preserve the topology-matching values for downstream lookups.
    Vector V1_orig = V1;
    Vector V2_orig = V2;
    hSSurface hAdjCapAtV1 = {};
    hSSurface hAdjCapAtV2 = {};
    for(SCurve &sc_pre : curve) {
        if(sc_pre.h == hSharedSC) continue;
        if(sc_pre.pts.n < 2) continue;
        Vector first_pre = sc_pre.pts[0].p;
        Vector last_pre = sc_pre.pts[sc_pre.pts.n - 1].p;
        bool touchesV1_pre = first_pre.Equals(V1) || last_pre.Equals(V1);
        bool touchesV2_pre = first_pre.Equals(V2) || last_pre.Equals(V2);
        if(!touchesV1_pre && !touchesV2_pre) continue;
        for(int side_pre = 0; side_pre < 2; side_pre++) {
            hSSurface hCand_pre = (side_pre == 0) ? sc_pre.surfA : sc_pre.surfB;
            if(hCand_pre.v == 0) continue;
            if(hCand_pre == hSurf1 || hCand_pre == hSurf2) continue;
            SSurface *sCand_pre = surface.FindById(hCand_pre);
            if(sCand_pre->degm != 1 || sCand_pre->degn != 1) continue; // only flat caps
            // Project candidate cap's ctrl points onto edge tangent
            double projMax = -1e20, projMin = 1e20;
            for(int ci = 0; ci < 2; ci++) {
                for(int cj = 0; cj < 2; cj++) {
                    double p = sCand_pre->ctrl[ci][cj].Dot(t);
                    if(p > projMax) projMax = p;
                    if(p < projMin) projMin = p;
                }
            }
            if(touchesV2_pre) {
                double v2proj = V2.Dot(t);
                if(projMax > v2proj + LENGTH_EPS) {
                    V2 = V2.Plus(t.ScaledBy(projMax - v2proj));
                    hAdjCapAtV2 = hCand_pre;
                }
            }
            if(touchesV1_pre) {
                double v1proj = V1.Dot(t);
                if(projMin < v1proj - LENGTH_EPS) {
                    V1 = V1.Plus(t.ScaledBy(projMin - v1proj));
                    hAdjCapAtV1 = hCand_pre;
                }
            }
        }
    }
    // If V1 or V2 was extended, recompute edge geometry
    if(!V1.Equals(V1_orig) || !V2.Equals(V2_orig)) {
        edgeVec = V2.Minus(V1);
        edgeLen = edgeVec.Magnitude();
        if(edgeLen < LENGTH_EPS) { booleanFailed = true; return; }
        if(dist < LENGTH_EPS || dist > edgeLen / 2.0) { booleanFailed = true; return; }
        t = edgeVec.WithMagnitude(1);
    }


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

    // Orientation normalization: ensure chamfer surface normal faces outward.
    // The chamfer surface FromPlane(A, B-A, D-A) has normal = (B-A).Cross(D-A)
    // = t.Cross(d2-d1)*dist^2. We want this to align with n1+n2 (outward bisector).
    // If not, swap surf1/surf2 roles so geometry is consistently oriented regardless
    // of which face was selected as entityB vs entityC.
    {
        Vector expectedNormal = t.Cross(d2.Minus(d1));
        if(expectedNormal.Dot(n1.Plus(n2)) > 0) {
            std::swap(hSurf1, hSurf2);
            std::swap(d1, d2);
            std::swap(n1, n2);
            surf1 = surface.FindById(hSurf1);
            surf2 = surface.FindById(hSurf2);
        }
    }

    // Step 8: compute chamfer corners
    // A, B: setback points on surf1 (at V1 and V2 ends)
    // D, C: setback points on surf2 (at V1 and V2 ends)
    Vector A = V1.Plus(d1.ScaledBy(dist));
    Vector B = V2.Plus(d1.ScaledBy(dist));
    Vector D = V1.Plus(d2.ScaledBy(dist));
    Vector C = V2.Plus(d2.ScaledBy(dist));

    // Step 8.5: Trim oversized cap at adjacent chamfer plane intersection.
    // When V was extended by Step 5.5, C (or D) overshoots beyond the adjacent
    // chamfer's cap plane. Intersect the D→C (or A→D) line with the adjacent
    // cap's plane to find the correct diagonal trim point.
    // ALSO trim B (surf1 at V2) and A (surf1 at V1) because orientation
    // normalization may swap which side (d1/d2) overshoots. The lambda ∈ (0,1)
    // guard ensures trimming only happens when the point actually overshoots.
    // For the V2 end: intersect A→B with adjCap plane → trim B
    //                 intersect D→C with adjCap plane → trim C
    // For the V1 end: intersect B→A with adjCap plane → trim A
    //                 intersect C→D with adjCap plane → trim D
    if(hAdjCapAtV2.v != 0) {
        SSurface *adjCapV2 = surface.FindById(hAdjCapAtV2);
        Point2d uvMidAdj2;
        uvMidAdj2.x = 0.5;
        uvMidAdj2.y = 0.5;
        Vector nAdj2 = adjCapV2->NormalAt(uvMidAdj2).WithMagnitude(1.0);
        Vector ptAdj2 = adjCapV2->ctrl[0][0];
        // Compute BOTH lambdas first to check for revert condition.
        // If either lambda > 1 (trim point beyond edge), the extension created
        // an oversized cap that physically overlaps the adjacent cap's surface.
        // This happens when dist2 > dist1 (current chamfer is LARGER than adjacent).
        // In that case, revert V2 to V2_orig and recompute B, C without extension.
        Vector DC = C.Minus(D);
        double denom2 = nAdj2.Dot(DC);
        double lambda2 = (fabs(denom2) > LENGTH_EPS)
                              ? nAdj2.Dot(ptAdj2.Minus(D)) / denom2
                              : 0.0;
        Vector AB = B.Minus(A);
        double denomB = nAdj2.Dot(AB);
        double lambdaB = (fabs(denomB) > LENGTH_EPS)
                              ? nAdj2.Dot(ptAdj2.Minus(A)) / denomB
                              : 0.0;
        // If either lambda exceeds 1, the V extension causes surface overlap.
        // Revert V2 to V2_orig and skip diagonal trim for this endpoint.
        if(lambda2 > 1.0 + 1e-3 || lambdaB > 1.0 + 1e-3) {
            V2 = V2_orig;
            B = V2.Plus(d1.ScaledBy(dist));
            C = V2.Plus(d2.ScaledBy(dist));
            hAdjCapAtV2 = {};
            // Recompute edge geometry with reverted V2
            edgeVec = V2.Minus(V1);
            edgeLen = edgeVec.Magnitude();
            if(edgeLen < LENGTH_EPS) { booleanFailed = true; return; }
            if(dist < LENGTH_EPS || dist > edgeLen / 2.0) { booleanFailed = true; return; }
            t = edgeVec.WithMagnitude(1);
        } else {
            // Normal case: apply diagonal trims where lambda ∈ (0,1)
            if(lambda2 > LENGTH_EPS && lambda2 < 1.0 - LENGTH_EPS) {
                C = D.Plus(DC.ScaledBy(lambda2));
            }
            if(lambdaB > LENGTH_EPS && lambdaB < 1.0 - LENGTH_EPS) {
                B = A.Plus(AB.ScaledBy(lambdaB));
            }
        }
    }
    if(hAdjCapAtV1.v != 0) {
        SSurface *adjCapV1 = surface.FindById(hAdjCapAtV1);
        Point2d uvMidAdj1;
        uvMidAdj1.x = 0.5;
        uvMidAdj1.y = 0.5;
        Vector nAdj1 = adjCapV1->NormalAt(uvMidAdj1).WithMagnitude(1.0);
        Vector ptAdj1 = adjCapV1->ctrl[0][0];
        // Compute BOTH lambdas first to check for revert condition.
        // If either lambda > 1, the extension causes surface overlap → revert V1.
        Vector CD = D.Minus(C);
        double denom1 = nAdj1.Dot(CD);
        double lambda1 = (fabs(denom1) > LENGTH_EPS)
                              ? nAdj1.Dot(ptAdj1.Minus(C)) / denom1
                              : 0.0;
        Vector BA = A.Minus(B);
        double denomA = nAdj1.Dot(BA);
        double lambdaA = (fabs(denomA) > LENGTH_EPS)
                              ? nAdj1.Dot(ptAdj1.Minus(B)) / denomA
                              : 0.0;
        // If either lambda exceeds 1, revert V1 to V1_orig.
        if(lambda1 > 1.0 + 1e-3 || lambdaA > 1.0 + 1e-3) {
            V1 = V1_orig;
            A = V1.Plus(d1.ScaledBy(dist));
            D = V1.Plus(d2.ScaledBy(dist));
            hAdjCapAtV1 = {};
            // Recompute edge geometry with reverted V1
            edgeVec = V2.Minus(V1);
            edgeLen = edgeVec.Magnitude();
            if(edgeLen < LENGTH_EPS) { booleanFailed = true; return; }
            if(dist < LENGTH_EPS || dist > edgeLen / 2.0) { booleanFailed = true; return; }
            t = edgeVec.WithMagnitude(1);
        } else {
            // Normal case: apply diagonal trims where lambda ∈ (0,1)
            if(lambda1 > LENGTH_EPS && lambda1 < 1.0 - LENGTH_EPS) {
                D = C.Plus(CD.ScaledBy(lambda1));
            }
            if(lambdaA > LENGTH_EPS && lambdaA < 1.0 - LENGTH_EPS) {
                A = B.Plus(BA.ScaledBy(lambdaA));
            }
        }
    }

    // Step 9: create chamfer surface
    // FromPlane(origin, u, v):
    //   ctrl[0][0] = origin     = A  (u=0,v=0)
    //   ctrl[0][1] = origin+u   = B  (u=1,v=0)
    //   ctrl[1][0] = origin+v   = D  (u=0,v=1)
    //   ctrl[1][1] = origin+u+v = C  (u=1,v=1)
    SSurface chamferSurf = SSurface::FromPlane(A, B.Minus(A), D.Minus(A));
    // After diagonal trim (Step 8.5), A/B/C/D may form a non-parallelogram
    // (trapezoid). FromPlane computes ctrl[1][1] = A+(B-A)+(D-A) = B+D-A,
    // which differs from C when the quad is a trapezoid. Fix ctrl[1][1].
    chamferSurf.ctrl[1][1] = C;
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
    double capV1Score = -1.0;
    double capV2Score = -1.0;
    for(SCurve &sc_scan : curve) {
        if(sc_scan.h == hSharedSC) continue;
        if(sc_scan.pts.n < 2) continue;
        Vector first = sc_scan.pts[0].p;
        Vector last = sc_scan.pts[sc_scan.pts.n - 1].p;
        bool touchesV1 = first.Equals(V1_orig) || last.Equals(V1_orig);
        bool touchesV2 = first.Equals(V2_orig) || last.Equals(V2_orig);
        if(!touchesV1 && !touchesV2) continue;
        for(int side = 0; side < 2; side++) {
            hSSurface hCand = (side == 0) ? sc_scan.surfA : sc_scan.surfB;
            if(hCand.v == 0) continue;
            if(hCand == hSurf1 || hCand == hSurf2) continue;
            SSurface *sCand = surface.FindById(hCand);
            Point2d uvMid; uvMid.x = 0.5; uvMid.y = 0.5;
            Vector nCand = sCand->NormalAt(uvMid);
            // Composite score: geometric alignment (0..2) + trim.n>0 bonus (0 or 1).
            // Prefer surfaces with existing trim (external real cap) over trim.n==0
            // (internal ASSEMBLE faces). DIFF bodies: pocket ceiling has trim.n==0 but
            // still wins on geoScore (|n.Dot(t)|=1) vs side faces (geoScore~0).
            double geoScore = fabs(nCand.Dot(t));
            double score = geoScore * 2.0 + (sCand->trim.n > 0 ? 1.0 : 0.0);
            if(touchesV1 && score >= capV1Score) {
                capV1Score = score;
                hCapSurfV1 = hCand;
            }
            if(touchesV2 && score > capV2Score) {
                capV2Score = score;
                hCapSurfV2 = hCand;
            }
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

    // Pre-compute: skip neighbor curves bordering a non-flat (curved) cap surface
    // (e.g. a prior fillet).  Modifying such curves via UpdateAllSurfaceTrimEndpoints
    // would corrupt the curved surface's trim chain.  The resulting gaps on surf1/surf2
    // will be handled by BridgeTrimGapIfOpen + corner post-processing.
    bool skipCapV1Curves = false, skipCapV2Curves = false;
    if(hCapSurfV1 != hChamfer) {
        SSurface *cs1 = surface.FindById(hCapSurfV1);
        skipCapV1Curves = (cs1->degm != 1 || cs1->degn != 1);
    }
    if(hCapSurfV2 != hChamfer) {
        SSurface *cs2 = surface.FindById(hCapSurfV2);
        skipCapV2Curves = (cs2->degm != 1 || cs2->degn != 1);
    }


    // Pre-scan: check whether hCapSurfV1/V2's OWN trims contain a curve passing
    // through V1/V2 that borders hSurf1/hSurf2 where the setback point can't be
    // inserted.  This mirrors the bridge-skip guard logic to correctly
    // differentiate doubleop cases (prior chamfer on SAME face → shares V1 in
    // its own pts) from adjacent-edge cases (prior chamfer on a DIFFERENT edge →
    // V1 not in its own curve pts).
    if(!skipCapV1Curves && hCapSurfV1 != hChamfer) {
        SSurface *capS1 = surface.FindById(hCapSurfV1);
        if(capS1->degm == 1 && capS1->degn == 1) {
            for(STrimBy &stb_s : capS1->trim) {
                SCurve *nc = curve.FindByIdNoOops(stb_s.curve);
                if(!nc) continue;
                bool bS1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
                bool bS2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
                if(!bS1 && !bS2) continue;
                if(stb_s.start.Equals(V1_orig) || stb_s.finish.Equals(V1_orig)) {
                    Vector pt = bS1 ? A : D;
                    if(!CanInsertPointIntoCurvePts(nc, pt)) {
                        // Directional discriminator: the failing curve's direction
                        // from V1 determines whether this is a doubleop diagonal
                        // (direction aligns with pt) or an adjacent-chamfer diagonal
                        // (direction aligns with the opposite setback).
                        Vector pOther = stb_s.start.Equals(V1_orig) ? stb_s.finish : stb_s.start;
                        Vector curveDir = pOther.Minus(V1_orig);
                        Vector dirPt  = pt.Minus(V1_orig);
                        Vector dirOpp = (bS1 ? D : A).Minus(V1_orig);
                        double magPt  = dirPt.Magnitude();
                        double magOpp = dirOpp.Magnitude();
                        double dotPt  = (magPt  > LENGTH_EPS) ? fabs(curveDir.Dot(dirPt.WithMagnitude(1.0)))  : 0;
                        double dotOpp = (magOpp > LENGTH_EPS) ? fabs(curveDir.Dot(dirOpp.WithMagnitude(1.0))) : 0;
                        if(dotPt > dotOpp) {
                            skipCapV1Curves = true; break;
                        }
                        // else: adjacent chamfer — don't skip
                    }
                }
            }
        }
    }
    if(!skipCapV2Curves && hCapSurfV2 != hChamfer) {
        SSurface *capS2 = surface.FindById(hCapSurfV2);
        if(capS2->degm == 1 && capS2->degn == 1) {
            for(STrimBy &stb_s : capS2->trim) {
                SCurve *nc = curve.FindByIdNoOops(stb_s.curve);
                if(!nc) continue;
                bool bS1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
                bool bS2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
                if(!bS1 && !bS2) continue;
                if(stb_s.start.Equals(V2_orig) || stb_s.finish.Equals(V2_orig)) {
                    Vector pt = bS1 ? B : C;
                    if(!CanInsertPointIntoCurvePts(nc, pt)) {
                        // Directional discriminator (V2 variant): same logic as V1.
                        // For V2, the setback points are B (bS1) or C (bS2),
                        // and the opposite setbacks are C (bS1) or B (bS2).
                        Vector pOther = stb_s.start.Equals(V2_orig) ? stb_s.finish : stb_s.start;
                        Vector curveDir = pOther.Minus(V2_orig);
                        Vector dirPt  = pt.Minus(V2_orig);
                        Vector dirOpp = (bS1 ? C : B).Minus(V2_orig);
                        double magPt  = dirPt.Magnitude();
                        double magOpp = dirOpp.Magnitude();
                        double dotPt  = (magPt  > LENGTH_EPS) ? fabs(curveDir.Dot(dirPt.WithMagnitude(1.0)))  : 0;
                        double dotOpp = (magOpp > LENGTH_EPS) ? fabs(curveDir.Dot(dirOpp.WithMagnitude(1.0))) : 0;
                        if(dotPt > dotOpp) {
                            skipCapV2Curves = true; break;
                        }
                        // else: adjacent chamfer — don't skip
                    }
                }
            }
        }
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
                v1AtStart = stb.start.Equals(V1_orig);
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
            if(nc && skipCapV1Curves && (nc->surfA == hCapSurfV1 || nc->surfB == hCapSurfV1)) continue;
            if(nc && skipCapV2Curves && (nc->surfA == hCapSurfV2 || nc->surfB == hCapSurfV2)) continue;

            if(stb_n.start.Equals(V1_orig)) {
                if(InsertPointIntoCurvePts(nc, A)) { TruncateCurveAtVertex(nc, V1_orig, A); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1_orig, A); stb_n.start = A; }
            } else if(stb_n.start.Equals(V2_orig)) {
                if(InsertPointIntoCurvePts(nc, B)) { TruncateCurveAtVertex(nc, V2_orig, B); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2_orig, B); stb_n.start = B; }
            }

            if(stb_n.finish.Equals(V1_orig)) {
                if(InsertPointIntoCurvePts(nc, A)) { TruncateCurveAtVertex(nc, V1_orig, A); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1_orig, A); stb_n.finish = A; }
            } else if(stb_n.finish.Equals(V2_orig)) {
                if(InsertPointIntoCurvePts(nc, B)) { TruncateCurveAtVertex(nc, V2_orig, B); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2_orig, B); stb_n.finish = B; }
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
                v1AtStart2 = stb.start.Equals(V1_orig);
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
            if(nc && skipCapV1Curves && (nc->surfA == hCapSurfV1 || nc->surfB == hCapSurfV1)) continue;
            if(nc && skipCapV2Curves && (nc->surfA == hCapSurfV2 || nc->surfB == hCapSurfV2)) continue;

            if(stb_n.start.Equals(V1_orig)) {
                if(InsertPointIntoCurvePts(nc, D)) { TruncateCurveAtVertex(nc, V1_orig, D); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1_orig, D); stb_n.start = D; }
            } else if(stb_n.start.Equals(V2_orig)) {
                if(InsertPointIntoCurvePts(nc, C)) { TruncateCurveAtVertex(nc, V2_orig, C); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2_orig, C); stb_n.start = C; }
            }

            if(stb_n.finish.Equals(V1_orig)) {
                if(InsertPointIntoCurvePts(nc, D)) { TruncateCurveAtVertex(nc, V1_orig, D); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1_orig, D); stb_n.finish = D; }
            } else if(stb_n.finish.Equals(V2_orig)) {
                if(InsertPointIntoCurvePts(nc, C)) { TruncateCurveAtVertex(nc, V2_orig, C); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2_orig, C); stb_n.finish = C; }
            }
        }
    }

    // Step 15: update cap surface trims at V1 and V2
    // The cap surfaces (hCapSurfV1, hCapSurfV2) also have trims referencing the same
    // neighbor curves that were modified in Steps 12/13. We must update their STrimBy
    // start/finish endpoints and add the cap curve as a new trim entry.
    if(hCapSurfV1 != hChamfer) {
        SSurface *capSurf1 = surface.FindById(hCapSurfV1);
        bool capSurf1IsFlat = (capSurf1->degm == 1 && capSurf1->degn == 1);
        if(capSurf1IsFlat && !skipCapV1Curves) {
        for(STrimBy &stb_c : capSurf1->trim) {
            SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
            if(!nc) continue;
            bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
            bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
            if(stb_c.start.Equals(V1_orig)) {
                if(bordersSurf1) { if(InsertPointIntoCurvePts(nc, A)) { TruncateCurveAtVertex(nc, V1_orig, A); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1_orig, A); stb_c.start = A; } }
                else if(bordersSurf2) { if(InsertPointIntoCurvePts(nc, D)) { TruncateCurveAtVertex(nc, V1_orig, D); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1_orig, D); stb_c.start = D; } }
            }
            if(stb_c.finish.Equals(V1_orig)) {
                if(bordersSurf1) { if(InsertPointIntoCurvePts(nc, A)) { TruncateCurveAtVertex(nc, V1_orig, A); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1_orig, A); stb_c.finish = A; } }
                else if(bordersSurf2) { if(InsertPointIntoCurvePts(nc, D)) { TruncateCurveAtVertex(nc, V1_orig, D); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1_orig, D); stb_c.finish = D; } }
            }
        }
        // Use graph traversal to find the actual gap endpoint in capSurf1.
        // Sequential scan is unreliable when trims are stored out of connectivity
        // order (insertion order from extrusion or prior chamfer).
        bool capV1Backwards = false;
        bool capV1GapBridgeable = false;
        int  capV1GapIdx = -1;
        {
            int n = capSurf1->trim.n;
            std::vector<bool> vis(n, false);
            vis[0] = true;
            Vector gStartPt = capSurf1->trim[0].start;
            Vector gCur     = capSurf1->trim[0].finish;
            int    gLast    = 0;
            bool   gStuck   = false;
            for(int iter = 1; iter < n; iter++) {
                bool found = false;
                for(int j = 0; j < n; j++) {
                    if(vis[j]) continue;
                    if(gCur.Equals(capSurf1->trim[j].start)) {
                        gCur = capSurf1->trim[j].finish;
                        gLast = j; vis[j] = true; found = true; break;
                    }
                }
                if(!found) { gStuck = true; break; }
            }
            bool gClosed = (!gStuck && gCur.Equals(gStartPt));
            if(!gClosed) {
                // gCur = chain endpoint (gapEnd); gLast = last visited index
                if(gCur.Equals(A))      { capV1Backwards = false; capV1GapBridgeable = true; capV1GapIdx = gLast; }
                else if(gCur.Equals(D)) { capV1Backwards = true;  capV1GapBridgeable = true; capV1GapIdx = gLast; }
            }
        }
        if(capV1GapBridgeable) {
            STrimBy stbCap1 = STrimBy::EntireCurve(this, hCapV1, capV1Backwards);
            InsertTrimAt(capSurf1, capV1GapIdx, &stbCap1);
        }
        }
    }
    if(hCapSurfV2 != hChamfer) {
        SSurface *capSurf2 = surface.FindById(hCapSurfV2);
        bool capSurf2IsFlat = (capSurf2->degm == 1 && capSurf2->degn == 1);
        if(capSurf2IsFlat && !skipCapV2Curves) {
        for(STrimBy &stb_c : capSurf2->trim) {
            SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
            if(!nc) continue;
            bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
            bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
            if(stb_c.start.Equals(V2_orig)) {
                if(bordersSurf1) { if(InsertPointIntoCurvePts(nc, B)) { TruncateCurveAtVertex(nc, V2_orig, B); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2_orig, B); stb_c.start = B; } }
                else if(bordersSurf2) { if(InsertPointIntoCurvePts(nc, C)) { TruncateCurveAtVertex(nc, V2_orig, C); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2_orig, C); stb_c.start = C; } }
            }
            if(stb_c.finish.Equals(V2_orig)) {
                if(bordersSurf1) { if(InsertPointIntoCurvePts(nc, B)) { TruncateCurveAtVertex(nc, V2_orig, B); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2_orig, B); stb_c.finish = B; } }
                else if(bordersSurf2) { if(InsertPointIntoCurvePts(nc, C)) { TruncateCurveAtVertex(nc, V2_orig, C); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2_orig, C); stb_c.finish = C; } }
            }
        }
        // Use graph traversal to find the actual gap endpoint in capSurf2.
        // Sequential scan is unreliable when trims are stored out of connectivity order.
        bool capV2Backwards = false;
        bool capV2GapBridgeable = false;
        int  capV2GapIdx = -1;
        {
            int n = capSurf2->trim.n;
            std::vector<bool> vis(n, false);
            vis[0] = true;
            Vector gStartPt = capSurf2->trim[0].start;
            Vector gCur     = capSurf2->trim[0].finish;
            int    gLast    = 0;
            bool   gStuck   = false;
            for(int iter = 1; iter < n; iter++) {
                bool found = false;
                for(int j = 0; j < n; j++) {
                    if(vis[j]) continue;
                    if(gCur.Equals(capSurf2->trim[j].start)) {
                        gCur = capSurf2->trim[j].finish;
                        gLast = j; vis[j] = true; found = true; break;
                    }
                }
                if(!found) { gStuck = true; break; }
            }
            bool gClosed = (!gStuck && gCur.Equals(gStartPt));
            if(!gClosed) {
                if(gCur.Equals(B))      { capV2Backwards = false; capV2GapBridgeable = true; capV2GapIdx = gLast; }
                else if(gCur.Equals(C)) { capV2Backwards = true;  capV2GapBridgeable = true; capV2GapIdx = gLast; }
            }
        }
        if(capV2GapBridgeable) {
            STrimBy stbCap2 = STrimBy::EntireCurve(this, hCapV2, capV2Backwards);
            InsertTrimAt(capSurf2, capV2GapIdx, &stbCap2);
        }
        }
    }

    // Bridge any remaining trim gaps in the surfaces we modified.
    // These arise when InsertPointIntoCurvePts failed for a prior-chamfer cap edge
    // (e.g., a diagonal curve from a previous chamfer), leaving an open trim loop.
    // We use hChamfer as surfA so these bridge curves are consistently attributed.
    // BridgeTrimGapIfOpen now verifies graph connectivity before bridging,
    // so it is safe to call unconditionally on all potentially modified surfaces.
    {
        hSSurface toCheck[4] = { hSurf1, hSurf2, hCapSurfV1, hCapSurfV2 };
        bool seen[4] = { false, false, false, false };
        for(int ti = 0; ti < 4; ti++) {
            if(toCheck[ti] == hChamfer) continue;
            bool dup = false;
            for(int tj = 0; tj < ti; tj++) { if(toCheck[tj] == toCheck[ti]) { dup = true; break; } }
            if(dup) { seen[ti] = true; continue; }
            seen[ti] = true;
            // Skip bridging for flat cap surfaces (not hSurf1/hSurf2) that share
            // curves passing through V1/V2 with hSurf1/hSurf2 where the setback
            // point can't be inserted. Such surfaces are prior chamfers that
            // would get a DUPLICATE bridge with identical geometry, causing
            // A0==B0 degeneracy in corner post-processing.
            // NOTE: We check curve pts directly (not STrimBy start/finish) because
            // V1/V2 may be intermediate points inserted by a prior chamfer.
            if(toCheck[ti] != hSurf1 && toCheck[ti] != hSurf2) {
                SSurface *capChk = surface.FindById(toCheck[ti]);
                if(capChk->degm == 1 && capChk->degn == 1) {
                    bool sharesDiag = false;
                    for(STrimBy &stb_chk : capChk->trim) {
                        SCurve *nc_chk = curve.FindByIdNoOops(stb_chk.curve);
                        if(!nc_chk) continue;
                        bool bS1 = (nc_chk->surfA == hSurf1 || nc_chk->surfB == hSurf1);
                        bool bS2 = (nc_chk->surfA == hSurf2 || nc_chk->surfB == hSurf2);
                        if(!bS1 && !bS2) continue;
                        // Check if curve passes through V1 or V2 via its pts array
                        if(CurvePtsContain(nc_chk, V1_orig)) {
                            Vector pt = bS1 ? A : D;
                            if(!CanInsertPointIntoCurvePts(nc_chk, pt)) { sharesDiag = true; break; }
                        }
                        if(CurvePtsContain(nc_chk, V2_orig)) {
                            Vector pt = bS1 ? B : C;
                            if(!CanInsertPointIntoCurvePts(nc_chk, pt)) { sharesDiag = true; break; }
                        }
                    }
                    if(sharesDiag) continue;
                }
            }
            BridgeTrimGapIfOpen(this, toCheck[ti], hChamfer);
        }
        (void)seen;
    }

    // Post-process: when BridgeTrimGapIfOpen created bridges for a prior chamfer flat surface,
    // the corner vertex V1 used as a bridge endpoint may not lie on hChamfer's surface,
    // causing wrong UV projections.  Fix: if two bridges share a common corner vertex V1
    // (pTo of one bridge == pFrom of another), create a flat corner triangle surface
    // (A0-V1-B0) that correctly borders the two side surfaces and hChamfer via
    // properly-surfaced curves (NC1, NC2, NC3).
    {
        // Collect bridges: linear 2-pt curves with surfA==hChamfer, not yet in
        // hChamfer's trim, but present in some other surface's trim.
        struct BridgeInfo {
            hSCurve   h;
            Vector    pFrom;   // pts[0] = gapEnd
            Vector    pTo;     // pts[1] = gapStart
            hSSurface otherH;  // surface that owns this bridge in its trim
        };
        std::vector<BridgeInfo> bridges;
        for(SCurve &sc_br : curve) {
            if(sc_br.surfA != hChamfer) continue;
            if(sc_br.pts.n != 2) continue;
            SSurface *hChamferSurf0 = surface.FindById(hChamfer);
            bool inChamfer = false;
            for(int k = 0; k < hChamferSurf0->trim.n; k++) {
                if(hChamferSurf0->trim[k].curve == sc_br.h) { inChamfer = true; break; }
            }
            if(inChamfer) continue;
            hSSurface otherH = {};
            for(SSurface &ss_br : surface) {
                if(ss_br.h == hChamfer) continue;
                for(int k = 0; k < ss_br.trim.n; k++) {
                    if(ss_br.trim[k].curve == sc_br.h) { otherH = ss_br.h; break; }
                }
                if(otherH.v != 0) break;
            }
            if(otherH.v == 0) continue;
            BridgeInfo bi = {};
            bi.h      = sc_br.h;
            bi.pFrom  = sc_br.pts[0].p;
            bi.pTo    = sc_br.pts[1].p;
            bi.otherH = otherH;
        bridges.push_back(bi);
    }



    // Look for a pair of bridges sharing a corner vertex V1.
        // Two bridge-connection patterns are handled:
        //   (a) Fillet-style: bridge bB ends AT V1 (pTo=V1), bridge bA starts FROM V1 (pFrom=V1)
        //       → corners: V1 = bridges[bB].pTo = bridges[bA].pFrom
        //   (b) Chamfer-style: BOTH bridges start FROM V1 (pFrom=V1)
        //       → corners: V1 = bridges[i].pFrom = bridges[j].pFrom
        //   In case (b) the bridge[bB]→NC1 replacement uses NC1 reversed, since bridge[bB]
        //   goes V1→A0 (pFrom=V1, pTo=A0) while NC1 goes A0→V1.
        bool cornerHandled = false;
        retry_pair_matching:
        for(int i = 0; i < (int)bridges.size() && i < 8 && !cornerHandled; i++) {
            for(int j = i+1; j < (int)bridges.size() && j < 8 && !cornerHandled; j++) {
                int bA = -1, bB = -1;
                bool bFromCorner = false;
                bool bToCorner = false;
                if(bridges[i].pTo.Equals(bridges[j].pFrom)) {
                    bB = i; bA = j;
                } else if(bridges[j].pTo.Equals(bridges[i].pFrom)) {
                    bB = j; bA = i;
                } else if(bridges[i].pFrom.Equals(bridges[j].pFrom)) {
                    // Chamfer case: both bridges emanate FROM the same corner V1.
                    bFromCorner = true; bB = i; bA = j;
                } else if(bridges[i].pTo.Equals(bridges[j].pTo)) {
                    // Mixed case: both bridges end AT the same corner V1.
                    bToCorner = true; bB = i; bA = j;
                }
                if(bA < 0) continue;


                cornerHandled = true;
                // For bFromCorner: cornerV1=pFrom (shared), A0=bB.pTo, B0=bA.pTo.
                // For bToCorner:   cornerV1=pTo (shared),   A0=bB.pFrom, B0=bA.pFrom.
                // For fillet-style: cornerV1=bB.pTo, A0=bB.pFrom, B0=bA.pTo.
                Vector cornerV1   = bFromCorner ? bridges[bB].pFrom :
                                    bToCorner   ? bridges[bB].pTo   : bridges[bB].pTo;
                Vector A0         = bFromCorner ? bridges[bB].pTo   :
                                    bToCorner   ? bridges[bB].pFrom : bridges[bB].pFrom;
                Vector B0         = bToCorner   ? bridges[bA].pFrom : bridges[bA].pTo;
                hSSurface hSurfA0 = bridges[bB].otherH;
                hSSurface hSurfB0 = bridges[bA].otherH;


                // -------------------------------------------------------
                // Correct B0 when it's an intermediate trim point rather
                // than a true corner vertex.  This happens after the
                // single-bridge handler creates a 2nd bridge via
                // BridgeTrimGapIfOpen: the new bridge's gap endpoint may
                // not be a corner vertex.  The correct 3rd vertex is an
                // intermediate point in the cap curve that connects A0
                // to B0 (the curve was truncated by prior chamfer ops,
                // turning the original endpoint into an interior point).
                // -------------------------------------------------------
                if((bToCorner || bFromCorner) && !A0.Equals(B0)) {
                    SSurface *chamSurfFix = surface.FindById(hChamfer);
                    // Find the cap curve whose endpoints are A0 and B0.
                    // Its intermediate point (if any) is the correct 3rd
                    // vertex that was truncated by a prior chamfer.
                    for(int ci = 0; ci < chamSurfFix->trim.n; ci++) {
                        SCurve *cc = curve.FindByIdNoOops(chamSurfFix->trim[ci].curve);
                        if(!cc || cc->pts.n < 3) continue;
                        Vector cS = cc->pts[0].p, cE = cc->pts[cc->pts.n-1].p;
                        bool match = (cS.Equals(A0) && cE.Equals(B0)) ||
                                     (cS.Equals(B0) && cE.Equals(A0));
                        if(!match) continue;
                        // Found the A0↔B0 cap curve. Look for an intermediate
                        // point that isn't A0, B0, or cornerV1.
                        for(int pi = 1; pi < cc->pts.n - 1; pi++) {
                            Vector mid = cc->pts[pi].p;
                            if(!mid.Equals(A0) && !mid.Equals(B0) && !mid.Equals(cornerV1)) {
                                B0 = mid;
                                break;
                            }
                        }
                        break;
                    }
                }
                // -------------------------------------------------------
                // Degenerate corner: A0 == B0 (triple-chamfer case).
                // Both bridges go cornerV1 → A0 (= B0). A corner triangle
                // would be zero-area with a degenerate NC3 edge. Instead,
                // merge the two bridges into a single shared curve from
                // cornerV1 to A0, properly shared between hSurfA0 and
                // hSurfB0.
                // -------------------------------------------------------
                if(A0.Equals(B0)) {
                    hSCurve hMerged = AddLinearCurve(this, cornerV1, A0, hSurfA0, hSurfB0);

                    SSurface *ssA0 = surface.FindById(hSurfA0);
                    for(int ti = 0; ti < ssA0->trim.n; ti++) {
                        if(ssA0->trim[ti].curve == bridges[bB].h) {
                            ssA0->trim[ti] = STrimBy::EntireCurve(this, hMerged, false);
                            break;
                        }
                    }
                    SSurface *ssB0 = surface.FindById(hSurfB0);
                    for(int ti = 0; ti < ssB0->trim.n; ti++) {
                        if(ssB0->trim[ti].curve == bridges[bA].h) {
                            ssB0->trim[ti] = STrimBy::EntireCurve(this, hMerged, false);
                            break;
                        }
                    }

                } else
                // -------------------------------------------------------
                // Corner surface synthesis: create flat triangular surface
                // at V1/A0/B0 junction to fill the topological gap between
                // the two adjacent chamfer caps and the chamfer surface.
                // For fillet-style corners, flipTriangleNormals is set so
                // that the EffectiveNormal() returns the outward direction,
                // eliminating the flat -Z display artifact.
                // -------------------------------------------------------
                {
                    SSurface cornerSurf = SSurface::FromPlane(
                        cornerV1,
                        A0.Minus(cornerV1),
                        B0.Minus(cornerV1));
                    if(bFromCorner) cornerSurf = SSurface::FromPlane(
                        cornerV1, B0.Minus(cornerV1), A0.Minus(cornerV1));
                    cornerSurf.color = surface.FindById(hChamfer)->color;
                    cornerSurf.face = faceH.v;
                    // The ear-clipping triangulation produces a Normal()
                    // that points INWARD for both default and bFromCorner
                    // cases. Setting flipTriangleNormals makes
                    // EffectiveNormal() point OUTWARD (away from solid).
                    // bToCorner already has correct outward normal.
                    if(!bToCorner)
                        cornerSurf.flipTriangleNormals = true;
                    cornerSurf.excludeFromDisplay = false;
                    hSSurface hCorner = surface.AddAndAssignId(&cornerSurf);
                    surface.FindById(hChamfer);

                    hSCurve hNC1 = AddLinearCurve(this, A0, cornerV1, hCorner, hSurfA0);
                    hSCurve hNC2 = AddLinearCurve(this, cornerV1, B0, hCorner, hSurfB0);
                    hSCurve hNC3 = AddLinearCurve(this, A0, B0, hCorner, hChamfer);

                    SSurface *ssA0 = surface.FindById(hSurfA0);

                    {
                        for(STrimBy &stb_fix : ssA0->trim) {
                            SCurve *nc_fix = curve.FindByIdNoOops(stb_fix.curve);
                            if(!nc_fix) continue;
                            if(nc_fix->surfA != hCapSurfV1 && nc_fix->surfB != hCapSurfV1) continue;
                            InsertPointIntoCurvePts(nc_fix, A0);
                        }
                        SSurface *ssB0_fix = surface.FindById(hSurfB0);
                        for(STrimBy &stb_fix : ssB0_fix->trim) {
                            SCurve *nc_fix = curve.FindByIdNoOops(stb_fix.curve);
                            if(!nc_fix) continue;
                            if(nc_fix->surfA != hCapSurfV1 && nc_fix->surfB != hCapSurfV1) continue;
                            InsertPointIntoCurvePts(nc_fix, B0);
                        }
                    }
                    {
                        SSurface *ssA0_v2 = surface.FindById(hSurfA0);
                        for(STrimBy &stb_fix : ssA0_v2->trim) {
                            SCurve *nc_fix = curve.FindByIdNoOops(stb_fix.curve);
                            if(!nc_fix) continue;
                            if(nc_fix->surfA != hCapSurfV2 && nc_fix->surfB != hCapSurfV2) continue;
                            InsertPointIntoCurvePts(nc_fix, A0);
                        }
                        SSurface *ssB0_v2 = surface.FindById(hSurfB0);
                        for(STrimBy &stb_fix : ssB0_v2->trim) {
                            SCurve *nc_fix = curve.FindByIdNoOops(stb_fix.curve);
                            if(!nc_fix) continue;
                            if(nc_fix->surfA != hCapSurfV2 && nc_fix->surfB != hCapSurfV2) continue;
                            InsertPointIntoCurvePts(nc_fix, B0);
                        }
                    }

                    for(int ti = 0; ti < ssA0->trim.n; ti++) {
                        if(ssA0->trim[ti].curve == bridges[bB].h) {
                            ssA0->trim[ti] = STrimBy::EntireCurve(this, hNC1, bFromCorner); break;
                        }
                    }
                    SSurface *ssB0 = surface.FindById(hSurfB0);
                    for(int ti = 0; ti < ssB0->trim.n; ti++) {
                        if(ssB0->trim[ti].curve == bridges[bA].h) {
                            ssB0->trim[ti] = STrimBy::EntireCurve(this, hNC2, bToCorner); break;
                        }
                    }

                    SSurface *hChamferSurf = surface.FindById(hChamfer);
                    bool capEdgeFound = false;
                    for(int ai = 0; ai < hChamferSurf->trim.n; ai++) {
                        if(hChamferSurf->trim[ai].curve != hCapV1 &&
                           hChamferSurf->trim[ai].curve != hCapV2) continue;
                        SCurve *ac = curve.FindByIdNoOops(hChamferSurf->trim[ai].curve);
                        if(!ac || ac->pts.n < 2) continue;
                        if(ac->pts[0].p.Equals(A0) || ac->pts[ac->pts.n-1].p.Equals(A0)) {
                            bool trimBkwd = hChamferSurf->trim[ai].backwards;
                            int npts = ac->pts.n;
                            Vector effectiveStart = trimBkwd ? ac->pts[npts-1].p : ac->pts[0].p;
                            bool nc3Backwards = !effectiveStart.Equals(A0);
                            hChamferSurf->trim[ai] = STrimBy::EntireCurve(this, hNC3, nc3Backwards);
                            capEdgeFound = true;
                            break;
                        }
                    }
                    // Fallback: if hCapV1/hCapV2 handles are stale
                    // (replaced by prior chamfer ops), search ALL
                    // trims on hChamfer for a short curve with A0
                    // as an endpoint. This handles bToCorner cases
                    // where permutation _102/_201 causes cap curve
                    // handles to change.
                    if(!capEdgeFound) {
                        hChamferSurf = surface.FindById(hChamfer);
                        for(int ai = 0; ai < hChamferSurf->trim.n; ai++) {
                            SCurve *ac = curve.FindByIdNoOops(hChamferSurf->trim[ai].curve);
                            if(!ac || ac->pts.n < 2) continue;
                            double capLen = ac->pts[0].p.Minus(ac->pts[ac->pts.n-1].p).Magnitude();
                            if(capLen > 3.0 * dist) continue; // skip long edges
                            if(ac->pts[0].p.Equals(A0) || ac->pts[ac->pts.n-1].p.Equals(A0)) {
                                bool trimBkwd = hChamferSurf->trim[ai].backwards;
                                int npts = ac->pts.n;
                                Vector effectiveStart = trimBkwd ? ac->pts[npts-1].p : ac->pts[0].p;
                                bool nc3Backwards = !effectiveStart.Equals(A0);
                                hChamferSurf->trim[ai] = STrimBy::EntireCurve(this, hNC3, nc3Backwards);
                                capEdgeFound = true;
                                break;
                            }
                        }
                    }
                    (void)capEdgeFound; // suppress unused warning

                    SSurface *hCornerSurf = surface.FindById(hCorner);
                    STrimBy  stb;
                    stb = STrimBy::EntireCurve(this, hNC2, false);
                    hCornerSurf->trim.Add(&stb);
                    hCornerSurf = surface.FindById(hCorner);
                    stb = STrimBy::EntireCurve(this, hNC3, true);
                    hCornerSurf->trim.Add(&stb);
                    hCornerSurf = surface.FindById(hCorner);
                    stb = STrimBy::EntireCurve(this, hNC1, false);
                    hCornerSurf->trim.Add(&stb);
                }

                // -------------------------------------------------------
                // Midpoint insertion into cap surface long edges.
                // The prior chamfer strip (hCapSurfV1) can have very long
                // trim edges (e.g. 78 units on a 20×20×80 box). The ear-
                // clipping triangulator creates mesh diagonals spanning
                // the full length, which cross adjacent chamfer surfaces
                // at interior points → self-intersections.
                // Fix: insert intermediate vertices at 'dist' units from
                // the far end of each long edge. This ensures ear-clipping
                // diagonals to those vertices cross adjacent surfaces at
                // endpoints only (t=0 or t=1) → NOT flagged.
                // -------------------------------------------------------
                if(bFromCorner) {
                    SSurface *ssCap = surface.FindById(hCapSurfV1);
                    if(ssCap->degm == 1 && ssCap->degn == 1 &&
                       hCapSurfV1 != hSurf1 && hCapSurfV1 != hSurf2)
                    {
                        for(int ki = 0; ki < ssCap->trim.n; ki++) {
                            STrimBy &stb_k = ssCap->trim[ki];
                            double edgeLen_k = stb_k.start.Minus(stb_k.finish).Magnitude();
                            if(edgeLen_k < 10.0) continue;
                            SCurve *sc_k = curve.FindByIdNoOops(stb_k.curve);
                            if(!sc_k) continue;
                            // Find endpoint farthest from cornerV1
                            double d1_k = stb_k.start.Minus(cornerV1).Magnitude();
                            double d2_k = stb_k.finish.Minus(cornerV1).Magnitude();
                            Vector farEnd = (d1_k > d2_k) ? stb_k.start : stb_k.finish;
                            Vector nearEnd = (d1_k > d2_k) ? stb_k.finish : stb_k.start;
                            Vector dir_k = nearEnd.Minus(farEnd).WithMagnitude(1.0);
                            Vector midpt = farEnd.Plus(dir_k.ScaledBy(dist));
                            InsertPointIntoCurvePts(sc_k, midpt);
                            // Re-fetch ssCap after potential reallocation
                            ssCap = surface.FindById(hCapSurfV1);
                        }
                    }
                }
                // Also insert midpoints into hCapSurfV2 long edges
                // (for triple-chamfer, the V2 cap surface also has very
                // long edges that cause ear-clipping self-intersections).
                if(bFromCorner && hCapSurfV2 != hCapSurfV1) {
                    SSurface *ssCap2 = surface.FindById(hCapSurfV2);
                    if(ssCap2->degm == 1 && ssCap2->degn == 1 &&
                       hCapSurfV2 != hSurf1 && hCapSurfV2 != hSurf2)
                    {
                        for(int ki = 0; ki < ssCap2->trim.n; ki++) {
                            STrimBy &stb_k = ssCap2->trim[ki];
                            double edgeLen_k = stb_k.start.Minus(stb_k.finish).Magnitude();
                            if(edgeLen_k < 10.0) continue;
                            SCurve *sc_k = curve.FindByIdNoOops(stb_k.curve);
                            if(!sc_k) continue;
                            double d1_k = stb_k.start.Minus(cornerV1).Magnitude();
                            double d2_k = stb_k.finish.Minus(cornerV1).Magnitude();
                            Vector farEnd = (d1_k > d2_k) ? stb_k.start : stb_k.finish;
                            Vector nearEnd = (d1_k > d2_k) ? stb_k.finish : stb_k.start;
                            Vector dir_k = nearEnd.Minus(farEnd).WithMagnitude(1.0);
                            Vector midpt = farEnd.Plus(dir_k.ScaledBy(dist));
                            InsertPointIntoCurvePts(sc_k, midpt);
                            ssCap2 = surface.FindById(hCapSurfV2);
                        }
                    }
                }

            }
        }
        if(!cornerHandled && !bridges.empty()) {
            // -------------------------------------------------------
            // Single-bridge degenerate corner (triple-chamfer case).
            // Only 1 bridge was found for hChamfer. The pair-matching
            // loop above requires 2 bridges and couldn't match.
            // Fix: find a second surface with a trim gap near the
            // bridge endpoints, create a merged curve shared between
            // the bridge's otherH and that second surface, and close
            // the gap.
            //
            // This handles permutations _102/_201 where
            // BridgeTrimGapIfOpen only created 1 bridge for hChamfer
            // because the second surface's chain was found "closed"
            // by the chain-finding algorithm, yet its trim polygon
            // assembly fails in UV space.
            // -------------------------------------------------------
            if(bridges.size() == 1) {
                Vector P1 = bridges[0].pFrom;
                Vector P2 = bridges[0].pTo;
                hSSurface hSurf1 = bridges[0].otherH;
                hSCurve hBridgeCurve = bridges[0].h;

                // Find a second surface that has a trim gap with at
                // least one endpoint matching P1 or P2.
                hSSurface hSurf2 = {};
                Vector gapEnd2 = {}, gapStart2 = {};
                int gapIdx2 = -1;
                for(SSurface &ss_scan : surface) {
                    if(ss_scan.h == hChamfer) continue;
                    if(ss_scan.h == hSurf1) continue;
                    Vector gE = {}, gS = {};
                    int gi = FindTrimGap(&ss_scan, &gE, &gS);
                    if(gi < 0) continue;
                    // Check if either gap endpoint matches a bridge endpoint
                    bool match = gE.Equals(P1) || gE.Equals(P2) ||
                                 gS.Equals(P1) || gS.Equals(P2);
                    if(match) {
                        hSurf2 = ss_scan.h;
                        gapEnd2 = gE;
                        gapStart2 = gS;
                        gapIdx2 = gi;
                        break;
                    }
                }

                if(hSurf2.v != 0) {
                    // Fix hSurf2's trim polygon by bridging its gap
                    // with surfA=hChamfer so the new bridge matches
                    // the same surface attribution as the existing
                    // bridge in hSurf1. BridgeTrimGapIfOpen removes
                    // orphaned trims and creates a proper bridge.
                    BridgeTrimGapIfOpen(this, hSurf2, hChamfer);

                    // Now re-collect the second bridge: a 2-pt curve
                    // with surfA=hChamfer, in hSurf2's trim, that is
                    // NOT the original bridge.
                    BridgeInfo bi2 = {};
                    for(SCurve &sc_br2 : curve) {
                        if(sc_br2.surfA != hChamfer) continue;
                        if(sc_br2.pts.n != 2) continue;
                        if(sc_br2.h == hBridgeCurve) continue;
                        SSurface *ss2chk = surface.FindById(hSurf2);
                        bool inSurf2 = false;
                        for(int k = 0; k < ss2chk->trim.n; k++) {
                            if(ss2chk->trim[k].curve == sc_br2.h) {
                                inSurf2 = true; break;
                            }
                        }
                        if(!inSurf2) continue;
                        bi2.h = sc_br2.h;
                        bi2.pFrom = sc_br2.pts[0].p;
                        bi2.pTo = sc_br2.pts[1].p;
                        bi2.otherH = hSurf2;
                        break;
                    }

                    if(bi2.h.v != 0) {
                        bridges.push_back(bi2);
                        // Retry pair matching now that we have 2 bridges
                        goto retry_pair_matching;
                    } else {
                        // BridgeTrimGapIfOpen didn't create a bridge
                        // (polygon was already closed). Fall through.
                        cornerHandled = true;
                    }
                } else {
                }
            } else {
            }
        }
    }

    // Trim closure enforcement: repair any open trim loops after corner synthesis
    RepairOpenTrimLoops(this, "post-corner-synthesis");

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
    // Use a small negative tolerance to handle floating-point rounding: for a
    // perfect 90° edge, cosAngle should be 0 but can round to ~-5.5e-17.
    if(cosAngle < -LENGTH_EPS) {
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


    // Orientation normalization: ensure fillet surface is consistently oriented
    // regardless of which face was selected as entityB vs entityC.
    // The fillet arc goes from A0 (on surf1) through the edge to B0 (on surf2).
    // We want t.Cross(d2-d1) to align with n1+n2 (outward bisector).
    // If not, swap surf1/surf2 roles.
    {
        Vector expectedNormal = t.Cross(d2.Minus(d1));
        if(expectedNormal.Dot(n1.Plus(n2)) > 0) {
            std::swap(hSurf1, hSurf2);
            std::swap(d1, d2);
            std::swap(n1, n2);
            surf1 = surface.FindById(hSurf1);
            surf2 = surface.FindById(hSurf2);
        }
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
    // Score = |normal.Dot(t)| * 2 + (trim.n==0 ? 1 : 0): highest = best cap.
    // Tiebreaker: prefer trim.n==0 (DIFF internal surface needing RECON).
    double capV1Score = -1.0;
    double capV2Score = -1.0;
    for(SCurve &sc_scan : curve) {
        if(sc_scan.h == hSharedSC) continue;
        if(sc_scan.pts.n < 2) continue;
        Vector first = sc_scan.pts[0].p;
        Vector last = sc_scan.pts[sc_scan.pts.n - 1].p;
        bool touchesV1 = first.Equals(V1) || last.Equals(V1);
        bool touchesV2 = first.Equals(V2) || last.Equals(V2);
        if(!touchesV1 && !touchesV2) continue;
        for(int side = 0; side < 2; side++) {
            hSSurface hCand = (side == 0) ? sc_scan.surfA : sc_scan.surfB;
            if(hCand.v == 0) continue;
            if(hCand == hSurf1 || hCand == hSurf2) continue;
            SSurface *sCand = surface.FindById(hCand);
            Point2d uvMid; uvMid.x = 0.5; uvMid.y = 0.5;
            Vector nCand = sCand->NormalAt(uvMid);
            // Composite score: geometric alignment (0..2) + trim.n>0 bonus (0 or 1).
            // Prefer external surfaces (trim.n>0) over internal ASSEMBLE faces.
            double geoScore = fabs(nCand.Dot(t));
            double score = geoScore * 2.0 + (sCand->trim.n == 0 ? 1.0 : 0.0);
            if(touchesV1 && score > capV1Score) {
                capV1Score = score;
                hCapSurfV1 = hCand;
            }
            if(touchesV2 && score > capV2Score) {
                capV2Score = score;
                hCapSurfV2 = hCand;
            }
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
                    if(InsertPointIntoCurvePts(nc, A0)) { TruncateCurveAtVertex(nc, V1, A0); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1, A0); stb_n.start = A0; }
                } else if(stb_n.start.Equals(V2)) {
                    if(InsertPointIntoCurvePts(nc, A1)) { TruncateCurveAtVertex(nc, V2, A1); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2, A1); stb_n.start = A1; }
                }
                if(stb_n.finish.Equals(V1)) {
                    if(InsertPointIntoCurvePts(nc, A0)) { TruncateCurveAtVertex(nc, V1, A0); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1, A0); stb_n.finish = A0; }
                } else if(stb_n.finish.Equals(V2)) {
                    if(InsertPointIntoCurvePts(nc, A1)) { TruncateCurveAtVertex(nc, V2, A1); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2, A1); stb_n.finish = A1; }
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
                    if(InsertPointIntoCurvePts(nc, B0)) { TruncateCurveAtVertex(nc, V1, B0); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1, B0); stb_n.start = B0; }
                } else if(stb_n.start.Equals(V2)) {
                    if(InsertPointIntoCurvePts(nc, B1)) { TruncateCurveAtVertex(nc, V2, B1); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2, B1); stb_n.start = B1; }
                }
                if(stb_n.finish.Equals(V1)) {
                    if(InsertPointIntoCurvePts(nc, B0)) { TruncateCurveAtVertex(nc, V1, B0); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1, B0); stb_n.finish = B0; }
                } else if(stb_n.finish.Equals(V2)) {
                    if(InsertPointIntoCurvePts(nc, B1)) { TruncateCurveAtVertex(nc, V2, B1); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2, B1); stb_n.finish = B1; }
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
            bool capSurf1IsFlat = (capSurf1->degm == 1 && capSurf1->degn == 1);
            if(capSurf1IsFlat) {
            for(STrimBy &stb_c : capSurf1->trim) {
                SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
                if(!nc) continue;
            bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
            bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
            if(stb_c.start.Equals(V1)) {
                if(bordersSurf1) { if(InsertPointIntoCurvePts(nc, A0)) { TruncateCurveAtVertex(nc, V1, A0); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1, A0); stb_c.start = A0; } }
                else if(bordersSurf2) { if(InsertPointIntoCurvePts(nc, B0)) { TruncateCurveAtVertex(nc, V1, B0); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1, B0); stb_c.start = B0; } }
            }
            if(stb_c.finish.Equals(V1)) {
                if(bordersSurf1) { if(InsertPointIntoCurvePts(nc, A0)) { TruncateCurveAtVertex(nc, V1, A0); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1, A0); stb_c.finish = A0; } }
                else if(bordersSurf2) { if(InsertPointIntoCurvePts(nc, B0)) { TruncateCurveAtVertex(nc, V1, B0); UpdateAllSurfaceTrimEndpoints(this, nc->h, V1, B0); stb_c.finish = B0; } }
            }
        }
        // ASSEMBLE topology stitch: after the main trim-update pass, check if any
        // trim entry still has V1 as an endpoint (because the edge borders a THIRD
        // surface, not hSurf1/hSurf2 — typical in ASSEMBLE bodies where the cap
        // face shares an edge with a base-solid side face that ends at V1).
        // Without this stitch, the gap traversal gets stuck at V1 (not A0/B0),
        // arcV1GapBridgeable stays false, and BridgeTrimGapIfOpen creates a wrong
        // straight bridge. The stitch adds a short V1->target segment so the
        // arc (A0->V1->B0) can be properly inserted.
        {
            capSurf1 = surface.FindById(hCapSurfV1);
            bool stitchAdded = false;
            for(int ti = 0; ti < capSurf1->trim.n && !stitchAdded; ti++) {
                STrimBy &stb_v = capSurf1->trim[ti];
                if(!stb_v.finish.Equals(V1) && !stb_v.start.Equals(V1)) continue;
                SCurve *nc2 = curve.FindByIdNoOops(stb_v.curve);
                if(!nc2 || nc2->pts.n < 2) continue;
                bool b1 = (nc2->surfA == hSurf1 || nc2->surfB == hSurf1);
                bool b2 = (nc2->surfA == hSurf2 || nc2->surfB == hSurf2);
                if(b1 || b2) continue; // already handled in main pass
                // Determine stitch target (B0 or A0) based on edge direction at V1.
                // The edge that arrives at V1 from the d2 (surf2) direction should
                // stitch to B0; from d1 (surf1) direction to A0.
                Vector stitchTarget = B0; // default
                if(stb_v.finish.Equals(V1)) {
                    // Edge arrives at V1: direction = last_pt - second_to_last_pt
                    Vector edgeDir = nc2->pts[nc2->pts.n-1].p.Minus(
                                     nc2->pts[nc2->pts.n-2].p).WithMagnitude(1);
                    // For an ARRIVING edge, the correct setback point lies BEHIND V1
                    // (between edge.start and V1), i.e. in the REVERSE edge direction.
                    // Use -edgeDir to find the point in the backward direction from V1.
                    double b0s = edgeDir.ScaledBy(-1.0).Dot((B0.Minus(V1)).WithMagnitude(1));
                    double a0s = edgeDir.ScaledBy(-1.0).Dot((A0.Minus(V1)).WithMagnitude(1));
                    if(a0s > b0s) stitchTarget = A0;
                } else {
                    // Edge leaves V1: direction = pts[1] - pts[0]
                    Vector edgeDir = nc2->pts[1].p.Minus(nc2->pts[0].p).WithMagnitude(1);
                    double b0s = edgeDir.Dot((B0.Minus(V1)).WithMagnitude(1));
                    double a0s = edgeDir.Dot((A0.Minus(V1)).WithMagnitude(1));
                    if(a0s > b0s) stitchTarget = A0;
                }
                // Truncate the shared SCurve at V1→stitchTarget using the same
                // TruncateCurveAtVertex + UpdateAllSurfaceTrimEndpoints approach
                // used by the main trim-update loop. This ensures ALL surfaces that
                // reference nc2 (including box1's side face in ASSEMBLE geometry)
                // get their V1 endpoint updated to stitchTarget — fixing broken trim
                // boundaries that would otherwise cause inverted/backfacing triangles.
                if(stb_v.finish.Equals(V1)) {
                    // Trim arrives at V1: truncate curve stb_v.start→V1 to stb_v.start→stitchTarget.
                    if(InsertPointIntoCurvePts(nc2, stitchTarget)) {
                        TruncateCurveAtVertex(nc2, V1, stitchTarget);
                        UpdateAllSurfaceTrimEndpoints(this, nc2->h, V1, stitchTarget);
                        capSurf1 = surface.FindById(hCapSurfV1);
                    }
                    else {
                        // stitchTarget not on curve (e.g. A0 is orthogonal to edge).
                        // Fall back to AddLinearCurve to replace the trim endpoint.
                        hSCurve hExt = AddLinearCurve(this, stb_v.start, stitchTarget, hFillet, hCapSurfV1);
                        capSurf1 = surface.FindById(hCapSurfV1);
                        capSurf1->trim[ti].curve = hExt;
                        capSurf1->trim[ti].finish = stitchTarget;
                    }
                } else {
                    // Trim leaves V1: truncate curve V1→stb_v.finish to stitchTarget→stb_v.finish.
                    if(InsertPointIntoCurvePts(nc2, stitchTarget)) {
                        TruncateCurveAtVertex(nc2, V1, stitchTarget);
                        UpdateAllSurfaceTrimEndpoints(this, nc2->h, V1, stitchTarget);
                        capSurf1 = surface.FindById(hCapSurfV1);
                    }
                    else {
                        hSCurve hExt = AddLinearCurve(this, stitchTarget, stb_v.finish, hFillet, hCapSurfV1);
                        capSurf1 = surface.FindById(hCapSurfV1);
                        capSurf1->trim[ti].curve = hExt;
                        capSurf1->trim[ti].start = stitchTarget;
                    }
                }
                stitchAdded = true;
            }
        }
        // Use graph traversal to find the actual gap endpoint in capSurf1 for fillet.
        // Sequential scan is unreliable when trims are stored out of connectivity order.
        // For boolean-difference bodies, the cap surface may have trim.n == 0 because
        // UpdateAllSurfaceTrimEndpoints had nothing to update. In that case, reconstruct
        // the trims from SCurves that border hCapSurfV1 (excluding hArcV1).
        if(capSurf1->trim.n == 0) {
            for(SCurve &sc_bd : curve) {
                if(sc_bd.h == hArcV1) continue;
                if(sc_bd.surfA != hCapSurfV1 && sc_bd.surfB != hCapSurfV1) continue;
                if(sc_bd.pts.n < 2) continue;
                bool bkwd = (sc_bd.surfB == hCapSurfV1);
                STrimBy stb_new = {};
                stb_new.curve = sc_bd.h;
                stb_new.backwards = bkwd;
                stb_new.start  = bkwd ? sc_bd.pts[sc_bd.pts.n-1].p : sc_bd.pts[0].p;
                stb_new.finish = bkwd ? sc_bd.pts[0].p : sc_bd.pts[sc_bd.pts.n-1].p;
                capSurf1->trim.Add(&stb_new);
            }
        }
        bool arcV1Backwards = false;
        bool arcV1GapBridgeable = false;
        int  arcV1GapIdx = -1;
        {
            int n = capSurf1->trim.n;
            if(n > 0) {
                std::vector<bool> vis(n, false);
                vis[0] = true;
                Vector gStartPt = capSurf1->trim[0].start;
                Vector gCur     = capSurf1->trim[0].finish;
                int    gLast    = 0;
                bool   gStuck   = false;
                for(int iter = 1; iter < n; iter++) {
                    bool found = false;
                    for(int j = 0; j < n; j++) {
                        if(vis[j]) continue;
                        if(gCur.Equals(capSurf1->trim[j].start)) {
                            gCur = capSurf1->trim[j].finish;
                            gLast = j; vis[j] = true; found = true; break;
                        }
                    }
                    if(!found) { gStuck = true; break; }
                }
           bool gClosed = (!gStuck && gCur.Equals(gStartPt));
              if(!gClosed) {
                   if(gCur.Equals(A0))      { arcV1Backwards = false; arcV1GapBridgeable = true; arcV1GapIdx = gLast; }
                   else if(gCur.Equals(B0)) { arcV1Backwards = true;  arcV1GapBridgeable = true; arcV1GapIdx = gLast; }
              }
          }
      }
        arcV1GapBridgeable = false; // Always use greedy to ensure correct array order
        // Check whether both arc endpoints (A0, B0) exist on capSurf1's trims.
        // When a prior chamfer left a diagonal cap edge, InsertPointIntoCurvePts for
        // B0 may have failed, so B0 is not on any trim of capSurf1. In that case
        // the direct-scan swap and greedy reorder would corrupt the trim polygon.
        // Skip them and let BridgeTrimGapIfOpen + corner post-processing handle the gap.
        bool capSurf1CanTakeArc = false;
        for(int i = 0; i < capSurf1->trim.n; i++) {
            if(capSurf1->trim[i].start.Equals(B0) || capSurf1->trim[i].finish.Equals(B0))
                { capSurf1CanTakeArc = true; break; }
        }
        if(!arcV1GapBridgeable && capSurf1CanTakeArc) {
            for(int i = 0; i < capSurf1->trim.n; i++) {
                if(capSurf1->trim[i].finish.Equals(A0)) {
                    arcV1Backwards = false; arcV1GapBridgeable = true; arcV1GapIdx = i; break;
                }
                if(capSurf1->trim[i].finish.Equals(B0)) {
                    arcV1Backwards = true; arcV1GapBridgeable = true; arcV1GapIdx = i; break;
                }
                if(capSurf1->trim[i].start.Equals(A0)) {
                    std::swap(capSurf1->trim[i].start, capSurf1->trim[i].finish);
                    capSurf1->trim[i].backwards = !capSurf1->trim[i].backwards;
                    arcV1Backwards = false; arcV1GapBridgeable = true; arcV1GapIdx = i; break;
                }
                if(capSurf1->trim[i].start.Equals(B0)) {
                    std::swap(capSurf1->trim[i].start, capSurf1->trim[i].finish);
                    capSurf1->trim[i].backwards = !capSurf1->trim[i].backwards;
                    arcV1Backwards = true; arcV1GapBridgeable = true; arcV1GapIdx = i; break;
                }
            }
            // Direct scan found a trim ending at A0/B0, but InsertTrimAt only places
            // the arc correctly if the next trim in array order starts at the other
            // arc endpoint. Reset bridgeable so the greedy assembly below rebuilds
            // the array in proper connectivity order.
            arcV1GapBridgeable = false;
        }
        // Bidirectional greedy assembly for capSurf1:
        // If graph traversal and direct scan both failed, or direct scan found an index
        // but trim array is not in connectivity order, use the same robust multi-start
        // greedy approach as capSurf2. Try each starting trim until we find a chain
        // ending at A0 or B0, then rebuild trim array in connectivity order so
        // InsertTrimAt places the arc at the correct position.
        if(!arcV1GapBridgeable && capSurf1CanTakeArc) {
            int n1 = capSurf1->trim.n;
            if(n1 > 0) {
                std::vector<bool> used1(n1, false);
                std::vector<int>  pass1Ord1, pass2Ord1;
                bool go1b = false;
                Vector gCur1b;
                bool p1AtA0 = false, p1AtB0 = false;
                for(int startTi = 0; startTi < n1 && !p1AtA0 && !p1AtB0; startTi++) {
                    std::fill(used1.begin(), used1.end(), false);
                    pass1Ord1.clear(); pass2Ord1.clear();
                    used1[startTi] = true;
                    pass1Ord1.push_back(startTi);
                    gCur1b = capSurf1->trim[startTi].finish;
                    go1b = true;
                    while(go1b) {
                        go1b = false;
                        for(int j = 0; j < n1; j++) {
                            if(used1[j]) continue;
                            if(gCur1b.Equals(capSurf1->trim[j].start)) {
                                gCur1b = capSurf1->trim[j].finish;
                                used1[j] = true; pass1Ord1.push_back(j); go1b = true; break;
                            } else if(gCur1b.Equals(capSurf1->trim[j].finish)) {
                                std::swap(capSurf1->trim[j].start, capSurf1->trim[j].finish);
                                capSurf1->trim[j].backwards = !capSurf1->trim[j].backwards;
                                gCur1b = capSurf1->trim[j].finish;
                                used1[j] = true; pass1Ord1.push_back(j); go1b = true; break;
                            }
                        }
                    }
                    p1AtA0 = gCur1b.Equals(A0); p1AtB0 = gCur1b.Equals(B0);
                    if(p1AtA0 || p1AtB0) {
                        Vector arcOther1 = p1AtA0 ? B0 : A0;
                        Vector gCur3b = arcOther1;
                        go1b = true;
                        while(go1b) {
                            go1b = false;
                            for(int j = 0; j < n1; j++) {
                                if(used1[j]) continue;
                                if(gCur3b.Equals(capSurf1->trim[j].start)) {
                                    gCur3b = capSurf1->trim[j].finish;
                                    used1[j] = true; pass2Ord1.push_back(j); go1b = true; break;
                                } else if(gCur3b.Equals(capSurf1->trim[j].finish)) {
                                    std::swap(capSurf1->trim[j].start, capSurf1->trim[j].finish);
                                    capSurf1->trim[j].backwards = !capSurf1->trim[j].backwards;
                                    gCur3b = capSurf1->trim[j].finish;
                                    used1[j] = true; pass2Ord1.push_back(j); go1b = true; break;
                                }
                            }
                        }
                        List<STrimBy> newT1 = {};
                        for(int ci : pass1Ord1) newT1.Add(&capSurf1->trim[ci]);
                        for(int ci : pass2Ord1) newT1.Add(&capSurf1->trim[ci]);
                        for(int j = 0; j < n1; j++) if(!used1[j]) newT1.Add(&capSurf1->trim[j]);
                        capSurf1->trim.Clear();
                        for(int i = 0; i < newT1.n; i++) capSurf1->trim.Add(&newT1[i]);
                        newT1.Clear();
                        arcV1GapIdx        = (int)pass1Ord1.size() - 1;
                        arcV1Backwards     = p1AtB0;
                        arcV1GapBridgeable = true;
                    }
                } // close for(startTi)
            }
        }
       if(arcV1GapBridgeable) {
           STrimBy stbArc1 = STrimBy::EntireCurve(this, hArcV1, arcV1Backwards);
           InsertTrimAt(capSurf1, arcV1GapIdx, &stbArc1);
        }
        }  // closes if(capSurf1IsFlat)
    }
    if(hCapSurfV2 != hFillet) {
        SSurface *capSurf2 = surface.FindById(hCapSurfV2);
        bool capSurf2IsFlat = (capSurf2->degm == 1 && capSurf2->degn == 1);
        if(capSurf2IsFlat) {
        for(STrimBy &stb_c : capSurf2->trim) {
            SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
            if(!nc) continue;
            bool bordersSurf1 = (nc->surfA == hSurf1 || nc->surfB == hSurf1);
            bool bordersSurf2 = (nc->surfA == hSurf2 || nc->surfB == hSurf2);
            if(stb_c.start.Equals(V2)) {
                if(bordersSurf1) { if(InsertPointIntoCurvePts(nc, A1)) { TruncateCurveAtVertex(nc, V2, A1); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2, A1); stb_c.start = A1; } }
                else if(bordersSurf2) { if(InsertPointIntoCurvePts(nc, B1)) { TruncateCurveAtVertex(nc, V2, B1); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2, B1); stb_c.start = B1; } }
            }
            if(stb_c.finish.Equals(V2)) {
                if(bordersSurf1) { if(InsertPointIntoCurvePts(nc, A1)) { TruncateCurveAtVertex(nc, V2, A1); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2, A1); stb_c.finish = A1; } }
                else if(bordersSurf2) { if(InsertPointIntoCurvePts(nc, B1)) { TruncateCurveAtVertex(nc, V2, B1); UpdateAllSurfaceTrimEndpoints(this, nc->h, V2, B1); stb_c.finish = B1; } }
            }
        }
            // Use graph traversal to find the actual gap endpoint in capSurf2 for fillet.
            // Sequential scan is unreliable when trims are stored out of connectivity order.
            // For boolean-difference bodies, the endcap may have trim.n == 0 because
            // UpdateAllSurfaceTrimEndpoints had nothing to update (empty trim list).
            // In that case, reconstruct trims from SCurves bordering hCapSurfV2 (excl. hArcV2).
            bool didReconV2 = false;
            if(capSurf2->trim.n == 0) {
                for(SCurve &sc_bd : curve) {
                    if(sc_bd.h == hArcV2) continue;
                    if(sc_bd.surfA != hCapSurfV2 && sc_bd.surfB != hCapSurfV2) continue;
                    if(sc_bd.pts.n < 2) continue;
                    bool bkwd = (sc_bd.surfB == hCapSurfV2);
                    STrimBy stb_new = {};
                    stb_new.curve = sc_bd.h;
                    stb_new.backwards = bkwd;
                    stb_new.start  = bkwd ? sc_bd.pts[sc_bd.pts.n-1].p : sc_bd.pts[0].p;
                    stb_new.finish = bkwd ? sc_bd.pts[0].p : sc_bd.pts[sc_bd.pts.n-1].p;
                    capSurf2->trim.Add(&stb_new);
                    didReconV2 = true;
                }
            }
            bool arcV2Backwards = false;
            bool arcV2GapBridgeable = false;
            int  arcV2GapIdx = -1;
            {
                int n = capSurf2->trim.n;
                if(n > 0) {
                    std::vector<bool> vis(n, false);
                    vis[0] = true;
                    Vector gStartPt = capSurf2->trim[0].start;
                    Vector gCur     = capSurf2->trim[0].finish;
                    int    gLast    = 0;
                    bool   gStuck   = false;
                    for(int iter = 1; iter < n; iter++) {
                        bool found = false;
                        for(int j = 0; j < n; j++) {
                            if(vis[j]) continue;
                            if(gCur.Equals(capSurf2->trim[j].start)) {
                                gCur = capSurf2->trim[j].finish;
                                gLast = j; vis[j] = true; found = true; break;
                            }
                        }
                        if(!found) { gStuck = true; break; }
                    }
                    bool gClosed = (!gStuck && gCur.Equals(gStartPt));
                    if(!gClosed) {
                        if(gCur.Equals(A1))      { arcV2Backwards = false; arcV2GapBridgeable = true; arcV2GapIdx = gLast; }
                        else if(gCur.Equals(B1)) { arcV2Backwards = true;  arcV2GapBridgeable = true; arcV2GapIdx = gLast; }
                }
            }
        }
            // RECON bidirectional greedy assembly:
            // If traversal failed, some trims may be stored backwards or in wrong
            // order. Use two greedy passes to build a correctly-oriented, contiguous
            // chain: pass1 extends from trim[0].finish toward A1 or B1, flipping
            // individual trims as needed; pass2 extends from the other arc endpoint
            // to orient remaining trims. Rebuilds the trim array in connectivity
            // order so InsertTrimAt places the arc at the correct position.
            if(!arcV2GapBridgeable) {
                int n2 = capSurf2->trim.n;
                if(n2 > 0) {
                    // Try each trim as starting point until pass1 reaches A1 or B1.
                    // This handles surfaces with multiple trim loops (e.g. outer
                    // rectangle + inner hole boundary for a through-hole pocket).
                    std::vector<bool> used2(n2, false);
                    std::vector<int>  pass1Ord, pass2Ord;
                    bool go2 = false;
                    Vector gCur2;
                    bool p1AtA1_pre = false, p1AtB1_pre = false;
                    for(int startTi = 0; startTi < n2 && !p1AtA1_pre && !p1AtB1_pre; startTi++) {
                    std::fill(used2.begin(), used2.end(), false);
                    pass1Ord.clear(); pass2Ord.clear();
                    used2[startTi] = true;
                    pass1Ord.push_back(startTi);
                    gCur2 = capSurf2->trim[startTi].finish;
                    go2 = true;
                    while(go2) {
                        go2 = false;
                        for(int j = 0; j < n2; j++) {
                            if(used2[j]) continue;
                            if(gCur2.Equals(capSurf2->trim[j].start)) {
                                gCur2 = capSurf2->trim[j].finish;
                                used2[j] = true; pass1Ord.push_back(j); go2 = true; break;
                            } else if(gCur2.Equals(capSurf2->trim[j].finish)) {
                                std::swap(capSurf2->trim[j].start, capSurf2->trim[j].finish);
                                capSurf2->trim[j].backwards = !capSurf2->trim[j].backwards;
                                gCur2 = capSurf2->trim[j].finish;
                                used2[j] = true; pass1Ord.push_back(j); go2 = true; break;
                            }
                        }
                    }
                    bool p1AtA1 = gCur2.Equals(A1), p1AtB1 = gCur2.Equals(B1);
                    p1AtA1_pre = p1AtA1; p1AtB1_pre = p1AtB1;
                    if(p1AtA1 || p1AtB1) {
                        // Pass 2: greedy from the other arc endpoint, correctly
                        // orienting remaining trims for the post-arc chain.
                        Vector arcOther = p1AtA1 ? B1 : A1;
                        Vector gCur3 = arcOther;
                        go2 = true;
                        while(go2) {
                            go2 = false;
                            for(int j = 0; j < n2; j++) {
                                if(used2[j]) continue;
                                if(gCur3.Equals(capSurf2->trim[j].start)) {
                                    gCur3 = capSurf2->trim[j].finish;
                                    used2[j] = true; pass2Ord.push_back(j); go2 = true; break;
                                } else if(gCur3.Equals(capSurf2->trim[j].finish)) {
                                    std::swap(capSurf2->trim[j].start, capSurf2->trim[j].finish);
                                    capSurf2->trim[j].backwards = !capSurf2->trim[j].backwards;
                                    gCur3 = capSurf2->trim[j].finish;
                                    used2[j] = true; pass2Ord.push_back(j); go2 = true; break;
                                }
                            }
                        }
                        // Rebuild trim array: [pass1 chain] [pass2 chain] [any remaining]
                        // This ensures connectivity order matches array order for InsertTrimAt.
                        List<STrimBy> newT = {};
                        for(int ci : pass1Ord) newT.Add(&capSurf2->trim[ci]);
                        for(int ci : pass2Ord) newT.Add(&capSurf2->trim[ci]);
                        for(int j = 0; j < n2; j++) if(!used2[j]) newT.Add(&capSurf2->trim[j]);
                        capSurf2->trim.Clear();
                        for(int i = 0; i < newT.n; i++) capSurf2->trim.Add(&newT[i]);
                        newT.Clear();
                        arcV2GapIdx     = (int)pass1Ord.size() - 1;
                        arcV2Backwards  = p1AtB1;
                        arcV2GapBridgeable = true;
                    }
                }
                } // close for(startTi)
            }
            if(arcV2GapBridgeable) {
                STrimBy stbArc2 = STrimBy::EntireCurve(this, hArcV2, arcV2Backwards);
                InsertTrimAt(capSurf2, arcV2GapIdx, &stbArc2);
            }
        }
    } // closes if(hCapSurfV2 != hFillet)

        // Step 16: Handle surfaces with intermediate vertices on V1V2.
        // When another surface (e.g. the pocket ceiling in a DIFFERENCE body)
        // has trim edges that pass through a point P strictly between V1 and V2
        // on the shared wall edge, those surfaces need their trim loops updated
        // to include the fillet arc cross-section at P (Amid->P->Bmid, lying on
        // both hFillet and the intermediate surface).
        {
            // Collect unique points P strictly between V1 and V2 on V1V2.
            std::vector<Vector> interPts;
            for(SCurve &sc_i : curve) {
                if(sc_i.pts.n < 2) continue;
                for(int pi = 0; pi < 2; pi++) {
                    Vector pt = (pi == 0) ? sc_i.pts[0].p
                                          : sc_i.pts[sc_i.pts.n-1].p;
                    if(pt.Equals(V1) || pt.Equals(V2)) continue;
                    Vector pv  = pt.Minus(V1);
                    double dot = pv.Dot(t);
                    if(dot < LENGTH_EPS || dot > edgeLen - LENGTH_EPS) continue;
                    Vector proj = V1.Plus(t.ScaledBy(dot));
                    if(!proj.Equals(pt)) continue;
                    bool dup = false;
                    for(int k = 0; k < (int)interPts.size(); k++) {
                        if(interPts[k].Equals(pt)) { dup = true; break; }
                    }
                    if(!dup) interPts.push_back(pt);
                }
            }

            for(int ip = 0; ip < (int)interPts.size(); ip++) {
                Vector P    = interPts[ip];
                Vector Amid = P.Plus(d1.ScaledBy(setback));
                Vector Bmid = P.Plus(d2.ScaledBy(setback));

                SBezier arcP = {};
                arcP.deg = 2;
                arcP.ctrl[0]  = Amid;
                arcP.ctrl[1]  = P;
                arcP.ctrl[2]  = Bmid;
                arcP.weight[0] = 1.0;
                arcP.weight[1] = arc_weight;
                arcP.weight[2] = 1.0;

                // Collect surfaces that have a trim-edge or SCurve endpoint at P
                // (other than hSurf1, hSurf2, hFillet, hCapSurfV1, hCapSurfV2).
                std::vector<hSSurface> ceilingSurfs;
                for(SCurve &sc_j : curve) {
                    if(sc_j.pts.n < 2) continue;
                    bool fp = sc_j.pts[0].p.Equals(P);
                    bool lp = sc_j.pts[sc_j.pts.n-1].p.Equals(P);
                    if(!fp && !lp) continue;
                    for(int side = 0; side < 2; side++) {
                        hSSurface hN = (side == 0) ? sc_j.surfA : sc_j.surfB;
                        if(hN.v == 0) continue;
                        if(hN == hSurf1 || hN == hSurf2 || hN == hFillet) continue;
                        if(hN == hCapSurfV1 || hN == hCapSurfV2) continue;
                        bool already = false;
                        for(int k = 0; k < (int)ceilingSurfs.size(); k++) {
                            if(ceilingSurfs[k] == hN) { already = true; break; }
                        }
                        if(!already) ceilingSurfs.push_back(hN);
                    }
                }

                for(int cs = 0; cs < (int)ceilingSurfs.size(); cs++) {
                    hSSurface hCS = ceilingSurfs[cs];

                    // Add arc SCurve at P (Amid->P->Bmid), bordering hFillet and hCS.
                    SCurve scP = {};
                    scP.isExact = true;
                    scP.exact = arcP;
                    scP.exact.MakePwlInto(&scP.pts);
                    scP.surfA = hFillet;
                    scP.surfB = hCS;
                    hSCurve hArcP = curve.AddAndAssignId(&scP);

                    SSurface *ceilSurf = surface.FindById(hCS);

                    // If trim.n == 0 (DIFFERENCE body edge case), reconstruct trims
                    // from SCurves that border hCS.
                    if(ceilSurf->trim.n == 0) {
                        for(SCurve &sc_bd : curve) {
                            if(sc_bd.h == hArcP) continue;
                            if(sc_bd.h == hArcV1 || sc_bd.h == hArcV2) continue;
                            if(sc_bd.surfA != hCS && sc_bd.surfB != hCS) continue;
                            if(sc_bd.pts.n < 2) continue;
                            bool bkwd = (sc_bd.surfB == hCS);
                            STrimBy stb_new = {};
                            stb_new.curve     = sc_bd.h;
                            stb_new.backwards = bkwd;
                            stb_new.start  = bkwd ? sc_bd.pts[sc_bd.pts.n-1].p
                                                   : sc_bd.pts[0].p;
                            stb_new.finish = bkwd ? sc_bd.pts[0].p
                                                   : sc_bd.pts[sc_bd.pts.n-1].p;
                            ceilSurf->trim.Add(&stb_new);
                        }
                        ceilSurf = surface.FindById(hCS);
                    }

                    // Update trim edges with endpoint P: truncate toward Amid or Bmid.
                    // Use InsertPointIntoCurvePts to determine which direction applies.
                    for(int ti2 = 0; ti2 < ceilSurf->trim.n; ti2++) {
                        STrimBy &stb_c = ceilSurf->trim[ti2];
                        if(!stb_c.start.Equals(P) && !stb_c.finish.Equals(P)) continue;
                        SCurve *nc = curve.FindByIdNoOops(stb_c.curve);
                        if(!nc) continue;
                        Vector newPt;
                        bool doAmid = InsertPointIntoCurvePts(nc, Amid);
                        if(doAmid) {
                            newPt = Amid;
                        } else {
                            if(!InsertPointIntoCurvePts(nc, Bmid)) continue;
                            newPt = Bmid;
                        }
                        TruncateCurveAtVertex(nc, P, newPt);
                        UpdateAllSurfaceTrimEndpoints(this, nc->h, P, newPt);
                        if(stb_c.start.Equals(P))  stb_c.start  = newPt;
                        if(stb_c.finish.Equals(P)) stb_c.finish = newPt;
                    }

                    ceilSurf = surface.FindById(hCS);

                    // Graph traversal: find the open gap and insert the arc.
                    bool arcPBackwards     = false;
                    bool arcPGapBridgeable = false;
                    int  arcPGapIdx        = -1;
                    {
                        int n = ceilSurf->trim.n;
                        if(n > 0) {
                            std::vector<bool> vis(n, false);
                            vis[0] = true;
                            Vector gStartPt = ceilSurf->trim[0].start;
                            Vector gCur     = ceilSurf->trim[0].finish;
                            int    gLast    = 0;
                            bool   gStuck   = false;
                            for(int iter = 1; iter < n; iter++) {
                                bool found = false;
                                for(int j = 0; j < n; j++) {
                                    if(vis[j]) continue;
                                    if(gCur.Equals(ceilSurf->trim[j].start)) {
                                        gCur  = ceilSurf->trim[j].finish;
                                        gLast = j;
                                        vis[j] = true;
                                        found  = true;
                                        break;
                                    }
                                }
                                if(!found) { gStuck = true; break; }
                            }
                            bool gClosed = (!gStuck && gCur.Equals(gStartPt));
                            if(!gClosed) {
                                if(gCur.Equals(Amid)) {
                                    arcPBackwards     = false;
                                    arcPGapBridgeable = true;
                                    arcPGapIdx        = gLast;
                                } else if(gCur.Equals(Bmid)) {
                                    arcPBackwards     = true;
                                    arcPGapBridgeable = true;
                                    arcPGapIdx        = gLast;
                                }
                            }
                        }
                    }
                    if(arcPGapBridgeable) {
                        STrimBy stbArcP = STrimBy::EntireCurve(this, hArcP, arcPBackwards);
                        ceilSurf = surface.FindById(hCS);
                        InsertTrimAt(ceilSurf, arcPGapIdx, &stbArcP);
                    }
                }
            }
        }

        // Bridge any remaining trim gaps in the surfaces we modified for fillet.
        // Same logic as for chamfer: when InsertPointIntoCurvePts fails for a prior-fillet
        // cap edge (diagonal arc curve), the trim polygon is left open. Bridge the gap.
        // BridgeTrimGapIfOpen verifies graph connectivity before bridging.
        {
            hSSurface toCheck[4] = { hSurf1, hSurf2, hCapSurfV1, hCapSurfV2 };
            bool seen[4] = { false, false, false, false };
            for(int ti = 0; ti < 4; ti++) {
                if(toCheck[ti] == hFillet) continue;
                bool dup = false;
                for(int tj = 0; tj < ti; tj++) { if(toCheck[tj] == toCheck[ti]) { dup = true; break; } }
                if(dup) { seen[ti] = true; continue; }
            seen[ti] = true;
                BridgeTrimGapIfOpen(this, toCheck[ti], hFillet);
            }
            (void)seen;
        }

        // Post-process: when BridgeTrimGapIfOpen created bridges for curved cap surfaces
        // (e.g. a prior fillet cylinder), the corner vertex V1 used as a bridge endpoint
        // may not lie on hFillet's surface, causing wrong UV projections.  Fix: if two
        // bridges share a common corner vertex V1 (pTo of one bridge == pFrom of another),
        // create a flat corner triangle surface (A0-V1-B0) that correctly borders h=7,
        // h=5 and h=8 via properly-surfaced curves (NC1, NC2, NC3).  This gives the
        // triangulator vertices that are all on their respective surfaces.
        {
            // Collect bridges: linear 2-pt curves with surfA==hFillet, not yet in
            // hFillet's trim, but present in some other surface's trim.
            struct BridgeInfo {
                hSCurve   h;
                Vector    pFrom;   // pts[0] = gapEnd
                Vector    pTo;     // pts[1] = gapStart
                hSSurface otherH;  // surface that owns this bridge in its trim
            };
            std::vector<BridgeInfo> bridges;
            for(SCurve &sc_br : curve) {
                if(sc_br.surfA != hFillet) continue;
                if(sc_br.pts.n != 2) continue;
                SSurface *hFilletSurf0 = surface.FindById(hFillet);
                bool inFillet = false;
                for(int k = 0; k < hFilletSurf0->trim.n; k++) {
                    if(hFilletSurf0->trim[k].curve == sc_br.h) { inFillet = true; break; }
                }
                if(inFillet) continue;
                hSSurface otherH = {};
                for(SSurface &ss_br : surface) {
                    if(ss_br.h == hFillet) continue;
                    for(int k = 0; k < ss_br.trim.n; k++) {
                        if(ss_br.trim[k].curve == sc_br.h) { otherH = ss_br.h; break; }
                    }
                    if(otherH.v != 0) break;
                }
                if(otherH.v == 0) continue;
                BridgeInfo bi = {};
                bi.h      = sc_br.h;
                bi.pFrom  = sc_br.pts[0].p;
                bi.pTo    = sc_br.pts[1].p;
                bi.otherH = otherH;
                bridges.push_back(bi);
            }

            // Look for a pair of bridges sharing a corner vertex V1.
            // If found: build a flat corner triangle surface to fill the gap.
            bool cornerCreated = false;
            for(int i = 0; i < (int)bridges.size() && i < 8; i++) {
                for(int j = i+1; j < (int)bridges.size() && j < 8; j++) {
                    // Check whether bridges[i].pTo == bridges[j].pFrom (V1 shared)
                    // or bridges[j].pTo == bridges[i].pFrom.
                    int bA = -1, bB = -1;  // bA: pFrom==V1, bB: pTo==V1
                    if(bridges[i].pTo.Equals(bridges[j].pFrom)) {
                        bB = i;  // bridges[bB].pTo = V1
                        bA = j;  // bridges[bA].pFrom = V1
                    } else if(bridges[j].pTo.Equals(bridges[i].pFrom)) {
                        bB = j;
                        bA = i;
                    } else if(bridges[i].pFrom.Equals(bridges[j].pFrom)) {
                        // FORK pattern: both bridges originate from same corner vertex V1.
                        // Chamfer-then-fillet creates bridges V1→A0 and V1→B0.
                        bB = i;
                        bA = j;
                    }
                    if(bA < 0) continue;

                    // Chain pattern: bB.pTo == bA.pFrom == V1.
                    // Fork pattern: bB.pFrom == bA.pFrom == V1 (both originate from V1).
                    bool forkPattern = bridges[bB].pFrom.Equals(bridges[bA].pFrom) &&
                                       !bridges[bB].pTo.Equals(bridges[bA].pFrom);
                    Vector cornerV1, A0, B0;
                    if(forkPattern) {
                        cornerV1 = bridges[bB].pFrom;  // shared origin V1
                        A0       = bridges[bB].pTo;    // endpoint on one surface
                        B0       = bridges[bA].pTo;    // endpoint on other surface
                    } else {
                        cornerV1 = bridges[bB].pTo;    // == bridges[bA].pFrom
                        A0       = bridges[bB].pFrom;
                        B0       = bridges[bA].pTo;
                    }
                    hSSurface hSurfA0 = bridges[bB].otherH;  // h=7 (has bridge to V1)
                    hSSurface hSurfB0 = bridges[bA].otherH;  // h=5 (has bridge from V1)

                    // Skip degenerate corners where A0 == B0 (zero-area triangle)
                    if(A0.Equals(B0)) continue;


                    // Create flat corner triangle surface FromPlane(V1, A0-V1, B0-V1).
                    // UV: V1=(0,0), A0=(0,1), B0=(1,0). Correct CCW winding: V1→B0→A0→V1.
                    // For forkPattern, swap u/v to flip the surface normal outward.
                    // Normal = u×v. Without swap: (A0-V1)×(B0-V1) may point inward.
                    // Swapping gives (B0-V1)×(A0-V1) which flips the normal.
                    SSurface cornerSurf = forkPattern
                        ? SSurface::FromPlane(
                            cornerV1,
                            B0.Minus(cornerV1),  // u direction: V1→B0
                            A0.Minus(cornerV1))  // v direction: V1→A0
                        : SSurface::FromPlane(
                            cornerV1,
                            A0.Minus(cornerV1),  // u direction: V1→A0
                            B0.Minus(cornerV1)); // v direction: V1→B0
                    cornerSurf.color = surface.FindById(hFillet)->color;
                    cornerSurf.face = faceH.v;
                    // Option B (iter-25 item-17): tag fillet-style fillet-corner surfaces.
                    // Symmetric to the MakeFromChamferOf corner-tag (iter-24 item-15).
                    // The chain-pattern branch (!forkPattern) produces a cornerSurf whose
                    // UV parametrization emits triangles that display-backface under raw
                    // winding.  forkPattern already swaps u/v to correct the normal
                    // geometrically, so it must NOT be tagged.  Setting flipTriangleNormals
                    // causes TriangulateInto (iter-14) to OR FLAG_FLIP_DISPLAY_NORMAL onto
                    // the emitted STriangles; EffectiveNormal() then reports the flipped
                    // normal to display/front-back classifiers without touching raw winding
                    // (so bsp.cpp edge-match and leak invariants are preserved).
                    // Iter-26 finding: the forkPattern swap does NOT reliably
                    // produce outward normals for all geometries. When V1/A0/B0
                    // are coplanar, the raw NormalAt() is perpendicular to all
                    // local geometric references, making the heuristic fail.
                    // Always set flipTriangleNormals for robustness, and also
                    // display the corner surface (was hidden as a bandaid).
                    cornerSurf.flipTriangleNormals = true;
                    cornerSurf.excludeFromDisplay = false;
                    hSSurface hCorner = surface.AddAndAssignId(&cornerSurf);
                    // Re-lookup after reallocation:
                    surface.FindById(hFillet);

                    // Create NC1 (A0→V1): shared between hCorner and hSurfA0.
                    // In hSurfA0 (h=7): replaces bridge[bB] (same path, correct surfA).
                    hSCurve hNC1 = AddLinearCurve(this, A0, cornerV1, hCorner, hSurfA0);

                    // Create NC2 (V1→B0): shared between hCorner and hSurfB0.
                    // In hSurfB0 (h=5): replaces bridge[bA] (same path, correct surfA).
                    hSCurve hNC2 = AddLinearCurve(this, cornerV1, B0, hCorner, hSurfB0);

                    // Re-lookup after curve additions:
                    SSurface *ssA0 = surface.FindById(hSurfA0);
                    // Replace bridge[bB] in hSurfA0 with NC1 (A0→V1).
                    bool replacedBridgeBB = false;
                    for(int ti = 0; ti < ssA0->trim.n; ti++) {
                        if(ssA0->trim[ti].curve == bridges[bB].h) {
                            ssA0->trim[ti] = STrimBy::EntireCurve(this, hNC1, forkPattern);
                            replacedBridgeBB = true;
                            break;
                        }
                    }

                    SSurface *ssB0 = surface.FindById(hSurfB0);
                    // Replace bridge[bA] in hSurfB0 with NC2 (V1→B0).
                    bool replacedBridgeBA = false;
                    for(int ti = 0; ti < ssB0->trim.n; ti++) {
                        if(ssB0->trim[ti].curve == bridges[bA].h) {
                            ssB0->trim[ti] = STrimBy::EntireCurve(this, hNC2, false);
                            replacedBridgeBA = true;
                            break;
                        }
                    }

                    // Create NC3 (A0→B0): shared between hCorner and hFillet.
                    hSCurve hNC3 = AddLinearCurve(this, A0, B0, hCorner, hFillet);

                    // Replace arc hArcV1 in hFillet's trim with NC3.
                    SSurface *hFilletSurf = surface.FindById(hFillet);
                    for(int ai = 0; ai < hFilletSurf->trim.n; ai++) {
                        // Only match arc edges (hArcV1/hArcV2), never side edges
                        if(hFilletSurf->trim[ai].curve != hArcV1 &&
                           hFilletSurf->trim[ai].curve != hArcV2) continue;
                        SCurve *ac = curve.FindByIdNoOops(hFilletSurf->trim[ai].curve);
                        if(!ac || ac->pts.n < 2) continue;
                        if(ac->pts[0].p.Equals(A0) || ac->pts[ac->pts.n-1].p.Equals(A0)) {
                            // Account for trim's backwards flag when computing NC3 direction
                            bool trimBkwd = hFilletSurf->trim[ai].backwards;
                            int npts = ac->pts.n;
                            Vector effectiveStart = trimBkwd ? ac->pts[npts-1].p : ac->pts[0].p;
                            bool nc3Backwards = !effectiveStart.Equals(A0);
                            hFilletSurf->trim[ai] = STrimBy::EntireCurve(this, hNC3, nc3Backwards);
                            break;
                        }
                    }

                    // Build cornerSurf's trim (CCW winding in UV: V1→B0→A0→V1):
                    //   NC2 fwd  (V1→B0): in UV goes (0,0)→(1,0)
                    //   NC3 bwd  (B0→A0): in UV goes (1,0)→(0,1)  [diagonal]
                    //   NC1 fwd  (A0→V1): in UV goes (0,1)→(0,0)
                    SSurface *hCornerSurf = surface.FindById(hCorner);
                    STrimBy  stb;
                    stb = STrimBy::EntireCurve(this, hNC2, false); // V1→B0
                    hCornerSurf->trim.Add(&stb);
                    hCornerSurf = surface.FindById(hCorner);
                    stb = STrimBy::EntireCurve(this, hNC3, true);  // B0→A0 (NC3 reversed)
                    hCornerSurf->trim.Add(&stb);
                    hCornerSurf = surface.FindById(hCorner);
                    stb = STrimBy::EntireCurve(this, hNC1, false); // A0→V1
                    hCornerSurf->trim.Add(&stb);
                    cornerCreated = true;
                }
            }

        // CF fallback: when no valid corner was created by the bridge pairing
        // (either no bridges exist, or all pairings were degenerate A0==B0),
        // and cap surface is flat
        // chamfer cap, create corner triangle to fill the topological gap.
        // CF fallback: when bridges are degenerate (A0==B0, both bridges go P→cA),
        // use bridge data directly to find the gap triangle in hSurf1's trim chain.
        // The gap is at: P (bridge pFrom), cA (bridge pTo), B (next trim endpoint after cA).
        // CF fallback: when bridges are degenerate (A0==B0, both go P→cA),
        // shortcut BOTH hSurf1 and the chamfer cap from P→B directly,
        // eliminating the off-plane point A from all trim chains.
        // No corner triangle needed — just cut out the triangular pocket.
        if(!cornerCreated && !bridges.empty()) {
            // Find which bridge is in hSurf1's trims (it might not be bridges[0]!)
            int surf1BridgeIdx = -1;
            int otherBridgeIdx = -1;
            for(int bi = 0; bi < (int)bridges.size(); bi++) {
                SSurface *ss1chk = surface.FindById(hSurf1);
                for(int ti = 0; ti < ss1chk->trim.n; ti++) {
                    if(ss1chk->trim[ti].curve == bridges[bi].h) {
                        surf1BridgeIdx = bi;
                        break;
                    }
                }
                if(surf1BridgeIdx >= 0) break;
            }
            // Find the other bridge (not in hSurf1)
            for(int bi = 0; bi < (int)bridges.size(); bi++) {
                if(bi != surf1BridgeIdx) {
                    otherBridgeIdx = bi;
                    break;
                }
            }
            if(surf1BridgeIdx < 0) {
                // Bridge not in hSurf1 — check hSurf2 and other surfaces
                // Try each bridge's otherH as the "host" surface
                for(int bi = 0; bi < (int)bridges.size(); bi++) {
                    hSSurface hostH = bridges[bi].otherH;
                    SSurface *hostSurf = surface.FindByIdNoOops(hostH);
                    if(!hostSurf) continue;
                    // Check if this bridge is in hostH's trims
                    bool inHost = false;
                    for(int ti = 0; ti < hostSurf->trim.n; ti++) {
                        if(hostSurf->trim[ti].curve == bridges[bi].h) { inHost = true; break; }
                    }
                    if(!inHost) continue;
                    // Find the bridge trim index in the host surface
                    int bridgeTrimIdx = -1;
                    for(int ti = 0; ti < hostSurf->trim.n; ti++) {
                        if(hostSurf->trim[ti].curve == bridges[bi].h) {
                            bridgeTrimIdx = ti;
                            break;
                        }
                    }
                    if(bridgeTrimIdx < 0) continue;
                    // Find next trim after bridge
                    int nextIdx = (bridgeTrimIdx + 1) % hostSurf->trim.n;
                    hSCurve hNextCurve = hostSurf->trim[nextIdx].curve;
                    SCurve *nextCrv = curve.FindByIdNoOops(hNextCurve);
                    if(!nextCrv || nextCrv->pts.n < 2) continue;
                    Vector gapP = bridges[bi].pFrom;
                    Vector gapA = bridges[bi].pTo;
                    Vector p0 = nextCrv->pts[0].p;
                    Vector pN = nextCrv->pts[nextCrv->pts.n-1].p;
                    Vector gapB = p0.Equals(gapA) ? pN : p0;

                    // Find the other bridge surface (the chamfer cap)
                    hSSurface hCapH = {};
                    hSCurve hCapBridgeCurve = {};
                    for(int bj = 0; bj < (int)bridges.size(); bj++) {
                        if(bj != bi) {
                            hCapH = bridges[bj].otherH;
                            hCapBridgeCurve = bridges[bj].h;
                            break;
                        }
                    }
                    if(hCapH.v == 0) continue;

                    // Create NC_PB (P→B): shared between hostH and hCapH
                    hSCurve hNC_PB = AddLinearCurve(this, gapP, gapB, hostH, hCapH);

                    // In hostH: replace bridge with NC_PB, remove next
                    hostSurf = surface.FindById(hostH);
                    for(int ti = 0; ti < hostSurf->trim.n; ti++) {
                        if(hostSurf->trim[ti].curve == bridges[bi].h) {
                            hostSurf->trim[ti] = STrimBy::EntireCurve(this, hNC_PB, false);
                            break;
                        }
                    }
                    hostSurf = surface.FindById(hostH);
                    for(int ti = 0; ti < hostSurf->trim.n; ti++) {
                        if(hostSurf->trim[ti].curve == hNextCurve) {
                            for(int ri = ti; ri < hostSurf->trim.n - 1; ri++)
                                hostSurf->trim[ri] = hostSurf->trim[ri + 1];
                            hostSurf->trim.n--;
                            break;
                        }
                    }

                    // In hCapH: replace cap bridge with NC_PB, remove hNextCurve
                    SSurface *capSurf = surface.FindByIdNoOops(hCapH);
                    if(capSurf) {
                        for(int ti = 0; ti < capSurf->trim.n; ti++) {
                            if(capSurf->trim[ti].curve == hCapBridgeCurve) {
                                capSurf->trim[ti] = STrimBy::EntireCurve(this, hNC_PB, false);
                                break;
                            }
                        }
                        capSurf = surface.FindByIdNoOops(hCapH);
                        if(capSurf) {
                            for(int ti = 0; ti < capSurf->trim.n; ti++) {
                                if(capSurf->trim[ti].curve == hNextCurve) {
                                    for(int ri = ti; ri < capSurf->trim.n - 1; ri++)
                                        capSurf->trim[ri] = capSurf->trim[ri + 1];
                                    capSurf->trim.n--;
                                    break;
                                }
                            }
                        }
                    }
                    break; // done with this bridge
                }
            } else {
                // Bridge IS in hSurf1 — original code path
                Vector gapP = bridges[surf1BridgeIdx].pFrom;
                Vector gapA = bridges[surf1BridgeIdx].pTo;
                hSCurve hBridgeCurve = bridges[surf1BridgeIdx].h;

                SSurface *ss1 = surface.FindById(hSurf1);
                int bridgeTrimIdx = -1;
                for(int ti = 0; ti < ss1->trim.n; ti++) {
                    if(ss1->trim[ti].curve == hBridgeCurve) {
                        bridgeTrimIdx = ti;
                        break;
                    }
                }

                if(bridgeTrimIdx >= 0) {
                    int nextIdx = (bridgeTrimIdx + 1) % ss1->trim.n;
                    hSCurve hNextCurve = ss1->trim[nextIdx].curve;
                    SCurve *nextCrv = curve.FindByIdNoOops(hNextCurve);
                    if(nextCrv && nextCrv->pts.n >= 2) {
                        Vector p0 = nextCrv->pts[0].p;
                        Vector pN = nextCrv->pts[nextCrv->pts.n-1].p;
                        Vector gapB = p0.Equals(gapA) ? pN : p0;

                        // Find the other bridge surface (chamfer cap)
                        hSSurface hOtherBridge = {};
                        hSCurve hOtherBridgeCurve = {};
                        if(otherBridgeIdx >= 0) {
                            hOtherBridge = bridges[otherBridgeIdx].otherH;
                            hOtherBridgeCurve = bridges[otherBridgeIdx].h;
                        }
                        if(hOtherBridge.v == 0) goto skipCfFallback;

                        {
                        // Create NC_PB (P→B): shared between hSurf1 and hOtherBridge
                        hSCurve hNC_PB = AddLinearCurve(this, gapP, gapB, hSurf1, hOtherBridge);

                        // In hSurf1: replace bridge with NC_PB(fwd → P→B), remove next
                        ss1 = surface.FindById(hSurf1);
                        for(int ti = 0; ti < ss1->trim.n; ti++) {
                            if(ss1->trim[ti].curve == hBridgeCurve) {
                                ss1->trim[ti] = STrimBy::EntireCurve(this, hNC_PB, false);
                                break;
                            }
                        }
                        ss1 = surface.FindById(hSurf1);
                        for(int ti = 0; ti < ss1->trim.n; ti++) {
                            if(ss1->trim[ti].curve == hNextCurve) {
                                for(int ri = ti; ri < ss1->trim.n - 1; ri++)
                                    ss1->trim[ri] = ss1->trim[ri + 1];
                                ss1->trim.n--;
                                break;
                            }
                        }

                        // In hOtherBridge (chamfer cap): replace bridge with NC_PB, remove hNextCurve
                        SSurface *ssOtherBr = surface.FindByIdNoOops(hOtherBridge);
                        if(ssOtherBr) {
                            for(int ti = 0; ti < ssOtherBr->trim.n; ti++) {
                                if(ssOtherBr->trim[ti].curve == hOtherBridgeCurve) {
                                    ssOtherBr->trim[ti] = STrimBy::EntireCurve(this, hNC_PB, false);
                                    break;
                                }
                            }
                            ssOtherBr = surface.FindByIdNoOops(hOtherBridge);
                            if(ssOtherBr) {
                                for(int ti = 0; ti < ssOtherBr->trim.n; ti++) {
                                    if(ssOtherBr->trim[ti].curve == hNextCurve) {
                                        for(int ri = ti; ri < ssOtherBr->trim.n - 1; ri++)
                                            ssOtherBr->trim[ri] = ssOtherBr->trim[ri + 1];
                                        ssOtherBr->trim.n--;
                                        break;
                                    }
                                }
                            }
                        }
                        }
                        skipCfFallback:;
                    }
                }
            }
        }

        }

        // Trim closure enforcement: repair any open trim loops after fillet corner synthesis
        RepairOpenTrimLoops(this, "post-fillet-corner-synthesis");

        // Safety: verify no surface trim still references hSharedSC before removing.
        bool hSharedStillReferenced = false;
        for(SSurface &ss : surface) {
            for(STrimBy *stb = ss.trim.First(); stb; stb = ss.trim.NextAfter(stb)) {
                if(stb->curve == hSharedSC) { hSharedStillReferenced = true; break; }
            }
            if(hSharedStillReferenced) break;
        }
        if(hSharedStillReferenced) { booleanFailed = true; return; }

        // Dump all surface trim chains for debugging.
        for(SSurface &ssDump : surface) {
            for(int ti = 0; ti < ssDump.trim.n; ti++) {
                STrimBy &stb = ssDump.trim[ti];
                SCurve *sc = curve.FindByIdNoOops(stb.curve);
                hSSurface sA = {0}, sB = {0};
                int npts = 0;
                if(sc) { sA = sc->surfA; sB = sc->surfB; npts = sc->pts.n; }
            }
        }
        curve.RemoveById(hSharedSC);
    }

    // Post-process: remove degenerate trims from ALL surfaces.
    // A degenerate trim is a 2-point curve whose endpoints coincide (zero-length).
    // This happens when a third-party surface (not in toCheck[]) gets a curve
    // truncated to zero length during fillet processing. The degenerate trim
    // breaks polygon assembly ("trim was empty"), causing naked edges.
    // Only match pts.n == 2 (exactly 2 points) to avoid removing valid closed loops.
    for(SSurface &ss : surface) {
        for(int ti = ss.trim.n - 1; ti >= 0; ti--) {
            SCurve *sc = curve.FindByIdNoOops(ss.trim[ti].curve);
            if(!sc) continue;
            if(sc->pts.n == 2 &&
               sc->pts[0].p.Equals(sc->pts[1].p))
            {
                List<STrimBy> keep = {};
                for(int ki = 0; ki < ss.trim.n; ki++) {
                    if(ki != ti) keep.Add(&ss.trim[ki]);
                }
                ss.trim.Clear();
                for(int ki = 0; ki < keep.n; ki++) ss.trim.Add(&keep[ki]);
                keep.Clear();
                break;  // restart outer loop will pick up next surface
            }
        }
    }

    // Second pass: collapse zero-area sliver surfaces.
    // After removing degenerate trims, a surface may have exactly 2 remaining
    // trims whose curves connect the same two endpoints in opposite directions.
    // This forms a zero-area polygon that produces no mesh, leaving the boundary
    // curves as naked edges. Fix by merging the two curves into one shared edge
    // between the actual neighbor surfaces.
    for(SSurface &ss : surface) {
        if(ss.trim.n != 2) continue;

        SCurve *sc0 = curve.FindByIdNoOops(ss.trim[0].curve);
        SCurve *sc1 = curve.FindByIdNoOops(ss.trim[1].curve);
        if(!sc0 || !sc1) continue;
        if(sc0->pts.n < 2 || sc1->pts.n < 2) continue;

        // Check if both curves connect the same two endpoints (reversed)
        Vector p0s = sc0->pts[0].p, p0e = sc0->pts[sc0->pts.n-1].p;
        Vector p1s = sc1->pts[0].p, p1e = sc1->pts[sc1->pts.n-1].p;
        if(!(p0s.Equals(p1e) && p0e.Equals(p1s))) continue;
        // Skip if both endpoints are the same (fully degenerate, already handled)
        if(p0s.Equals(p0e)) continue;

        // This is a zero-area sliver surface. Merge boundary curves.
        hSSurface hDeg = ss.h;
        hSCurve hKeep = ss.trim[0].curve;
        hSCurve hRemove = ss.trim[1].curve;

        SCurve *scKeep = curve.FindById(hKeep);
        SCurve *scRemove = curve.FindById(hRemove);

        // Find the "other" surface for each curve (not the degenerate one)
        hSSurface otherKeep = (scKeep->surfA == hDeg) ? scKeep->surfB : scKeep->surfA;
        hSSurface otherRemove = (scRemove->surfA == hDeg) ? scRemove->surfB : scRemove->surfA;

        // Update scKeep: replace degenerate surface ref with otherRemove
        if(scKeep->surfA == hDeg) scKeep->surfA = otherRemove;
        else                      scKeep->surfB = otherRemove;

        // In otherRemove's surface, find trim referencing hRemove, replace with hKeep.
        // Since scKeep is the geometric reverse of scRemove, flip the backwards flag.
        SSurface *ssOther = surface.FindByIdNoOops(otherRemove);
        if(ssOther) {
            for(int ti = 0; ti < ssOther->trim.n; ti++) {
                if(ssOther->trim[ti].curve == hRemove) {
                    bool oldBkwd = ssOther->trim[ti].backwards;
                    ssOther->trim[ti] = STrimBy::EntireCurve(this, hKeep, !oldBkwd);
                    break;
                }
            }
        }

        // Clear the degenerate surface's trims so it produces no mesh.
        ss.trim.Clear();
    }

    // Third pass: remove phantom surfaces (trim.n == 0) entirely from the shell.
    // After sliver collapse, the degenerate surface has 0 trims and produces no
    // mesh, but its mere presence can confuse curve lookups and surface iteration.
    {
        // Collect handles to remove (can't remove during iteration)
        std::vector<hSSurface> phantoms;
        for(SSurface &ss : surface) {
            if(ss.trim.n == 0) {
                phantoms.push_back(ss.h);
            }
        }
        for(hSSurface hPh : phantoms) {
            surface.RemoveById(hPh);
        }
    }

}

} // namespace SolveSpace
