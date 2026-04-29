//-----------------------------------------------------------------------------
// Triple operation tests — 3-edge corner chamfer/fillet combinations
//
// Tests all 3 chamfer/fillet operations on the 3 edges meeting at a box
// corner, exercising every permutation order and operation type combination.
//
// Two representative corners get full coverage (6 perms × 8 op combos = 48):
//   - LEFT+FRONT+TOP corner (0,0,20)
//   - RIGHT+BACK+BOTTOM corner (20,20,0)
//
// Six remaining corners get smoke-test coverage (CCC + FFF, perm 012 = 2):
//   - LEFT+FRONT+BOTTOM, RIGHT+FRONT+BOTTOM, LEFT+BACK+BOTTOM
//   - RIGHT+FRONT+TOP, RIGHT+BACK+TOP, LEFT+BACK+TOP
//
// Total: 48 + 48 + 12 = 108 tests.
//
// These are TDD RED phase tests: they are expected to FAIL on mesh quality
// checks (naked edges, self-intersections, backfacing) because the 3rd
// chamfer/fillet on a box corner currently produces bad geometry.
//-----------------------------------------------------------------------------
#include "helpers.h"

// Permutation arrays: indices into the edges[] array
static int P012[] = {0, 1, 2};
static int P021[] = {0, 2, 1};
static int P102[] = {1, 0, 2};
static int P120[] = {1, 2, 0};
static int P201[] = {2, 0, 1};
static int P210[] = {2, 1, 0};

// Operation type arrays: true=chamfer, false=fillet
static bool CCC[] = {true,  true,  true };
static bool CCF[] = {true,  true,  false};
static bool CFC[] = {true,  false, true };
static bool CFF[] = {true,  false, false};
static bool FCC[] = {false, true,  true };
static bool FCF[] = {false, true,  false};
static bool FFC[] = {false, false, true };
static bool FFF[] = {false, false, false};

//=============================================================================
// Corner: LEFT + FRONT + TOP  (vertex at 0, 0, 20)
// edges[0] = LEFT∩FRONT, edges[1] = LEFT∩TOP, edges[2] = FRONT∩TOP
// Full coverage: 6 permutations × 8 op combos = 48 tests
//=============================================================================

// --- Permutation 012 ---
TEST_CASE(tripleop_left_front_top_CCC_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P012, CCC);
}
TEST_CASE(tripleop_left_front_top_CCF_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P012, CCF);
}
TEST_CASE(tripleop_left_front_top_CFC_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P012, CFC);
}
TEST_CASE(tripleop_left_front_top_CFF_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P012, CFF);
}
TEST_CASE(tripleop_left_front_top_FCC_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P012, FCC);
}
TEST_CASE(tripleop_left_front_top_FCF_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P012, FCF);
}
TEST_CASE(tripleop_left_front_top_FFC_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P012, FFC);
}
TEST_CASE(tripleop_left_front_top_FFF_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P012, FFF);
}

// --- Permutation 021 ---
TEST_CASE(tripleop_left_front_top_CCC_021) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P021, CCC);
}
TEST_CASE(tripleop_left_front_top_CCF_021) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P021, CCF);
}
TEST_CASE(tripleop_left_front_top_CFC_021) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P021, CFC);
}
TEST_CASE(tripleop_left_front_top_CFF_021) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P021, CFF);
}
TEST_CASE(tripleop_left_front_top_FCC_021) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P021, FCC);
}
TEST_CASE(tripleop_left_front_top_FCF_021) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P021, FCF);
}
TEST_CASE(tripleop_left_front_top_FFC_021) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P021, FFC);
}
TEST_CASE(tripleop_left_front_top_FFF_021) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P021, FFF);
}

// --- Permutation 102 ---
TEST_CASE(tripleop_left_front_top_CCC_102) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P102, CCC);
}
TEST_CASE(tripleop_left_front_top_CCF_102) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P102, CCF);
}
TEST_CASE(tripleop_left_front_top_CFC_102) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P102, CFC);
}
TEST_CASE(tripleop_left_front_top_CFF_102) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P102, CFF);
}
TEST_CASE(tripleop_left_front_top_FCC_102) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P102, FCC);
}
TEST_CASE(tripleop_left_front_top_FCF_102) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P102, FCF);
}
TEST_CASE(tripleop_left_front_top_FFC_102) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P102, FFC);
}
TEST_CASE(tripleop_left_front_top_FFF_102) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P102, FFF);
}

// --- Permutation 120 ---
TEST_CASE(tripleop_left_front_top_CCC_120) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P120, CCC);
}
TEST_CASE(tripleop_left_front_top_CCF_120) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P120, CCF);
}
TEST_CASE(tripleop_left_front_top_CFC_120) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P120, CFC);
}
TEST_CASE(tripleop_left_front_top_CFF_120) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P120, CFF);
}
TEST_CASE(tripleop_left_front_top_FCC_120) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P120, FCC);
}
TEST_CASE(tripleop_left_front_top_FCF_120) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P120, FCF);
}
TEST_CASE(tripleop_left_front_top_FFC_120) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P120, FFC);
}
TEST_CASE(tripleop_left_front_top_FFF_120) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P120, FFF);
}

// --- Permutation 201 ---
TEST_CASE(tripleop_left_front_top_CCC_201) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P201, CCC);
}
TEST_CASE(tripleop_left_front_top_CCF_201) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P201, CCF);
}
TEST_CASE(tripleop_left_front_top_CFC_201) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P201, CFC);
}
TEST_CASE(tripleop_left_front_top_CFF_201) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P201, CFF);
}
TEST_CASE(tripleop_left_front_top_FCC_201) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P201, FCC);
}
TEST_CASE(tripleop_left_front_top_FCF_201) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P201, FCF);
}
TEST_CASE(tripleop_left_front_top_FFC_201) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P201, FFC);
}
TEST_CASE(tripleop_left_front_top_FFF_201) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P201, FFF);
}

// --- Permutation 210 ---
TEST_CASE(tripleop_left_front_top_CCC_210) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P210, CCC);
}
TEST_CASE(tripleop_left_front_top_CCF_210) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P210, CCF);
}
TEST_CASE(tripleop_left_front_top_CFC_210) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P210, CFC);
}
TEST_CASE(tripleop_left_front_top_CFF_210) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P210, CFF);
}
TEST_CASE(tripleop_left_front_top_FCC_210) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P210, FCC);
}
TEST_CASE(tripleop_left_front_top_FCF_210) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P210, FCF);
}
TEST_CASE(tripleop_left_front_top_FFC_210) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P210, FFC);
}
TEST_CASE(tripleop_left_front_top_FFF_210) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_TOP, P210, FFF);
}

//=============================================================================
// Corner: RIGHT + BACK + BOTTOM  (vertex at 20, 20, 0)
// edges[0] = RIGHT∩BACK, edges[1] = RIGHT∩BOTTOM, edges[2] = BACK∩BOTTOM
// Full coverage: 6 permutations × 8 op combos = 48 tests
//=============================================================================

// --- Permutation 012 ---
TEST_CASE(tripleop_right_back_bottom_CCC_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P012, CCC);
}
TEST_CASE(tripleop_right_back_bottom_CCF_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P012, CCF);
}
TEST_CASE(tripleop_right_back_bottom_CFC_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P012, CFC);
}
TEST_CASE(tripleop_right_back_bottom_CFF_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P012, CFF);
}
TEST_CASE(tripleop_right_back_bottom_FCC_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P012, FCC);
}
TEST_CASE(tripleop_right_back_bottom_FCF_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P012, FCF);
}
TEST_CASE(tripleop_right_back_bottom_FFC_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P012, FFC);
}
TEST_CASE(tripleop_right_back_bottom_FFF_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P012, FFF);
}

// --- Permutation 021 ---
TEST_CASE(tripleop_right_back_bottom_CCC_021) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P021, CCC);
}
TEST_CASE(tripleop_right_back_bottom_CCF_021) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P021, CCF);
}
TEST_CASE(tripleop_right_back_bottom_CFC_021) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P021, CFC);
}
TEST_CASE(tripleop_right_back_bottom_CFF_021) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P021, CFF);
}
TEST_CASE(tripleop_right_back_bottom_FCC_021) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P021, FCC);
}
TEST_CASE(tripleop_right_back_bottom_FCF_021) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P021, FCF);
}
TEST_CASE(tripleop_right_back_bottom_FFC_021) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P021, FFC);
}
TEST_CASE(tripleop_right_back_bottom_FFF_021) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P021, FFF);
}

// --- Permutation 102 ---
TEST_CASE(tripleop_right_back_bottom_CCC_102) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P102, CCC);
}
TEST_CASE(tripleop_right_back_bottom_CCF_102) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P102, CCF);
}
TEST_CASE(tripleop_right_back_bottom_CFC_102) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P102, CFC);
}
TEST_CASE(tripleop_right_back_bottom_CFF_102) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P102, CFF);
}
TEST_CASE(tripleop_right_back_bottom_FCC_102) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P102, FCC);
}
TEST_CASE(tripleop_right_back_bottom_FCF_102) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P102, FCF);
}
TEST_CASE(tripleop_right_back_bottom_FFC_102) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P102, FFC);
}
TEST_CASE(tripleop_right_back_bottom_FFF_102) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P102, FFF);
}

// --- Permutation 120 ---
TEST_CASE(tripleop_right_back_bottom_CCC_120) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P120, CCC);
}
TEST_CASE(tripleop_right_back_bottom_CCF_120) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P120, CCF);
}
TEST_CASE(tripleop_right_back_bottom_CFC_120) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P120, CFC);
}
TEST_CASE(tripleop_right_back_bottom_CFF_120) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P120, CFF);
}
TEST_CASE(tripleop_right_back_bottom_FCC_120) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P120, FCC);
}
TEST_CASE(tripleop_right_back_bottom_FCF_120) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P120, FCF);
}
TEST_CASE(tripleop_right_back_bottom_FFC_120) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P120, FFC);
}
TEST_CASE(tripleop_right_back_bottom_FFF_120) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P120, FFF);
}

// --- Permutation 201 ---
TEST_CASE(tripleop_right_back_bottom_CCC_201) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P201, CCC);
}
TEST_CASE(tripleop_right_back_bottom_CCF_201) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P201, CCF);
}
TEST_CASE(tripleop_right_back_bottom_CFC_201) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P201, CFC);
}
TEST_CASE(tripleop_right_back_bottom_CFF_201) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P201, CFF);
}
TEST_CASE(tripleop_right_back_bottom_FCC_201) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P201, FCC);
}
TEST_CASE(tripleop_right_back_bottom_FCF_201) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P201, FCF);
}
TEST_CASE(tripleop_right_back_bottom_FFC_201) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P201, FFC);
}
TEST_CASE(tripleop_right_back_bottom_FFF_201) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P201, FFF);
}

// --- Permutation 210 ---
TEST_CASE(tripleop_right_back_bottom_CCC_210) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P210, CCC);
}
TEST_CASE(tripleop_right_back_bottom_CCF_210) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P210, CCF);
}
TEST_CASE(tripleop_right_back_bottom_CFC_210) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P210, CFC);
}
TEST_CASE(tripleop_right_back_bottom_CFF_210) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P210, CFF);
}
TEST_CASE(tripleop_right_back_bottom_FCC_210) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P210, FCC);
}
TEST_CASE(tripleop_right_back_bottom_FCF_210) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P210, FCF);
}
TEST_CASE(tripleop_right_back_bottom_FFC_210) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P210, FFC);
}
TEST_CASE(tripleop_right_back_bottom_FFF_210) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_BOTTOM, P210, FFF);
}

//=============================================================================
// Remaining 6 corners — smoke tests (CCC + FFF with permutation 012 only)
// 2 tests per corner × 6 corners = 12 tests
//=============================================================================

// Corner: LEFT + FRONT + BOTTOM  (vertex at 0, 0, 0)
TEST_CASE(tripleop_left_front_bottom_CCC_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_BOTTOM, P012, CCC);
}
TEST_CASE(tripleop_left_front_bottom_FFF_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_FRONT, FS_BOTTOM, P012, FFF);
}

// Corner: RIGHT + FRONT + BOTTOM  (vertex at 20, 0, 0)
TEST_CASE(tripleop_right_front_bottom_CCC_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_FRONT, FS_BOTTOM, P012, CCC);
}
TEST_CASE(tripleop_right_front_bottom_FFF_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_FRONT, FS_BOTTOM, P012, FFF);
}

// Corner: LEFT + BACK + BOTTOM  (vertex at 0, 20, 0)
TEST_CASE(tripleop_left_back_bottom_CCC_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_BACK, FS_BOTTOM, P012, CCC);
}
TEST_CASE(tripleop_left_back_bottom_FFF_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_BACK, FS_BOTTOM, P012, FFF);
}

// Corner: RIGHT + FRONT + TOP  (vertex at 20, 0, 20)
TEST_CASE(tripleop_right_front_top_CCC_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_FRONT, FS_TOP, P012, CCC);
}
TEST_CASE(tripleop_right_front_top_FFF_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_FRONT, FS_TOP, P012, FFF);
}

// Corner: RIGHT + BACK + TOP  (vertex at 20, 20, 20)
TEST_CASE(tripleop_right_back_top_CCC_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_TOP, P012, CCC);
}
TEST_CASE(tripleop_right_back_top_FFF_012) {
    RunTripleOpTest(helper, FS_RIGHT, FS_BACK, FS_TOP, P012, FFF);
}

// Corner: LEFT + BACK + TOP  (vertex at 0, 20, 20)
TEST_CASE(tripleop_left_back_top_CCC_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_BACK, FS_TOP, P012, CCC);
}
TEST_CASE(tripleop_left_back_top_FFF_012) {
    RunTripleOpTest(helper, FS_LEFT, FS_BACK, FS_TOP, P012, FFF);
}

//=============================================================================
// GUI "Show Interfering Parts" false positive regression test
//
// The GUI command (Command::INTERFERENCE in src/solvespace.cpp) calls
// MakeCertainEdgesInto with EdgeKind::SELF_INTER, coplanarIsInter=false.
// The existing RunTripleOpTest uses NAKED_OR_SELF_INTER, coplanarIsInter=true,
// which has chamfer-vs-chamfer tolerance in the coplanarIsInter=true branch.
// This test exercises the EXACT GUI code path to catch false positives.
//=============================================================================

TEST_CASE(tripleop_interference_gui_check) {
    // Same setup as tripleop_left_front_top_CCC_012:
    // Box + 3 chamfers on LEFT/FRONT/TOP corner, permutation 012, all chamfer
    int perm[] = {0, 1, 2};
    double offset = 2.0;

    hGroup extrudeH = CreateBoxExtrude();
    Group *eg = SK.GetGroup(extrudeH);
    CHECK_TRUE(eg != nullptr);

    // Resolve all 3 faces from the extrude group
    hEntity face1 = GetFace(extrudeH, FS_LEFT);
    hEntity face2 = GetFace(extrudeH, FS_FRONT);
    hEntity face3 = GetFace(extrudeH, FS_TOP);
    CHECK_TRUE(face1.v != 0);
    CHECK_TRUE(face2.v != 0);
    CHECK_TRUE(face3.v != 0);

    // Define 3 edges from the 3 face-pairs at this corner
    hEntity edgeFaceA[3] = { face1, face1, face2 };
    hEntity edgeFaceB[3] = { face2, face3, face3 };

    // Apply 3 chamfer operations in permutation order
    hGroup prevH = extrudeH;
    hGroup opH[3];
    for(int i = 0; i < 3; i++) {
        int ei = perm[i];
        opH[i] = AddChamferGroup(prevH, edgeFaceA[ei], edgeFaceB[ei], offset);
        CHECK_FALSE(SK.GetGroup(opH[i])->booleanFailed);
        prevH = opH[i];
    }

    Group *gLast = SK.GetGroup(opH[2]);
    CHECK_FALSE(gLast->booleanFailed);

    // Generate display mesh on the final group
    gLast->GenerateDisplayItems();
    SMesh *m = &gLast->displayMesh;
    CHECK_TRUE(m->l.n > 0);

    // Check using EXACT same parameters as GUI "Show Interfering Parts":
    //   EdgeKind::SELF_INTER, coplanarIsInter=false
    SEdgeList el = {};
    bool inters, leaks;
    SKdNode::From(m)->MakeCertainEdgesInto(&el,
        EdgeKind::SELF_INTER, /*coplanarIsInter=*/false, &inters, &leaks);
    el.Clear();

    // No false positive self-intersections should be reported
    CHECK_FALSE(inters);
}

//=============================================================================
// Visual verification: Save .slvs files for GUI screenshot testing
//
// These tests build triple-chamfer models and save them to /tmp/ so that
// the visual_verify.sh script can render thumbnails, launch the GUI,
// trigger "Show Naked Edges" / "Show Interfering Parts", take screenshots,
// and analyze for red error pixels.
//
// Each test uses a different permutation to capture both passing and
// failing configurations for TDD RED/GREEN visual comparison.
//=============================================================================

// Helper: Build a triple-chamfer box and save to .slvs file.
// Does NOT use CHECK macros for mesh quality — the point is to save the
// model even if geometry is broken, so visual_verify.sh can screenshot it.
static inline void BuildAndSaveTripleChamfer(
    FaceSpec f1, FaceSpec f2, FaceSpec f3,
    int perm[3], double offset,
    const char *outputPath)
{
    hGroup extrudeH = CreateBoxExtrude();

    // Resolve all 3 faces from the extrude group
    hEntity face1 = GetFace(extrudeH, f1);
    hEntity face2 = GetFace(extrudeH, f2);
    hEntity face3 = GetFace(extrudeH, f3);

    // Define 3 edges from the 3 face-pairs at this corner
    hEntity edgeFaceA[3] = { face1, face1, face2 };
    hEntity edgeFaceB[3] = { face2, face3, face3 };

    // Apply 3 chamfer operations in permutation order
    hGroup prevH = extrudeH;
    hGroup opH[3];
    for(int i = 0; i < 3; i++) {
        int ei = perm[i];
        opH[i] = AddChamferGroup(prevH, edgeFaceA[ei], edgeFaceB[ei], offset);
        prevH = opH[i];
    }

    // Generate display items so the model renders properly in GUI
    Group *gLast = SK.GetGroup(opH[2]);
    if(!gLast->booleanFailed) {
        gLast->GenerateDisplayItems();
    }

    // Save to .slvs file
    Platform::Path savePath = Platform::Path::From(outputPath);
    SS.SaveToFile(savePath);
}

// CCC_012: LEFT+FRONT+TOP, permutation 012 — this is the PASSING baseline
TEST_CASE(tripleop_visual_save_012) {
    int perm[] = {0, 1, 2};
    BuildAndSaveTripleChamfer(FS_LEFT, FS_FRONT, FS_TOP, perm, 2.0,
        "/tmp/triple_chamfer_CCC_012.slvs");

    // Verify the file was created
    FILE *f = fopen("/tmp/triple_chamfer_CCC_012.slvs", "r");
    CHECK_TRUE(f != nullptr);
    if(f) fclose(f);
}

//=============================================================================
// DIAGNOSTIC: Dump backfacing triangle info for PASSING vs FAILING _012 cases
// This helps identify WHICH surfaces produce wrong normals.
//=============================================================================
static inline void DumpBackfacingDiag(
    const char *label,
    FaceSpec f1, FaceSpec f2, FaceSpec f3,
    int perm[3], double offset)
{
    hGroup extrudeH = CreateBoxExtrude();

    hEntity face1 = GetFace(extrudeH, f1);
    hEntity face2 = GetFace(extrudeH, f2);
    hEntity face3 = GetFace(extrudeH, f3);

    hEntity edgeFaceA[3] = { face1, face1, face2 };
    hEntity edgeFaceB[3] = { face2, face3, face3 };

    hGroup prevH = extrudeH;
    hGroup opH[3];
    for(int i = 0; i < 3; i++) {
        int ei = perm[i];
        opH[i] = AddChamferGroup(prevH, edgeFaceA[ei], edgeFaceB[ei], offset);
        prevH = opH[i];
    }

    Group *gLast = SK.GetGroup(opH[2]);
    if(gLast->booleanFailed) {
        fprintf(stderr, "DIAG[%s]: booleanFailed!\n", label);
        return;
    }
    gLast->GenerateDisplayItems();
    SMesh *m = &gLast->displayMesh;

    Vector boxCenter = Vector::From(10, 10, 10);
    int backfaceCount = 0;
    fprintf(stderr, "DIAG[%s]: %d triangles total\n", label, m->l.n);
    for(int ti = 0; ti < m->l.n; ti++) {
        STriangle *tr = &m->l[ti];
        Vector normal = tr->Normal().WithMagnitude(1);
        Vector effNormal = tr->EffectiveNormal().WithMagnitude(1);
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        double dot = effNormal.Dot(centroid.Minus(boxCenter));
        bool hasFlipFlag = (tr->flags & STriangle::FLAG_FLIP_DISPLAY_NORMAL) != 0;
        uint32_t face = tr->meta.face;
        uint32_t surfH = face & 0xFFFF;
        uint32_t groupH = (face >> 16) & 0xFFFF;
        if(dot < -0.01) {
            backfaceCount++;
            fprintf(stderr, "  BACKFACE[%d]: face=0x%08x (surf=%u grp=%u) "
                "centroid=(%.2f,%.2f,%.2f) "
                "normal=(%.3f,%.3f,%.3f) effNormal=(%.3f,%.3f,%.3f) "
                "flipFlag=%d dot=%.4f\n",
                ti, face, surfH, groupH,
                centroid.x, centroid.y, centroid.z,
                normal.x, normal.y, normal.z,
                effNormal.x, effNormal.y, effNormal.z,
                (int)hasFlipFlag, dot);
            fprintf(stderr, "    vertices: a=(%.4f,%.4f,%.4f) b=(%.4f,%.4f,%.4f) c=(%.4f,%.4f,%.4f)\n",
                tr->a.x, tr->a.y, tr->a.z,
                tr->b.x, tr->b.y, tr->b.z,
                tr->c.x, tr->c.y, tr->c.z);
            Vector cross = (tr->b.Minus(tr->a)).Cross(tr->c.Minus(tr->a));
            fprintf(stderr, "    cross=(%.4f,%.4f,%.4f) mag=%.6f\n", cross.x, cross.y, cross.z, cross.Magnitude());
        }
    }
    fprintf(stderr, "DIAG[%s]: %d backfacing out of %d\n", label, backfaceCount, m->l.n);

    // Per-face summary: how many triangles per face?
    fprintf(stderr, "DIAG[%s]: === Per-face triangle counts ===\n", label);
    struct FaceInfo { uint32_t face; int count; int backfaceCount; };
    FaceInfo faces[64];
    int nFaces = 0;
    for(int ti = 0; ti < m->l.n; ti++) {
        uint32_t face = m->l[ti].meta.face;
        int fi = -1;
        for(int j = 0; j < nFaces; j++) {
            if(faces[j].face == face) { fi = j; break; }
        }
        if(fi < 0) { fi = nFaces++; faces[fi] = {face, 0, 0}; }
        faces[fi].count++;
        // check backfacing
        STriangle *tr = &m->l[ti];
        Vector effNormal = tr->EffectiveNormal().WithMagnitude(1);
        Vector centroid = tr->a.Plus(tr->b).Plus(tr->c).ScaledBy(1.0/3.0);
        double dot = effNormal.Dot(centroid.Minus(boxCenter));
        if(dot < -0.01) faces[fi].backfaceCount++;
    }
    for(int fi = 0; fi < nFaces; fi++) {
        uint32_t surfH = faces[fi].face & 0xFFFF;
        uint32_t groupH = (faces[fi].face >> 16) & 0xFFFF;
        fprintf(stderr, "  face=0x%08x (surf=%u grp=%u): %d tris, %d backfacing\n",
            faces[fi].face, surfH, groupH, faces[fi].count, faces[fi].backfaceCount);
    }
}

TEST_CASE(tripleop_backface_diag_pass) {
    // LEFT+FRONT+TOP _012 — PASSES backface check
    int perm[] = {0, 1, 2};
    DumpBackfacingDiag("LFT_012_PASS", FS_LEFT, FS_FRONT, FS_TOP, perm, 2.0);
}

TEST_CASE(tripleop_backface_diag_fail1) {
    // RIGHT+FRONT+TOP _012 — FAILS backface check
    int perm[] = {0, 1, 2};
    DumpBackfacingDiag("RFT_012_FAIL", FS_RIGHT, FS_FRONT, FS_TOP, perm, 2.0);
}

TEST_CASE(tripleop_backface_diag_fail2) {
    // LEFT+FRONT+BOTTOM _012 — FAILS backface check
    int perm[] = {0, 1, 2};
    DumpBackfacingDiag("LFB_012_FAIL", FS_LEFT, FS_FRONT, FS_BOTTOM, perm, 2.0);
}

TEST_CASE(tripleop_backface_diag_fail3) {
    // LEFT+FRONT+TOP _021 — FAILS backface check (non-degenerate)
    int perm[] = {0, 2, 1};
    DumpBackfacingDiag("LFT_021_FAIL", FS_LEFT, FS_FRONT, FS_TOP, perm, 2.0);
}

// CCC_102: LEFT+FRONT+TOP, permutation 102 — known to produce LEAKS (TDD RED)
TEST_CASE(tripleop_visual_save_102) {
    int perm[] = {1, 0, 2};
    BuildAndSaveTripleChamfer(FS_LEFT, FS_FRONT, FS_TOP, perm, 2.0,
        "/tmp/triple_chamfer_CCC_102.slvs");

    FILE *f = fopen("/tmp/triple_chamfer_CCC_102.slvs", "r");
    CHECK_TRUE(f != nullptr);
    if(f) fclose(f);
}

// CCC_021: LEFT+FRONT+TOP, permutation 021 — known BACKFACING issue (TDD RED)
TEST_CASE(tripleop_visual_save_021) {
    int perm[] = {0, 2, 1};
    BuildAndSaveTripleChamfer(FS_LEFT, FS_FRONT, FS_TOP, perm, 2.0,
        "/tmp/triple_chamfer_CCC_021.slvs");

    FILE *f = fopen("/tmp/triple_chamfer_CCC_021.slvs", "r");
    CHECK_TRUE(f != nullptr);
    if(f) fclose(f);
}

// CCC_201: LEFT+FRONT+TOP, permutation 201 — known to produce LEAKS (TDD RED)
TEST_CASE(tripleop_visual_save_201) {
    int perm[] = {2, 0, 1};
    BuildAndSaveTripleChamfer(FS_LEFT, FS_FRONT, FS_TOP, perm, 2.0,
        "/tmp/triple_chamfer_CCC_201.slvs");

    FILE *f = fopen("/tmp/triple_chamfer_CCC_201.slvs", "r");
    CHECK_TRUE(f != nullptr);
    if(f) fclose(f);
}
