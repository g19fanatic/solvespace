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
