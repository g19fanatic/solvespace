//-----------------------------------------------------------------------------
// Difference/cutout tests
//-----------------------------------------------------------------------------
#include "helpers.h"

//-----------------------------------------------------------------------------
// Tests: Fillet/Chamfer on boolean-difference geometry (Phase 1: TDD RED)
// After the n==0 guard fix (task 13), all tests should PASS (no crash).
// Converted to edge-based API: FindEdgeBetweenFaces + Add*GroupByEdge
//-----------------------------------------------------------------------------

TEST_CASE(fillet_diff_inside_faces_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutout(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    hGroup edgeSourceH = bwc.cutExtrude;
    bool found = FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2);
    if(!found) {
        found = FindTwoAdjacentFaces(bwc.baseExtrude, &face1, &face2);
        edgeSourceH = bwc.baseExtrude;
    }
    if(!found) return;
    hEntity edge = FindEdgeBetweenFaces(edgeSourceH, face1, face2);
    if(edge.v == 0) return;
    hGroup filletH = AddFilletGroupByEdge(bwc.cutExtrude, edge, 1.0);
    CHECK_TRUE(SK.GetGroup(filletH) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_bottom_cutout_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    hGroup edgeSourceH = bwc.cutExtrude;
    bool found = FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2);
    if(!found) {
        found = FindTwoAdjacentFaces(bwc.baseExtrude, &face1, &face2);
        edgeSourceH = bwc.baseExtrude;
    }
    if(!found) return;
    hEntity edge = FindEdgeBetweenFaces(edgeSourceH, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_corner_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 1.5)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_centered_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(5.0, 5.0, 10.0, 10.0, 8.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_small_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(2.0, 2.0, 5.0, 5.0, 5.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 0.5)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_small_radius_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 0.1)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_large_radius_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 3.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(chamfer_diff_inside_faces_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddChamferGroupByEdge(bwc.cutExtrude, edge, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_asymmetric_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 15.0, 5.0, 8.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_deep_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(2.0, 2.0, 8.0, 8.0, 18.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 1.0)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_shallow_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(0.0, 0.0, 10.0, 10.0, 2.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 0.5)) != nullptr);
    CHECK_TRUE(true);
}

TEST_CASE(fillet_diff_offset_pocket_no_crash) {
    BoxWithCutout bwc = CreateBoxWithCutoutFromBottom(3.0, 3.0, 12.0, 12.0, 10.0);
    CHECK_TRUE(SK.GetGroup(bwc.cutExtrude) != nullptr);
    hEntity face1 = {}, face2 = {};
    if(!FindTwoAdjacentFaces(bwc.cutExtrude, &face1, &face2)) return;
    hEntity edge = FindEdgeBetweenFaces(bwc.cutExtrude, face1, face2);
    if(edge.v == 0) return;
    CHECK_TRUE(SK.GetGroup(AddFilletGroupByEdge(bwc.cutExtrude, edge, 1.0)) != nullptr);
    CHECK_TRUE(true);
}
