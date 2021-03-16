// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "open3d/geometry/Bvh.h"

#include "open3d/geometry/Line3D.h"
#include "tests/UnitTest.h"

using namespace open3d::geometry;
using namespace Eigen;
using namespace ::testing;

using AABB = AxisAlignedBoundingBox;
using BoxVec = std::vector<AABB>;

using v_t = Eigen::Vector3d;
using intr_t = std::tuple<v_t, v_t, std::vector<size_t>>;

AABB ShiftBox(Eigen::Index cardinal, double amount) {
    Vector3d v;
    v[cardinal] = amount;
    AABB box{{}, {1, 1, 1}};
    box.min_bound_ += v;
    box.max_bound_ += v;
    return box;
}

AABB XBox(double shift) { return ShiftBox(0, shift); }

AABB YBox(double shift) { return ShiftBox(1, shift); }

AABB ZBox(double shift) { return ShiftBox(2, shift); }

/// Simple primitive for testing the BVH and its components
struct Sphere {
    Vector3d center;
    double radius;

    AxisAlignedBoundingBox GetBox() const {
        Vector3d r{radius, radius, radius};
        return {center - r, center + r};
    }
};

using Spheres = std::vector<Sphere>;

std::unique_ptr<bvh::BvhNode> make_node_box(Vector3d min,
                                                    Vector3d max) {
    BoxVec vec{{min, max}};
    auto node = std::make_unique<bvh::BvhNode>();
    node->indices_.push_back(0);
    node->SetBox([&vec](size_t i) { return vec[i]; });
    return node;
}

namespace open3d {
namespace tests {

TEST(BvhNode, EmptyNodeCreate) {
    bvh::BvhNode node;

    EXPECT_TRUE(node.indices_.empty());
    EXPECT_TRUE(node.IsLeaf());
    EXPECT_TRUE(node.Box().IsEmpty());
}

TEST(BvhNode, IsLeaf) {
    bvh::BvhNode node;
    node.left_ = std::make_unique<bvh::BvhNode>();
    EXPECT_FALSE(node.IsLeaf());
}

TEST(BvhNode, SetBoxLeaf) {
    BoxVec vec{{{-10, -1, -1}, {10, 1, 1}}, {{-1, -7, -1}, {1, 7, 1}}};
    bvh::BvhNode node;
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox([&vec](size_t i) { return vec[i]; });

    ExpectEQ(node.Box().min_bound_, {-10, -7, -1});
    ExpectEQ(node.Box().max_bound_, {10, 7, 1});
}

TEST(BvhNode, SetBoxNonLeaf) {
    bvh::BvhNode node;
    BoxVec v;
    node.left_ = make_node_box({-5, -1, -1}, {5, 1, 1});
    node.right_ = make_node_box({-1, -6, -1}, {1, 6, 1});
    node.SetBox([&v](size_t i) { return v[i]; });

    ExpectEQ(node.Box().min_bound_, {-5, -6, -1});
    ExpectEQ(node.Box().max_bound_, {5, 6, 1});
}

TEST(BvhNode, SplitNonLeafNoOp) {
    bvh::BvhNode node;
    BoxVec v;
    node.left_ = make_node_box({-5, -1, -1}, {5, 1, 1});
    node.right_ = make_node_box({-1, -6, -1}, {1, 6, 1});

    auto l = node.left_.get();
    auto r = node.right_.get();

    bvh::BvhNode::SplitLeafObjMean(node, [&v](size_t i) { return v[i]; }, bvh::SplitOptions::None());

    EXPECT_EQ(l, node.left_.get());
    EXPECT_EQ(r, node.right_.get());
}

TEST(BvhNode, SplitLeafMinPrimitives) {
    bvh::BvhNode node;
    BoxVec v{{{4, 0, 0}, {5, 1, 1}}, {{0, 6, 0}, {1, 7, 1}}};
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox([&v](size_t i) { return v[i]; });

    bvh::SplitOptions options{};
    options.min_primitives = 2;
    bvh::BvhNode::SplitLeafObjMean(node, [&v](size_t i) { return v[i]; }, options);

    EXPECT_EQ(nullptr, node.left_);
    EXPECT_EQ(nullptr, node.right_);
}

TEST(BvhNode, SplitLeafTwoEntites) {
    bvh::BvhNode node;
    BoxVec v{{{4, 0, 0}, {5, 1, 1}}, {{0, 6, 0}, {1, 7, 1}}};
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox([&v](size_t i) { return v[i]; });

    bvh::BvhNode::SplitLeafObjMean(node, [&v](size_t i) { return v[i]; }, bvh::SplitOptions::None());

    EXPECT_FALSE(node.IsLeaf());
    EXPECT_EQ(node.left_->indices_.size(), 1);
    EXPECT_EQ(node.right_->indices_.size(), 1);
    EXPECT_TRUE(node.indices_.empty());
    ExpectEQ(node.left_->Box().min_bound_, {4, 0, 0});
    ExpectEQ(node.left_->Box().max_bound_, {5, 1, 1});
    ExpectEQ(node.right_->Box().min_bound_, {0, 6, 0});
    ExpectEQ(node.right_->Box().max_bound_, {1, 7, 1});
}

TEST(BvhNode, VolumeAbort) {
    bvh::BvhNode node;
    BoxVec v{{{0, 0, 0}, {1, 1, 1}}, {{0, 0, 0}, {1, 0.9, 1}}};
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox([&v](size_t i) { return v[i]; });

    bvh::SplitOptions options{};
    options.volume_ratio = 1.5;
    bvh::BvhNode::SplitLeafObjMean(node, [&v](size_t i) { return v[i]; }, options);

    EXPECT_EQ(nullptr, node.left_);
    EXPECT_EQ(nullptr, node.right_);
}

TEST(BvhNode, VolumeOk) {
    bvh::BvhNode node;
    BoxVec v{{{0, 0, 0}, {1, 1, 1}}, {{0, 0, 0}, {1, 0.9, 1}}};
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox([&v](size_t i) { return v[i]; });

    bvh::SplitOptions options{};
    options.volume_ratio = 1.95;
    bvh::BvhNode::SplitLeafObjMean(node, [&v](size_t i) { return v[i]; }, options);

    EXPECT_FALSE(node.IsLeaf());
}

TEST(BvhNode, SplitLeafCardinalY) {
    bvh::BvhNode node;
    BoxVec v{YBox(-1), YBox(5), YBox(5.5)};
    for (size_t i = 0; i < 3; ++i) node.indices_.push_back(i);
    node.SetBox([&v](size_t i) { return v[i]; });

    bvh::SplitOptions options;
    options.min_primitives = 2;  // Need this or it will further split right
    bvh::BvhNode::SplitLeafObjMean(node, [&v](size_t i) { return v[i]; }, options);

    EXPECT_EQ(node.left_->indices_.size(), 1);
    EXPECT_EQ(node.right_->indices_.size(), 2);
}

TEST(BvhNode, SplitLeafCardinalZ) {
    bvh::BvhNode node;
    BoxVec v{ZBox(-1), ZBox(5), ZBox(5.5)};
    for (size_t i = 0; i < 3; ++i) node.indices_.push_back(i);
    node.SetBox([&v](size_t i) { return v[i]; });

    bvh::SplitOptions options;
    options.min_primitives = 2;  // Need this or it will further split right
    bvh::BvhNode::SplitLeafObjMean(node, [&v](size_t i) { return v[i]; }, options);

    EXPECT_EQ(node.left_->indices_.size(), 1);
    EXPECT_EQ(node.right_->indices_.size(), 2);
}

TEST(BvhNode, SplitLeafCardinalX) {
    bvh::BvhNode node;
    BoxVec v{XBox(-1), XBox(5), XBox(5.5)};
    for (size_t i = 0; i < 3; ++i) node.indices_.push_back(i);
    node.SetBox([&v](size_t i) { return v[i]; });

    bvh::SplitOptions options;
    options.min_primitives = 2;  // Need this or it will further split right
    bvh::BvhNode::SplitLeafObjMean(node, [&v](size_t i) { return v[i]; }, options);

    EXPECT_EQ(node.left_->indices_.size(), 1);
    EXPECT_EQ(node.right_->indices_.size(), 2);
}

TEST(Bvh, CreateTopDown) {
    Spheres spheres {
        Sphere{{5, 0, 0}, 1},
        Sphere{{6, 1, 0}, 1},
        Sphere{{0, 7, 0}, 1},
        Sphere{{0, 9, 0}, 1},
        Sphere{{0, 0, 8}, 1},
        Sphere{{0, 0, 6}, 1},
    };

    // Create the BVH
    auto bvh = Bvh::CreateTopDown(
            [&](size_t i) { return spheres[i].GetBox(); },
            spheres.size(),
            bvh::SplitOptions::None());

    // Find the accumulated bounding box
    AABB expected_box{};
    for (const auto& s : spheres) expected_box += s.GetBox();

    // Traverse the hierarchy to gather information about the nodes and
    // primitives
    size_t total_primitives = 0;
    size_t leaf_nodes = 0;
    std::function<void(const bvh::BvhNode&)> recurse;
    recurse = [&](const bvh::BvhNode& n) {
        if (n.IsLeaf()) {
            total_primitives += n.indices_.size();
            leaf_nodes++;
        } else {
            recurse(*n.left_);
            recurse(*n.right_);
        }
    };
    recurse(bvh->Root());

    EXPECT_EQ(6, total_primitives);
    EXPECT_EQ(6, leaf_nodes);
    ExpectEQ(expected_box.max_bound_, bvh->Root().Box().max_bound_);
    ExpectEQ(expected_box.min_bound_, bvh->Root().Box().min_bound_);
}

/*
 * Possible intersection tests
 * This parameterized test uses the Ray3D intersection test to exercise the
 * traversal method and check that potential intersections are correctly
 * pruned by the hierarchy.
 */
class BvhIntersectionTests : public TestWithParam<intr_t> {};

TEST_P(BvhIntersectionTests, PossibleIntersections) {
    Ray3D ray{std::get<0>(GetParam()), std::get<1>(GetParam())};
    auto expected = std::get<2>(GetParam());

    Spheres spheres {
        Sphere{{5, 0, 0}, 1},
        Sphere{{6, 0, 0}, 1},
        Sphere{{0, 7, 0}, 1},
        Sphere{{0, 9, 0}, 1},
        Sphere{{0, 0, 8}, 1},
        Sphere{{0, 0, 6}, 1}
    };

    // Create the BVH
    auto bvh = Bvh::CreateTopDown(
            [&](size_t i) { return spheres[i].GetBox(); },
            spheres.size(),
            bvh::SplitOptions::None());

    // Find the possible intersections
    auto results = bvh->PossibleIntersections(
            [&ray](const AABB& bx) { return ray.SlabAABB(bx).has_value(); });

    // Verify that the possible intersections contain the expected indices
    EXPECT_EQ(expected.size(), results.size());
    for (auto i : expected) {
        EXPECT_TRUE(std::find(results.begin(), results.end(), i) !=
                    results.end());
    }
}

INSTANTIATE_TEST_CASE_P(Tests,
                        BvhIntersectionTests,
                        Values(intr_t{{0, 0, 0}, {1, 0, 0}, {0, 1}},
                               intr_t{{0, 0, 0}, {0, 0, 1}, {5, 4}},
                               intr_t{{0, 0, 0}, {0, 1, 0}, {2, 3}},
                               intr_t{{6, 0, 0}, {-6, 0, 7}, {0, 1, 4, 5}},
                               intr_t{{0, 9, 0}, {0, -9, 7}, {2, 3, 4, 5}}));

/*
 * Use the point to AABB distance functions to test the closest and furthest
 * primitive check pruning
 */

TEST(BvhDistanceTests, TraverseClosest) {
    using namespace std;
    Vector3d v{10, 0, 0};

    auto closest = [&v](const AABB& box) {
        // The closest point on the box is found by clamping the test point to
        // the bounds
        auto cx = max(min(v.x(), box.max_bound_.x()), box.min_bound_.x());
        auto cy = max(min(v.y(), box.max_bound_.y()), box.min_bound_.y());
        auto cz = max(min(v.z(), box.max_bound_.z()), box.min_bound_.z());
        Vector3d c{cx, cy, cz};
        return (c - v).norm();
    };

    auto furthest = [&v](const AABB& box) {
        // The furthest point on the AABB is always going to be one of the
        // eight corners
        double dist = 0;
        for (const auto& p : box.GetBoxPoints()) {
            dist = std::max((p - v).norm(), dist);
        }
        return dist;
    };

    Spheres spheres{Sphere{{5, 0, 0}, 1},
    Sphere{{6, 0, 0}, 1},
    Sphere{{0, 7, 0}, 1},
    Sphere{{0, 9, 0}, 1},
    Sphere{{0, 0, 8}, 1},
    Sphere{{0, 0, 6}, 1}};

    // Create the BVH
    auto bvh = Bvh::CreateTopDown(
            [&](size_t i) { return spheres[i].GetBox(); },
            spheres.size(),
            bvh::SplitOptions::None());

    auto results = bvh->PossibleClosest(closest, furthest);
}

}  // namespace tests
}  // namespace open3d