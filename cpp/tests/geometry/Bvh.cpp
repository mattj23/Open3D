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

#include "tests/UnitTest.h"

using namespace open3d::geometry;
using namespace Eigen;
using namespace ::testing;

using AABB = AxisAlignedBoundingBox;
using BoxVec = std::vector<AABB>;

/// Simple primitive for testing the BVH and its components
struct Sphere {
    Vector3d center;
    double radius;

    AxisAlignedBoundingBox GetBox() const {
        Vector3d r{radius, radius, radius};
        return {center - r, center + r};
    }
};

std::unique_ptr<bvh::BvhNode<Sphere>> make_node_box(Vector3d min,
                                                    Vector3d max) {
    BoxVec vec{{min, max}};
    auto node = std::make_unique<bvh::BvhNode<Sphere>>();
    node->indices_.push_back(0);
    node->SetBox(vec);
    return node;
}

namespace open3d {
namespace tests {

TEST(BvhNode, EmptyNodeCreate) {
    bvh::BvhNode<Sphere> node;

    EXPECT_TRUE(node.indices_.empty());
    EXPECT_TRUE(node.IsLeaf());
    EXPECT_TRUE(node.Box().IsEmpty());
}

TEST(BvhNode, IsLeaf) {
    bvh::BvhNode<Sphere> node;
    node.left_ = std::make_unique<bvh::BvhNode<Sphere>>();
    EXPECT_FALSE(node.IsLeaf());
}

TEST(BvhNode, SetBoxLeaf) {
    BoxVec vec{{{-10, -1, -1}, {10, 1, 1}}, {{-1, -7, -1}, {1, 7, 1}}};
    bvh::BvhNode<Sphere> node;
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox(vec);

    ExpectEQ(node.Box().min_bound_, {-10, -7, -1});
    ExpectEQ(node.Box().max_bound_, {10, 7, 1});
}

TEST(BvhNode, SetBoxNonLeaf) {
    bvh::BvhNode<Sphere> node;
    BoxVec v;
    node.left_ = make_node_box({-5, -1, -1}, {5, 1, 1});
    node.right_ = make_node_box({-1, -6, -1}, {1, 6, 1});
    node.SetBox(v);

    ExpectEQ(node.Box().min_bound_, {-5, -6, -1});
    ExpectEQ(node.Box().max_bound_, {5, 6, 1});
}

TEST(BvhNode, SplitLeafTwoEntites) { EXPECT_TRUE(false); }

TEST(BvhNode, SplitLeafMinPrimitives) { EXPECT_TRUE(false); }

TEST(BvhNode, VolumeAbort) { EXPECT_TRUE(false); }

TEST(BvhNode, SplitLeafCardinalY) { EXPECT_TRUE(false); }

TEST(BvhNode, SplitLeafCardinalZ) { EXPECT_TRUE(false); }

TEST(BvhNode, SplitLeafCardinalX) { EXPECT_TRUE(false); }

TEST(Bvh, CreateTopDown) { EXPECT_TRUE(false); }

TEST(Bvh, PossibleIntersections) { EXPECT_TRUE(false); }

}  // namespace tests
}  // namespace open3d