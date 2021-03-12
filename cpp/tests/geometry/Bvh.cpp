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

TEST(BvhNode, SplitNonLeafNoOp) {
    bvh::BvhNode<Sphere> node;
    BoxVec v;
    node.left_ = make_node_box({-5, -1, -1}, {5, 1, 1});
    node.right_ = make_node_box({-1, -6, -1}, {1, 6, 1});

    auto l = node.left_.get();
    auto r = node.right_.get();

    bvh::BvhNode<Sphere>::SplitLeafObjMean(node, v, bvh::SplitOptions::None());

    EXPECT_EQ(l, node.left_.get());
    EXPECT_EQ(r, node.right_.get());
}

TEST(BvhNode, SplitLeafMinPrimitives) {
    bvh::BvhNode<Sphere> node;
    BoxVec v{{{4, 0, 0}, {5, 1, 1}}, {{0, 6, 0}, {1, 7, 1}}};
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox(v);

    bvh::SplitOptions options{};
    options.min_primitives = 2;
    bvh::BvhNode<Sphere>::SplitLeafObjMean(node, v, options);

    EXPECT_EQ(nullptr, node.left_);
    EXPECT_EQ(nullptr, node.right_);
}

TEST(BvhNode, SplitLeafTwoEntites) {
    bvh::BvhNode<Sphere> node;
    BoxVec v{{{4, 0, 0}, {5, 1, 1}}, {{0, 6, 0}, {1, 7, 1}}};
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox(v);

    bvh::BvhNode<Sphere>::SplitLeafObjMean(node, v, bvh::SplitOptions::None());

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
    bvh::BvhNode<Sphere> node;
    BoxVec v{{{0, 0, 0}, {1, 1, 1}}, {{0, 0, 0}, {1, 0.9, 1}}};
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox(v);

    bvh::SplitOptions options{};
    options.volume_ratio = 1.5;
    bvh::BvhNode<Sphere>::SplitLeafObjMean(node, v, options);

    EXPECT_EQ(nullptr, node.left_);
    EXPECT_EQ(nullptr, node.right_);
}

TEST(BvhNode, VolumeOk) {
    bvh::BvhNode<Sphere> node;
    BoxVec v{{{0, 0, 0}, {1, 1, 1}}, {{0, 0, 0}, {1, 0.9, 1}}};
    node.indices_.push_back(0);
    node.indices_.push_back(1);
    node.SetBox(v);

    bvh::SplitOptions options{};
    options.volume_ratio = 1.95;
    bvh::BvhNode<Sphere>::SplitLeafObjMean(node, v, options);

    EXPECT_FALSE(node.IsLeaf());
}

TEST(BvhNode, SplitLeafCardinalY) {
    bvh::BvhNode<Sphere> node;
    BoxVec v{YBox(-1), YBox(5), YBox(5.5)};
    for (size_t i = 0; i < 3; ++i) node.indices_.push_back(i);
    node.SetBox(v);

    bvh::SplitOptions options;
    options.min_primitives = 2;  // Need this or it will further split right
    bvh::BvhNode<Sphere>::SplitLeafObjMean(node, v, options);

    EXPECT_EQ(node.left_->indices_.size(), 1);
    EXPECT_EQ(node.right_->indices_.size(), 2);
}

TEST(BvhNode, SplitLeafCardinalZ) {
    bvh::BvhNode<Sphere> node;
    BoxVec v{ZBox(-1), ZBox(5), ZBox(5.5)};
    for (size_t i = 0; i < 3; ++i) node.indices_.push_back(i);
    node.SetBox(v);

    bvh::SplitOptions options;
    options.min_primitives = 2;  // Need this or it will further split right
    bvh::BvhNode<Sphere>::SplitLeafObjMean(node, v, options);

    EXPECT_EQ(node.left_->indices_.size(), 1);
    EXPECT_EQ(node.right_->indices_.size(), 2);
}

TEST(BvhNode, SplitLeafCardinalX) {
    bvh::BvhNode<Sphere> node;
    BoxVec v{XBox(-1), XBox(5), XBox(5.5)};
    for (size_t i = 0; i < 3; ++i) node.indices_.push_back(i);
    node.SetBox(v);

    bvh::SplitOptions options;
    options.min_primitives = 2;  // Need this or it will further split right
    bvh::BvhNode<Sphere>::SplitLeafObjMean(node, v, options);

    EXPECT_EQ(node.left_->indices_.size(), 1);
    EXPECT_EQ(node.right_->indices_.size(), 2);
}

TEST(Bvh, CreateTopDown) { EXPECT_TRUE(false); }

TEST(Bvh, PossibleIntersections) { EXPECT_TRUE(false); }

}  // namespace tests
}  // namespace open3d