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

#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open3d/geometry/BoundingVolume.h"
#include "open3d/geometry/Geometry.h"

namespace open3d {
namespace geometry {

/// \class BvhNode
///
/// This is a singular node in a binary bounding volume hierarchy which contains a unique
/// pointer to both a left and right child node.
/// \tparam T
template <class T>
class BvhNode {
public:

    const AxisAlignedBoundingBox& Box() const { return box_; }
    void SetBox(const AxisAlignedBoundingBox& box);

    std::unique_ptr<BvhNode<T>> left;
    std::unique_ptr<BvhNode<T>> right;

   const std::vector<size_t>& Primitives() const { return primitive_indices_; }

private:
    AxisAlignedBoundingBox box_;
    std::vector<size_t> primitive_indices_;
};

/// \class Bvh
///
/// \tparam T
template <class T>
class Bvh {
    using Container = std::shared_ptr<std::vector<T>>;
    using BoxFunc = std::function<AxisAlignedBoundingBox(T)>;
    using Node = BvhNode<T>;

public:
    Bvh<T>(BoxFunc to_box, Container c);

    const Node& Root() const { return root_; }

private:
    Node root_;

    BoxFunc box_func_;
    Container primitives_;
};

}

}
