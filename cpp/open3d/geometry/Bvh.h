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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <vector>

#include "open3d/geometry/BoundingVolume.h"
#include "open3d/geometry/Geometry.h"

namespace open3d {
namespace geometry {

/// \class BvhNode
///
/// This is a singular node in a binary bounding volume hierarchy which contains
/// a unique pointer to both a left and right child node. \tparam T
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

template <class T>
void BvhNode<T>::SetBox(const AxisAlignedBoundingBox& box) {
    box_ = box;
}


/// \class Bvh
///
/// \tparam T
template <class T>
class Bvh {
    using Node = BvhNode<T>;
    using BoxFn = std::function<AxisAlignedBoundingBox(T)>;
    using Container = std::shared_ptr<std::vector<T>>;
public:
    Bvh<T>(BoxFn to_box,
           std::shared_ptr<std::vector<T>> c) {
            primitives_ = c;
            box_func_ = to_box;
    };

    const Node& Root() const { return root_; }

    // Static Construction Methods
    static std::unique_ptr<Bvh<T>> CreateTopDown(BoxFn to_box,
                                                 Container primitives);

private:
    std::unique_ptr<Node> root_;

    BoxFn box_func_;
    Container primitives_;
};


template <class T>
std::unique_ptr<Bvh<T>> Bvh<T>::CreateTopDown(BoxFn to_box,
                                              Container primitives) {
    // Prepare the vector of indices and the vector of bounding boxes
    std::vector<size_t> indices;
    std::vector<AxisAlignedBoundingBox> boxes;

    indices.reserve(primitives->size());
    boxes.reserve(primitives->size());

    for (size_t i = 0; i < primitives->size(); ++i) {
        indices.push_back(i);
        boxes.push_back(to_box((*primitives)[i]));
    }

    auto ptr = std::make_unique<Bvh<T>>(to_box, primitives);
    ptr->root_ = std::make_unique<BvhNode<T>>();

    return ptr;
}

}  // namespace geometry
}  // namespace open3d
