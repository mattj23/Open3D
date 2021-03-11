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
#include <memory>
#include <vector>

#include "open3d/geometry/BoundingVolume.h"
#include "open3d/geometry/Geometry.h"
#include "open3d/geometry/Line3D.h"
#include "open3d/utility/Optional.h"

namespace open3d {
namespace geometry {

namespace bvh {
using BoxVec = std::vector<AxisAlignedBoundingBox>;

struct SplitOptions {
    utility::optional<size_t> min_primitives;
    utility::optional<double> volume_ratio;

    static SplitOptions None() { return {{}, {}}; }
};

/// \class BvhNode
///
/// This is a singular node in a binary bounding volume hierarchy which contains
/// a unique pointer to both a left and right child node. Each node contains a
/// vector of indices, which refer to the element position of primitives in the
/// container vector used during construction of the BVH. The indices contained
/// in this node are refer to the specific primitives which are contained within
/// the bounding box of this specific node.
///
/// A node may *either* have child nodes *or* have child indices, it may not
/// have both. Nodes with indices are leaf nodes.  Nodes without will have
/// *both* left and right child nodes.
///
/// This paradigm diverges from some BVH implementations where there is a 1:1
/// correspondence between a node and a primitive. What this allows is for nodes
/// that contain multiple primitives, which can become more efficient in cases
/// where the additional layers of bounding box checks near leaves can be more
/// work than simply checking multiple primitives. \tparam T
template <class T>
class BvhNode {
public:
    const AxisAlignedBoundingBox& Box() const { return box_; }

    void SetBox(const BoxVec& boxes);

    /// \brief Returns whether or not this node is a leaf node
    bool IsLeaf() const { return left_ == nullptr; }

    /// \brief A pointer to the left child of the node. Will be nullptr if this
    /// node is a leaf node.
    std::unique_ptr<BvhNode<T>> left_;

    /// \brief A pointer to the right child of the node. Will be nullptr if this
    /// node is a leaf node.
    std::unique_ptr<BvhNode<T>> right_;

    /// \brief A vector containing the indices of the primitives contained
    /// within this node. The indices refer to positions in the vector of
    /// original primitives used to create the BVH.
    std::vector<size_t> indices_;

    /// \brief Splits a leaf node along the longest dimension of the bounding
    /// box at the mean of the child bounding box centroids, producing two child
    /// nodes and moving all indices into these children.
    ///
    /// \details This is a static method because it is not inherently a feature
    /// of a BVH node, but rather an operation performed during top-down
    /// construction. Bottom-up construction, on the other hand, does not
    /// perform splits on nodes.
    ///
    /// This method will no-op on nodes which are not leaves (and thus do not
    /// have child primitives)
    static void SplitLeafObjMean(BvhNode<T>& node,
                                 const BoxVec& boxes,
                                 const SplitOptions& options);

private:
    AxisAlignedBoundingBox box_;
};

template <class T>
void BvhNode<T>::SplitLeafObjMean(BvhNode<T>& node,
                                  const BoxVec& boxes,
                                  const SplitOptions& options) {
    /* This is an operation used in the top-down construction of a BVH and
     * involves finding the longest cardinal direction of the node, splitting it
     * in half, and then transferring the primitives into one node or the other
     * based on where they sit in space.
     */

    // First we will check to see if we should abort the splitting of this node.
    // An indices_ size of 0 means this is not a leaf node, and should not be
    // split, while a size of 1 means there's nothing to split. Alternately, if
    // the options given have a minimum primitives per node value, we should
    // check against it before we decide to split. If any of these conditions
    // for abandoning the split exist we simply return out of the function
    // before doing anything.
    if (options.min_primitives.has_value()) {
        if (node.indices_.size() < options.min_primitives.value()) return;
    } else if (node.indices_.size() <= 1) {
        return;
    }

    if (node.indices_.size() == 2) {
        // There is a special case for splitting a node which only has two items
        // in it, and that is to simply move one element into each of the
        // children, there is no need to perform the complex splitting
        // operation.
        node.left_ = std::make_unique<BvhNode<T>>();
        node.right_ = std::make_unique<BvhNode<T>>();
        node.left_->indices_.push_back(node.indices_.front());
        node.right_->indices_.push_back(node.indices_.back());

    } else {
        // If there are more than two primitives in the node, we will perform
        // the split based on the object means.  The split direction will come
        // from the longest cardinal direction of the node's bounding box.
        auto extent = node.box_.GetExtent();
        Eigen::Index cardinal;
        if (extent.x() > extent.y() && extent.x() > extent.z()) {
            // Longest in X direction
            cardinal = 0;
        } else if (extent.y() > extent.z()) {
            // Longest in Y direction
            cardinal = 1;
        } else {
            // Longest in Z direction
            cardinal = 2;
        }

        // The object mean is the mean of the centroids in the cardinal
        // direction
        double sum = 0;
        for (auto i : node.indices_) {
            sum += boxes[i].GetCenter()[cardinal];
        }
        auto split = sum / node.indices_.size();

        // Now we can create the two child nodes and push the indices of the
        // primitives contained by this node into the appropriate child node.
        node.left_ = std::make_unique<BvhNode<T>>();
        node.right_ = std::make_unique<BvhNode<T>>();
        for (auto i : node.indices_) {
            if (boxes[i].GetCenter()[cardinal] < split) {
                node.left_->indices_.push_back(i);
            } else {
                node.right_->indices_.push_back(i);
            }
        }
    }

    // At this point we have not finalized the split; the indices of the
    // primitives exist in both the parent node and the child nodes. Here we
    // can check for conditions that would make us want to abandon the split.
    node.left_->SetBox(boxes);
    node.right_->SetBox(boxes);

    // Check by how much the volume has changed as a fraction of the original
    // volume. For instance, if two identically sized primitives were both
    // contained in this node, than both of the child nodes will have the same
    // volume as the parent node, and the combined volume of the children will
    // be 2.0 x the parent. This might be a reason to roll back the split.
    if (options.volume_ratio.has_value()) {
        auto volume = node.box_.Volume();

        if (node.left_->Box().Volume() + node.right_->Box().Volume() >
            volume * options.volume_ratio.value()) {
            node.left_ = nullptr;
            node.right_ = nullptr;
            return;
        }
    }

    // Finalize the split by clearing the local indices and recursively
    // splitting the child nodes.
    node.indices_.clear();
    BvhNode<T>::SplitLeafObjMean(*node.left_, boxes, options);
    BvhNode<T>::SplitLeafObjMean(*node.right_, boxes, options);
}

template <class T>
void BvhNode<T>::SetBox(const BoxVec& boxes) {
    box_.Clear();
    if (!IsLeaf()) {
        box_ += left_->Box();
        box_ += right_->Box();
    } else {
        for (auto i : indices_) {
            box_ += boxes[i];
        }
    }
}

}  // namespace bvh

/// \class Bvh
///
/// \summary This is a general purpose Bounding Volume Hierarchy for
/// non-specialized intersection and collision testing. It uses a template to
/// allow for user defined primitives; the only requirements are that the user
/// provide a function capable of retrieving an AxisAlignedBoundingBox from a
/// primitive and that the user supplies a shared_ptr to a std::vector
/// containing the primitives to be mapped.
///
/// \details This BVH implementation was designed with several competing
/// objectives. First, to be relatively performant for a variety of
/// applications. Second, to be flexible enough that it can be used with
/// user-defined underlying primitives.  Third, to be straightforward and
/// obvious in its use.
///
/// This is not a BVH that has been highly optimized for raytracing like Embree.
/// \tparam T
template <class T>
class Bvh {
    using Node = bvh::BvhNode<T>;
    using Container = std::shared_ptr<std::vector<T>>;

public:
    explicit Bvh<T>(Container primitives) { primitives_ = primitives; };

    /// \brief Get a const reference to the root node. Use this for short lived
    /// access to the root node if needed; it does not confer shared ownership.
    /// \return
    const Node& Root() const { return *root_; }

    /// \brief Check the BVH for the indices of all possible primitives that
    /// might intersect given some test function
    /// \param fn a std::function which tests bounding boxes to see if they
    /// intersect
    /// \return a vector of indicies contained by nodes which intersect with the
    /// test function

    /// \brief Check the BVH for the indices of all possible primitives that
    /// might intersect given a predicate for checking bounding boxes. This
    /// performs a depth first search against the predicate, returning a
    /// std::vector of the indices of primitives contained by positive testing
    /// bounding boxes.
    ///
    /// \code{.cpp}
    /// // This example shows the process of testing against a Line3D using the
    /// // slab intersection method.
    /// Line3D line{{0, 0, 0}, {1, 0, 0}};
    /// auto indices = bvh->PossibleIntersections(
    ///     [&line](const AxisAlignedBoundingBox& box) {
    ///         return line.SlabAABB(box).has_value();
    ///         });
    /// \endcode
    ///
    /// \tparam F a function or functor which takes an axis aligned bounding
    /// box and returns a bool
    /// \param fn a function which returns true if an axis aligned bounding box
    /// intersects.
    /// \return
    template <class F>
    std::vector<size_t> PossibleIntersections(F fn);

    /// \brief A simple top-down construction method that builds a BVH
    ///
    /// \param to_box a function to take a primitive of type T and return the
    /// axis aligned bounding box which encloses it
    /// \param primitives a shared_ptr to a vector<T> containing the primitives
    /// \return a constructed bounding volume hierarchy for type T
    template <class F>
    static std::unique_ptr<Bvh<T>> CreateTopDown(
            F box_fn, Container primitives, const bvh::SplitOptions& options);

private:
    std::unique_ptr<Node> root_;

    Container primitives_;
    std::vector<AxisAlignedBoundingBox> boxes_;
};

template <class T>
template <class F>
std::vector<size_t> Bvh<T>::PossibleIntersections(F fn) {
    using Node = bvh::BvhNode<T>;
    std::vector<size_t> indices;
    std::vector<std::reference_wrapper<Node>> nodes{*root_};

    while (!nodes.empty()) {
        auto working = nodes.back();
        nodes.pop_back();

        if (!fn(working.get().Box())) continue;

        if (working.get().IsLeaf()) {
            for (auto i : working.get().indices_) {
                indices.push_back(i);
            }
        } else {
            nodes.push_back(*working.get().left_);
            nodes.push_back(*working.get().right_);
        }
    }

    return indices;
}

template <class T>
template <class F>
std::unique_ptr<Bvh<T>> Bvh<T>::CreateTopDown(
        F box_fn, Container primitives, const bvh::SplitOptions& options) {
    std::vector<AxisAlignedBoundingBox> boxes;

    auto bvh = std::make_unique<Bvh<T>>(primitives);
    bvh->root_ = std::make_unique<Node>();
    bvh->root_->indices_.reserve(primitives->size());
    boxes.reserve(primitives->size());

    for (size_t i = 0; i < primitives->size(); ++i) {
        bvh->root_->indices_.push_back(i);
        boxes.push_back(box_fn((*primitives)[i]));
    }

    // Splitting
    bvh->root_->SetBox(boxes);
    bvh::BvhNode<T>::SplitLeafObjMean(*bvh->root_, boxes, options);

    return bvh;
}
}  // namespace geometry
}  // namespace open3d
