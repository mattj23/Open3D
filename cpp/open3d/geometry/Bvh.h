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

/// \struct SplitOptions
/// \brief This is a set of options which sets the behavior of top-down node
/// splitting operations during BVH construction.
struct SplitOptions {
    /// \brief A node must contain more than this amount of primitives in order
    /// to be considered for splitting. Note that a node may end up with less
    /// than this number of primitives as a result of a split (for example if
    /// min_primitives is 2, a node with 3 will split to create a child with 1).
    /// If this is left empty, the default size is 1.
    utility::optional<size_t> min_primitives;

    /// \brief A maximum acceptable child volume ratio to produce a split. For
    /// example, if a node contains two identically sized, overlapping
    /// primitives, splitting them into separate nodes will produce two children
    /// whose combined volume is now 2.0 x the original node volume. In general
    /// the smaller the ratio of the child volumes to the parent volume the
    /// better the split, with the worst possible case being 2.0. Set this value
    /// to specify a maximum permissible ratio, above which the split will not
    /// occur.
    utility::optional<double> volume_ratio;

    /// \brief Return an empty option set, such that the splitting algorithm
    /// will perform its default behavior.
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
class BvhNode {
public:
    /// \brief Get a reference to the axis aligned bounding box for this node
    const AxisAlignedBoundingBox& Box() const { return box_; }

    /// \brief Set the bounding box of the node based on its internal state. If
    /// the node is a leaf node, this will be the sum of the bounding boxes of
    /// all of the contained primitives. If it is not a leaf node, this box
    /// will be the sum of its left and right child boxes.
    template <class Fn>
    void SetBox(Fn box_func);

    /// \brief Returns whether or not this node is a leaf node
    bool IsLeaf() const { return left_ == nullptr; }

    /// \brief A pointer to the left child of the node. Will be nullptr if this
    /// node is a leaf node.
    std::unique_ptr<BvhNode> left_;

    /// \brief A pointer to the right child of the node. Will be nullptr if this
    /// node is a leaf node.
    std::unique_ptr<BvhNode> right_;

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
    /// \param node a temporary reference to a BvhNode<T> to be split
    /// \param boxes the std::vector of all bounding boxes where each element
    /// i is the bounding box associated with the primitive with index i in the
    /// Bvh's primitives vector
    /// \param options a SplitOptions struct which will set the behavior for the
    /// splitting operation
    template <class Fn>
    static void SplitLeafObjMean(BvhNode& node,
                                 Fn box_func,
                                 const SplitOptions& options);

private:
    /// Each BvhNode<T> has a bounding box associated with it, such that all of
    /// the combined primitives contained by all of the leaf nodes which are
    /// descendants of this node are contained within this box. In the case of
    /// a leaf node, this box is simply the sum of the bounding boxes for the
    /// primitives contained by the node.  For non-leaf nodes, this box is the
    /// sum of the left and right child nodes.
    AxisAlignedBoundingBox box_;
};

template <class Fn>
void BvhNode::SplitLeafObjMean(BvhNode& node,
                                  Fn box_func,
                                  const SplitOptions& options) {
    /* This is an operation used in the top-down construction of a BVH and
     * involves finding the longest cardinal direction of the node, splitting it
     * in half, and then transferring the primitives into one node or the other
     * based on where they sit in space.
     */
    if (!node.IsLeaf()) return;

    // First we will check to see if we should abort the splitting of this node.
    // An indices_ size of 0 means this is not a leaf node, and should not be
    // split, while a size of 1 means there's nothing to split. Alternately, if
    // the options given have a minimum primitives per node value, we should
    // check against it before we decide to split. If any of these conditions
    // for abandoning the split exist we simply return out of the function
    // before doing anything.
    if (options.min_primitives.has_value()) {
        if (node.indices_.size() <= options.min_primitives.value()) return;
    } else if (node.indices_.size() <= 1) {
        return;
    }

    if (node.indices_.size() == 2) {
        // There is a special case for splitting a node which only has two items
        // in it, and that is to simply move one element into each of the
        // children, there is no need to perform the complex splitting
        // operation.
        node.left_ = std::make_unique<BvhNode>();
        node.right_ = std::make_unique<BvhNode>();
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
            sum += box_func(i).GetCenter()[cardinal];
        }
        auto split = sum / static_cast<double>(node.indices_.size());

        // Now we can create the two child nodes and push the indices of the
        // primitives contained by this node into the appropriate child node.
        node.left_ = std::make_unique<BvhNode>();
        node.right_ = std::make_unique<BvhNode>();
        for (auto i : node.indices_) {
            if (box_func(i).GetCenter()[cardinal] < split) {
                node.left_->indices_.push_back(i);
            } else {
                node.right_->indices_.push_back(i);
            }
        }
    }

    // At this point we have not finalized the split; the indices of the
    // primitives exist in both the parent node and the child nodes. Here we
    // can check for conditions that would make us want to abandon the split.
    node.left_->SetBox(box_func);
    node.right_->SetBox(box_func);

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
    BvhNode::SplitLeafObjMean(*node.left_, box_func, options);
    BvhNode::SplitLeafObjMean(*node.right_, box_func, options);
}

template <class Fn>
void BvhNode::SetBox(Fn box_func) {
    box_.Clear();
    if (!IsLeaf()) {
        box_ += left_->Box();
        box_ += right_->Box();
    } else {
        for (auto i : indices_) {
            box_ += box_func(i);
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
/// containing the primitives to be mapped. Use this template class as a
/// foundation to create BVHs for different concrete types.
///
/// \details This BVH implementation is meant to be a general purpose building
/// block for creating bounding volume hierarchies for concrete types. It was
/// designed with a few overall objectives:
///     1. Be general enough to handle user implemented types
///     2. Be performance conscious; avoid major inefficiencies but not at the
///         cost of ceasing to be generalizable
///     3. Be easy to use, but only with minor performance concessions
///
/// To that end this BVH is generally suitable for geometric operations like
/// intersection tests, measurements, and basic collision pruning in simple
/// graphics and scientific applications, where it will perform many orders of
/// magnitude faster than exhaustive checking.  And though it will perform
/// reasonably well for ray/line intersections you should not expect to see
/// performance on par with highly optimized parallel/SIMD implementations like
/// Embree.
///
/// The implementation consists of a binary hierarchy of BvhNode<T> objects,
/// owned exclusively by the Bvh<T> object itself through the use of
/// std::unique_ptr. It also requires a std::shared_ptr to a std::vector of
/// the primitive type T. This type T is immaterial except that it must be
/// convertible to an AxisAlignedBoundingBox by a function provided to the
/// creation.
///
/// The std::shared_ptr to the vector of primitives *implies shared ownership*,
/// in that the Bvh is meaningless without these primitives in the exact state
/// and order that they were in when the Bvh was constructed. If the primitives
/// or the containing vector is altered while the Bvh is extant, the Bvh will
/// have no way of knowing, but will no longer accurately represent the
/// underlying data is was constructed for.
/// \tparam T the primitive type contained by the BVH
class Bvh {
public:

    /// \brief Get a const reference to the root node. Use this for short lived
    /// access to the root node if needed; it does not confer shared ownership.
    /// \return
    const bvh::BvhNode& Root() const { return *root_; }

    /// \brief Check the BVH for the indices of all possible primitives that
    /// might intersect given some test function
    /// \param fn a std::function which tests bounding boxes to see if they
    /// intersect
    /// \return a vector of indices contained by nodes which intersect with the
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

    template <class Fc, class Ff>
    std::vector<size_t> PossibleClosest(Fc closest, Ff furthest);

    /// \brief A simple top-down construction method that builds a BVH
    ///
    /// \param to_box a function to take a primitive of type T and return the
    /// axis aligned bounding box which encloses it
    /// \param primitives a shared_ptr to a vector<T> containing the primitives
    /// \param options a SplitOptions object which contains various optional
    /// criteria for the top-down recursive splitting
    /// \return a constructed bounding volume hierarchy for type T
    template <class F>
    static std::unique_ptr<Bvh> CreateTopDown(F box_fn, size_t primitive_count, const bvh::SplitOptions& options);

private:
    std::unique_ptr<bvh::BvhNode> root_;
};

template <class F>
std::vector<size_t> Bvh::PossibleIntersections(F fn) {
    std::vector<size_t> indices;
    std::vector<std::reference_wrapper<bvh::BvhNode>> nodes{*root_};

    while (!nodes.empty()) {
        auto working = nodes.back();
        nodes.pop_back();

        if (!fn(working.get().Box())) continue;

        if (working.get().IsLeaf()) {
            for (auto i : working.get().indices_) {
                indices.push_back(i);
            }
        } else {
            nodes.emplace_back(*working.get().left_);
            nodes.emplace_back(*working.get().right_);
        }
    }

    return indices;
}

template <class Fc, class Ff>
std::vector<size_t> Bvh::PossibleClosest(Fc closest, Ff furthest) {
    /*
     * Closest distance pruning
     *
     * The underlying principle is that out of any
     * set of bounding boxes, one of them will have the smallest *furthest*
     * distance from a given entity. Any bounding box which does not have a
     * *closest* distance smaller than that threshold cannot possibly contain
     * anything closer to the entity than the box which produced the
     * threshold.
     */
    using std::find_if;
    using std::min;
    using std::remove_if;

    struct search_t {
        double close;
        double far;
        std::reference_wrapper<bvh::BvhNode> node;
    };

    std::vector<search_t> nodes{
            {closest(root_->Box()), furthest(root_->Box()), *root_}};

    double closest_furthest = nodes.front().far;
    while (true) {
        auto non_leaf = find_if(
                nodes.begin(), nodes.end(),
                [](const search_t& x) { return !x.node.get().IsLeaf(); });
        if (non_leaf == nodes.end()) {
            break;
        }

        search_t left{closest(non_leaf->node.get().left_->Box()),
                      furthest(non_leaf->node.get().left_->Box()),
                      *non_leaf->node.get().left_};
        search_t right{closest(non_leaf->node.get().right_->Box()),
                       furthest(non_leaf->node.get().right_->Box()),
                       *non_leaf->node.get().right_};
        nodes.erase(non_leaf);
        nodes.push_back(left);
        nodes.push_back(right);

        closest_furthest = min(closest_furthest, min(left.far, right.far));
        nodes.erase(remove_if(nodes.begin(), nodes.end(),
                              [&](const search_t& t) {
                                  return t.close > closest_furthest;
                              }),
                    nodes.end());
    }

    std::vector<size_t> indices;
    for (const auto& n : nodes) {
        for (auto i : n.node.get().indices_) {
            indices.push_back(i);
        }
    }

    return indices;
}

template <class F>
std::unique_ptr<Bvh> Bvh::CreateTopDown(F box_fn, size_t primitive_count, const bvh::SplitOptions& options) {
    // Create the BVH and reserve memory for the indices
    auto bvh = std::make_unique<Bvh>();
    bvh->root_ = std::make_unique<bvh::BvhNode>();

    bvh->root_->indices_.reserve(primitive_count);
    for (size_t i = 0; i < primitive_count; ++i) {
        bvh->root_->indices_.push_back(i);
    }

    // At this point the root node contains all primitives. We compute the
    // bounding box for the root node from its contained primitives and then
    // begin the recursive splitting algorithm.
    bvh->root_->SetBox(box_fn);
    bvh::BvhNode::SplitLeafObjMean(*bvh->root_, box_fn, options);

    return bvh;
}

}  // namespace geometry
}  // namespace open3d
