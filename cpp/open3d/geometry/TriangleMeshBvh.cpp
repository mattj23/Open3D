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

#include "open3d/geometry/TriangleMeshBvh.h"

namespace open3d {
namespace geometry {

TriangleMeshBvh TriangleMeshBvh::TopDown(const TriangleMesh& mesh,
                                         const bvh::SplitOptions& options) {
    TriangleMeshBvh bvh;
    bvh.triangles_.reserve(mesh.triangles_.size());
    bvh.bounds_.reserve(mesh.triangles_.size());

    for (const auto& t : mesh.triangles_) {
        bvh.triangles_.emplace_back(mesh.vertices_[t[0]], mesh.vertices_[t[1]],
                                    mesh.vertices_[t[2]]);
        bvh.bounds_.emplace_back(bvh.triangles_.back().GetBoundingBox());
    }

    bvh.bvh_ = Bvh::CreateTopDown([&bvh](size_t i) { return bvh.bounds_[i]; },
                                  bvh.bounds_.size(), options);

    return bvh;
}

std::vector<double> TriangleMeshBvh::IntersectionParameters(const Line3D& line,
                                                            bool exact) const {
    std::vector<double> results;
    auto candidates = LineIntersectionCandidates(line, exact);

    for (auto c : candidates) {
        auto result = triangles_[c].IntersectionParameter(line);
        if (result.has_value()) results.push_back(result.value());
    }
    return results;
}

utility::optional<double> TriangleMeshBvh::LowestIntersectionParameter(
        const Line3D& line, bool exact) const {
    auto results = IntersectionParameters(line, exact);
    if (results.empty()) return {};

    return *std::min_element(results.begin(), results.end());
}

utility::optional<Eigen::Vector3d> TriangleMeshBvh::LowestIntersection(
        const Line3D& line, bool exact) const {
    auto result = LowestIntersectionParameter(line, exact);
    if (!result.has_value()) return {};

    return line.Line().pointAt(result.value());
}

std::vector<Eigen::Vector3d> TriangleMeshBvh::Intersections(const Line3D& line,
                                                            bool exact) const {
    std::vector<Eigen::Vector3d> results;
    auto candidates = LineIntersectionCandidates(line, exact);

    for (auto c : candidates) {
        auto result = triangles_[c].Intersection(line);
        if (result.has_value()) results.push_back(result.value());
    }
    return results;
}

bool TriangleMeshBvh::HasIntersectionWith(const Line3D& line,
                                          bool exact) const {
    auto candidates = LineIntersectionCandidates(line, exact);

    for (auto c : candidates) {
        auto result = triangles_[c].Intersection(line);
        if (result.has_value()) return true;
    }

    return false;
}

bool TriangleMeshBvh::IsPointInside(const Eigen::Vector3d& point) const {
    // This test works by casting an infinite ray from the point of interest in
    // an arbitrary direction and counting the number of intersections. If the
    // point lies inside a watertight mesh there will be an odd number of
    // intersections, if the point is on the outside the count will be even.

    // In order to ensure that we can use the ultra-fast slab method for AABB
    // to ray intersections, we will test against a ray that has some non-zero
    // component in all three cartesian directions, avoiding the unreliable
    // case where the ray lies in one of the AABB's planes.
    auto intersections = Intersections(Ray3D(point, {1, 0.5, 0.25}));
    return intersections.size() % 2 == 1;
}

std::vector<size_t> TriangleMeshBvh::LineIntersectionCandidates(
        const Line3D& line, bool exact) const {
    return bvh_->PossibleIntersections(
            [&line, exact](const AxisAlignedBoundingBox& box) {
                return exact ? line.ExactAABB(box).has_value()
                             : line.SlabAABB(box).has_value();
            });
}

Eigen::Vector3d TriangleMeshBvh::ClosestPointTo(
        const Eigen::Vector3d& point) const {
    using std::max;
    using std::min;

    auto closest = [&point](const AxisAlignedBoundingBox& box) {
        // The closest point on the box is found by clamping the test point to
        // the bounds
        auto cx = max(min(point.x(), box.max_bound_.x()), box.min_bound_.x());
        auto cy = max(min(point.y(), box.max_bound_.y()), box.min_bound_.y());
        auto cz = max(min(point.z(), box.max_bound_.z()), box.min_bound_.z());
        Eigen::Vector3d c{cx, cy, cz};
        return (c - point).norm();
    };

    auto farthest = [&point](const AxisAlignedBoundingBox& box) {
        // The farthest point on the AABB is always going to be one of the
        // eight corners
        double dist = 0;
        for (const auto& p : box.GetBoxPoints()) {
            dist = std::max((p - point).norm(), dist);
        }
        return dist;
    };

    auto candidates = bvh_->PossibleClosest(closest, farthest);
    double closest_distance;
    Eigen::Vector3d closest_point{};

    for (size_t i = 0; i < candidates.size(); ++i) {
        auto cp = triangles_[i].ClosestPoint(point);
        double d = (point - cp).norm();

        if (i == 0 || d < closest_distance) {
            closest_distance = d;
            closest_point = cp;
        }
    }

    return closest_point;
}

Eigen::Vector3d TriangleMeshBvh::FarthestPointFrom(
        const Eigen::Vector3d& point) const {
    using std::max;
    using std::min;

    auto closest = [&point](const AxisAlignedBoundingBox& box) {
        // The closest point on the box is found by clamping the test point to
        // the bounds
        auto cx = max(min(point.x(), box.max_bound_.x()), box.min_bound_.x());
        auto cy = max(min(point.y(), box.max_bound_.y()), box.min_bound_.y());
        auto cz = max(min(point.z(), box.max_bound_.z()), box.min_bound_.z());
        Eigen::Vector3d c{cx, cy, cz};
        return (c - point).norm();
    };

    auto farthest = [&point](const AxisAlignedBoundingBox& box) {
        // The farthest point on the AABB is always going to be one of the
        // eight corners
        double dist = 0;
        for (const auto& p : box.GetBoxPoints()) {
            dist = std::max((p - point).norm(), dist);
        }
        return dist;
    };

    auto candidates = bvh_->PossibleFarthest(closest, farthest);
    double farthest_distance;
    Eigen::Vector3d farthest_point{};

    for (size_t i = 0; i < candidates.size(); ++i) {
        auto fp = triangles_[i].FarthestPoint(point);
        double d = (point - fp).norm();

        if (i == 0 || d > farthest_distance) {
            farthest_distance = d;
            farthest_point = fp;
        }
    }

    return farthest_point;
}
}  // namespace geometry
}  // namespace open3d
