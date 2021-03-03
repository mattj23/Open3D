// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2020 www.open3d.org
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
#include <Eigen/Dense>

#include "BoundingVolume.h"
#include "Line3D.h"
#include "open3d/utility/Optional.h"

namespace open3d {
namespace geometry {

/// \class TriangleBounds
///
/// \brief Contains pre-calculated values used to perform checks against a
/// triangle and its edges
class TriangleBounds {
public:
    /// \brief Constructor requires the three vertices of the triangle
    TriangleBounds(const Eigen::Vector3d& v0,
                   const Eigen::Vector3d& v1,
                   const Eigen::Vector3d& v2);

    /// \brief Return the underlying plane on which the triangle exists
    const Eigen::Hyperplane<double, 3>& Plane() const { return plane_; }

    /// \brief Returns true if the cross product between the two edges anchored
    /// at v0 is less than an epsilon of 1e-10, which would indicate either a
    /// 0 or 180 degree angle, or a zero-length edge. In either case, if this
    /// returns true, none of the triangle parameters are calculated and any
    /// check against the triangle will return false.
    bool IsDegenerate() const { return is_degenerate_; }

    /// \brief Returns true if the dot product between the provided direction
    /// vector and the cross product of the two triangle vectors is less than
    /// an epsilon of 1e-10, which would indicate that the direction vector
    /// lies parallel to the triangle plane.
    bool IsParallelTo(const Eigen::Vector3d& direction) const {
        return fabs(plane_.normal().dot(direction)) < 1e-10;
    }

    /// \brief Returns true if the point is enclosed within the triangle,
    /// including if it lies on one of the edges. This function assumes the
    /// point lies on the triangle's plane.
    bool IsPointInTriangle(const Eigen::Vector3d& point) const;

    /// \brief Performs an intersection between a Line3D (or one of its derived
    /// classes, Ray3D or Segment3D) and returns a point if the intersection
    /// exists. This function honors the semantics of the different types of
    /// lines.
    utility::optional<Eigen::Vector3d> Intersection(const Line3D& line) const;

    /// \brief Performs a projection of a point onto the plane of the triangle
    /// and returns the point if it lies within the bounds of the triangle. This
    /// is different from ClosestPoint because this function will return an
    /// empty value if the point is not enclosed within the bounds of the
    /// triangle.
    utility::optional<Eigen::Vector3d> Projection(
            const Eigen::Vector3d& point) const;

    /// \brief Calculates the closest point on the triangle to the test point,
    /// which may be any of the three vertices, may lie on any of the three
    /// edges, or lie somewhere on the face of the triangle.
    Eigen::Vector3d ClosestPoint(const Eigen::Vector3d& point) const;

    /// \brief Calculates the axis aligned bounding box for the underlying
    /// triangle
    AxisAlignedBoundingBox GetBoundingBox() const;

private:
    Eigen::Vector3d u_;
    Eigen::Vector3d v_;
    Eigen::Vector3d v0_;
    Eigen::Hyperplane<double, 3> plane_;

    double uu_;
    double uv_;
    double vv_;
    double d_;

    bool is_degenerate_ = false;
};

}  // namespace geometry
}  // namespace open3d
