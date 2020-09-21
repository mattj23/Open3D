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

namespace open3d {
namespace geometry {

/// \class TriangleIntersectionData
///
/// \brief Contains pre-calculated values used to perform intersection tests
/// between lines/rays and triangles
class TriangleIntersectionData {
public:
    /// \brief Constructor requires the three verticies of the triangle
    TriangleIntersectionData(const Eigen::Vector3d& v0,
                             const Eigen::Vector3d& v1,
                             const Eigen::Vector3d& v2);

    /// \brief Computes the intersection of the triangle and the given line. The
    /// line can be treated as a ray or a line segment. Returns a pair in which
    /// the first value specifies whether or not an intersection occurred.
    ///
    /// \param line the line along which to attempt an intersection. In the case
    /// that the line is to be treated as a ray the line.origin() vector is
    /// important. If it is to be treated as a line segment, the direction()
    /// vector must be the vector between the origin and the endpoint, such that
    /// any point lying at a parameter > 1.0 is beyond the end of the segment.
    /// \param is_ray set true if only intersections with positive parameters
    /// should be considered
    /// \param is_segment set true if only intersections with parameters between
    /// 0 and 1 should be considered
    /// \return the first element of the return pair will be true only if the
    /// intersection succeeded, and in that case the second value of the pair
    /// will contain the intersection point
    std::pair<bool, Eigen::Vector3d> Intersect(
            const Eigen::ParametrizedLine<double, 3>& line,
            bool is_ray,
            bool is_segment);

    /// \brief Calculates the axis aligned bounding box for the underlying
    /// triangle
    AxisAlignedBoundingBox GetBoundingBox() const;

public:
    Eigen::Vector3d u_;
    Eigen::Vector3d v_;
    Eigen::Vector3d v0_;
    Eigen::Vector3d n_;
    Eigen::Hyperplane<double, 3> plane_;

    double uu_;
    double uv_;
    double vv_;
    double d_;

    bool is_degenerate_ = false;
};

}  // namespace geometry
}  // namespace open3d
