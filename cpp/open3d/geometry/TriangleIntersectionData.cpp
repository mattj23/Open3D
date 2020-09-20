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

#include "TriangleIntersectionData.h"

namespace open3d{
namespace geometry {

TriangleIntersectionData::TriangleIntersectionData(
        const Eigen::Vector3d& v0,
        const Eigen::Vector3d& v1,
        const Eigen::Vector3d& v2) {
    v0_ = v0;
    u_ = v1 - v0;
    v_ = v2 - v0;

    n_ = u_.cross(v_);

    if (n_.norm() <= 1e-10) {
        is_degenerate_ = true;
        return;
    }

    plane_ = Eigen::Hyperplane<double, 3>::Through(v0, v1, v2);

    uu_ = u_.dot(u_);
    uv_ = u_.dot(v_);
    vv_ = v_.dot(v_);
    d_ = uv_ * uv_ - uu_ * vv_;
}

std::pair<bool, Eigen::Vector3d>TriangleIntersectionData::Intersect(
        const Eigen::ParametrizedLine<double, 3>& line,
        const bool is_ray,
        const bool is_segment) {

    if (is_degenerate_)
        return {false, {}};

    // Check if line is parallel to the triangle
    if (fabs(n_.dot(line.direction())) < 1e-10)
        return {false, {}};

    auto i_scalar = line.intersectionParameter(plane_);

    // If the intersection is behind the ray origin we can ignore it
    if ((is_ray || is_segment) && i_scalar < 0)
        return {false, {}};

    // if we are considering the line to be a segment between two points, we
    // also want to check if the intersection lies beyond the second point, in
    // which case we can also ignore it
    if (is_segment && i_scalar > 1)
        return {false, {}};

    // Is the intersection point inside the triangle?
    Eigen::Vector3d i_point = line.pointAt(i_scalar);
    Eigen::Vector3d w = i_point - v0_;
    double wu = w.dot(u_);
    double wv = w.dot(v_);

    double s = (uv_ * wv - vv_ * wu) / d_;
    if (s < 0.0 || s > 1.0)
        return {false, {}};
    double t = (uv_ * wu - uu_ * wv) / d_;
    if (t < 0.0 || (s + t) > 1.0)
        return {false, {}};

    return {true, i_point};
}

AxisAlignedBoundingBox TriangleIntersectionData::GetBoundingBox() const {
    // Reconstruct the original v1 and v2 vertices
    auto v1 = v0_ + u_;
    auto v2 = v0_ + v_;

    // Because there are only three points here, the overhead of creating a
    // std::vector in order to use AxisAlignedBoundingBox::CreateFromPoints
    // ends up taking about 4x longer than nesting std::min and std::max

    Eigen::Vector3d max_bound{
            std::max(std::max(v0_.x(), v1.x()), v2.x()),
            std::max(std::max(v0_.y(), v1.y()), v2.y()),
            std::max(std::max(v0_.z(), v1.z()), v2.z())
    };

    Eigen::Vector3d min_bound{
            std::min(std::min(v0_.x(), v1.x()), v2.x()),
            std::min(std::min(v0_.y(), v1.y()), v2.y()),
            std::min(std::min(v0_.z(), v1.z()), v2.z())
    };

    return {min_bound, max_bound};
}

}
}
