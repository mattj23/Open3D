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

#include "TriangleBounds.h"

namespace open3d {
namespace geometry {

TriangleBounds::TriangleBounds(const Eigen::Vector3d& v0,
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

bool TriangleBounds::IsPointInTriangle(const Eigen::Vector3d& point) const {
    /* Based on the latter part of the algorithm in
     * http://geomalgorithms.com/a06-_intersect-2.html#intersect3D_RayTriangle()
     * this method takes a point and checks whether or not it lies within the
     * bounds of the triangle.
     */
    if (is_degenerate_) return false;

    Eigen::Vector3d w = point - v0_;
    double wu = w.dot(u_);
    double wv = w.dot(v_);

    double s = (uv_ * wv - vv_ * wu) / d_;
    if (s < 0.0 || s > 1.0) return false;

    double t = (uv_ * wu - uu_ * wv) / d_;
    return !(t < 0.0 || (s + t) > 1.0);
}

utility::optional<Eigen::Vector3d> TriangleBounds::Intersection(
        const Line3D& line) const {
    if (is_degenerate_ || IsParallelTo(line.Direction())) {
        return {};
    }

    auto result = line.IntersectionParameter(plane_);
    if (!result.has_value()) return {};

    auto point = line.LinePointAt(result.value());
    if (IsPointInTriangle(point)) {
        return point;
    }
    return {};
}

utility::optional<Eigen::Vector3d> TriangleBounds::Projection(
        const Eigen::Vector3d& point) const {
    if (is_degenerate_) {
        return {};
    }

    auto p = plane_.projection(point);
    if (IsPointInTriangle(p)) {
        return p;
    }
    return {};
}

AxisAlignedBoundingBox TriangleBounds::GetBoundingBox() const {
    // Reconstruct the original v1 and v2 vertices
    auto v1 = v0_ + u_;
    auto v2 = v0_ + v_;

    // Because there are only three points here, the overhead of creating a
    // std::vector in order to use AxisAlignedBoundingBox::CreateFromPoints
    // ends up taking about 4x longer than nesting std::min and std::max

    Eigen::Vector3d max_bound{std::max(std::max(v0_.x(), v1.x()), v2.x()),
                              std::max(std::max(v0_.y(), v1.y()), v2.y()),
                              std::max(std::max(v0_.z(), v1.z()), v2.z())};

    Eigen::Vector3d min_bound{std::min(std::min(v0_.x(), v1.x()), v2.x()),
                              std::min(std::min(v0_.y(), v1.y()), v2.y()),
                              std::min(std::min(v0_.z(), v1.z()), v2.z())};

    return {min_bound, max_bound};
}

Eigen::Vector3d TriangleBounds::ClosestPoint(
        const Eigen::Vector3d& point) const {
    /* This function is built from Christer Ericson's "Real-Time Collision
     * Detection" implementation on page 141 and 142
     */

    using Vector = Eigen::Vector3d;

    // Check if P in vertex region outside A
    Vector ap = point - v0_;
    double d1 = u_.dot(ap);
    double d2 = v_.dot(ap);
    if (d1 <= 0. && d2 <= 0.) return v0_;

    // Check if P in vertex region outside B
    //    Vector bp = point - v1;
    Vector bp = ap - u_;
    double d3 = u_.dot(bp);
    double d4 = v_.dot(bp);
    if (d3 >= 0. && d4 <= d3) return v0_ + u_;

    // Check if P in edge region of AB, if so return projection of P onto AB
    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0. && d1 >= 0. && d3 <= 0.) {
        double v = d1 / (d1 - d3);
        return v0_ + v * u_;
    }

    // Check if P in vertex region outside C
    //    Vector cp = point - v2;
    Vector cp = ap - v_;
    double d5 = u_.dot(cp);
    double d6 = v_.dot(cp);
    if (d6 >= 0. && d5 <= d6) return v0_ + v_;

    // Check if P in edge region of AC, if so return projection of P onto AC
    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0. && d2 >= 0. && d6 <= 0.) {
        double w = d2 / (d2 - d6);
        return v0_ + w * v_;
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    double va = d3 * d6 - d5 * d4;
    if (va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0.) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return v0_ + u_ + w * (v_ - u_);
        //        return v1 + w * (v2 - v1); // barycentric coordinates
        //        (0,1-w,w)
    }

    // P inside face region. Compute Q through its barycentric coordinates
    // (u,v,w)
    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    return v0_ + u_ * v + v_ * w;
}

}  // namespace geometry
}  // namespace open3d
