// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2019 www.open3d.org
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

namespace open3d {
namespace geometry {

class IntersectionTest {
public:
    static bool AABBAABB(const Eigen::Vector3d& min0,
                         const Eigen::Vector3d& max0,
                         const Eigen::Vector3d& min1,
                         const Eigen::Vector3d& max1);

    static bool TriangleTriangle3d(const Eigen::Vector3d& p0,
                                   const Eigen::Vector3d& p1,
                                   const Eigen::Vector3d& p2,
                                   const Eigen::Vector3d& q0,
                                   const Eigen::Vector3d& q1,
                                   const Eigen::Vector3d& q2);

    /// \brief Tests if a line/ray/segment intersects with a triangle that
    /// consists of three vertices. Use for quick, one-off tests, for multiple
    /// tests against a triangle or set of triangles, or to recover the point of
    /// intersection, use the TriangleIntersectionData class.
    ///
    /// \param line The parameterized line to test against the triangle. If the
    /// test is a ray test or a segment test, the origin is important. If the
    /// test is for a segment, the line origin is the segment start point and
    /// the direction is the vector to the endpoint. Do not normalize the
    /// segment direction, as it will then become a segment of length 1.
    /// \param v0 The root vertex of the triangle
    /// \param v1 The second vertex of the triangle
    /// \param v2 The third vertex of the triangle
    /// \param is_ray Set true to disregard intersections behind the origin
    /// \param is_segment Set true to disregard intersections that are either
    /// behind the origin or beyond line.origin() + line.direction(). This flag
    /// will supersede is_ray.
    /// \return true if the line intersects.
    static bool LineTriangle3d(const Eigen::ParametrizedLine<double, 3>& line,
                               const Eigen::Vector3d& v0,
                               const Eigen::Vector3d& v1,
                               const Eigen::Vector3d& v2,
                               bool is_ray=false,
                               bool is_segment=false);

    static bool TriangleAABB(const Eigen::Vector3d& box_center,
                             const Eigen::Vector3d& box_half_size,
                             const Eigen::Vector3d& vert0,
                             const Eigen::Vector3d& vert1,
                             const Eigen::Vector3d& vert2);

    /// Tests if the given four points all lie on the same plane.
    static bool PointsCoplanar(const Eigen::Vector3d& p0,
                               const Eigen::Vector3d& p1,
                               const Eigen::Vector3d& p2,
                               const Eigen::Vector3d& p3);

    /// Computes the minimum distance between two lines. The first line is
    /// defined by 3D points \p p0 and \p p1, the second line is defined
    /// by 3D points \p q0 and \p q1. The returned distance is negative
    /// if no minimum distance can be computed. This implementation is based on
    /// the description of Paul Bourke
    /// (http://paulbourke.net/geometry/pointlineplane/).
    static double LinesMinimumDistance(const Eigen::Vector3d& p0,
                                       const Eigen::Vector3d& p1,
                                       const Eigen::Vector3d& q0,
                                       const Eigen::Vector3d& q1);

    /// Computes the minimum distance between two line segments. The first line
    /// segment is defined by 3D points \p p0 and \p p1, the second line
    /// segment is defined by 3D points \p q0 and \p q1. This
    /// implementation is based on the description of David Eberly
    /// (https://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf).
    static double LineSegmentsMinimumDistance(const Eigen::Vector3d& p0,
                                              const Eigen::Vector3d& p1,
                                              const Eigen::Vector3d& q0,
                                              const Eigen::Vector3d& q1);
};

}  // namespace geometry
}  // namespace open3d
