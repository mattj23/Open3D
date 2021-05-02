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

#include <open3d/geometry/BoundingVolume.h>
#include <open3d/geometry/TriangleBounds.h>

#include "open3d/utility/Optional.h"
#include "tests/UnitTest.h"

using namespace ::testing;
using namespace open3d::geometry;
using v_t = Eigen::Vector3d;
using lt_t = Line3D::LineType;

// Bounding box test parameter container
using box_t = std::tuple<v_t, v_t, v_t, v_t, v_t>;

// Line intersection test parameter container
using intr_t = std::tuple<v_t, v_t, v_t>;

// Line/Ray/Segment test parameter container
using lrs_t = std::tuple<lt_t, v_t, v_t, bool>;

// Closest point tests
using cp_t = std::tuple<v_t, v_t>;

// Factory function to build appropriate type from enum
inline std::shared_ptr<Line3D> LineFactory(lt_t type,
                                           const v_t& v0,
                                           const v_t& v1) {
    if (type == lt_t::Line) {
        return std::make_shared<Line3D>(v0, v1);
    } else if (type == lt_t::Ray) {
        return std::make_shared<Ray3D>(v0, v1);
    } else if (type == lt_t::Segment) {
        return std::make_shared<Segment3D>(v0, v1);
    } else {
        throw std::exception();
    }
}

namespace open3d {
namespace tests {

// Bounding box tests
// ===========================================================================
class TriangleBoundingBoxTests : public TestWithParam<box_t> {};

TEST_P(TriangleBoundingBoxTests, CheckBoundingBoxes) {
    const auto& v0 = std::get<0>(GetParam());
    const auto& v1 = std::get<1>(GetParam());
    const auto& v2 = std::get<2>(GetParam());
    const auto& min = std::get<3>(GetParam());
    const auto& max = std::get<4>(GetParam());

    geometry::TriangleBounds t{v0, v1, v2};
    auto box = t.GetBoundingBox();

    ExpectEQ(min, box.min_bound_);
    ExpectEQ(max, box.max_bound_);
}

/* Because the triangle intersection data class has its own optimized
 * implementation of the bounding box creation, it needs to be thoroughly
 * tested. The following nine tests each start with a simple triangle from
 * {0,0,1} to {0,0,0} to {1,1,1}, which should result in a bounding box from
 * {0,0,0} to {1,1,1}.
 *
 * Each of the nine tests extends one dimension from one vertex to -2 and one
 * other dimension to +2, then checks to make sure that the final bounding box
 * ends up with the correct minimum and maximum bounds. Each test is thus
 * checking one single argument in the nested std::min's and one argument in
 * the nested std::max's to make sure that each gets promoted correctly to the
 * final bounds.
 */
INSTANTIATE_TEST_CASE_P(
        TriangleBoundingBox,
        TriangleBoundingBoxTests,
        Values(box_t{{-2, 2, 0}, {0, 0, 1}, {1, 1, 1}, {-2, 0, 0}, {1, 2, 1}},
               box_t{{0, -2, 2}, {0, 0, 1}, {1, 1, 1}, {0, -2, 1}, {1, 1, 2}},
               box_t{{2, 0, -2}, {0, 0, 1}, {1, 1, 1}, {0, 0, -2}, {2, 1, 1}},
               box_t{{0, 0, 0}, {-2, 2, 1}, {1, 1, 1}, {-2, 0, 0}, {1, 2, 1}},
               box_t{{0, 0, 0}, {0, -2, 2}, {1, 1, 1}, {0, -2, 0}, {1, 1, 2}},
               box_t{{0, 0, 0}, {2, 0, -2}, {1, 1, 1}, {0, 0, -2}, {2, 1, 1}},
               box_t{{0, 0, 0}, {0, 0, 1}, {-2, 2, 1}, {-2, 0, 0}, {0, 2, 1}},
               box_t{{0, 0, 0}, {0, 0, 1}, {1, -2, 2}, {0, -2, 0}, {1, 0, 2}},
               box_t{{0, 0, 0}, {0, 0, 1}, {2, 1, -2}, {0, 0, -2}, {2, 1, 1}}));

// ClosestPoint tests
// ===========================================================================
class TriangleClosestPointTests : public TestWithParam<cp_t> {
protected:
    geometry::TriangleBounds triangle{{1, 0, 0}, {0, 0, 0}, {0, 1, 0}};
};

TEST_P(TriangleClosestPointTests, CorrectPoints) {
    const auto& test = std::get<0>(GetParam());
    const auto& expected = std::get<1>(GetParam());

    auto result = triangle.ClosestPoint(test);

    EXPECT_LT((expected - result).norm(), 0.0001);
}

INSTANTIATE_TEST_CASE_P(ClosestPoint,
                        TriangleClosestPointTests,
                        Values(
                                // Points on the face
                                cp_t{{.1, .1, 1}, {.1, .1, 0}},
                                cp_t{{.5, .3, -1}, {.5, .3, 0}},
                                cp_t{{.1, .7, 1}, {.1, .7, 0}},

                                // Points on edge v0 -> v1
                                cp_t{{0.7, -.3, -.5}, {0.7, 0, 0}},
                                cp_t{{0.3, -.1, .5}, {0.3, 0, 0}},

                                // Points on edge v1 -> v2
                                cp_t{{-.7, .3, -.5}, {0, .3, 0}},
                                cp_t{{-.3, .8, .5}, {0, 0.8, 0}},

                                // Points on edge v0 -> v2
                                cp_t{{.8, .4, -.1}, {.7, .3, 0}},
                                cp_t{{1, 1, .5}, {.5, .5, 0}},

                                // Points in vertex region around v0
                                cp_t{{2., -.1, 1}, {1, 0, 0}},
                                cp_t{{1.5, .2, -.5}, {1, 0, 0}},

                                // Points in vertex region around v1
                                cp_t{{-.1, -1, 1}, {0, 0, 0}},
                                cp_t{{-.2, -.1, -1}, {0, 0, 0}},

                                // Points in vertex region around v2
                                cp_t{{.1, 1.5, 1}, {0, 1, 0}},
                                cp_t{{-.2, 1.1, -1}, {0, 1, 0}}));

// Farthest point tests
// ===========================================================================
class TriangleFarthestPointTests : public TestWithParam<cp_t> {
protected:
    geometry::TriangleBounds triangle{{1, 0, 0}, {0, 0, 0}, {0, 1, 0}};
};

TEST_P(TriangleFarthestPointTests, CorrectPoints) {
    const auto& test = std::get<0>(GetParam());
    const auto& expected = std::get<1>(GetParam());

    auto result = triangle.FarthestPoint(test);

    EXPECT_LT((expected - result).norm(), 0.0001);
}

INSTANTIATE_TEST_CASE_P(FarthestPoint,
                        TriangleFarthestPointTests,
                        Values( cp_t{{-1, 0, 0}, {1, 0, 0}},
                                cp_t{{0, -1, 0}, {0, 1, 0}},
                                cp_t{{1, 1, 0}, {0, 0, 0}}));

// Intersection tests
// ===========================================================================
class SimpleTriangleTest : public TestWithParam<intr_t> {
protected:
    geometry::TriangleBounds triangle{{1, 0, 0}, {0, 0, 0}, {0, 1, 0}};
};

TEST_P(SimpleTriangleTest, Intersections) {
    const auto& line_origin = std::get<0>(GetParam());
    const auto& line_direction = std::get<1>(GetParam());
    const auto& expected = std::get<2>(GetParam());

    Line3D line{line_origin, line_direction};
    auto result = triangle.Intersection(line);

    EXPECT_TRUE(result.has_value());
    ExpectEQ(expected, result.value());
}

/* The following tests check intersections with a simple triangle that goes from
 * {1,0,0} -> {0,0,0} and {1,0,0} -> {0,1,0}. We test intersection points on the
 * corners, two of the edges, and on the triangle face from all different
 * directions to make sure that they all return the expected intersection point.
 * The line will be evaluated as a line and not a ray or segment, so the
 * line may be pointing towards or away from the triangle.
 */
INSTANTIATE_TEST_CASE_P(
        SimpleTriangleIntersections,
        SimpleTriangleTest,
        Values(
                // Intersections with face from different directions
                intr_t{{-1.3, -1.0, -2.0}, {-0.8, -0.7, -1.0}, {0.3, 0.4, 0.0}},
                intr_t{{-0.8, 0.6, -2.0}, {-0.6, 0.3, -1.0}, {0.4, 0.0, 0.0}},
                intr_t{{1.8, 2.1, -2.0}, {0.9, 1.0, -1.0}, {0.0, 0.1, 0.0}},
                intr_t{{1.5, -0.4, -2.0}, {-0.5, 0.3, 1.0}, {0.5, 0.2, 0.0}},
                intr_t{{0.9, 2.2, 2.0}, {0.3, 1.0, 1.0}, {0.3, 0.2, 0.0}},
                intr_t{{-0.6, -1.5, -2.0}, {-0.4, -0.9, -1.0}, {0.2, 0.3, 0.0}},
                intr_t{{-0.5, -1.2, -2.0}, {0.5, 0.8, 1.0}, {0.5, 0.4, 0.0}},
                intr_t{{1.3, -0.8, -2.0}, {-0.6, 0.5, 1.0}, {0.1, 0.2, 0.0}},
                intr_t{{0.0, -0.6, -2.0}, {0.2, 0.4, 1.0}, {0.4, 0.2, 0.0}},
                intr_t{{1.0, -0.6, 2.0}, {-0.5, 0.3, -1.0}, {0.0, 0.0, 0.0}},

                // Intersections with edges from different directions
                intr_t{{-1.8, 1.4, -2.0}, {1.0, -0.7, 1.0}, {0.2, 0, 0}},
                intr_t{{0.2, 1.0, -2.0}, {-0.0, -0.5, 1.0}, {0.2, 0, 0}},
                intr_t{{-1.2, 0.6, -2.0}, {-1.0, 0.3, -1.0}, {0.8, 0, 0}},
                intr_t{{0.4, -1.6, -2.0}, {0.2, 0.8, 1.0}, {0.8, 0, 0}},
                intr_t{{-0.2, -0.2, 2.0}, {0.1, 0.2, -1.0}, {0, 0.2, 0}},
                intr_t{{-1.8, -0.6, 2.0}, {-0.9, -0.4, 1.0}, {0, 0.2, 0}},
                intr_t{{2.0, 1.6, 2.0}, {-1.0, -0.4, -1.0}, {0, 0.8, 0}},
                intr_t{{-1.2, 0.2, 2.0}, {-0.6, -0.3, 1.0}, {0, 0.8, 0}},

                // Intersections with corners from different directions
                intr_t{{0.8, 0.6, 2.0}, {0.1, -0.3, -1.0}, {1, 0, 0}},
                intr_t{{1.0, -0.2, -2.0}, {-0.0, -0.1, -1.0}, {1, 0, 0}},
                intr_t{{-0.6, -1.0, -2.0}, {0.8, 0.5, 1.0}, {1, 0, 0}},
                intr_t{{0.8, 2.0, 2.0}, {-0.4, -1.0, -1.0}, {0, 0, 0}},
                intr_t{{-1.4, 0.6, -2.0}, {-0.7, 0.3, -1.0}, {0, 0, 0}},
                intr_t{{-1.2, -2.0, 2.0}, {0.6, 1.0, -1.0}, {0, 0, 0}},
                intr_t{{-1.0, 2.0, 2.0}, {0.5, -0.5, -1.0}, {0, 1, 0}},
                intr_t{{0.0, 1.2, 2.0}, {-0.0, 0.1, 1.0}, {0, 1, 0}},
                intr_t{{0.2, 0.6, 2.0}, {0.1, -0.2, 1.0}, {0, 1, 0}}));

// Line/Ray/Segment tests
// ===========================================================================
class LineRaySegmentTests : public TestWithParam<lrs_t> {
protected:
    geometry::TriangleBounds triangle{{1, 0, 0}, {0, 0, 0}, {0, 1, 0}};
};

TEST_P(LineRaySegmentTests, HitsAndMisses) {
    auto line_type = std::get<0>(GetParam());
    const auto& line_origin = std::get<1>(GetParam());
    const auto& line_direction = std::get<2>(GetParam());
    bool hit = std::get<3>(GetParam());

    auto line = LineFactory(line_type, line_origin, line_direction);
    auto result = triangle.Intersection(*line);

    EXPECT_EQ(hit, result.has_value());
}

/* These tests validate the correctness of the the line/segment/ray handling by
 * simply checking whether or not intersections occur. They count on the above
 * line tests to have verified that the intersection points are computed
 * correctly.
 *
 * Each test case is given a line origin and direction, then an integer which
 * specifies what type of intersection this is (0=line, 1=ray, 2=segment) and
 * what the expected result is.
 */
INSTANTIATE_TEST_CASE_P(
        LineRaySegTests,
        LineRaySegmentTests,
        Values(
                // Lines that should hit
                lrs_t{lt_t::Line, {1.7, 1.8, 2.0}, {0.5, 0.8, 1.0}, true},
                lrs_t{lt_t::Line, {-0.4, -0.8, -2.0}, {-0.4, -0.5, -1.0}, true},
                lrs_t{lt_t::Line, {0.2, 2.4, -2.0}, {0.1, 1.0, -1.0}, true},
                lrs_t{lt_t::Line, {2.0, -1.4, -2.0}, {0.9, -0.9, -1.0}, true},
                lrs_t{lt_t::Line, {-0.4, 1.1, 2.0}, {0.4, -0.3, -1.0}, true},

                // Lines that should miss
                lrs_t{lt_t::Line, {2.4, -1.6, 2.0}, {-0.8, 1.0, -1.0}, false},
                lrs_t{lt_t::Line, {-0.2, 0.8, -2.0}, {0.6, -0.1, 1.0}, false},
                lrs_t{lt_t::Line, {2.2, -0.7, -2.0}, {-0.8, 0.8, 1.0}, false},
                lrs_t{lt_t::Line, {1.0, 2.8, -2.0}, {-0.2, -1.0, 1.0}, false},
                lrs_t{lt_t::Line, {1.9, 1.6, 2.0}, {-0.6, -0.3, -1.0}, false},

                // Rays that should hit
                lrs_t{lt_t::Ray, {-0.4, 2.1, 2.0}, {0.3, -0.7, -1.0}, true},
                lrs_t{lt_t::Ray, {-0.2, 0.7, -2.0}, {0.1, -0.2, 1.0}, true},
                lrs_t{lt_t::Ray, {0.1, 2.2, 2.0}, {0.2, -0.9, -1.0}, true},
                lrs_t{lt_t::Ray, {2.2, -0.7, -2.0}, {-1.0, 0.5, 1.0}, true},
                lrs_t{lt_t::Ray, {0.0, 2.2, 2.0}, {0.0, -1.0, -1.0}, true},

                // Rays that would hit but are facing the wrong way
                lrs_t{lt_t::Ray, {-1.3, 2.4, 2.0}, {-0.7, 1.0, 1.0}, false},
                lrs_t{lt_t::Ray, {-0.1, 2.3, -2.0}, {-0.1, 0.8, -1.0}, false},
                lrs_t{lt_t::Ray, {2.2, -0.2, -2.0}, {0.9, -0.2, -1.0}, false},
                lrs_t{lt_t::Ray, {-0.9, 1.0, 2.0}, {-0.7, 0.3, 1.0}, false},
                lrs_t{lt_t::Ray, {-1.2, -1.6, 2.0}, {-1.0, -0.8, 1.0}, false},

                // Rays that should miss completely
                lrs_t{lt_t::Ray, {0.0, -1.1, 2.0}, {0.1, 1.0, -1.0}, false},
                lrs_t{lt_t::Ray, {-0.4, 2.8, 2.0}, {0.4, -1.0, -1.0}, false},
                lrs_t{lt_t::Ray, {1.9, -0.1, -2.0}, {-0.5, 0.5, 1.0}, false},
                lrs_t{lt_t::Ray, {-0.3, -0.9, -2.0}, {0.5, 0.8, 1.0}, false},
                lrs_t{lt_t::Ray, {-1.1, 2.0, -2.0}, {1.0, -0.5, 1.0}, false},

                // Segments starting on the triangle
                lrs_t{lt_t::Segment, {0.0, 0.8, 0.0}, {-0.6, 0.7, -1.0}, true},
                lrs_t{lt_t::Segment, {0.0, 0.2, 0.0}, {0.2, 0.8, 1.0}, true},
                lrs_t{lt_t::Segment, {0.2, 0.5, 0.0}, {1.0, 1.0, 1.0}, true},
                lrs_t{lt_t::Segment, {0.4, 0.3, 0.0}, {-1.0, 0.9, 1.0}, true},
                lrs_t{lt_t::Segment, {0.0, 0.1, 0.0}, {0.4, 0.5, 1.0}, true},

                // Segments ending on the triangle
                lrs_t{lt_t::Segment, {0.8, 0.9, 1.0}, {-0.6, -0.8, -1.0}, true},
                lrs_t{lt_t::Segment, {1.0, 0.1, 1.0}, {-0.6, 0.4, -1.0}, true},
                lrs_t{lt_t::Segment, {0.3, -0.7, -1.0}, {-0.1, 0.8, 1.0}, true},
                lrs_t{lt_t::Segment, {0.3, 0.1, -1.0}, {0.3, 0.2, 1.0}, true},
                lrs_t{lt_t::Segment, {0.6, 0.9, -1.0}, {-0.1, -0.8, 1.0}, true},

                // Segments passing through the triangle
                lrs_t{lt_t::Segment, {-0.4, 0.6, -1.0}, {1.4, 0.0, 2.0}, true},
                lrs_t{lt_t::Segment, {0.6, 0.0, 1.0}, {-1.2, 0.4, -2.0}, true},
                lrs_t{lt_t::Segment, {0.9, 0.3, 1.0}, {-1.6, -0.2, -2.0}, true},
                lrs_t{lt_t::Segment, {0.1, -0.3, -1.0}, {-0.2, 2.0, 2.0}, true},
                lrs_t{lt_t::Segment, {0.9, 0.4, -1.0}, {-0.6, -0.6, 2.0}, true},

                // Segments too short to reach the triangle
                lrs_t{lt_t::Segment, {0.3, 0.3, -2.0}, {0.0, 0.1, -1.0}, false},
                lrs_t{lt_t::Segment, {0.8, 1.2, 2.0}, {-0.3, -0.5, .5}, false},
                lrs_t{lt_t::Segment, {-1.9, -1.1, 2.0}, {1.0, 0.8, .1}, false},
                lrs_t{lt_t::Segment, {2.5, -0.9, 2.0}, {-1.0, 0.6, 1.0}, false},
                lrs_t{lt_t::Segment, {2.2, 1.9, -2.}, {-1., -.9, -.1}, false}));

}  // namespace tests
}  // namespace open3d