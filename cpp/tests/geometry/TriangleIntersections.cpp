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

#include <open3d/geometry/TriangleIntersectionData.h>
#include <open3d/geometry/BoundingVolume.h>
#include "tests/UnitTest.h"

using v_t = Eigen::Vector3d;
using box_param_t = std::tuple<v_t, v_t, v_t, v_t, v_t>;
using simple_param_t = std::tuple<v_t, v_t, v_t>;

namespace open3d {
namespace tests {

// Bounding box tests
// ===========================================================================
class TriangleBoundingBoxTests : public ::testing::TestWithParam<box_param_t> {
};

TEST_P(TriangleBoundingBoxTests, CheckBoundingBoxes) {
    const auto& v0 = std::get<0>(GetParam());
    const auto& v1 = std::get<1>(GetParam());
    const auto& v2 = std::get<2>(GetParam());
    const auto& min = std::get<3>(GetParam());
    const auto& max = std::get<4>(GetParam());

    geometry::TriangleIntersectionData t{v0, v1, v2};
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
INSTANTIATE_TEST_CASE_P(TriangleBoundingBox,
                        TriangleBoundingBoxTests,
                        ::testing::Values(box_param_t{{-2, 2, 0},
                                                      {0, 0, 1},
                                                      {1, 1, 1},
                                                      {-2, 0, 0},
                                                      {1, 2, 1}},
                                          box_param_t{{0, -2, 2},
                                                      {0, 0, 1},
                                                      {1, 1, 1},
                                                      {0, -2, 1},
                                                      {1, 1, 2}},
                                          box_param_t{{2, 0, -2},
                                                      {0, 0, 1},
                                                      {1, 1, 1},
                                                      {0, 0, -2},
                                                      {2, 1, 1}},
                                          box_param_t{{0, 0, 0},
                                                      {-2, 2, 1},
                                                      {1, 1, 1},
                                                      {-2, 0, 0},
                                                      {1, 2, 1}},
                                          box_param_t{{0, 0, 0},
                                                      {0, -2, 2},
                                                      {1, 1, 1},
                                                      {0, -2, 0},
                                                      {1, 1, 2}},
                                          box_param_t{{0, 0, 0},
                                                      {2, 0, -2},
                                                      {1, 1, 1},
                                                      {0, 0, -2},
                                                      {2, 1, 1}},
                                          box_param_t{{0, 0, 0},
                                                      {0, 0, 1},
                                                      {-2, 2, 1},
                                                      {-2, 0, 0},
                                                      {0, 2, 1}},
                                          box_param_t{{0, 0, 0},
                                                      {0, 0, 1},
                                                      {1, -2, 2},
                                                      {0, -2, 0},
                                                      {1, 0, 2}},
                                          box_param_t{{0, 0, 0},
                                                      {0, 0, 1},
                                                      {2, 1, -2},
                                                      {0, 0, -2},
                                                      {2, 1, 1}}));

// Intersection tests
// ===========================================================================
class SimpleTriangleTest : public ::testing::TestWithParam<simple_param_t> {
protected:
    geometry::TriangleIntersectionData triangle{
            {1, 0, 0}, {0, 0, 0}, {0, 1, 0}};
};

TEST_P(SimpleTriangleTest, Intersections) {
    const auto& line_origin = std::get<0>(GetParam());
    const auto& line_direction = std::get<1>(GetParam());
    const auto& expected = std::get<2>(GetParam());

    Eigen::ParametrizedLine<double, 3> line{line_origin, line_direction};

    auto result = triangle.Intersect(line, false, false);
    v_t point = result.second;

    EXPECT_TRUE(result.first);
    ExpectEQ(expected, point);
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
        ::testing::Values(
                // Intersections with face from different directions
                simple_param_t{{-1.3, -1.0, -2.0},
                               {-0.8, -0.7, -1.0},
                               {0.3, 0.4, 0.0}},
                simple_param_t{
                        {-0.8, 0.6, -2.0}, {-0.6, 0.3, -1.0}, {0.4, 0.0, 0.0}},
                simple_param_t{
                        {1.8, 2.1, -2.0}, {0.9, 1.0, -1.0}, {0.0, 0.1, 0.0}},
                simple_param_t{
                        {1.5, -0.4, -2.0}, {-0.5, 0.3, 1.0}, {0.5, 0.2, 0.0}},
                simple_param_t{
                        {0.9, 2.2, 2.0}, {0.3, 1.0, 1.0}, {0.3, 0.2, 0.0}},
                simple_param_t{{-0.6, -1.5, -2.0},
                               {-0.4, -0.9, -1.0},
                               {0.2, 0.3, 0.0}},
                simple_param_t{
                        {-0.5, -1.2, -2.0}, {0.5, 0.8, 1.0}, {0.5, 0.4, 0.0}},
                simple_param_t{
                        {1.3, -0.8, -2.0}, {-0.6, 0.5, 1.0}, {0.1, 0.2, 0.0}},
                simple_param_t{
                        {0.0, -0.6, -2.0}, {0.2, 0.4, 1.0}, {0.4, 0.2, 0.0}},
                simple_param_t{
                        {1.0, -0.6, 2.0}, {-0.5, 0.3, -1.0}, {0.0, 0.0, 0.0}},
                // Intersections with edges from different directions
                simple_param_t{
                        {-1.8, 1.4, -2.0}, {1.0, -0.7, 1.0}, {0.2, 0, 0}},
                simple_param_t{
                        {0.2, 1.0, -2.0}, {-0.0, -0.5, 1.0}, {0.2, 0, 0}},
                simple_param_t{
                        {-1.2, 0.6, -2.0}, {-1.0, 0.3, -1.0}, {0.8, 0, 0}},
                simple_param_t{{0.4, -1.6, -2.0}, {0.2, 0.8, 1.0}, {0.8, 0, 0}},
                simple_param_t{
                        {-0.2, -0.2, 2.0}, {0.1, 0.2, -1.0}, {0, 0.2, 0}},
                simple_param_t{
                        {-1.8, -0.6, 2.0}, {-0.9, -0.4, 1.0}, {0, 0.2, 0}},
                simple_param_t{
                        {2.0, 1.6, 2.0}, {-1.0, -0.4, -1.0}, {0, 0.8, 0}},
                simple_param_t{
                        {-1.2, 0.2, 2.0}, {-0.6, -0.3, 1.0}, {0, 0.8, 0}},
                // Intersections with corners from different directions
                simple_param_t{{0.8, 0.6, 2.0}, {0.1, -0.3, -1.0}, {1, 0, 0}},
                simple_param_t{
                        {1.0, -0.2, -2.0}, {-0.0, -0.1, -1.0}, {1, 0, 0}},
                simple_param_t{{-0.6, -1.0, -2.0}, {0.8, 0.5, 1.0}, {1, 0, 0}},
                simple_param_t{{0.8, 2.0, 2.0}, {-0.4, -1.0, -1.0}, {0, 0, 0}},
                simple_param_t{{-1.4, 0.6, -2.0}, {-0.7, 0.3, -1.0}, {0, 0, 0}},
                simple_param_t{{-1.2, -2.0, 2.0}, {0.6, 1.0, -1.0}, {0, 0, 0}},
                simple_param_t{{-1.0, 2.0, 2.0}, {0.5, -0.5, -1.0}, {0, 1, 0}},
                simple_param_t{{0.0, 1.2, 2.0}, {-0.0, 0.1, 1.0}, {0, 1, 0}},
                simple_param_t{{0.2, 0.6, 2.0}, {0.1, -0.2, 1.0}, {0, 1, 0}}));

}
}