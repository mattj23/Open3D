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

#include "open3d/geometry/TriangleMeshBvh.h"

#include "tests/UnitTest.h"

using namespace ::testing;
using namespace open3d::geometry;
using v_t = Eigen::Vector3d;
using ins_t = std::tuple<v_t, bool>;

namespace open3d {
namespace geometry {

// Inside Mesh tests
// ===========================================================================
class InsideTests : public TestWithParam<ins_t> {};

TEST_P(InsideTests, CheckPointInside) {
    const auto& point = std::get<0>(GetParam());
    bool expected = std::get<1>(GetParam());

    auto mesh = TriangleMesh::CreateTorus();
    auto bvh = TriangleMeshBvh::TopDown(*mesh, {});

    bool result = bvh.IsPointInside(point);
    EXPECT_EQ(expected, result);
}

INSTANTIATE_TEST_CASE_P(InsideTests, InsideTests,
                        Values( ins_t{{0, 0, 0}, false},
                                ins_t{{0, -1, 0}, true},
                                ins_t{{0, 0, 2}, false},
                                ins_t{{0, 1, 0}, true}));


}  // namespace geometry
}  // namespace open3d