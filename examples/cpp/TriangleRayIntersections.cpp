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

#include <iostream>
#include <chrono>
#include <random>
#include <vector>

#include "open3d/Open3D.h"
#include "open3d/geometry/TriangleIntersectionData.h"

#define TEST_COUNT 1000000

int main() {
    using namespace open3d::geometry;
    using namespace std::chrono;
    using Clock = steady_clock;

    std::random_device rd;
    std::mt19937_64 mt(rd());
    std::uniform_real_distribution<double> pos_dist(-10.0, 10.0);

    printf("Generating %i test triangles\n", TEST_COUNT);
    std::vector<TriangleIntersectionData> triangles;
    for (int i = 0; i < TEST_COUNT; ++i) {
        Eigen::Vector3d v0{pos_dist(mt), pos_dist(mt), pos_dist(mt)};
        Eigen::Vector3d v1{pos_dist(mt), pos_dist(mt), pos_dist(mt)};
        Eigen::Vector3d v2{pos_dist(mt), pos_dist(mt), pos_dist(mt)};
        triangles.emplace_back(v0, v1, v2);
    }

    std::vector<AxisAlignedBoundingBox> boxes;
    boxes.reserve(TEST_COUNT);

    auto start = Clock::now();
    for (const auto& t : triangles) {
//        auto box = t.GetBoundingBox();
        boxes.emplace_back(t.GetBoundingBox());
    }
    double elapsed = duration_cast<microseconds>(Clock::now() - start).count() * 0.001;

    printf("Took %f ms\n", elapsed);


}



