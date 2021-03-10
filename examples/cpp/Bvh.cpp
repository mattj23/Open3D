// --------------------------------------------------------EXAMPLE_CPP(LineSet
// ${CMAKE_PROJECT_NAME})--------------------
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

#include "open3d/geometry/Bvh.h"

#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include "open3d/Open3D.h"

open3d::geometry::AxisAlignedBoundingBox Fn(
        open3d::geometry::TriangleBounds b) {
    return b.GetBoundingBox();
}

struct AABB {
    Eigen::Vector3d min_bound_;
    Eigen::Vector3d max_bound_;
};

int main(int argc, char** argv) {
    using namespace open3d::geometry;
    using namespace open3d::visualization;
    using namespace std::chrono;

    // Create a torus and a vector of triangle bounds
    auto mesh = TriangleMesh::CreateTorus();
    auto bounds = std::make_shared<std::vector<TriangleBounds>>();

    for (auto t : mesh->triangles_) {
        bounds->emplace_back(mesh->vertices_[t[0]], mesh->vertices_[t[1]],
                             mesh->vertices_[t[2]]);
    }

    // Create a top down BVH
    auto f = [](const TriangleBounds& tri) { return tri.GetBoundingBox(); };
    auto bvh = Bvh<TriangleBounds>::CreateTopDown(f, bounds);

    // Create some random rays
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_real_distribution<double> dist(-1, 1);

    std::vector<Ray3D> rays;
    for (size_t i = 0; i < 1000000; ++i) {
        Ray3D r{{dist(mt), dist(mt), dist(mt)}, {dist(mt), dist(mt), dist(mt)}};
        rays.emplace_back(r);
    }

    //    auto line = std::make_shared<LineSet>();
    //    line->points_.emplace_back(0, 0, 0);
    //    line->points_.emplace_back(ray.Direction() * 3.);
    //    line->lines_.emplace_back(0, 1);
    //    line->PaintUniformColor({1, 0, 0});
    ulong c_f = 0;

    auto start = steady_clock::now();
    for (const auto& r : rays) {
        auto potential = bvh->PossibleIntersections(
                [&r](const AxisAlignedBoundingBox& box) {
                    return r.SlabAABB(box).has_value();
                });
        c_f += potential.size();
    }
    auto end = steady_clock::now();
    auto function_time = end - start;
    std::cout << "lambda found " << c_f << " potential and took "
              << duration_cast<milliseconds>(function_time).count() << " ms"
              << std::endl;

    ulong c_t = 0;
    start = steady_clock::now();
    for (const auto& r : rays) {
        auto potential = bvh->PossibleIntersections(r);
        c_t += potential.size();
    }
    end = steady_clock::now();
    auto trad_time = end - start;
    std::cout << "traditional found " << c_t << " potential and took "
              << duration_cast<milliseconds>(trad_time).count() << " ms"
              << std::endl;

    //    // Visualization
    //    //
    //    //
    //    ======================================================================
    //    mesh->PaintUniformColor({.5, .5, .5});
    //    mesh->ComputeVertexNormals();
    //
    //    std::vector<std::shared_ptr<const Geometry>> geometries{mesh, line};
    //
    //    for (auto i : potential) {
    //        auto box = LineSet::CreateFromAxisAlignedBoundingBox(
    //                (*bounds)[i].GetBoundingBox());
    //        geometries.push_back(box);
    //    }
    //
    //    std::function<void(const bvh::BvhNode<TriangleBounds>&)> recurse;
    //    recurse = [&geometries,
    //               &recurse](const bvh::BvhNode<TriangleBounds>& node) {
    //        auto box = LineSet::CreateFromAxisAlignedBoundingBox(node.Box());
    //        geometries.push_back(box);
    //        if (node.left_) recurse(*node.left_);
    //        if (node.right_) recurse(*node.right_);
    //    };
    //
    //    recurse(bvh->Root());

    //    for (const auto& b : *bounds) {
    //        auto box =
    //                LineSet::CreateFromAxisAlignedBoundingBox(b.GetBoundingBox());
    //        geometries.push_back(box);
    //    }
    //    DrawGeometries(geometries);

    //    printf("DOne!\n");
}