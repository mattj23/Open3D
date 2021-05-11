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

#include "open3d/geometry/TriangleMeshBvh.h"

#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include "open3d/Open3D.h"

using namespace open3d::geometry;

size_t test_fn(std::function<AxisAlignedBoundingBox(TriangleBounds)> fn,
               std::shared_ptr<std::vector<TriangleBounds>> bounds) {
    std::vector<AxisAlignedBoundingBox> boxes;
    boxes.reserve(bounds->size());

    for (size_t i = 0; i < bounds->size(); ++i) {
        boxes.push_back(fn((*bounds)[i]));
    }
    return boxes.size();
}

size_t test_manual(std::shared_ptr<std::vector<TriangleBounds>> bounds) {
    std::vector<AxisAlignedBoundingBox> boxes;
    boxes.reserve(bounds->size());

    for (size_t i = 0; i < bounds->size(); ++i) {
        boxes.push_back((*bounds)[i].GetBoundingBox());
    }
    return boxes.size();
}

int main(int argc, char** argv) {
    using namespace open3d::visualization;
    using namespace std::chrono;

    // Create a torus and a vector of triangle bounds
    auto mesh = TriangleMesh::CreateBox();
    auto box = mesh->GetAxisAlignedBoundingBox();
    auto extent = box.GetExtent();
    printf("%f\n", extent.z());
    auto bvh = TriangleMeshBvh::TopDown(*mesh, {});

    Ray3D ray{{0, 0, 0}, {1, 1, 1}};

    auto line = std::make_shared<LineSet>();
    line->points_.emplace_back(0, 0, 0);
    line->points_.emplace_back(ray.Direction() * 3.);
    line->lines_.emplace_back(0, 1);
    line->PaintUniformColor({1, 0, 0});

    // Visualization
    // ======================================================================
    mesh->PaintUniformColor({.5, .5, .5});
    mesh->ComputeVertexNormals();

    auto m2 = TriangleMesh::CreateCoordinateFrame();
    std::vector<std::shared_ptr<const Geometry>> geometries{mesh, line, m2};

//    auto potential = bvh->PossibleIntersections(
//            [&ray](const AxisAlignedBoundingBox& box) {
//                return ray.SlabAABB(box).has_value();
//            });
//
//    for (auto i : potential) {
//        auto box = LineSet::CreateFromAxisAlignedBoundingBox(
//                (*bounds)[i].GetBoundingBox());
//        geometries.push_back(box);
//    }

    //    std::function<void(const bvh::BvhNode<TriangleBounds>&)> recurse;
    //    recurse = [&geometries,
    //               &recurse](const bvh::BvhNode<TriangleBounds>& node) {
    //        auto box = LineSet::CreateFromAxisAlignedBoundingBox(node.Box());
    //        geometries.push_back(box);
    //        if (node.left_) recurse(*node.left_);
    //        if (node.right_) recurse(*node.right_);
    //    };

    //    recurse(bvh->Root());

    //    for (const auto& b : *bounds) {
    //        auto box =
    //                LineSet::CreateFromAxisAlignedBoundingBox(b.GetBoundingBox());
    //        geometries.push_back(box);
    //    }
    DrawGeometries(geometries);

    printf("DOne!\n");
}