// ----------------------------------------------------------------------------
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

namespace open3d {
namespace geometry {

TriangleMeshBvh TriangleMeshBvh::TopDown(const TriangleMesh& mesh,
                                         const bvh::SplitOptions& options) {
    TriangleMeshBvh bvh;
    bvh.triangles_.reserve(mesh.triangles_.size());
    bvh.bounds_.reserve(mesh.triangles_.size());

    for (const auto& t : mesh.triangles_) {
        bvh.triangles_.emplace_back(mesh.vertices_[t[0]], mesh.vertices_[t[1]],
                                    mesh.vertices_[t[2]]);
        bvh.bounds_.emplace_back(bvh.triangles_.back().GetBoundingBox());
    }

    bvh.bvh_ = Bvh::CreateTopDown([&bvh](size_t i) { return bvh.bounds_[i]; },
                                  bvh.bounds_.size(), options);

    return bvh;
}
}  // namespace geometry
}  // namespace open3d
