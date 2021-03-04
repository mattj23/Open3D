// --------------------------------------------------------EXAMPLE_CPP(LineSet                   ${CMAKE_PROJECT_NAME})--------------------
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

#include <memory>
#include <vector>
#include "open3d/Open3D.h"

int main(int argc, char **argv) {
    using namespace open3d::geometry;
    using namespace open3d::visualization;

    auto mesh = TriangleMesh::CreateTorus();
    auto bounds = std::make_shared<std::vector<TriangleBounds>>();

    for (auto t : mesh->triangles_) {
        bounds->emplace_back(mesh->vertices_[t[0]],
                             mesh->vertices_[t[1]],
                             mesh->vertices_[t[2]]);
    }

    mesh->PaintUniformColor({.5, .5, .5});
    mesh->ComputeVertexNormals();

    std::vector<std::shared_ptr<const Geometry>> geometries { mesh };
    DrawGeometries(geometries);

    printf("DOne!\n");
}