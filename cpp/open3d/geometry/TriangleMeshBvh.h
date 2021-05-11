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

#pragma once

#include "open3d/geometry/Bvh.h"
#include "open3d/geometry/Line3D.h"
#include "open3d/geometry/TriangleBounds.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/utility/Optional.h"

namespace open3d {
namespace geometry {

/// \class TriangleMeshBvh
///
/// \brief An acceleration structure for performing efficient intersection and
/// distance checks against a collection of triangles, i.e. from a TriangleMesh
///
/// \details This class uses a Bounding Volume Hierarchy to perform accelerated
/// distance and intersection checks against a collection of triangles.  It can
/// be created directly from a TriangleMesh object.
///
/// Internally, a TriangleMeshBvh uses an open3d::geometry::Bvh and a
/// std::vector of TriangleBounds objects.  The TriangleBounds objects are
/// created and stored independently and maintain no link to the original
/// triangles, so transformations and changes to whatever was used to create the
/// TriangleMeshBvh are not synchronized.
///
/// While a TriangleMeshBvh can reduce the computation needed for distance and
/// intersection checks against a TriangleMesh by many orders of magnitude, be
/// aware that it may take up quite a bit more space in memory than the original
/// TriangleMesh.
class TriangleMeshBvh {
public:
    /// \brief Create a TriangleMeshBvh directly from a TriangleMesh using a top
    /// down BVH construction method. Be aware that no link is maintained
    /// between the original TriangleMesh and the resulting TriangleMeshBvh, so
    /// any change to the TriangleMesh will effectively invalidate the Bvh.
    /// \code{.cpp}
    /// auto mesh = TriangleMesh::CreateTorus();
    ///
    /// // Create with default options
    /// auto bvh = TriangleMeshBvh::TopDown(*mesh, {});
    /// auto bvh = TriangleMeshBvh::TopDown(*mesh, bvh::SplitOptions::None());
    ///
    /// // Create with options
    /// bvh::SplitOptions opts;
    /// opts.min_primitives = 3; // exclude for default of 1
    /// opts.volume_ratio = 1.5; // exclude for no volume ratio check
    /// auto bvh = TriangleMeshBvh::TopDown(*mesh, opts);
    /// \endcode
    /// \param mesh a reference to the TriangleMesh to create the Bvh from
    /// \param options construction options for the top-down method. See
    /// SplitOptions class for more details.
    static TriangleMeshBvh TopDown(const TriangleMesh& mesh,
                                   const bvh::SplitOptions& options);

    /// \brief Checks if a point lies inside the TriangleMeshBvh by using the
    /// ray intersection counting method. This method can only be ensured to
    /// produce valid results when the mesh is watertight and does not contain
    /// internal, enclosed faces.
    /// \param point A point to test
    bool IsPointInside(const Eigen::Vector3d& point) const;

    /// \brief Returns true if any triangle in the Bvh intersects with a test
    /// Line3D/Ray3D/Segment3D. This method will stop when the first
    /// intersection is found, so it is more efficient than other methods at
    /// determining whether any intersection exists.
    /// \param line A Line3D/Ray3D/Segment3D to perform the intersections with
    /// \param exact If true use the exact intersection method with the AABB,
    /// otherwise uses the branchless slab method. See the Line3D class'
    /// SlabAABB and ExactAABB methods for further information.
    bool HasIntersectionWith(const Line3D& line, bool exact = false) const;

    /// \brief Returns the lowest intersection parameter of a
    /// Line3D/Ray3D/Segment3D with any triangle in the Bvh, or an empty
    /// optional if no such intersection exists. The lower a parameter value is
    /// the further back in the direction of the line it lies, so the lowest
    /// intersection parameter on a Ray3D or a Segment3D the closer it is to the
    /// line origin. You can use this to determine the first point intersected
    /// by a ray or a segment.
    /// \param line A Line3D/Ray3D/Segment3D to perform the intersections with
    /// \param exact If true use the exact intersection method with the AABB,
    /// otherwise uses the branchless slab method. See the Line3D class'
    /// SlabAABB and ExactAABB methods for further information.
    utility::optional<double> LowestIntersectionParameter(
            const Line3D& line, bool exact = false) const;

    /// \brief Returns the intersection point with the lowest line parameter of
    /// a Line3D/Ray3D/Segment3D with any triangle in the Bvh, or an empty
    /// optional if no such intersection exists. The lower a parameter value is
    /// the further back in the direction of the line it lies, so the lowest
    /// intersection parameter on a Ray3D or a Segment3D the closer it is to the
    /// line origin. You can use this to determine the first point intersected
    /// by a ray or a segment.
    /// \param line A Line3D/Ray3D/Segment3D to perform the intersections with
    /// \param exact If true use the exact intersection method with the AABB,
    /// otherwise uses the branchless slab method. See the Line3D class'
    /// SlabAABB and ExactAABB methods for further information.
    utility::optional<Eigen::Vector3d> LowestIntersection(
            const Line3D& line, bool exact = false) const;

    /// \brief Returns all intersection parameters of a Line3D/Ray3D/Segment3D
    /// with the triangles in the Bvh.
    /// \param line A Line3D/Ray3D/Segment3D to perform the intersections with
    /// \param exact If true use the exact intersection method with the AABB,
    /// otherwise uses the branchless slab method. See the Line3D class'
    /// SlabAABB and ExactAABB methods for further information.
    std::vector<double> IntersectionParameters(const Line3D& line,
                                               bool exact = false) const;

    /// \brief Returns all intersection points of a Line3D/Ray3D/Segment3D
    /// with the triangles in the Bvh.
    /// \param line A Line3D/Ray3D/Segment3D to perform the intersections with
    /// \param exact If true use the exact intersection method with the AABB,
    /// otherwise uses the branchless slab method. See the Line3D class'
    /// SlabAABB and ExactAABB methods for further information.
    std::vector<Eigen::Vector3d> Intersections(const Line3D& line,
                                               bool exact = false) const;

    /// \brief Return the closest point on any triangle in the Bvh from a test
    /// point.
    /// \param point the test point
    Eigen::Vector3d ClosestPointTo(const Eigen::Vector3d& point) const;

    /// \brief Return the farthest point on any triangle in the Bvh from a test
    /// point.
    /// \param point the test point
    Eigen::Vector3d FarthestPointFrom(const Eigen::Vector3d& point) const;

private:
    std::unique_ptr<Bvh> bvh_;
    std::vector<TriangleBounds> triangles_;
    std::vector<AxisAlignedBoundingBox> bounds_;

    std::vector<size_t> LineIntersectionCandidates(const Line3D& line,
                                                   bool exact) const;
};
}  // namespace geometry
}  // namespace open3d
