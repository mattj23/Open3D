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

#pragma once

#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open3d/geometry/BoundingVolume.h"
#include "open3d/geometry/Geometry.h"
#include "open3d/utility/Optional.h"

#pragma once

namespace open3d {
namespace geometry {

class LineBase : protected Eigen::ParametrizedLine<double, 3> {
public:
    /// \enum LineType
    ///
    /// \brief Specifies different semantic interpretations of 3d lines
    enum class LineType {
        /// Lines extend infinitely in both directions
        Line = 0,

        /// Rays have an origin and a direction, and extend to infinity in
        /// that direction
        Ray = 1,

        /// Segments have both an origin and an endpoint and are finite in
        /// nature
        Segment = 2,
    };

    /// \brief Gets the semantic type of the line
    LineType GetLineType() const { return line_type_; }

    const Eigen::Vector3d& Origin() const { return m_origin; }
    const Eigen::Vector3d& Direction() const { return m_direction; }

    utility::optional<double> SlabAABB(const AxisAlignedBoundingBox& box) const;
    double SlabAABBTest(const AxisAlignedBoundingBox& box) const;
    utility::optional<double> ExactAABB(const AxisAlignedBoundingBox& box) const;


protected:
    LineBase(const Eigen::Vector3d& origin,
             const Eigen::Vector3d& direction,
             LineType type);

private:
    const LineType line_type_ = LineType::Line;
    double x_inv_;
    double y_inv_;
    double z_inv_;

    constexpr static const double limits_[]{std::numeric_limits<double>::min(), 0, 0};
};

/// \class Line3D
///
/// \brief A line is a semantic interpretation of Eigen::ParametrizedLine which
/// has an origin and a direction, but extends infinitely in both directions.
class Line3D : public LineBase {
public:
    Line3D(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction);
};

/// \class Ray3D
///
/// \brief A ray is a semantic interpretation of Eigen::ParametrizedLine which
/// has an origin and a direction and extends infinitely only in that specific
/// direction.
class Ray3D : public LineBase {
public:
    Ray3D(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction);
};

/// \class Segment3D
///
/// \brief A segment is a semantic interpretation of Eigen::ParametrizedLine
/// which has an origin and an endpoint and exists finitely between them.
///
/// \details One of the main motivations behind this class, and the LineBase
/// taxonomy in general, is the ambiguity of the Eigen documentation with
/// regards to the ParametrizedLine's direction. The documentation warns that
/// the direction vector is expected to be normalized and makes no guarantees
/// about behavior when this expectation is not met.  However, ParametrizedLine
/// does behave correctly when the direction vector is scaled. This class
/// exists as a seam to ensure the correct behavior can be produced regardless
/// of what happens in the underlying Eigen implementation without changing
/// the api surface for client code.
class Segment3D : public LineBase {
public:
    Segment3D(const Eigen::Vector3d& start_point,
              const Eigen::Vector3d& end_point);

    double GetLength() const { return length_; }
    const Eigen::Vector3d& EndPoint() const { return end_point_; }

private:
    Eigen::Vector3d end_point_;
    double length_;
};

}  // namespace geometry
}  // namespace open3d
