#include "LineBase.h"

namespace open3d {
namespace geometry {

LineBase::LineBase(const Eigen::Vector3d& origin,
                   const Eigen::Vector3d& direction,
                   LineBase::LineType type)
    : Eigen::ParametrizedLine<double, 3>(origin, direction), line_type_(type) {}

Line3D::Line3D(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction)
    : LineBase(origin, direction, LineType::Line) {}

Ray3D::Ray3D(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction)
    : LineBase(origin, direction, LineType::Ray) {}

Segment3D::Segment3D(const Eigen::Vector3d& start_point,
                     const Eigen::Vector3d& end_point)
    : LineBase(start_point, end_point - start_point, LineType::Segment),
      end_point_(end_point),
      length_((start_point - end_point_).norm()) {}

}  // namespace geometry
}  // namespace open3d