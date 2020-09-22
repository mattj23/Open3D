#include "LineBase.h"

namespace open3d {
namespace geometry {

LineBase::LineBase(const Eigen::Vector3d& origin,
                   const Eigen::Vector3d& direction,
                   LineBase::LineType type)
    : Eigen::ParametrizedLine<double, 3>(origin, direction), line_type_(type) {
    x_inv_ = 1. / direction.x();
    y_inv_ = 1. / direction.y();
    z_inv_ = 1. / direction.z();
}

utility::optional<double> LineBase::SlabAABB(
        const AxisAlignedBoundingBox& box) const {
    double t_x0 = x_inv_ * (box.min_bound_.x() - origin().x());
    double t_x1 = x_inv_ * (box.max_bound_.x() - origin().x());
    double t_min = std::min(t_x0, t_x1);
    double t_max = std::max(t_x0, t_x1);

    double t_y0 = y_inv_ * (box.min_bound_.y() - origin().y());
    double t_y1 = y_inv_ * (box.max_bound_.y() - origin().y());
    t_min = std::max(t_min, std::min(t_y0, t_y1));
    t_max = std::min(t_max, std::max(t_y0, t_y1));

    double t_z0 = z_inv_ * (box.min_bound_.z() - origin().z());
    double t_z1 = z_inv_ * (box.max_bound_.z() - origin().z());
    t_min = std::max(t_min, std::min(t_z0, t_z1));
    t_max = std::min(t_max, std::max(t_z0, t_z1));

    // Because line_type_ is a const enum, in cases where the line type is
    // known at compile time the optimizer should reduce the function to the
    // necessary options
    if (line_type_ == LineType::Ray || line_type_ == LineType::Segment) {
        t_min = std::max(0., t_min);
//        t_min = std::max(LineBase::limits_[static_cast<int>(line_type_)], t_min);
    }

    // If segment intersections beyond the endpoint of the segment ever stop
    // working, the likely culprit is a change in the underlying Eigen library
    // enforcing normalization on ParametrizedLine's direction
    if (line_type_ == LineType::Segment) {
        if (t_max >= t_min && t_min <= 1.) return t_min;
        return {};
    }

    if (t_max >= t_min) return t_min;
    return {};
}

utility::optional<double> LineBase::ExactAABB(
        const AxisAlignedBoundingBox& box) const {
    return utility::optional<double>();
}

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