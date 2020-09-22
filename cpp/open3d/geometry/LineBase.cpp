#include "LineBase.h"

namespace open3d {
namespace geometry {

LineBase::LineBase(const Eigen::Vector3d& origin,
                   const Eigen::Vector3d& direction,
                   LineBase::LineType type)
    : Eigen::ParametrizedLine<double, 3>(origin, direction), line_type_(type) {
    // Here we pre-compute the inverses of the direction vector for use during
    // the slab AABB intersection algorithm. The choice to do this regardless
    // of the purpose of the line's creation isn't optimal, but has to do with
    // trying to keep both the outer API clean and avoid the overhead of having
    // to check that the values have been computed before running any of the
    // slab intersection algorithms.
    x_inv_ = 1. / direction.x();
    y_inv_ = 1. / direction.y();
    z_inv_ = 1. / direction.z();
}

std::pair<double, double> LineBase::SlabAABBBase(
        const AxisAlignedBoundingBox& box) const {
    /* This code is based off of Tavian Barnes' branchless implementation of
     * the slab method for determining ray/AABB intersections. It treats the
     * space inside the bounding box as three sets of parallel planes and clips
     * the line where it passes between these planes to produce an
     * ever-shrinking line segment.
     *
     * The line segment is represented by two scalar parameters, one for the
     * clipped end on the far plane and one for the clipped end on the near
     * plane. When the line does not pass through the bounding box the t_max
     * and t_min distances will invert.
     *
     * https://tavianator.com/2011/ray_box.html */
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

    return {t_min, t_max};
}

utility::optional<double> Line3D::SlabAABB(
        const AxisAlignedBoundingBox& box) const {
    /* The base case of the Line/AABB intersection allows for any intersection
     * along the direction of the line at any distance, in accordance with the
     * semantic meaning of a line.
     *
     * In such a case the only test is to determine if t_min and t_max have
     * inverted, indicating that the line does not pass through the box at all.
     */
    const auto& t = SlabAABBBase(box);
    double t_min = std::get<0>(t);
    double t_max = std::get<1>(t);

    if (t_max >= t_min) return t_min;
    return {};
}

utility::optional<double> LineBase::ExactAABB(
        const AxisAlignedBoundingBox& box) const {
    /* This is a naive, exact method of computing the intersection with a
     * bounding box.  It is much slower than the highly optimized slab method,
     * but will perform correctly in the one case where the slab method
     * degenerates: when a ray lies exactly within one of the bounding planes.
     * If your problem is structured such that the slab method is likely to
     * encounter a degenerate scenario, AND you need an exact solution that can
     * not allow the occasional non-intersection, AND you care about maximal
     * performance, consider implementing a special check which takes advantage
     * of the reduced dimensionality of your problem.
     */
    using namespace Eigen;

    // When running the stress test in examples/LineToAABB.cpp about 1% to 2%
    // of the randomly generated cases will fail when using this method due to
    // the round-trip vector coming back from the ParameterizedLine's
    // intersectionParameter method being off in the 11th or greater decimal
    // position from the original plane point. This tolerance seems to
    // eliminate the issue.
    double tol = 1e-10;
    AxisAlignedBoundingBox b_tol{box.min_bound_ - Vector3d(tol, tol, tol),
                                 box.max_bound_ + Vector3d(tol, tol, tol)};

    using plane_t = Eigen::Hyperplane<double, 3>;
    std::array<plane_t, 6> planes{{{{-1, 0, 0}, box.min_bound_},
                                   {{1, 0, 0}, box.max_bound_},
                                   {{0, -1, 0}, box.min_bound_},
                                   {{0, 1, 0}, box.max_bound_},
                                   {{0, 0, -1}, box.min_bound_},
                                   {{0, 0, 1}, box.max_bound_}}};

    // Get the intersections
    std::vector<double> parameters;
    std::vector<Eigen::Vector3d> points;
    parameters.reserve(7);
    points.reserve(7);

    if (line_type_ == LineType::Ray || line_type_ == LineType::Segment) {
        parameters.push_back(0);
        points.push_back(origin());
    }

    for (int i = 0; i < 6; ++i) {
        double t = intersectionParameter(planes[i]);
        if (!std::isinf(t) && t >= 0) {
            parameters.push_back(t);
            auto p = pointAt(t);
            points.push_back(p);
        }
    }

    // Find the ones which are contained
    auto contained_indices = b_tol.GetPointIndicesWithinBoundingBox(points);
    if (contained_indices.empty()) return {};

    // Return the lowest parameter
    double minimum = parameters[contained_indices[0]];
    for (auto i : contained_indices) {
        minimum = std::min(minimum, parameters[i]);
    }
    return minimum;
}

Line3D::Line3D(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction)
    : LineBase(origin, direction, LineType::Line) {}

Ray3D::Ray3D(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction)
    : LineBase(origin, direction, LineType::Ray) {}

utility::optional<double> Ray3D::SlabAABB(
        const AxisAlignedBoundingBox& box) const {
    /* The ray case of the Line/AABB intersection allows for any intersection
     * along the positive direction of the line at any distance, in accordance
     * with the semantic meaning of a ray.
     *
     * In this case there are two conditions which must be met: t_max must be
     * greater than t_min (the check for inversion), but it must also be greater
     * than zero, as a negative t_max would indicate that the most positive
     * intersection along the ray direction still lies behind the ray origin.
     */
    const auto& t = SlabAABBBase(box);
    double t_min = std::get<0>(t);
    double t_max = std::get<1>(t);

    t_min = std::max(0., t_min);

    if (t_max >= t_min) return t_min;
    return {};
}

Segment3D::Segment3D(const Eigen::Vector3d& start_point,
                     const Eigen::Vector3d& end_point)
    : LineBase(start_point, end_point - start_point, LineType::Segment),
      end_point_(end_point),
      length_((start_point - end_point_).norm()) {}

utility::optional<double> Segment3D::SlabAABB(
        const AxisAlignedBoundingBox& box) const {
    /* The segment case of the Line/AABB intersection only allows intersections
     * along the positive direction of the line which are at a distance less
     * than the segment length, in accordance with the semantic meaning of a
     * line segment which is finite.
     *
     * In this case the same conditions apply as with the ray case, but with one
     * additional requirement: t_min must be less than the length of the
     * segment. If t_min were greater than the length of the segment it would
     * indicate that the very earliest intersection along the line direction
     * occurs beyond the endpoint of the segment.
     */
    const auto& t = SlabAABBBase(box);
    double t_min = std::get<0>(t);
    double t_max = std::get<1>(t);

    t_min = std::max(0., t_min);

    if (t_max >= t_min && t_min <= length_) return t_min;
    return {};
}

utility::optional<double> Segment3D::ExactAABB(
        const AxisAlignedBoundingBox& box) const {
    auto result = LineBase::ExactAABB(box);
    if (!result.has_value() || result.value() <= length_) return result;

    return {};
}

}  // namespace geometry
}  // namespace open3d