#include "registration.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>

namespace relocalization {
namespace {
constexpr double pi = 3.14159265358979323846;
using Pose = std::array<double, 3>;
using Pairs = std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;

// Interrupt native work without acquiring the Python GIL.
void check_stop(std::chrono::steady_clock::time_point deadline, const std::atomic_bool& cancelled) {
    if (cancelled.load(std::memory_order_relaxed)) throw RegistrationCancelled();
    if (std::chrono::steady_clock::now() >= deadline) throw RegistrationTimeout();
}

// Bound coordinates for float32 PCL queries and reject invalid caller geometry.
bool valid_point(const Point3& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
           std::max({std::abs(point.x), std::abs(point.y), std::abs(point.z)}) <= 100000;
}

// Validate all thresholds before allocating or trusting any geometric evidence.
void validate(const std::vector<Point3>& scan, const Pose& pose, const RegistrationOptions& o) {
    const double positive[] = {o.max_correspondence_m, o.min_overlap, o.max_rmse_m,
        o.min_spread_m, o.max_translation_m, o.max_yaw_rad,
        o.translation_tolerance_m, o.yaw_tolerance_rad};
    for (double value : positive)
        if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid registration threshold");
    if (scan.size() < 3 || scan.size() > 100000 || o.min_correspondences < 3 ||
        o.min_correspondences > scan.size() || o.max_iterations < 1 || o.max_iterations > 200 ||
        o.min_overlap > 1 || o.max_correspondence_m > 5 || o.max_rmse_m > o.max_correspondence_m ||
        o.max_translation_m > 5 || o.max_yaw_rad > pi ||
        !std::isfinite(o.base_z) || std::abs(o.base_z) > 100000 ||
        !std::isfinite(o.roll) || std::abs(o.roll) > pi ||
        !std::isfinite(o.pitch) || std::abs(o.pitch) > pi ||
        !std::isfinite(pose[0]) || std::abs(pose[0]) > 100000 ||
        !std::isfinite(pose[1]) || std::abs(pose[1]) > 100000 || !std::isfinite(pose[2]))
        throw std::invalid_argument("invalid registration scan, pose or options");
    for (const auto& point : scan)
        if (!valid_point(point)) throw std::invalid_argument("nonfinite or oversized registration scan");
}

// Preserve calibrated height, roll and pitch; only yaw participates in fitting.
Eigen::Matrix3d rotation(const Pose& pose, const RegistrationOptions& options) {
    return (Eigen::AngleAxisd(pose[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(options.pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(options.roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// Reject insufficient, duplicated or nearly collinear evidence conservatively.
RegistrationResult measure(const Pairs& pairs, std::size_t total, const Pose& pose,
                           const RegistrationOptions& options, std::size_t iterations) {
    const double overlap = static_cast<double>(pairs.size()) / total;
    if (pairs.size() < options.min_correspondences || overlap < options.min_overlap)
        throw RegistrationRejected("point cloud registration has insufficient overlap");
    Eigen::Vector2d source_mean = Eigen::Vector2d::Zero(), target_mean = Eigen::Vector2d::Zero();
    for (const auto& pair : pairs) {
        source_mean += pair.first.head<2>();
        target_mean += pair.second.head<2>();
    }
    source_mean /= pairs.size();
    target_mean /= pairs.size();
    Eigen::Matrix2d source_covariance = Eigen::Matrix2d::Zero(), target_covariance = Eigen::Matrix2d::Zero();
    double squared_error = 0;
    for (const auto& pair : pairs) {
        const Eigen::Vector2d source = pair.first.head<2>() - source_mean;
        const Eigen::Vector2d target = pair.second.head<2>() - target_mean;
        source_covariance += source * source.transpose();
        target_covariance += target * target.transpose();
        squared_error += (pair.first - pair.second).squaredNorm();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> source_solver(source_covariance / pairs.size());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> target_solver(target_covariance / pairs.size());
    if (source_solver.info() != Eigen::Success || target_solver.info() != Eigen::Success)
        throw std::runtime_error("point cloud registration spatial support calculation failed");
    const double spread = std::sqrt(std::max(0.0,
        std::min(source_solver.eigenvalues()[0], target_solver.eigenvalues()[0])));
    if (!std::isfinite(spread) || spread < options.min_spread_m)
        throw RegistrationRejected("point cloud registration has degenerate spatial support");
    return {pose[0], pose[1], std::remainder(pose[2], 2 * pi),
            std::sqrt(squared_error / pairs.size()), overlap, spread, pairs.size(), iterations};
}
}  // namespace

struct PlanarRegistration::Impl {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;

    // Correspondences use full 3D distance; unmatched endpoints remain in the
    // overlap denominator. Each operation owns its query buffers.
    Pairs correspond(const std::vector<Point3>& scan, const Pose& pose, const RegistrationOptions& options,
                     std::chrono::steady_clock::time_point deadline, const std::atomic_bool& cancelled) const {
        const auto r = rotation(pose, options);
        const Eigen::Vector3d translation(pose[0], pose[1], options.base_z);
        const double limit = options.max_correspondence_m * options.max_correspondence_m;
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        Pairs pairs;
        pairs.reserve(scan.size());
        for (std::size_t i = 0; i < scan.size(); ++i) {
            if ((i & 63) == 0) check_stop(deadline, cancelled);
            const auto& p = scan[i];
            const Eigen::Vector3d transformed = r * Eigen::Vector3d(p.x, p.y, p.z) + translation;
            pcl::PointXYZ query(static_cast<float>(transformed.x()), static_cast<float>(transformed.y()),
                                static_cast<float>(transformed.z()));
            if (tree.nearestKSearch(query, 1, indices, distances) != 1)
                throw std::runtime_error("PCL registration nearest-neighbour query failed");
            if (distances[0] > limit) continue;
            const auto& target = (*cloud)[indices[0]];
            pairs.emplace_back(transformed, Eigen::Vector3d(target.x, target.y, target.z));
        }
        check_stop(deadline, cancelled);
        return pairs;
    }
};

// Copy map geometry once so Python mutation cannot invalidate the spatial index.
PlanarRegistration::PlanarRegistration(const std::vector<Point3>& map) : impl_(new Impl) {
    if (map.size() < 3 || map.size() > 2000000)
        throw std::invalid_argument("registration map must contain 3 to 2000000 points");
    impl_->cloud->reserve(map.size());
    for (const auto& point : map) {
        if (!valid_point(point)) throw std::invalid_argument("nonfinite or oversized registration map");
        impl_->cloud->emplace_back(point.x, point.y, point.z);
    }
    impl_->tree.setInputCloud(impl_->cloud);
}
PlanarRegistration::~PlanarRegistration() = default;

// Score without changing the supplied pose; no fitting frames are selected here.
RegistrationResult PlanarRegistration::evaluate(const std::vector<Point3>& scan, const Pose& pose,
        const RegistrationOptions& options, std::chrono::steady_clock::time_point deadline,
        const std::atomic_bool& cancelled) const {
    check_stop(deadline, cancelled);
    validate(scan, pose, options);
    const auto pairs = impl_->correspond(scan, pose, options, deadline, cancelled);
    const auto result = measure(pairs, scan.size(), pose, options, 0);
    if (result.rmse_m > options.max_rmse_m) throw RegistrationRejected("point cloud registration residual too large");
    check_stop(deadline, cancelled);
    return result;
}

// Exact planar least-squares updates preserve supplied height and tilt. Recheck
// correspondences and quality after convergence rather than returning stale fit metrics.
RegistrationResult PlanarRegistration::refine(const std::vector<Point3>& scan, const Pose& initial,
        const RegistrationOptions& options, std::chrono::steady_clock::time_point deadline,
        const std::atomic_bool& cancelled) const {
    check_stop(deadline, cancelled);
    validate(scan, initial, options);
    Pose pose = initial;
    for (std::size_t iteration = 1; iteration <= options.max_iterations; ++iteration) {
        const auto pairs = impl_->correspond(scan, pose, options, deadline, cancelled);
        measure(pairs, scan.size(), pose, options, iteration);
        Eigen::Vector2d source_mean = Eigen::Vector2d::Zero(), target_mean = Eigen::Vector2d::Zero();
        for (const auto& pair : pairs) {
            source_mean += pair.first.head<2>();
            target_mean += pair.second.head<2>();
        }
        source_mean /= pairs.size();
        target_mean /= pairs.size();
        double cosine = 0, sine = 0;
        for (const auto& pair : pairs) {
            const Eigen::Vector2d a = pair.first.head<2>() - source_mean;
            const Eigen::Vector2d b = pair.second.head<2>() - target_mean;
            cosine += a.dot(b);
            sine += a.x() * b.y() - a.y() * b.x();
        }
        if (std::hypot(cosine, sine) < 1e-12)
            throw RegistrationRejected("point cloud registration has degenerate correspondences");
        const double delta_yaw = std::atan2(sine, cosine);
        const Eigen::Rotation2Dd delta_rotation(delta_yaw);
        const Eigen::Vector2d delta_translation = target_mean - delta_rotation * source_mean;
        const Eigen::Vector2d next_xy = delta_rotation * Eigen::Vector2d(pose[0], pose[1]) + delta_translation;
        const double translation_step = (next_xy - Eigen::Vector2d(pose[0], pose[1])).norm();
        pose = {next_xy.x(), next_xy.y(), std::remainder(pose[2] + delta_yaw, 2 * pi)};
        if (std::hypot(pose[0] - initial[0], pose[1] - initial[1]) > options.max_translation_m ||
            std::abs(std::remainder(pose[2] - initial[2], 2 * pi)) > options.max_yaw_rad)
            throw RegistrationRejected("point cloud registration escaped candidate bounds");
        if (translation_step <= options.translation_tolerance_m && std::abs(delta_yaw) <= options.yaw_tolerance_rad) {
            auto result = evaluate(scan, pose, options, deadline, cancelled);
            result.iterations = iteration;
            return result;
        }
    }
    throw RegistrationRejected("point cloud registration did not converge");
}
}  // namespace relocalization
