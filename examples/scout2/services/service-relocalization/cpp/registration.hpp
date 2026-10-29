#pragma once

#include "bbs.hpp"
#include <array>
#include <memory>

namespace relocalization {
struct Point3 { double x, y, z; };
struct RegistrationOptions {
    double base_z = 0, roll = 0, pitch = 0;
    double max_correspondence_m = 0.5;
    double min_overlap = 0.5;
    double max_rmse_m = 0.15;
    double min_spread_m = 0.05;
    double max_translation_m = 0.75, max_yaw_rad = 0.35;
    double translation_tolerance_m = 0.0001, yaw_tolerance_rad = 0.0001;
    std::size_t min_correspondences = 20, max_iterations = 40;
};
struct RegistrationResult {
    double x, y, yaw, rmse_m, overlap, spread_m;
    std::size_t correspondences, iterations;
};
struct RegistrationCancelled : std::runtime_error {
    RegistrationCancelled() : std::runtime_error("point cloud registration cancelled") {}
};
struct RegistrationTimeout : std::runtime_error {
    RegistrationTimeout() : std::runtime_error("point cloud registration deadline exceeded") {}
};
// Expected geometric rejection is distinct from invalid input or backend faults.
struct RegistrationRejected : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// Owns a checked immutable map cloud and its PCL nearest-neighbour index.
// Matching only changes x/y/yaw; height and tilt are supplied by a trusted caller.
// Results are geometric measurements, never a robot localization-valid decision.
class PlanarRegistration {
public:
    explicit PlanarRegistration(const std::vector<Point3>& map);
    ~PlanarRegistration();

    // Fit with bounded planar point-to-point ICP and reject nonconvergence, poor
    // overlap, large residual, spatial degeneracy or escape from the BBS seed.
    // Cancellation/deadline throw and discard all partial results.
    RegistrationResult refine(const std::vector<Point3>& scan, const std::array<double, 3>& initial,
                              const RegistrationOptions& options,
                              std::chrono::steady_clock::time_point deadline,
                              const std::atomic_bool& cancelled) const;

    // Measure an independent scan at a fixed pose without fitting it. Applies
    // the same evidence gates; caller must ensure independence and freshness.
    RegistrationResult evaluate(const std::vector<Point3>& scan, const std::array<double, 3>& pose,
                                const RegistrationOptions& options,
                                std::chrono::steady_clock::time_point deadline,
                                const std::atomic_bool& cancelled) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace relocalization
