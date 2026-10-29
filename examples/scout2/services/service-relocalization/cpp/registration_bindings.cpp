#include "registration.hpp"

#include <cmath>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace relocalization;
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;

// Copy buffers before releasing the GIL; reject oversized input before copying.
static std::vector<Point3> copy_points(const Array& array, py::ssize_t maximum) {
    if (array.ndim() != 2 || array.shape(1) != 3 || array.shape(0) < 3 || array.shape(0) > maximum)
        throw std::invalid_argument("registration points must have shape (N, 3) within size limits");
    const auto input = array.unchecked<2>();
    std::vector<Point3> points;
    points.reserve(array.shape(0));
    for (py::ssize_t i = 0; i < array.shape(0); ++i) points.push_back({input(i, 0), input(i, 1), input(i, 2)});
    return points;
}

// Map construction is offline; the native object owns both points and index.
static std::unique_ptr<PlanarRegistration> make_registration(const Array& array) {
    const auto points = copy_points(array, 2000000);
    py::gil_scoped_release release;
    return std::make_unique<PlanarRegistration>(points);
}

// Copy options, pose and token before releasing the GIL. Include scan copying
// in the deadline, throw on interruption, and never expose a partial result.
static RegistrationResult run_registration(const PlanarRegistration& registration, const Array& array,
        const std::array<double, 3>& pose, const RegistrationOptions& options, double timeout_s,
        const std::shared_ptr<std::atomic_bool>& cancelled, bool fit) {
    if (!std::isfinite(timeout_s) || timeout_s <= 0 || timeout_s > 120 || !cancelled)
        throw std::invalid_argument("registration requires timeout_s in (0, 120] and a cancellation token");
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_s));
    const auto points = copy_points(array, 100000);
    const auto owned_pose = pose;
    const auto owned_options = options;
    const auto owned_token = cancelled;
    py::gil_scoped_release release;
    return fit ? registration.refine(points, owned_pose, owned_options, deadline, *owned_token)
               : registration.evaluate(points, owned_pose, owned_options, deadline, *owned_token);
}

// Internal geometry API; it adds no RPC, status names or robot side effects.
void bind_registration(py::module_& module) {
    py::register_exception<RegistrationCancelled>(module, "RegistrationCancelled", PyExc_RuntimeError);
    py::register_exception<RegistrationTimeout>(module, "RegistrationTimeout", PyExc_RuntimeError);
    py::register_exception<RegistrationRejected>(module, "RegistrationRejected", PyExc_RuntimeError);
    py::class_<RegistrationOptions>(module, "RegistrationOptions")
        .def(py::init<>())
        .def("__copy__", [](const RegistrationOptions& options) { return options; })
        .def_readwrite("base_z", &RegistrationOptions::base_z)
        .def_readwrite("roll", &RegistrationOptions::roll)
        .def_readwrite("pitch", &RegistrationOptions::pitch)
        .def_readwrite("max_correspondence_m", &RegistrationOptions::max_correspondence_m)
        .def_readwrite("min_overlap", &RegistrationOptions::min_overlap)
        .def_readwrite("max_rmse_m", &RegistrationOptions::max_rmse_m)
        .def_readwrite("min_spread_m", &RegistrationOptions::min_spread_m)
        .def_readwrite("max_translation_m", &RegistrationOptions::max_translation_m)
        .def_readwrite("max_yaw_rad", &RegistrationOptions::max_yaw_rad)
        .def_readwrite("translation_tolerance_m", &RegistrationOptions::translation_tolerance_m)
        .def_readwrite("yaw_tolerance_rad", &RegistrationOptions::yaw_tolerance_rad)
        .def_readwrite("min_correspondences", &RegistrationOptions::min_correspondences)
        .def_readwrite("max_iterations", &RegistrationOptions::max_iterations);
    py::class_<RegistrationResult>(module, "RegistrationResult")
        .def_readonly("x", &RegistrationResult::x).def_readonly("y", &RegistrationResult::y)
        .def_readonly("yaw", &RegistrationResult::yaw).def_readonly("rmse_m", &RegistrationResult::rmse_m)
        .def_readonly("overlap", &RegistrationResult::overlap)
        .def_readonly("spread_m", &RegistrationResult::spread_m)
        .def_readonly("correspondences", &RegistrationResult::correspondences)
        .def_readonly("iterations", &RegistrationResult::iterations);
    py::class_<PlanarRegistration>(module, "PlanarRegistration")
        .def(py::init(&make_registration), py::arg("map_points"))
        .def("refine", [](const PlanarRegistration& r, const Array& p, const std::array<double, 3>& pose,
                          const RegistrationOptions& o, double timeout, const std::shared_ptr<std::atomic_bool>& c) {
            return run_registration(r, p, pose, o, timeout, c, true);
        }, py::arg("points"), py::arg("initial"), py::arg("options"), py::arg("timeout_s"), py::arg("cancelled"))
        .def("evaluate", [](const PlanarRegistration& r, const Array& p, const std::array<double, 3>& pose,
                            const RegistrationOptions& o, double timeout, const std::shared_ptr<std::atomic_bool>& c) {
            return run_registration(r, p, pose, o, timeout, c, false);
        }, py::arg("points"), py::arg("pose"), py::arg("options"), py::arg("timeout_s"), py::arg("cancelled"));
}
