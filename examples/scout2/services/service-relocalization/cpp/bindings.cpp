#include "bbs.hpp"

#include <cmath>
#include <memory>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace relocalization;
void bind_registration(py::module_& module);

// Copy Python buffers before releasing the GIL; callers cannot mutate a search.
static std::unique_ptr<BbsSearch> make_search(const py::list& arrays, double resolution,
                                            double origin_x, double origin_y) {
    if (arrays.empty() || arrays.size() > 16) throw std::invalid_argument("expected 1 to 16 BBS layers");
    std::vector<std::vector<std::uint8_t>> layers;
    int width = 0, height = 0;
    for (const auto& object : arrays) {
        auto array = py::array_t<std::uint8_t, py::array::c_style>::ensure(object);
        if (!array || array.ndim() != 2 || array.shape(0) <= 0 || array.shape(1) <= 0 ||
            array.size() > 100000000)
            throw std::invalid_argument("BBS layers must be contiguous 2D uint8 arrays");
        if (layers.empty()) { width = static_cast<int>(array.shape(1)); height = static_cast<int>(array.shape(0)); }
        if (array.shape(1) != width || array.shape(0) != height)
            throw std::invalid_argument("BBS layer dimensions differ");
        layers.emplace_back(array.data(), array.data() + array.size());
    }
    py::gil_scoped_release release;
    return std::make_unique<BbsSearch>(width, height, resolution, origin_x, origin_y, std::move(layers));
}

// Include scan copying in the deadline and discard partial results on failure.
static SearchResult run_search(const BbsSearch& search,
                               const py::array_t<double, py::array::c_style | py::array::forcecast>& array,
                               const SearchOptions& options, double timeout_s,
                               const std::shared_ptr<std::atomic_bool>& cancelled) {
    if (!std::isfinite(timeout_s) || timeout_s <= 0 || timeout_s > 120)
        throw std::invalid_argument("BBS timeout_s must be in (0, 120]");
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_s));
    if (!cancelled) throw std::invalid_argument("BBS cancellation token is required");
    if (array.ndim() != 2 || array.shape(1) != 2 || array.shape(0) < 1 || array.shape(0) > 10000)
        throw std::invalid_argument("BBS scan must have shape (N, 2), 1 <= N <= 10000");
    const auto input = array.unchecked<2>();
    std::vector<Point2> points;
    points.reserve(array.shape(0));
    for (py::ssize_t i = 0; i < array.shape(0); ++i) points.push_back({input(i, 0), input(i, 1)});
    const SearchOptions owned_options = options;
    const auto owned_token = cancelled;
    py::gil_scoped_release release;
    return search.search(points, owned_options, deadline, *owned_token);
}

// Internal extension only: no new Atlas contracts or robot actions.
PYBIND11_MODULE(_relocalization_bbs, module) {
    py::register_exception<SearchCancelled>(module, "SearchCancelled", PyExc_RuntimeError);
    py::register_exception<SearchTimeout>(module, "SearchTimeout", PyExc_RuntimeError);
    py::class_<std::atomic_bool, std::shared_ptr<std::atomic_bool>>(module, "CancellationToken")
        .def(py::init([] { return std::make_shared<std::atomic_bool>(false); }))
        .def_property_readonly("cancelled", [](const std::atomic_bool& token) { return token.load(std::memory_order_relaxed); })
        .def("cancel", [](std::atomic_bool& token) { token.store(true, std::memory_order_relaxed); });
    py::class_<SearchOptions>(module, "SearchOptions")
        .def(py::init<>())
        .def("__copy__", [](const SearchOptions& options) { return options; })
        .def_readwrite("yaw_step", &SearchOptions::yaw_step)
        .def_readwrite("min_score", &SearchOptions::min_score)
        .def_readwrite("separation_m", &SearchOptions::separation_m)
        .def_readwrite("separation_rad", &SearchOptions::separation_rad)
        .def_readwrite("max_candidates", &SearchOptions::max_candidates);
    py::class_<Candidate>(module, "Candidate")
        .def_readonly("x", &Candidate::x).def_readonly("y", &Candidate::y)
        .def_readonly("yaw", &Candidate::yaw).def_readonly("score", &Candidate::score);
    py::class_<SearchResult>(module, "SearchResult")
        .def_readonly("candidates", &SearchResult::candidates)
        .def_readonly("evaluated_nodes", &SearchResult::evaluated_nodes);
    py::class_<BbsSearch>(module, "BbsSearch")
        .def(py::init(&make_search), py::arg("layers"), py::arg("resolution"),
             py::arg("origin_x"), py::arg("origin_y"))
        .def("search", &run_search, py::arg("points"), py::arg("options"),
             py::arg("timeout_s"), py::arg("cancelled"));
    bind_registration(module);
}
