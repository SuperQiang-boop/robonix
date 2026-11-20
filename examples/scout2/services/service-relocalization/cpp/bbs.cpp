#include "bbs.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <tuple>

namespace relocalization {
namespace {
constexpr double pi = 3.14159265358979323846;
struct Cell { int x, y; };
struct Node {
    int hits, heading, x, y, level;
    // The first potential leaf breaks score ties deterministically.
    bool operator<(const Node& other) const {
        return std::make_tuple(hits, -heading, -y, -x, -level) <
               std::make_tuple(other.hits, -other.heading, -other.y, -other.x, -other.level);
    }
};

// Use a monotonic deadline and an atomic token without calling back into Python.
void check_stop(std::chrono::steady_clock::time_point deadline, const std::atomic_bool& cancelled) {
    if (cancelled.load(std::memory_order_relaxed)) throw SearchCancelled();
    if (std::chrono::steady_clock::now() >= deadline) throw SearchTimeout();
}
}  // namespace

// Reject malformed or underestimated bounds before any search can prune a pose.
BbsSearch::BbsSearch(int width, int height, double resolution, double origin_x,
                     double origin_y, std::vector<std::vector<std::uint8_t>> layers)
    : width_(width), height_(height), resolution_(resolution), origin_x_(origin_x),
      origin_y_(origin_y), layers_(std::move(layers)) {
    if (width <= 0 || height <= 0 || static_cast<std::int64_t>(width) * height > 100000000 ||
        !std::isfinite(resolution) || resolution <= 0 || !std::isfinite(origin_x) ||
        !std::isfinite(origin_y) || !std::isfinite(origin_x + width * resolution) ||
        !std::isfinite(origin_y + height * resolution) || layers_.empty() || layers_.size() > 16)
        throw std::invalid_argument("invalid BBS grid geometry or layer count");
    bool occupied = false;
    for (std::size_t level = 0; level < layers_.size(); ++level) {
        const auto& layer = layers_[level];
        if (layer.size() != static_cast<std::size_t>(width) * height)
            throw std::invalid_argument("BBS layer dimensions differ");
        const int stride = level ? 1 << (level - 1) : 0;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const auto index = static_cast<std::size_t>(y) * width + x;
                if (layer[index] > 1) throw std::invalid_argument("BBS layers must contain 0 or 1");
                if (!level) { occupied |= layer[index] != 0; continue; }
                const auto& previous = layers_[level - 1];
                auto expected = previous[index];
                if (x + stride < width) expected = std::max(expected, previous[index + stride]);
                const auto row_stride = static_cast<std::size_t>(stride) * width;
                if (y + stride < height) expected = std::max(expected, previous[index + row_stride]);
                if (x + stride < width && y + stride < height)
                    expected = std::max(expected, previous[index + row_stride + stride]);
                if (layer[index] != expected) throw std::invalid_argument("invalid BBS maximum pyramid");
            }
        }
    }
    if (!occupied) throw std::invalid_argument("BBS grid has no occupied cells");
}

// Best-first branch and bound enumerates exact leaves in nonincreasing score.
// Until K separated leaves exist, only the configured minimum may prune.
SearchResult BbsSearch::search(const std::vector<Point2>& points, const SearchOptions& options,
                             std::chrono::steady_clock::time_point deadline,
                             const std::atomic_bool& cancelled) const {
    check_stop(deadline, cancelled);
    if (points.empty() || points.size() > 10000 || !std::isfinite(options.yaw_step) ||
        options.yaw_step < 0.0001 || options.yaw_step > 2 * pi ||
        !std::isfinite(options.min_score) || options.min_score <= 0 || options.min_score > 1 ||
        !std::isfinite(options.separation_m) || options.separation_m < 0 ||
        !std::isfinite(options.separation_rad) || options.separation_rad < 0 ||
        options.separation_rad > pi || options.max_candidates < 1 || options.max_candidates > 100)
        throw std::invalid_argument("invalid BBS scan or search options");
    const int headings = static_cast<int>(std::ceil(2 * pi / options.yaw_step));
    if (points.size() * headings > 8000000)
        throw std::invalid_argument("BBS rotated scan exceeds 8 million points; downsample the scan");
    for (const auto& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            std::hypot(point.x, point.y) / resolution_ > 100000000)
            throw std::invalid_argument("nonfinite or oversized BBS scan coordinates");
    }
    std::vector<std::vector<Cell>> rotated(headings);
    std::priority_queue<Node> queue;
    SearchResult result;
    // A clamped negative start uses a larger window, which remains an upper
    // bound. Taking zero for an intersecting out-of-grid window is incorrect.
    const auto score = [&](int heading, int x, int y, int level) {
        check_stop(deadline, cancelled);
        ++result.evaluated_nodes;
        int hits = 0;
        const int size = 1 << level;
        const auto& cells = rotated[heading];
        for (std::size_t i = 0; i < cells.size(); ++i) {
            if ((i & 255) == 0) check_stop(deadline, cancelled);
            const int px = x + cells[i].x, py = y + cells[i].y;
            if (px >= width_ || py >= height_ || px + size <= 0 || py + size <= 0) continue;
            hits += layers_[level][static_cast<std::size_t>(std::max(py, 0)) * width_ + std::max(px, 0)];
        }
        return hits;
    };
    const auto push = [&](int heading, int x, int y, int level) {
        const int hits = score(heading, x, y, level);
        if (static_cast<double>(hits) / points.size() >= options.min_score) {
            // Bound memory independently of timeout on repetitive large maps.
            if (queue.size() >= 1000000) throw std::runtime_error("BBS queue exceeds 1 million nodes");
            queue.push(Node{hits, heading, x, y, level});
        }
    };
    const int top = static_cast<int>(layers_.size()) - 1;
    const int stride = 1 << top;
    for (int heading = 0; heading < headings; ++heading) {
        const double yaw = -pi + heading * (2 * pi / headings);
        const double c = std::cos(yaw), s = std::sin(yaw);
        auto& cells = rotated[heading];
        cells.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            if ((i & 255) == 0) check_stop(deadline, cancelled);
            const auto& point = points[i];
            cells.push_back(Cell{static_cast<int>(std::floor(0.5 + (c * point.x - s * point.y) / resolution_)),
                                 static_cast<int>(std::floor(0.5 + (s * point.x + c * point.y) / resolution_))});
        }
        for (int y = 0; y < height_; y += stride)
            for (int x = 0; x < width_; x += stride) push(heading, x, y, top);
    }
    while (!queue.empty()) {
        check_stop(deadline, cancelled);
        const Node node = queue.top();
        queue.pop();
        if (node.level) {
            const int step = 1 << (node.level - 1);
            for (int dy : {0, step})
                for (int dx : {0, step})
                    if (node.x + dx < width_ && node.y + dy < height_)
                        push(node.heading, node.x + dx, node.y + dy, node.level - 1);
            continue;
        }
        const Candidate candidate{origin_x_ + (node.x + 0.5) * resolution_,
                                  origin_y_ + (node.y + 0.5) * resolution_,
                                  -pi + node.heading * (2 * pi / headings),
                                  static_cast<double>(node.hits) / points.size()};
        bool suppressed = false;
        for (const auto& accepted : result.candidates) {
            if (std::hypot(candidate.x - accepted.x, candidate.y - accepted.y) <= options.separation_m &&
                std::abs(std::remainder(candidate.yaw - accepted.yaw, 2 * pi)) <= options.separation_rad) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed) result.candidates.push_back(candidate);
        // The remaining queue is bounded by this Kth exact score. This is
        // equivalent to pruning at the Kth separated candidate, never the first.
        if (result.candidates.size() == options.max_candidates) break;
    }
    check_stop(deadline, cancelled);
    return result;
}
}  // namespace relocalization
