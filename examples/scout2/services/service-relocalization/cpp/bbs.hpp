#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace relocalization {

struct Point2 { double x, y; };
struct Candidate { double x, y, yaw, score; };
struct SearchOptions {
    double yaw_step = 0.05;
    double min_score = 0.3;
    double separation_m = 0.5;
    double separation_rad = 0.35;
    std::size_t max_candidates = 5;
};
struct SearchResult {
    std::vector<Candidate> candidates;
    std::size_t evaluated_nodes = 0;
};
struct SearchCancelled : std::runtime_error {
    SearchCancelled() : std::runtime_error("BBS search cancelled") {}
};
struct SearchTimeout : std::runtime_error {
    SearchTimeout() : std::runtime_error("BBS search deadline exceeded") {}
};

// Owns immutable forward-window bounds; constructed only from checked layers.
class BbsSearch {
public:
    BbsSearch(int width, int height, double resolution, double origin_x,
              double origin_y, std::vector<std::vector<std::uint8_t>> layers);

    // Searches all base cell centres and uniform headings in [-pi, pi).
    // Leaves are visited in score order. Greedy suppression rejects a pose
    // only when BOTH its position and wrapped heading are near an accepted one.
    // Returned scores are occupied endpoint fractions, never probabilities.
    // Interruption throws and discards partial results. No robot side effects.
    SearchResult search(const std::vector<Point2>& points, const SearchOptions& options,
                        std::chrono::steady_clock::time_point deadline,
                        const std::atomic_bool& cancelled) const;

private:
    int width_, height_;
    double resolution_, origin_x_, origin_y_;
    std::vector<std::vector<std::uint8_t>> layers_;
};

}  // namespace relocalization
