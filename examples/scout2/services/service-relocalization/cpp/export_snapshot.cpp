// Read a saved RTAB-Map snapshot without substituting raw odometry poses.
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/SensorData.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>
#include <cstdint>
#include <iterator>

namespace fs = std::filesystem;
using rtabmap::Transform;

// Reject invalid transforms before a scan can contaminate the global cloud.
void validate(const Transform &pose) {
    if (pose.isNull()) {
        throw std::runtime_error("missing transform");
    }
    const cv::Mat rotation = pose.rotationMatrix();
    if (!cv::checkRange(pose.dataMatrix()) ||
        cv::norm(rotation * rotation.t() - cv::Mat::eye(3, 3, CV_32F)) > 1e-3 ||
        std::abs(cv::determinant(rotation) - 1.0) > 1e-3) {
        throw std::runtime_error("nonfinite or nonrigid transform");
    }
}

// Serialize the full row-major base/sensor or map/base rigid transform.
void writeTransform(cv::FileStorage &report, const char *key, const Transform &pose) {
    report << key << "[";
    for (int i = 0; i < 12; ++i) {
        report << pose[i];
    }
    report << "]";
}

// Only physical graph constraints establish connectivity, never virtual links.
bool physical(const rtabmap::Link &link) {
    return link.type() == rtabmap::Link::kNeighbor ||
        link.type() == rtabmap::Link::kNeighborMerged ||
        link.type() == rtabmap::Link::kGlobalClosure ||
        link.type() == rtabmap::Link::kLocalSpaceClosure ||
        link.type() == rtabmap::Link::kLocalTimeClosure ||
        link.type() == rtabmap::Link::kUserClosure;
}

// Partition the supplied node IDs using valid physical pose graph constraints.
std::vector<std::set<int>> components(const std::set<int> &ids,
                                    const std::multimap<int, rtabmap::Link> &links) {
    std::map<int, std::set<int>> edges;
    for (const auto &entry : links) {
        const auto &link = entry.second;
        if (physical(link) && ids.count(link.from()) && ids.count(link.to())) {
            validate(link.transform());
            edges[link.from()].insert(link.to());
            edges[link.to()].insert(link.from());
        }
    }
    std::set<int> remaining = ids;
    std::vector<std::set<int>> result;
    while (!remaining.empty()) {
        std::queue<int> pending;
        pending.push(*remaining.begin());
        std::set<int> group;
        while (!pending.empty()) {
            const int id = pending.front();
            pending.pop();
            if (!remaining.erase(id)) {
                continue;
            }
            group.insert(id);
            for (int neighbor : edges[id]) {
                pending.push(neighbor);
            }
        }
        result.push_back(std::move(group));
    }
    return result;
}

// Apply an explicitly directed transform; scan data itself is never in map frame.
std::array<float, 3> apply(const Transform &pose, const std::array<float, 3> &p) {
    return {pose.r11()*p[0] + pose.r12()*p[1] + pose.r13()*p[2] + pose.x(),
            pose.r21()*p[0] + pose.r22()*p[1] + pose.r23()*p[2] + pose.y(),
            pose.r31()*p[0] + pose.r32()*p[1] + pose.r33()*p[2] + pose.z()};
}

// Decode scans with the database's own codec and preserve per-node provenance.
void exportScans(rtabmap::DBDriver &db, const std::set<int> &selected,
                 const std::map<int, Transform> &poses, const fs::path &output,
                 cv::FileStorage &report) {
    std::ofstream cloud(output / "cloud.bin", std::ios::binary);
    std::ofstream scans(output / "scans.bin", std::ios::binary);
    cloud.exceptions(std::ios::badbit | std::ios::failbit);
    scans.exceptions(std::ios::badbit | std::ios::failbit);
    double offset = 0;
    report << "nodes" << "[";
    for (int id : selected) {
        Transform unusedPose, groundTruth;
        int mapId = -1, weight = 0;
        std::string label;
        double stamp = 0;
        std::vector<float> velocity;
        rtabmap::GPS gps;
        rtabmap::EnvSensors sensors;
        if (!db.getNodeInfo(id, unusedPose, mapId, weight, label, stamp,
                            groundTruth, velocity, gps, sensors)) {
            throw std::runtime_error("missing node metadata: " + std::to_string(id));
        }
        rtabmap::SensorData data;
        db.getNodeData(id, data, false, true, false, false);
        rtabmap::LaserScan scan;
        data.uncompressData(nullptr, nullptr, &scan);
        if (scan.empty() || scan.isCompressed() || scan.format() == rtabmap::LaserScan::kUnknown ||
            scan.data().depth() != CV_32F || scan.channels() < (scan.is2d() ? 2 : 3)) {
            throw std::runtime_error("missing or invalid decoded scan: " + std::to_string(id));
        }
        const auto &pose = poses.at(id);
        const Transform extrinsic = scan.localTransform();
        validate(pose);
        validate(extrinsic);
        int count = 0, discarded = 0;
        for (int row = 0; row < scan.data().rows; ++row) {
            for (int col = 0; col < scan.data().cols; ++col) {
                const float *raw = scan.data().ptr<float>(row) + col * scan.channels();
                const std::array<float, 3> point{raw[0], raw[1], scan.is2d() ? 0.0f : raw[2]};
                if (!std::all_of(point.begin(), point.end(), [](float v) { return std::isfinite(v); })) {
                    ++discarded;
                    continue;
                }
                const auto base = apply(extrinsic, point);
                const auto global = apply(pose, base);
                if (!std::all_of(global.begin(), global.end(), [](float v) { return std::isfinite(v); })) {
                    throw std::runtime_error("transformed scan is nonfinite");
                }
                scans.write(reinterpret_cast<const char *>(base.data()), sizeof(float)*3);
                cloud.write(reinterpret_cast<const char *>(global.data()), sizeof(float)*3);
                ++count;
            }
        }
        if (!count) {
            throw std::runtime_error("scan contains no finite points: " + std::to_string(id));
        }
        report << "{" << "id" << id << "internal_map_id" << mapId << "stamp" << stamp
               << "scan_format" << scan.formatName() << "offset" << offset
               << "count" << count << "discarded_nonfinite" << discarded;
        writeTransform(report, "map_from_base", pose);
        writeTransform(report, "base_from_sensor", extrinsic);
        report << "}";
        offset += count;
    }
    report << "]" << "point_count" << offset;
    cloud.close();
    scans.close();
}

// Save the occupancy grid from this exact snapshot, keeping its actual origin.
void exportGrid(rtabmap::DBDriver &db, const fs::path &output, cv::FileStorage &report) {
    float x = 0, y = 0, resolution = 0;
    cv::Mat grid = db.load2DMap(x, y, resolution);
    if (grid.empty() || grid.type() != CV_8SC1 || !std::isfinite(x) ||
        !std::isfinite(y) || !std::isfinite(resolution) || resolution <= 0) {
        throw std::runtime_error("snapshot has no valid saved occupancy grid");
    }
    std::ofstream file(output / "grid.bin", std::ios::binary);
    file.exceptions(std::ios::badbit | std::ios::failbit);
    for (int row = 0; row < grid.rows; ++row) {
        for (int col = 0; col < grid.cols; ++col) {
            const int cell = grid.at<signed char>(row, col);
            if (cell != -1 && cell != 0 && cell != 100) {
                throw std::runtime_error("unsupported occupancy value");
            }
        }
        file.write(grid.ptr<char>(row), grid.cols);
    }
    file.close();
    report << "grid" << "{" << "width" << grid.cols << "height" << grid.rows
           << "origin_x" << x << "origin_y" << y << "resolution" << resolution
           << "source" << "snapshot_saved_2d_map" << "}";
}

// Export only the unambiguous connected optimized graph; raw poses are unused.
int main(int argc, char **argv) {
    try {
        if (argc != 4) {
            throw std::runtime_error("usage: relocalization_export SNAPSHOT.db OUTPUT_DIR ANCHOR_NODE_OR_0");
        }
        const std::uint16_t endian = 1;
        if (*reinterpret_cast<const unsigned char *>(&endian) != 1) {
            throw std::runtime_error("only little-endian export is supported");
        }
        const fs::path output = argv[2];
        fs::create_directories(output);
        if (!fs::is_empty(output)) {
            throw std::runtime_error("output directory must be empty");
        }
        std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create());
        if (!db->openConnection(argv[1], false, true)) {
            throw std::runtime_error("cannot open snapshot read-only");
        }
        if (db->getDatabaseVersion() != "0.23.7") {
            throw std::runtime_error("only RTAB-Map database 0.23.7 is validated");
        }
        std::set<int> ids, optimizedIds;
        db->getAllNodeIds(ids);
        const auto poses = db->loadOptimizedPoses();
        for (const auto &entry : poses) {
            if (entry.first > 0 && ids.count(entry.first)) {
                validate(entry.second);
                optimizedIds.insert(entry.first);
            }
        }
        if (optimizedIds.empty()) {
            throw std::runtime_error("no saved optimized poses; raw odometry is not a substitute");
        }
        std::multimap<int, rtabmap::Link> links;
        db->getAllLinks(links);
        // Intermediate nodes need not have saved optimized poses. They still
        // establish graph connectivity, but never contribute scan coordinates.
        const auto graphGroups = components(ids, links);
        std::vector<std::set<int>> groups;
        for (const auto &graphGroup : graphGroups) {
            std::set<int> group;
            std::set_intersection(graphGroup.begin(), graphGroup.end(),
                                  optimizedIds.begin(), optimizedIds.end(),
                                  std::inserter(group, group.end()));
            if (!group.empty()) { groups.push_back(std::move(group)); }
        }
        const int anchor = std::stoi(argv[3]);
        std::set<int> selected;
        if (anchor == 0 && groups.size() == 1) {
            selected = groups.front();
        } else if (anchor > 0) {
            for (const auto &group : groups) {
                if (group.count(anchor)) {
                    selected = group;
                }
            }
        }
        // Always leave a coverage report for disconnected or missing-anchor
        // failures. A report alone is never a usable localization index.
        cv::FileStorage coverage((output / "coverage.json").string(), cv::FileStorage::WRITE |
                                 cv::FileStorage::FORMAT_JSON);
        if (!coverage.isOpened()) { throw std::runtime_error("cannot write coverage report"); }
        coverage << "total_nodes" << static_cast<int>(ids.size())
                 << "optimized_nodes" << static_cast<int>(optimizedIds.size())
                 << "optimized_components" << "[";
        for (const auto &group : groups) {
            coverage << "[";
            for (int id : group) { coverage << id; }
            coverage << "]";
        }
        coverage << "]" << "nodes_without_optimized_pose" << "[";
        for (int id : ids) {
            if (!optimizedIds.count(id)) { coverage << id; }
        }
        coverage << "]";
        coverage.release();
        if (selected.empty()) {
            std::string detail;
            for (const auto &group : groups) {
                detail += " [first=" + std::to_string(*group.begin()) +
                          ", count=" + std::to_string(group.size()) + "]";
            }
            throw std::runtime_error("select a verified anchor node; optimized components:" + detail);
        }
        cv::FileStorage report((output / "export.json").string(), cv::FileStorage::WRITE |
                               cv::FileStorage::FORMAT_JSON);
        if (!report.isOpened()) {
            throw std::runtime_error("cannot create export report");
        }
        report << "format_version" << 1 << "database_version" << db->getDatabaseVersion()
               << "total_nodes" << static_cast<int>(ids.size()) << "graph_components" << "[";
        for (const auto &group : graphGroups) {
            report << "[";
            for (int id : group) { report << id; }
            report << "]";
        }
        report << "]" << "optimized_components" << "[";
        for (const auto &group : groups) {
            report << "[";
            for (int id : group) { report << id; }
            report << "]";
        }
        report << "]" << "excluded_ids" << "[";
        for (int id : ids) {
            if (!selected.count(id)) { report << id; }
        }
        report << "]";
        exportGrid(*db, output, report);
        exportScans(*db, selected, poses, output, report);
        report.release();
        db->closeConnection(false);
        std::cout << "exported " << selected.size() << " optimized nodes; excluded "
                  << ids.size() - selected.size() << " nodes\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "snapshot export failed: " << error.what() << '\n';
        return 1;
    }
}
