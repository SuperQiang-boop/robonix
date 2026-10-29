// SPDX-License-Identifier: MulanPSL-2.0
// Write a complete checkpoint without touching the engine's live SQLite file.
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rtabmap_msgs/srv/get_map.hpp>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/VisualWord.h>
#include <rtabmap/core/global_map/OccupancyGrid.h>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <set>
#include <stdexcept>
#include <vector>

namespace fs = std::filesystem;
using rtabmap::Transform;

// Use the node's serialized map callback, which includes signatures still in RAM.
template<class Service>
typename Service::Response::SharedPtr request(
    const rclcpp::Node::SharedPtr &node, const std::string &name,
    const typename Service::Request::SharedPtr &message, double timeout) {
    auto client = node->create_client<Service>(name);
    const auto duration = std::chrono::duration<double>(timeout);
    if (!client->wait_for_service(duration)) {
        throw std::runtime_error("service unavailable: " + name);
    }
    auto future = client->async_send_request(message);
    if (rclcpp::spin_until_future_complete(node, future, duration) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("service timed out: " + name);
    }
    return future.get();
}

// Preserve the engine's grid settings when assembling this exact optimized graph.
rtabmap::ParametersMap gridParameters(const rclcpp::Node::SharedPtr &node,
                                    const std::string &ns, double timeout) {
    auto message = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    for (const auto &entry : rtabmap::Parameters::getDefaultParameters()) {
        if (entry.first == "Grid/CellSize" || entry.first.find("GridGlobal/") == 0) {
            message->names.push_back(entry.first);
        }
    }
    auto response = request<rcl_interfaces::srv::GetParameters>(
        node, ns + "/get_parameters", message, timeout);
    if (response->values.size() != message->names.size()) {
        throw std::runtime_error("incomplete grid parameters");
    }
    rtabmap::ParametersMap parameters;
    for (size_t i = 0; i < response->values.size(); ++i) {
        const auto &value = response->values[i];
        using Type = rcl_interfaces::msg::ParameterType;
        switch (value.type) {
            case Type::PARAMETER_STRING: parameters[message->names[i]] = value.string_value; break;
            case Type::PARAMETER_DOUBLE: parameters[message->names[i]] = std::to_string(value.double_value); break;
            case Type::PARAMETER_INTEGER: parameters[message->names[i]] = std::to_string(value.integer_value); break;
            case Type::PARAMETER_BOOL: parameters[message->names[i]] = value.bool_value ? "true" : "false"; break;
            default: throw std::runtime_error("grid parameter unavailable: " + message->names[i]);
        }
    }
    return parameters;
}

// Reject missing/nonrigid poses instead of manufacturing poses from raw odometry.
void validate(const Transform &pose) {
    if (pose.isNull() || !cv::checkRange(pose.dataMatrix())) {
        throw std::runtime_error("missing or nonfinite transform");
    }
    const auto rotation = pose.rotationMatrix();
    if (cv::norm(rotation * rotation.t() - cv::Mat::eye(3, 3, CV_32F)) > 1e-3 ||
        std::abs(cv::determinant(rotation) - 1.0) > 1e-3) {
        throw std::runtime_error("nonrigid transform");
    }
}

// Only physical constraints join saved node coordinates into a single map.
bool physical(const rtabmap::Link &link) {
    return link.type() == rtabmap::Link::kNeighbor ||
        link.type() == rtabmap::Link::kNeighborMerged ||
        link.type() == rtabmap::Link::kGlobalClosure ||
        link.type() == rtabmap::Link::kLocalSpaceClosure ||
        link.type() == rtabmap::Link::kLocalTimeClosure ||
        link.type() == rtabmap::Link::kUserClosure;
}

// Refuse disconnected/missing coverage rather than silently exporting one segment.
void validateGraph(const std::map<int, Transform> &poses,
                   const std::multimap<int, rtabmap::Link> &links,
                   const std::map<int, rtabmap::Signature> &signatures) {
    std::set<int> ids;
    for (const auto &entry : signatures) {
        if (entry.first > 0 && entry.second.getWeight() >= -1) {
            if (!poses.count(entry.first)) {
                throw std::runtime_error("valid node missing from optimized graph: " + std::to_string(entry.first));
            }
            ids.insert(entry.first);
        }
    }
    if (ids.size() < 2) { throw std::runtime_error("map not ready: fewer than two valid nodes"); }
    for (const auto &entry : poses) {
        validate(entry.second);
        if (entry.first > 0 && !ids.count(entry.first)) {
            throw std::runtime_error("optimized node has no valid sensor signature");
        }
    }
    std::map<int, std::set<int>> edges;
    for (const auto &entry : links) {
        const auto &link = entry.second;
        if (physical(link) && ids.count(link.from()) && ids.count(link.to())) {
            validate(link.transform());
            edges[link.from()].insert(link.to());
            edges[link.to()].insert(link.from());
        }
    }
    std::queue<int> pending;
    pending.push(*ids.begin());
    while (!pending.empty()) {
        int id = pending.front();
        pending.pop();
        if (!ids.erase(id)) { continue; }
        for (int neighbor : edges[id]) { pending.push(neighbor); }
    }
    if (!ids.empty()) { throw std::runtime_error("optimized graph has disconnected components"); }
}

// Render the saved grid itself, so previews cannot come from a newer ROS sample.
void writePreview(const fs::path &output, const cv::Mat &grid, float x, float y,
                  float resolution, const std::string &frame) {
    cv::Mat image(grid.size(), CV_8UC1);
    for (int row = 0; row < grid.rows; ++row) {
        for (int col = 0; col < grid.cols; ++col) {
            const int value = grid.at<signed char>(row, col);
            image.at<unsigned char>(grid.rows - 1 - row, col) =
                value < 0 ? 205 : (value >= 50 ? 0 : 254);
        }
    }
    if (!cv::imwrite((output / "occupancy.pgm").string(), image) ||
        !cv::imwrite((output / "occupancy.png").string(), image)) {
        throw std::runtime_error("cannot write occupancy preview");
    }
    std::ofstream yaml(output / "occupancy.yaml");
    yaml.exceptions(std::ios::badbit | std::ios::failbit);
    yaml << "image: occupancy.pgm\nresolution: " << resolution
         << "\norigin: [" << x << ", " << y << ", 0.0]\n"
         << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
    yaml.close();
    std::ofstream meta(output / "meta.yaml");
    meta.exceptions(std::ios::badbit | std::ios::failbit);
    meta << "spatial_artifact: rtabmap.db\nframe_id: " << frame
         << "\nresolution: " << resolution << "\nwidth: " << grid.cols
         << "\nheight: " << grid.rows << "\norigin: [" << x << ", " << y << ", 0.0]\n";
    meta.close();
}

// Keep the optional fused cloud in exactly the saved optimized coordinate frame.
void writeCloud(const fs::path &output, const std::vector<std::array<float, 3>> &points) {
    std::ofstream file(output / "cloud.pcd", std::ios::binary);
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
         << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
         << points.size() << "\nDATA binary\n";
    for (const auto &point : points) {
        file.write(reinterpret_cast<const char *>(point.data()), sizeof(float) * 3);
    }
    file.close();
}

// Rebuild an independent database with official codecs, optimized poses and grid.
void save(const rtabmap_msgs::msg::MapData &message,
          const rtabmap::ParametersMap &parameters, const fs::path &output) {
    std::map<int, Transform> poses;
    std::multimap<int, rtabmap::Link> links;
    std::map<int, rtabmap::Signature> signatures;
    Transform correction;
    rtabmap_conversions::mapDataFromROS(message, poses, links, signatures, correction);
    validateGraph(poses, links, signatures);
    if (fs::exists(output / "rtabmap.db")) { throw std::runtime_error("output database already exists"); }
    rtabmap::LocalGridCache cache;
    std::map<int, cv::Mat> words;
    std::vector<std::array<float, 3>> cloud;
    size_t scanPoints = 0, nodes = 0;
    std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create());
    if (!db->openConnection((output / "rtabmap.db").string())) {
        throw std::runtime_error("cannot create checkpoint database");
    }
    for (auto &entry : signatures) {
        if (entry.first <= 0 || entry.second.getWeight() < -1) { continue; }
        auto signature = std::make_unique<rtabmap::Signature>(entry.second);
        signature->setSaved(false);
        signature->removeLinks();
        // MapData carries constraints separately; persist both directions.
        for (const auto &edge : links) {
            auto link = edge.second;
            if (link.from() != entry.first && link.to() == entry.first) { link = link.inverse(); }
            if (link.from() != entry.first || link.type() == rtabmap::Link::kVirtualClosure) { continue; }
            if (link.type() == rtabmap::Link::kLandmark) {
                signature->addLandmark(link);
            } else if ((poses.count(link.to()) || link.to() == entry.first) &&
                       !signature->hasLink(link.to(), link.type())) {
                signature->addLink(link);
            }
        }
        cv::Mat ground, obstacles, empty;
        rtabmap::LaserScan scan;
        signature->sensorData().uncompressData(nullptr, nullptr, &scan, nullptr, &ground, &obstacles, &empty);
        if (scan.empty() || scan.isCompressed() || scan.format() == rtabmap::LaserScan::kUnknown ||
            scan.data().depth() != CV_32F || scan.channels() < (scan.is2d() ? 2 : 3)) {
            throw std::runtime_error("missing decoded laser scan: " + std::to_string(entry.first));
        }
        validate(scan.localTransform());
        const auto mapFromSensor = poses.at(entry.first) * scan.localTransform();
        size_t finitePoints = 0;
        for (int row = 0; row < scan.data().rows; ++row) {
            for (int col = 0; col < scan.data().cols; ++col) {
                const float *point = scan.data().ptr<float>(row) + col * scan.channels();
                if (std::isfinite(point[0]) && std::isfinite(point[1]) &&
                    (scan.is2d() || std::isfinite(point[2]))) {
                    const float z = scan.is2d() ? 0.0f : point[2];
                    std::array<float, 3> global{
                        mapFromSensor.r11()*point[0] + mapFromSensor.r12()*point[1] + mapFromSensor.r13()*z + mapFromSensor.x(),
                        mapFromSensor.r21()*point[0] + mapFromSensor.r22()*point[1] + mapFromSensor.r23()*z + mapFromSensor.y(),
                        mapFromSensor.r31()*point[0] + mapFromSensor.r32()*point[1] + mapFromSensor.r33()*z + mapFromSensor.z()};
                    if (!std::isfinite(global[0]) || !std::isfinite(global[1]) || !std::isfinite(global[2])) {
                        throw std::runtime_error("nonfinite transformed laser point");
                    }
                    cloud.push_back(global);
                    ++finitePoints;
                }
            }
        }
        if (!finitePoints) { throw std::runtime_error("laser scan has no finite points"); }
        scanPoints += finitePoints;
        const auto &data = signature->sensorData();
        if (data.gridCellSize() <= 0 || (ground.empty() && obstacles.empty() && empty.empty())) {
            throw std::runtime_error("missing local occupancy grid: " + std::to_string(entry.first));
        }
        cache.add(entry.first, ground, obstacles, empty, data.gridCellSize(), data.gridViewPoint());
        // Preserve visual references too when RGB/depth is enabled by deployment.
        const auto &descriptors = signature->getWordsDescriptors();
        for (const auto &word : signature->getWords()) {
            if (word.first <= 0 || words.count(word.first)) { continue; }
            if (word.second < 0 || word.second >= descriptors.rows) {
                throw std::runtime_error("missing visual word descriptor");
            }
            words.emplace(word.first, descriptors.row(word.second).clone());
        }
        db->asyncSave(signature.release());
        ++nodes;
    }
    for (const auto &word : words) { db->asyncSave(new rtabmap::VisualWord(word.first, word.second)); }
    db->emptyTrashes(false);
    rtabmap::OccupancyGrid occupancy(&cache, parameters);
    occupancy.update(poses);
    float x = 0, y = 0;
    const auto grid = occupancy.getMap(x, y);
    if (grid.empty() || grid.type() != CV_8SC1) { throw std::runtime_error("empty occupancy grid"); }
    // Store a real optimized last pose, never the map/odom correction as a pose.
    db->saveOptimizedPoses(poses, poses.rbegin()->second);
    db->save2DMap(grid, x, y, occupancy.getCellSize());
    db->closeConnection(true);
    writePreview(output, grid, x, y, occupancy.getCellSize(), message.header.frame_id);
    writeCloud(output, cloud);
    cv::FileStorage report((output / "checkpoint.json").string(), cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    report << "source" << "rtabmap_get_map_data_optimized" << "nodes" << static_cast<int>(nodes)
           << "physical_components" << 1 << "scan_points" << static_cast<double>(scanPoints)
           << "visual_words" << static_cast<int>(words.size()) << "stamp_sec" << message.header.stamp.sec;
    report.release();
    std::cout << "saved optimized graph: nodes=" << nodes << " components=1 scan_points=" << scanPoints << '\n';
}

// Bound service waits and leave incomplete output in caller-owned staging only.
int main(int argc, char **argv) {
    try {
        if (argc != 4) { throw std::runtime_error("usage: mapping_snapshot OUTPUT_DIR RTABMAP_NAMESPACE TIMEOUT_SECONDS"); }
        const double timeout = std::stod(argv[3]);
        if (!std::isfinite(timeout) || timeout <= 0) { throw std::runtime_error("invalid snapshot timeout"); }
        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("mapping_snapshot");
        const auto parameters = gridParameters(node, argv[2], timeout);
        auto message = std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
        message->global_map = true;
        message->optimized = true;
        message->graph_only = false;
        auto response = request<rtabmap_msgs::srv::GetMap>(node, std::string(argv[2]) + "/get_map_data", message, timeout);
        save(response->data, parameters, argv[1]);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "map checkpoint failed: " << error.what() << '\n';
        if (rclcpp::ok()) { rclcpp::shutdown(); }
        return 1;
    }
}
