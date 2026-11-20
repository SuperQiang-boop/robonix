// Deterministic RTAB-Map database fixture, written through the official API.
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Signature.h>
#include <filesystem>
#include <iostream>
#include <memory>

// Produce points whose sensor/base/map coordinates are independently checkable.
rtabmap::SensorData scanData(int id, bool missing) {
    rtabmap::SensorData data;
    if (!missing) {
        cv::Mat points = (cv::Mat_<cv::Vec3f>(1, 3) << cv::Vec3f(1, 2, 3),
                          cv::Vec3f(2, 3, 4), cv::Vec3f(3, 4, 5));
        const rtabmap::Transform extrinsic(0.5f, 0.25f, 0.75f, 0, 0, 0);
        data.setLaserScan(rtabmap::LaserScan(rtabmap::compressData2(points),
            3, 20, rtabmap::LaserScan::kXYZ, extrinsic));
    }
    data.setId(id);
    return data;
}

// Save connected/disconnected and incomplete graphs without manipulating BLOBs.
int main(int argc, char **argv) {
    if (argc != 3 || std::filesystem::exists(argv[1])) { return 2; }
    const std::string mode = argv[2];
    std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create());
    if (!db->openConnection(argv[1])) { return 3; }
    const rtabmap::Transform raw(100.0f, 100.0f, 0.0f);
    for (int id = 1; id <= 3; ++id) {
        auto *node = new rtabmap::Signature(id, id, 0, id, "", raw,
            rtabmap::Transform(), scanData(id, mode == "missing_scan" && id == 2));
        if (id <= 2) {
            node->addLink(rtabmap::Link(id, 3-id, rtabmap::Link::kNeighbor,
                rtabmap::Transform(id == 1 ? 1.0f : -1.0f, 0.0f, 0.0f)));
        }
        db->asyncSave(node);
    }
    db->emptyTrashes(false);
    if (mode != "missing_poses") {
        std::map<int, rtabmap::Transform> optimized{
            {1, rtabmap::Transform(10.0f, 20.0f, 0.0f)},
            {2, rtabmap::Transform(11.0f, 20.0f, 0.0f)}};
        if (mode == "disconnected") { optimized[3] = raw; }
        db->saveOptimizedPoses(optimized, rtabmap::Transform::getIdentity());
    }
    if (mode != "missing_grid") {
        cv::Mat grid(8, 9, CV_8SC1, cv::Scalar(0));
        grid.at<signed char>(1, 2) = 100;
        grid.at<signed char>(5, 6) = 100;
        grid.at<signed char>(0, 0) = -1;
        db->save2DMap(grid, -2.5f, -1.25f, 0.05f);
    }
    db->closeConnection(true);
    std::cout << "created " << mode << " fixture\n";
    return 0;
}
