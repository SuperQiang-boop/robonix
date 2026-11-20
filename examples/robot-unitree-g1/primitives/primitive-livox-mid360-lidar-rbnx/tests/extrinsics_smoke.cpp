// Exercise the driver's packet callback without a sensor or ROS graph.
#include <bits/stdc++.h>
#include "comm/comm.h"
#define private public
#include "comm/pub_handler.h"
#undef private
using namespace livox_ros;

// Check the configured rotation for points and for IMU arriving before points.
int main() {
  LidarExtParameter config{};
  config.lidar_type = LidarProtoType::kLivoxLidarType;
  config.handle = 1;
  config.param.roll = 180;
  PubHandler handler;
  handler.AddLidarsExtParam(config);
  int count = 0;
  handler.SetImuDataCallback([&](ImuData* imu, void*) {
    ++count;
    const float sign = imu->handle == 1 ? -1 : 1;
    assert(std::abs(imu->gyro_x - 1) < 1e-5);
    assert(std::abs(imu->gyro_y - sign * 2) < 1e-5);
    assert(std::abs(imu->gyro_z - sign * 3) < 1e-5);
    assert(std::abs(imu->acc_y - sign * 5) < 1e-5);
    assert(std::abs(imu->acc_z - sign * 6) < 1e-5);
  }, nullptr);
  std::vector<uint8_t> storage(sizeof(LivoxLidarEthernetPacket) + sizeof(RawImuPoint));
  auto* packet = reinterpret_cast<LivoxLidarEthernetPacket*>(storage.data());
  packet->data_type = kLivoxLidarImuData;
  RawImuPoint raw{1, 2, 3, 4, 5, 6};
  std::memcpy(packet->data, &raw, sizeof(raw));
  PubHandler::OnLivoxLidarPointCloudCallback(1, kLivoxLidarTypeMid360, packet, &handler);
  config.handle = 2;
  config.param.roll = 0;
  handler.AddLidarsExtParam(config);
  PubHandler::OnLivoxLidarPointCloudCallback(2, kLivoxLidarTypeMid360, packet, &handler);
  PubHandler::OnLivoxLidarPointCloudCallback(1, kLivoxLidarTypeMid360, packet, &handler);
  assert(count == 3);
  config.param.roll = 180;
  LidarPubHandler points;
  points.SetLidarsExtParam(config);
  LivoxLidarCartesianHighRawPoint xyz{};
  xyz.x = 1000; xyz.y = 2000; xyz.z = 3000;
  RawPacket cloud{};
  cloud.lidar_type = LidarProtoType::kLivoxLidarType;
  cloud.data_type = kLivoxLidarCartesianCoordinateHighData;
  cloud.point_num = 1; cloud.line_num = 4;
  auto* bytes = reinterpret_cast<uint8_t*>(&xyz);
  cloud.raw_data.assign(bytes, bytes + sizeof(xyz));
  points.PointCloudProcess(cloud);
  std::vector<PointXyzlt> result;
  points.GetLidarPointClouds(result);
  assert(result.size() == 1);
  assert(std::abs(result[0].x - 1) < 1e-5);
  assert(std::abs(result[0].y + 2) < 1e-5);
  assert(std::abs(result[0].z + 3) < 1e-5);
  std::cout << "Point cloud and IMU extrinsics passed\n";
}
