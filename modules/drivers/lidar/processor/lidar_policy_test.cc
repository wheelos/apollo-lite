// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "Eigen/Geometry"

#include "cyber/time/time.h"
#include "modules/drivers/lidar/processor/control/time_contract.h"
#include "modules/drivers/lidar/processor/lidar_unified_component.h"
#include "modules/drivers/lidar/processor/policy/cpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/gpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

constexpr uint64_t kTestSecondToNano = 1000000000ULL;

class MockBuffer : public apollo::transform::BufferInterface {
 public:
  using Key = std::tuple<std::string, std::string, uint64_t>;

  void AddTransform(const std::string& target_frame,
                    const std::string& source_frame, const cyber::Time& time,
                    const Eigen::Affine3d& transform) {
    transforms_[Key(target_frame, source_frame, time.ToNanosecond())] =
        ToStamped(target_frame, source_frame, time, transform);
  }

  apollo::transform::TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const cyber::Time& time, const float timeout_second) const override {
    (void)timeout_second;
    return Get(target_frame, source_frame, time);
  }

  apollo::transform::TransformStamped lookupTransform(
      const std::string& target_frame, const cyber::Time& target_time,
      const std::string& source_frame, const cyber::Time& source_time,
      const std::string& fixed_frame,
      const float timeout_second) const override {
    (void)target_time;
    (void)fixed_frame;
    (void)timeout_second;
    return Get(target_frame, source_frame, source_time);
  }

  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame, const cyber::Time& time,
                    const float timeout_second,
                    std::string* errstr) const override {
    (void)timeout_second;
    return Exists(target_frame, source_frame, time, errstr);
  }

  bool canTransform(const std::string& target_frame,
                    const cyber::Time& target_time,
                    const std::string& source_frame,
                    const cyber::Time& source_time,
                    const std::string& fixed_frame, const float timeout_second,
                    std::string* errstr) const override {
    (void)target_time;
    (void)fixed_frame;
    (void)timeout_second;
    return Exists(target_frame, source_frame, source_time, errstr);
  }

  bool GetLatestStaticTransform(
      const std::string& target_frame, const std::string& source_frame,
      apollo::transform::TransformStamped* transform) const override {
    if (transform == nullptr) {
      return false;
    }
    const auto it =
        transforms_.lower_bound(Key(target_frame, source_frame, 0U));
    if (it == transforms_.end() || std::get<0>(it->first) != target_frame ||
        std::get<1>(it->first) != source_frame) {
      return false;
    }
    *transform = it->second;
    return true;
  }

 private:
  static apollo::transform::TransformStamped ToStamped(
      const std::string& target_frame, const std::string& source_frame,
      const cyber::Time& time, const Eigen::Affine3d& transform) {
    apollo::transform::TransformStamped stamped;
    stamped.mutable_header()->set_frame_id(target_frame);
    stamped.mutable_header()->set_timestamp_sec(time.ToSecond());
    stamped.set_child_frame_id(source_frame);
    stamped.mutable_transform()->mutable_translation()->set_x(
        transform.translation().x());
    stamped.mutable_transform()->mutable_translation()->set_y(
        transform.translation().y());
    stamped.mutable_transform()->mutable_translation()->set_z(
        transform.translation().z());
    const Eigen::Quaterniond rotation(transform.linear());
    stamped.mutable_transform()->mutable_rotation()->set_qw(rotation.w());
    stamped.mutable_transform()->mutable_rotation()->set_qx(rotation.x());
    stamped.mutable_transform()->mutable_rotation()->set_qy(rotation.y());
    stamped.mutable_transform()->mutable_rotation()->set_qz(rotation.z());
    return stamped;
  }

  apollo::transform::TransformStamped Get(const std::string& target_frame,
                                          const std::string& source_frame,
                                          const cyber::Time& time) const {
    const auto it =
        transforms_.find(Key(target_frame, source_frame, time.ToNanosecond()));
    if (it == transforms_.end()) {
      throw std::runtime_error("Missing mock transform");
    }
    return it->second;
  }

  bool Exists(const std::string& target_frame, const std::string& source_frame,
              const cyber::Time& time, std::string* errstr) const {
    const bool found = transforms_.count(Key(target_frame, source_frame,
                                             time.ToNanosecond())) > 0;
    if (!found && errstr != nullptr) {
      *errstr = "mock transform not found";
    }
    return found;
  }

  std::map<Key, apollo::transform::TransformStamped> transforms_;
};

std::shared_ptr<PointCloud> MakePointCloud(
    const std::string& frame_id, double measurement_time,
    const std::vector<std::tuple<float, float, float, uint64_t>>& points) {
  auto cloud = std::make_shared<PointCloud>();
  cloud->set_frame_id(frame_id);
  cloud->set_measurement_time(measurement_time);
  cloud->set_is_dense(true);
  for (const auto& point : points) {
    auto* out = cloud->add_point();
    out->set_x(std::get<0>(point));
    out->set_y(std::get<1>(point));
    out->set_z(std::get<2>(point));
    out->set_timestamp(std::get<3>(point));
  }
  cloud->set_width(cloud->point_size());
  cloud->set_height(1);
  return cloud;
}

TimeContract MakeTimeContract(double begin_sec, double end_sec,
                              int64_t offset_ns = 0) {
  TimeContract contract;
  contract.scan_begin_ns =
      static_cast<int64_t>(std::llround(begin_sec * kTestSecondToNano));
  contract.scan_end_ns =
      static_cast<int64_t>(std::llround(end_sec * kTestSecondToNano));
  contract.canonical_anchor_ns = contract.scan_end_ns;
  contract.static_offset_ns = offset_ns;
  contract.quality = TimestampQuality::kPointTimestamps;
  return contract;
}

std::shared_ptr<BufferedFrame> MakeBufferedFrame(
    const std::string& sensor_id, double begin_sec, double end_sec,
    uint64_t frame_id = 1) {
  auto frame = std::make_shared<BufferedFrame>();
  frame->frame_id = frame_id;
  frame->point_cloud = MakePointCloud(sensor_id, end_sec, {});
  frame->time_contract = MakeTimeContract(begin_sec, end_sec);
  frame->pose_prefetch_ok = true;
  return frame;
}

LidarUnifiedComponentConfig MakeConfig() {
  LidarUnifiedComponentConfig config;
  config.set_map_frame_id("map");
  config.set_base_link_frame_id("base_link");
  config.set_motion_compensation_bins(3);
  config.set_voxel_size(1.0f);
  config.set_enable_voxel_filter(true);
  config.set_enable_ego_query_filter(true);
  config.set_ego_box_forward_x(0.5f);
  config.set_ego_box_backward_x(-0.5f);
  config.set_ego_box_forward_y(0.5f);
  config.set_ego_box_backward_y(-0.5f);
  return config;
}

}  // namespace

TEST(TimeContractTest, NormalizesScanEndFallback) {
  LidarUnifiedComponentConfig::TimeSettings settings;
  settings.set_measurement_time_anchor(LidarUnifiedComponentConfig::SCAN_END);
  settings.set_expected_scan_duration_ms(100.0);
  settings.set_max_scan_duration_ms(200.0);

  TimeContract contract;
  ASSERT_TRUE(NormalizePointCloudTime(
      *MakePointCloud("lidar", 10.0, {}), settings, &contract));
  EXPECT_EQ(contract.scan_begin_ns, 9900000000LL);
  EXPECT_EQ(contract.scan_end_ns, 10000000000LL);
  EXPECT_EQ(contract.canonical_anchor_ns, 10000000000LL);
  EXPECT_EQ(contract.quality, TimestampQuality::kMeasurementTimeFallback);
}

TEST(TimeContractTest, NormalizesScanBeginFallback) {
  LidarUnifiedComponentConfig::TimeSettings settings;
  settings.set_measurement_time_anchor(
      LidarUnifiedComponentConfig::SCAN_BEGIN);
  settings.set_expected_scan_duration_ms(80.0);
  settings.set_max_scan_duration_ms(100.0);
  settings.set_static_time_offset_ns(-1000000);

  TimeContract contract;
  ASSERT_TRUE(NormalizePointCloudTime(
      *MakePointCloud("lidar", 10.0, {}), settings, &contract));
  EXPECT_EQ(contract.scan_begin_ns, 9999000000LL);
  EXPECT_EQ(contract.scan_end_ns, 10079000000LL);
  EXPECT_EQ(contract.canonical_anchor_ns, 10079000000LL);
}

TEST(TimeContractTest, PointTimestampsOverrideMeasurementAnchor) {
  LidarUnifiedComponentConfig::TimeSettings settings;
  settings.set_measurement_time_anchor(
      LidarUnifiedComponentConfig::SCAN_BEGIN);
  settings.set_expected_scan_duration_ms(100.0);
  settings.set_max_scan_duration_ms(200.0);

  TimeContract contract;
  ASSERT_TRUE(NormalizePointCloudTime(
      *MakePointCloud("lidar", 9.99,
                      {{0.0f, 0.0f, 0.0f, 9990000000ULL},
                       {0.0f, 0.0f, 0.0f, 10000000000ULL}}),
      settings, &contract));
  EXPECT_EQ(contract.scan_begin_ns, 9990000000LL);
  EXPECT_EQ(contract.scan_end_ns, 10000000000LL);
  EXPECT_EQ(contract.quality, TimestampQuality::kPointTimestamps);
  EXPECT_TRUE(contract.all_points_have_timestamps);
}

TEST(TimeContractTest, FallsBackFromImplausiblePointTimestamps) {
  LidarUnifiedComponentConfig::TimeSettings settings;
  settings.set_expected_scan_duration_ms(100.0);
  settings.set_max_scan_duration_ms(200.0);

  TimeContract contract;
  ASSERT_TRUE(NormalizePointCloudTime(
      *MakePointCloud("lidar", 10.0,
                      {{0.0f, 0.0f, 0.0f, 9000000000ULL},
                       {0.0f, 0.0f, 0.0f, 10000000000ULL}}),
      settings, &contract));
  EXPECT_EQ(contract.scan_begin_ns, 9900000000LL);
  EXPECT_EQ(contract.scan_end_ns, 10000000000LL);
  EXPECT_EQ(contract.quality, TimestampQuality::kMeasurementTimeFallback);
  EXPECT_TRUE(contract.all_points_have_timestamps);
}

TEST(TimeContractTest, FallsBackFromSecondUnitPointTimestamps) {
  LidarUnifiedComponentConfig::TimeSettings settings;
  settings.set_measurement_time_anchor(
      LidarUnifiedComponentConfig::SCAN_BEGIN);
  settings.set_expected_scan_duration_ms(100.0);
  settings.set_max_scan_duration_ms(200.0);

  TimeContract contract;
  ASSERT_TRUE(NormalizePointCloudTime(
      *MakePointCloud("seyond", 1700000000.0,
                      {{0.0f, 0.0f, 0.0f, 1700000000ULL}}),
      settings, &contract));
  EXPECT_EQ(contract.scan_begin_ns, 1700000000000000000LL);
  EXPECT_EQ(contract.scan_end_ns, 1700000000100000000LL);
  EXPECT_EQ(contract.quality, TimestampQuality::kMeasurementTimeFallback);
  EXPECT_TRUE(contract.all_points_have_timestamps);
}

TEST(LidarPolicyFactoryTest, CreatesPoliciesForKnownModes) {
#ifdef APOLLO_LIDAR_POLICY_FORCE_CPU
  EXPECT_NE(LidarPolicyFactory::CreateDeskewPolicy("cpu"), nullptr);
  EXPECT_EQ(LidarPolicyFactory::CreateDeskewPolicy("gpu"), nullptr);
  EXPECT_NE(LidarPolicyFactory::CreateFusionPolicy("cpu"), nullptr);
  EXPECT_EQ(LidarPolicyFactory::CreateFusionPolicy("gpu"), nullptr);
  EXPECT_NE(LidarPolicyFactory::CreateFilterPolicy("cpu"), nullptr);
  EXPECT_EQ(LidarPolicyFactory::CreateFilterPolicy("gpu"), nullptr);
#elif defined(APOLLO_LIDAR_POLICY_FORCE_GPU)
  EXPECT_EQ(LidarPolicyFactory::CreateDeskewPolicy("cpu"), nullptr);
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  EXPECT_NE(LidarPolicyFactory::CreateDeskewPolicy("gpu"), nullptr);
#else
  EXPECT_EQ(LidarPolicyFactory::CreateDeskewPolicy("gpu"), nullptr);
#endif
  EXPECT_EQ(LidarPolicyFactory::CreateFusionPolicy("cpu"), nullptr);
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  EXPECT_NE(LidarPolicyFactory::CreateFusionPolicy("gpu"), nullptr);
#else
  EXPECT_EQ(LidarPolicyFactory::CreateFusionPolicy("gpu"), nullptr);
#endif
  EXPECT_EQ(LidarPolicyFactory::CreateFilterPolicy("cpu"), nullptr);
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  EXPECT_NE(LidarPolicyFactory::CreateFilterPolicy("gpu"), nullptr);
#else
  EXPECT_EQ(LidarPolicyFactory::CreateFilterPolicy("gpu"), nullptr);
#endif
#else
  EXPECT_NE(LidarPolicyFactory::CreateDeskewPolicy("cpu"), nullptr);
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  EXPECT_NE(LidarPolicyFactory::CreateDeskewPolicy("gpu"), nullptr);
#else
  EXPECT_EQ(LidarPolicyFactory::CreateDeskewPolicy("gpu"), nullptr);
#endif
  EXPECT_NE(LidarPolicyFactory::CreateFusionPolicy("cpu"), nullptr);
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  EXPECT_NE(LidarPolicyFactory::CreateFusionPolicy("gpu"), nullptr);
#else
  EXPECT_EQ(LidarPolicyFactory::CreateFusionPolicy("gpu"), nullptr);
#endif
  EXPECT_NE(LidarPolicyFactory::CreateFilterPolicy("cpu"), nullptr);
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  EXPECT_NE(LidarPolicyFactory::CreateFilterPolicy("gpu"), nullptr);
#else
  EXPECT_EQ(LidarPolicyFactory::CreateFilterPolicy("gpu"), nullptr);
#endif
  EXPECT_EQ(LidarPolicyFactory::CreateFilterPolicy("unknown"), nullptr);
#endif
}

TEST(CpuLidarDeskewPolicyTest, ComputesSampledPosesFromPointTimestamps) {
  MockBuffer tf_buffer;
  tf_buffer.AddTransform(
    "map", "lidar", cyber::Time(10.0),
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  tf_buffer.AddTransform(
    "map", "lidar", cyber::Time(10.5),
      Eigen::Translation3d(0.5, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  tf_buffer.AddTransform(
    "map", "lidar", cyber::Time(11.0),
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  CpuLidarDeskewPolicy policy;
  ASSERT_TRUE(policy.Init(MakeConfig(), &tf_buffer));

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.point_cloud =
      MakePointCloud("lidar", 11.0,
                     {{0.0f, 0.0f, 0.0f, 10 * kTestSecondToNano},
                      {0.0f, 0.0f, 0.0f, 11 * kTestSecondToNano}});

  std::vector<double> sample_times;
  std::vector<Eigen::Affine3d> poses;
  ASSERT_TRUE(policy.ComputeMotionCompensationPoses(frame_context,
                                                    &sample_times, &poses));
  ASSERT_EQ(sample_times.size(), 3U);
  ASSERT_EQ(poses.size(), 3U);
  EXPECT_DOUBLE_EQ(sample_times[0], 10.0);
  EXPECT_DOUBLE_EQ(sample_times[1], 10.5);
  EXPECT_DOUBLE_EQ(sample_times[2], 11.0);
  EXPECT_DOUBLE_EQ(poses[0].translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(poses[1].translation().x(), 0.5);
  EXPECT_DOUBLE_EQ(poses[2].translation().x(), 1.0);
}

TEST(CpuLidarDeskewPolicyTest,
     FallsBackToMeasurementTimeForInvalidPointTimestamps) {
  MockBuffer tf_buffer;
  tf_buffer.AddTransform(
      "map", "lidar", cyber::Time(12.0),
      Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  CpuLidarDeskewPolicy policy;
  ASSERT_TRUE(policy.Init(MakeConfig(), &tf_buffer));

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.point_cloud = MakePointCloud(
      "lidar", 12.0,
      {{0.0f, 0.0f, 0.0f, 2085983134000164270ULL},
       {0.0f, 0.0f, 0.0f, 2085983134000263691ULL}});

  std::vector<double> sample_times;
  std::vector<Eigen::Affine3d> poses;
  ASSERT_TRUE(policy.ComputeMotionCompensationPoses(frame_context,
                                                    &sample_times, &poses));
  ASSERT_EQ(sample_times.size(), 3U);
  ASSERT_EQ(poses.size(), 3U);
  EXPECT_DOUBLE_EQ(sample_times[0], 12.0);
  EXPECT_DOUBLE_EQ(sample_times[1], 12.0);
  EXPECT_DOUBLE_EQ(sample_times[2], 12.0);
  EXPECT_DOUBLE_EQ(poses[0].translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(poses[1].translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(poses[2].translation().x(), 2.0);
}

TEST(CpuLidarFusionPolicyTest, FusesPointsIntoReferenceBaseFrame) {
  MockBuffer tf_buffer;
  CpuLidarFusionPolicy policy;
  ASSERT_TRUE(policy.Init(MakeConfig(), &tf_buffer));
  const Eigen::Affine3d map2base_ref =
      (Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::Quaterniond::Identity())
          .inverse();

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.point_cloud =
      MakePointCloud("lidar", 12.0,
                     {{1.0f, 0.0f, 0.0f, 10 * kTestSecondToNano},
                      {1.0f, 0.0f, 0.0f, 12 * kTestSecondToNano}});

  std::vector<PointXYZIT> storage(8);
  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = 0;
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;
  buffer.device_id = -1;

  const std::vector<std::vector<double>> sample_times{{10.0, 12.0}};
  const std::vector<std::vector<Eigen::Affine3d>> poses{{
      Eigen::Translation3d(5.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(),
      Eigen::Translation3d(7.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(),
  }};

  ASSERT_TRUE(policy.FuseToBaseLink(12.0, map2base_ref, {frame_context}, poses,
                                    sample_times, &buffer));
  ASSERT_EQ(buffer.valid_count, 2U);
  EXPECT_FLOAT_EQ(storage[0].x(), 4.0f);
  EXPECT_FLOAT_EQ(storage[1].x(), 6.0f);
}

TEST(CpuLidarFusionPolicyTest, InterpolatesIntermediatePoseBins) {
  MockBuffer tf_buffer;
  CpuLidarFusionPolicy policy;
  ASSERT_TRUE(policy.Init(MakeConfig(), &tf_buffer));
  const Eigen::Affine3d map2base_ref = Eigen::Affine3d::Identity();

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.point_cloud =
      MakePointCloud(
          "lidar", 11.0,
          {{0.0f, 0.0f, 0.0f, 11 * kTestSecondToNano}});

  std::vector<PointXYZIT> storage(4);
  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = 0;
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;
  buffer.device_id = -1;

  const std::vector<std::vector<double>> sample_times{{10.0, 12.0}};
  const std::vector<std::vector<Eigen::Affine3d>> poses{{
      Eigen::Translation3d(5.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(),
      Eigen::Translation3d(7.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(),
  }};

  ASSERT_TRUE(policy.FuseToBaseLink(11.0, map2base_ref, {frame_context}, poses,
                                    sample_times, &buffer));
  ASSERT_EQ(buffer.valid_count, 1U);
  EXPECT_FLOAT_EQ(storage[0].x(), 6.0f);
}

TEST(LidarPolicyCommonTest,
     UniformPoseInterpolationMatchesGenericInterpolation) {
  const std::vector<double> sample_times{10.0, 10.5, 11.0};
  const std::vector<Eigen::Affine3d> poses{
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(),
      Eigen::Translation3d(1.0, 0.5, 0.0) *
          Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()),
      Eigen::Translation3d(2.0, 1.0, 0.0) *
          Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()),
  };
  UniformPoseInterpolation interpolation;
  ASSERT_TRUE(
      BuildUniformPoseInterpolation(sample_times, poses, &interpolation));

  for (const uint64_t timestamp :
       {10000000000ULL, 10125000000ULL, 10500000000ULL, 10875000000ULL,
        11000000000ULL, 11250000000ULL}) {
    PointXYZIT point;
    point.set_x(1.0f);
    point.set_y(-2.0f);
    point.set_z(0.5f);
    point.set_intensity(3.0f);
    point.set_timestamp(timestamp);

    PointXYZIT generic;
    PointXYZIT uniform;
    ASSERT_TRUE(TransformPointWithInterpolatedPoses(
        point, 0U, 0, sample_times, poses, &generic));
    ASSERT_TRUE(TransformPointWithUniformInterpolatedPoses(
        point, 0U, 0, sample_times, poses, interpolation, &uniform));
    EXPECT_NEAR(uniform.x(), generic.x(), 1e-6);
    EXPECT_NEAR(uniform.y(), generic.y(), 1e-6);
    EXPECT_NEAR(uniform.z(), generic.z(), 1e-6);
    EXPECT_FLOAT_EQ(uniform.intensity(), generic.intensity());
    EXPECT_EQ(uniform.timestamp(), generic.timestamp());
  }
}

TEST(CpuLidarFusionPolicyTest,
     AppliesEgoFilterDuringStaticFusionWithTimestampFastPath) {
  MockBuffer tf_buffer;
  auto config = MakeConfig();
  config.set_enable_voxel_filter(false);
  CpuLidarFusionPolicy fusion_policy;
  CpuLidarFilterPolicy filter_policy;
  ASSERT_TRUE(fusion_policy.Init(config, &tf_buffer));
  ASSERT_TRUE(filter_policy.Init(config));

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.all_points_have_timestamps = true;
  frame_context.point_cloud =
      MakePointCloud("lidar", 12.0,
                     {{0.1f, 0.1f, 0.0f, 10 * kTestSecondToNano},
                      {0.6f, 0.1f, 0.0f, 12 * kTestSecondToNano}});

  std::vector<PointXYZIT> storage(2);
  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;

  const std::vector<std::vector<double>> sample_times{{12.0}};
  const std::vector<std::vector<Eigen::Affine3d>> poses{{
      Eigen::Affine3d::Identity(),
  }};
  ASSERT_TRUE(fusion_policy.FuseToBaseLink(
      12.0, Eigen::Affine3d::Identity(), {frame_context}, poses,
      sample_times, &buffer));
  EXPECT_EQ(buffer.unfiltered_valid_count, 2U);
  EXPECT_TRUE(buffer.ego_filter_applied);
  EXPECT_EQ(buffer.prefiltered_ego_count, 1U);
  ASSERT_EQ(buffer.valid_count, 1U);
  EXPECT_FLOAT_EQ(storage[0].x(), 0.6f);
  EXPECT_EQ(storage[0].timestamp(), 12 * kTestSecondToNano);

  size_t ego_filtered_count = 0;
  size_t voxel_filtered_count = 0;
  EXPECT_EQ(filter_policy.ApplyFilters(&buffer, &ego_filtered_count,
                                       &voxel_filtered_count),
            1U);
  EXPECT_EQ(ego_filtered_count, 1U);
  EXPECT_EQ(voxel_filtered_count, 0U);
}

TEST(CpuLidarFusionPolicyTest,
     ReportsEgoFilterCountWhenStaticFusionRejectsAllPoints) {
  MockBuffer tf_buffer;
  auto config = MakeConfig();
  config.set_enable_voxel_filter(false);
  CpuLidarFusionPolicy fusion_policy;
  CpuLidarFilterPolicy filter_policy;
  ASSERT_TRUE(fusion_policy.Init(config, &tf_buffer));
  ASSERT_TRUE(filter_policy.Init(config));

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.all_points_have_timestamps = true;
  frame_context.point_cloud = MakePointCloud(
      "lidar", 12.0, {{0.1f, 0.1f, 0.0f, 12 * kTestSecondToNano}});

  std::vector<PointXYZIT> storage(1);
  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;

  const std::vector<std::vector<double>> sample_times{{12.0}};
  const std::vector<std::vector<Eigen::Affine3d>> poses{{
      Eigen::Affine3d::Identity(),
  }};
  ASSERT_TRUE(fusion_policy.FuseToBaseLink(
      12.0, Eigen::Affine3d::Identity(), {frame_context}, poses,
      sample_times, &buffer));
  EXPECT_EQ(buffer.valid_count, 0U);
  EXPECT_EQ(buffer.prefiltered_ego_count, 1U);

  size_t ego_filtered_count = 0;
  size_t voxel_filtered_count = 0;
  EXPECT_EQ(filter_policy.ApplyFilters(&buffer, &ego_filtered_count,
                                       &voxel_filtered_count),
            0U);
  EXPECT_EQ(ego_filtered_count, 1U);
  EXPECT_EQ(voxel_filtered_count, 0U);
}

TEST(CpuLidarFilterPolicyTest, AppliesEgoFilterBeforeVoxelDownsample) {
  CpuLidarFilterPolicy policy;
  ASSERT_TRUE(policy.Init(MakeConfig()));

  std::vector<PointXYZIT> storage(4);
  storage[0].set_x(0.1f);
  storage[0].set_y(0.1f);
  storage[1].set_x(0.6f);
  storage[1].set_y(0.1f);
  storage[2].set_x(1.6f);
  storage[2].set_y(0.1f);

  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = 3;
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;
  buffer.device_id = -1;

  size_t ego_filtered_count = 0;
  size_t voxel_filtered_count = 0;
  const size_t final_count =
      policy.ApplyFilters(&buffer, &ego_filtered_count, &voxel_filtered_count);
  EXPECT_EQ(final_count, 2U);
  EXPECT_EQ(buffer.valid_count, 2U);
  EXPECT_EQ(ego_filtered_count, 1U);
  EXPECT_EQ(voxel_filtered_count, 0U);
  EXPECT_FLOAT_EQ(storage[0].x(), 0.6f);
  EXPECT_FLOAT_EQ(storage[1].x(), 1.6f);
}

TEST(CpuLidarFilterPolicyTest,
     AggregatesDeterministicVoxelCentroidAndIntensity) {
  CpuLidarFilterPolicy policy;
  auto config = MakeConfig();
  config.set_enable_ego_query_filter(false);
  ASSERT_TRUE(policy.Init(config));

  std::vector<PointXYZIT> storage(4);
  storage[0].set_x(0.1f);
  storage[0].set_y(0.1f);
  storage[0].set_z(0.0f);
  storage[0].set_intensity(1.0f);
  storage[0].set_timestamp(10U);
  storage[1].set_x(0.3f);
  storage[1].set_y(0.3f);
  storage[1].set_z(0.0f);
  storage[1].set_intensity(3.0f);
  storage[1].set_timestamp(14U);

  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = 2;
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;
  buffer.device_id = -1;

  size_t ego_filtered_count = 0;
  size_t voxel_filtered_count = 0;
  const size_t final_count =
      policy.ApplyFilters(&buffer, &ego_filtered_count, &voxel_filtered_count);
  EXPECT_EQ(final_count, 1U);
  EXPECT_EQ(voxel_filtered_count, 1U);
  EXPECT_FLOAT_EQ(storage[0].x(), 0.2f);
  EXPECT_FLOAT_EQ(storage[0].y(), 0.2f);
  EXPECT_FLOAT_EQ(storage[0].intensity(), 2.0f);
  EXPECT_EQ(storage[0].timestamp(), 12U);
}

TEST(CpuLidarFilterPolicyTest, LeavesVoxelsUntouchedWhenDisabled) {
  CpuLidarFilterPolicy policy;
  auto config = MakeConfig();
  config.set_enable_ego_query_filter(false);
  config.set_enable_voxel_filter(false);
  ASSERT_TRUE(policy.Init(config));

  std::vector<PointXYZIT> storage(2);
  storage[0].set_x(0.1f);
  storage[1].set_x(0.2f);
  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = storage.size();
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;

  size_t ego_filtered_count = 0;
  size_t voxel_filtered_count = 0;
  EXPECT_EQ(policy.ApplyFilters(&buffer, &ego_filtered_count,
                                &voxel_filtered_count),
            2U);
  EXPECT_EQ(voxel_filtered_count, 0U);
}

TEST(LidarUnifiedComponentTest, RejectsPrimarySensorIdDrift) {
  LidarUnifiedComponent component;
  component.primary_sensor_id_ = "lidar_primary";

  EXPECT_FALSE(component.OnReceiveMainLidar(
      MakePointCloud("lidar_secondary", 12.0, {{0.0f, 0.0f, 0.0f, 0U}})));
  EXPECT_EQ(component.primary_sensor_id_, "lidar_primary");
}

TEST(LidarUnifiedComponentTest, FindsNearestFrameFromOutOfOrderBuffer) {
  LidarUnifiedComponent component;
  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  sensor_state->frames.push_back(
      MakeBufferedFrame("lidar", 10.10, 10.20, 1));
  sensor_state->frames.push_back(
      MakeBufferedFrame("lidar", 9.90, 10.00, 2));
  sensor_state->frames.push_back(
      MakeBufferedFrame("lidar", 10.00, 10.10, 3));

  FrameHandle nearest_frame;
  auto failure_reason = LidarUnifiedComponent::FrameLookupFailureReason::kNone;
  ASSERT_TRUE(component.FindNearestFrame(
      sensor_state, "lidar", MakeTimeContract(9.95, 10.05), 100,
      &nearest_frame, &failure_reason));
  ASSERT_NE(nearest_frame.point_cloud, nullptr);
  EXPECT_EQ(failure_reason,
            LidarUnifiedComponent::FrameLookupFailureReason::kNone);
  EXPECT_DOUBLE_EQ(nearest_frame.point_cloud->measurement_time(), 10.00);
}

TEST(LidarUnifiedComponentTest, ReportsTimeDeltaExceededForNearestFrame) {
  LidarUnifiedComponent component;
  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  auto late_frame = MakeBufferedFrame("lidar", 10.20, 10.30, 1);
  auto early_frame = MakeBufferedFrame("lidar", 9.90, 10.00, 2);
  sensor_state->frames.push_back(late_frame);
  sensor_state->frames.push_back(early_frame);

  FrameHandle nearest_frame;
  auto failure_reason = LidarUnifiedComponent::FrameLookupFailureReason::kNone;
  EXPECT_FALSE(component.FindNearestFrame(
      sensor_state, "lidar", MakeTimeContract(9.95, 10.05), 40,
      &nearest_frame, &failure_reason));
  EXPECT_EQ(
      failure_reason,
      LidarUnifiedComponent::FrameLookupFailureReason::kTimeDeltaExceeded);
}

TEST(LidarUnifiedComponentTest, AppliesFixedDelayDuringFrameLookup) {
  LidarUnifiedComponent component;
  component.config_.set_enable_online_time_offset_update(true);
  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  sensor_state->fixed_delay_initialized = true;
  sensor_state->fixed_delay_sec = -0.02;

  auto aligned_frame = MakeBufferedFrame("lidar", 9.92, 10.02, 1);
  auto farther_frame = MakeBufferedFrame("lidar", 9.95, 10.05, 2);
  sensor_state->frames.push_back(farther_frame);
  sensor_state->frames.push_back(aligned_frame);

  FrameHandle nearest_frame;
  auto failure_reason = LidarUnifiedComponent::FrameLookupFailureReason::kNone;
  ASSERT_TRUE(component.FindNearestFrame(
      sensor_state, "lidar", MakeTimeContract(9.90, 10.0), 40,
      &nearest_frame, &failure_reason));
  EXPECT_DOUBLE_EQ(nearest_frame.point_cloud->measurement_time(), 10.02);
  EXPECT_NEAR(nearest_frame.clock_offset_residual_ms, 0.0, 1e-6);
}

TEST(LidarUnifiedComponentTest, PrefersMaximumIntervalOverlap) {
  LidarUnifiedComponent component;
  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  sensor_state->frames.push_back(
      MakeBufferedFrame("lidar", 9.90, 10.01, 1));
  sensor_state->frames.push_back(
      MakeBufferedFrame("lidar", 9.99, 10.04, 2));

  FrameHandle selected;
  auto failure_reason = LidarUnifiedComponent::FrameLookupFailureReason::kNone;
  ASSERT_TRUE(component.FindNearestFrame(
      sensor_state, "lidar", MakeTimeContract(9.90, 10.0), 100, &selected,
      &failure_reason));
  EXPECT_EQ(selected.frame_id, 1U);
}

TEST(LidarUnifiedComponentTest, BreaksOverlapTieByAnchorDistance) {
  LidarUnifiedComponent component;
  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  sensor_state->frames.push_back(
      MakeBufferedFrame("lidar", 9.95, 10.05, 1));
  sensor_state->frames.push_back(
      MakeBufferedFrame("lidar", 9.94, 9.99, 2));

  FrameHandle selected;
  auto failure_reason = LidarUnifiedComponent::FrameLookupFailureReason::kNone;
  ASSERT_TRUE(component.FindNearestFrame(
      sensor_state, "lidar", MakeTimeContract(9.90, 10.0), 100, &selected,
      &failure_reason));
  EXPECT_EQ(selected.frame_id, 2U);
}

TEST(LidarUnifiedComponentTest, ExcludesFramesOnlyAfterCommit) {
  LidarUnifiedComponent component;
  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  auto frame = MakeBufferedFrame("aux", 9.9, 10.0, 7);
  sensor_state->frames.push_back(frame);
  component.sensor_states_["aux"] = sensor_state;

  FrameHandle first;
  auto failure_reason = LidarUnifiedComponent::FrameLookupFailureReason::kNone;
  ASSERT_TRUE(component.FindNearestFrame(
      sensor_state, "aux", MakeTimeContract(9.9, 10.0), 10, &first,
      &failure_reason));
  FrameHandle retry;
  ASSERT_TRUE(component.FindNearestFrame(
      sensor_state, "aux", MakeTimeContract(9.9, 10.0), 10, &retry,
      &failure_reason));

  component.CommitSelectedFrames({first});
  EXPECT_FALSE(component.FindNearestFrame(
      sensor_state, "aux", MakeTimeContract(9.9, 10.0), 10, &retry,
      &failure_reason));
}

TEST(LidarUnifiedComponentTest, UpdatesSensorTimingModel) {
  LidarUnifiedComponent component;
  component.config_.set_enable_online_time_offset_update(true);
  component.config_.set_fixed_delay_ema_alpha(0.5);
  component.config_.set_fixed_delay_update_limit_ms(100.0);
  component.config_.set_clock_offset_ema_alpha(0.5);

  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  component.sensor_states_["aux_lidar"] = sensor_state;

  LidarUnifiedComponent::FrameMetrics metrics;
  FrameHandle first_handle;
  first_handle.sensor_id = "aux_lidar";
  first_handle.point_cloud = MakePointCloud("aux_lidar", 10.02, {});
  first_handle.time_contract = MakeTimeContract(9.92, 10.02);
  first_handle.clock_offset_residual_ms = 4.0;
  component.UpdateSensorTimingModel(first_handle, 10.0, &metrics);

  EXPECT_TRUE(sensor_state->fixed_delay_initialized);
  EXPECT_NEAR(sensor_state->fixed_delay_sec, -0.02, 1e-9);
  EXPECT_DOUBLE_EQ(sensor_state->smoothed_clock_offset_ms, 4.0);
  EXPECT_DOUBLE_EQ(metrics.max_abs_clock_offset_ms, 4.0);

  FrameHandle second_handle;
  second_handle.sensor_id = "aux_lidar";
  second_handle.point_cloud = MakePointCloud("aux_lidar", 10.01, {});
  second_handle.time_contract = MakeTimeContract(9.91, 10.01);
  second_handle.clock_offset_residual_ms = 2.0;
  component.UpdateSensorTimingModel(second_handle, 10.0, &metrics);

  EXPECT_NEAR(sensor_state->fixed_delay_sec, -0.015, 1e-9);
  EXPECT_DOUBLE_EQ(sensor_state->smoothed_clock_offset_ms, 3.0);
}

TEST(LidarUnifiedComponentTest,
     UpdatesLargeFixedDelayWhenInnovationIsWithinLimit) {
  LidarUnifiedComponent component;
  component.config_.set_enable_online_time_offset_update(true);
  component.config_.set_fixed_delay_ema_alpha(0.5);
  component.config_.set_fixed_delay_update_limit_ms(10.0);
  component.config_.set_clock_offset_ema_alpha(0.5);

  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  sensor_state->fixed_delay_initialized = true;
  sensor_state->fixed_delay_sec = 0.09;
  component.sensor_states_["aux_lidar"] = sensor_state;

  LidarUnifiedComponent::FrameMetrics metrics;
  FrameHandle handle;
  handle.sensor_id = "aux_lidar";
  handle.point_cloud = MakePointCloud("aux_lidar", 9.905, {});
  handle.time_contract = MakeTimeContract(9.805, 9.905);
  handle.clock_offset_residual_ms = 5.0;
  component.UpdateSensorTimingModel(handle, 10.0, &metrics);

  EXPECT_NEAR(sensor_state->fixed_delay_sec, 0.0925, 1e-9);
  EXPECT_DOUBLE_EQ(sensor_state->smoothed_clock_offset_ms, 5.0);
}

TEST(LidarUnifiedComponentTest, KeepsOnlineOffsetDisabledForMatching) {
  LidarUnifiedComponent component;
  component.config_.set_enable_online_time_offset_update(false);
  component.config_.set_clock_offset_ema_alpha(0.5);
  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  component.sensor_states_["aux"] = sensor_state;

  FrameHandle handle;
  handle.sensor_id = "aux";
  handle.point_cloud = MakePointCloud("aux", 10.02, {});
  handle.time_contract = MakeTimeContract(9.92, 10.02);
  handle.clock_offset_residual_ms = -20.0;
  LidarUnifiedComponent::FrameMetrics metrics;
  component.UpdateSensorTimingModel(handle, 10.0, &metrics);

  EXPECT_FALSE(sensor_state->fixed_delay_initialized);
  EXPECT_DOUBLE_EQ(sensor_state->fixed_delay_sec, 0.0);
  EXPECT_DOUBLE_EQ(sensor_state->smoothed_clock_offset_ms, -20.0);
}

TEST(LidarUnifiedComponentTest, CollectNearestFramesSkipsLowQualityAuxiliary) {
  LidarUnifiedComponent component;
  component.config_.set_strict_auxiliary_sync(false);
  component.config_.set_max_ref_time_delta_ms(100);
  component.config_.set_auxiliary_min_overlap_quality_weight(0.2);
  component.config_.set_enable_overlap_quality_gate(true);
  component.auxiliary_inputs_.push_back(
      LidarUnifiedComponent::SensorInput{
          "/aux", LidarUnifiedComponentConfig::TimeSettings()});
  component.auxiliary_sensor_ids_by_topic_["/aux"] = "aux_lidar";

  auto primary_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  auto primary_frame = MakeBufferedFrame("primary", 9.9, 10.0, 1);
  primary_state->frames.push_back(primary_frame);
  component.sensor_states_["primary"] = primary_state;

  auto auxiliary_state =
      std::make_shared<LidarUnifiedComponent::SensorState>(4);
  auxiliary_state->overlap_quality_weight = 0.1;
  auxiliary_state->frames.push_back(
      MakeBufferedFrame("aux_lidar", 9.9, 10.0, 2));
  component.sensor_states_["aux_lidar"] = auxiliary_state;

  std::vector<FrameHandle> frame_handles;
  LidarUnifiedComponent::FrameMetrics metrics;
  ASSERT_TRUE(component.CollectNearestFrames(
      "primary", primary_frame, &frame_handles, &metrics));
  ASSERT_EQ(frame_handles.size(), 1U);
  EXPECT_TRUE(frame_handles.front().is_primary);
  EXPECT_EQ(metrics.missing_auxiliary_count, 1U);
}

TEST(LidarUnifiedComponentTest, CollectNearestFramesMatchesThreeSensors) {
  LidarUnifiedComponent component;
  component.config_.set_strict_auxiliary_sync(false);
  component.config_.set_max_ref_time_delta_ms(80);
  component.config_.set_auxiliary_min_overlap_quality_weight(0.0);
  component.auxiliary_inputs_.push_back(
      LidarUnifiedComponent::SensorInput{
          "/left", LidarUnifiedComponentConfig::TimeSettings()});
  component.auxiliary_inputs_.push_back(
      LidarUnifiedComponent::SensorInput{
          "/right", LidarUnifiedComponentConfig::TimeSettings()});
  component.auxiliary_sensor_ids_by_topic_["/left"] = "left_lidar";
  component.auxiliary_sensor_ids_by_topic_["/right"] = "right_lidar";

  auto primary_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  auto primary_frame = MakeBufferedFrame("primary", 9.9, 10.0, 1);
  primary_state->frames.push_back(primary_frame);
  component.sensor_states_["primary"] = primary_state;

  auto left_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  left_state->frames.push_back(
      MakeBufferedFrame("left_lidar", 9.92, 10.02, 2));
  component.sensor_states_["left_lidar"] = left_state;

  auto right_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  right_state->frames.push_back(
      MakeBufferedFrame("right_lidar", 9.87, 9.97, 3));
  component.sensor_states_["right_lidar"] = right_state;

  std::vector<FrameHandle> frame_handles;
  LidarUnifiedComponent::FrameMetrics metrics;
  ASSERT_TRUE(component.CollectNearestFrames(
      "primary", primary_frame, &frame_handles, &metrics));
  EXPECT_EQ(frame_handles.size(), 3U);
  EXPECT_EQ(metrics.expected_sensor_count, 3U);
  EXPECT_EQ(metrics.matched_sensor_count, 3U);
  EXPECT_EQ(metrics.missing_auxiliary_count, 0U);
}

TEST(LidarUnifiedComponentTest, CollectNearestFramesAllowsMissingAuxiliary) {
  LidarUnifiedComponent component;
  component.config_.set_strict_auxiliary_sync(false);
  component.config_.set_max_ref_time_delta_ms(80);
  component.auxiliary_inputs_.push_back(
      LidarUnifiedComponent::SensorInput{
          "/left", LidarUnifiedComponentConfig::TimeSettings()});
  component.auxiliary_inputs_.push_back(
      LidarUnifiedComponent::SensorInput{
          "/right", LidarUnifiedComponentConfig::TimeSettings()});
  component.auxiliary_sensor_ids_by_topic_["/left"] = "left_lidar";

  auto primary_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  auto primary_frame = MakeBufferedFrame("primary", 9.9, 10.0, 1);
  primary_state->frames.push_back(primary_frame);
  component.sensor_states_["primary"] = primary_state;

  auto left_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  left_state->frames.push_back(
      MakeBufferedFrame("left_lidar", 9.92, 10.02, 2));
  component.sensor_states_["left_lidar"] = left_state;

  std::vector<FrameHandle> frame_handles;
  LidarUnifiedComponent::FrameMetrics metrics;
  ASSERT_TRUE(component.CollectNearestFrames(
      "primary", primary_frame, &frame_handles, &metrics));
  EXPECT_EQ(frame_handles.size(), 2U);
  EXPECT_EQ(metrics.expected_sensor_count, 3U);
  EXPECT_EQ(metrics.matched_sensor_count, 2U);
  EXPECT_EQ(metrics.missing_auxiliary_count, 1U);
}

TEST(LidarUnifiedComponentTest,
     CollectNearestFramesFailsStrictMissingAuxiliary) {
  LidarUnifiedComponent component;
  component.config_.set_strict_auxiliary_sync(true);
  component.config_.set_max_ref_time_delta_ms(80);
  component.auxiliary_inputs_.push_back(
      LidarUnifiedComponent::SensorInput{
          "/left", LidarUnifiedComponentConfig::TimeSettings()});
  component.auxiliary_sensor_ids_by_topic_["/left"] = "left_lidar";

  auto primary_frame = MakeBufferedFrame("primary", 9.9, 10.0, 1);
  auto primary_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  primary_state->frames.push_back(primary_frame);
  component.sensor_states_["primary"] = primary_state;

  std::vector<FrameHandle> frame_handles;
  LidarUnifiedComponent::FrameMetrics metrics;
  EXPECT_FALSE(component.CollectNearestFrames(
      "primary", primary_frame, &frame_handles, &metrics));
}

TEST(LidarUnifiedComponentTest, RejectsDuplicateAuxiliaryTopics) {
  LidarUnifiedComponent component;
  component.config_ = MakeConfig();
  component.config_.set_output_channel("/out");
  component.config_.set_sensor_buffer_size(2);
  component.config_.set_max_full_pointcloud_points(16);
  component.config_.set_max_ref_time_delta_ms(80);
  component.config_.set_metrics_log_interval(1);
  component.config_.set_sensor_pose_cache_duration_sec(1.0);
  component.config_.set_sensor_pose_cache_max_extrapolation_sec(0.1);
  component.config_.set_sensor_pose_query_timeout_sec(0.0);
  component.config_.set_fixed_delay_ema_alpha(0.5);
  component.config_.set_clock_offset_ema_alpha(0.5);
  component.config_.set_overlap_quality_ema_alpha(0.5);
  component.config_.set_overlap_quality_sample_stride(1);
  component.config_.set_pending_fusion_queue_size(2);
  component.config_.set_fusion_flush_interval_ms(5);
  auto* first = component.config_.add_auxiliary_lidar_inputs();
  first->set_topic_name("/aux");
  auto* second = component.config_.add_auxiliary_lidar_inputs();
  second->set_topic_name("/aux");

  EXPECT_FALSE(component.ValidateConfig());
}

TEST(LidarUnifiedComponentTest, RejectsImpossibleScanDurations) {
  LidarUnifiedComponent component;
  component.config_ = MakeConfig();
  component.config_.set_output_channel("/out");
  component.config_.set_sensor_buffer_size(2);
  component.config_.set_max_full_pointcloud_points(16);
  component.config_.set_max_ref_time_delta_ms(80);
  component.config_.set_metrics_log_interval(1);
  component.config_.set_sensor_pose_cache_duration_sec(1.0);
  component.config_.set_sensor_pose_cache_max_extrapolation_sec(0.1);
  component.config_.set_sensor_pose_query_timeout_sec(0.0);
  component.config_.set_fixed_delay_ema_alpha(0.5);
  component.config_.set_clock_offset_ema_alpha(0.5);
  component.config_.set_overlap_quality_ema_alpha(0.5);
  component.config_.set_overlap_quality_sample_stride(1);
  component.config_.set_pending_fusion_queue_size(2);
  component.config_.set_fusion_flush_interval_ms(5);
  component.config_.mutable_primary_time_settings()
      ->set_expected_scan_duration_ms(201.0);
  component.config_.mutable_primary_time_settings()
      ->set_max_scan_duration_ms(200.0);

  EXPECT_FALSE(component.ValidateConfig());
}

TEST(LidarUnifiedComponentTest, OffCompensationUsesStaticExtrinsicOnly) {
  MockBuffer tf_buffer;
  tf_buffer.AddTransform(
      "base_link", "lidar", cyber::Time(1.0),
      Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  LidarUnifiedComponent component;
  component.config_ = MakeConfig();
  component.config_.set_compensation_mode(LidarUnifiedComponentConfig::OFF);
  component.config_.set_enable_ego_query_filter(false);
  component.config_.set_enable_voxel_filter(false);
  component.tf_buffer_ = &tf_buffer;
  component.deskew_policy_ = std::make_unique<CpuLidarDeskewPolicy>();
  component.fusion_policy_ = std::make_unique<CpuLidarFusionPolicy>();
  component.filter_policy_ = std::make_unique<CpuLidarFilterPolicy>();
  ASSERT_TRUE(component.deskew_policy_->Init(component.config_, &tf_buffer));
  ASSERT_TRUE(component.fusion_policy_->Init(component.config_, &tf_buffer));
  ASSERT_TRUE(component.filter_policy_->Init(component.config_));
  component.full_pointcloud_buffer_.resize(8);

  auto cloud = MakePointCloud(
      "lidar", 10.0, {{1.0f, 0.0f, 0.0f, 10000000000ULL}});
  auto buffered = MakeBufferedFrame("lidar", 9.9, 10.0, 1);
  buffered->point_cloud = cloud;
  buffered->time_contract.scan_begin_ns += 250;
  buffered->time_contract.scan_end_ns += 250;
  buffered->time_contract.canonical_anchor_ns += 250;
  buffered->time_contract.static_offset_ns = 250;
  FrameHandle handle;
  handle.sensor_id = "lidar";
  handle.point_cloud = cloud;
  handle.buffered_frame = buffered;
  handle.frame_id = buffered->frame_id;
  handle.time_contract = buffered->time_contract;
  handle.is_primary = true;

  LidarUnifiedComponent::FrameMetrics metrics;
  std::shared_ptr<PointCloud> output;
  ASSERT_TRUE(
      component.BuildUnifiedPointCloud(cloud, {handle}, &metrics, &output));
  ASSERT_NE(output, nullptr);
  ASSERT_EQ(output->point_size(), 1);
  EXPECT_NEAR(output->point(0).x(), 3.0, 1e-6);
  EXPECT_DOUBLE_EQ(output->measurement_time(), 10.00000025);
  EXPECT_EQ(output->point(0).timestamp(), 10000000250ULL);
}

TEST(LidarUnifiedComponentTest, EstimatesOverlapQualityWeight) {
  LidarUnifiedComponent component;
  component.config_.set_overlap_quality_sample_stride(1);
  component.config_.set_overlap_region_forward_x(1.0);
  component.config_.set_overlap_region_backward_x(-1.0);
  component.config_.set_overlap_region_left_y(1.0);
  component.config_.set_overlap_region_right_y(-1.0);
  component.config_.set_overlap_region_min_z(-1.0);
  component.config_.set_overlap_region_max_z(1.0);

  BufferedFrame frame;
  frame.point_cloud = MakePointCloud(
      "lidar", 10.0, {{0.0f, 0.0f, 0.0f, 10U}, {5.0f, 5.0f, 0.0f, 10U}});
  frame.motion_sample_times = {10.0};
  frame.motion_poses = {Eigen::Affine3d::Identity()};
  frame.pose_prefetch_ok = true;

  const double overlap_weight = component.EstimateOverlapQualityWeight(
      frame, Eigen::Affine3d::Identity());
  EXPECT_DOUBLE_EQ(overlap_weight, 0.5);
}

TEST(GpuLidarFilterPolicyTest, AppliesGpuFilteringOrFailsWithoutBackend) {
  GpuLidarFilterPolicy policy;
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  ASSERT_TRUE(policy.Init(MakeConfig()));
#else
  EXPECT_FALSE(policy.Init(MakeConfig()));
  return;
#endif

  std::vector<PointXYZIT> storage(4);
  storage[0].set_x(0.1f);
  storage[0].set_y(0.1f);
  storage[1].set_x(0.6f);
  storage[1].set_y(0.1f);
  storage[2].set_x(1.6f);
  storage[2].set_y(0.1f);

  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = 3;
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;
  buffer.device_id = 0;

  size_t ego_filtered_count = 0;
  size_t voxel_filtered_count = 0;
  const size_t final_count =
      policy.ApplyFilters(&buffer, &ego_filtered_count, &voxel_filtered_count);
  EXPECT_EQ(final_count, 2U);
  EXPECT_EQ(ego_filtered_count, 1U);
  EXPECT_EQ(voxel_filtered_count, 0U);
}

#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
TEST(GpuLidarFilterPolicyTest,
     AggregatesDeterministicVoxelCentroidAndIntensity) {
  GpuLidarFilterPolicy policy;
  auto config = MakeConfig();
  config.set_enable_ego_query_filter(false);
  config.set_gpu_device_id(0);
  ASSERT_TRUE(policy.Init(config));

  std::vector<PointXYZIT> storage(4);
  storage[0].set_x(0.1f);
  storage[0].set_y(0.1f);
  storage[0].set_intensity(1.0f);
  storage[0].set_timestamp(10U);
  storage[1].set_x(0.3f);
  storage[1].set_y(0.3f);
  storage[1].set_intensity(3.0f);
  storage[1].set_timestamp(14U);

  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = 2;
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;
  buffer.device_id = 0;

  size_t ego_filtered_count = 0;
  size_t voxel_filtered_count = 0;
  const size_t final_count =
      policy.ApplyFilters(&buffer, &ego_filtered_count, &voxel_filtered_count);
  EXPECT_EQ(final_count, 1U);
  EXPECT_EQ(voxel_filtered_count, 1U);
  EXPECT_FLOAT_EQ(storage[0].x(), 0.2f);
  EXPECT_FLOAT_EQ(storage[0].y(), 0.2f);
  EXPECT_FLOAT_EQ(storage[0].intensity(), 2.0f);
  EXPECT_EQ(storage[0].timestamp(), 12U);
}

TEST(GpuLidarFusionPolicyTest, FusesPointsIntoReferenceBaseFrame) {
  MockBuffer tf_buffer;
  GpuLidarFusionPolicy policy;
  auto config = MakeConfig();
  config.set_gpu_device_id(0);
  ASSERT_TRUE(policy.Init(config, &tf_buffer));
  const Eigen::Affine3d map2base_ref =
      (Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::Quaterniond::Identity())
          .inverse();

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.point_cloud =
      MakePointCloud("lidar", 12.0,
                     {{1.0f, 0.0f, 0.0f, 10 * kTestSecondToNano},
                      {1.0f, 0.0f, 0.0f, 12 * kTestSecondToNano}});

  std::vector<PointXYZIT> storage(8);
  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = 0;
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;
  buffer.device_id = -1;

  const std::vector<std::vector<double>> sample_times{{10.0, 12.0}};
  const std::vector<std::vector<Eigen::Affine3d>> poses{{
      Eigen::Translation3d(5.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(),
      Eigen::Translation3d(7.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(),
  }};

  ASSERT_TRUE(policy.FuseToBaseLink(12.0, map2base_ref, {frame_context},
                                    poses, sample_times, &buffer));
  ASSERT_EQ(buffer.valid_count, 2U);
  std::vector<float> xs{storage[0].x(), storage[1].x()};
  std::sort(xs.begin(), xs.end());
  EXPECT_FLOAT_EQ(xs[0], 4.0f);
  EXPECT_FLOAT_EQ(xs[1], 6.0f);
}

TEST(GpuLidarFusionPolicyTest, InterpolatesIntermediatePoseBins) {
  MockBuffer tf_buffer;
  GpuLidarFusionPolicy policy;
  auto config = MakeConfig();
  config.set_gpu_device_id(0);
  ASSERT_TRUE(policy.Init(config, &tf_buffer));
  const Eigen::Affine3d map2base_ref = Eigen::Affine3d::Identity();

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.point_cloud =
      MakePointCloud(
          "lidar", 11.0,
          {{0.0f, 0.0f, 0.0f, 11 * kTestSecondToNano}});

  std::vector<PointXYZIT> storage(4);
  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = 0;
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;
  buffer.device_id = -1;

  const std::vector<std::vector<double>> sample_times{{10.0, 12.0}};
  const std::vector<std::vector<Eigen::Affine3d>> poses{{
      Eigen::Translation3d(5.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(),
      Eigen::Translation3d(7.0, 0.0, 0.0) * Eigen::Quaterniond::Identity(),
  }};

  ASSERT_TRUE(policy.FuseToBaseLink(11.0, map2base_ref, {frame_context},
                                    poses, sample_times, &buffer));
  ASSERT_EQ(buffer.valid_count, 1U);
  EXPECT_NEAR(storage[0].x(), 6.0f, 1e-4f);
}

TEST(GpuLidarFusionPolicyTest,
    PreservesRelativePrecisionWithLargeMapOffsets) {
  constexpr double kMapOffset = 1e8;

  MockBuffer tf_buffer;

  GpuLidarFusionPolicy policy;
  auto config = MakeConfig();
  config.set_gpu_device_id(0);
  ASSERT_TRUE(policy.Init(config, &tf_buffer));
    const Eigen::Affine3d map2base_ref =
      (Eigen::Translation3d(kMapOffset + 2.0, 0.0, 0.0) *
       Eigen::Quaterniond::Identity())
          .inverse();

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.point_cloud =
      MakePointCloud("lidar", 12.0,
                     {{1.0f, 0.0f, 0.0f, 10 * kTestSecondToNano},
                      {1.0f, 0.0f, 0.0f, 12 * kTestSecondToNano}});

  std::vector<PointXYZIT> storage(8);
  PointCloudBuffer buffer;
  buffer.data_ptr = storage.data();
  buffer.capacity = storage.size();
  buffer.valid_count = 0;
  buffer.item_size = sizeof(PointXYZIT);
  buffer.device_type = MemoryDeviceType::kHost;
  buffer.device_id = -1;

  const std::vector<std::vector<double>> sample_times{{10.0, 12.0}};
  const std::vector<std::vector<Eigen::Affine3d>> poses{{
      Eigen::Translation3d(kMapOffset + 5.0, 0.0, 0.0) *
          Eigen::Quaterniond::Identity(),
      Eigen::Translation3d(kMapOffset + 7.0, 0.0, 0.0) *
          Eigen::Quaterniond::Identity(),
  }};

    ASSERT_TRUE(policy.FuseToBaseLink(12.0, map2base_ref, {frame_context},
                                    poses, sample_times, &buffer));
  ASSERT_EQ(buffer.valid_count, 2U);
  std::vector<float> xs{storage[0].x(), storage[1].x()};
  std::sort(xs.begin(), xs.end());
  EXPECT_NEAR(xs[0], 4.0f, 1e-3f);
  EXPECT_NEAR(xs[1], 6.0f, 1e-3f);
}

TEST(GpuLidarDeskewPolicyTest, ComputesSampledPosesFromPointTimestamps) {
  MockBuffer tf_buffer;
  tf_buffer.AddTransform(
    "map", "lidar", cyber::Time(10.0),
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  tf_buffer.AddTransform(
    "map", "lidar", cyber::Time(10.5),
      Eigen::Translation3d(0.5, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  tf_buffer.AddTransform(
    "map", "lidar", cyber::Time(11.0),
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  GpuLidarDeskewPolicy policy;
  auto config = MakeConfig();
  config.set_gpu_device_id(0);
  ASSERT_TRUE(policy.Init(config, &tf_buffer));

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.point_cloud =
      MakePointCloud("lidar", 11.0,
                     {{0.0f, 0.0f, 0.0f, 10 * kTestSecondToNano},
                      {0.0f, 0.0f, 0.0f, 11 * kTestSecondToNano}});

  std::vector<double> sample_times;
  std::vector<Eigen::Affine3d> poses;
  ASSERT_TRUE(policy.ComputeMotionCompensationPoses(frame_context,
                                                    &sample_times, &poses));
  ASSERT_EQ(sample_times.size(), 3U);
  ASSERT_EQ(poses.size(), 3U);
  EXPECT_DOUBLE_EQ(sample_times.front(), 10.0);
  EXPECT_DOUBLE_EQ(sample_times.back(), 11.0);
  EXPECT_DOUBLE_EQ(poses[0].translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(poses[1].translation().x(), 0.5);
  EXPECT_DOUBLE_EQ(poses[2].translation().x(), 1.0);
}
#endif

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
