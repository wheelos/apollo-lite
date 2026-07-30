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
#include "modules/drivers/lidar/processor/lidar_unified_component.h"
#include "modules/drivers/lidar/processor/policy/cpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/gpu_lidar_policy.h"

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

constexpr uint64_t kSecondToNano = 1000000000ULL;

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

LidarUnifiedComponentConfig MakeConfig() {
  LidarUnifiedComponentConfig config;
  config.set_map_frame_id("map");
  config.set_base_link_frame_id("base_link");
  config.set_motion_compensation_bins(3);
  config.set_voxel_size(1.0f);
  config.set_enable_ego_query_filter(true);
  config.set_ego_box_forward_x(0.5f);
  config.set_ego_box_backward_x(-0.5f);
  config.set_ego_box_forward_y(0.5f);
  config.set_ego_box_backward_y(-0.5f);
  return config;
}

}  // namespace

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
                     {{0.0f, 0.0f, 0.0f, 10 * kSecondToNano},
                      {0.0f, 0.0f, 0.0f, 11 * kSecondToNano}});

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
                     {{1.0f, 0.0f, 0.0f, 10 * kSecondToNano},
                      {1.0f, 0.0f, 0.0f, 12 * kSecondToNano}});

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
      MakePointCloud("lidar", 11.0, {{0.0f, 0.0f, 0.0f, 11 * kSecondToNano}});

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
  auto make_buffered_frame = [](double timestamp_sec) {
    auto frame = std::make_shared<BufferedFrame>();
    frame->point_cloud = MakePointCloud("lidar", timestamp_sec, {});
    frame->pose_prefetch_ok = true;
    return frame;
  };
  sensor_state->frames.push_back(make_buffered_frame(10.20));
  sensor_state->frames.push_back(make_buffered_frame(10.00));
  sensor_state->frames.push_back(make_buffered_frame(10.10));

  FrameHandle nearest_frame;
  auto failure_reason = LidarUnifiedComponent::FrameLookupFailureReason::kNone;
  ASSERT_TRUE(component.FindNearestFrame(sensor_state, "lidar", 10.05, 100,
                                         &nearest_frame, &failure_reason));
  ASSERT_NE(nearest_frame.point_cloud, nullptr);
  EXPECT_EQ(failure_reason,
            LidarUnifiedComponent::FrameLookupFailureReason::kNone);
  EXPECT_DOUBLE_EQ(nearest_frame.point_cloud->measurement_time(), 10.00);
}

TEST(LidarUnifiedComponentTest, ReportsTimeDeltaExceededForNearestFrame) {
  LidarUnifiedComponent component;
  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  auto late_frame = std::make_shared<BufferedFrame>();
  late_frame->point_cloud = MakePointCloud("lidar", 10.30, {});
  late_frame->pose_prefetch_ok = true;
  auto early_frame = std::make_shared<BufferedFrame>();
  early_frame->point_cloud = MakePointCloud("lidar", 10.00, {});
  early_frame->pose_prefetch_ok = true;
  sensor_state->frames.push_back(late_frame);
  sensor_state->frames.push_back(early_frame);

  FrameHandle nearest_frame;
  auto failure_reason = LidarUnifiedComponent::FrameLookupFailureReason::kNone;
  EXPECT_FALSE(component.FindNearestFrame(sensor_state, "lidar", 10.05, 40,
                                          &nearest_frame, &failure_reason));
  EXPECT_EQ(
      failure_reason,
      LidarUnifiedComponent::FrameLookupFailureReason::kTimeDeltaExceeded);
}

TEST(LidarUnifiedComponentTest, AppliesFixedDelayDuringFrameLookup) {
  LidarUnifiedComponent component;
  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  sensor_state->fixed_delay_initialized = true;
  sensor_state->fixed_delay_sec = -0.02;

  auto aligned_frame = std::make_shared<BufferedFrame>();
  aligned_frame->point_cloud = MakePointCloud("lidar", 10.02, {});
  aligned_frame->pose_prefetch_ok = true;
  auto farther_frame = std::make_shared<BufferedFrame>();
  farther_frame->point_cloud = MakePointCloud("lidar", 10.05, {});
  farther_frame->pose_prefetch_ok = true;
  sensor_state->frames.push_back(farther_frame);
  sensor_state->frames.push_back(aligned_frame);

  FrameHandle nearest_frame;
  auto failure_reason = LidarUnifiedComponent::FrameLookupFailureReason::kNone;
  ASSERT_TRUE(component.FindNearestFrame(sensor_state, "lidar", 10.0, 40,
                                         &nearest_frame, &failure_reason));
  EXPECT_DOUBLE_EQ(nearest_frame.point_cloud->measurement_time(), 10.02);
  EXPECT_NEAR(nearest_frame.clock_offset_residual_ms, 0.0, 1e-6);
}

TEST(LidarUnifiedComponentTest, UpdatesSensorTimingModel) {
  LidarUnifiedComponent component;
  component.config_.set_fixed_delay_ema_alpha(0.5);
  component.config_.set_fixed_delay_update_limit_ms(100.0);
  component.config_.set_clock_offset_ema_alpha(0.5);

  auto sensor_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  component.sensor_states_["aux_lidar"] = sensor_state;

  LidarUnifiedComponent::FrameMetrics metrics;
  FrameHandle first_handle;
  first_handle.sensor_id = "aux_lidar";
  first_handle.point_cloud = MakePointCloud("aux_lidar", 10.02, {});
  first_handle.clock_offset_residual_ms = 4.0;
  component.UpdateSensorTimingModel(first_handle, 10.0, &metrics);

  EXPECT_TRUE(sensor_state->fixed_delay_initialized);
  EXPECT_NEAR(sensor_state->fixed_delay_sec, -0.02, 1e-9);
  EXPECT_DOUBLE_EQ(sensor_state->smoothed_clock_offset_ms, 4.0);
  EXPECT_DOUBLE_EQ(metrics.max_abs_clock_offset_ms, 4.0);

  FrameHandle second_handle;
  second_handle.sensor_id = "aux_lidar";
  second_handle.point_cloud = MakePointCloud("aux_lidar", 10.01, {});
  second_handle.clock_offset_residual_ms = 2.0;
  component.UpdateSensorTimingModel(second_handle, 10.0, &metrics);

  EXPECT_NEAR(sensor_state->fixed_delay_sec, -0.015, 1e-9);
  EXPECT_DOUBLE_EQ(sensor_state->smoothed_clock_offset_ms, 3.0);
}

TEST(LidarUnifiedComponentTest,
     UpdatesLargeFixedDelayWhenInnovationIsWithinLimit) {
  LidarUnifiedComponent component;
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
  handle.clock_offset_residual_ms = 5.0;
  component.UpdateSensorTimingModel(handle, 10.0, &metrics);

  EXPECT_NEAR(sensor_state->fixed_delay_sec, 0.0925, 1e-9);
  EXPECT_DOUBLE_EQ(sensor_state->smoothed_clock_offset_ms, 5.0);
}

TEST(LidarUnifiedComponentTest, CollectNearestFramesSkipsLowQualityAuxiliary) {
  LidarUnifiedComponent component;
  component.config_.set_strict_auxiliary_sync(false);
  component.config_.set_max_ref_time_delta_ms(100);
  component.config_.set_auxiliary_min_overlap_quality_weight(0.2);
  component.auxiliary_inputs_.push_back(
      LidarUnifiedComponent::SensorInput{"/aux"});
  component.auxiliary_sensor_ids_by_topic_["/aux"] = "aux_lidar";

  auto make_buffered_frame = [](const std::string& sensor_id,
                                double timestamp_sec) {
    auto frame = std::make_shared<BufferedFrame>();
    frame->point_cloud = MakePointCloud(sensor_id, timestamp_sec, {});
    frame->pose_prefetch_ok = true;
    return frame;
  };

  auto primary_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  primary_state->frames.push_back(make_buffered_frame("primary", 10.0));
  component.sensor_states_["primary"] = primary_state;

  auto auxiliary_state =
      std::make_shared<LidarUnifiedComponent::SensorState>(4);
  auxiliary_state->overlap_quality_weight = 0.1;
  auxiliary_state->frames.push_back(make_buffered_frame("aux_lidar", 10.0));
  component.sensor_states_["aux_lidar"] = auxiliary_state;

  std::vector<FrameHandle> frame_handles;
  LidarUnifiedComponent::FrameMetrics metrics;
  ASSERT_TRUE(component.CollectNearestFrames(10.0, "primary", &frame_handles,
                                             &metrics));
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
      LidarUnifiedComponent::SensorInput{"/left"});
  component.auxiliary_inputs_.push_back(
      LidarUnifiedComponent::SensorInput{"/right"});
  component.auxiliary_sensor_ids_by_topic_["/left"] = "left_lidar";
  component.auxiliary_sensor_ids_by_topic_["/right"] = "right_lidar";

  auto make_buffered_frame = [](const std::string& sensor_id,
                                double timestamp_sec) {
    auto frame = std::make_shared<BufferedFrame>();
    frame->point_cloud = MakePointCloud(sensor_id, timestamp_sec, {});
    frame->pose_prefetch_ok = true;
    return frame;
  };

  auto primary_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  primary_state->frames.push_back(make_buffered_frame("primary", 10.0));
  component.sensor_states_["primary"] = primary_state;

  auto left_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  left_state->frames.push_back(make_buffered_frame("left_lidar", 10.02));
  component.sensor_states_["left_lidar"] = left_state;

  auto right_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  right_state->frames.push_back(make_buffered_frame("right_lidar", 9.97));
  component.sensor_states_["right_lidar"] = right_state;

  std::vector<FrameHandle> frame_handles;
  LidarUnifiedComponent::FrameMetrics metrics;
  ASSERT_TRUE(component.CollectNearestFrames(10.0, "primary", &frame_handles,
                                             &metrics));
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
      LidarUnifiedComponent::SensorInput{"/left"});
  component.auxiliary_inputs_.push_back(
      LidarUnifiedComponent::SensorInput{"/right"});
  component.auxiliary_sensor_ids_by_topic_["/left"] = "left_lidar";

  auto make_buffered_frame = [](const std::string& sensor_id,
                                double timestamp_sec) {
    auto frame = std::make_shared<BufferedFrame>();
    frame->point_cloud = MakePointCloud(sensor_id, timestamp_sec, {});
    frame->pose_prefetch_ok = true;
    return frame;
  };

  auto primary_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  primary_state->frames.push_back(make_buffered_frame("primary", 10.0));
  component.sensor_states_["primary"] = primary_state;

  auto left_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  left_state->frames.push_back(make_buffered_frame("left_lidar", 10.02));
  component.sensor_states_["left_lidar"] = left_state;

  std::vector<FrameHandle> frame_handles;
  LidarUnifiedComponent::FrameMetrics metrics;
  ASSERT_TRUE(component.CollectNearestFrames(10.0, "primary", &frame_handles,
                                             &metrics));
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
      LidarUnifiedComponent::SensorInput{"/left"});
  component.auxiliary_sensor_ids_by_topic_["/left"] = "left_lidar";

  auto primary_frame = std::make_shared<BufferedFrame>();
  primary_frame->point_cloud = MakePointCloud("primary", 10.0, {});
  primary_frame->pose_prefetch_ok = true;
  auto primary_state = std::make_shared<LidarUnifiedComponent::SensorState>(4);
  primary_state->frames.push_back(primary_frame);
  component.sensor_states_["primary"] = primary_state;

  std::vector<FrameHandle> frame_handles;
  LidarUnifiedComponent::FrameMetrics metrics;
  EXPECT_FALSE(component.CollectNearestFrames(10.0, "primary", &frame_handles,
                                              &metrics));
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
                     {{1.0f, 0.0f, 0.0f, 10 * kSecondToNano},
                      {1.0f, 0.0f, 0.0f, 12 * kSecondToNano}});

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
      MakePointCloud("lidar", 11.0, {{0.0f, 0.0f, 0.0f, 11 * kSecondToNano}});

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
                     {{1.0f, 0.0f, 0.0f, 10 * kSecondToNano},
                      {1.0f, 0.0f, 0.0f, 12 * kSecondToNano}});

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
    "map", "lidar", cyber::Time(11.0),
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  tf_buffer.AddTransform(
    "map", "lidar", cyber::Time(12.0),
      Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  GpuLidarDeskewPolicy policy;
  auto config = MakeConfig();
  config.set_gpu_device_id(0);
  ASSERT_TRUE(policy.Init(config, &tf_buffer));

  SensorFrameContext frame_context;
  frame_context.sensor_id = "lidar";
  frame_context.point_cloud =
      MakePointCloud("lidar", 12.0,
                     {{0.0f, 0.0f, 0.0f, 10 * kSecondToNano},
                      {0.0f, 0.0f, 0.0f, 12 * kSecondToNano}});

  std::vector<double> sample_times;
  std::vector<Eigen::Affine3d> poses;
  ASSERT_TRUE(policy.ComputeMotionCompensationPoses(frame_context,
                                                    &sample_times, &poses));
  ASSERT_EQ(sample_times.size(), 3U);
  ASSERT_EQ(poses.size(), 3U);
  EXPECT_DOUBLE_EQ(sample_times.front(), 10.0);
  EXPECT_DOUBLE_EQ(sample_times.back(), 12.0);
  EXPECT_DOUBLE_EQ(poses[0].translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(poses[2].translation().x(), 2.0);
}
#endif

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
