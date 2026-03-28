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
  config.set_world_frame_id("world");
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
      "world", "lidar", cyber::Time(10.0),
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  tf_buffer.AddTransform(
      "world", "lidar", cyber::Time(11.0),
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  tf_buffer.AddTransform(
      "world", "lidar", cyber::Time(12.0),
      Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  CpuLidarDeskewPolicy policy;
  ASSERT_TRUE(policy.Init(MakeConfig(), &tf_buffer));

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
  EXPECT_DOUBLE_EQ(sample_times[0], 10.0);
  EXPECT_DOUBLE_EQ(sample_times[1], 11.0);
  EXPECT_DOUBLE_EQ(sample_times[2], 12.0);
  EXPECT_DOUBLE_EQ(poses[0].translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(poses[1].translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(poses[2].translation().x(), 2.0);
}

TEST(CpuLidarFusionPolicyTest, FusesPointsIntoReferenceBaseFrame) {
  MockBuffer tf_buffer;
  tf_buffer.AddTransform(
      "world", "base_link", cyber::Time(12.0),
      Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  CpuLidarFusionPolicy policy;
  ASSERT_TRUE(policy.Init(MakeConfig(), &tf_buffer));

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

  ASSERT_TRUE(policy.FuseToBaseLink(12.0, {frame_context}, poses, sample_times,
                                    &buffer));
  ASSERT_EQ(buffer.valid_count, 2U);
  EXPECT_FLOAT_EQ(storage[0].x(), 4.0f);
  EXPECT_FLOAT_EQ(storage[1].x(), 6.0f);
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
TEST(GpuLidarFusionPolicyTest, FusesPointsIntoReferenceBaseFrame) {
  MockBuffer tf_buffer;
  tf_buffer.AddTransform(
      "world", "base_link", cyber::Time(12.0),
      Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  GpuLidarFusionPolicy policy;
  auto config = MakeConfig();
  config.set_gpu_device_id(0);
  ASSERT_TRUE(policy.Init(config, &tf_buffer));

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

  ASSERT_TRUE(policy.FuseToBaseLink(12.0, {frame_context}, poses, sample_times,
                                    &buffer));
  ASSERT_EQ(buffer.valid_count, 2U);
  std::vector<float> xs{storage[0].x(), storage[1].x()};
  std::sort(xs.begin(), xs.end());
  EXPECT_FLOAT_EQ(xs[0], 4.0f);
  EXPECT_FLOAT_EQ(xs[1], 6.0f);
}

TEST(GpuLidarDeskewPolicyTest, ComputesSampledPosesFromPointTimestamps) {
  MockBuffer tf_buffer;
  tf_buffer.AddTransform(
      "world", "lidar", cyber::Time(10.0),
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  tf_buffer.AddTransform(
      "world", "lidar", cyber::Time(11.0),
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  tf_buffer.AddTransform(
      "world", "lidar", cyber::Time(12.0),
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
