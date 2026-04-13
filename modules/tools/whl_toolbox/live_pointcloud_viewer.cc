#include <algorithm>
#include <chrono>
#include <array>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "CivetServer.h"
#include "gflags/gflags.h"
#include "nlohmann/json.hpp"

#include "Eigen/Geometry"
#include "cyber/cyber.h"
#include "cyber/common/log.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"
#include "modules/transform/transform_query.h"
#include "modules/transform/buffer.h"

DEFINE_string(channel, "", "PointCloud channel to subscribe to.");
DEFINE_int32(port, 8891, "CivetWeb listening port for live pointcloud viewer.");
DEFINE_int32(max_points, 30000, "Maximum points to send per frame.");
DEFINE_string(bind_address, "0.0.0.0", "Bind address for the CivetWeb server.");
DEFINE_string(ready_file, "", "Optional JSON file written when the server is ready.");
DEFINE_string(imu_frame_id, "",
              "Optional target IMU frame. When set, point clouds are transformed"
              " into this frame using static transforms from tf buffer.");

namespace apollo {
namespace whl_toolbox {

using Json = nlohmann::json;
using apollo::dreamview::WebSocketHandler;
using apollo::drivers::PointCloud;
using ::apollo::perception::PerceptionObstacle;
using ::apollo::perception::PerceptionObstacles;

constexpr char kPerceptionObstacleChannel[] = "/apollo/perception/obstacles";
constexpr char kWorldFrame[] = "world";


const char *ObstacleTypeName(PerceptionObstacle::Type type) {
  switch (type) {
    case PerceptionObstacle::PEDESTRIAN:
      return "PEDESTRIAN";
    case PerceptionObstacle::BICYCLE:
      return "BICYCLE";
    case PerceptionObstacle::VEHICLE:
      return "VEHICLE";
    case PerceptionObstacle::UNKNOWN_MOVABLE:
      return "UNKNOWN_MOVABLE";
    case PerceptionObstacle::UNKNOWN_UNMOVABLE:
      return "UNKNOWN_UNMOVABLE";
    case PerceptionObstacle::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}


class HealthHandler : public CivetHandler {
 public:
  explicit HealthHandler(std::function<std::string()> payload_builder)
      : payload_builder_(std::move(payload_builder)) {}

  bool handleGet(CivetServer *server, struct mg_connection *conn) override {
    (void)server;
    const std::string payload = payload_builder_();
    mg_printf(conn,
              "HTTP/1.1 200 OK\r\n"
              "Content-Type: application/json\r\n"
              "Cache-Control: no-store\r\n"
              "Content-Length: %zu\r\n"
              "\r\n",
              payload.size());
    mg_write(conn, payload.data(), payload.size());
    return true;
  }

 private:
  std::function<std::string()> payload_builder_;
};

class LivePointCloudViewer {
 public:
  LivePointCloudViewer(std::string channel, int port, int max_points)
      : channel_(std::move(channel)),
        port_(port),
        max_points_(max_points),
        target_frame_(FLAGS_imu_frame_id) {}

  bool Start() {
    const std::string listening_ports = FLAGS_bind_address.empty()
                                            ? std::to_string(port_)
                                            : FLAGS_bind_address + ":" + std::to_string(port_);
    std::vector<std::string> options = {
        "listening_ports", listening_ports,
        "websocket_timeout_ms", "10000",
        "request_timeout_ms", "10000",
        "enable_keep_alive", "yes",
        "tcp_nodelay", "1",
        "num_threads", "4",
    };
    server_.reset(new CivetServer(options));
    websocket_.reset(new WebSocketHandler("LivePointCloud"));
    websocket_->RegisterConnectionReadyHandler(
        [this](WebSocketHandler::Connection *conn) {
          websocket_->SendData(conn, BuildStatusJson().dump());
        });
    server_->addWebSocketHandler("/ws", *websocket_);
    health_handler_.reset(
        new HealthHandler([this]() { return BuildStatusJson().dump(); }));
    server_->addHandler("/health", *health_handler_);

    node_ = apollo::cyber::CreateNode("live_pointcloud_viewer");
    if (node_ == nullptr) {
      AERROR << "failed to create cyber node";
      return false;
    }
    reader_ = node_->CreateReader<PointCloud>(
        channel_, [this](const std::shared_ptr<PointCloud> &message) {
          this->OnPointCloud(message);
        });
    if (reader_ == nullptr) {
      AERROR << "failed to create pointcloud reader for " << channel_;
      return false;
    }
    obstacle_reader_ = node_->CreateReader<PerceptionObstacles>(
        kPerceptionObstacleChannel,
        [this](const std::shared_ptr<PerceptionObstacles> &message) {
          this->OnPerceptionObstacles(message);
        });
    if (obstacle_reader_ == nullptr) {
      AERROR << "failed to subscribe perception obstacle channel "
             << kPerceptionObstacleChannel;
      return false;
    }
    if (target_frame_.empty()) {
      MarkReady();
    }
    AINFO << "live pointcloud viewer is listening on " << FLAGS_bind_address
          << ":" << port_ << ", channel=" << channel_;
    return true;
  }

  void Stop() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (server_ != nullptr) {
      server_->close();
      server_.reset();
    }
  }

  void WriteReadyFile(const std::string &path) {
    if (path.empty()) {
      return;
    }
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
      AERROR << "failed to open ready file " << path;
      return;
    }
    output << BuildStatusJson().dump(2) << std::endl;
  }

  bool WaitUntilReady(double timeout_sec, std::string *error) {
    std::unique_lock<std::mutex> lock(ready_mutex_);
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0));
    while (!ready_ && !startup_failed_) {
      if (ready_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
        break;
      }
    }
    if (ready_) {
      return true;
    }
    if (startup_failed_) {
      if (error != nullptr) {
        *error = startup_error_;
      }
      return false;
    }
    if (error != nullptr) {
      *error =
          target_frame_.empty()
              ? "backend did not become ready in time"
              : "timed out waiting for the first pointcloud frame to resolve "
                "the IMU static transform";
    }
    return false;
  }

 private:
  void OnPointCloud(const std::shared_ptr<PointCloud> &message) {
    if (message == nullptr) {
      return;
    }
    const std::string source_frame = ResolveSourceFrame(*message);
    if (!target_frame_.empty()) {
      std::string error;
      if (!EnsureTransform(source_frame, &error)) {
        FailStartup(error);
        AERROR << error;
        return;
      }
      MarkReady();
    }
    const std::string payload = BuildBinaryPayload(*message);
    if (!payload.empty()) {
      websocket_->BroadcastBinaryData(payload, true);
    }
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      ++frame_count_;
      last_timestamp_sec_ = ResolveTimestamp(*message);
      last_point_count_ = message->point_size();
      source_frame_ = source_frame;
    }
  }

  void OnPerceptionObstacles(
      const std::shared_ptr<PerceptionObstacles> &message) {
    if (message == nullptr) {
      return;
    }

    std::string display_frame;
    double display_timestamp_sec = 0.0;
    bool waiting_for_source_frame = false;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      display_frame = target_frame_.empty() ? source_frame_ : target_frame_;
      display_timestamp_sec = last_timestamp_sec_;
      if (display_frame.empty()) {
        obstacle_overlay_error_ =
            "waiting for pointcloud frame before rendering perception boxes";
        waiting_for_source_frame = true;
      }
    }
    if (waiting_for_source_frame) {
      websocket_->BroadcastData(BuildStatusJson().dump(), true);
      return;
    }

    const double obstacle_timestamp_sec = ResolveObstacleTimestamp(*message);
    const double timestamp_sec =
        display_timestamp_sec > 0.0 ? display_timestamp_sec : obstacle_timestamp_sec;
    std::string error;
    Eigen::Affine3d world_to_display = Eigen::Affine3d::Identity();
    if (!LookupWorldToDisplayPose(display_frame, timestamp_sec, &world_to_display,
                                  &error)) {
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        obstacle_overlay_error_ = error;
      }
      websocket_->BroadcastData(BuildStatusJson().dump(), true);
      return;
    }

    Json payload;
    payload["type"] = "obstacles";
    payload["channel"] = kPerceptionObstacleChannel;
    payload["frame"] = display_frame;
    payload["timestamp_sec"] = timestamp_sec;
    payload["obstacles"] = Json::array();

    for (const auto &obstacle : message->perception_obstacle()) {
      const Json obstacle_json =
          BuildObstacleJson(obstacle, world_to_display);
      if (!obstacle_json.is_null()) {
        payload["obstacles"].push_back(obstacle_json);
      }
    }

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_obstacle_count_ =
          static_cast<std::uint32_t>(payload["obstacles"].size());
      last_obstacle_timestamp_sec_ = timestamp_sec;
      obstacle_display_frame_ = display_frame;
      obstacle_overlay_error_.clear();
    }
    websocket_->BroadcastData(payload.dump(), true);
  }

  double ResolveObstacleTimestamp(const PerceptionObstacles &message) const {
    if (message.has_header() && message.header().has_timestamp_sec()) {
      return message.header().timestamp_sec();
    }
    if (message.perception_obstacle_size() > 0 &&
        message.perception_obstacle(0).has_timestamp()) {
      return message.perception_obstacle(0).timestamp();
    }
    return 0.0;
  }

  double ResolveTimestamp(const PointCloud &message) const {
    if (message.header().has_timestamp_sec()) {
      return message.header().timestamp_sec();
    }
    return message.measurement_time();
  }

  Json BuildStatusJson() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    Json payload;
    payload["type"] = "status";
    payload["channel"] = channel_;
    payload["port"] = port_;
    payload["max_points"] = max_points_;
    payload["frame_count"] = frame_count_;
    payload["last_timestamp_sec"] = last_timestamp_sec_;
    payload["last_point_count"] = last_point_count_;
    payload["source_frame"] = source_frame_;
    payload["target_frame"] = target_frame_;
    payload["transform_enabled"] = !target_frame_.empty();
    payload["perception_obstacle_channel"] = kPerceptionObstacleChannel;
    payload["last_obstacle_count"] = last_obstacle_count_;
    payload["last_obstacle_timestamp_sec"] = last_obstacle_timestamp_sec_;
    payload["obstacle_display_frame"] = obstacle_display_frame_;
    payload["obstacle_overlay_error"] = obstacle_overlay_error_;
    return payload;
  }

  std::string BuildBinaryPayload(const PointCloud &message) const {
    const int total_points = message.point_size();
    if (total_points <= 0) {
      return "";
    }

    const int max_points = std::max(1, max_points_);
    const int stride =
        std::max(1, static_cast<int>(std::ceil(static_cast<double>(total_points) /
                                               static_cast<double>(max_points))));
    uint32_t selected_points = 0;
    for (int index = 0; index < total_points; index += stride) {
      ++selected_points;
    }

    const std::size_t header_bytes = sizeof(uint32_t) + sizeof(double);
    const std::size_t point_bytes =
        static_cast<std::size_t>(selected_points) * 3 * sizeof(float);
    std::string payload(header_bytes + point_bytes, '\0');

    char *cursor = payload.data();
    std::memcpy(cursor, &selected_points, sizeof(uint32_t));
    cursor += sizeof(uint32_t);

    const double timestamp_sec = ResolveTimestamp(message);
    std::memcpy(cursor, &timestamp_sec, sizeof(double));
    cursor += sizeof(double);

    Eigen::Affine3d source_to_target = Eigen::Affine3d::Identity();
    bool use_transform = false;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      source_to_target = source_to_target_pose_;
      use_transform = transform_ready_;
    }

    for (int index = 0; index < total_points; index += stride) {
      const auto &point = message.point(index);
      float xyz[3] = {point.x(), point.y(), point.z()};
      if (use_transform) {
        const Eigen::Vector3d transformed =
            source_to_target *
            Eigen::Vector3d(static_cast<double>(point.x()),
                            static_cast<double>(point.y()),
                            static_cast<double>(point.z()));
        xyz[0] = static_cast<float>(transformed.x());
        xyz[1] = static_cast<float>(transformed.y());
        xyz[2] = static_cast<float>(transformed.z());
      }
      std::memcpy(cursor, xyz, sizeof(xyz));
      cursor += sizeof(xyz);
    }
    return payload;
  }

  std::string ResolveSourceFrame(const PointCloud &message) const {
    if (!message.frame_id().empty()) {
      return message.frame_id();
    }
    if (message.has_header() && message.header().has_frame_id()) {
      return message.header().frame_id();
    }
    return "";
  }

  bool EnsureTransform(const std::string &source_frame, std::string *error) {
    if (source_frame.empty()) {
      if (error != nullptr) {
        *error = "pointcloud frame_id is empty, cannot resolve IMU transform";
      }
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      if (transform_ready_ && source_frame == source_frame_) {
        return true;
      }
    }

    std::string err;
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    if (!transform_query_.LookupTransformToAffine(
            target_frame_, source_frame, apollo::cyber::Time(0), &pose, 1.0f,
            &err)) {
      if (error != nullptr) {
        *error = std::string("failed to resolve transform from ") +
                 source_frame + " to " + target_frame_ + ": " + err;
      }
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      source_to_target_pose_ = pose;
      source_frame_ = source_frame;
      transform_ready_ = true;
    }
    return true;
  }

  bool LookupWorldToDisplayPose(const std::string &display_frame,
                                double timestamp_sec,
                                Eigen::Affine3d *world_to_display,
                                std::string *error) const {
    auto *buffer = ::apollo::transform::Buffer::Instance();
    const ::apollo::cyber::Time query_time(timestamp_sec);
    std::string err;
    if (!buffer->canTransform(display_frame, kWorldFrame, query_time, 0.2f,
                              &err)) {
      if (error != nullptr) {
        *error = std::string("failed to resolve transform from world to ") +
                 display_frame + ": " + err;
      }
      return false;
    }

    ::apollo::transform::TransformStamped transform_stamped;
    try {
      transform_stamped =
          buffer->lookupTransform(display_frame, kWorldFrame, query_time, 0.2f);
    } catch (const std::exception &ex) {
      if (error != nullptr) {
        *error = std::string("failed to lookup transform from world to ") +
                 display_frame + ": " + ex.what();
      }
      return false;
    }

    const auto &tf = transform_stamped.transform();
    const Eigen::Quaterniond rotation(tf.rotation().qw(), tf.rotation().qx(),
                                      tf.rotation().qy(), tf.rotation().qz());
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.linear() = rotation.toRotationMatrix();
    pose.translation() << tf.translation().x(), tf.translation().y(),
        tf.translation().z();
    *world_to_display = pose;
    return true;
  }

  Eigen::Vector3d TransformPoint(const apollo::common::Point3D &point,
                                 const Eigen::Affine3d &world_to_display) const {
    return world_to_display *
           Eigen::Vector3d(point.x(), point.y(), point.z());
  }

  std::vector<Eigen::Vector3d> BuildBasePolygonWorld(
      const PerceptionObstacle &obstacle) const {
    std::vector<Eigen::Vector3d> polygon;
    polygon.reserve(static_cast<std::size_t>(obstacle.polygon_point_size()));
    for (const auto &point : obstacle.polygon_point()) {
      polygon.emplace_back(point.x(), point.y(), point.z());
    }
    if (!polygon.empty()) {
      return polygon;
    }

    if (!obstacle.has_position() || !obstacle.has_length() ||
        !obstacle.has_width()) {
      return {};
    }

    const double center_x = obstacle.position().x();
    const double center_y = obstacle.position().y();
    const double center_z = obstacle.position().z();
    const double half_length = obstacle.length() * 0.5;
    const double half_width = obstacle.width() * 0.5;
    const double yaw = obstacle.has_theta() ? obstacle.theta() : 0.0;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const std::array<std::pair<double, double>, 4> local_corners = {
        std::make_pair(half_length, half_width),
        std::make_pair(half_length, -half_width),
        std::make_pair(-half_length, -half_width),
        std::make_pair(-half_length, half_width),
    };
    polygon.reserve(local_corners.size());
    for (const auto &corner : local_corners) {
      const double x =
          center_x + corner.first * cos_yaw - corner.second * sin_yaw;
      const double y =
          center_y + corner.first * sin_yaw + corner.second * cos_yaw;
      polygon.emplace_back(x, y, center_z);
    }
    return polygon;
  }

  Json BuildObstacleJson(const PerceptionObstacle &obstacle,
                         const Eigen::Affine3d &world_to_display) const {
    std::vector<Eigen::Vector3d> base_polygon =
        BuildBasePolygonWorld(obstacle);
    if (base_polygon.size() < 2) {
      return Json();
    }

    const double height = obstacle.has_height()
                              ? std::max(0.1, obstacle.height())
                              : 1.0;
    Json base = Json::array();
    Json top = Json::array();
    for (const auto &point : base_polygon) {
      apollo::common::Point3D world_point;
      world_point.set_x(point.x());
      world_point.set_y(point.y());
      world_point.set_z(point.z());
      const Eigen::Vector3d transformed =
          TransformPoint(world_point, world_to_display);
      base.push_back(transformed.x());
      base.push_back(transformed.y());
      base.push_back(transformed.z());

      world_point.set_z(point.z() + height);
      const Eigen::Vector3d transformed_top =
          TransformPoint(world_point, world_to_display);
      top.push_back(transformed_top.x());
      top.push_back(transformed_top.y());
      top.push_back(transformed_top.z());
    }

    Json payload;
    payload["id"] = obstacle.id();
    payload["type"] = ObstacleTypeName(obstacle.type());
    payload["height"] = height;
    payload["base"] = std::move(base);
    payload["top"] = std::move(top);
    return payload;
  }

  void MarkReady() {
    std::lock_guard<std::mutex> lock(ready_mutex_);
    ready_ = true;
    ready_cv_.notify_all();
  }

  void FailStartup(const std::string &error) {
    std::lock_guard<std::mutex> lock(ready_mutex_);
    startup_failed_ = true;
    startup_error_ = error;
    ready_cv_.notify_all();
  }

  const std::string channel_;
  const int port_ = 0;
  const int max_points_ = 0;
  const std::string target_frame_;
  apollo::transform::TransformQuery transform_query_;

  mutable std::mutex status_mutex_;
  std::uint64_t frame_count_ = 0;
  std::uint32_t last_point_count_ = 0;
  double last_timestamp_sec_ = 0.0;
  std::string source_frame_;
  bool transform_ready_ = false;
  Eigen::Affine3d source_to_target_pose_ = Eigen::Affine3d::Identity();

  std::uint32_t last_obstacle_count_ = 0;
  double last_obstacle_timestamp_sec_ = 0.0;
  std::string obstacle_display_frame_;
  std::string obstacle_overlay_error_;

  std::mutex ready_mutex_;
  std::condition_variable ready_cv_;
  bool ready_ = false;
  bool startup_failed_ = false;
  std::string startup_error_;

  std::unique_ptr<CivetServer> server_;
  std::unique_ptr<WebSocketHandler> websocket_;
  std::unique_ptr<HealthHandler> health_handler_;
  std::shared_ptr<apollo::cyber::Node> node_;
  std::shared_ptr<apollo::cyber::Reader<PointCloud>> reader_;
  std::shared_ptr<::apollo::cyber::Reader<PerceptionObstacles>>
      obstacle_reader_;
};

}  // namespace whl_toolbox
}  // namespace apollo

int main(int argc, char **argv) {
  google::SetUsageMessage("live_pointcloud_viewer");
  google::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_channel.empty()) {
    std::cerr << "--channel is required" << std::endl;
    return 1;
  }
  apollo::cyber::Init(argv[0]);
  apollo::whl_toolbox::LivePointCloudViewer viewer(FLAGS_channel, FLAGS_port,
                                                   FLAGS_max_points);
  if (!viewer.Start()) {
    return 1;
  }
  std::string startup_error;
  if (!viewer.WaitUntilReady(10.0, &startup_error)) {
    std::cerr << startup_error << std::endl;
    return 1;
  }
  viewer.WriteReadyFile(FLAGS_ready_file);
  apollo::cyber::WaitForShutdown();
  viewer.Stop();
  return 0;
}
