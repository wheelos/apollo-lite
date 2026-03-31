#include <algorithm>
#include <chrono>
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
#include "modules/dreamview/backend/handlers/websocket_handler.h"
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

    auto *buffer = apollo::transform::Buffer::Instance();
    std::string err;
    if (!buffer->canTransform(target_frame_, source_frame, apollo::cyber::Time(0),
                              1.0f, &err)) {
      if (error != nullptr) {
        *error = std::string("failed to resolve static transform from ") +
                 source_frame + " to " + target_frame_ + ": " + err;
      }
      return false;
    }

    apollo::transform::TransformStamped transform;
    try {
      transform = buffer->lookupTransform(target_frame_, source_frame,
                                          apollo::cyber::Time(0), 1.0f);
    } catch (const std::exception &ex) {
      if (error != nullptr) {
        *error = std::string("failed to lookup transform from ") +
                 source_frame + " to " + target_frame_ + ": " + ex.what();
      }
      return false;
    }

    const Eigen::Quaterniond rotation(
        transform.transform().rotation().qw(),
        transform.transform().rotation().qx(),
        transform.transform().rotation().qy(),
        transform.transform().rotation().qz());
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.linear() = rotation.toRotationMatrix();
    pose.translation() << transform.transform().translation().x(),
        transform.transform().translation().y(),
        transform.transform().translation().z();

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      source_to_target_pose_ = pose;
      source_frame_ = source_frame;
      transform_ready_ = true;
    }
    return true;
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

  mutable std::mutex status_mutex_;
  std::uint64_t frame_count_ = 0;
  std::uint32_t last_point_count_ = 0;
  double last_timestamp_sec_ = 0.0;
  std::string source_frame_;
  bool transform_ready_ = false;
  Eigen::Affine3d source_to_target_pose_ = Eigen::Affine3d::Identity();

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
