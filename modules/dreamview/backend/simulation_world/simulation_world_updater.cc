/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/dreamview/backend/simulation_world/simulation_world_updater.h"

#include <cmath>
#include <memory>

#include "absl/strings/str_cat.h"
#include "google/protobuf/util/json_util.h"

#include "wheelos_msgs/mission_msgs/mission_request.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/perception_camera_updater/perception_camera_updater.h"
#include "modules/dreamview/backend/plugins/plugin_manager.h"
#include "modules/dreamview/backend/sim_control_manager/sim_control_manager.h"
#include "modules/dreamview/backend/simulation_world/simulation_world_service.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace dreamview {

SimulationWorldUpdater::~SimulationWorldUpdater() = default;

using apollo::common::monitor::MonitorMessageItem;
using apollo::common::util::ContainsKey;
using apollo::common::util::JsonUtil;
using apollo::cyber::common::GetProtoFromASCIIFile;
using apollo::cyber::common::SetProtoToASCIIFile;
using apollo::hdmap::DefaultRoutingFile;
using apollo::hdmap::EndWayPointFile;
using apollo::hdmap::ParkGoRoutingFile;
using apollo::relative_map::NavigationInfo;
using apollo::routing::LaneWaypoint;
using apollo::routing::ParkingSpaceType;
using apollo::routing::RoutingRequest;

using Json = nlohmann::json;
using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::MessageToJsonString;

namespace {

bool NeedsParkingInfoSupplement(const apollo::routing::RoutingRequest& request) {
  if (!request.has_parking_info() ||
      !request.parking_info().has_parking_space_id()) {
    return false;
  }
  const auto& parking_info = request.parking_info();
  return !parking_info.has_parking_point() ||
         !parking_info.has_parking_space_type() ||
         parking_info.corner_point().point_size() < 4;
}

bool SupplementParkingRoutingRequest(apollo::routing::RoutingRequest* request) {
  if (!NeedsParkingInfoSupplement(*request)) {
    return true;
  }
  auto* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    AERROR << "Failed to supplement parking routing request: hdmap unavailable.";
    return false;
  }
  const auto parking_space_id =
      hdmap::MakeMapId(request->parking_info().parking_space_id());
  const auto parking_space_info = hdmap->GetParkingSpaceById(parking_space_id);
  if (parking_space_info == nullptr) {
    AERROR << "Failed to supplement parking routing request for parking space "
           << request->parking_info().parking_space_id();
    return false;
  }
  const auto& points = parking_space_info->polygon().points();
  if (points.size() < 4) {
    AERROR << "Failed to supplement parking routing request for parking space "
           << request->parking_info().parking_space_id()
           << ": polygon corner count is " << points.size();
    return false;
  }

  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto& point : points) {
    center_x += point.x();
    center_y += point.y();
  }
  center_x /= static_cast<double>(points.size());
  center_y /= static_cast<double>(points.size());

  apollo::common::PointENU center_enu;
  center_enu.set_x(center_x);
  center_enu.set_y(center_y);
  apollo::hdmap::LaneInfoConstPtr nearest_lane;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  if (0 != hdmap->GetNearestLane(center_enu, &nearest_lane, &nearest_s,
                                 &nearest_l) ||
      nearest_lane == nullptr) {
    AERROR << "Failed to supplement parking routing request for parking space "
           << request->parking_info().parking_space_id()
           << ": cannot find nearest lane.";
    return false;
  }

  auto* request_parking_info = request->mutable_parking_info();
  request_parking_info->mutable_parking_point()->set_x(center_x);
  request_parking_info->mutable_parking_point()->set_y(center_y);
  request_parking_info->mutable_parking_point()->set_z(0.0);
  request_parking_info->clear_corner_point();
  for (size_t i = 0; i < 4; ++i) {
    auto* corner = request_parking_info->mutable_corner_point()->add_point();
    corner->set_x(points[i].x());
    corner->set_y(points[i].y());
  }

  const double lane_heading = nearest_lane->Heading(nearest_s);
  const double parking_heading = parking_space_info->parking_space().heading();
  const double diff_angle = std::atan2(std::sin(lane_heading - parking_heading),
                                       std::cos(lane_heading - parking_heading));
  request_parking_info->set_parking_space_type(
      std::fabs(diff_angle) < M_PI / 3.0 ? ParkingSpaceType::PARALLEL_PARKING
                                         : ParkingSpaceType::VERTICAL_PLOT);

  if (request->waypoint_size() == 0) {
    AERROR << "Failed to supplement parking routing request for parking space "
           << request->parking_info().parking_space_id()
           << ": request has no waypoint.";
    return false;
  }
  const auto last_waypoint = request->waypoint(request->waypoint_size() - 1);
  static constexpr double kExtendParkingLength = 20.0;
  apollo::common::PointENU extend_point;
  extend_point.set_x(last_waypoint.pose().x() +
                     kExtendParkingLength * std::cos(lane_heading));
  extend_point.set_y(last_waypoint.pose().y() +
                     kExtendParkingLength * std::sin(lane_heading));
  if (0 != hdmap->GetNearestLaneWithHeading(extend_point, 20.0, lane_heading,
                                            M_PI_2, &nearest_lane, &nearest_s,
                                            &nearest_l) ||
      nearest_lane == nullptr) {
    AERROR << "Failed to extend parking routing request for parking space "
           << request->parking_info().parking_space_id()
           << ": cannot project extended waypoint to lane.";
    return false;
  }
  extend_point = nearest_lane->GetSmoothPoint(nearest_s);
  auto* extend_waypoint = request->add_waypoint();
  extend_waypoint->mutable_pose()->set_x(extend_point.x());
  extend_waypoint->mutable_pose()->set_y(extend_point.y());
  extend_waypoint->set_id(nearest_lane->id().id());
  extend_waypoint->set_s(nearest_s);
  return true;
}

}  // namespace

SimulationWorldUpdater::SimulationWorldUpdater(
    WebSocketHandler *websocket, WebSocketHandler *map_ws,
    WebSocketHandler *point_cloud_ws,
    WebSocketHandler *camera_ws, SimControlManager *sim_control_manager,
    WebSocketHandler *plugin_ws, const MapService *map_service,
    PerceptionCameraUpdater *perception_camera_updater,
    PluginManager *plugin_manager, bool routing_from_file)
    : sim_world_service_(
          std::make_unique<SimulationWorldService>(map_service,
                                                   routing_from_file)),
      map_service_(map_service),
      websocket_(websocket),
      map_ws_(map_ws),
      point_cloud_ws_(point_cloud_ws),
      camera_ws_(camera_ws),
      plugin_ws_(plugin_ws),
      sim_control_manager_(sim_control_manager),
      perception_camera_updater_(perception_camera_updater),
      plugin_manager_(plugin_manager) {
  RegisterMessageHandlers();
}

void SimulationWorldUpdater::RegisterMessageHandlers() {
  // Send current sim_control status to the new client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "SimControlStatus";
        response["enabled"] = sim_control_manager_->IsEnabled();
        websocket_->SendData(conn, response.dump());
      });

  map_ws_->RegisterMessageHandler(
      "RetrieveMapData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto iter = json.find("elements");
        if (iter != json.end()) {
          MapElementIds map_element_ids;
          if (JsonStringToMessage(iter->dump(), &map_element_ids).ok()) {
            auto retrieved = map_service_->RetrieveMapElements(map_element_ids);

            std::string retrieved_map_string;
            retrieved.SerializeToString(&retrieved_map_string);

            map_ws_->SendBinaryData(conn, retrieved_map_string, true);
          } else {
            AERROR << "Failed to parse MapElementIds from json";
          }
        }
      });

  map_ws_->RegisterMessageHandler(
      "RetrieveRelativeMapData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::string to_send;
        {
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          to_send = relative_map_string_;
        }
        map_ws_->SendBinaryData(conn, to_send, true);
      });

  websocket_->RegisterMessageHandler(
      "Binary",
      [this](const std::string &data, WebSocketHandler::Connection *conn) {
        // Navigation info in binary format
        auto navigation_info = std::make_shared<NavigationInfo>();
        if (navigation_info->ParseFromString(data)) {
          sim_world_service_->PublishNavigationInfo(navigation_info);
        } else {
          AERROR << "Failed to parse navigation info from string. String size: "
                 << data.size();
        }
      });

  websocket_->RegisterMessageHandler(
      "RetrieveMapElementIdsByRadius",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto radius = json.find("radius");
        if (radius == json.end()) {
          AERROR << "Cannot retrieve map elements with unknown radius.";
          return;
        }

        if (!radius->is_number()) {
          AERROR << "Expect radius with type 'number', but was "
                 << radius->type_name();
          return;
        }

        Json response;
        response["type"] = "MapElementIds";
        response["mapRadius"] = *radius;

        MapElementIds ids;
        sim_world_service_->GetMapElementIds(*radius, &ids);
        std::string elementIds;
        MessageToJsonString(ids, &elementIds);
        response["mapElementIds"] = Json::parse(elementIds);

        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "CheckRoutingPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response = CheckRoutingPoint(json);
        response["type"] = "RoutingPointCheckResult";
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "SendRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto routing_request = std::make_shared<RoutingRequest>();

        bool succeed = ConstructRoutingRequest(json, routing_request.get());
        if (succeed) {
          sim_world_service_->PublishRoutingRequest(routing_request);
          sim_world_service_->PublishMonitorMessage(MonitorMessageItem::INFO,
                                                   "Routing request sent.");
        } else {
          sim_world_service_->PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to send a routing request.");
        }
      });

  websocket_->RegisterMessageHandler(
      "SendMissionRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto mission_request =
            std::make_shared<apollo::mission::MissionRequest>();

        // Parse common fields
        if (ContainsKey(json, "mission_id")) {
          mission_request->set_mission_id(json["mission_id"]);
        }
        if (ContainsKey(json, "task_name")) {
          mission_request->set_task_name(json["task_name"]);
        }
        if (ContainsKey(json, "enable_loop")) {
          mission_request->set_enable_loop(json["enable_loop"]);
        }

        // Analyzing key points (Waypoints)
        if (ContainsKey(json, "waypoints")) {
          for (const auto &wp_json : json["waypoints"]) {
            auto *wp = mission_request->add_waypoints();
            wp->set_name(wp_json["name"]);
            wp->mutable_pose()->set_x(wp_json["x"]);
            wp->mutable_pose()->set_y(wp_json["y"]);
          }
        }

        sim_world_service_->PublishMissionRequest(mission_request);
      });

  websocket_->RegisterMessageHandler(
      "SendDefaultCycleRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto routing_request = std::make_shared<RoutingRequest>();
        if (ContainsKey(json, "cycleNumber") &&
            json.find("cycleNumber")->is_number()) {
          AINFO << "Ignoring cycleNumber="
                << json.find("cycleNumber")->get<int>()
                << " because task_manager was removed.";
        }
        bool succeed = ConstructRoutingRequest(json, routing_request.get());
        if (succeed) {
          sim_world_service_->PublishRoutingRequest(routing_request);
          AINFO << "Direct cycle routing request sent without task_manager:\n"
                << routing_request->DebugString();
          sim_world_service_->PublishMonitorMessage(
              MonitorMessageItem::WARN,
              "Default cycle routing request sent once. task_manager was "
              "removed, so automatic repeated cycling is disabled.");
        } else {
          sim_world_service_->PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a default cycle routing request.");
        }
      });

  websocket_->RegisterMessageHandler(
      "SendParkGoRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (ContainsKey(json, "parkTime") &&
            json.find("parkTime")->is_number()) {
          AINFO << "Ignoring parkTime=" << json.find("parkTime")->get<double>()
                << " because task_manager was removed.";
        }
        auto routing_request = std::make_shared<RoutingRequest>();
        bool succeed = ConstructRoutingRequest(json, routing_request.get());
        if (succeed) {
          sim_world_service_->PublishRoutingRequest(routing_request);
          sim_world_service_->PublishMonitorMessage(
              MonitorMessageItem::WARN,
              "Park-go routing task_manager flow was removed. Sent a direct "
              "routing request only, without park-time automation.");
        } else {
          sim_world_service_->PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a park go routing request.");
        }
      });

  websocket_->RegisterMessageHandler(
      "SendParkingRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto routing_request = std::make_shared<RoutingRequest>();
        bool succeed = ConstructRoutingRequest(json, routing_request.get());
        if (succeed) {
          sim_world_service_->PublishRoutingRequest(routing_request);
          AINFO << "Parking routing request sent directly:\n"
                << routing_request->DebugString();
          sim_world_service_->PublishMonitorMessage(
              MonitorMessageItem::INFO, "Parking routing request sent.");
        } else {
          sim_world_service_->PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to send a parking routing request.");
        }
      });

  websocket_->RegisterMessageHandler(
      "RequestSimulationWorld",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!sim_world_service_->ReadyToPush()) {
          AWARN_EVERY(100)
              << "Not sending simulation world as the data is not ready!";
          return;
        }

        bool enable_pnc_monitor = false;
        auto planning = json.find("planning");
        if (planning != json.end() && planning->is_boolean()) {
          enable_pnc_monitor = json["planning"];
        }
        std::string to_send;
        {
          // Pay the price to copy the data instead of sending data over the
          // wire while holding the lock.
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          to_send = enable_pnc_monitor ? simulation_world_with_planning_data_
                                       : simulation_world_;
        }
        if (FLAGS_enable_update_size_check && !enable_pnc_monitor &&
            to_send.size() > FLAGS_max_update_size) {
          AWARN << "update size is too big:" << to_send.size();
          return;
        }
        websocket_->SendBinaryData(conn, to_send, true);
      });

  websocket_->RegisterMessageHandler(
      "RequestRoutePath",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response = sim_world_service_->GetRoutePathAsJson();
        response["type"] = "RoutePath";
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetDefaultEndPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "DefaultEndPoint";

        Json poi_list = Json::array();
        if (LoadPOI()) {
          for (const auto &landmark : poi_.landmark()) {
            Json place;
            place["name"] = landmark.name();

            Json parking_info =
                apollo::common::util::JsonUtil::ProtoToTypedJson(
                    "parkingInfo", landmark.parking_info());
            place["parkingInfo"] = parking_info["data"];

            Json waypoint_list;
            for (const auto &waypoint : landmark.waypoint()) {
              waypoint_list.push_back(GetPointJsonFromLaneWaypoint(waypoint));
            }
            place["waypoint"] = waypoint_list;

            poi_list.push_back(place);
          }
        } else {
          sim_world_service_->PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   "Failed to load default "
                                                   "POI. Please make sure the "
                                                   "file exists at " +
                                                       EndWayPointFile());
        }
        response["poi"] = poi_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetDefaultRoutings",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "DefaultRoutings";
        response["threshold"] =
            FLAGS_loop_routing_end_to_start_distance_threshold;

        Json default_routing_list = Json::array();
        if (LoadUserDefinedRoutings(DefaultRoutingFile(), &default_routings_)) {
          for (const auto &landmark : default_routings_.landmark()) {
            Json drouting;
            drouting["name"] = landmark.name();

            Json point_list;
            for (const auto &point : landmark.waypoint()) {
              point_list.push_back(GetPointJsonFromLaneWaypoint(point));
            }
            drouting["point"] = point_list;
            default_routing_list.push_back(drouting);
          }
        } else {
          sim_world_service_->PublishMonitorMessage(
              MonitorMessageItem::ERROR,
              "Failed to load default "
              "routing. Please make sure the "
              "file exists at " +
                  DefaultRoutingFile());
        }
        response["defaultRoutings"] = default_routing_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetParkAndGoRoutings",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "ParkAndGoRoutings";
        Json park_go_routing_list = Json::array();
        if (LoadUserDefinedRoutings(ParkGoRoutingFile(), &park_go_routings_)) {
          for (const auto &landmark : park_go_routings_.landmark()) {
            Json park_go_routing;
            park_go_routing["name"] = landmark.name();

            Json point_list;
            for (const auto &point : landmark.waypoint()) {
              point_list.push_back(GetPointJsonFromLaneWaypoint(point));
            }
            park_go_routing["point"] = point_list;
            park_go_routing_list.push_back(park_go_routing);
          }
          //  } else {
          //   sim_world_service_->PublishMonitorMessage(
          //       MonitorMessageItem::ERROR,
          //       "Failed to load park go "
          //       "routing. Please make sure the "
          //       "file exists at " +
          //           ParkGoRoutingFile());
        }
        response["parkAndGoRoutings"] = park_go_routing_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "Reset", [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_->SetToClear();
        sim_control_manager_->Reset();
      });

  websocket_->RegisterMessageHandler(
      "Dump", [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_->DumpMessages();
      });

  websocket_->RegisterMessageHandler(
      "ToggleSimControl",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto enable = json.find("enable");
        if (enable != json.end() && enable->is_boolean()) {
          if (*enable) {
            sim_control_manager_->Start();
          } else {
            sim_control_manager_->Stop();
          }
        }
      });

  websocket_->RegisterMessageHandler(
      "GetParkingRoutingDistance",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "ParkingRoutingDistance";
        response["threshold"] = FLAGS_parking_routing_distance_threshold;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "SaveDefaultRouting",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        bool succeed = AddDefaultRouting(json);
        if (succeed) {
          sim_world_service_->PublishMonitorMessage(
              MonitorMessageItem::INFO, "Successfully add a routing.");
          if (!default_routing_) {
            AERROR << "Failed to add a routing" << std::endl;
          }
          Json response = JsonUtil::ProtoToTypedJson("AddDefaultRoutingPath",
                                                     *default_routing_);
          response["routingType"] = json["routingType"];
          websocket_->SendData(conn, response.dump());
        } else {
          sim_world_service_->PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   "Failed to add a routing.");
        }
      });

  camera_ws_->RegisterMessageHandler(
      "RequestCameraData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!perception_camera_updater_->IsEnabled()) {
          return;
        }
        std::string to_send;
        perception_camera_updater_->GetUpdate(&to_send);
        camera_ws_->SendBinaryData(conn, to_send, true);
      });

  camera_ws_->RegisterMessageHandler(
      "GetCameraChannel",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        std::vector<std::string> channels;
        perception_camera_updater_->GetChannelMsg(&channels);
        Json response({});
        response["data"]["name"] = "GetCameraChannelListSuccess";
        for (unsigned int i = 0; i < channels.size(); i++) {
          response["data"]["info"]["channel"][i] = channels[i];
        }
        camera_ws_->SendData(conn, response.dump());
      });
  camera_ws_->RegisterMessageHandler(
      "ChangeCameraChannel",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto channel_info = json.find("data");
        Json response({});
        if (channel_info == json.end()) {
          AERROR << "Cannot  retrieve channel info with unknown channel.";
          response["type"] = "ChangeCameraChannelFail";
          camera_ws_->SendData(conn, response.dump());
          return;
        }
        std::string channel =
            channel_info->dump().substr(1, channel_info->dump().length() - 2);
        if (perception_camera_updater_->ChangeChannel(channel)) {
          Json response({});
          response["type"] = "ChangeCameraChannelSuccess";
          camera_ws_->SendData(conn, response.dump());
        } else {
          response["type"] = "ChangeCameraChannelFail";
          camera_ws_->SendData(conn, response.dump());
        }
      });
  plugin_ws_->RegisterMessageHandler(
      "PluginRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!plugin_manager_->IsEnabled()) {
          return;
        }
        auto iter = json.find("data");
        if (iter == json.end()) {
          AERROR << "Failed to get plugin msg!";
          return;
        }
        if (!plugin_manager_->SendMsgToPlugin(iter->dump())) {
          AERROR << "Failed to send msg to plugin";
        }
      });
}

Json SimulationWorldUpdater::CheckRoutingPoint(const Json &json) {
  Json result;
  if (!ContainsKey(json, "point")) {
    result["error"] = "Failed to check routing point: point not found.";
    AERROR << result["error"];
    return result;
  }
  auto point = json["point"];
  if (!ValidateCoordinate(point) || !ContainsKey(point, "id")) {
    result["error"] = "Failed to check routing point: invalid point.";
    AERROR << result["error"];
    return result;
  }
  // TODO(zero): In order to be compatible with the pure path point planning
  // algorithm, we remove the legality check of routing (currently mainly for
  // roads)
  //
  // if (!ContainsKey(point, "heading")) {
  //   if (!map_service_->CheckRoutingPoint(point["x"], point["y"])) {
  //     result["pointId"] = point["id"];
  //     result["error"] = "Selected point cannot be a routing point.";
  //     AWARN << result["error"];
  //   }
  // } else {
  //   if (!map_service_->CheckRoutingPointWithHeading(point["x"], point["y"],
  //                                                   point["heading"])) {
  //     result["pointId"] = point["id"];
  //     result["error"] = "Selected point cannot be a routing point.";
  //     AWARN << result["error"];
  //   }
  // }
  return result;
}

Json SimulationWorldUpdater::GetPointJsonFromLaneWaypoint(
    const apollo::routing::LaneWaypoint &waypoint) {
  Json point;
  point["x"] = waypoint.pose().x();
  point["y"] = waypoint.pose().y();
  if (waypoint.has_heading()) {
    point["heading"] = waypoint.heading();
  }
  return point;
}

bool SimulationWorldUpdater::ConstructLaneWayPoint(const Json &point,
                                                   LaneWaypoint *laneWayPoint,
                                                   std::string description) {
  if (ContainsKey(point, "heading")) {
    if (!map_service_->ConstructLaneWayPointWithHeading(
            point["x"], point["y"], point["heading"], laneWayPoint)) {
      AERROR << "Failed to prepare a routing request with heading: "
             << point["heading"] << " cannot locate " << description
             << " on map.";
      return false;
    }
  } else {
    if (!map_service_->ConstructLaneWayPoint(point["x"], point["y"],
                                             laneWayPoint)) {
      AERROR << "Failed to prepare a routing request:"
             << " cannot locate " << description << " on map.";
      return false;
    }
  }
  return true;
}

bool SimulationWorldUpdater::ConstructRoutingRequest(
    const Json &json, RoutingRequest *routing_request) {
  routing_request->clear_waypoint();
  // set start point
  if (!ContainsKey(json, "start")) {
    AERROR << "Failed to prepare a routing request: start point not found.";
    return false;
  }

  auto start = json["start"];
  if (!ValidateCoordinate(start)) {
    AERROR << "Failed to prepare a routing request: invalid start point.";
    return false;
  }
  if (!ConstructLaneWayPoint(start, routing_request->add_waypoint(),
                             "start point")) {
    return false;
  }

  // set way point(s) if any
  auto iter = json.find("waypoint");
  if (iter != json.end() && iter->is_array()) {
    auto *waypoint = routing_request->mutable_waypoint();
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      if (!ValidateCoordinate(point)) {
        AERROR << "Failed to prepare a routing request: invalid waypoint.";
        return false;
      }

      if (!ConstructLaneWayPoint(point, waypoint->Add(), "point")) {
        AERROR << "Failed to construct a LaneWayPoint, skipping.";
        waypoint->RemoveLast();
      }
    }
  }

  // set end point
  if (!ContainsKey(json, "end")) {
    AERROR << "Failed to prepare a routing request: end point not found.";
    return false;
  }

  auto end = json["end"];
  if (!ValidateCoordinate(end)) {
    AERROR << "Failed to prepare a routing request: invalid end point.";
    return false;
  }
  if (ContainsKey(end, "id")) {
    if (!map_service_->ConstructLaneWayPointWithLaneId(
            end["x"], end["y"], end["id"], routing_request->add_waypoint())) {
      AERROR << "Failed to prepare a routing request with lane id: "
             << end["id"] << " cannot locate end point on map.";
      return false;
    }
  } else {
    if (!ConstructLaneWayPoint(end, routing_request->add_waypoint(),
                               "end point")) {
      return false;
    }
  }

  // set parking info
  if (ContainsKey(json, "parkingInfo")) {
    auto *requested_parking_info = routing_request->mutable_parking_info();
    if (!JsonStringToMessage(json["parkingInfo"].dump(), requested_parking_info)
             .ok()) {
      AERROR << "Failed to prepare a routing request: invalid parking info."
             << json["parkingInfo"].dump();
      return false;
    }
  }
  if (!SupplementParkingRoutingRequest(routing_request)) {
    AERROR << "Failed to prepare a routing request: unable to supplement "
              "parking info.";
    return false;
  }

  AINFO << "Constructed RoutingRequest to be sent:\n"
        << routing_request->DebugString();

  return true;
}

Json SimulationWorldUpdater::GetConstructRoutingRequestJson(
    const nlohmann::json &start, const nlohmann::json &end) {
  Json result;
  result["start"] = start;
  result["end"] = end;
  return result;
}

bool SimulationWorldUpdater::ValidateCoordinate(const nlohmann::json &json) {
  if (!ContainsKey(json, "x") || !ContainsKey(json, "y")) {
    AERROR << "Failed to find x or y coordinate.";
    return false;
  }
  if (json.find("x")->is_number() && json.find("y")->is_number()) {
    return true;
  }
  AERROR << "Both x and y coordinate should be a number.";
  return false;
}

void SimulationWorldUpdater::Start() {
  timer_.reset(new cyber::Timer(
      kSimWorldTimeIntervalMs, [this]() { this->OnTimer(); }, false));
  timer_->Start();
}

void SimulationWorldUpdater::OnTimer() {
  const bool needs_sim_world = websocket_->HasConnections();
  const bool needs_map_data = map_ws_->HasConnections();
  const bool needs_adc_timestamp =
      point_cloud_ws_ != nullptr && point_cloud_ws_->HasConnections();

  if (!needs_sim_world && !needs_map_data && !needs_adc_timestamp) {
    if (cached_outputs_cleared_) {
      return;
    }
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    last_pushed_adc_timestamp_sec_.store(0.0, std::memory_order_relaxed);
    std::string().swap(simulation_world_);
    std::string().swap(simulation_world_with_planning_data_);
    std::string().swap(relative_map_string_);
    cached_outputs_cleared_ = true;
    return;
  }

  cached_outputs_cleared_ = false;
  sim_world_service_->Update();

  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    last_pushed_adc_timestamp_sec_.store(
        needs_adc_timestamp
            ? sim_world_service_->world().auto_driving_car().timestamp_sec()
            : 0.0,
        std::memory_order_relaxed);
    if (needs_sim_world) {
      sim_world_service_->GetWireFormatString(
          FLAGS_sim_map_radius, &simulation_world_,
          &simulation_world_with_planning_data_);
    } else {
      std::string().swap(simulation_world_);
      std::string().swap(simulation_world_with_planning_data_);
    }
    if (needs_map_data) {
      sim_world_service_->GetRelativeMap().SerializeToString(
          &relative_map_string_);
    } else {
      std::string().swap(relative_map_string_);
    }
  }
}

bool SimulationWorldUpdater::LoadPOI() {
  if (GetProtoFromASCIIFile(EndWayPointFile(), &poi_)) {
    return true;
  }

  AWARN << "Failed to load default list of POI from " << EndWayPointFile();
  return false;
}

bool SimulationWorldUpdater::LoadUserDefinedRoutings(
    const std::string &file_name, google::protobuf::Message *message) {
  if (GetProtoFromASCIIFile(file_name, message)) {
    return true;
  }

  AWARN << "Failed to load routings from " << file_name;
  return false;
}

bool SimulationWorldUpdater::AddDefaultRouting(const Json &json) {
  if (!ContainsKey(json, "name")) {
    AERROR << "Failed to save a routing: routing name not found.";
    return false;
  }

  if (!ContainsKey(json, "point")) {
    AERROR << "Failed to save a routing: routing points not "
              "found.";
    return false;
  }

  if (!ContainsKey(json, "routingType")) {
    AERROR << "Failed to save a routing: routing type not "
              "found.";
    return false;
  }

  std::string name = json["name"];
  auto iter = json.find("point");
  std::string routingType = json["routingType"];
  bool isDefaultRouting = (routingType == "defaultRouting");
  default_routing_ = isDefaultRouting ? default_routings_.add_landmark()
                                      : park_go_routings_.add_landmark();
  default_routing_->clear_name();
  default_routing_->clear_waypoint();
  default_routing_->set_name(name);
  auto *waypoint = default_routing_->mutable_waypoint();
  if (iter != json.end() && iter->is_array()) {
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      if (!ValidateCoordinate(point)) {
        AERROR << "Failed to save a routing: invalid waypoint.";
        return false;
      }
      auto *p = waypoint->Add();
      auto *pose = p->mutable_pose();
      pose->set_x(static_cast<double>(point["x"]));
      pose->set_y(static_cast<double>(point["y"]));
      if (ContainsKey(point, "heading")) {
        p->set_heading(point["heading"]);
      }
    }
  }
  AINFO << "User Defined Routing Points to be saved:\n";
  std::string file_name =
      isDefaultRouting ? DefaultRoutingFile() : ParkGoRoutingFile();
  if (!SetProtoToASCIIFile(
          isDefaultRouting ? default_routings_ : park_go_routings_,
          file_name)) {
    AERROR << "Failed to set proto to ascii file " << file_name;
    return false;
  }
  AINFO << "Success in setting proto to file" << file_name;

  return true;
}

}  // namespace dreamview
}  // namespace apollo
