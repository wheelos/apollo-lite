#include "modules/dreamview/backend/simulator/entity/ego/ego_model_base.h"

namespace apollo {
namespace dreamview {

EgoModel::EgoModel(std::shared_ptr<cyber::Node> node, MapService* map_service)
    : node_(node), map_service_(map_service) {
  localization_writer_ =
      node_->CreateWriter<apollo::localization::LocalizationEstimate>(
          "/apollo/localization/pose");
  chassis_writer_ =
      node_->CreateWriter<apollo::canbus::Chassis>("/apollo/canbus/chassis");
  routing_writer_ = node_->CreateWriter<apollo::routing::RoutingRequest>(
      "/apollo/routing_request");
}

}  // namespace dreamview
}  // namespace apollo
