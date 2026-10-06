#include "modules/control/controller/controller_profile_manager.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace control {
namespace {

using common::ErrorCode;
using common::Status;

Status Invalid(const std::string& reason) {
  AERROR << reason;
  return Status(ErrorCode::CONTROL_INIT_ERROR, reason);
}

bool HasController(const ControlConf& conf, ControlConf::ControllerType type) {
  return std::find(conf.active_controllers().begin(),
                   conf.active_controllers().end(),
                   type) != conf.active_controllers().end();
}

bool LatLon(const ControlConf& conf) {
  return conf.active_controllers_size() == 2 &&
         HasController(conf, ControlConf::LAT_CONTROLLER) &&
         HasController(conf, ControlConf::LON_CONTROLLER);
}

template <typename Weights>
bool ValidWeights(const Weights& weights, int size, bool strictly_positive) {
  if (weights.size() != size) {
    return false;
  }
  bool positive = false;
  for (double weight : weights) {
    if (!std::isfinite(weight) || weight < 0.0 ||
        (strictly_positive && weight == 0.0)) {
      return false;
    }
    positive = positive || weight > 0.0;
  }
  return positive;
}

bool ValidPid(const PidConf& pid) {
  if (!pid.has_kp() || !pid.has_ki() || !pid.has_kd() ||
      !std::isfinite(pid.kp()) || pid.kp() <= 0.0 || !std::isfinite(pid.ki()) ||
      pid.ki() < 0.0 || !std::isfinite(pid.kd()) || pid.kd() < 0.0) {
    return false;
  }
  for (double value : {pid.integrator_saturation_level(),
                       pid.output_saturation_level(), pid.kaw()}) {
    if (!std::isfinite(value) || value < 0.0) {
      return false;
    }
  }
  return !pid.integrator_enable() || (pid.has_integrator_saturation_level() &&
                                      pid.integrator_saturation_level() > 0.0);
}

Status Configure(const ControllerTuningProfile& tuning, const ControlConf& base,
                 ControlConf* output) {
  *output = base;
  output->clear_controller_profiles();
  if (!tuning.active_controllers().empty()) {
    output->mutable_active_controllers()->CopyFrom(tuning.active_controllers());
  }
  const auto validated = ControllerAgent::ValidateControllerSet(*output);
  if (!validated.ok()) {
    return Invalid(validated.error_message());
  }
  const bool lat_lon = LatLon(*output);
  const bool mpc = output->active_controllers_size() == 1 &&
                   HasController(*output, ControlConf::MPC_CONTROLLER);
  if (!lat_lon && !mpc) {
    return Invalid(
        "profiled banks require LAT/LON or exclusive MPC; "
        "other controller families remain legacy-only");
  }
  const bool lateral_tuned =
      tuning.lateral_q_size() != 0 || tuning.lateral_reverse_q_size() != 0;
  const bool pid_tuned =
      tuning.has_station_pid() || tuning.has_low_speed_pid() ||
      tuning.has_high_speed_pid() || tuning.has_reverse_station_pid() ||
      tuning.has_reverse_speed_pid();
  const bool mpc_tuned = tuning.mpc_q_size() != 0 || tuning.mpc_r_size() != 0;
  if ((!tuning.inherit_base_tuning() && !lateral_tuned && !pid_tuned &&
       !mpc_tuned) ||
      ((lateral_tuned || pid_tuned) && !lat_lon) || (mpc_tuned && !mpc)) {
    return Invalid(
        "profile must explicitly inherit tuning or provide "
        "overrides for its selected controller family");
  }
  if (tuning.has_max_entry_speed_mps() &&
      (!std::isfinite(tuning.max_entry_speed_mps()) ||
       tuning.max_entry_speed_mps() < 0.0)) {
    return Invalid("profile entry speed must be finite and nonnegative");
  }
  if (lat_lon) {
    if (!base.has_lat_controller_conf() || !base.has_lon_controller_conf()) {
      return Invalid(
          "LAT/LON profile requires platform controller configuration");
    }
    auto* lat = output->mutable_lat_controller_conf();
    if (lateral_tuned) {
      if (tuning.lateral_q_size() != 0) {
        lat->mutable_matrix_q()->CopyFrom(tuning.lateral_q());
      }
      if (tuning.lateral_reverse_q_size() != 0) {
        lat->mutable_reverse_matrix_q()->CopyFrom(tuning.lateral_reverse_q());
      }
    }
    if (lat->preview_window() < 0 || lat->matrix_q_size() < 4 ||
        lat->preview_window() != lat->matrix_q_size() - 4 ||
        !ValidWeights(lat->matrix_q(), lat->matrix_q_size(), false) ||
        !ValidWeights(lat->reverse_matrix_q(), lat->matrix_q_size(), false)) {
      return Invalid("invalid forward/reverse LQR profile weights");
    }
    auto* lon = output->mutable_lon_controller_conf();
    if (tuning.has_station_pid()) {
      lon->mutable_station_pid_conf()->MergeFrom(tuning.station_pid());
    }
    if (tuning.has_low_speed_pid()) {
      lon->mutable_low_speed_pid_conf()->MergeFrom(tuning.low_speed_pid());
    }
    if (tuning.has_high_speed_pid()) {
      lon->mutable_high_speed_pid_conf()->MergeFrom(tuning.high_speed_pid());
    }
    if (tuning.has_reverse_station_pid()) {
      lon->mutable_reverse_station_pid_conf()->MergeFrom(
          tuning.reverse_station_pid());
    }
    if (tuning.has_reverse_speed_pid()) {
      lon->mutable_reverse_speed_pid_conf()->MergeFrom(
          tuning.reverse_speed_pid());
    }
    if (!ValidPid(lon->station_pid_conf()) ||
        !ValidPid(lon->low_speed_pid_conf()) ||
        !ValidPid(lon->high_speed_pid_conf()) ||
        !ValidPid(lon->reverse_station_pid_conf()) ||
        !ValidPid(lon->reverse_speed_pid_conf())) {
      return Invalid("invalid effective PID profile gains or saturation");
    }
  } else {
    if (!base.has_mpc_controller_conf()) {
      return Invalid("MPC profile requires platform MPC configuration");
    }
    auto* config = output->mutable_mpc_controller_conf();
    if (tuning.mpc_q_size() != 0) {
      config->mutable_matrix_q()->CopyFrom(tuning.mpc_q());
    }
    if (tuning.mpc_r_size() != 0) {
      config->mutable_matrix_r()->CopyFrom(tuning.mpc_r());
    }
    if (!ValidWeights(config->matrix_q(), 6, false) ||
        (config->matrix_r_size() != 0 &&
         !ValidWeights(config->matrix_r(), 2, true))) {
      return Invalid("invalid MPC profile weight dimensions or values");
    }
  }
  return Status::OK();
}

bool SameOwner(const planning::MotionCommandIdentity& lhs,
               const planning::MotionCommandIdentity& rhs) {
  return lhs.producer_epoch() == rhs.producer_epoch() &&
         lhs.aggregate_id() == rhs.aggregate_id() &&
         lhs.command_id() == rhs.command_id();
}

}  // namespace

const char* ControllerProfileName(ControllerParameterProfile profile) {
  switch (profile) {
    case CONTROL_PROFILE_ROAD_TRACKING:
      return "road-tracking";
    case CONTROL_PROFILE_LOW_SPEED_PRECISION:
      return "low-speed-precision";
    case CONTROL_PROFILE_STANDSTILL_HOLD:
      return "standstill-hold";
    default:
      return "unsupported";
  }
}

Status ControllerProfileManager::BuildConfigurations(
    const ControlConf& base, double stopped_speed_mps,
    std::array<std::optional<ControlConf>, 3>* configurations) {
  if (configurations == nullptr) {
    return Invalid("profile configuration output is null");
  }
  const auto validated = ControllerAgent::ValidateControllerSet(base);
  if (!validated.ok()) {
    return Invalid(validated.error_message());
  }
  std::array<std::optional<ControlConf>, 3> candidate;
  if (!base.has_controller_profiles()) {
    candidate[0] = base;
  } else {
    const auto& profiles = base.controller_profiles();
    if (profiles.version().empty() || !profiles.has_max_switch_speed_mps() ||
        !std::isfinite(profiles.max_switch_speed_mps()) ||
        profiles.max_switch_speed_mps() < 0.0 ||
        !std::isfinite(stopped_speed_mps) || stopped_speed_mps < 0.0 ||
        !profiles.has_road_tracking() || !profiles.has_standstill_hold() ||
        !profiles.standstill_hold().has_max_entry_speed_mps() ||
        profiles.standstill_hold().max_entry_speed_mps() > stopped_speed_mps ||
        (profiles.has_low_speed_precision() &&
         (!profiles.low_speed_precision().has_max_entry_speed_mps() ||
          profiles.low_speed_precision().max_entry_speed_mps() <= 0.0))) {
      return Invalid(
          "profile version, road/hold banks and calibrated "
          "switch/entry speed limits are required");
    }
    const ControllerTuningProfile* tunings[] = {
        &profiles.road_tracking(),
        profiles.has_low_speed_precision() ? &profiles.low_speed_precision()
                                           : nullptr,
        &profiles.standstill_hold()};
    for (size_t i = 0; i < candidate.size(); ++i) {
      if (tunings[i] == nullptr) {
        continue;
      }
      ControlConf effective;
      const auto configured = Configure(*tunings[i], base, &effective);
      if (!configured.ok()) {
        return configured;
      }
      candidate[i] = std::move(effective);
    }
  }
  *configurations = std::move(candidate);
  return Status::OK();
}

Status ControllerProfileManager::Init(
    std::shared_ptr<DependencyInjector> injector, const ControlConf& conf,
    double stopped_speed_mps, double max_steer_rate_pct_per_sec,
    double max_acceleration_mps2, double max_deceleration_mps2) {
  if (entries_[0] != nullptr || injector == nullptr) {
    return Invalid("profile manager requires an injector and fresh state");
  }
  std::array<std::optional<ControlConf>, 3> configurations;
  const auto configured =
      BuildConfigurations(conf, stopped_speed_mps, &configurations);
  if (!configured.ok()) {
    return configured;
  }
  if (conf.has_controller_profiles() &&
      (!conf.has_control_period() || !std::isfinite(conf.control_period()) ||
       conf.control_period() <= 0.0 ||
       !std::isfinite(conf.steer_angle_rate()) ||
       conf.steer_angle_rate() <= 0.0 ||
       !std::isfinite(max_steer_rate_pct_per_sec) ||
       max_steer_rate_pct_per_sec <= 0.0 ||
       !std::isfinite(max_acceleration_mps2) || max_acceleration_mps2 <= 0.0 ||
       !std::isfinite(max_deceleration_mps2) || max_deceleration_mps2 <= 0.0)) {
    return Invalid(
        "profiled control requires a valid cycle and platform actuation "
        "limits");
  }
  std::array<std::unique_ptr<Entry>, 3> entries;
  for (size_t i = 0; i < entries.size(); ++i) {
    if (!configurations[i]) {
      continue;
    }
    auto entry = std::make_unique<Entry>();
    entry->conf = std::move(*configurations[i]);
    if (conf.has_controller_profiles()) {
      const auto& profiles = conf.controller_profiles();
      const auto& tuning = i == 0   ? profiles.road_tracking()
                           : i == 1 ? profiles.low_speed_precision()
                                    : profiles.standstill_hold();
      if (tuning.has_max_entry_speed_mps()) {
        entry->max_entry_speed_mps = tuning.max_entry_speed_mps();
      }
    }
    entry->agent = std::make_unique<ControllerAgent>();
    const auto initialized = entry->agent->Init(injector, &entry->conf);
    if (!initialized.ok()) {
      AERROR << "Profile controller initialization failed: "
             << initialized.error_message();
      return initialized;
    }
    entries[i] = std::move(entry);
  }
  entries_ = std::move(entries);
  enabled_ = conf.has_controller_profiles();
  stopped_speed_mps_ = stopped_speed_mps;
  max_steer_rate_pct_per_sec_ = max_steer_rate_pct_per_sec;
  max_acceleration_mps2_ = max_acceleration_mps2;
  max_deceleration_mps2_ = max_deceleration_mps2;
  if (enabled_) {
    max_switch_speed_mps_ = conf.controller_profiles().max_switch_speed_mps();
  }
  status_.set_profiled_configuration(enabled_);
  status_.set_configuration_version(
      enabled_ ? conf.controller_profiles().version() : "legacy-fixed");
  return Status::OK();
}

Status ControllerProfileManager::SelectionError(const std::string& reason) {
  status_.set_selection_failed(true);
  status_.set_reason(reason);
  AERROR << reason;
  return Status(ErrorCode::CONTROL_COMPUTE_ERROR, reason);
}

Status ControllerProfileManager::Bind(
    ControllerParameterProfile profile,
    const planning::MotionExecutionCommand& command,
    canbus::Chassis::GearPosition gear, double speed_mps) {
  status_.set_requested_profile(ControllerProfileName(profile));
  status_.mutable_requested_motion_identity()->CopyFrom(command.identity());
  const auto requested = static_cast<size_t>(profile);
  if (requested >= entries_.size() || !command.has_identity() ||
      command.identity().producer_epoch().empty() ||
      command.identity().aggregate_id().empty() ||
      command.identity().command_id().empty() ||
      command.identity().revision() == 0 || !std::isfinite(speed_mps)) {
    return SelectionError(
        "profile binding requires a valid motion and live speed");
  }
  const size_t target = enabled_ ? requested : 0;
  if (!entries_[target]) {
    return SelectionError("requested parameter profile is not configured");
  }
  const bool has_gear = command.has_trajectory()
                            ? command.trajectory().has_gear()
                            : command.has_start_condition() &&
                                  command.start_condition().has_expected_gear();
  const auto expected_gear = command.has_trajectory()
                                 ? command.trajectory().gear()
                                 : command.start_condition().expected_gear();
  if (!has_gear || gear != expected_gear) {
    return SelectionError(
        "profile binding gear differs from authorized motion");
  }
  if (command.has_primitive() &&
      command.primitive().type() !=
          planning::MOTION_PRIMITIVE_STANDSTILL_HOLD &&
      !LatLon(entries_[target]->conf)) {
    return SelectionError("spatial primitive backend requires LAT/LON profile");
  }
  if (enabled_) {
    if (!command.has_constraints() ||
        !command.constraints().has_max_acceleration_mps2() ||
        !command.constraints().has_max_deceleration_mps2() ||
        !std::isfinite(command.constraints().max_acceleration_mps2()) ||
        command.constraints().max_acceleration_mps2() <= 0.0 ||
        !std::isfinite(command.constraints().max_deceleration_mps2()) ||
        command.constraints().max_deceleration_mps2() <= 0.0) {
      return SelectionError(
          "profile binding requires authorized acceleration limits");
    }
    const auto limit = entries_[target]->max_entry_speed_mps;
    if (limit && (std::abs(speed_mps) > *limit || !command.has_constraints() ||
                  !command.constraints().has_max_speed_mps() ||
                  !std::isfinite(command.constraints().max_speed_mps()) ||
                  command.constraints().max_speed_mps() < 0.0 ||
                  command.constraints().max_speed_mps() > *limit)) {
      return SelectionError(
          "live/authorized speed exceeds local profile limit");
    }
    if (target != selected_ && std::abs(speed_mps) > max_switch_speed_mps_) {
      return SelectionError(
          "profile transition exceeds calibrated switch speed");
    }
    const bool family_changed =
        HasController(entries_[target]->conf, ControlConf::MPC_CONTROLLER) !=
        HasController(entries_[selected_]->conf, ControlConf::MPC_CONTROLLER);
    if ((family_changed || (bound_ && gear != gear_)) &&
        std::abs(speed_mps) > stopped_speed_mps_) {
      return SelectionError(
          "controller family or gear change requires standstill");
    }
  }
  const bool unchanged =
      bound_ && target == selected_ && gear == gear_ &&
      SameOwner(status_.bound_motion_identity(), command.identity()) &&
      status_.authorized_mission_identity().SerializeAsString() ==
          command.authorized_mission_identity().SerializeAsString();
  if (!unchanged) {
    if (bound_) {
      const auto reset = entries_[selected_]->agent->Reset();
      if (!reset.ok()) {
        return SelectionError("previous controller reset failed: " +
                              reset.error_message());
      }
    }
    if (!bound_ || target != selected_) {
      const auto reset = entries_[target]->agent->Reset();
      if (!reset.ok()) {
        return SelectionError("target controller reset failed: " +
                              reset.error_message());
      }
    }
    selected_ = target;
    gear_ = gear;
    bound_ = true;
    status_.set_parameter_profile(enabled_ ? ControllerProfileName(profile)
                                           : "legacy-fixed");
    status_.clear_controller();
    for (const auto& name : entries_[target]->agent->ControllerNames()) {
      status_.add_controller(name);
    }
  }
  status_.set_binding_active(true);
  status_.mutable_bound_motion_identity()->CopyFrom(command.identity());
  if (command.has_authorized_mission_identity()) {
    status_.mutable_authorized_mission_identity()->CopyFrom(
        command.authorized_mission_identity());
  } else {
    status_.clear_authorized_mission_identity();
  }
  status_.set_selection_failed(false);
  motion_constraints_ = command.constraints();
  status_.set_reason(
      enabled_ ? "authorized local parameter profile"
               : "dynamic profiles disabled; fixed legacy controller set");
  return Status::OK();
}

Status ControllerProfileManager::ComputeControlCommand(
    const localization::LocalizationEstimate* localization,
    const canbus::Chassis* chassis, const planning::ADCTrajectory* trajectory,
    ControlCommand* command) {
  if (!bound_ || status_.selection_failed() || !entries_[selected_]) {
    return SelectionError("controller compute has no valid profile binding");
  }
  if (command == nullptr || chassis == nullptr) {
    return SelectionError("controller compute requires command and chassis");
  }
  ControlCommand candidate = *command;
  const auto computed = entries_[selected_]->agent->ComputeControlCommand(
      localization, chassis, trajectory, &candidate);
  if (!computed.ok()) {
    return computed;
  }
  if (enabled_) {
    if (!candidate.has_acceleration() ||
        !std::isfinite(candidate.acceleration()) ||
        candidate.acceleration() >
            std::min(max_acceleration_mps2_,
                     motion_constraints_.max_acceleration_mps2()) ||
        candidate.acceleration() <
            -std::min(max_deceleration_mps2_,
                      motion_constraints_.max_deceleration_mps2())) {
      return SelectionError(
          "controller acceleration exceeds platform/motion limits");
    }
    const double seed =
        have_previous_steer_ ? previous_steer_ : chassis->steering_percentage();
    if ((!have_previous_steer_ && !chassis->has_steering_percentage()) ||
        !candidate.has_steering_target() ||
        !std::isfinite(candidate.steering_target()) ||
        std::abs(candidate.steering_target()) > 100.0 ||
        !candidate.has_steering_rate() ||
        !std::isfinite(candidate.steering_rate()) ||
        candidate.steering_rate() < 0.0 || !std::isfinite(seed) ||
        std::abs(seed) > 100.0) {
      return SelectionError("invalid steering output or continuity seed");
    }
    const auto& conf = entries_[selected_]->conf;
    const double rate =
        std::min(max_steer_rate_pct_per_sec_, conf.steer_angle_rate());
    const double step = rate * conf.control_period();
    const double steering =
        std::clamp(candidate.steering_target(), std::max(-100.0, seed - step),
                   std::min(100.0, seed + step));
    candidate.set_steering_target(steering);
    candidate.set_steering_rate(rate);
    previous_steer_ = steering;
    have_previous_steer_ = true;
  }
  command->Swap(&candidate);
  return Status::OK();
}

Status ControllerProfileManager::Reset() {
  Status reset = entries_[selected_]
                     ? entries_[selected_]->agent->Reset()
                     : Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                              "profile manager is not initialized");
  bound_ = false;
  motion_constraints_.Clear();
  have_previous_steer_ = false;
  status_.set_binding_active(false);
  status_.clear_bound_motion_identity();
  status_.clear_authorized_mission_identity();
  if (!reset.ok()) {
    return SelectionError("controller profile reset failed: " +
                          reset.error_message());
  }
  return reset;
}

bool ControllerProfileManager::SupportsSpatialPrimitives() const {
  const auto& entry = entries_[enabled_ ? 1 : 0];
  return entry && LatLon(entry->conf);
}

}  // namespace control
}  // namespace apollo
