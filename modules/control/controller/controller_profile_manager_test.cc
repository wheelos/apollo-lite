#include "modules/control/controller/controller_profile_manager.h"

#include <limits>
#include <memory>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace control {
namespace {

using common::ErrorCode;
using common::Status;

struct Calls {
  int compute = 0;
  int reset = 0;
  bool fail_compute = false;
  bool fail_reset = false;
  double steering = 50.0;
  double acceleration = 0.0;
};

class FakeController : public Controller {
 public:
  explicit FakeController(std::shared_ptr<Calls> calls) : calls_(calls) {}
  Status Init(std::shared_ptr<DependencyInjector>,
              const ControlConf*) override {
    return Status::OK();
  }
  Status ComputeControlCommand(const localization::LocalizationEstimate*,
                               const canbus::Chassis*,
                               const planning::ADCTrajectory*,
                               ControlCommand* command) override {
    ++calls_->compute;
    command->set_steering_target(calls_->steering);
    command->set_steering_rate(100.0);
    command->set_acceleration(calls_->acceleration);
    return calls_->fail_compute ? Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                                         "fake controller failure")
                                : Status::OK();
  }
  Status Reset() override {
    ++calls_->reset;
    return calls_->fail_reset
               ? Status(ErrorCode::CONTROL_COMPUTE_ERROR, "fake reset failure")
               : Status::OK();
  }
  std::string Name() const override { return "fake-controller"; }
  void Stop() override {}

 private:
  std::shared_ptr<Calls> calls_;
};

std::unique_ptr<ControllerAgent> Agent(const std::shared_ptr<Calls>& calls) {
  auto agent = std::make_unique<ControllerAgent>();
  agent->controller_list_.push_back(std::make_unique<FakeController>(calls));
  agent->initialized_ = true;
  return agent;
}

ControlConf Base() {
  ControlConf conf;
  conf.add_active_controllers(ControlConf::LAT_CONTROLLER);
  conf.add_active_controllers(ControlConf::LON_CONTROLLER);
  conf.set_control_period(0.01);
  conf.set_steer_angle_rate(100.0);
  auto* lat = conf.mutable_lat_controller_conf();
  lat->set_cf(123.0);
  lat->set_mass_fl(100);
  lat->set_ts(0.01);
  for (int i = 0; i < 4; ++i) {
    lat->add_matrix_q(1.0);
    lat->add_reverse_matrix_q(2.0);
  }
  auto* lon = conf.mutable_lon_controller_conf();
  lon->set_ts(0.01);
  lon->set_standstill_acceleration(-0.3);
  for (auto* pid :
       {lon->mutable_station_pid_conf(), lon->mutable_low_speed_pid_conf(),
        lon->mutable_high_speed_pid_conf(),
        lon->mutable_reverse_station_pid_conf(),
        lon->mutable_reverse_speed_pid_conf()}) {
    pid->set_kp(1.0);
    pid->set_ki(0.0);
    pid->set_kd(0.0);
    pid->set_integrator_enable(false);
  }
  return conf;
}

ControlConf Profiled() {
  auto conf = Base();
  auto* profiles = conf.mutable_controller_profiles();
  profiles->set_version("test-calibration-v1");
  profiles->set_max_switch_speed_mps(0.5);
  profiles->mutable_road_tracking()->set_inherit_base_tuning(true);
  profiles->mutable_low_speed_precision()->set_inherit_base_tuning(true);
  profiles->mutable_low_speed_precision()->set_max_entry_speed_mps(1.0);
  profiles->mutable_standstill_hold()->set_inherit_base_tuning(true);
  profiles->mutable_standstill_hold()->set_max_entry_speed_mps(0.1);
  return conf;
}

planning::MotionExecutionCommand Command(uint64_t revision = 1) {
  planning::MotionExecutionCommand command;
  auto* identity = command.mutable_identity();
  identity->set_producer_epoch("planning-test");
  identity->set_aggregate_id("motion");
  identity->set_command_id("follow-path");
  identity->set_revision(revision);
  auto* mission = command.mutable_authorized_mission_identity();
  mission->set_producer_epoch("mission-test");
  mission->set_aggregate_id("mission");
  mission->set_command_id("task");
  mission->set_revision(1);
  command.mutable_trajectory()->set_gear(canbus::Chassis::GEAR_DRIVE);
  command.mutable_constraints()->set_max_speed_mps(1.0);
  command.mutable_constraints()->set_max_acceleration_mps2(1.0);
  command.mutable_constraints()->set_max_deceleration_mps2(2.0);
  return command;
}

class ControllerProfileManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto conf = Profiled();
    std::array<std::optional<ControlConf>, 3> configurations;
    ASSERT_TRUE(ControllerProfileManager::BuildConfigurations(conf, 0.1,
                                                              &configurations)
                    .ok());
    for (size_t i = 0; i < calls_.size(); ++i) {
      calls_[i] = std::make_shared<Calls>();
      auto entry = std::make_unique<ControllerProfileManager::Entry>();
      entry->conf = *configurations[i];
      entry->agent = Agent(calls_[i]);
      if (i != 0) {
        entry->max_entry_speed_mps = i == 1 ? 1.0 : 0.1;
      }
      manager_.entries_[i] = std::move(entry);
    }
    manager_.enabled_ = true;
    manager_.stopped_speed_mps_ = 0.1;
    manager_.max_switch_speed_mps_ = 0.5;
    manager_.max_steer_rate_pct_per_sec_ = 50.0;
    manager_.max_acceleration_mps2_ = 2.0;
    manager_.max_deceleration_mps2_ = 3.0;
    manager_.status_.set_profiled_configuration(true);
    manager_.status_.set_configuration_version("test-calibration-v1");
  }

  std::array<std::shared_ptr<Calls>, 3> calls_;
  ControllerProfileManager manager_;
};

TEST(ControllerAgentTest, RejectsConflictingOutputOwners) {
  auto conf = Base();
  EXPECT_TRUE(ControllerAgent::ValidateControllerSet(conf).ok());
  conf.add_active_controllers(ControlConf::MPC_CONTROLLER);
  EXPECT_FALSE(ControllerAgent::ValidateControllerSet(conf).ok());
  conf = Base();
  conf.add_active_controllers(ControlConf::LAT_CONTROLLER);
  EXPECT_FALSE(ControllerAgent::ValidateControllerSet(conf).ok());
  conf = Base();
  conf.add_active_controllers(ControlConf::LON_SPEED_CONTROLLER);
  EXPECT_FALSE(ControllerAgent::ValidateControllerSet(conf).ok());
  conf = Base();
  conf.add_active_controllers(ControlConf::DIFF_DRIVE_LAT_CONTROLLER);
  EXPECT_FALSE(ControllerAgent::ValidateControllerSet(conf).ok());
  conf.clear_active_controllers();
  EXPECT_FALSE(ControllerAgent::ValidateControllerSet(conf).ok());
  conf.add_active_controllers(ControlConf::MPC_CONTROLLER);
  EXPECT_TRUE(ControllerAgent::ValidateControllerSet(conf).ok());
}

TEST(ControllerAgentTest, FailureDoesNotPublishPartialCommandOrRunSuccessor) {
  auto first = std::make_shared<Calls>();
  auto second = std::make_shared<Calls>();
  auto third = std::make_shared<Calls>();
  second->fail_compute = true;
  auto agent = Agent(first);
  agent->controller_list_.push_back(std::make_unique<FakeController>(second));
  agent->controller_list_.push_back(std::make_unique<FakeController>(third));
  ControlCommand output;
  output.set_steering_target(7.0);
  const auto original = output.SerializeAsString();
  localization::LocalizationEstimate localization;
  canbus::Chassis chassis;
  planning::ADCTrajectory trajectory;
  const auto computed = agent->ComputeControlCommand(&localization, &chassis,
                                                     &trajectory, &output);
  EXPECT_FALSE(computed.ok());
  EXPECT_EQ(computed.error_message(), "fake controller failure");
  EXPECT_EQ(first->compute, 1);
  EXPECT_EQ(second->compute, 1);
  EXPECT_EQ(third->compute, 0);
  EXPECT_EQ(output.SerializeAsString(), original);
}

TEST(ControllerAgentTest, ResetFailureBlocksComputeUntilSuccessfulReset) {
  auto calls = std::make_shared<Calls>();
  calls->fail_reset = true;
  auto agent = Agent(calls);
  EXPECT_FALSE(agent->Reset().ok());
  ControlCommand output;
  localization::LocalizationEstimate localization;
  canbus::Chassis chassis;
  planning::ADCTrajectory trajectory;
  EXPECT_FALSE(
      agent
          ->ComputeControlCommand(&localization, &chassis, &trajectory, &output)
          .ok());
  EXPECT_EQ(calls->compute, 0);
  calls->fail_reset = false;
  EXPECT_TRUE(agent->Reset().ok());
  EXPECT_TRUE(
      agent
          ->ComputeControlCommand(&localization, &chassis, &trajectory, &output)
          .ok());
}

TEST(ControllerProfileConfigurationTest,
     OverridesOnlyLocalTuningAndPreservesBase) {
  auto conf = Profiled();
  const auto original = conf.SerializeAsString();
  auto* precision =
      conf.mutable_controller_profiles()->mutable_low_speed_precision();
  for (int i = 0; i < 4; ++i) {
    precision->add_lateral_q(3.0);
    precision->add_lateral_reverse_q(4.0);
  }
  precision->mutable_low_speed_pid()->set_kp(5.0);
  const auto tuned_input = conf.SerializeAsString();
  std::array<std::optional<ControlConf>, 3> configurations;
  ASSERT_TRUE(
      ControllerProfileManager::BuildConfigurations(conf, 0.1, &configurations)
          .ok());
  ASSERT_TRUE(configurations[0]);
  ASSERT_TRUE(configurations[1]);
  EXPECT_EQ(configurations[0]->lat_controller_conf().matrix_q(0), 1.0);
  EXPECT_EQ(configurations[1]->lat_controller_conf().matrix_q_size(), 4);
  EXPECT_EQ(configurations[1]->lat_controller_conf().matrix_q(0), 3.0);
  EXPECT_EQ(configurations[1]->lat_controller_conf().reverse_matrix_q(0), 4.0);
  EXPECT_EQ(configurations[1]->lon_controller_conf().low_speed_pid_conf().kp(),
            5.0);
  EXPECT_EQ(configurations[1]->lat_controller_conf().cf(), 123.0);
  EXPECT_EQ(configurations[1]->lat_controller_conf().mass_fl(), 100);
  EXPECT_EQ(configurations[1]->lon_controller_conf().standstill_acceleration(),
            -0.3);
  EXPECT_FALSE(configurations[1]->has_controller_profiles());
  EXPECT_EQ(conf.SerializeAsString(), tuned_input);
  EXPECT_NE(tuned_input, original);
}

TEST(ControllerProfileConfigurationTest,
     MissingAndInvalidCalibrationIsExplicit) {
  auto conf = Profiled();
  std::array<std::optional<ControlConf>, 3> configurations;
  ASSERT_TRUE(
      ControllerProfileManager::BuildConfigurations(conf, 0.1, &configurations)
          .ok());
  const auto prior = configurations[0]->SerializeAsString();
  conf.mutable_controller_profiles()->clear_version();
  EXPECT_FALSE(
      ControllerProfileManager::BuildConfigurations(conf, 0.1, &configurations)
          .ok());
  EXPECT_EQ(configurations[0]->SerializeAsString(), prior);
  conf = Profiled();
  conf.mutable_controller_profiles()
      ->mutable_low_speed_precision()
      ->set_max_entry_speed_mps(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(
      ControllerProfileManager::BuildConfigurations(conf, 0.1, &configurations)
          .ok());
  conf = Profiled();
  conf.mutable_controller_profiles()
      ->mutable_low_speed_precision()
      ->add_lateral_q(1.0);
  EXPECT_FALSE(
      ControllerProfileManager::BuildConfigurations(conf, 0.1, &configurations)
          .ok());
  conf = Profiled();
  conf.mutable_controller_profiles()
      ->mutable_standstill_hold()
      ->set_max_entry_speed_mps(0.2);
  EXPECT_FALSE(
      ControllerProfileManager::BuildConfigurations(conf, 0.1, &configurations)
          .ok());
}

TEST(ControllerProfileConfigurationTest,
     LegacyConfigurationDoesNotInventPrecision) {
  std::array<std::optional<ControlConf>, 3> configurations;
  const auto conf = Base();
  ASSERT_TRUE(
      ControllerProfileManager::BuildConfigurations(conf, 0.1, &configurations)
          .ok());
  ASSERT_TRUE(configurations[0]);
  EXPECT_EQ(configurations[0]->SerializeAsString(), conf.SerializeAsString());
  EXPECT_FALSE(configurations[1]);
  EXPECT_FALSE(configurations[2]);
}

TEST_F(ControllerProfileManagerTest, RenewalDoesNotResetAndTaskChangeDoes) {
  auto command = Command();
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, command,
                        canbus::Chassis::GEAR_DRIVE, 0.2)
                  .ok());
  EXPECT_EQ(calls_[0]->reset, 1);
  command.mutable_identity()->set_revision(2);
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, command,
                        canbus::Chassis::GEAR_DRIVE, 0.2)
                  .ok());
  EXPECT_EQ(calls_[0]->reset, 1);
  EXPECT_EQ(manager_.status().bound_motion_identity().revision(), 2);
  command.mutable_authorized_mission_identity()->set_revision(2);
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, command,
                        canbus::Chassis::GEAR_DRIVE, 0.2)
                  .ok());
  EXPECT_EQ(calls_[0]->reset, 2);
}

TEST_F(ControllerProfileManagerTest,
       MissingProfileAndUnsafeSwitchPreserveBinding) {
  const auto command = Command();
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, command,
                        canbus::Chassis::GEAR_DRIVE, 0.8)
                  .ok());
  EXPECT_FALSE(manager_
                   .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                         canbus::Chassis::GEAR_DRIVE, 0.8)
                   .ok());
  EXPECT_EQ(manager_.status().parameter_profile(), "road-tracking");
  EXPECT_EQ(manager_.status().requested_profile(), "low-speed-precision");
  EXPECT_TRUE(manager_.status().selection_failed());
  EXPECT_EQ(calls_[1]->reset, 0);
  manager_.entries_[1].reset();
  EXPECT_FALSE(manager_
                   .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                         canbus::Chassis::GEAR_DRIVE, 0.0)
                   .ok());
  EXPECT_EQ(manager_.status().bound_motion_identity().SerializeAsString(),
            command.identity().SerializeAsString());
}

TEST_F(ControllerProfileManagerTest,
       FamilyAndGearChangesRequirePhysicalStandstill) {
  auto command = Command();
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, command,
                        canbus::Chassis::GEAR_DRIVE, 0.0)
                  .ok());
  auto* conf = &manager_.entries_[1]->conf;
  conf->clear_active_controllers();
  conf->add_active_controllers(ControlConf::MPC_CONTROLLER);
  EXPECT_FALSE(manager_
                   .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                         canbus::Chassis::GEAR_DRIVE, 0.2)
                   .ok());
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                        canbus::Chassis::GEAR_DRIVE, 0.05)
                  .ok());
  command.mutable_trajectory()->set_gear(canbus::Chassis::GEAR_REVERSE);
  EXPECT_FALSE(manager_
                   .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                         canbus::Chassis::GEAR_REVERSE, 0.2)
                   .ok());
  EXPECT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                        canbus::Chassis::GEAR_REVERSE, 0.05)
                  .ok());
}

TEST_F(ControllerProfileManagerTest, ResetFailureCannotCommitCandidateProfile) {
  const auto command = Command();
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, command,
                        canbus::Chassis::GEAR_DRIVE, 0.0)
                  .ok());
  calls_[1]->fail_reset = true;
  EXPECT_FALSE(manager_
                   .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                         canbus::Chassis::GEAR_DRIVE, 0.0)
                   .ok());
  EXPECT_EQ(manager_.status().parameter_profile(), "road-tracking");
  EXPECT_EQ(manager_.selected_, 0);
  EXPECT_TRUE(manager_.status().selection_failed());
}

TEST_F(ControllerProfileManagerTest,
       SteeringOutputRemainsContinuousAcrossProfiles) {
  auto command = Command();
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, command,
                        canbus::Chassis::GEAR_DRIVE, 0.0)
                  .ok());
  localization::LocalizationEstimate localization;
  canbus::Chassis chassis;
  chassis.set_steering_percentage(10.0);
  planning::ADCTrajectory trajectory;
  ControlCommand output;
  ASSERT_TRUE(
      manager_
          .ComputeControlCommand(&localization, &chassis, &trajectory, &output)
          .ok());
  EXPECT_DOUBLE_EQ(output.steering_target(), 10.5);
  command.mutable_identity()->set_command_id("precision");
  calls_[1]->steering = -50.0;
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                        canbus::Chassis::GEAR_DRIVE, 0.0)
                  .ok());
  ASSERT_TRUE(
      manager_
          .ComputeControlCommand(&localization, &chassis, &trajectory, &output)
          .ok());
  EXPECT_DOUBLE_EQ(output.steering_target(), 10.0);
  EXPECT_DOUBLE_EQ(output.steering_rate(), 50.0);
  ControllerSelectionStatus decoded;
  ASSERT_TRUE(decoded.ParseFromString(manager_.status().SerializeAsString()));
  EXPECT_EQ(decoded.configuration_version(), "test-calibration-v1");
  EXPECT_EQ(decoded.parameter_profile(), "low-speed-precision");
  EXPECT_TRUE(decoded.binding_active());
  EXPECT_EQ(decoded.controller(0), "fake-controller");
  ASSERT_TRUE(manager_.Reset().ok());
  EXPECT_FALSE(manager_.status().binding_active());
  EXPECT_FALSE(manager_.status().has_bound_motion_identity());
}

TEST_F(ControllerProfileManagerTest,
       LiveAndAuthorizedProfileSpeedLimitsAreEnforced) {
  auto command = Command();
  EXPECT_FALSE(manager_
                   .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                         canbus::Chassis::GEAR_DRIVE, 1.1)
                   .ok());
  command.mutable_constraints()->set_max_speed_mps(2.0);
  EXPECT_FALSE(manager_
                   .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                         canbus::Chassis::GEAR_DRIVE, 0.0)
                   .ok());
  command.mutable_constraints()->set_max_speed_mps(1.0);
  EXPECT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, command,
                        canbus::Chassis::GEAR_DRIVE, 0.0)
                  .ok());
}

TEST_F(ControllerProfileManagerTest,
       LegacyBindingExplicitlyUsesFixedControllerSet) {
  manager_.enabled_ = false;
  manager_.entries_[1].reset();
  manager_.entries_[2].reset();
  manager_.status_.set_profiled_configuration(false);
  manager_.status_.set_configuration_version("legacy-fixed");
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_LOW_SPEED_PRECISION, Command(),
                        canbus::Chassis::GEAR_DRIVE, 2.0)
                  .ok());
  EXPECT_EQ(manager_.status().parameter_profile(), "legacy-fixed");
  EXPECT_EQ(manager_.status().requested_profile(), "low-speed-precision");
  EXPECT_FALSE(manager_.status().profiled_configuration());
  EXPECT_EQ(calls_[0]->reset, 1);
  localization::LocalizationEstimate localization;
  canbus::Chassis chassis;
  planning::ADCTrajectory trajectory;
  ControlCommand command;
  ASSERT_TRUE(
      manager_
          .ComputeControlCommand(&localization, &chassis, &trajectory, &command)
          .ok());
  EXPECT_DOUBLE_EQ(command.steering_target(), calls_[0]->steering);
}

TEST_F(ControllerProfileManagerTest,
       InvalidSteeringDoesNotBecomeClampedSuccess) {
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, Command(),
                        canbus::Chassis::GEAR_DRIVE, 0.0)
                  .ok());
  localization::LocalizationEstimate localization;
  canbus::Chassis chassis;
  chassis.set_steering_percentage(0.0);
  planning::ADCTrajectory trajectory;
  ControlCommand command;
  command.set_brake(10.0);
  const auto original = command.SerializeAsString();
  calls_[0]->steering = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      manager_
          .ComputeControlCommand(&localization, &chassis, &trajectory, &command)
          .ok());
  EXPECT_EQ(command.SerializeAsString(), original);
  EXPECT_FALSE(manager_.have_previous_steer_);
}

TEST_F(ControllerProfileManagerTest,
       ProfileGainsCannotExceedMotionActuationLimits) {
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, Command(),
                        canbus::Chassis::GEAR_DRIVE, 0.0)
                  .ok());
  localization::LocalizationEstimate localization;
  canbus::Chassis chassis;
  chassis.set_steering_percentage(0.0);
  planning::ADCTrajectory trajectory;
  ControlCommand command;
  command.set_brake(10.0);
  const auto original = command.SerializeAsString();
  calls_[0]->acceleration = 1.1;
  EXPECT_FALSE(
      manager_
          .ComputeControlCommand(&localization, &chassis, &trajectory, &command)
          .ok());
  EXPECT_EQ(command.SerializeAsString(), original);
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, Command(),
                        canbus::Chassis::GEAR_DRIVE, 0.0)
                  .ok());
  calls_[0]->acceleration = -2.1;
  EXPECT_FALSE(
      manager_
          .ComputeControlCommand(&localization, &chassis, &trajectory, &command)
          .ok());
  EXPECT_EQ(command.SerializeAsString(), original);
}

TEST_F(ControllerProfileManagerTest,
       PlatformLimitsRemainTighterThanMissionLimits) {
  auto motion = Command();
  motion.mutable_constraints()->set_max_acceleration_mps2(5.0);
  motion.mutable_constraints()->set_max_deceleration_mps2(5.0);
  ASSERT_TRUE(manager_
                  .Bind(CONTROL_PROFILE_ROAD_TRACKING, motion,
                        canbus::Chassis::GEAR_DRIVE, 0.0)
                  .ok());
  localization::LocalizationEstimate localization;
  canbus::Chassis chassis;
  chassis.set_steering_percentage(0.0);
  planning::ADCTrajectory trajectory;
  ControlCommand command;
  const auto original = command.SerializeAsString();
  for (double acceleration :
       {2.1, -3.1, std::numeric_limits<double>::quiet_NaN()}) {
    ASSERT_TRUE(manager_
                    .Bind(CONTROL_PROFILE_ROAD_TRACKING, motion,
                          canbus::Chassis::GEAR_DRIVE, 0.0)
                    .ok());
    calls_[0]->acceleration = acceleration;
    EXPECT_FALSE(manager_
                     .ComputeControlCommand(&localization, &chassis,
                                            &trajectory, &command)
                     .ok());
    EXPECT_EQ(command.SerializeAsString(), original);
  }
  for (double acceleration : {2.0, -3.0}) {
    ASSERT_TRUE(manager_
                    .Bind(CONTROL_PROFILE_ROAD_TRACKING, motion,
                          canbus::Chassis::GEAR_DRIVE, 0.0)
                    .ok());
    calls_[0]->acceleration = acceleration;
    EXPECT_TRUE(manager_
                    .ComputeControlCommand(&localization, &chassis, &trajectory,
                                           &command)
                    .ok());
    EXPECT_DOUBLE_EQ(command.acceleration(), acceleration);
  }
}

}  // namespace
}  // namespace control
}  // namespace apollo
