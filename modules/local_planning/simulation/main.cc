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

#include <iostream>
#include <string>

#include "modules/local_planning/simulation/closed_loop.h"

int main(int argc, char** argv) {
  apollo::local_planning::SimulationOptions options;
  std::string output;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help") {
      std::cout
          << "local_planning_sim --backend kinematic|mujoco "
             "--scenario straight|curve|obstacle|moving_obstacle|stale|"
             "epoch|odom_loss|temporal_noise|occlusion|delayed|odom_bias "
             "--output PREFIX [--model MODEL.xml]\n"
             "Runs 30 simulated seconds; writes PREFIX.csv and PREFIX.svg."
             "\nThe output directory must already exist.\n";
      return 0;
    }
    if (i + 1 >= argc) {
      std::cerr << "Missing option value: " << arg << '\n';
      return 2;
    }
    const std::string value = argv[++i];
    if (arg == "--backend") {
      options.backend = value;
    } else if (arg == "--scenario") {
      options.scenario = value;
    } else if (arg == "--model") {
      options.model_path = value;
    } else if (arg == "--output") {
      output = value;
    } else {
      std::cerr << "Unknown option: " << arg << '\n';
      return 2;
    }
  }
  if (output.empty()) {
    std::cerr << "--output PREFIX is required\n";
    return 2;
  }
  apollo::local_planning::SimulationReport report;
  const auto run = apollo::local_planning::RunClosedLoop(options, &report);
  if (!report.samples.empty()) {
    const auto written =
        apollo::local_planning::WriteSimulationArtifacts(output, report);
    if (!written.ok()) {
      std::cerr << written.ToString() << '\n';
      return 1;
    }
  }
  if (!run.ok()) {
    std::cerr << run.ToString() << '\n';
    return 1;
  }
  std::cout << "PASS backend=" << options.backend
            << " scenario=" << options.scenario
            << " progress_m=" << report.progress
            << " max_lateral_error_m=" << report.max_lateral_error
            << " final_speed_mps=" << report.final_speed
            << " rejected_cycles=" << report.rejected_cycles << '\n';
  std::cout << "lane_updates=" << report.lane_updates
            << " stitched_cycles=" << report.stitched_cycles << '\n';
  if (!report.first_rejection.empty()) {
    std::cout << "Injected fault: " << report.first_rejection << '\n';
  }
  std::cout << "Artifacts: " << output << ".csv " << output << ".svg\n";
  return 0;
}
