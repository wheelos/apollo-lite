#include <chrono>
#include <fstream>
#include <thread>

#include "gflags/gflags.h"
#include "nlohmann/json.hpp"

#include "cyber/cyber.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/simulator/simulation_manager.h"

DEFINE_string(scenario_file, "", "Path to the scenario JSON file to play.");
DEFINE_int32(timeout_s, 60,
             "Wall-clock timeout in seconds for a single scenario run.");
DEFINE_int32(
    max_steps, 0,
    "Maximum simulation ticks before graceful stop (0 means no step limit).");
DEFINE_int32(poll_interval_ms, 10,
             "Polling interval in milliseconds for run-state checks.");
DEFINE_string(result_json, "",
              "Optional output path for machine-readable run result JSON.");

namespace {

void WriteResultJson(const std::string& path, const nlohmann::json& result) {
  if (path.empty()) {
    return;
  }
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    AERROR << "Failed to open result JSON path: " << path;
    return;
  }
  ofs << result.dump(2);
}

}  // namespace

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  // 初始化 CyberRT 环境
  apollo::cyber::Init(argv[0]);
  AINFO << "Apollo Simulator Engine is starting...";

  if (FLAGS_scenario_file.empty()) {
    AERROR << "Please provide a scenario file using "
              "--scenario_file=/path/to/scenario.json";
    return -1;
  }

  // 1. 初始化依赖组件 (MapService)
  // 参数为 false 表示不加载 routing topology (简化加载)
  apollo::dreamview::MapService map_service(false);

  // 2. 构造并初始化仿真管家
  apollo::dreamview::SimulationManager sim_manager(&map_service);
  if (!sim_manager.Init(true)) {
    AERROR << "Failed to initialize Simulator Manager.";
    return -1;
  }

  // 3. 开始加载并运行场景
  AINFO << "Loading and Playing Scenario: " << FLAGS_scenario_file;
  if (!sim_manager.PlayScenario(FLAGS_scenario_file)) {
    AERROR << "Failed to load or play scenario: " << FLAGS_scenario_file;
    nlohmann::json result = {
        {"scenario_file", FLAGS_scenario_file},
        {"status", "fault"},
        {"reason", "play_scenario_failed"},
        {"tick_count", 0},
        {"sim_time_sec", 0.0},
    };
    WriteResultJson(FLAGS_result_json, result);
    apollo::cyber::Clear();
    return 2;
  }

  // 4. 单次任务运行（P1）：满足 timeout/max_steps 即停止并退出
  const auto wall_start = std::chrono::steady_clock::now();
  std::string stop_reason = "max_steps_reached";
  bool success = true;

  while (apollo::cyber::OK() && sim_manager.IsRunning()) {
    const auto now = std::chrono::steady_clock::now();
    const auto wall_elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(now -
                                                                  wall_start)
            .count();

    if (FLAGS_timeout_s > 0 && wall_elapsed >= FLAGS_timeout_s) {
      stop_reason = "timeout";
      success = false;
      break;
    }

    if (FLAGS_max_steps > 0 &&
        sim_manager.GetTickCount() >= static_cast<uint64_t>(FLAGS_max_steps)) {
      stop_reason = "max_steps_reached";
      success = true;
      break;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(FLAGS_poll_interval_ms));
  }

  sim_manager.Stop();

  const auto wall_end = std::chrono::steady_clock::now();
  const double wall_time_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(wall_end -
                                                                wall_start)
          .count();

  nlohmann::json result = {
      {"scenario_file", FLAGS_scenario_file},
      {"status", success ? "ok" : "timeout"},
      {"reason", stop_reason},
      {"tick_count", sim_manager.GetTickCount()},
      {"sim_time_sec", sim_manager.GetSimTimeSec()},
      {"wall_time_sec", wall_time_sec},
  };
  WriteResultJson(FLAGS_result_json, result);

  AINFO << "Apollo Simulator Engine shuts down.";
  apollo::cyber::Clear();
  return success ? 0 : 3;
}
