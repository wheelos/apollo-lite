#include "modules/dreamview/backend/simulator/scenario/parser.h"

#include <string>

namespace apollo {
namespace dreamview {

bool ScenarioParser::FromJson(const nlohmann::json& j, Scenario* out) {
  if (!out) return false;

  auto convert_to_double = [](const nlohmann::json& val) -> double {
    if (val.is_number()) return val.get<double>();
    if (val.is_string()) return std::stod(val.get<std::string>());
    return 0.0;
  };

  out->ego_waypoints.clear();
  out->static_obstacles.clear();
  out->dynamic_obstacles.clear();
  out->traffic_lights.clear();

  // Ego waypoints from "p" array (like previous format)
  if (j.contains("p")) {
    const auto& p_array = j["p"];
    if (p_array.is_array() && p_array.size() >= 4) {
      // create waypoints from pairs
      for (size_t i = 0; i + 1 < p_array.size(); i += 2) {
        out->ego_waypoints.emplace_back(convert_to_double(p_array[i]),
                                        convert_to_double(p_array[i + 1]));
      }
    }
  }

  // Static obstacles under "s"
  if (j.contains("s")) {
    for (const auto& item : j["s"]) {
      if (!item.contains("p") || item["p"].size() < 2) continue;
      Scenario::StaticObstacle so;
      if (item.contains("id"))
        so.id = static_cast<int>(convert_to_double(item["id"]));
      so.x = convert_to_double(item["p"][0]);
      so.y = convert_to_double(item["p"][1]);
      so.heading = item.contains("r") ? convert_to_double(item["r"]) : 0.0;
      so.width = item.contains("w") ? convert_to_double(item["w"]) : 2.0;
      so.length = item.contains("h") ? convert_to_double(item["h"]) : 5.0;
      out->static_obstacles.push_back(so);
    }
  }

  // Dynamic obstacles under "d". Expect positions as Cartesian (x,y) and
  // velocities vx,vy
  if (j.contains("d")) {
    for (const auto& item : j["d"]) {
      if (!item.contains("p") || item["p"].size() < 2) continue;
      if (!item.contains("v") || item["v"].size() < 2) continue;
      Scenario::DynamicObstacle dobj;
      if (item.contains("id"))
        dobj.id = static_cast<int>(convert_to_double(item["id"]));
      dobj.x = convert_to_double(item["p"][0]);
      dobj.y = convert_to_double(item["p"][1]);
      dobj.vx = convert_to_double(item["v"][0]);
      dobj.vy = convert_to_double(item["v"][1]);
      dobj.width = item.contains("w") ? convert_to_double(item["w"]) : 2.0;
      dobj.length = item.contains("h") ? convert_to_double(item["h"]) : 5.0;
      dobj.trigger_time =
          item.contains("t") ? convert_to_double(item["t"]) : 0.0;
      out->dynamic_obstacles.push_back(dobj);
    }
  }

  // Traffic lights under "t" or "traffic_lights"
  if (j.contains("t")) {
    for (const auto& item : j["t"]) {
      if (!item.contains("p") || item["p"].size() < 2) continue;
      Scenario::TrafficLight tl;
      if (item.contains("id"))
        tl.id = static_cast<int>(convert_to_double(item["id"]));
      tl.x = convert_to_double(item["p"][0]);
      tl.y = convert_to_double(item["p"][1]);
      tl.state = item.contains("s")
                     ? static_cast<int>(convert_to_double(item["s"]))
                     : 0;
      tl.trigger_time = item.contains("t") ? convert_to_double(item["t"]) : 0.0;
      out->traffic_lights.push_back(tl);
    }
  }

  return true;
}

}  // namespace dreamview
}  // namespace apollo
