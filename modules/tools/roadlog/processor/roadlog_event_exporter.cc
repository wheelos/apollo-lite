/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include "modules/tools/roadlog/processor/roadlog_event_exporter.h"

#include <fstream>
#include <unistd.h>

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/time.h"

namespace apollo {
namespace data {

namespace {

using apollo::cyber::Time;
using cyber::common::EnsureDirectory;
using cyber::common::GetFileName;

std::string TriggerGroupToString(const TriggerGroup group) {
  switch (group) {
    case TriggerGroup::kIncident:
      return "incident";
    case TriggerGroup::kPeriodicSnapshot:
      return "periodic_snapshot";
  }
  return "unknown";
}

}  // namespace

bool RoadlogEventExporter::Export(const RoadlogEventExportPlan& plan) const {
  if (!EnsureDirectory(plan.output_dir)) {
    AERROR << "failed to create event output dir: " << plan.output_dir;
    return false;
  }
  const std::string manifest_path =
      absl::StrCat(plan.output_dir, "/event_manifest.txt");
  std::ofstream manifest(manifest_path, std::ios::out | std::ios::trunc);
  if (!manifest) {
    AERROR << "failed to open event manifest: " << manifest_path;
    return false;
  }
  manifest << "event_id=" << plan.metadata.event_id << "\n";
  manifest << "group=" << TriggerGroupToString(plan.metadata.group) << "\n";
  manifest << "first_trigger_ns=" << plan.metadata.first_trigger_time << "\n";
  manifest << "last_trigger_ns=" << plan.metadata.last_trigger_time << "\n";
  manifest << "window_begin_ns=" << plan.metadata.window_begin_time << "\n";
  manifest << "window_end_ns=" << plan.metadata.window_end_time << "\n";
  manifest << "export_ready_ns=" << plan.metadata.export_ready_time << "\n";
  manifest << "export_time_ns=" << Time::Now().ToNanosecond() << "\n";
  manifest << "coverage=" << (plan.partial ? "partial" : "complete") << "\n";
  manifest << "trigger_hits=" << plan.metadata.total_trigger_count << "\n";
  manifest << "suppressed_duplicate_hits="
           << plan.metadata.suppressed_duplicate_count << "\n";
  manifest << "trigger_types=" << plan.metadata.trigger_summaries.size() << "\n";
  for (const auto& trigger_name_and_summary : plan.metadata.trigger_summaries) {
    const auto& summary = trigger_name_and_summary.second;
    manifest << "trigger name=" << summary.trigger_name
             << " count=" << summary.count
             << " first_ns=" << summary.first_trigger_time
             << " last_ns=" << summary.last_trigger_time
             << " backward_ns=" << summary.backward_time
             << " forward_ns=" << summary.forward_time
             << " cooldown_ns=" << summary.cooldown_time
             << " group=" << TriggerGroupToString(summary.group)
             << " description=\"" << summary.description << "\"\n";
  }
  manifest << "segments=" << plan.segments.size() << "\n";
  for (const auto& segment : plan.segments) {
    const std::string target_path =
        absl::StrCat(plan.output_dir, "/", GetFileName(segment.path));
    if (!ExportSegment(segment.path, target_path)) {
      AERROR << "failed to export retained segment " << segment.path << " -> "
             << target_path;
      return false;
    }
    manifest << GetFileName(segment.path) << " begin_ns=" << segment.begin_time
             << " end_ns=" << segment.end_time << " bytes=" << segment.bytes
             << "\n";
  }
  return true;
}

bool RoadlogEventExporter::ExportSegment(const std::string& source_path,
                                         const std::string& target_path) const {
  if (link(source_path.c_str(), target_path.c_str()) == 0) {
    return true;
  }
  return CopyFile(source_path, target_path);
}

bool RoadlogEventExporter::CopyFile(const std::string& source_path,
                                    const std::string& target_path) const {
  std::ifstream input(source_path, std::ios::binary);
  if (!input) {
    AERROR << "failed to open source segment for copy: " << source_path;
    return false;
  }
  std::ofstream output(target_path, std::ios::binary | std::ios::trunc);
  if (!output) {
    AERROR << "failed to open target segment for copy: " << target_path;
    return false;
  }
  output << input.rdbuf();
  return input.good() || input.eof();
}

}  // namespace data
}  // namespace apollo
