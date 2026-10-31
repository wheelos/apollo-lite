// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <filesystem>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <thread>

#include "google/protobuf/text_format.h"
#include "google/protobuf/util/json_util.h"
#include "native_global_cyber.h"

namespace host = apollo::localization::openloc_diagnostic;
namespace {

struct Received {
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<host::NativeGlobalDiagnostic> outputs;
  std::vector<host::NativeGlobalProcessed> processed;
};

template <typename T>
T Parse(const std::string& json) {
  T message;
  const auto status = google::protobuf::util::JsonStringToMessage(json, &message);
  if (!status.ok() || !message.IsInitialized())
    throw std::runtime_error("invalid prepared replay JSON: " + status.ToString());
  return message;
}

template <typename T>
void Write(const std::filesystem::path& path, const std::vector<T>& values) {
  std::ofstream file(path);
  for (const auto& value : values) {
    std::string json;
    const auto status = google::protobuf::util::MessageToJsonString(value, &json);
    if (!status.ok()) throw std::runtime_error(status.ToString());
    file << json << '\n';
  }
  file.close();
  if (!file) throw std::runtime_error("incomplete replay output: " + path.string());
}

void Replay(const host::NativeGlobalHostConfig& config,
            const std::string& input, const std::filesystem::path& output) {
  if (!config.recorded_input() ||
      !std::filesystem::create_directory(output))
    throw std::runtime_error("rehosting needs recorded mode and a new output");
  auto received = std::make_shared<Received>();
  std::shared_ptr<apollo::cyber::Node> node =
      apollo::cyber::CreateNode("openloc_global_recorded_rehost");
  if (!node) throw std::runtime_error("cannot create global replay node");
  auto reader = node->CreateReader<host::NativeGlobalDiagnostic>(
      config.output_topic(), [received](const auto& message) {
        {
          std::lock_guard<std::mutex> lock(received->mutex);
          received->outputs.push_back(*message);
        }
        received->condition.notify_all();
      });
  auto completion = node->CreateReader<host::NativeGlobalProcessed>(
      config.output_topic() + "/processed", [received](const auto& message) {
        {
          std::lock_guard<std::mutex> lock(received->mutex);
          received->processed.push_back(*message);
        }
        received->condition.notify_all();
      });
  auto odometry = node->CreateWriter<host::NativeOdometryDiagnostic>(
      config.odometry_topic());
  std::shared_ptr<apollo::cyber::Writer<host::NativeGlobalGnssInput>> gnss;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::RawData>> raw;
  if (config.has_raw_gga())
    raw = node->CreateWriter<apollo::drivers::gnss::RawData>(config.gnss_topic());
  else
    gnss = node->CreateWriter<host::NativeGlobalGnssInput>(config.gnss_topic());
  std::shared_ptr<apollo::cyber::Writer<host::NativeGlobalMapInput>> map;
  if (config.has_map_topic())
    map = node->CreateWriter<host::NativeGlobalMapInput>(config.map_topic());
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> raw_map;
  if (config.has_online_map()) {
    raw_map = node->CreateWriter<apollo::drivers::PointCloud>(
        config.online_map().cloud_topic());
    if (!raw_map) throw std::runtime_error("cannot create raw map endpoint");
  }
  std::shared_ptr<apollo::cyber::Writer<host::NativeGlobalResetRequest>> reset;
  std::shared_ptr<apollo::cyber::Writer<host::NativeGlobalResetAck>> ack;
  if (config.has_reset_topic()) {
    reset = node->CreateWriter<host::NativeGlobalResetRequest>(config.reset_topic());
    ack = node->CreateWriter<host::NativeGlobalResetAck>(config.reset_topic() + "/ack");
    if (!reset || !ack) throw std::runtime_error("cannot create reset endpoints");
  }
  if (!reader || !completion || !odometry || (!gnss && !raw))
    throw std::runtime_error("cannot create recorded replay Cyber endpoints");
  if (config.has_map_topic() && !map)
    throw std::runtime_error("cannot create recorded map endpoint");
  host::NativeGlobalCyber global(node, config);
  for (int i = 0; i < 500 &&
       (!odometry->HasReader() || (gnss && !gnss->HasReader()) ||
        (raw && !raw->HasReader()) ||
        (map && !map->HasReader()) ||
        (raw_map && !raw_map->HasReader()) ||
        (reset && !reset->HasReader()) || (ack && !ack->HasReader()) ||
        !reader->HasWriter() || !completion->HasWriter()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (!odometry->HasReader() || (gnss && !gnss->HasReader()) ||
      (raw && !raw->HasReader()) ||
      (map && !map->HasReader()) ||
      (raw_map && !raw_map->HasReader()) ||
      (reset && !reset->HasReader()) || (ack && !ack->HasReader()) ||
      !reader->HasWriter() || !completion->HasWriter())
    throw std::runtime_error("recorded global replay discovery timeout");
  std::ifstream stream(input);
  if (!stream) throw std::runtime_error("cannot open prepared global events");
  std::string line;
  std::size_t count = 0;
  std::size_t completed = 0, map_tasks = 0;
  std::map<std::pair<std::string, uint64_t>, std::size_t> occurrences;
  std::vector<std::pair<uint64_t, int64_t>> pending_maps;
  int64_t odometry_frontier = 0, map_reference = 0;
  while (std::getline(stream, line)) {
    if (++count > 100000)
      throw std::runtime_error("recorded global replay exceeds bounded window");
    const auto split = line.find(' ');
    if (split == std::string::npos)
      throw std::runtime_error("expected odom/gnss/raw_gga/map/raw_map/reset/reset_ack JSON");
    const auto kind = line.substr(0, split);
    uint64_t sequence;
    if (kind == "odom") {
      auto message = Parse<host::NativeOdometryDiagnostic>(line.substr(split + 1));
      if (message.usable() && message.new_measurement())
        odometry_frontier =
            host::NativeMapMatcher::Nanoseconds(message.measurement_time());
      message.set_publication_time(apollo::cyber::Time::Now().ToSecond());
      sequence = message.publication_sequence();
      if (!odometry->Write(message))
        throw std::runtime_error("recorded ODOM transport failed");
    } else if (kind == "gnss") {
      if (!gnss) throw std::runtime_error("prepared GNSS not enabled");
      const auto message = Parse<host::NativeGlobalGnssInput>(line.substr(split + 1));
      sequence = message.sequence();
      if (!gnss->Write(message))
        throw std::runtime_error("recorded GNSS transport failed");
    } else if (kind == "raw_gga") {
      if (!raw) throw std::runtime_error("raw GGA not enabled");
      const auto message =
          Parse<apollo::drivers::gnss::RawData>(line.substr(split + 1));
      sequence = message.header().sequence_num();
      if (!raw->Write(message))
        throw std::runtime_error("recorded raw GGA transport failed");
    } else if (kind == "map") {
      if (!map) throw std::runtime_error("recorded map input is not enabled");
      const auto message = Parse<host::NativeGlobalMapInput>(line.substr(split + 1));
      sequence = message.sequence();
      if (!map->Write(message))
        throw std::runtime_error("recorded map transport failed");
    } else if (kind == "raw_map") {
      if (!raw_map) throw std::runtime_error("raw map is not enabled");
      const auto message = Parse<apollo::drivers::PointCloud>(line.substr(split + 1));
      sequence = message.header().sequence_num();
      map_reference = host::NativeMapMatcher::Reference(message);
      if (!raw_map->Write(message))
        throw std::runtime_error("recorded raw map transport failed");
    } else if (kind == "reset") {
      if (!reset) throw std::runtime_error("global reset not enabled");
      const auto message = Parse<host::NativeGlobalResetRequest>(line.substr(split + 1));
      sequence = message.sequence();
      if (!reset->Write(message))
        throw std::runtime_error("recorded reset transport failed");
    } else if (kind == "reset_ack") {
      if (!ack) throw std::runtime_error("reset ACK not enabled");
      const auto message = Parse<host::NativeGlobalResetAck>(line.substr(split + 1));
      sequence = message.sequence();
      if (!ack->Write(message))
        throw std::runtime_error("recorded reset ACK transport failed");
    } else {
      throw std::runtime_error("unknown recorded global event type");
    }
    ++completed;
    const auto occurrence = ++occurrences[{kind, sequence}];
    std::unique_lock<std::mutex> lock(received->mutex);
    const auto matches = [&] {
      return std::count_if(received->processed.begin(), received->processed.end(),
          [&](const auto& p) { return p.kind() == kind && p.sequence() == sequence; });
    };
    if (!received->condition.wait_for(lock, std::chrono::seconds(5), [&] {
          return static_cast<std::size_t>(matches()) == occurrence;
        })) {
      lock.unlock();
      global.Check();
      throw std::runtime_error("recorded global input processing timeout");
    }
    const auto response = std::find_if(received->processed.rbegin(),
        received->processed.rend(), [&](const auto& p) {
          return p.kind() == kind && p.sequence() == sequence;
        });
    if (kind == "raw_map" && response->status() ==
        static_cast<int>(::openloc::localization::StatusCode::kPending)) {
      ++map_tasks;
      ++completed;
      pending_maps.emplace_back(sequence, map_reference);
    }
    for (auto it = pending_maps.begin(); it != pending_maps.end();) {
      if (odometry_frontier < it->second) { ++it; continue; }
      if (!received->condition.wait_for(lock, std::chrono::seconds(5), [&] {
            return std::any_of(received->processed.begin(), received->processed.end(),
                [&](const auto& p) {
                  return p.kind() == "map_match" && p.sequence() == it->first;
                });
          })) {
        lock.unlock();
        global.Check();
        throw std::runtime_error("recorded raw map task completion timeout");
      }
      it = pending_maps.erase(it);
    }
    lock.unlock();
    global.Check();
  }
  if (!stream.eof() || count == 0)
    throw std::runtime_error("empty/incomplete global replay events");
  {
    std::unique_lock<std::mutex> lock(received->mutex);
    if (!received->condition.wait_for(lock, std::chrono::seconds(5), [&] {
          return !received->outputs.empty() &&
              received->outputs.back().processed_events() == count &&
              received->outputs.back().map_tasks_completed() == map_tasks &&
              received->processed.size() == completed;
        }))
      throw std::runtime_error("final global diagnostic publication missing");
  }
  global.Finish();
  std::lock_guard<std::mutex> lock(received->mutex);
  if (received->processed.size() != completed)
    throw std::runtime_error("global replay completion accounting mismatch");
  Write(output / "output.jsonl", received->outputs);
  Write(output / "processed.jsonl", received->processed);
  std::cout << "processed " << count << " recorded events; "
            << received->outputs.size() << " diagnostic publications\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: native_global_cyber_replay PROFILE.pb.txt EVENTS.jsonl "
                 "NEW_OUTPUT\n";
    return 1;
  }
  bool initialized = false;
  try {
    std::ifstream file(argv[1]);
    const std::string text((std::istreambuf_iterator<char>(file)), {});
    host::NativeGlobalHostConfig config;
    if (!file || !google::protobuf::TextFormat::ParseFromString(text, &config))
      throw std::runtime_error("unreadable recorded global profile");
    if (!apollo::cyber::Init(argv[0]))
      throw std::runtime_error("global replay Cyber initialization failed");
    initialized = true;
    Replay(config, argv[2], argv[3]);
    apollo::cyber::Clear();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "OpenLoc global rehost failed: " << error.what() << '\n';
    if (initialized) apollo::cyber::Clear();
    return 1;
  }
}
