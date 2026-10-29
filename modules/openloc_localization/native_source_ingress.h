// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <functional>
#include <optional>
#include <string>

namespace apollo::localization::openloc_diagnostic {

// The host serializes admission and failure recording with its input mutex.
// Sequence continuity measures producer-to-callback loss, not its location.
class NativeSourceIngress {
 public:
  enum class Stream { kImu, kLidar, kWorker, kPublisher };
  NativeSourceIngress(const std::string& path, bool require_source_sequences);
  void Admit(Stream stream, std::optional<uint32_t> sequence, double receive,
             const std::function<void()>& enqueue);
  void Failure(Stream stream, double receive, std::exception_ptr error);

 private:
  struct State {
    std::optional<uint32_t> last;
    uint64_t admitted = 0;
  };
  void Record(Stream stream, double receive, std::optional<uint32_t> sequence,
              const char* outcome, const std::string& detail);
  const bool strict_;
  std::array<State, 4> states_;
  std::ofstream journal_;
};

}  // namespace apollo::localization::openloc_diagnostic
