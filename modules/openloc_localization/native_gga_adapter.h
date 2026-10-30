// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "native_global_diagnostic.pb.h"
#include "openloc/localization/online/gnss_position_gate.h"

namespace apollo::localization::openloc_diagnostic {

struct NativeGgaDecision {
  uint64_t sequence = 0;
  int64_t measurement_ns = 0;
  int64_t receive_ns = 0;
  ::openloc::localization::Status status;
  std::optional<NativeGlobalGnssInput> observation;
};

// Bounded NMEA framing, checksum, UTC GGA time and declared local UTM XY.
// All methods run on the existing global worker; no second global owner.
class NativeGgaAdapter {
 public:
  using Callback = std::function<void(const NativeGgaDecision&)>;
  explicit NativeGgaAdapter(const NativeGlobalHostConfig& config);
  ~NativeGgaAdapter();
  void Reset(const std::string& session, uint64_t generation,
             int64_t minimum_measurement_ns = 0);
  void Feed(const std::string& bytes, int64_t receive_ns, Callback callback);
  void Finish() const;

 private:
  class Projection;
  NativeGgaDecision Parse(const std::string& sentence, int64_t receive_ns);
  NativeGlobalHostConfig config_;
  ::openloc::localization::GnssPositionGate gate_;
  std::unique_ptr<Projection> projection_;
  std::string buffer_;
  std::string session_;
  uint64_t generation_ = 0;
  uint64_t sequence_ = 0;
  int64_t last_receive_ns_ = 0;
  int64_t minimum_measurement_ns_ = 0;
};

}  // namespace apollo::localization::openloc_diagnostic
