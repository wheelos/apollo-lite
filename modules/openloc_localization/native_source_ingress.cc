// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_source_ingress.h"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <stdexcept>

namespace apollo::localization::openloc_diagnostic {

NativeSourceIngress::NativeSourceIngress(const std::string& path, bool strict)
    : strict_(strict) {
  if (std::filesystem::exists(path))
    throw std::invalid_argument("source ingress journal must not already exist");
  journal_.open(path);
  if (!journal_) throw std::runtime_error("cannot open source ingress journal");
  journal_ << std::setprecision(17)
           << "stream,receive_s,source_sequence,admitted,sequence_checked,outcome,detail\n";
  journal_.flush();
  if (!journal_) throw std::runtime_error("cannot write source ingress header");
}

void NativeSourceIngress::Admit(
    Stream stream, std::optional<uint32_t> sequence, double receive,
    const std::function<void()>& enqueue) {
  auto& state = states_.at(static_cast<size_t>(stream));
  if ((stream != Stream::kImu && stream != Stream::kLidar) ||
      !std::isfinite(receive) || receive <= 0 || !enqueue)
    throw std::invalid_argument("invalid source ingress receive time/admission");
  if (strict_) {
    if (!sequence)
      throw std::invalid_argument("strict live source requires producer header sequence");
    if (state.last) {
      const uint32_t delta = *sequence - *state.last;
      if (delta != 1) {
        const auto issue = delta == 0 ? "duplicate" :
            delta < 0x80000000U ? "gap" : "regression";
        throw std::runtime_error(
            std::string("live source sequence ") + issue + ": previous=" +
            std::to_string(*state.last) + " received=" + std::to_string(*sequence) +
            (delta > 1 && delta < 0x80000000U
                 ? " missing=" + std::to_string(delta - 1) : ""));
      }
    }
  }
  enqueue();
  const bool baseline = !state.last && sequence.has_value();
  state.last = sequence;
  ++state.admitted;
  Record(stream, receive, sequence, baseline ? "baseline_admitted" : "admitted",
         strict_ ? "producer continuity checked after first observed baseline"
                 : "producer continuity unqualified");
}

void NativeSourceIngress::Failure(
    Stream stream, double receive, std::exception_ptr error) {
  if (!error) throw std::invalid_argument("source ingress failure needs exception");
  std::string detail;
  try {
    std::rethrow_exception(error);
  } catch (const std::exception& value) {
    detail = value.what();
  } catch (...) {
    detail = "non-standard source admission exception";
  }
  Record(stream, receive, std::nullopt, "fatal", detail);
}

void NativeSourceIngress::Record(
    Stream stream, double receive, std::optional<uint32_t> sequence,
    const char* outcome, const std::string& detail) {
  constexpr std::array<const char*, 4> names = {"imu", "lidar", "worker", "publisher"};
  journal_ << names.at(static_cast<size_t>(stream)) << ',' << receive << ',';
  if (sequence) journal_ << *sequence;
  journal_ << ',' << states_.at(static_cast<size_t>(stream)).admitted << ','
           << strict_ << ',' << outcome << ',' << std::quoted(detail) << '\n';
  journal_.flush();
  if (!journal_) throw std::runtime_error("incomplete source ingress journal");
}

}  // namespace apollo::localization::openloc_diagnostic
