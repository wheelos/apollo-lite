#pragma once

#include <string>
#include <utility>

namespace apollo {
namespace open_space_planning {

enum class StatusCode {
  kOk = 0,
  kInvalidInput,
  kNotReady,
  kRouteSearchFailed,
  kTrajectoryGenerationFailed,
  kSafetyValidationFailed,
  kFallbackFailed,
  kInternalError,
};

class Status {
 public:
  Status() = default;
  Status(StatusCode code, std::string message)
      : code_(code), message_(std::move(message)) {}

  static Status Ok() { return Status(); }

  bool ok() const { return code_ == StatusCode::kOk; }
  StatusCode code() const { return code_; }
  const std::string& message() const { return message_; }

 private:
  StatusCode code_ = StatusCode::kOk;
  std::string message_;
};

}  // namespace open_space_planning
}  // namespace apollo

