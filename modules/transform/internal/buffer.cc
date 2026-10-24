// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/transform/buffer.h"

#include <memory>
#include <string>

#include "tf2/exceptions.h"

#include "cyber/common/log.h"
#include "modules/transform/internal/buffer_impl.h"

namespace apollo {
namespace transform {

Buffer::Buffer() : Buffer(true) {}

Buffer::Buffer(bool initialize_runtime) : impl_(std::make_unique<Impl>()) {
  if (initialize_runtime) {
    CHECK_EQ(Init(), cyber::SUCC);
  }
}

Buffer::~Buffer() = default;

int Buffer::Init() { return impl_->Init(); }

TransformStamped Buffer::lookupTransform(const std::string& target_frame,
                                         const std::string& source_frame,
                                         const cyber::Time& time,
                                         float timeout) const {
  try {
    return impl_->lookupTransform(target_frame, source_frame, time, timeout);
  } catch (const tf2::TransformException& error) {
    throw TransformException(error.what());
  }
}

TransformStamped Buffer::lookupTransform(const std::string& target_frame,
                                         const cyber::Time& target_time,
                                         const std::string& source_frame,
                                         const cyber::Time& source_time,
                                         const std::string& fixed_frame,
                                         float timeout) const {
  try {
    return impl_->lookupTransform(target_frame, target_time, source_frame,
                                  source_time, fixed_frame, timeout);
  } catch (const tf2::TransformException& error) {
    throw TransformException(error.what());
  }
}

bool Buffer::canTransform(const std::string& target_frame,
                          const std::string& source_frame,
                          const cyber::Time& time, float timeout,
                          std::string* error) const {
  return impl_->canTransform(target_frame, source_frame, time, timeout, error);
}

bool Buffer::canTransform(const std::string& target_frame,
                          const cyber::Time& target_time,
                          const std::string& source_frame,
                          const cyber::Time& source_time,
                          const std::string& fixed_frame, float timeout,
                          std::string* error) const {
  return impl_->canTransform(target_frame, target_time, source_frame,
                             source_time, fixed_frame, timeout, error);
}

bool Buffer::GetLatestStaticTransform(const std::string& target_frame,
                                      const std::string& source_frame,
                                      TransformStamped* transform) const {
  return impl_->GetLatestStaticTransform(target_frame, source_frame, transform);
}

TransformBufferDiagnostics Buffer::GetDiagnosticsSnapshot() const {
  return impl_->GetDiagnosticsSnapshot();
}

uint64_t Buffer::GetTimeEpoch() { return impl_->GetTimeEpoch(); }

void Buffer::clear() { impl_->clear(); }

}  // namespace transform
}  // namespace apollo
