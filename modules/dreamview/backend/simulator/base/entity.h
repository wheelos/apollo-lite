#pragma once

namespace apollo {
namespace dreamview {

class SimEntity {
 public:
  virtual ~SimEntity() = default;

  virtual bool Init() = 0;

  // dt: delta time in seconds
  virtual void Step(double dt) = 0;

  // Reset the entity to its initial configuration
  virtual void Reset() = 0;

  // Publish resulting data/messages to Cyber RT channels
  virtual void Publish() = 0;
};

}  // namespace dreamview
}  // namespace apollo
