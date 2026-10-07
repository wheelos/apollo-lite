#pragma once

#include <string>

namespace apollo {
namespace dreamview {

class Metric {
 public:
  virtual ~Metric() = default;
  virtual std::string Name() const = 0;

  // 使用统一的 Context
  virtual void Evaluate(const FrameContext& context) = 0;
  virtual void Reset() = 0;

  virtual bool IsTerminal() const = 0;
  virtual std::string TerminalReason() const = 0;

  virtual double GetValue() const { return 0.0; }
};

}  // namespace dreamview
}  // namespace apollo
