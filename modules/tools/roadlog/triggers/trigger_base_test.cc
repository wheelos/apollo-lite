#include "modules/tools/roadlog/triggers/trigger_base.h"

#include "gtest/gtest.h"

namespace apollo {
namespace data {
namespace {

class FakeTrigger : public TriggerBase {
 public:
  FakeTrigger() { trigger_name_ = "FakeTrigger"; }

  void Pull(const cyber::record::RecordMessage& msg) override {
    last_msg_time_ = msg.time;
  }

  uint64_t last_msg_time() const { return last_msg_time_; }

 private:
  uint64_t last_msg_time_ = 0;
};

TEST(TriggerBaseTest, MissingTriggerConfigDefaultsToDisabled) {
  SmartRecordTrigger trigger_conf;
  FakeTrigger trigger;

  EXPECT_TRUE(trigger.Init(trigger_conf, [](const TriggerEvent&) {}));
  EXPECT_FALSE(trigger.IsEnabled());
}

}  // namespace
}  // namespace data
}  // namespace apollo
