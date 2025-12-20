#include <filesystem>
#include <string>
#include <vector>

#include <benchmark/benchmark.h>

#include "cyber/proto/record.pb.h"

#include "cyber/record/file/record_file_writer.h"

namespace apollo {
namespace cyber {
namespace record {

proto::SingleMessage CreateDummyMessage(size_t size_kb) {
  proto::SingleMessage msg;
  msg.set_channel_name("/apollo/sensor/camera/front_60");
  msg.set_time(Time::Now().ToNanosecond());
  msg.set_content(std::string(size_kb * 1024, 'x'));  // Fill data
  return msg;
}

static void BM_RecordWritePerformance(benchmark::State& state) {
  const size_t message_size_kb = state.range(0);
  const std::string test_file =
      "test_perf_" + std::to_string(message_size_kb) + "kb.record";

  RecordFileWriter writer;
  if (!writer.Open(test_file)) {
    state.SkipWithError("Could not open file for writing");
    return;
  }

  // Write to Header and Channel
  proto::Header header;
  header.set_is_complete(false);
  writer.WriteHeader(header);

  proto::Channel channel;
  channel.set_name("/apollo/sensor/camera/front_60");
  channel.set_message_type("apollo.drivers.Image");
  writer.WriteChannel(channel);

  auto msg = CreateDummyMessage(message_size_kb);
  size_t total_bytes = 0;

  // Performance test core loop
  for (auto _ : state) {
    if (!writer.WriteMessage(msg)) {
      state.SkipWithError("WriteMessage failed");
      break;
    }
    total_bytes += msg.ByteSizeLong();
  }

  writer.Close();

  // Statistics
  state.SetBytesProcessed(static_cast<int64_t>(total_bytes));
  state.SetLabel("MsgSize_" + std::to_string(message_size_kb) + "KB");

  // Clean up test files.
  std::filesystem::remove(test_file);
}

// Register test cases of different sizes
// 1: 1KB (IMU/Control)
// 64: 64KB (Lidar Clusters)
// 4096: 4MB (HD Camera Frame)
BENCHMARK(BM_RecordWritePerformance)
    ->Arg(1)
    ->Arg(64)
    ->Arg(4096)
    ->Unit(benchmark::kMillisecond);

}  // namespace record
}  // namespace cyber
}  // namespace apollo

BENCHMARK_MAIN();
