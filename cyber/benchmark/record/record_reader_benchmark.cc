#include "record_reader.h"  // 示例模块

#include <benchmark/benchmark.h>

static void BM_ReadRecord(benchmark::State &state) {
  RecordReader reader("path_to_record_file");
  for (auto _ : state) {
    reader.Read();  // 假设有一个 Read() 方法
  }
}

// 将基准程序注册到 Google Benchmark
BENCHMARK(BM_ReadRecord)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
