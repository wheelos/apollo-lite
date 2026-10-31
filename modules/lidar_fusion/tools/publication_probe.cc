// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cyber/cyber.h"
#include "modules/lidar_fusion/publication/pod.h"

namespace fs = std::filesystem;
namespace fusion = apollo::lidar_fusion;
namespace {
using Clock = std::chrono::steady_clock;
constexpr char kBootstrapFrame[] = "lidar_fusion_publication_probe_bootstrap";

void Require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

uint64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Clock::now().time_since_epoch())
      .count();
}

double CpuMs() {
  rusage value{};
  Require(getrusage(RUSAGE_SELF, &value) == 0, "cannot read CPU time");
  return (value.ru_utime.tv_sec + value.ru_stime.tv_sec) * 1000.0 +
         (value.ru_utime.tv_usec + value.ru_stime.tv_usec) / 1000.0;
}

template <typename Predicate>
void Wait(Predicate predicate) {
  const auto deadline = Clock::now() + std::chrono::seconds(15);
  while (!predicate()) {
    Require(Clock::now() < deadline, "publication/discovery timeout");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void Save(const fs::path& path, const std::string& contents) {
  Require(!fs::exists(path), "receipt already exists: " + path.string());
  const fs::path temporary = path.string() + ".tmp";
  Require(!fs::exists(temporary), "temporary receipt already exists");
  {
    std::ofstream stream(temporary);
    stream << contents;
    Require(stream.good(), "cannot save receipt");
  }
  fs::rename(temporary, path);
}

int VerifyCodec(const fs::path& manifest, const fs::path& receipt) {
  Require(!fs::exists(receipt), "codec receipt already exists");
  std::ifstream inputs(manifest);
  Require(inputs.good(), "cannot read codec input manifest");
  std::ofstream csv(receipt);
  csv << "index,file,bytes,fragments,repeats\n";
  std::string filename;
  std::size_t count = 0;
  while (std::getline(inputs, filename)) {
    Require(!filename.empty(), "empty codec manifest entry");
    fusion::FusionOutput frame;
    std::ifstream file(filename, std::ios::binary);
    Require(file.good() && frame.ParseFromIstream(&file),
            "invalid codec input: " + filename);
    const auto expected = frame.SerializeAsString();
    const auto limit = fusion::PublicationOptions().max_pod_frame_bytes();
    Require(!expected.empty() && expected.size() <= limit,
            "codec input exceeds byte budget");
    std::size_t parts = 0;
    for (int repeat = 0; repeat < 2; ++repeat) {
      std::vector<std::shared_ptr<std::vector<uint8_t>>> storage;
      std::vector<fusion::FusionPodBuffer> buffers;
      for (std::size_t offset = 0; offset < expected.size();
           offset += fusion::kFusionPodFragmentBytes) {
        const auto size =
            sizeof(apollo::cyber::transport::PodChunkHeader) +
            std::min(fusion::kFusionPodFragmentBytes, expected.size() - offset);
        storage.push_back(std::make_shared<std::vector<uint8_t>>(size));
        buffers.push_back({storage.back()->data(), size});
      }
      std::string error;
      Require(fusion::EncodeFusionPod(frame, 1, count + 1, buffers, &error),
              error);
      fusion::FusionPodAssembler assembler(limit);
      fusion::FusionOutput decoded;
      for (std::size_t i = 0; i < storage.size(); ++i) {
        auto part = std::make_shared<apollo::cyber::transport::PodMessage>();
        Require(part->BorrowFromArray(storage[i]->data(), storage[i]->size(),
                                      storage[i]),
                "cannot borrow codec fixture");
        const auto result = assembler.Push(part, &decoded, &error);
        Require(result == (i + 1 == storage.size()
                               ? fusion::PodAssemblyResult::COMPLETE
                               : fusion::PodAssemblyResult::INCOMPLETE),
                error);
      }
      Require(decoded.SerializeAsString() == expected,
              "complete POD codec bytes differ: " + filename);
      parts = storage.size();
    }
    csv << count++ << ',' << filename << ',' << expected.size() << ',' << parts
        << ",2\n";
    csv.flush();
    Require(csv.good(), "cannot save codec receipt");
  }
  Require(count > 0 && inputs.eof(), "empty or unreadable codec manifest");
  std::cout
      << "codec payloads=" << count
      << " repeats=2; heap-backed fixture views, not runtime loan proof\n";
  return 0;
}

int Run(int argc, char** argv) {
  bool use_pod = false, pressure = false;
  while (argc > 1) {
    const std::string option(argv[argc - 1]);
    if (option == "--core-pod") {
      use_pod = true;
    } else if (option == "--pool-pressure") {
      pressure = true;
    } else {
      break;
    }
    --argc;
  }
  Require(argc == 6 || argc == 8,
          "usage: publication_probe receive EXPECTED CHANNEL RECEIPTS COUNT | "
          "send EXPECTED CHANNEL RECEIPTS WARMUPS REPEATS NEW_CSV "
          "[--core-pod] [--pool-pressure]");
  const std::string operation(argv[1]), channel(argv[3]);
  const fs::path receipts(argv[4]);
  Require(!pressure || (use_pod && operation == "send"),
          "pool pressure requires a POD sender");
  const char* ip = std::getenv("CYBER_IP");
  Require(ip != nullptr && std::string(ip) == "127.0.0.1" &&
              std::getenv("CYBER_DOMAIN_ID") != nullptr &&
              channel.rfind("/test/lidar_fusion_publication/", 0) == 0,
          "probe requires explicit loopback, isolated domain and test channel");
  Require(fs::is_directory(receipts), "receipt directory must already exist");
  auto frame = std::make_shared<fusion::FusionOutput>();
  std::ifstream input(argv[2], std::ios::binary);
  Require(input.good() && frame->ParseFromIstream(&input),
          "invalid expected PB");
  const std::string expected = frame->SerializeAsString();
  Require(frame->header().frame_id() != kBootstrapFrame,
          "expected frame must not be a bootstrap marker");
  auto bootstrap = std::make_shared<fusion::FusionOutput>();
  bootstrap->mutable_header()->set_frame_id(kBootstrapFrame);
  const std::string bootstrap_bytes = bootstrap->SerializeAsString();
  auto node =
      apollo::cyber::CreateNode(std::string("lidar_fusion_publication_") +
                                operation + "_" + std::to_string(getpid()));
  Require(node != nullptr, "cannot create probe node");
  using Pod = apollo::cyber::transport::PodMessage;
  const auto max_bytes = fusion::PublicationOptions().max_pod_frame_bytes();
  if (operation == "receive" && argc == 6) {
    const int count = std::stoi(argv[5]);
    Require(count > 0 && count <= 110, "invalid receive count");
    Require(fs::is_empty(receipts), "receipt directory must be new/empty");
    std::mutex mutex;
    std::atomic<int> received{0};
    std::atomic<bool> failed{false};
    const auto consume =
        [&](const std::shared_ptr<fusion::FusionOutput>& message,
            uint64_t arrival, uint64_t decoded, std::size_t parts,
            double decode_ms) {
          std::lock_guard<std::mutex> lock(mutex);
          const int index = received.load();
          try {
            if (message->header().frame_id() == kBootstrapFrame) {
              Require(message->SerializeAsString() == bootstrap_bytes,
                      "invalid bootstrap marker");
              if (!fs::exists(receipts / "bootstrap")) {
                Save(receipts / "bootstrap", "data-callback-ready\n");
              }
              return;
            }
            Require(index < count, "unexpected duplicate publication");
            Require(message->SerializeAsString() == expected,
                    "received complete bytes differ");
            Save(receipts / (std::to_string(index) + ".ack"),
                 std::to_string(arrival) + " " +
                     std::to_string(expected.size()) + " " +
                     std::to_string(decoded) + " " + std::to_string(parts) +
                     " " + std::to_string(decode_ms) + "\n");
            received.store(index + 1);
          } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
            failed.store(true);
          }
        };
    std::shared_ptr<apollo::cyber::Reader<fusion::FusionOutput>> reader;
    std::shared_ptr<apollo::cyber::Reader<Pod>> pod_reader;
    fusion::FusionPodAssembler assembler(max_bytes);
    std::vector<std::shared_ptr<Pod>> held, current;
    bool held_bootstrap = true;
    const auto verify_parts =
        [&](const std::vector<std::shared_ptr<Pod>>& parts,
            const std::string& bytes) {
          std::size_t offset = 0;
          for (const auto& part : parts) {
            apollo::cyber::transport::PodChunkView view;
            Require(
                part->is_borrowed() && apollo::cyber::transport::ParsePodChunk(
                                           part->data(), part->size(), &view),
                "POD callback is not a valid borrowed chunk");
            Require(
                offset <= bytes.size() &&
                    view.payload_size <= bytes.size() - offset &&
                    std::memcmp(view.payload, bytes.data() + offset,
                                view.payload_size) == 0,
                "borrowed payload changed while held or differs from source");
            offset += view.payload_size;
          }
          Require(offset == bytes.size(), "incomplete held POD frame");
        };
    if (use_pod) {
      apollo::cyber::ReaderConfig config;
      std::string error;
      Require(
          fusion::ConfigureFusionPodReader(channel, max_bytes, &config, &error),
          error);
      pod_reader = node->CreateReader<Pod>(
          config, [&](const std::shared_ptr<Pod>& message) {
            const uint64_t arrival = NowNs();
            try {
              Require(message->is_borrowed(),
                      "POD route copied the received message");
              current.push_back(message);
              fusion::FusionOutput output;
              std::string error;
              const auto result = assembler.Push(message, &output, &error);
              Require(result != fusion::PodAssemblyResult::REJECTED, error);
              if (result == fusion::PodAssemblyResult::INCOMPLETE) {
                return;
              }
              auto decoded =
                  std::make_shared<fusion::FusionOutput>(std::move(output));
              const uint64_t ready = NowNs();
              if (!held.empty()) {
                verify_parts(held, held_bootstrap ? bootstrap_bytes : expected);
              }
              const bool marker =
                  decoded->header().frame_id() == kBootstrapFrame;
              verify_parts(current, marker ? bootstrap_bytes : expected);
              const auto parts = current.size();
              held = std::move(current);
              current.clear();
              held_bootstrap = marker;
              consume(decoded, arrival, ready, parts, (ready - arrival) / 1e6);
            } catch (const std::exception& error) {
              std::cerr << error.what() << '\n';
              failed.store(true);
            }
          });
      Require(pod_reader != nullptr, "cannot create POD reader");
    } else {
      reader = node->CreateReader<fusion::FusionOutput>(
          channel, [&](const std::shared_ptr<fusion::FusionOutput>& message) {
            const uint64_t arrival = NowNs();
            consume(message, arrival, arrival, 0, 0);
          });
      Require(reader != nullptr, "cannot create probe reader");
    }
    Save(receipts / "layout.json",
         "{\"pod_header_bytes\":" +
             std::to_string(sizeof(apollo::cyber::transport::PodChunkHeader)) +
             ",\"fragment_payload_bytes\":" +
             std::to_string(fusion::kFusionPodFragmentBytes) + "}\n");
    Save(receipts / "communication.pb.txt",
         apollo::cyber::common::GlobalData::Instance()
             ->Config()
             .transport_conf()
             .communication_mode()
             .DebugString());
    Save(receipts / "ready", "reader-created\n");
    Wait([&] { return received.load() == count || failed.load(); });
    Require(!failed.load(), "receiver integrity check failed");
    // Keep the receiver-owned private RouDi alive until sender Clear finishes.
    Wait([&] { return fs::exists(receipts / "sender-finished"); });
    if (reader != nullptr) {
      reader->Shutdown();
    }
    if (pod_reader != nullptr) {
      pod_reader->Shutdown();
      pod_reader.reset();
      verify_parts(held, held_bootstrap ? bootstrap_bytes : expected);
      Save(receipts / "borrowed-lifetime",
           "retained_fragments=" + std::to_string(held.size()) +
               "\nunchanged_after_reader_shutdown_and_release=true\n");
      held.clear();
    }
    std::cout << "received=" << received.load()
              << " complete_payload_bytes=" << expected.size() << '\n';
    return 0;
  }
  Require(operation == "send" && argc == 8, "invalid probe operation");
  const int warmups = std::stoi(argv[5]), repeats = std::stoi(argv[6]);
  Require(warmups >= 0 && warmups <= 10 && repeats > 0 && repeats <= 100,
          "invalid sample count");
  Require(!fs::exists(argv[7]), "sample CSV already exists");
  Wait([&] { return fs::exists(receipts / "ready"); });
  std::shared_ptr<apollo::cyber::Writer<fusion::FusionOutput>> writer;
  std::shared_ptr<apollo::cyber::Writer<Pod>> pod_writer;
  std::unique_ptr<fusion::FusionPodPublisher> pod_publisher;
  if (use_pod) {
    pod_writer = node->CreateWriter<Pod>(channel);
    Require(pod_writer != nullptr, "cannot create POD writer");
    pod_publisher =
        std::make_unique<fusion::FusionPodPublisher>(pod_writer, max_bytes);
    Wait([&] { return pod_publisher->HasReader(); });
  } else {
    writer = node->CreateWriter<fusion::FusionOutput>(channel);
    Require(writer != nullptr, "cannot create probe writer");
    Wait([&] { return writer->HasReader(); });
  }
  const auto publish = [&](const std::shared_ptr<fusion::FusionOutput>& message,
                           fusion::PodPublicationWork* work) {
    if (pod_publisher == nullptr) {
      Require(writer->Write(message), "Cyber writer rejected frame");
    } else {
      std::string error;
      Require(pod_publisher->Publish(*message, &error, work) ==
                  fusion::PodPublishResult::PUBLISHED,
              "POD writer rejected frame: " + error);
    }
  };
  // Topology discovery does not imply a matched volatile RTPS data endpoint.
  const auto bootstrap_deadline = Clock::now() + std::chrono::seconds(15);
  int bootstrap_publications = 0;
  while (!fs::exists(receipts / "bootstrap")) {
    Require(Clock::now() < bootstrap_deadline, "data bootstrap timeout");
    publish(bootstrap, nullptr);
    ++bootstrap_publications;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cerr << "data bootstrap publications=" << bootstrap_publications << '\n';
  if (pressure) {
    using Loan = apollo::cyber::Writer<Pod>::LoanedMessage;
    std::vector<Loan> occupied;
    bool exhausted = false;
    for (int i = 0; i < 64; ++i) {
      Loan loan;
      if (!pod_writer->Loan(
              apollo::cyber::transport::kIceoryxChunkPayloadCapacity, &loan)) {
        exhausted = true;
        break;
      }
      occupied.push_back(std::move(loan));
    }
    Require(exhausted, "pool did not exhaust within the bounded private test");
    const auto fragments =
        (expected.size() + fusion::kFusionPodFragmentBytes - 1) /
        fusion::kFusionPodFragmentBytes;
    Require(fragments > 1 && occupied.size() >= fragments,
            "partial-loan test requires a multi-fragment frame");
    for (std::size_t i = 1; i < fragments; ++i) {
      occupied.pop_back();
    }
    std::string error;
    Require(pod_publisher->Publish(*frame, &error) ==
                fusion::PodPublishResult::REJECTED,
            "pool pressure was not rejected");
    Save(receipts / "pool-pressure",
         "held_loans=" + std::to_string(occupied.size()) +
             "\nfree_chunks_before_rejected_frame=" +
             std::to_string(fragments - 1) + "\nrejected=" + error + "\n");
    occupied.clear();
  }
  std::ofstream csv(argv[7]);
  csv << "iteration,warmup,bytes,write_wall_ms,writer_process_cpu_ms,"
         "publish_to_callback_ms,publish_to_decoded_callback_ms,fragments,"
         "size_ms,loan_ms,encode_ms,publish_ms,decode_ms\n"
      << std::setprecision(12);
  for (int i = 0; i < warmups + repeats; ++i) {
    const fs::path receipt = receipts / (std::to_string(i) + ".ack");
    Require(!fs::exists(receipt), "unexpected receipt before publication");
    const double cpu = CpuMs();
    const uint64_t begin = NowNs();
    fusion::PodPublicationWork work;
    publish(frame, &work);
    const uint64_t end = NowNs();
    const double cpu_ms = CpuMs() - cpu;
    Wait([&] { return fs::exists(receipt); });
    uint64_t arrival = 0, bytes = 0, decoded = 0;
    std::size_t parts = 0;
    double decode_ms = 0;
    std::ifstream ack(receipt);
    ack >> arrival >> bytes >> decoded >> parts >> decode_ms;
    Require(ack.good() && arrival >= begin && decoded >= arrival &&
                bytes == expected.size() && parts == work.fragments,
            "invalid receipt/clock/byte count");
    csv << i << ',' << (i < warmups) << ',' << bytes << ','
        << (end - begin) / 1e6 << ',' << cpu_ms << ','
        << (arrival - begin) / 1e6 << ',' << (decoded - begin) / 1e6 << ','
        << parts << ',' << work.size_ms << ',' << work.loan_ms << ','
        << work.encode_ms << ',' << work.publish_ms << ',' << decode_ms << '\n';
    csv.flush();
    Require(csv.good(), "cannot save publication sample");
  }
  if (writer != nullptr) {
    writer->Shutdown();
  }
  if (pod_writer != nullptr) {
    pod_writer->Shutdown();
  }
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 4 && std::string(argv[1]) == "codec-matrix") {
    try {
      return VerifyCodec(argv[2], argv[3]);
    } catch (const std::exception& error) {
      std::cerr << "Codec verification stopped: " << error.what() << '\n';
      return 1;
    }
  }
  const char* ip = std::getenv("CYBER_IP");
  if (ip == nullptr || std::string(ip) != "127.0.0.1" ||
      std::getenv("CYBER_DOMAIN_ID") == nullptr) {
    std::cerr << "Explicit loopback and isolated CYBER_DOMAIN_ID required\n";
    return 1;
  }
  apollo::cyber::Init(argv[0]);
  int result = 1;
  try {
    result = Run(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << "Publication probe stopped: " << error.what() << '\n';
  }
  apollo::cyber::Clear();
  if (result == 0 && argc > 4 && std::string(argv[1]) == "send") {
    try {
      Save(fs::path(argv[4]) / "sender-finished", "sender-cleared\n");
    } catch (const std::exception& error) {
      std::cerr << error.what() << '\n';
      result = 1;
    }
  }
  return result;
}
