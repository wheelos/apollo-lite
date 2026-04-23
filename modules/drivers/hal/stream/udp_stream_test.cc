// Copyright 2025 WheelOS All Rights Reserved.
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

//  Created Date: 2025-11-10
//  Author: daohu527

#include <future>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "modules/drivers/hal/stream/stream_factory.h"

namespace apollo {
namespace drivers {
namespace hal {

// Integration testing to verify UDP communication
TEST(UdpStreamIntegrationTest, SendAndReceiveOnLoopback) {
  const uint16_t test_port = 54321;
  const std::string loopback_addr = "127.0.0.1";
  const uint32_t timeout_ms = 500;

  // 1. Prepare data
  const std::string payload = "MagicPacket_IntegrationTest_!@#$";
  const uint8_t* payload_data =
      reinterpret_cast<const uint8_t*>(payload.c_str());
  const size_t payload_size = payload.length();

  // 2. Create receiver
  auto receiver =
      StreamFactory::CreateUdp("0.0.0.0", test_port, timeout_ms * 1000);
  ASSERT_NE(receiver, nullptr);
  ASSERT_TRUE(receiver->Connect());

  // 3. Start receiving in a new thread
  std::promise<std::vector<uint8_t>> received_data_promise;
  auto received_data_future = received_data_promise.get_future();

  std::promise<void> receiver_ready_promise;
  auto receiver_ready_future = receiver_ready_promise.get_future();

  std::thread receiver_thread([&]() {
    // Notify the main thread: I have started and am about to enter a blocking
    // read state.
    receiver_ready_promise.set_value();

    uint8_t buffer[1024];
    size_t bytes_read = receiver->read(buffer, sizeof(buffer));

    if (bytes_read > 0) {
      received_data_promise.set_value(
          std::vector<uint8_t>(buffer, buffer + bytes_read));
    } else {
      // If the read fails or times out, set an empty value.
      try {
        received_data_promise.set_value({});
      } catch (const std::future_error& e) {
        // Ignore the exception, because the promise may have already been set
        // to an exception by the main thread's timeout logic.
      }
    }
  });

  auto ready_status = receiver_ready_future.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(ready_status, std::future_status::ready)
      << "Receiver thread failed to get ready in time.";

  // 4. Create a sender and send data
  auto sender =
      StreamFactory::CreateUdp(loopback_addr, test_port, timeout_ms * 1000);
  ASSERT_NE(sender, nullptr);
  ASSERT_TRUE(sender->Connect());
  size_t bytes_sent = sender->write(payload_data, payload_size);
  ASSERT_EQ(bytes_sent, payload_size);

  // 5. Wait for the result, set a timeout.
  auto future_status =
      received_data_future.wait_for(std::chrono::milliseconds(timeout_ms));
  ASSERT_EQ(future_status, std::future_status::ready)
      << "Test timed out. Packet was not received.";

  // 6. Verification results
  std::vector<uint8_t> received_data;
  try {
    received_data = received_data_future.get();
  } catch (const std::exception& e) {
    FAIL() << "Failed to get data from future: " << e.what();
  }

  ASSERT_EQ(received_data.size(), payload_size);
  ASSERT_EQ(0, memcmp(received_data.data(), payload_data, payload_size));

  // 7. Clear
  receiver_thread.join();
  receiver->Disconnect();
  sender->Disconnect();
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
