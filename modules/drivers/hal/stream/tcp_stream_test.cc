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

#include "modules/drivers/hal/stream/tcp_stream.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <future>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "modules/drivers/hal/stream/stream_factory.h"

namespace apollo {
namespace drivers {
namespace hal {

TEST(TcpStreamIntegrationTest, ConnectAndEchoData) {
  const uint16_t test_port = 54322;  // Use a port different from the UDP test
  const std::string loopback_addr = "127.0.0.1";
  const uint32_t timeout_ms = 1000;

  // For synchronization: ensure the server thread is ready to accept
  // connections
  std::promise<void> server_ready_promise;
  auto server_ready_future = server_ready_promise.get_future();

  // For synchronization: mark that the server thread should exit
  std::atomic<bool> server_should_stop(false);

  // 1. Start the server thread (this is a simple Echo server)
  std::thread server_thread([&]() {
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_NE(listen_fd, -1);

    // Allow port reuse to prevent port occupation after test failures
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(test_port);

    ASSERT_NE(
        bind(listen_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)),
        -1);
    ASSERT_NE(listen(listen_fd, 1), -1);

    // Notify main thread: server is ready, can connect now
    server_ready_promise.set_value();

    // Wait for client connection
    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int client_fd =
        accept(listen_fd, (struct sockaddr*)&client_addr, &client_len);

    // If accept succeeds, enter echo loop
    if (client_fd != -1) {
      std::cout << "[Server Thread] Accepted connection. Entering echo loop."
                << std::endl;
      char buffer[1024];
      while (!server_should_stop.load()) {
        ssize_t bytes_read = recv(client_fd, buffer, sizeof(buffer), 0);
        if (bytes_read > 0) {
          std::cout << "[Server Thread] Received " << bytes_read
                    << " bytes. Echoing back." << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          send(client_fd, buffer, bytes_read, 0);
        } else {
          std::cout << "[Server Thread] recv returned " << bytes_read
                    << ". Closing connection." << std::endl;
          break;
        }
      }
      close(client_fd);
    } else {
      std::cerr << "[Server Thread] accept() failed with error: "
                << strerror(errno) << std::endl;
    }
    close(listen_fd);
    std::cout << "[Server Thread] Exiting." << std::endl;
  });

  // Ensure the server thread is ready
  auto ready_status = server_ready_future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(ready_status, std::future_status::ready)
      << "Server failed to start in time.";

  // 2. Client logic
  auto client =
      StreamFactory::CreateTcp(loopback_addr, test_port, timeout_ms * 1000);

  ASSERT_NE(client, nullptr);

  // 3. Connect to the server
  ASSERT_TRUE(client->Connect()) << "Client failed to connect to the server.";

  // 4. Send data
  const std::string payload = "TCP_Echo_Test_Packet_12345";
  const uint8_t* payload_data =
      reinterpret_cast<const uint8_t*>(payload.c_str());
  const size_t payload_size = payload.length();

  size_t bytes_written = client->write(payload_data, payload_size);
  ASSERT_EQ(bytes_written, payload_size);
  std::cout << "[Client Thread] Wrote " << bytes_written
            << " bytes. Waiting for echo." << std::endl;

  // 5. Receive echo data (must read in a loop since it's a stream protocol)
  std::vector<uint8_t> received_buffer;
  received_buffer.resize(payload_size);
  size_t total_bytes_read = 0;
  while (total_bytes_read < payload_size) {
    size_t bytes_read = client->read(received_buffer.data() + total_bytes_read,
                                     payload_size - total_bytes_read);
    ASSERT_GT(bytes_read, 0)
        << "Read failed or timed out while waiting for echo.";
    total_bytes_read += bytes_read;
  }

  // 6. Verify data
  ASSERT_EQ(total_bytes_read, payload_size);
  ASSERT_EQ(0, memcmp(received_buffer.data(), payload_data, payload_size));

  // 7. Cleanup
  client->Disconnect();
  server_should_stop = true;  // Notify the server thread to exit the loop
  server_thread.join();
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
