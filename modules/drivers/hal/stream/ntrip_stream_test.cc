#include "modules/drivers/hal/stream/ntrip_stream.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include <gtest/gtest.h>

namespace apollo {
namespace drivers {
namespace hal {
namespace {

bool SendAll(int fd, const std::string& data) {
  size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t count =
        send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
    if (count <= 0) {
      return false;
    }
    sent += static_cast<size_t>(count);
  }
  return true;
}

int CreateListener(uint16_t* port) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ||
      listen(fd, 1) < 0) {
    close(fd);
    return -1;
  }

  socklen_t address_size = sizeof(address);
  if (getsockname(fd, reinterpret_cast<sockaddr*>(&address), &address_size) <
      0) {
    close(fd);
    return -1;
  }
  *port = ntohs(address.sin_port);
  return fd;
}

}  // namespace

TEST(NtripStreamRegressionTest, RetainsCorrectionBytesAfterHandshakeHeader) {
  uint16_t port = 0;
  const int listener_fd = CreateListener(&port);
  ASSERT_GE(listener_fd, 0);

  const std::string correction_bytes("\xD3\x00\x02", 3);
  const std::string response =
      "ICY 200 OK\r\nServer: local-test\r\n\r\n" + correction_bytes;
  std::atomic<bool> server_sent_response{false};
  std::thread server([&]() {
    const int client_fd = accept(listener_fd, nullptr, nullptr);
    if (client_fd >= 0) {
      std::array<char, 512> request{};
      recv(client_fd, request.data(), request.size(), 0);
      server_sent_response = SendAll(client_fd, response);
      while (recv(client_fd, request.data(), request.size(), 0) > 0) {
      }
      close(client_fd);
    }
  });

  bool connected = false;
  size_t bytes_read = 0;
  std::array<uint8_t, 3> received{};
  std::string error;
  NtripStream stream("127.0.0.1", port, "MOUNT", "", "", 5);
  try {
    connected = stream.Connect();
    if (connected) {
      bytes_read = stream.read(received.data(), received.size());
    }
  } catch (const std::exception& exception) {
    error = exception.what();
  }
  stream.Disconnect();
  shutdown(listener_fd, SHUT_RDWR);
  close(listener_fd);
  server.join();

  EXPECT_TRUE(connected) << error;
  EXPECT_TRUE(server_sent_response.load());
  EXPECT_EQ(bytes_read, correction_bytes.size());
  EXPECT_EQ(std::string(received.begin(), received.end()), correction_bytes);
}

TEST(NtripStreamRegressionTest, ReconnectSerializesConcurrentGgaWrites) {
  uint16_t port = 0;
  const int listener_fd = CreateListener(&port);
  ASSERT_GE(listener_fd, 0);

  std::atomic<int> connection_count{0};
  std::thread server([&]() {
    for (int attempt = 0; attempt < 2; ++attempt) {
      const int client_fd = accept(listener_fd, nullptr, nullptr);
      if (client_fd < 0) {
        return;
      }
      ++connection_count;
      std::array<char, 512> request{};
      if (recv(client_fd, request.data(), request.size(), 0) <= 0 ||
          !SendAll(client_fd, "ICY 200 OK\r\n\r\n")) {
        close(client_fd);
        return;
      }
      while (recv(client_fd, request.data(), request.size(), 0) > 0) {
      }
      close(client_fd);
    }
  });

  bool connected = false;
  bool reconnected = false;
  size_t reconnect_read = 1;
  std::string error;
  NtripStream stream("127.0.0.1", port, "MOUNT", "", "", 1);
  try {
    connected = stream.Connect();
    if (connected) {
      std::atomic<bool> stop_writer{false};
      std::atomic<bool> write_failed{false};
      const std::string gga = "$GPGGA,regression\r\n";
      std::thread writer([&]() {
        while (!stop_writer.load()) {
          try {
            stream.write(reinterpret_cast<const uint8_t*>(gga.data()),
                         gga.size());
          } catch (...) {
            write_failed = true;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });

      std::this_thread::sleep_for(std::chrono::milliseconds(1200));
      std::array<uint8_t, 16> buffer{};
      try {
        reconnect_read = stream.read(buffer.data(), buffer.size());
      } catch (const std::exception& exception) {
        error = exception.what();
      }
      stop_writer = true;
      writer.join();
      EXPECT_FALSE(write_failed.load());
      reconnected = stream.get_status() == Stream::Status::CONNECTED;
    }
  } catch (const std::exception& exception) {
    error = exception.what();
  }
  stream.Disconnect();
  shutdown(listener_fd, SHUT_RDWR);
  close(listener_fd);
  server.join();

  EXPECT_TRUE(connected) << error;
  EXPECT_TRUE(reconnected) << error;
  EXPECT_EQ(reconnect_read, 0);
  EXPECT_EQ(connection_count.load(), 2);
  EXPECT_EQ(stream.get_status(), Stream::Status::DISCONNECTED);
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
