#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "modules/drivers/hal/stream/tcp_stream.h"

namespace apollo {
namespace drivers {
namespace hal {
namespace {

volatile sig_atomic_t sigpipe_count = 0;

void CountSigpipe(int) { ++sigpipe_count; }

void IgnoreSigalrm(int) {}

int CreateListener(uint16_t* port) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }
  int reuse = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

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

TEST(TcpStreamRegressionTest, ZeroTimeoutReadAndWriteDoNotBlock) {
  uint16_t port = 0;
  const int listener_fd = CreateListener(&port);
  ASSERT_GE(listener_fd, 0);

  std::thread server([listener_fd]() {
    const int client_fd = accept(listener_fd, nullptr, nullptr);
    if (client_fd >= 0) {
      std::this_thread::sleep_for(std::chrono::seconds(3));
      close(client_fd);
    }
  });

  TcpStream stream("127.0.0.1", port, 0, false);
  const bool connected = stream.Connect();
  std::array<uint8_t, 1> byte{};
  const auto read_start = std::chrono::steady_clock::now();
  const size_t bytes_read = stream.read(byte.data(), byte.size());
  const auto read_elapsed = std::chrono::steady_clock::now() - read_start;

  struct sigaction alarm_action {};
  struct sigaction previous_alarm_action {};
  alarm_action.sa_handler = IgnoreSigalrm;
  sigemptyset(&alarm_action.sa_mask);
  sigaction(SIGALRM, &alarm_action, &previous_alarm_action);

  std::mutex watchdog_mutex;
  std::condition_variable watchdog_cv;
  bool write_finished = false;
  const pthread_t test_thread = pthread_self();
  std::thread watchdog([&]() {
    std::unique_lock<std::mutex> lock(watchdog_mutex);
    if (!watchdog_cv.wait_for(lock, std::chrono::seconds(2),
                              [&]() { return write_finished; })) {
      pthread_kill(test_thread, SIGALRM);
    }
  });

  std::vector<uint8_t> payload(32 * 1024 * 1024, 0x5A);
  const auto write_start = std::chrono::steady_clock::now();
  const size_t bytes_written = stream.write(payload.data(), payload.size());
  const auto write_elapsed = std::chrono::steady_clock::now() - write_start;
  {
    std::lock_guard<std::mutex> lock(watchdog_mutex);
    write_finished = true;
  }
  watchdog_cv.notify_one();
  watchdog.join();
  sigaction(SIGALRM, &previous_alarm_action, nullptr);

  stream.Disconnect();
  shutdown(listener_fd, SHUT_RDWR);
  close(listener_fd);
  server.join();

  EXPECT_TRUE(connected);
  EXPECT_EQ(bytes_read, 0);
  EXPECT_LT(read_elapsed, std::chrono::milliseconds(250));
  EXPECT_LT(bytes_written, payload.size());
  EXPECT_LT(write_elapsed, std::chrono::seconds(1));
}

TEST(TcpStreamRegressionTest, BrokenPeerDoesNotRaiseSigpipe) {
  uint16_t port = 0;
  const int listener_fd = CreateListener(&port);
  ASSERT_GE(listener_fd, 0);

  std::promise<void> peer_closed_promise;
  auto peer_closed = peer_closed_promise.get_future();
  std::thread server([listener_fd, &peer_closed_promise]() {
    const int client_fd = accept(listener_fd, nullptr, nullptr);
    if (client_fd >= 0) {
      close(client_fd);
    }
    peer_closed_promise.set_value();
  });

  struct sigaction pipe_action {};
  struct sigaction previous_pipe_action {};
  pipe_action.sa_handler = CountSigpipe;
  sigemptyset(&pipe_action.sa_mask);
  sigpipe_count = 0;
  sigaction(SIGPIPE, &pipe_action, &previous_pipe_action);

  TcpStream stream("127.0.0.1", port, 100000, false);
  const bool connected = stream.Connect();
  const bool server_closed = peer_closed.wait_for(std::chrono::seconds(2)) ==
                             std::future_status::ready;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const uint8_t payload = 0x5A;
  bool write_failed = false;
  for (int attempt = 0; connected && server_closed && attempt < 100;
       ++attempt) {
    try {
      if (stream.write(&payload, sizeof(payload)) == 0) {
        break;
      }
    } catch (const std::runtime_error&) {
      write_failed = true;
      break;
    }
  }

  stream.Disconnect();
  sigaction(SIGPIPE, &previous_pipe_action, nullptr);
  shutdown(listener_fd, SHUT_RDWR);
  close(listener_fd);
  server.join();

  EXPECT_TRUE(connected);
  EXPECT_TRUE(server_closed);
  EXPECT_TRUE(write_failed);
  EXPECT_EQ(sigpipe_count, 0);
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
