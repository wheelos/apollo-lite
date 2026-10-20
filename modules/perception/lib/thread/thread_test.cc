/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/lib/thread/thread.h"

#include <atomic>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace lib {

class MyThread : public Thread {
 public:
  explicit MyThread(bool wait_for_release = false)
      : Thread(true),
        value_(0),
        wait_for_release_(wait_for_release),
        release_(false) {}
  virtual ~MyThread() {}

  int get_value() const { return value_; }
  void Release() { release_.store(true, std::memory_order_release); }

  MyThread(const MyThread &) = delete;
  MyThread &operator=(const MyThread &) = delete;

 protected:
  void Run() override {
    int count = 0;
    while (count < 10) {
      value_ += 10;
      ++count;
    }
    while (wait_for_release_ && !release_.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  }

 private:
  int value_;
  bool wait_for_release_;
  std::atomic<bool> release_;
};

TEST(TestThread, Test) {
  MyThread my_thread;
  EXPECT_EQ(my_thread.get_value(), 0);
  my_thread.set_thread_name("my_thread");
  EXPECT_EQ(my_thread.get_thread_name(), "my_thread");

  my_thread.Start();
  my_thread.Join();
  EXPECT_EQ(my_thread.get_value(), 100);
  EXPECT_FALSE(my_thread.IsAlive());
  MyThread my_thread2(true);
  my_thread2.Start();
  EXPECT_TRUE(my_thread2.IsAlive());
  my_thread2.Release();
  my_thread2.Join();
  EXPECT_EQ(my_thread2.tid(), 0);

  MyThread my_thread3(true);
  my_thread3.set_joinable(false);
  my_thread3.Start();
  my_thread3.set_joinable(false);
  EXPECT_TRUE(my_thread3.IsAlive());
  my_thread3.Release();
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (my_thread3.IsAlive() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  EXPECT_FALSE(my_thread3.IsAlive());
}

class ExitingThread : public Thread {
 public:
  explicit ExitingThread(bool wait_for_cancel)
      : Thread(true), wait_for_cancel_(wait_for_cancel) {}

 protected:
  void Run() override {
    if (wait_for_cancel_) {
      while (true) {
        pthread_testcancel();
        std::this_thread::yield();
      }
    }
    pthread_exit(nullptr);
  }

 private:
  bool wait_for_cancel_;
};

TEST(TestThread, PthreadExitClearsAlive) {
  ExitingThread thread(false);
  thread.Start();
  thread.Join();
  EXPECT_FALSE(thread.IsAlive());
}

TEST(TestThread, CancellationClearsAlive) {
  ExitingThread thread(true);
  thread.Start();
  EXPECT_EQ(pthread_cancel(thread.tid()), 0);
  thread.Join();
  EXPECT_FALSE(thread.IsAlive());
}

TEST(TestThread, StartPreservesCallerCancellationSettings) {
  int original_state = 0;
  int original_type = 0;
  ASSERT_EQ(pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &original_state), 0);
  ASSERT_EQ(pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, &original_type), 0);

  MyThread thread;
  thread.Start();
  thread.Join();

  int state_after_start = 0;
  int type_after_start = 0;
  EXPECT_EQ(pthread_setcancelstate(original_state, &state_after_start), 0);
  EXPECT_EQ(pthread_setcanceltype(original_type, &type_after_start), 0);
  EXPECT_EQ(state_after_start, PTHREAD_CANCEL_DISABLE);
  EXPECT_EQ(type_after_start, PTHREAD_CANCEL_DEFERRED);
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
