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

#ifndef CYBER_BASE_THREAD_SAFE_QUEUE_H_
#define CYBER_BASE_THREAD_SAFE_QUEUE_H_

#include <queue>
#include <utility>

#include "absl/synchronization/mutex.h"

namespace apollo {
namespace cyber {
namespace base {

template <typename T>
class ThreadSafeQueue {
 public:
  ThreadSafeQueue() {}
  ThreadSafeQueue& operator=(const ThreadSafeQueue& other) = delete;
  ThreadSafeQueue(const ThreadSafeQueue& other) = delete;

  ~ThreadSafeQueue() { BreakAllWait(); }

  void Enqueue(const T& element) {
    absl::MutexLock lock(&mutex_);
    queue_.emplace(element);
  }

  bool Dequeue(T* element) {
    absl::MutexLock lock(&mutex_);
    if (queue_.empty()) {
      return false;
    }
    *element = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  bool WaitDequeue(T* element) {
    absl::MutexLock lock(&mutex_);
    mutex_.Await(absl::Condition(this, &ThreadSafeQueue::IsReadyForDequeue));
    if (break_all_wait_) {
      return false;
    }
    *element = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  typename std::queue<T>::size_type Size() {
    absl::MutexLock lock(&mutex_);
    return queue_.size();
  }

  bool Empty() {
    absl::MutexLock lock(&mutex_);
    return queue_.empty();
  }

  void BreakAllWait() {
    absl::MutexLock lock(&mutex_);
    break_all_wait_ = true;
  }

 private:
  // Returns true when the queue is ready for dequeue: either the queue has
  // elements or BreakAllWait() has been called.
  bool IsReadyForDequeue() const ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return break_all_wait_ || !queue_.empty();
  }

  bool break_all_wait_ ABSL_GUARDED_BY(mutex_) = false;
  absl::Mutex mutex_;
  std::queue<T> queue_;
};

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_THREAD_SAFE_QUEUE_H_
