#ifndef MOTION_PRIMITIVE_PLANNER_CANDIDATE_EXECUTOR_H
#define MOTION_PRIMITIVE_PLANNER_CANDIDATE_EXECUTOR_H

#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace motion_primitive_planner
{
//! A synchronous indexed batch on persistent workers. Each index executes once;
//! callers own the result slots and may read them after run() returns.
class CandidateExecutor
{
public:
  explicit CandidateExecutor(size_t threads)
  {
    if (threads <= 1) return;
    try
    {
      for (size_t index = 0; index < threads; ++index)
        workers_.emplace_back([this]() { work(); });
    }
    catch (...)
    {
      stop();
      throw;
    }
  }

  ~CandidateExecutor() { stop(); }
  CandidateExecutor(const CandidateExecutor&) = delete;
  CandidateExecutor& operator=(const CandidateExecutor&) = delete;

  void run(size_t count, const std::function<void(size_t)>& task)
  {
    // Also serialize independent callers of this executor.
    std::lock_guard<std::mutex> batch_lock(batch_mutex_);
    if (workers_.empty() || count <= 1)
    {
      for (size_t index = 0; index < count; ++index) task(index);
      return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    failures_.assign(count, nullptr);
    task_ = task;
    next_ = 0;
    count_ = count;
    remaining_ = count;
    ready_.notify_all();
    done_.wait(lock, [this]() { return remaining_ == 0; });
    task_ = {};
    // Preserve candidate order when more than one task throws.
    for (const auto& failure : failures_)
      if (failure) std::rethrow_exception(failure);
  }

private:
  void work()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;)
    {
      ready_.wait(lock, [this]() { return stopping_ || next_ < count_; });
      if (stopping_) return;
      const size_t index = next_++;
      lock.unlock();
      try { task_(index); }
      catch (...) { failures_[index] = std::current_exception(); }
      lock.lock();
      if (--remaining_ == 0) done_.notify_one();
    }
  }

  void stop()
  {
    std::lock_guard<std::mutex> batch_lock(batch_mutex_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    ready_.notify_all();
    for (auto& worker : workers_) worker.join();
  }

  std::mutex batch_mutex_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::condition_variable done_;
  std::vector<std::thread> workers_;
  std::function<void(size_t)> task_;
  std::vector<std::exception_ptr> failures_;
  size_t next_ = 0;
  size_t count_ = 0;
  size_t remaining_ = 0;
  bool stopping_ = false;
};
}  // namespace motion_primitive_planner

#endif
