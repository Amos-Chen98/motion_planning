#include <motion_primitive_planner/detail/candidate_executor.h>
#include <gtest/gtest.h>

#include <atomic>
#include <stdexcept>

using motion_primitive_planner::detail::CandidateExecutor;

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(CandidateExecutor, SerialAndEmptyBatchesStayOnCaller)
{
  CandidateExecutor executor(1);
  const auto caller = std::this_thread::get_id();
  executor.run(0, [](size_t) { FAIL() << "empty batch executed"; });
  executor.run(3, [&](size_t) { EXPECT_EQ(std::this_thread::get_id(), caller); });
}

TEST(CandidateExecutor, WorkersOverlapAndPublishEveryResultBeforeReturning)
{
  CandidateExecutor executor(3);
  std::mutex mutex;
  std::condition_variable ready;
  int arrived = 0;
  std::vector<int> results(3, 0);
  executor.run(3, [&](size_t index) {
    std::unique_lock<std::mutex> lock(mutex);
    ++arrived;
    ready.notify_all();
    ASSERT_TRUE(ready.wait_until(lock, std::chrono::system_clock::now() + std::chrono::seconds(5),
                                 [&]() { return arrived == 3; }));
    lock.unlock();
    results[index] = static_cast<int>(index + 1);
  });
  EXPECT_EQ(results, (std::vector<int>{1, 2, 3}));
  std::vector<int> calls(17, 0);
  for (int repeat = 0; repeat < 100; ++repeat)
    executor.run(calls.size(), [&](size_t index) { ++calls[index]; });
  EXPECT_EQ(calls, std::vector<int>(17, 100));
  executor.run(0, [](size_t) { FAIL(); });
  executor.run(1, [&](size_t index) { EXPECT_EQ(index, 0u); });
}

TEST(CandidateExecutor, DrainsFailuresAndCanRunAnotherBatch)
{
  CandidateExecutor executor(4);
  std::atomic<int> finished{0};
  try
  {
    executor.run(9, [&](size_t index) {
      ++finished;
      if (index == 2 || index == 5) throw std::runtime_error(std::to_string(index));
    });
    FAIL() << "worker exception was lost";
  }
  catch (const std::runtime_error& error) { EXPECT_EQ(std::string(error.what()), "2"); }
  EXPECT_EQ(finished.load(), 9);
  executor.run(9, [&](size_t) { ++finished; });
  EXPECT_EQ(finished.load(), 18);
}

TEST(CandidateExecutor, ConcurrentCallersDoNotReplaceAnActiveBatch)
{
  CandidateExecutor executor(3);
  std::vector<int> first(19, 0), second(7, 0);
  std::thread caller([&]() {
    for (int repeat = 0; repeat < 50; ++repeat)
      executor.run(first.size(), [&](size_t i) { ++first[i]; });
  });
  for (int repeat = 0; repeat < 50; ++repeat)
    executor.run(second.size(), [&](size_t i) { ++second[i]; });
  caller.join();
  EXPECT_EQ(first, std::vector<int>(19, 50));
  EXPECT_EQ(second, std::vector<int>(7, 50));
}
