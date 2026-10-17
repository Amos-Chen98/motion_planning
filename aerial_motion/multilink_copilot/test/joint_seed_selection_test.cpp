#include <multilink_copilot/joint_seed_selection.h>

#include <gtest/gtest.h>

#include <random>

namespace multilink_copilot
{
namespace
{
Eigen::VectorXd joints(double x, double y = 0.0)
{
  return Eigen::Vector2d(x, y);
}

JointSeedEvaluation evaluation(bool safe, double score = 1.0)
{
  JointSeedEvaluation result;
  result.valid = true;
  result.metrics.safe = safe;
  result.violation_score = safe ? 0.0 : score;
  return result;
}

class JointSeedSelectionTest : public ::testing::Test
{
protected:
  JointSeedSelection select(bool has_published = true)
  {
    return selectJointSeed(desired, measured, has_published ? &published : nullptr, history,
                           evaluation(false, 5.0), [this](const Eigen::VectorXd& q) {
                             calls.push_back(q(0));
                             return evaluation(is_safe(q(0)));
                           });
  }

  Eigen::VectorXd desired = joints(100.0);
  Eigen::VectorXd measured = joints(0.0);
  Eigen::VectorXd published = joints(10.0);
  std::deque<Eigen::VectorXd> history;
  std::vector<double> calls;
  std::function<bool(double)> is_safe = [](double) { return false; };
};

TEST_F(JointSeedSelectionTest, PublishedReferenceStopsBeforeMeasuredAndHistory)
{
  history = {joints(1.0)};
  is_safe = [](double) { return true; };
  const auto result = select();
  ASSERT_TRUE(result.reference_index);
  EXPECT_EQ(result.candidates[*result.reference_index].source, JointSeedSource::kLatestPublished);
  EXPECT_EQ(calls, (std::vector<double>{10.0}));
}

TEST_F(JointSeedSelectionTest, MeasuredReferenceStopsBeforeHistory)
{
  history = {joints(1.0)};
  is_safe = [](double x) { return x != 10.0; };
  const auto result = select();
  ASSERT_TRUE(result.reference_index);
  EXPECT_EQ(result.candidates[*result.reference_index].source, JointSeedSource::kCurrentMeasured);
  EXPECT_EQ(calls, (std::vector<double>{10.0, 0.0}));
}

TEST_F(JointSeedSelectionTest, MeasuredReferenceWorksWithoutPublishedCommand)
{
  is_safe = [](double) { return true; };
  ASSERT_TRUE(select(false).reference_index);
  EXPECT_EQ(calls, (std::vector<double>{0.0}));
}

TEST_F(JointSeedSelectionTest, ChoosesNearestFeasibleHistoryAndStops)
{
  history = {joints(3.0), joints(1.0), joints(2.0)};
  is_safe = [](double x) { return x == 2.0 || x == 3.0; };
  const auto result = select();
  ASSERT_TRUE(result.reference_index);
  EXPECT_DOUBLE_EQ(result.candidates[*result.reference_index].joint_positions(0), 2.0);
  EXPECT_EQ(calls, (std::vector<double>{10.0, 0.0, 1.0, 2.0}));
}

TEST_F(JointSeedSelectionTest, EqualHistoryDistancesPreferNewerRecord)
{
  history = {joints(-2.0), joints(2.0)};
  is_safe = [](double x) { return std::abs(x) == 2.0; };
  const auto result = select();
  ASSERT_TRUE(result.reference_index);
  EXPECT_DOUBLE_EQ(result.candidates[*result.reference_index].joint_positions(0), 2.0);
}

TEST_F(JointSeedSelectionTest, NewestHistoryHasNoSeparatePriority)
{
  history = {joints(1.0), joints(2.0)};
  is_safe = [](double x) { return x == 1.0 || x == 2.0; };
  const auto result = select();
  ASSERT_TRUE(result.reference_index);
  EXPECT_DOUBLE_EQ(result.candidates[*result.reference_index].joint_positions(0), 1.0);
}

TEST_F(JointSeedSelectionTest, SearchesPastSixHistoryCandidates)
{
  for (int x = 8; x >= 1; --x)
    history.push_back(joints(x));
  is_safe = [](double x) { return x == 8.0; };
  const auto result = select();
  ASSERT_TRUE(result.reference_index);
  EXPECT_DOUBLE_EQ(result.candidates[*result.reference_index].joint_positions(0), 8.0);
  EXPECT_EQ(calls.size(), 10u);
}

TEST_F(JointSeedSelectionTest, ExactDuplicatesReuseNominalEvaluationAndHighestSource)
{
  published = desired;
  measured = desired;
  history = {desired, desired};
  const auto result = select();
  EXPECT_FALSE(result.reference_index);
  EXPECT_TRUE(calls.empty());
  ASSERT_EQ(result.candidates.size(), 1u);
  EXPECT_EQ(result.candidates.front().source, JointSeedSource::kLatestPublished);
  EXPECT_DOUBLE_EQ(result.candidates.front().evaluation.violation_score, 5.0);
}

TEST_F(JointSeedSelectionTest, DuplicateHistoryIsEvaluatedOnceAndNominalRetainsRepairPriority)
{
  history = {desired, joints(2.0), measured, published, joints(2.0)};
  const auto result = select();
  EXPECT_FALSE(result.reference_index);
  EXPECT_EQ(calls, (std::vector<double>{10.0, 0.0, 2.0}));
  ASSERT_EQ(result.candidates.size(), 4u);
  EXPECT_EQ(result.candidates.back().source, JointSeedSource::kDesiredSeed);
}

TEST_F(JointSeedSelectionTest, NearbyMeasuredConfigurationIsNotDiscarded)
{
  published = joints(0.0);
  measured = joints(0.0005);
  is_safe = [](double x) { return x > 0.0; };
  const auto result = select();
  ASSERT_TRUE(result.reference_index);
  EXPECT_EQ(result.candidates[*result.reference_index].source, JointSeedSource::kCurrentMeasured);
  EXPECT_EQ(calls.size(), 2u);
}

TEST_F(JointSeedSelectionTest, NearbyHistoryConfigurationsCanHaveDifferentFeasibility)
{
  history = {joints(1.0), joints(1.0005)};
  is_safe = [](double x) { return x == 1.0005; };
  const auto result = select();
  ASSERT_TRUE(result.reference_index);
  EXPECT_DOUBLE_EQ(result.candidates[*result.reference_index].joint_positions(0), 1.0005);
  EXPECT_EQ(calls.size(), 4u);
}

TEST_F(JointSeedSelectionTest, EmptyHistoryStillReturnsMeasuredAndNominalRepairSeeds)
{
  const auto result = select(false);
  EXPECT_FALSE(result.reference_index);
  EXPECT_EQ(result.candidates.size(), 2u);
  EXPECT_EQ(calls, (std::vector<double>{0.0}));
}

TEST_F(JointSeedSelectionTest, RechecksFeasibilityOnEveryCall)
{
  is_safe = [](double) { return true; };
  ASSERT_TRUE(select().reference_index);
  is_safe = [](double) { return false; };
  EXPECT_FALSE(select().reference_index);
  EXPECT_EQ(calls, (std::vector<double>{10.0, 10.0, 0.0}));
}

TEST_F(JointSeedSelectionTest, InvalidConfigurationsDoNotReachEvaluator)
{
  published(0) = std::numeric_limits<double>::quiet_NaN();
  history = {Eigen::VectorXd::Zero(3), published, joints(1.0)};
  is_safe = [](double x) { return x == 1.0; };
  const auto result = select();
  ASSERT_TRUE(result.reference_index);
  EXPECT_EQ(calls, (std::vector<double>{0.0, 1.0}));
}

TEST(JointSeedSelection, FailedOrNonfiniteEvaluationsCannotBecomeReferencesOrRepairSeeds)
{
  const auto measured = joints(0);
  const auto published = joints(1);
  const auto result = selectJointSeed(joints(100), measured, &published, {joints(2)},
      evaluation(false, 5), [](const Eigen::VectorXd& q) {
        auto value = evaluation(true);
        if (q(0) == 1)
          value.valid = false;
        else
          value.metrics.static_thrust_max = std::numeric_limits<double>::quiet_NaN();
        return value;
      });
  EXPECT_FALSE(result.reference_index);
  std::vector<double> calls;
  Eigen::VectorXd output;
  EXPECT_FALSE(repairJointSeeds(joints(100), result.candidates, 2, [&](const auto& seed, auto&) {
    calls.push_back(seed(0));
    return false;
  }, output));
  // Only the cached, valid nominal evaluation remains eligible for repair.
  EXPECT_EQ(calls, (std::vector<double>{100}));
}

TEST(JointSeedSelection, MatchesExhaustiveOracleAcrossFeasibilityPatternsAndDistanceTies)
{
  std::mt19937 generator(42);
  std::uniform_int_distribution<int> coordinate(-4, 4);
  for (int trial = 0; trial < 500; ++trial)
  {
    const Eigen::VectorXd desired = joints(100.0, 100.0);
    const Eigen::VectorXd measured = joints(coordinate(generator), coordinate(generator));
    const Eigen::VectorXd published = joints(coordinate(generator), coordinate(generator));
    std::deque<Eigen::VectorXd> history;
    for (int index = 0; index < 32; ++index)
      history.push_back(joints(coordinate(generator), coordinate(generator)));
    const auto safe = [trial](const Eigen::VectorXd& q) {
      return (static_cast<int>(3 * q(0) + q(1)) + trial) % 7 == 0;
    };
    const bool has_published = trial % 2 == 0;
    const auto result = selectJointSeed(desired, measured, has_published ? &published : nullptr,
                                        history, evaluation(false), [&](const Eigen::VectorXd& q) {
                                          return evaluation(safe(q));
                                        });

    std::optional<Eigen::VectorXd> expected;
    JointSeedSource source = JointSeedSource::kStableHistory;
    if (has_published && safe(published))
    {
      expected = published;
      source = JointSeedSource::kLatestPublished;
    }
    else if (safe(measured))
    {
      expected = measured;
      source = JointSeedSource::kCurrentMeasured;
    }
    else
    {
      double minimum = std::numeric_limits<double>::infinity();
      // Exhaustively check every history item; equality replaces older items.
      for (const auto& q : history)
      {
        if (safe(q) && (q - measured).squaredNorm() <= minimum)
        {
          expected = q;
          minimum = (q - measured).squaredNorm();
        }
      }
    }
    ASSERT_EQ(result.reference_index.has_value(), expected.has_value()) << trial;
    if (expected)
    {
      EXPECT_TRUE(sameJointSeed(result.candidates[*result.reference_index].joint_positions, *expected)) << trial;
      EXPECT_EQ(result.candidates[*result.reference_index].source, source) << trial;
    }
  }
}

TEST(JointSeedHistory, KeepsLatest32RecordsAndMergesOnlyConsecutiveNearbyRecords)
{
  std::deque<Eigen::VectorXd> history;
  for (int index = 0; index < 40; ++index)
    rememberJointSeed(history, joints(index), 32, 0.001);
  ASSERT_EQ(history.size(), 32u);
  EXPECT_DOUBLE_EQ(history.front()(0), 8.0);
  rememberJointSeed(history, joints(39.0005), 32, 0.001);
  EXPECT_EQ(history.size(), 32u);
  EXPECT_DOUBLE_EQ(history.back()(0), 39.0005);
  rememberJointSeed(history, joints(8.0), 32, 0.001);
  EXPECT_DOUBLE_EQ(history.front()(0), 9.0);
  EXPECT_DOUBLE_EQ(history.back()(0), 8.0);
}

TEST(JointSeedHistory, ZeroCapacityClearsHistoryAndRetainsNoHiddenLatestRecord)
{
  std::deque<Eigen::VectorXd> history{joints(1.0)};
  rememberJointSeed(history, joints(2.0), 0, 0.001);
  EXPECT_TRUE(history.empty());
  rememberJointSeed(history, joints(3.0), 0, 0.001);
  EXPECT_TRUE(history.empty());
}

JointSeedCandidate repairCandidate(double x, JointSeedSource source, double score)
{
  JointSeedCandidate candidate;
  candidate.joint_positions = joints(x);
  candidate.source = source;
  candidate.evaluation = evaluation(false, score);
  candidate.desired_distance = std::abs(x);
  candidate.measured_distance = std::abs(x - 1.0);
  return candidate;
}

TEST(JointSeedRepair, AttemptsOnlyLowestViolationSeedsWithinBudgetAndKeepsClosestOutput)
{
  const std::vector<JointSeedCandidate> candidates{
      repairCandidate(3, JointSeedSource::kLatestPublished, 3.0),
      repairCandidate(1, JointSeedSource::kStableHistory, 0.1),
      repairCandidate(2, JointSeedSource::kStableHistory, 0.2)};
  std::vector<double> calls;
  Eigen::VectorXd output;
  ASSERT_TRUE(repairJointSeeds(joints(0), candidates, 2, [&](const auto& seed, auto& projected) {
    calls.push_back(seed(0));
    projected = joints(3.0 - seed(0));
    return true;
  }, output));
  EXPECT_EQ(calls, (std::vector<double>{1, 2}));
  EXPECT_DOUBLE_EQ(output(0), 1.0);
}

TEST(JointSeedRepair, TiedScoresUseSourceThenDesiredThenMeasuredDistance)
{
  const std::vector<JointSeedCandidate> candidates{
      repairCandidate(3, JointSeedSource::kStableHistory, 1),
      repairCandidate(-2, JointSeedSource::kStableHistory, 1),
      repairCandidate(2, JointSeedSource::kStableHistory, 1),
      repairCandidate(4, JointSeedSource::kDesiredSeed, 1),
      repairCandidate(5, JointSeedSource::kCurrentMeasured, 1),
      repairCandidate(6, JointSeedSource::kLatestPublished, 1)};
  std::vector<double> calls;
  Eigen::VectorXd output;
  repairJointSeeds(joints(0), candidates, 6, [&](const auto& seed, auto&) {
    calls.push_back(seed(0));
    return false;
  }, output);
  EXPECT_EQ(calls, (std::vector<double>{6, 5, 4, 2, -2, 3}));
}

TEST(JointSeedRepair, InvalidEvaluationsAndNonfiniteScoresDoNotConsumeAttempts)
{
  auto invalid = repairCandidate(1, JointSeedSource::kLatestPublished, -1);
  invalid.evaluation.valid = false;
  auto nan = repairCandidate(2, JointSeedSource::kCurrentMeasured,
                              std::numeric_limits<double>::quiet_NaN());
  auto infinity = repairCandidate(3, JointSeedSource::kStableHistory,
                                   std::numeric_limits<double>::infinity());
  auto safe = repairCandidate(4, JointSeedSource::kStableHistory, 0);
  safe.evaluation.metrics.safe = true;
  std::vector<double> calls;
  Eigen::VectorXd output;
  EXPECT_FALSE(repairJointSeeds(joints(0), {invalid, nan, infinity, safe,
      repairCandidate(5, JointSeedSource::kDesiredSeed, 1)}, 2, [&](const auto& seed, auto&) {
    calls.push_back(seed(0));
    return false;
  }, output));
  EXPECT_EQ(calls, (std::vector<double>{5}));
}

TEST(JointSeedRepair, FailedAttemptsRespectBudgetAndDisabledRepairMakesNoCalls)
{
  const std::vector<JointSeedCandidate> candidates{
      repairCandidate(1, JointSeedSource::kLatestPublished, 1),
      repairCandidate(2, JointSeedSource::kCurrentMeasured, 2),
      repairCandidate(3, JointSeedSource::kStableHistory, 3)};
  std::vector<double> calls;
  Eigen::VectorXd output;
  const auto project = [&](const Eigen::VectorXd& seed, Eigen::VectorXd& projected) {
    calls.push_back(seed(0));
    projected = seed;
    return seed(0) == 3;
  };
  EXPECT_FALSE(repairJointSeeds(joints(0), candidates, 2, project, output));
  EXPECT_EQ(calls, (std::vector<double>{1, 2}));
  calls.clear();
  EXPECT_FALSE(repairJointSeeds(joints(0), candidates, 0, project, output));
  EXPECT_TRUE(calls.empty());
}
}  // namespace
}  // namespace multilink_copilot

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
