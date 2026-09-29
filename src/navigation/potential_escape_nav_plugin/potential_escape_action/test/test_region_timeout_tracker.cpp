#include <chrono>

#include <gtest/gtest.h>

#include "potential_escape_action/potential_escape_action.hpp"

using namespace std::chrono_literals;

namespace nav2_behavior_tree
{
namespace
{

TEST(RegionTimeoutTrackerTest, TriggersAfterContinuousTimeout)
{
  RegionTimeoutTracker tracker;
  tracker.setTimeout(3.0);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(tracker.update(true, start));
  EXPECT_FALSE(tracker.update(true, start + 2s));
  EXPECT_TRUE(tracker.update(true, start + 3s));
}

TEST(RegionTimeoutTrackerTest, LeavingRegionResetsTimer)
{
  RegionTimeoutTracker tracker;
  tracker.setTimeout(3.0);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(tracker.update(true, start));
  EXPECT_FALSE(tracker.update(false, start + 2s));
  EXPECT_FALSE(tracker.update(true, start + 3s));
  EXPECT_TRUE(tracker.update(true, start + 6s));
}

TEST(RegionTimeoutTrackerTest, ResetClearsLatchedTrigger)
{
  RegionTimeoutTracker tracker;
  tracker.setTimeout(1.0);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(tracker.update(true, start));
  EXPECT_TRUE(tracker.update(true, start + 1s));

  tracker.reset();

  EXPECT_FALSE(tracker.update(true, start + 2s));
  EXPECT_TRUE(tracker.update(true, start + 3s));
}

}  // namespace
}  // namespace nav2_behavior_tree
