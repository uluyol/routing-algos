#include "routing-algos/conversion.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace routing_algos {
namespace {

TEST(BpsToFracTest, SinglePathGetsAllDemand) {
  const PathSplit with_capacity_result = BpsToFrac({{Path{0, 1}, 123}});
  EXPECT_THAT(with_capacity_result, testing::Eq(PathSplit{{Path{0, 1}, 1.0}}));

  const PathSplit no_capacity_result = BpsToFrac({{Path{4, 2, 6}, 0}});
  EXPECT_THAT(no_capacity_result, testing::Eq(PathSplit{{Path{4, 2, 6}, 1.0}}));
}

TEST(BpsToFracTest, NoCapacityIsEvenlyDistributed) {
  const PathSplit result = BpsToFrac({
      {Path{0}, 0},
      {Path{1, 2}, 0},
  });

  EXPECT_THAT(result, testing::Eq(PathSplit{
                          {Path{0}, 0.5},
                          {Path{1, 2}, 0.5},
                      }));
}

TEST(BpsToFracTest, WithCapacityIsWeighted) {
  const PathSplit result = BpsToFrac({
      {Path{2, 3, 4, 5}, 9},
      {Path{1}, 1},
  });

  EXPECT_THAT(result, testing::Eq(PathSplit{
                          {Path{2, 3, 4, 5}, 0.9},
                          {Path{1}, 0.1},
                      }));
}

}  // namespace
}  // namespace routing_algos
