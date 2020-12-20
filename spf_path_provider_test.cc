#include "spf_path_provider.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace routing_algos {
namespace {

TEST(SPFPathProviderTest, Triangle) {
  std::vector<Node> nodes{
      {.name = "A"},
      {.name = "B"},
      {.name = "C"},
  };
  std::vector<Link> links{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 1000,
          .delay_ms = 1,
      },
      {
          .src = 1,
          .dst = 2,
          .capacity_bps = 1000,
          .delay_ms = 1,
      },
      {
          .src = 0,
          .dst = 2,
          .capacity_bps = 1000,
          .delay_ms = 1,
      },
  };

  SPFPathProvider path_provider(nodes, links);
  EXPECT_THAT(path_provider.NextBestPath(FG{0, 1}, {}),
              testing::ElementsAre(0));
  EXPECT_THAT(path_provider.NextBestPath(FG{0, 2}, {}),
              testing::ElementsAre(2));
  EXPECT_THAT(path_provider.NextBestPath(FG{1, 2}, {}),
              testing::ElementsAre(1));

  EXPECT_THAT(path_provider.NextBestPath(FG{0, 2}, {2}),
              testing::ElementsAre(0, 1));

  EXPECT_THAT(path_provider.NextBestPath(FG{2, 0}, {}), testing::IsEmpty());
}

TEST(SPFPathProviderTest, TriangleWeighted) {
  std::vector<Node> nodes{
      {.name = "A"},
      {.name = "B"},
      {.name = "C"},
  };
  std::vector<Link> links{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 1000,
          .delay_ms = 1,
      },
      {
          .src = 1,
          .dst = 2,
          .capacity_bps = 1000,
          .delay_ms = 1,
      },
      {
          .src = 0,
          .dst = 2,
          .capacity_bps = 1000,
          .delay_ms = 10,
      },
  };

  SPFPathProvider path_provider(nodes, links);
  EXPECT_THAT(path_provider.NextBestPath(FG{0, 1}, {}),
              testing::ElementsAre(0));
  EXPECT_THAT(path_provider.NextBestPath(FG{0, 2}, {}),
              testing::ElementsAre(0, 1));
  EXPECT_THAT(path_provider.NextBestPath(FG{1, 2}, {}),
              testing::ElementsAre(1));

  EXPECT_THAT(path_provider.NextBestPath(FG{0, 2}, {0}),
              testing::ElementsAre(2));

  EXPECT_THAT(path_provider.NextBestPath(FG{2, 0}, {}), testing::IsEmpty());
}

TEST(SPFPathProviderTest, Diamond) {
  std::vector<Node> nodes{
      {.name = "A"},
      {.name = "B"},
      {.name = "C"},
      {.name = "D"},
  };
  std::vector<Link> links{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 1000,
          .delay_ms = 1,
      },
      {
          .src = 0,
          .dst = 2,
          .capacity_bps = 1000,
          .delay_ms = 1,
      },
      {
          .src = 1,
          .dst = 3,
          .capacity_bps = 1000,
          .delay_ms = 1,
      },
      {
          .src = 2,
          .dst = 3,
          .capacity_bps = 1000,
          .delay_ms = 1,
      },
  };

  SPFPathProvider path_provider(nodes, links);
  EXPECT_THAT(
      path_provider.NextBestPath(FG{0, 3}, {}),
      testing::AnyOf(testing::ElementsAre(0, 2), testing::ElementsAre(1, 3)));

  EXPECT_THAT(path_provider.NextBestPath(FG{0, 3}, {0}),
              testing::ElementsAre(1, 3));
  EXPECT_THAT(path_provider.NextBestPath(FG{0, 3}, {2}),
              testing::ElementsAre(1, 3));

  EXPECT_THAT(path_provider.NextBestPath(FG{0, 3}, {1}),
              testing::ElementsAre(0, 2));
  EXPECT_THAT(path_provider.NextBestPath(FG{0, 3}, {3}),
              testing::ElementsAre(0, 2));

  EXPECT_THAT(path_provider.NextBestPath(FG{0, 3}, {0, 1}), testing::IsEmpty());
}

}  // namespace
}  // namespace routing_algos
