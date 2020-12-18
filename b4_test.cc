#include "b4.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "spf_path_provider.h"

namespace routing_algos {
namespace {

std::vector<Link> LinearNetwork() {
  return std::vector<Link>{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
      {
          .src = 1,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
  };
}

MATCHER_P2(LinksNear, expected, margin_bps, "") {
  if (arg.size() != expected.size()) {
    return false;
  }
  for (LinkId id = 0; id < expected.size(); id++) {
    Link a = arg[id];
    Link b = expected[id];
    a.capacity_bps = 0;
    b.capacity_bps = 0;
    if (!(a == b)) {
      return false;
    }
    if (std::abs(a.capacity_bps - b.capacity_bps) > margin_bps) {
      return false;
    }
  }
  return true;
}

MATCHER_P2(PathSplitsNear, expected, margin_bps, "") {
  if (arg.size() != expected.size()) {
    return false;
  }
  for (const auto& fg_path_split_pair : expected) {
    const FG& fg = fg_path_split_pair.first;
    auto got_fg_path_split_iter = arg.find(fg);
    if (got_fg_path_split_iter == arg.end()) {
      return false;
    }
    const PathSplit& path_split_a = got_fg_path_split_iter->second;
    const PathSplit& path_split_b = fg_path_split_pair.second;
    if (path_split_a.size() != path_split_b.size()) {
      return false;
    }
    for (const auto& path_split_pair : path_split_b) {
      auto it = path_split_a.find(path_split_pair.first);
      if (it == path_split_a.end()) {
        return false;
      }
      double split_a = it->second;
      double split_b = path_split_pair.second;
      if (std::abs(split_a - split_b) > margin_bps) {
        return false;
      }
    }
  }
  return true;
}

TEST(B4Test, LinearFair_One_To_Three) {
  B4 b4(absl::make_unique<SPFPathProvider>(LinearNetwork()),
        {.path_budget_per_fg = 100});

  BandwidthFunc bw_func_1_2;
  bw_func_1_2.Push(4, 100);

  BandwidthFunc bw_func_0_2;
  bw_func_0_2.Push(1, 100);

  std::vector<Link> residual_links = LinearNetwork();
  PerFG<PathSplit> path_splits = b4.Solve(
      {
          {FG{1, 2}, bw_func_1_2},
          {FG{0, 2}, bw_func_0_2},
      },
      residual_links);

  std::vector<Link> expected_residual_links = LinearNetwork();
  expected_residual_links[0].capacity_bps = 20;
  expected_residual_links[1].capacity_bps = 0;

  EXPECT_THAT(residual_links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{1, 2}, PathSplit{{Path{1}, 20}}},
                               {FG{0, 2}, PathSplit{{Path{0, 1}, 80}}},
                           }));
}

TEST(B4Test, LinearFair_AllJustSatisfied) {
  B4 b4(absl::make_unique<SPFPathProvider>(LinearNetwork()),
        {.path_budget_per_fg = 100});

  BandwidthFunc bw_func_1_2;
  bw_func_1_2.Push(4, 70);

  BandwidthFunc bw_func_0_2;
  bw_func_0_2.Push(1, 30);

  std::vector<Link> residual_links = LinearNetwork();
  PerFG<PathSplit> path_splits = b4.Solve(
      {
          {FG{1, 2}, bw_func_1_2},
          {FG{0, 2}, bw_func_0_2},
      },
      residual_links);

  std::vector<Link> expected_residual_links = LinearNetwork();
  expected_residual_links[0].capacity_bps = 70;
  expected_residual_links[1].capacity_bps = 0;

  EXPECT_THAT(residual_links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{1, 2}, PathSplit{{Path{1}, 70}}},
                               {FG{0, 2}, PathSplit{{Path{0, 1}, 30}}},
                           }));
}

TEST(B4Test, LinearFair_LotsOfSpareCapacity) {
  B4 b4(absl::make_unique<SPFPathProvider>(LinearNetwork()),
        {.path_budget_per_fg = 100});

  BandwidthFunc bw_func_1_2;
  bw_func_1_2.Push(4, 40);

  BandwidthFunc bw_func_0_2;
  bw_func_0_2.Push(1, 30);

  std::vector<Link> residual_links = LinearNetwork();
  PerFG<PathSplit> path_splits = b4.Solve(
      {
          {FG{1, 2}, bw_func_1_2},
          {FG{0, 2}, bw_func_0_2},
      },
      residual_links);

  std::vector<Link> expected_residual_links = LinearNetwork();
  expected_residual_links[0].capacity_bps = 70;
  expected_residual_links[1].capacity_bps = 30;

  EXPECT_THAT(residual_links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{1, 2}, PathSplit{{Path{1}, 40}}},
                               {FG{0, 2}, PathSplit{{Path{0, 1}, 30}}},
                           }));
}

TEST(B4Test, LinearFair_UnroutableTraffic) {
  B4 b4(absl::make_unique<SPFPathProvider>(LinearNetwork()),
        {.path_budget_per_fg = 100});

  BandwidthFunc bw_func_1_2;
  bw_func_1_2.Push(4, 40);

  BandwidthFunc bw_func_2_0;
  bw_func_2_0.Push(1, 30);

  std::vector<Link> residual_links = LinearNetwork();
  PerFG<PathSplit> path_splits = b4.Solve(
      {
          {FG{1, 2}, bw_func_1_2},
          {FG{2, 0}, bw_func_2_0},
      },
      residual_links);

  std::vector<Link> expected_residual_links = LinearNetwork();
  expected_residual_links[0].capacity_bps = 100;
  expected_residual_links[1].capacity_bps = 60;

  EXPECT_THAT(residual_links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{1, 2}, PathSplit{{Path{1}, 40}}},
                               {FG{2, 0}, PathSplit{}},
                           }));
}

TEST(B4Test, LinearFair_ZeroDemandGetsShortestPath) {
  B4 b4(absl::make_unique<SPFPathProvider>(LinearNetwork()),
        {.path_budget_per_fg = 100});

  BandwidthFunc bw_func_1_2;
  bw_func_1_2.Push(4, 100);

  BandwidthFunc bw_func_0_2;
  bw_func_0_2.Push(1, 0);

  std::vector<Link> residual_links = LinearNetwork();
  PerFG<PathSplit> path_splits = b4.Solve(
      {
          {FG{1, 2}, bw_func_1_2},
          {FG{0, 2}, bw_func_0_2},
      },
      residual_links);

  std::vector<Link> expected_residual_links = LinearNetwork();
  expected_residual_links[0].capacity_bps = 100;
  expected_residual_links[1].capacity_bps = 0;

  EXPECT_THAT(residual_links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{1, 2}, PathSplit{{Path{1}, 100}}},
                               {FG{0, 2}, PathSplit{{Path{0, 1}, 0}}},
                           }));
}

TEST(B4Test, TriangleLowDemand) {
  std::vector<Link> links{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
      {
          .src = 0,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
      {
          .src = 1,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
  };

  B4 b4(absl::make_unique<SPFPathProvider>(links), {.path_budget_per_fg = 2});

  BandwidthFunc bw_func;
  bw_func.Push(1, 50);

  std::vector<Link> residual_links = links;
  PerFG<PathSplit> path_splits = b4.Solve(
      {
          {FG{1, 2}, bw_func},
          {FG{0, 2}, bw_func},
      },
      residual_links);

  std::vector<Link> expected_residual_links = links;
  expected_residual_links[1].capacity_bps = 50;
  expected_residual_links[2].capacity_bps = 50;

  EXPECT_THAT(residual_links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{1, 2}, PathSplit{{Path{2}, 50}}},
                               {FG{0, 2}, PathSplit{{Path{1}, 50}}},
                           }));
}

TEST(B4Test, TriangleUsesFasterPath) {
  std::vector<Link> links{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
      {
          .src = 0,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 100,
      },
      {
          .src = 1,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
  };

  B4 b4(absl::make_unique<SPFPathProvider>(links), {.path_budget_per_fg = 1});

  BandwidthFunc bw_func;
  bw_func.Push(1, 50);

  std::vector<Link> residual_links = links;
  PerFG<PathSplit> path_splits = b4.Solve(
      {
          {FG{1, 2}, bw_func},
          {FG{0, 2}, bw_func},
      },
      residual_links);

  std::vector<Link> expected_residual_links = links;
  expected_residual_links[0].capacity_bps = 50;
  expected_residual_links[1].capacity_bps = 100;
  expected_residual_links[2].capacity_bps = 0;

  EXPECT_THAT(residual_links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{1, 2}, PathSplit{{Path{2}, 50}}},
                               {FG{0, 2}, PathSplit{{Path{0, 2}, 50}}},
                           }));
}

TEST(B4Test, TriangleLowPriUsesJustSlowPath) {
  std::vector<Link> links{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
      {
          .src = 0,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 100,
      },
      {
          .src = 1,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
  };

  B4 b4(absl::make_unique<SPFPathProvider>(links), {.path_budget_per_fg = 2});

  BandwidthFunc bw_func_1_2;
  bw_func_1_2.Push(1, 100);

  BandwidthFunc bw_func_0_2;
  bw_func_0_2.Push(4, 0);
  bw_func_0_2.Push(5, 100);

  std::vector<Link> residual_links = links;
  PerFG<PathSplit> path_splits = b4.Solve(
      {
          {FG{1, 2}, bw_func_1_2},
          {FG{0, 2}, bw_func_0_2},
      },
      residual_links);

  std::vector<Link> expected_residual_links = links;
  expected_residual_links[1].capacity_bps = 0;
  expected_residual_links[2].capacity_bps = 0;

  EXPECT_THAT(residual_links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{1, 2}, PathSplit{{Path{2}, 100}}},
                               {FG{0, 2}, PathSplit{{Path{1}, 100}}},
                           }));
}

TEST(B4Test, TriangleLowPriUsesJustSlowPathWithBudgetOne) {
  std::vector<Link> links{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
      {
          .src = 0,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 100,
      },
      {
          .src = 1,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
  };

  // Difference from previous test is that path_budget_per_fg is 1
  B4 b4(absl::make_unique<SPFPathProvider>(links), {.path_budget_per_fg = 1});

  BandwidthFunc bw_func_1_2;
  bw_func_1_2.Push(1, 100);

  BandwidthFunc bw_func_0_2;
  bw_func_0_2.Push(4, 0);
  bw_func_0_2.Push(5, 100);

  std::vector<Link> residual_links = links;
  PerFG<PathSplit> path_splits = b4.Solve(
      {
          {FG{1, 2}, bw_func_1_2},
          {FG{0, 2}, bw_func_0_2},
      },
      residual_links);

  std::vector<Link> expected_residual_links = links;
  expected_residual_links[1].capacity_bps = 0;
  expected_residual_links[2].capacity_bps = 0;

  EXPECT_THAT(residual_links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{1, 2}, PathSplit{{Path{2}, 100}}},
                               {FG{0, 2}, PathSplit{{Path{1}, 100}}},
                           }));
}

std::vector<Link> Sigcomm13ExampleLinks() {
  return std::vector<Link>{
      {
          // Link 0
          .src = 0,  // A
          .dst = 1,  // B
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 1
          .src = 0,  // A
          .dst = 2,  // C
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 2
          .src = 0,  // A
          .dst = 3,  // D
          .capacity_bps = 10'000'000'000,
          .delay_ms = 10,
      },
      {
          // Link 3
          .src = 1,  // B
          .dst = 0,  // A
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 4
          .src = 1,  // B
          .dst = 2,  // C
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 5
          .src = 2,  // C
          .dst = 0,  // A
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 6
          .src = 2,  // C
          .dst = 1,  // B
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 7
          .src = 2,  // C
          .dst = 3,  // D
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 8
          .src = 3,  // D
          .dst = 0,  // A
          .capacity_bps = 10'000'000'000,
          .delay_ms = 10,
      },
      {
          // Link 9
          .src = 3,  // D
          .dst = 2,  // C
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
  };
}

PerFG<BandwidthFunc> Sigcomm13Example1Funcs() {
  BandwidthFunc func_a_b;
  func_a_b.Push(1.5, 17'000'000'000);
  func_a_b.Push(5, 20'000'000'000);
  BandwidthFunc func_a_c;
  func_a_c.Push(1.5, 1'000'000'000);
  func_a_c.Push(5, 2'500'000'000);
  func_a_c.Push(10, 5'000'000'000);

  return {
      {FG{0, 1}, func_a_b},
      {FG{0, 2}, func_a_c},
  };
}

TEST(B4Test, Sigcomm13Example1) {
  std::vector<Link> expected_residual_links = Sigcomm13ExampleLinks();
  expected_residual_links[0].capacity_bps = 0;              // A->B
  expected_residual_links[1].capacity_bps = 0;              // A->C
  expected_residual_links[6].capacity_bps = 0;              // C->B
  expected_residual_links[2].capacity_bps = 5'000'000'000;  // A->D
  expected_residual_links[9].capacity_bps = 5'000'000'000;  // D->C

  std::vector<Link> links = Sigcomm13ExampleLinks();
  B4 b4(absl::make_unique<SPFPathProvider>(links), {.path_budget_per_fg = 10});
  PerFG<PathSplit> path_splits = b4.Solve(Sigcomm13Example1Funcs(), links);

  EXPECT_THAT(links, LinksNear(expected_residual_links, 1));
  EXPECT_THAT(path_splits,
              PathSplitsNear(
                  PerFG<PathSplit>{
                      {FG{0, 1}, PathSplit{{Path{0}, 10'000'000'000},
                                           {Path{1, 6}, 8'333'333'333},
                                           {Path{2, 9, 6}, 1'666'666'667}}},
                      {FG{0, 2},
                       PathSplit{
                           {Path{1}, 1'666'666'666},
                           {Path{2, 9}, 3'333'333'333},
                       }},
                  },
                  1));
}

PerFG<BandwidthFunc> Sigcomm13Example2Funcs() {
  BandwidthFunc func_a_b;
  func_a_b.Push(1.5, 7'000'000'000);
  func_a_b.Push(5, 10'000'000'000);
  BandwidthFunc func_a_c;
  func_a_c.Push(1.5, 1'000'000'000);
  func_a_c.Push(5, 2'500'000'000);
  func_a_c.Push(10, 5'000'000'000);
  func_a_c.Push(20, 10'000'000'000);  // Seems OK? Missing detail in paper

  return {
      {FG{0, 1}, func_a_b},
      {FG{0, 2}, func_a_c},
  };
}
TEST(B4Test, Sigcomm13Example2) {
  std::vector<Link> expected_residual_links = Sigcomm13ExampleLinks();
  expected_residual_links[0].capacity_bps = 0;  // A->B
  expected_residual_links[1].capacity_bps = 0;  // A->C

  std::vector<Link> links = Sigcomm13ExampleLinks();
  B4 b4(absl::make_unique<SPFPathProvider>(links), {.path_budget_per_fg = 10});
  PerFG<PathSplit> path_splits = b4.Solve(Sigcomm13Example2Funcs(), links);

  EXPECT_THAT(links, testing::ContainerEq(expected_residual_links));
  EXPECT_THAT(path_splits, testing::Eq(PerFG<PathSplit>{
                               {FG{0, 1},
                                PathSplit{
                                    {Path{0}, 10'000'000'000},
                                }},
                               {FG{0, 2},
                                PathSplit{
                                    {Path{1}, 10'000'000'000},
                                }},
                           }));
}

}  // namespace
}  // namespace routing_algos
