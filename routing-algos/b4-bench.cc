#include <random>

#include "absl/memory/memory.h"
#include "benchmark/benchmark.h"
#include "routing-algos/alg/max-min-fairness.h"
#include "routing-algos/b4.h"
#include "routing-algos/path-provider.h"

namespace routing_algos {
namespace {

// We benchmark the waterfill using the following topology.
//
//     N0    N1   ..   N[k-1]
//     |     |           |
//     L0    L1        L[k-1]
//     \_____|___________/
//          N[k]
//           |
//          L[k]
//           |
//         N[k+1]
//
// Link k is the bottleneck for all FGs.
//
// The only path for any FG i is L[i]->L[k].

class BenchWaterfillPathProvider : public PathProvider {
 public:
  explicit BenchWaterfillPathProvider(int num_fgs) : num_fgs_(num_fgs) {}

  Path NextBestPath(FG fg,
                    const absl::flat_hash_set<LinkId>& links_to_avoid) const {
    if (links_to_avoid.contains(LinkId(fg.src)) ||
        links_to_avoid.contains(LinkId(num_fgs_))) {
      return Path{};
    }
    return Path{LinkId(fg.src), LinkId(num_fgs_)};
  }

 private:
  int num_fgs_;
};

BandwidthFunc MakeFunc(std::mt19937_64& rng, int n, bool weighted,
                       int num_fgs) {
  BandwidthFunc fn;
  int64_t total_bps = 0;
  for (int i = 0; i < n; i++) {
    total_bps +=
        std::uniform_int_distribution<int>(0, 100'000'000 / num_fgs)(rng);
    double fair_share = i + 1;
    if (!weighted) {
      // Allocation should increase at the same rate for all FGs
      fair_share = total_bps;
    }
    fn.Push(fair_share, total_bps);
  }
  return fn;
}

void BenchB4SingleLink(benchmark::State& state, int num_func_steps,
                       bool weighted) {
  // Create network
  std::vector<Link> links;
  links.reserve(state.range(0) + 1);
  for (int i = 0; i < state.range(0); i++) {
    links.push_back({
        .src = NodeId(i),
        .dst = NodeId(state.range(0)),
        .capacity_bps = 1'000'000'000,
        .delay_ms = 1,
    });
  }
  links.push_back({
      .src = NodeId(state.range(0)),
      .dst = NodeId(state.range(0) + 1),
      .capacity_bps = 1'000'000'000,
      .delay_ms = 1,
  });

  B4 b4(absl::make_unique<BenchWaterfillPathProvider>(state.range(0)),
        {.path_budget_per_fg = 1});

  std::mt19937_64 rng(0);

  PerFG<BandwidthFunc> funcs;
  for (NodeId id = 0; id < state.range(0); id++) {
    funcs[FG{id, NodeId(state.range(0))}] =
        MakeFunc(rng, num_func_steps, weighted, state.range(0));
  }

  for (auto _ : state) {
    // This code gets timed
    std::vector<Link> residual_links = links;
    benchmark::DoNotOptimize(b4.Solve(funcs, links));
  }
}

void BM_B4_SingleLink_BandwidthFunc_8(benchmark::State& state) {
  BenchB4SingleLink(state, 8, true);
}

BENCHMARK(BM_B4_SingleLink_BandwidthFunc_8)
    ->RangeMultiplier(10)
    ->Range(10, 1000);

void BM_B4_SingleLink_Unweighted(benchmark::State& state) {
  BenchB4SingleLink(state, 1, false);
}

void BenchSingleLinkMaxMinFairnessProblem(
    benchmark::State& state, SingleLinkMaxMinFairnessProblem* problem) {
  std::vector<std::vector<int64_t>> demands(
      4, std::vector<int64_t>(state.range(0) / 4, 0));
  std::mt19937_64 rng(0);
  for (size_t i = 0; i < demands.size(); i++) {
    for (size_t j = 0; j < demands[i].size(); j++) {
      demands[i][j] = std::uniform_int_distribution<int64_t>(
          0, 100'000'000 / state.range(0))(rng);
    }
  }
  const int64_t capacity = 1'000'000'000;
  std::vector<std::vector<int64_t>> allocs;
  for (auto _ : state) {
    int64_t waterlevel = problem->ComputeWaterlevel(capacity, demands);
    problem->SetAllocations(waterlevel, demands, &allocs);
  }
  benchmark::DoNotOptimize(allocs);
}

void BM_MMF_SingleLink_Unweighted(benchmark::State& state) {
  SingleLinkMaxMinFairnessProblem problem({.enable_tiny_flow_opt = true});
  BenchSingleLinkMaxMinFairnessProblem(state, &problem);
}

BENCHMARK(BM_B4_SingleLink_Unweighted)->RangeMultiplier(10)->Range(10, 1000);
BENCHMARK(BM_MMF_SingleLink_Unweighted)->RangeMultiplier(10)->Range(10, 1000);

}  // namespace
}  // namespace routing_algos

BENCHMARK_MAIN();
