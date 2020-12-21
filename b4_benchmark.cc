#include <random>

#include "absl/memory/memory.h"
#include "b4.h"
#include "benchmark/benchmark.h"
#include "path_provider.h"

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

BandwidthFunc MakeFunc(std::mt19937_64& rng, int n, int num_fgs) {
  BandwidthFunc fn;
  int64_t total_bps = 0;
  for (int i = 0; i < n; i++) {
    total_bps +=
        std::uniform_int_distribution<int>(0, 100'000'000 / num_fgs)(rng);
    fn.Push(i + 1, total_bps);
  }
  return fn;
}

void BM_B4_Waterfill(benchmark::State& state) {
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
        MakeFunc(rng, state.range(1), state.range(0));
  }

  for (auto _ : state) {
    // This code gets timed
    std::vector<Link> residual_links = links;
    benchmark::DoNotOptimize(b4.Solve(funcs, links));
  }
}

BENCHMARK(BM_B4_Waterfill)
    ->Args({16, 1})
    ->Args({16, 4})
    ->Args({16, 16})
    ->Args({64, 1})
    ->Args({64, 4})
    ->Args({64, 16})
    ->Args({256, 1})
    ->Args({256, 4})
    ->Args({256, 16});

}  // namespace
}  // namespace routing_algos

BENCHMARK_MAIN();
