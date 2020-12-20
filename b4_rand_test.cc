// A very basic fuzz test that doesn't take advantage of any coverage
// information. Perhaps when fuzzing works on macOS, that can be fixed.

#include <cstddef>
#include <cstdint>
#include <random>

#include "absl/debugging/failure_signal_handler.h"
#include "absl/debugging/symbolize.h"
#include "b4.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "spf_path_provider.h"
#include "test_data.h"

namespace routing_algos {
namespace {

std::vector<FG> EnumerateFGs(const std::vector<Node> &nodes) {
  std::vector<FG> flowgroups;
  for (NodeId src_id = 0; src_id < nodes.size(); src_id++) {
    if (nodes[src_id].transit_only) {
      continue;
    }
    for (NodeId dst_id = 0; dst_id < nodes.size(); dst_id++) {
      if (src_id == dst_id) {
        continue;
      }
      if (nodes[dst_id].transit_only) {
        continue;
      }
      flowgroups.push_back({src_id, dst_id});
    }
  }
  return flowgroups;
}

class MiniFuzzerState {
 public:
  explicit MiniFuzzerState(uint64_t seed) : rng_(seed) {}

  template <typename T>
  uint64_t Int(T min, T max) {
    return std::uniform_int_distribution<T>(min, max)(rng_);
  }

  double Double(double min, double max) {
    return std::uniform_real_distribution<double>(min, max)(rng_);
  }

 private:
  std::mt19937_64 rng_;
};

void FuzzB4(MiniFuzzerState &state) {
  // This set of networks should not change.
  static std::vector<TestTopology> test_topologies = {
      LinearNetwork(),       TriangleNetwork(),  FourNodeNetwork(),
      TracedAkamaiNetwork(), TracedAWSNetwork(), TracedCloudflareNetwork(),
      TracedB4Network(),
  };

  const size_t topology_idx = state.Int<size_t>(0, test_topologies.size());

  TestTopology topology = test_topologies[topology_idx];

  for (Link &l : topology.links) {
    l.capacity_bps = state.Int<int64_t>(0, 100'000'000'000);
    l.delay_ms = state.Double(0.0001, 500);
  }

  auto flowgroups = EnumerateFGs(topology.nodes);
  PerFG<BandwidthFunc> funcs;
  const uint8_t num_fgs = state.Int<uint8_t>(0, 65);
  while (funcs.size() < num_fgs) {
    FG fg = flowgroups[state.Int<uint32_t>(0, flowgroups.size())];
    if (funcs.contains(fg)) {
      continue;  // already added
    }

    uint8_t num_steps = state.Int<uint8_t>(0, 5);
    BandwidthFunc func;
    double total_fair_share = 0;
    int64_t total_bps = 0;
    while (func.Func().size() < num_steps) {
      total_fair_share += state.Double(0, 3);
      total_bps += state.Int<int64_t>(0, 100'000'000'000);

      if (total_fair_share == 0 && total_bps > 0) {
        continue;
      }

      func.Push(total_fair_share, total_bps);
    }

    funcs[fg] = func;
  }

  uint8_t max_paths_per_fg = state.Int<uint8_t>(0, 17);

  B4 b4(absl::make_unique<SPFPathProvider>(topology.links),
        {
            .path_budget_per_fg = max_paths_per_fg,
        });

  std::vector<Link> links = topology.links;
  PerFG<PathSplit> fg_path_splits = b4.Solve(funcs, links);

  std::vector<Link> residual_check = topology.links;

  // Check the outputs
  for (const auto &fg_path_splits_pair : fg_path_splits) {
    FG fg = fg_path_splits_pair.first;

    for (const auto &path_split_pair : fg_path_splits_pair.second) {
      const Path &p = path_split_pair.first;
      CHECK_EQ(topology.links[p.front()].src, fg.src);
      CHECK_EQ(topology.links[p.back()].dst, fg.dst);
      for (size_t i = 1; i < p.size(); i++) {
        CHECK_EQ(topology.links[p[i]].src, topology.links[p[i - 1]].dst)
            << "links are not adjacent";
      }

      int64_t bps_used = path_split_pair.second;
      for (LinkId id : p) {
        residual_check[id].capacity_bps -= bps_used;
      }
    }
  }

  for (LinkId id = 0; id < residual_check.size(); id++) {
    int64_t max_bps =
        std::max(links[id].capacity_bps, residual_check[id].capacity_bps);
    CHECK_NEAR(links[id].capacity_bps, residual_check[id].capacity_bps,
               std::max<double>(max_bps / 50, 2));
  }
}

}  // namespace
}  // namespace routing_algos

DEFINE_int32(num_iters, 64, "number of iterations to execute");
DEFINE_int64(seed, 0, "seed to use for random number generation");

int main(int argc, char **argv) {
  absl::InitializeSymbolizer(argv[0]);
  absl::InstallFailureSignalHandler(absl::FailureSignalHandlerOptions());
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  for (int32_t iter = 0; iter < FLAGS_num_iters; iter++) {
    LOG(INFO) << "iteration: " << iter;
    routing_algos::MiniFuzzerState state(static_cast<uint64_t>(FLAGS_seed) *
                                         static_cast<uint64_t>(iter));

    routing_algos::FuzzB4(state);
  }
}
