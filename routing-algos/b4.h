#ifndef ROUTING_ALGOS_B4_H_
#define ROUTING_ALGOS_B4_H_

#include <stdint.h>

#include <ostream>
#include <vector>

#include "absl/container/btree_set.h"
#include "absl/container/flat_hash_set.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "routing-algos/path-provider.h"
#include "routing-algos/types.h"

namespace routing_algos {

class BandwidthFunc {
 public:
  struct Step {
    double fair_share;
    int64_t bps;
    double bps_per_share;
  };

  void Clear();
  void Push(double fair_share, int64_t bps);

  int64_t DemandBps() const;
  const std::vector<Step>& Func() const;

 private:
  std::vector<Step> func_;
};

std::ostream& operator<<(std::ostream& os, const BandwidthFunc& func);

class B4 {
 public:
  struct Config {
    // Maximum number of paths that can be created for each FG
    int32_t path_budget_per_fg = 1;
  };

  B4(std::unique_ptr<PathProvider> path_provider, Config config);

  PerFG<PathSplit> Solve(const PerFG<BandwidthFunc>& bandwidth_funcs,
                         std::vector<Link>& links);

 private:
  const Config config_;
  std::unique_ptr<PathProvider> path_provider_;
};

}  // namespace routing_algos

#endif  // ROUTING_ALGOS_B4_H_
