#ifndef ULUYOL_ROUTING_ALGOS_B4_H_
#define ULUYOL_ROUTING_ALGOS_B4_H_

#include <stdint.h>

#include <ostream>
#include <vector>

#include "absl/container/btree_set.h"
#include "absl/container/flat_hash_set.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "path_provider.h"
#include "types.h"

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

namespace internal {

struct B4FGState {
  B4FGState(FG flowgroup, const BandwidthFunc& fn)
      : fg(flowgroup),
        active_steps(fn.Func()),
        fair_share(0),
        alloc_bps(0),
        current_path_capacity(0) {}

  FG fg;
  absl::Span<const BandwidthFunc::Step> active_steps;
  double fair_share;
  int64_t alloc_bps;

  // Path Allocation
  //
  // All path manipulation happens in B4::Solve so that we can avoid counting
  // paths with zero capacity against the path budget (unless there are no other
  // options).
  //
  // To make this work, we stash the current_path's capacity here in
  // B4::FreezeFGs.
  Path current_path;
  double current_path_capacity;
  PathSplit path_to_capacity;
};

}  // namespace internal

class B4LinkProblem {
 public:
  explicit B4LinkProblem(LinkId link_id, int64_t capacity_bps);

  void Add(internal::B4FGState* state);
  void Remove(FG fg, int64_t used_capacity_bps);

  double MaxMinFairShare();
  void MarkBottleneck();

  int64_t capacity_bps() const;
  const absl::btree_map<FG, internal::B4FGState*>& FGStates() const;

 private:
  double MaxMinFairShareNoCache();

  const LinkId link_id_;
  double capacity_bps_;
  bool is_bottleneck_;
  absl::btree_map<FG, internal::B4FGState*> fg_states_;

  // State used during allocation when we don't want to manipulate the B4FGState
  // right away.
  struct FGLinkState {
    FG fg;
    absl::Span<const BandwidthFunc::Step> steps;
    double fair_share;
    int64_t alloc_bps;
  };

  friend std::ostream& operator<<(std::ostream& os, const FGLinkState& state);

  std::vector<FGLinkState> scratch_;

  // cached_fair_share_ contains a cached value for the fair share.
  // It is invalidated whenever a new FG is added or removed.
  absl::optional<double> cached_fair_share_;
};

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
  void FreezeFGs(const double fair_share, const LinkId bottleneck_link,
                 std::vector<internal::B4FGState*>& frozen_fgs);

  bool AllLinksSaturated() const;
  bool AllDemandsSatisfied() const;

  const Config config_;
  std::unique_ptr<PathProvider> path_provider_;

  std::vector<B4LinkProblem> link_problems_;
  std::vector<internal::B4FGState> fg_states_;

  absl::flat_hash_set<LinkId> links_to_avoid_;
};

}  // namespace routing_algos

#endif  // ULUYOL_ROUTING_ALGOS_B4_H_
