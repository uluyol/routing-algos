#include "b4.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"

namespace routing_algos {

namespace {

constexpr bool kDebugB4 = false;

struct ParenStepFormatter {
  void operator()(std::string* out, const BandwidthFunc::Step& step) {
    out->append(absl::StrFormat("(%d, %f)", step.bps, step.fair_share));
  }
};

std::string ToString(absl::Span<const BandwidthFunc::Step> steps) {
  return absl::StrCat("[", absl::StrJoin(steps, ",", ParenStepFormatter()),
                      "]");
}

struct FGState {
  FGState(FG flowgroup, const BandwidthFunc& fn)
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

class LinkProblem {
 public:
  explicit LinkProblem(LinkId link_id, int64_t capacity_bps);

  void Add(FGState* state);
  void Remove(FG fg, int64_t used_capacity_bps);

  double MaxMinFairShare();
  void MarkBottleneck();

  int64_t capacity_bps() const;
  const absl::btree_map<FG, FGState*>& FGStates() const;

 private:
  double MaxMinFairShareNoCache();

  const LinkId link_id_;
  double capacity_bps_;
  bool is_bottleneck_;
  absl::btree_map<FG, FGState*> fg_states_;

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

std::ostream& operator<<(std::ostream& os,
                         const LinkProblem::FGLinkState& state) {
  return os << "{" << state.fg << " " << ToString(state.steps)
            << " share: " << state.fair_share << " bps: " << state.alloc_bps
            << "}";
}

LinkProblem::LinkProblem(LinkId link_id, int64_t capacity_bps)
    : link_id_(link_id), capacity_bps_(capacity_bps), is_bottleneck_(false) {}

void LinkProblem::Add(FGState* state) {
  ABSL_ASSERT(!is_bottleneck_);
  cached_fair_share_.reset();
  fg_states_.insert({state->fg, state});
}

void LinkProblem::Remove(FG fg, int64_t used_capacity_bps) {
  ABSL_ASSERT(!is_bottleneck_);
  cached_fair_share_.reset();
  fg_states_.erase(fg);
  capacity_bps_ -= used_capacity_bps;
  ABSL_ASSERT(capacity_bps_ >= 0);
}

double LinkProblem::MaxMinFairShare() {
  if (cached_fair_share_) {
    if (kDebugB4) {
      double recomputed = MaxMinFairShareNoCache();
      CHECK(*cached_fair_share_ == recomputed)
          << absl::StreamFormat("cached fair share is %f, should be %f",
                                *cached_fair_share_, recomputed);
    }
  } else {
    cached_fair_share_ = MaxMinFairShareNoCache();
  }

  return *cached_fair_share_;
}

double LinkProblem::MaxMinFairShareNoCache() {
  ABSL_ASSERT(!is_bottleneck_);

  std::vector<FGLinkState>& active_states = scratch_;
  active_states.clear();

  double min_fair_share = std::numeric_limits<double>::infinity();
  for (auto fg_state_pair : fg_states_) {
    FGState* state = fg_state_pair.second;
    min_fair_share = std::min(min_fair_share, state->fair_share);

    if (!state->active_steps.empty()) {
      ABSL_ASSERT(state->active_steps.front().fair_share >= state->fair_share);
      ABSL_ASSERT(state->active_steps.front().bps >= state->alloc_bps);

      active_states.push_back({
          .fg = fg_state_pair.first,
          .steps = state->active_steps,
          .fair_share = state->fair_share,
          .alloc_bps = state->alloc_bps,
      });
    }
  }

  double cap_bps = capacity_bps_;
  if (kDebugB4) {
    LOG(INFO) << absl::StreamFormat(
        "Link-MM: computing fair share on Link %d; active states:\n%s",
        link_id_, absl::StrJoin(active_states, "\n", absl::StreamFormatter()));
  }

  // TODO: Maybe change the representation to make this allocation faster.
  // We iterate over active_states in its entirety very often.
  // Another possibility: cache the result of this function.
  while (cap_bps > 1 && !active_states.empty()) {
    if (kDebugB4) {
      LOG(INFO) << absl::StreamFormat(
          "Link-MM: next iteration: cap_bps: %f #active_states: %d", cap_bps,
          active_states.size());
    }
    double next_step_share = std::numeric_limits<double>::infinity();

    for (FGLinkState& state : active_states) {
      next_step_share =
          std::min(next_step_share, state.steps.front().fair_share);
    }

    double bps_per_share = 0;
    for (FGLinkState& state : active_states) {
      if (state.fair_share < next_step_share) {
        bps_per_share += state.steps.front().bps_per_share;
      }
    }

    if (kDebugB4) {
      LOG(INFO) << absl::StreamFormat(
          "Link-MM: next_step_share: %f bps_per_share: %f", next_step_share,
          bps_per_share);
    }

    double share_diff = next_step_share - min_fair_share;
    double bps_needed = bps_per_share * share_diff;
    if (bps_needed <= cap_bps) {
      cap_bps -= bps_needed;
    } else {
      share_diff = cap_bps / bps_needed;
      cap_bps = 0;
    }
    min_fair_share += share_diff;

    double double_check_alloc_bps = 0;
    for (size_t i = 0; i < active_states.size(); /* explicit index changes */) {
      FGLinkState& state = active_states[i];
      if (state.fair_share < min_fair_share) {
        double to_add = (min_fair_share - state.fair_share) *
                        state.steps.front().bps_per_share;
        state.fair_share = min_fair_share;
        state.alloc_bps += to_add;
        double_check_alloc_bps += to_add;
        if (state.steps.front().fair_share <= min_fair_share) {
          state.steps.remove_prefix(1);
          if (state.steps.empty()) {
            // Current FG is satisfied, remove from active list.
            active_states[i] = active_states.back();
            active_states.resize(active_states.size() - 1);
            continue;
          }
        }
      }
      i++;
    }
    constexpr double kAllocErrorTolerance = 0.5;
    ABSL_ASSERT(std::abs(double_check_alloc_bps - share_diff * bps_per_share) <=
                kAllocErrorTolerance);
  }

  if (kDebugB4) {
    LOG(INFO) << "Link-MM: fair share: " << min_fair_share;
  }
  return min_fair_share;
}

void LinkProblem::MarkBottleneck() {
  ABSL_ASSERT(!is_bottleneck_);
  is_bottleneck_ = true;
}

int64_t LinkProblem::capacity_bps() const { return capacity_bps_; }

const absl::btree_map<FG, FGState*>& LinkProblem::FGStates() const {
  return fg_states_;
}

struct B4SolverState {
  std::vector<LinkProblem> link_problems;
  std::vector<FGState> fg_states;
  absl::flat_hash_set<LinkId> links_to_avoid;
};

void FreezeFGs(const double fair_share, const LinkId bottleneck_link_id,
               std::vector<FGState*>& frozen_fgs, B4SolverState& solver) {
  if (kDebugB4) {
    LOG(INFO) << "B4: freeze all " << frozen_fgs.size()
              << " FGs on bottleneck link";
  }

  LinkProblem& problem = solver.link_problems[bottleneck_link_id];

  frozen_fgs.clear();
  frozen_fgs.reserve(problem.FGStates().size());

  // Copy list of frozen FGs so we can mutate all link state.
  for (auto& fg_state_pair : problem.FGStates()) {
    frozen_fgs.push_back(fg_state_pair.second);
  }

  for (FGState* state : frozen_fgs) {
    // First, allocate link bandwidth to FG
    int64_t initial_bps = state->alloc_bps;
    while (!state->active_steps.empty() &&
           (fair_share >= state->active_steps.front().fair_share ||
            fair_share > state->fair_share)) {
      auto& cur_step = state->active_steps.front();
      if (cur_step.fair_share <= fair_share) {
        state->alloc_bps +=
            cur_step.bps_per_share * (cur_step.fair_share - state->fair_share);
        state->fair_share = cur_step.fair_share;
        ABSL_ASSERT(state->alloc_bps == cur_step.bps);

        state->active_steps.remove_prefix(1);  // Done with this step
      } else /* cur_step.fair_share > fair_share */ {
        state->alloc_bps +=
            cur_step.bps_per_share * (fair_share - state->fair_share);
        state->fair_share = fair_share;

        // Haven't fully allocated this step yet
      }
    }

    // Second, subtract added BW from all links on the path
    int64_t added_bps = state->alloc_bps - initial_bps;
    for (LinkId link_id : state->current_path) {
      solver.link_problems[link_id].Remove(state->fg, added_bps);
    }

    // Third, record the path capacity.
    state->current_path_capacity = added_bps;
  }
}

bool AllLinksSaturated(const B4SolverState& s) {
  for (LinkId link_id = 0; link_id < s.link_problems.size(); link_id++) {
    if (!s.links_to_avoid.contains(link_id)) {
      return false;
    }
  }
  return true;
}

bool AllDemandsSatisfied(const B4SolverState& s) {
  for (auto& fg_state : s.fg_states) {
    if (!fg_state.active_steps.empty()) {
      return false;
    }
  }
  return true;
}

}  // namespace

void BandwidthFunc::Clear() { func_.clear(); }

void BandwidthFunc::Push(double fair_share, int64_t bps) {
  double last_bps = 0;
  double last_fair_share = 0;
  if (!func_.empty()) {
    last_bps = func_.back().bps;
    last_fair_share = func_.back().fair_share;

    if (kDebugB4) {
      CHECK(last_bps <= bps && last_fair_share <= fair_share);
    }
  }
  func_.push_back({
      .fair_share = fair_share,
      .bps = bps,
      .bps_per_share = (static_cast<double>(bps) - last_bps) /
                       (fair_share - last_fair_share),
  });
}

int64_t BandwidthFunc::DemandBps() const {
  if (func_.empty()) {
    return 0;
  }
  return func_.back().bps;
}

const std::vector<BandwidthFunc::Step>& BandwidthFunc::Func() const {
  return func_;
}

std::ostream& operator<<(std::ostream& os, const BandwidthFunc& func) {
  return os << ToString(absl::MakeSpan(func.Func()));
}

B4::B4(std::unique_ptr<PathProvider> path_provider, Config config)
    : config_(config), path_provider_(std::move(path_provider)) {}

PerFG<PathSplit> B4::Solve(const PerFG<BandwidthFunc>& bandwidth_funcs,
                           std::vector<Link>& links) {
  B4SolverState solver;

  solver.link_problems.reserve(links.size());
  for (LinkId id = 0; id < links.size(); id++) {
    solver.link_problems.push_back(LinkProblem(id, links[id].capacity_bps));
  }

  solver.fg_states.clear();
  solver.fg_states.reserve(bandwidth_funcs.size());
  for (auto& p : bandwidth_funcs) {
    solver.fg_states.push_back(FGState(p.first, p.second));
    auto state = &solver.fg_states.back();

    state->current_path =
        path_provider_->NextBestPath(state->fg, solver.links_to_avoid);

    for (LinkId link_id : state->current_path) {
      solver.link_problems[link_id].Add(state);
    }
  }

  std::vector<FGState*> frozen_fgs;

  while (!AllLinksSaturated(solver) && !AllDemandsSatisfied(solver)) {
    double min_fair_share = std::numeric_limits<double>::infinity();
    LinkId link_id_of_min = 0;

    for (LinkId link_id = 0; link_id < solver.link_problems.size(); link_id++) {
      if (solver.links_to_avoid.contains(link_id)) {
        continue;
      }

      double fair_share = solver.link_problems[link_id].MaxMinFairShare();
      if (fair_share < min_fair_share) {
        min_fair_share = fair_share;
        link_id_of_min = link_id;
      }
    }

    if (std::isinf(min_fair_share)) {
      break;  // no more work
    }

    if (kDebugB4) {
      LOG(INFO) << "B4: Link " << link_id_of_min << " is the bottleneck";
    }

    FreezeFGs(min_fair_share, link_id_of_min, frozen_fgs, solver);
    solver.links_to_avoid.insert(link_id_of_min);
    solver.link_problems[link_id_of_min].MarkBottleneck();

    // Add next-best paths for any frozen FGs
    for (FGState* state : frozen_fgs) {
      // FG's demand is zero. Use shortest path allocated.
      if ((state->active_steps.empty() && state->alloc_bps == 0) ||
          (!state->active_steps.empty() &&
           state->active_steps.back().bps <= 0)) {
        // use allocated path despite zero capacity
        state->path_to_capacity[state->current_path] =
            state->current_path_capacity;
        state->current_path.clear();
        state->current_path_capacity = 0;
        continue;
      }

      // FG's demand is non-zero, so we are in one of the following states:
      //
      // 1. The current path has non-zero capacity -> add it.
      //
      // 2. The current path has zero capacity and we have paths already
      //    -> ignore it. It will just waste the budget.
      //
      // 3. The current path has zero capacity and we have no other paths
      //    -> add it only if no next path exists.
      //

      Path next_path =
          path_provider_->NextBestPath(state->fg, solver.links_to_avoid);
      bool should_add_current = false;
      if (state->current_path_capacity > 0) {
        // State 1: always add path with capacity
        should_add_current = true;
      } else if (!state->path_to_capacity.empty()) {
        // State 2: we have paths already so ignore paths with zero capacity
        should_add_current = false;
      } else {
        // State 3: add only if next_path does not exist
        if (next_path.empty()) {
          should_add_current = true;
        } else {
          should_add_current = false;
        }
      }

      if (should_add_current) {
        state->path_to_capacity[std::move(state->current_path)] =
            state->current_path_capacity;
        state->current_path_capacity = 0;
      }

      if (state->path_to_capacity.size() >= config_.path_budget_per_fg) {
        // Set demand to zero so we don't try to allocate more for this FG
        state->active_steps = state->active_steps.subspan(0, 0);
      } else {
        state->current_path = std::move(next_path);

        for (LinkId link_id : state->current_path) {
          solver.link_problems[link_id].Add(state);
        }
      }
    }
  }

  for (LinkId link_id = 0; link_id < links.size(); link_id++) {
    links[link_id].capacity_bps = solver.link_problems[link_id].capacity_bps();
  }

  PerFG<PathSplit> path_splits;
  for (auto& state : solver.fg_states) {
    path_splits[state.fg] = std::move(state.path_to_capacity);
  }

  return path_splits;
}  // namespace routing_algos

}  // namespace routing_algos