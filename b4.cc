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

// FGState contains each flowgroup's allocation so far.
struct FGState {
  FGState(FG flowgroup, const BandwidthFunc& fn)
      : fg(flowgroup),
        active_steps(fn.Func()),
        fair_share(0),
        alloc_bps(0),
        current_path_capacity(0) {}

  FG fg;

  // active_steps are the parts of the BW function that have yet to be satisfied
  absl::Span<const BandwidthFunc::Step> active_steps;

  // Allocated fair_share and capacity (bps)
  double fair_share;
  int64_t alloc_bps;

  // Path Allocation
  //
  // All path manipulation happens in B4::Solve so that we can avoid counting
  // paths with zero capacity against the path budget (unless there are no other
  // options).
  //
  // To make this work, we stash the current_path's capacity here in FreezeLink.
  Path current_path;
  double current_path_capacity;
  PathSplit path_to_capacity;
};

// FGLinkState is temporary state used to compute the waterlevel on a given
// link.
struct FGLinkState {
  FG fg;
  absl::Span<const BandwidthFunc::Step> steps;
  double fair_share;
  int64_t alloc_bps;
};

double ComputeLinkWaterlevel(const bool is_double_check, const LinkId link_id,
                             const double initial_waterlevel,
                             double capacity_bps,
                             std::vector<FGLinkState>& link_states);

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
  double MaxMinFairShareNoCache(bool is_double_check);

  const LinkId link_id_;
  double capacity_bps_;
  bool is_bottleneck_;
  absl::btree_map<FG, FGState*> fg_states_;

  std::vector<FGLinkState> scratch_;

  // cached_fair_share_ contains a cached value for the fair share.
  // It is invalidated whenever a new FG is added or removed.
  absl::optional<double> cached_fair_share_;
};

std::ostream& operator<<(std::ostream& os, const FGLinkState& state) {
  return os << "{" << state.fg << " " << ToString(state.steps)
            << " share: " << state.fair_share << " bps: " << state.alloc_bps
            << "}";
}

LinkProblem::LinkProblem(LinkId link_id, int64_t capacity_bps)
    : link_id_(link_id), capacity_bps_(capacity_bps), is_bottleneck_(false) {}

void LinkProblem::Add(FGState* state) {
  ABSL_ASSERT(state != nullptr);
  ABSL_ASSERT(!is_bottleneck_);
  cached_fair_share_.reset();
  fg_states_.insert({state->fg, state});
}

void LinkProblem::Remove(FG fg, int64_t used_capacity_bps) {
  ABSL_ASSERT(!is_bottleneck_);
  cached_fair_share_.reset();
  fg_states_.erase(fg);
  capacity_bps_ -= used_capacity_bps;
  CHECK_GE(capacity_bps_, 0);
}

double LinkProblem::MaxMinFairShare() {
  if (cached_fair_share_) {
    if (kDebugB4) {
      double recomputed = MaxMinFairShareNoCache(true);
      CHECK(*cached_fair_share_ == recomputed)
          << absl::StreamFormat("cached fair share is %f, should be %f",
                                *cached_fair_share_, recomputed);
    }
  } else {
    ABSL_ASSERT(!is_bottleneck_);

    cached_fair_share_ = MaxMinFairShareNoCache(false);
  }

  return *cached_fair_share_;
}

double LinkProblem::MaxMinFairShareNoCache(bool is_double_check) {
  std::vector<FGLinkState>& link_states = scratch_;
  link_states.clear();

  // Collect the FGs with unsatisfied demand and record the current waterlevel
  double waterlevel = std::numeric_limits<double>::infinity();
  for (auto fg_state_pair : fg_states_) {
    FGState* state = fg_state_pair.second;
    waterlevel = std::min(waterlevel, state->fair_share);

    // Ignore FGs that have had their demand satisfied
    if (!state->active_steps.empty()) {
      ABSL_ASSERT(state->active_steps.front().fair_share >= state->fair_share);
      ABSL_ASSERT(state->active_steps.front().bps >= state->alloc_bps);

      link_states.push_back({
          .fg = fg_state_pair.first,
          .steps = state->active_steps,
          .fair_share = state->fair_share,
          .alloc_bps = state->alloc_bps,
      });
    }
  }

  if (link_states.empty()) {
    if (kDebugB4 && !is_double_check) {
      LOG(INFO) << "B4: Link " << link_id_ << ": fair share: " << waterlevel;
    }
    return waterlevel;
  }

  if (kDebugB4 && !is_double_check) {
    LOG(INFO) << absl::StreamFormat(
        "B4: Link %d: compute fair share; active states:\n%s", link_id_,
        absl::StrJoin(link_states, "\n",
                      [](std::string* out, const FGLinkState& link_state) {
                        out->append("\t");
                        absl::StreamFormatter()(out, link_state);
                      }));
  }

  return ComputeLinkWaterlevel(is_double_check, link_id_, waterlevel,
                               capacity_bps_, link_states);
}

double ComputeLinkWaterlevel(const bool is_double_check, const LinkId link_id,
                             const double initial_waterlevel,
                             double capacity_bps,
                             std::vector<FGLinkState>& link_states) {
  // Compute a max-min fair share with a progressive waterfill

  double waterlevel = initial_waterlevel;

  // Maximize the waterlevel while staying within the link's capacity.
  //
  // TODO: Maybe change the representation to make this allocation faster.
  // We iterate over link_states in its entirety very often.
  // Another possibility: cache the result of this function.
  while (capacity_bps > 1 && !link_states.empty()) {
    if (kDebugB4 && !is_double_check) {
      LOG(INFO) << absl::StreamFormat(
          "B4: Link %d: next iteration: capacity_bps: %f #link_states: %d",
          link_id, capacity_bps, link_states.size());
    }

    // Find the largest fair share that we can possibly increase the waterlevel
    // to in a single step (i.e. where the slope of the sum of BW functions is
    // constant).
    double next_step_share = std::numeric_limits<double>::infinity();
    for (FGLinkState& state : link_states) {
      // We need to catch up any FGs that are behind others.
      if (state.fair_share > waterlevel) {
        next_step_share = std::min(next_step_share, state.fair_share);
      }
      // Only jump one step at a time to keep slope constant.
      next_step_share =
          std::min(next_step_share, state.steps.front().fair_share);
    }

    // Figure out how much capacity we need per fair share unit from here until
    // next_step_share.
    double bps_per_share = 0;
    for (FGLinkState& state : link_states) {
      if (state.fair_share < next_step_share) {
        bps_per_share += state.steps.front().bps_per_share;
      }
    }

    if (kDebugB4 && !is_double_check) {
      LOG(INFO) << absl::StreamFormat(
          "B4: Link %d: next_step_share: %f bps_per_share: %f", link_id,
          next_step_share, bps_per_share);
    }

    double share_diff = next_step_share - waterlevel;
    double bps_needed = bps_per_share * share_diff;
    if (bps_needed <= capacity_bps) {
      capacity_bps -= bps_needed;
    } else {
      share_diff *= capacity_bps / bps_needed;
      capacity_bps = 0;
    }
    waterlevel += share_diff;

    double double_check_alloc_bps = 0;
    for (size_t i = 0; i < link_states.size(); /* explicit index changes */) {
      FGLinkState& state = link_states[i];
      if (state.fair_share < waterlevel) {
        double to_add =
            (waterlevel - state.fair_share) * state.steps.front().bps_per_share;
        state.fair_share = waterlevel;
        state.alloc_bps += to_add;
        double_check_alloc_bps += to_add;
      }
      if (state.fair_share == waterlevel) {
        if (!state.steps.empty() &&
            state.steps.front().fair_share <= waterlevel) {
          state.steps.remove_prefix(1);
          if (state.steps.empty()) {
            // Current FG is satisfied, remove from active list.
            link_states[i] = link_states.back();
            link_states.resize(link_states.size() - 1);
            continue;
          }
        }
      }
      // Done with this FG, move onto the next one
      i++;
    }
    constexpr double kAllocErrorTolerance = 0.5;
    CHECK_NEAR(double_check_alloc_bps, share_diff * bps_per_share,
               kAllocErrorTolerance);
  }

  if (kDebugB4 && !is_double_check) {
    LOG(INFO) << "B4: Link " << link_id << ": fair share: " << waterlevel;
  }
  return waterlevel;
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

void FreezeLink(const double fair_share, const LinkId bottleneck_link_id,
                std::vector<FGState*>& frozen_fgs, B4SolverState& solver) {
  LinkProblem& problem = solver.link_problems[bottleneck_link_id];

  frozen_fgs.clear();
  frozen_fgs.reserve(problem.FGStates().size());

  // Copy list of frozen FGs so we can mutate all link state.
  for (auto& fg_state_pair : problem.FGStates()) {
    frozen_fgs.push_back(fg_state_pair.second);
  }

  if (kDebugB4) {
    LOG(INFO) << "B4: freeze all " << frozen_fgs.size()
              << " FGs on bottleneck Link " << bottleneck_link_id
              << " and set fair_share ≥ " << fair_share;
  }

  for (FGState* state : frozen_fgs) {
    // First, allocate link bandwidth to FG
    int64_t initial_bps = state->alloc_bps;
    while (!state->active_steps.empty() &&
           (fair_share >= state->active_steps.front().fair_share ||
            fair_share > state->fair_share)) {
      auto& cur_step = state->active_steps.front();
      if (cur_step.fair_share <= fair_share) {
        ABSL_ASSERT(state->alloc_bps >= 0);
        ABSL_ASSERT(cur_step.bps_per_share >= 0);
        ABSL_ASSERT(cur_step.fair_share - state->fair_share >= 0);
        state->alloc_bps +=
            cur_step.bps_per_share * (cur_step.fair_share - state->fair_share);
        CHECK_NEAR(state->alloc_bps, cur_step.bps, 10);
        state->fair_share = cur_step.fair_share;

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

      // Don't try routing over a saturated link
      if (solver.link_problems[link_id].capacity_bps() <= 0) {
        solver.links_to_avoid.insert(link_id);
      }
    }

    // Third, record the path capacity.
    state->current_path_capacity = added_bps;
  }

  solver.links_to_avoid.insert(bottleneck_link_id);
  solver.link_problems[bottleneck_link_id].MarkBottleneck();
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
  ABSL_ASSERT(fair_share >= 0);
  ABSL_ASSERT(bps >= 0);

  if (fair_share == 0) {
    ABSL_ASSERT(bps == 0);
  }

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

// B4 Route Allocation Algorithm (SIGCOMM'13)
//
// First, each flow group is assigned to its shortest tunnel.
//
// Then, we search for the link that has the bottleneck fair share when all flow
// groups have their fair share increased together.
//
// The tricky part here is that a bottleneck on one link acts as a hard limit on
// how much bandwidth a flow group can get.
// If we look at links independently of one another, then it may seem that we
// will overestimate how much certain flow groups will get, and underestimate
// how much others will get.
//
// As it turns out, this doesn't matter:
// - Assume that we compute bottleneck fair share independently for each link,
//   and that Link i is the bottleneck with fair share s.
// - If some FG x traverses another Link j, the question is whether or not Link
//   j will throttle FG x to a bandwidith lower than Link i.
// - Let f(s) be the bandwidth function for FG x and s' be the fair share on
//   Link j (again, when looked at independently of other links).
// - We know (because Link i is the bottleneck), that s ≤ s'.
// - We also know that f(s) is monotonically increasing.
// - Hence f(s) ≤ f(s'), so it is impossible for Link j to bottleneck FG x more
//   than Link i.
//
// So, to identify which link is the bottleneck, we can simply look at the state
// of each link independently, and pick the one with the lowest fair share.
//
// Once we've done this, we stop using that link and add the paths that cross it
// to the set of paths used by the FGs. For FGs with more demand, we add the
// next-best path, and repeat this process.
//
// A few notes:
//
// - Staying within the path budget is slightly tricky:
//   We want to ensure that all FGs get a path, even those that have zero demand
//   or end up with zero capacity allocated to them, but we otherwise want to
//   ignore as they eat up the path budget with no contribution.
//
// - We need to account for FGs' previously allocated bandwidth when computing
//   the fair share on each link:
//   Consider FGs with an unweighted max-min fair allocation (i.e. all
//   constant, equal slopes in their bandwidth functions). If FG x gets 10 G on
//   its first path and is assigned a new path, we don't want to immediately
//   give it more bandwidth: it should only start to get bandwidth when others
//   have caught up and also gotten 10 G.
//
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
    if (state->current_path.empty()) {
      if (kDebugB4) {
        LOG(INFO) << "B4: no path for " << p.first;
      }
      // Set demand to zero so we don't try to allocate more for this FG
      state->active_steps = state->active_steps.subspan(0, 0);
    } else {
      if (kDebugB4) {
        LOG(INFO) << "B4: added path ["
                  << absl::StrJoin(state->current_path, " ") << "] for "
                  << p.first;
      }

      for (LinkId link_id : state->current_path) {
        ABSL_ASSERT(link_id >= 0);
        ABSL_ASSERT(link_id < solver.link_problems.size());
        solver.link_problems[link_id].Add(state);
      }
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

    FreezeLink(min_fair_share, link_id_of_min, frozen_fgs, solver);

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
        if (kDebugB4) {
          LOG(INFO) << "B4: added path ["
                    << absl::StrJoin(state->current_path, " ") << "] for "
                    << state->fg;
        }

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
}

}  // namespace routing_algos
