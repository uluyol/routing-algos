#include "routing-algos/conversion.h"

namespace routing_algos {

PathSplit BpsToFrac(const PathSplit& path_splits_bps) {
  double total_bps = 0;
  for (const auto& path_bps_pair : path_splits_bps) {
    total_bps += path_bps_pair.second;
  }
  PathSplit path_frac;
  if (total_bps == 0) {
    // No capacity, evenly distribute across paths
    for (const auto& path_bps_pair : path_splits_bps) {
      path_frac[path_bps_pair.first] =
          1 / static_cast<double>(path_splits_bps.size());
    }
  } else {
    // Otherwise, distribute in proportion to capacity
    for (const auto& path_bps_pair : path_splits_bps) {
      path_frac[path_bps_pair.first] = path_bps_pair.second / total_bps;
    }
  }
  return path_frac;
}

PerFG<PathSplit> BpsToFracAll(const PerFG<PathSplit>& fg_path_bps) {
  PerFG<PathSplit> fg_path_frac;

  for (const auto& fg_data : fg_path_bps) {
    fg_path_frac[fg_data.first] = BpsToFrac(fg_data.second);
  }

  return fg_path_frac;
}

}  // namespace routing_algos
