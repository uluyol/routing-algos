#ifndef ULUYOL_ROUTING_ALGOS_PATH_PROVIDER_H_
#define ULUYOL_ROUTING_ALGOS_PATH_PROVIDER_H_

#include "absl/container/flat_hash_set.h"
#include "types.h"

namespace routing_algos {

class PathProvider {
 public:
  virtual ~PathProvider() {}
  virtual Path NextBestPath(
      FG fg, const absl::flat_hash_set<LinkId>& links_to_avoid) const = 0;
};

}  // namespace routing_algos

#endif  // ULUYOL_ROUTING_ALGOS_PATH_PROVIDER_H_