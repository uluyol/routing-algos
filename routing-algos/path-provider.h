#ifndef ROUTING_ALGOS_PATH_PROVIDER_H_
#define ROUTING_ALGOS_PATH_PROVIDER_H_

#include "absl/container/flat_hash_set.h"
#include "routing-algos/types.h"

namespace routing_algos {

class PathProvider {
 public:
  virtual ~PathProvider() {}
  virtual Path NextBestPath(
      FG fg, const absl::flat_hash_set<LinkId>& links_to_avoid) const = 0;
};

}  // namespace routing_algos

#endif  // ROUTING_ALGOS_PATH_PROVIDER_H_