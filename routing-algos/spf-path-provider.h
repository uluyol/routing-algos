#ifndef ROUTING_ALGOS_SPF_PATH_PROVIDER_H_
#define ROUTING_ALGOS_SPF_PATH_PROVIDER_H_

#include <vector>

#include "absl/types/optional.h"
#include "routing-algos/path-provider.h"
#include "routing-algos/third_party/ncode-common/algorithm.h"
#include "routing-algos/types.h"

namespace routing_algos {

class SPFPathProvider : public PathProvider {
 public:
  explicit SPFPathProvider(const std::vector<Node>& nodes,
                           const std::vector<Link>& links);

  Path NextBestPath(FG fg,
                    const absl::flat_hash_set<LinkId>& links_to_avoid) const;

 private:
  std::unique_ptr<nc::net::GraphStorage> graph_;
  std::vector<LinkId> nc_link_index_to_link_id_;
  std::vector<absl::optional<nc::net::GraphLinkIndex>>
      link_id_to_nc_link_index_;
  std::vector<absl::optional<nc::net::GraphNodeIndex>>
      node_id_to_nc_node_index_;
};

}  // namespace routing_algos

#endif  // ROUTING_ALGOS_SPF_PATH_PROVIDER_H_