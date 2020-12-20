#ifndef ULUYOL_ROUTING_ALGOS_SPF_PATH_PROVIDER_H_
#define ULUYOL_ROUTING_ALGOS_SPF_PATH_PROVIDER_H_

#include <vector>

#include "path_provider.h"
#include "third_party/ncode-common/algorithm.h"
#include "types.h"

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
  std::vector<nc::net::GraphLinkIndex> link_id_to_nc_link_index_;
  std::vector<nc::net::GraphNodeIndex> node_id_to_nc_node_index_;
};

}  // namespace routing_algos

#endif  // ULUYOL_ROUTING_ALGOS_SPF_PATH_PROVIDER_H_