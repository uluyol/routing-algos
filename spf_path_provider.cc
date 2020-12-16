#include "spf_path_provider.h"

#include "third_party/ncode-common/algorithm.h"

namespace routing_algos {

SPFPathProvider::SPFPathProvider(const std::vector<Link>& links) {
  nc::net::GraphBuilder builder;

  NodeId max_node_id = 0;
  for (LinkId id = 0; id < links.size(); id++) {
    const Link& l = links[id];
    builder.AddLink({
        absl::StrCat(l.src),
        absl::StrCat(l.dst),
        nc::net::Bandwidth::FromBitsPerSecond(l.capacity_bps),
        std::chrono::microseconds(static_cast<uint64_t>(l.delay_ms * 1000)),
    });
    max_node_id = std::max({max_node_id, l.src, l.dst});
  }

  graph_ = absl::make_unique<nc::net::GraphStorage>(builder);
  nc_link_index_to_link_id_.resize(links.size(), -1);
  link_id_to_nc_link_index_.resize(links.size(), nc::net::GraphLinkIndex(-1));

  for (LinkId id = 0; id < links.size(); id++) {
    nc::net::GraphLinkIndex index = graph_->LinkOrDie(
        absl::StrCat(links[id].src), absl::StrCat(links[id].dst));
    nc_link_index_to_link_id_[index] = id;
    link_id_to_nc_link_index_[id] = index;
  }

  node_id_to_nc_node_index_.resize(max_node_id + 1,
                                   nc::net::GraphNodeIndex(-1));
  for (NodeId id = 0; id < node_id_to_nc_node_index_.size(); id++) {
    nc::net::GraphNodeIndex index =
        graph_->NodeFromStringOrDie(absl::StrCat(id));
    node_id_to_nc_node_index_[id] = index;
  }
}

Path SPFPathProvider::NextBestPath(
    FG fg, const absl::flat_hash_set<LinkId>& links_to_avoid) const {
  nc::net::ConstraintSet constraints;

  for (LinkId id : links_to_avoid) {
    constraints.Exclude().Links({link_id_to_nc_link_index_[id]});
  }

  std::unique_ptr<nc::net::Walk> walk = nc::net::ShortestPathWithConstraints(
      node_id_to_nc_node_index_[fg.src], node_id_to_nc_node_index_[fg.dst],
      *graph_, constraints);

  Path path;
  if (walk == nullptr) {
    return path;
  }

  path.reserve(walk->size());

  for (nc::net::GraphLinkIndex index : walk->links()) {
    path.push_back(nc_link_index_to_link_id_[index]);
  }

  return path;
}

}  // namespace routing_algos