#include "routing-algos/spf-path-provider.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "glog/logging.h"
#include "routing-algos/third_party/ncode-common/algorithm.h"

namespace routing_algos {
namespace {
#ifdef NDEBUG
constexpr bool kValidateSPFMapping = false;
#else
constexpr bool kValidateSPFMapping = true;
#endif

std::string MappingToString(const std::vector<LinkId>& mapping) {
  std::vector<std::string> parts;
  for (ssize_t i = 0; i < mapping.size(); i++) {
    ssize_t j = static_cast<ssize_t>(mapping[i]);
    parts.push_back(absl::Substitute("$0->$1", i, j));
  }
  return absl::StrCat("[", absl::StrJoin(parts, " "), "]");
}

std::string MappingToString(
    const std::vector<absl::optional<nc::net::GraphLinkIndex>>& mapping) {
  std::vector<std::string> parts;
  for (ssize_t i = 0; i < mapping.size(); i++) {
    if (mapping[i]) {
      ssize_t j = static_cast<ssize_t>(mapping[i].value());
      parts.push_back(absl::Substitute("$0->$1", i, j));
    } else {
      parts.push_back(absl::Substitute("$0->-1", i));
    }
  }
  return absl::StrCat("[", absl::StrJoin(parts, " "), "]");
}

}  // namespace

SPFPathProvider::SPFPathProvider(const std::vector<Node>& nodes,
                                 const std::vector<Link>& links) {
  nc::net::GraphBuilder builder;

  for (LinkId id = 0; id < links.size(); id++) {
    const Link& l = links[id];
    builder.AddLink({
        absl::StrCat(l.src),
        absl::StrCat(l.dst),
        nc::net::Bandwidth::FromBitsPerSecond(l.capacity_bps),
        std::chrono::microseconds(static_cast<uint64_t>(l.delay_ms * 1000)),
    });
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

  if (kValidateSPFMapping) {
    bool all_ok = true;

    for (LinkId id = 0; all_ok && id < link_id_to_nc_link_index_.size(); id++) {
      if (link_id_to_nc_link_index_[id] < 0) {
        all_ok = false;
      }
      if (link_id_to_nc_link_index_[id] >= links.size()) {
        all_ok = false;
      }
    }

    for (nc::net::GraphLinkIndex index(0);
         all_ok && index < nc_link_index_to_link_id_.size();
         index = nc::net::GraphLinkIndex(static_cast<size_t>(index) + 1)) {
      if (nc_link_index_to_link_id_[index] < 0) {
        all_ok = false;
      }
      if (nc_link_index_to_link_id_[index] >= links.size()) {
        all_ok = false;
      }
    }

    if (!all_ok) {
      LOG(FATAL) << "Bad link id mapping:\nLinkId -> nc::net::GraphLinkIndex = "
                 << MappingToString(link_id_to_nc_link_index_)
                 << "\nnc::net::GraphLinkIndex -> LinkId = "
                 << MappingToString(nc_link_index_to_link_id_);
    }
  }

  node_id_to_nc_node_index_.resize(nodes.size(), absl::nullopt);
  for (NodeId id = 0; id < node_id_to_nc_node_index_.size(); id++) {
    const nc::net::GraphNodeIndex* index =
        graph_->NodeFromStringOrNull(absl::StrCat(id));
    if (index != nullptr) {
      node_id_to_nc_node_index_[id] = *index;
    }
  }
}

Path SPFPathProvider::NextBestPath(
    FG fg, const absl::flat_hash_set<LinkId>& links_to_avoid) const {
  nc::net::ConstraintSet constraints;

  for (LinkId id : links_to_avoid) {
    constraints.Exclude().Links({link_id_to_nc_link_index_[id].value()});
  }

  auto src_index = node_id_to_nc_node_index_[fg.src];
  auto dst_index = node_id_to_nc_node_index_[fg.dst];

  if (!src_index.has_value() || !dst_index.has_value()) {
    // Unknown src or dst: can happen when no links contain the node.
    // Since no links contains src or dst, there can't possibly be any path.
    return Path{};
  }

  std::unique_ptr<nc::net::Walk> walk = nc::net::ShortestPathWithConstraints(
      src_index.value(), dst_index.value(), *graph_, constraints);

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
