#ifndef ULUYOL_ROUTING_ALGOS_TYPES_H_
#define ULUYOL_ROUTING_ALGOS_TYPES_H_

#include <stdint.h>

#include <ostream>
#include <string>
#include <vector>

#include "absl/container/btree_map.h"

namespace routing_algos {

struct Node {
  std::string name;
};

using NodeId = uint32_t;

struct Link {
  NodeId src = 0;
  NodeId dst = 0;
  int64_t capacity_bps = 0;
  double delay_ms = 0;
};

bool operator==(const Link& lhs, const Link& rhs);
std::ostream& operator<<(std::ostream& os, const Link& link);

using LinkId = uint32_t;

using Path = std::vector<LinkId>;
using PathSplit = absl::btree_map<Path, double>;

struct FG {
  NodeId src = 0;
  NodeId dst = 0;
};

bool operator==(const FG& lhs, const FG& rhs);
bool operator<(const FG& lhs, const FG& rhs);
std::ostream& operator<<(std::ostream& os, const FG& fg);

template <typename T>
using PerFG = absl::btree_map<FG, T>;

}  // namespace routing_algos

#endif  // ULUYOL_ROUTING_ALGOS_TYPES_H_
