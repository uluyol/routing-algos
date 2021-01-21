#include "routing-algos/types.h"

#include "absl/strings/substitute.h"

namespace routing_algos {

bool operator==(const Link& lhs, const Link& rhs) {
  return (lhs.src == rhs.src) && (lhs.dst == rhs.dst) &&
         (lhs.capacity_bps == rhs.capacity_bps) &&
         (lhs.delay_ms == rhs.delay_ms);
}

std::ostream& operator<<(std::ostream& os, const Link& link) {
  return os << absl::Substitute("Link{$0->$1, $2 bps, $3 ms}", link.src,
                                link.dst, link.capacity_bps, link.delay_ms);
}

bool operator==(const FG& lhs, const FG& rhs) {
  return (lhs.src == rhs.src) && (lhs.dst == rhs.dst);
}

bool operator<(const FG& lhs, const FG& rhs) {
  if (lhs.src == rhs.src) {
    return lhs.dst < rhs.dst;
  }
  return lhs.src < rhs.src;
}

std::ostream& operator<<(std::ostream& os, const FG& fg) {
  return os << absl::Substitute("$0->$1", fg.src, fg.dst);
}

}  // namespace routing_algos
