#include "test_data.h"

namespace routing_algos {

TestTopology LinearNetwork() {
  std::vector<Node> nodes{
      {
          .name = "A",
          .transit_only = false,
      },
      {
          .name = "B",
          .transit_only = false,
      },
      {
          .name = "C",
          .transit_only = false,
      },
  };

  std::vector<Link> links{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
      {
          .src = 1,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
  };

  return {nodes, links};
}

TestTopology TriangleNetwork() {
  std::vector<Node> nodes{
      {
          .name = "A",
          .transit_only = false,
      },
      {
          .name = "B",
          .transit_only = false,
      },
      {
          .name = "C",
          .transit_only = false,
      },
  };

  std::vector<Link> links{
      {
          .src = 0,
          .dst = 1,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
      {
          .src = 0,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
      {
          .src = 1,
          .dst = 2,
          .capacity_bps = 100,
          .delay_ms = 1,
      },
  };
  return {nodes, links};
};

TestTopology FourNodeNetwork() {
  std::vector<Node> nodes{
      {
          .name = "A",
          .transit_only = false,
      },
      {
          .name = "B",
          .transit_only = false,
      },
      {
          .name = "C",
          .transit_only = false,
      },
      {
          .name = "D",
          .transit_only = false,
      },
  };

  std::vector<Link> links{
      {
          // Link 0
          .src = 0,  // A
          .dst = 1,  // B
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 1
          .src = 0,  // A
          .dst = 2,  // C
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 2
          .src = 0,  // A
          .dst = 3,  // D
          .capacity_bps = 10'000'000'000,
          .delay_ms = 10,
      },
      {
          // Link 3
          .src = 1,  // B
          .dst = 0,  // A
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 4
          .src = 1,  // B
          .dst = 2,  // C
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 5
          .src = 2,  // C
          .dst = 0,  // A
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 6
          .src = 2,  // C
          .dst = 1,  // B
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 7
          .src = 2,  // C
          .dst = 3,  // D
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
      {
          // Link 8
          .src = 3,  // D
          .dst = 0,  // A
          .capacity_bps = 10'000'000'000,
          .delay_ms = 10,
      },
      {
          // Link 9
          .src = 3,  // D
          .dst = 2,  // C
          .capacity_bps = 10'000'000'000,
          .delay_ms = 1,
      },
  };

  return {nodes, links};
}

}  // namespace routing_algos
