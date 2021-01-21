#include "routing-algos/test-data.h"

namespace routing_algos {

namespace {

TestTopology RawTracedCloudflareNetwork() {
  std::vector<Node> nodes{
      {
          .name = "node_0",
          .transit_only = false,
      },
      {
          .name = "node_1",
          .transit_only = false,
      },
      {
          .name = "node_2",
          .transit_only = false,
      },
      {
          .name = "node_3",
          .transit_only = false,
      },
      {
          .name = "node_4",
          .transit_only = false,
      },
      {
          .name = "node_5",
          .transit_only = false,
      },
      {
          .name = "node_6",
          .transit_only = false,
      },
      {
          .name = "node_7",
          .transit_only = false,
      },
      {
          .name = "node_8",
          .transit_only = false,
      },
      {
          .name = "node_9",
          .transit_only = false,
      },
      {
          .name = "node_10",
          .transit_only = false,
      },
      {
          .name = "node_11",
          .transit_only = false,
      },
      {
          .name = "node_12",
          .transit_only = false,
      },
      {
          .name = "node_13",
          .transit_only = false,
      },
      {
          .name = "node_14",
          .transit_only = false,
      },
      {
          .name = "node_15",
          .transit_only = false,
      },
      {
          .name = "node_16",
          .transit_only = false,
      },
      {
          .name = "node_17",
          .transit_only = false,
      },
      {
          .name = "node_18",
          .transit_only = false,
      },
      {
          .name = "node_19",
          .transit_only = false,
      },
      {
          .name = "node_20",
          .transit_only = false,
      },
  };
  std::vector<Link> links{
      {
          // edge_0
          .src = 0,
          .dst = 1,
          .capacity_bps = 1000000000,
          .delay_ms = 3.929000,
      },
      {
          // edge_1
          .src = 0,
          .dst = 2,
          .capacity_bps = 1000000000,
          .delay_ms = 2.732000,
      },
      {
          // edge_2
          .src = 1,
          .dst = 0,
          .capacity_bps = 1000000000,
          .delay_ms = 3.929000,
      },
      {
          // edge_3
          .src = 1,
          .dst = 3,
          .capacity_bps = 1000000000,
          .delay_ms = 1.204000,
      },
      {
          // edge_4
          .src = 2,
          .dst = 0,
          .capacity_bps = 1000000000,
          .delay_ms = 2.732000,
      },
      {
          // edge_5
          .src = 2,
          .dst = 4,
          .capacity_bps = 1000000000,
          .delay_ms = 9.459000,
      },
      {
          // edge_6
          .src = 3,
          .dst = 1,
          .capacity_bps = 1000000000,
          .delay_ms = 1.204000,
      },
      {
          // edge_7
          .src = 3,
          .dst = 5,
          .capacity_bps = 1000000000,
          .delay_ms = 9.996000,
      },
      {
          // edge_8
          .src = 4,
          .dst = 2,
          .capacity_bps = 1000000000,
          .delay_ms = 9.459000,
      },
      {
          // edge_9
          .src = 4,
          .dst = 6,
          .capacity_bps = 1000000000,
          .delay_ms = 5.710000,
      },
      {
          // edge_10
          .src = 5,
          .dst = 3,
          .capacity_bps = 1000000000,
          .delay_ms = 9.996000,
      },
      {
          // edge_11
          .src = 5,
          .dst = 10,
          .capacity_bps = 1000000000,
          .delay_ms = 8.249000,
      },
      {
          // edge_12
          .src = 6,
          .dst = 4,
          .capacity_bps = 1000000000,
          .delay_ms = 5.710000,
      },
      {
          // edge_13
          .src = 6,
          .dst = 7,
          .capacity_bps = 1000000000,
          .delay_ms = 4.563000,
      },
      {
          // edge_14
          .src = 6,
          .dst = 8,
          .capacity_bps = 1000000000,
          .delay_ms = 3.812000,
      },
      {
          // edge_15
          .src = 7,
          .dst = 6,
          .capacity_bps = 1000000000,
          .delay_ms = 4.563000,
      },
      {
          // edge_16
          .src = 8,
          .dst = 6,
          .capacity_bps = 1000000000,
          .delay_ms = 3.812000,
      },
      {
          // edge_17
          .src = 8,
          .dst = 9,
          .capacity_bps = 1000000000,
          .delay_ms = 0.925000,
      },
      {
          // edge_18
          .src = 9,
          .dst = 8,
          .capacity_bps = 1000000000,
          .delay_ms = 0.925000,
      },
      {
          // edge_19
          .src = 9,
          .dst = 10,
          .capacity_bps = 1000000000,
          .delay_ms = 3.286000,
      },
      {
          // edge_20
          .src = 10,
          .dst = 5,
          .capacity_bps = 1000000000,
          .delay_ms = 8.249000,
      },
      {
          // edge_21
          .src = 10,
          .dst = 9,
          .capacity_bps = 1000000000,
          .delay_ms = 3.286000,
      },
      {
          // edge_22
          .src = 10,
          .dst = 13,
          .capacity_bps = 1000000000,
          .delay_ms = 38.474000,
      },
      {
          // edge_23
          .src = 10,
          .dst = 14,
          .capacity_bps = 1000000000,
          .delay_ms = 24.849000,
      },
      {
          // edge_24
          .src = 10,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 27.160000,
      },
      {
          // edge_25
          .src = 11,
          .dst = 13,
          .capacity_bps = 1000000000,
          .delay_ms = 5.536000,
      },
      {
          // edge_26
          .src = 12,
          .dst = 13,
          .capacity_bps = 1000000000,
          .delay_ms = 3.279000,
      },
      {
          // edge_27
          .src = 13,
          .dst = 10,
          .capacity_bps = 1000000000,
          .delay_ms = 38.474000,
      },
      {
          // edge_28
          .src = 13,
          .dst = 11,
          .capacity_bps = 1000000000,
          .delay_ms = 5.536000,
      },
      {
          // edge_29
          .src = 13,
          .dst = 12,
          .capacity_bps = 1000000000,
          .delay_ms = 3.279000,
      },
      {
          // edge_30
          .src = 14,
          .dst = 10,
          .capacity_bps = 1000000000,
          .delay_ms = 24.849000,
      },
      {
          // edge_31
          .src = 14,
          .dst = 15,
          .capacity_bps = 1000000000,
          .delay_ms = 2.004000,
      },
      {
          // edge_32
          .src = 14,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 2.313000,
      },
      {
          // edge_33
          .src = 15,
          .dst = 14,
          .capacity_bps = 1000000000,
          .delay_ms = 2.004000,
      },
      {
          // edge_34
          .src = 15,
          .dst = 17,
          .capacity_bps = 1000000000,
          .delay_ms = 2.571000,
      },
      {
          // edge_35
          .src = 16,
          .dst = 10,
          .capacity_bps = 1000000000,
          .delay_ms = 27.160000,
      },
      {
          // edge_36
          .src = 16,
          .dst = 14,
          .capacity_bps = 1000000000,
          .delay_ms = 2.313000,
      },
      {
          // edge_37
          .src = 16,
          .dst = 17,
          .capacity_bps = 1000000000,
          .delay_ms = 2.682000,
      },
      {
          // edge_38
          .src = 17,
          .dst = 15,
          .capacity_bps = 1000000000,
          .delay_ms = 2.571000,
      },
      {
          // edge_39
          .src = 17,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 2.682000,
      },
      {
          // edge_40
          .src = 17,
          .dst = 18,
          .capacity_bps = 1000000000,
          .delay_ms = 3.463000,
      },
      {
          // edge_41
          .src = 18,
          .dst = 17,
          .capacity_bps = 1000000000,
          .delay_ms = 3.463000,
      },
      {
          // edge_42
          .src = 18,
          .dst = 19,
          .capacity_bps = 1000000000,
          .delay_ms = 3.149000,
      },
      {
          // edge_43
          .src = 18,
          .dst = 20,
          .capacity_bps = 1000000000,
          .delay_ms = 5.892000,
      },
      {
          // edge_44
          .src = 19,
          .dst = 18,
          .capacity_bps = 1000000000,
          .delay_ms = 3.149000,
      },
      {
          // edge_45
          .src = 20,
          .dst = 18,
          .capacity_bps = 1000000000,
          .delay_ms = 5.892000,
      },
  };
  return TestTopology{nodes, links};
}

}  // namespace

TestTopology TracedCloudflareNetwork() {
  return DedupLinks(RawTracedCloudflareNetwork());
}

}  // namespace routing_algos
