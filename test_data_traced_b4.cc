#include "test_data.h"

namespace routing_algos {

TestTopology TracedB4Topology() {
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
  };
  std::vector<Link> links{
    {
      // edge_0
      .src = 0,
      .dst = 1,
      .capacity_bps = 1000000000,
      .delay_ms = 12.654000,
    },
    {
      // edge_1
      .src = 0,
      .dst = 2,
      .capacity_bps = 1000000000,
      .delay_ms = 15.209000,
    },
    {
      // edge_2
      .src = 0,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 24.862000,
    },
    {
      // edge_3
      .src = 0,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 26.206000,
    },
    {
      // edge_4
      .src = 1,
      .dst = 0,
      .capacity_bps = 1000000000,
      .delay_ms = 12.654000,
    },
    {
      // edge_5
      .src = 1,
      .dst = 2,
      .capacity_bps = 1000000000,
      .delay_ms = 3.300000,
    },
    {
      // edge_6
      .src = 1,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 12.909000,
    },
    {
      // edge_7
      .src = 1,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 14.412000,
    },
    {
      // edge_8
      .src = 1,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 51.170000,
    },
    {
      // edge_9
      .src = 2,
      .dst = 0,
      .capacity_bps = 1000000000,
      .delay_ms = 15.209000,
    },
    {
      // edge_10
      .src = 2,
      .dst = 1,
      .capacity_bps = 1000000000,
      .delay_ms = 3.300000,
    },
    {
      // edge_11
      .src = 3,
      .dst = 0,
      .capacity_bps = 1000000000,
      .delay_ms = 24.862000,
    },
    {
      // edge_12
      .src = 3,
      .dst = 1,
      .capacity_bps = 1000000000,
      .delay_ms = 12.909000,
    },
    {
      // edge_13
      .src = 3,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 38.976000,
    },
    {
      // edge_14
      .src = 3,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 43.555000,
    },
    {
      // edge_15
      .src = 4,
      .dst = 0,
      .capacity_bps = 1000000000,
      .delay_ms = 26.206000,
    },
    {
      // edge_16
      .src = 4,
      .dst = 1,
      .capacity_bps = 1000000000,
      .delay_ms = 14.412000,
    },
    {
      // edge_17
      .src = 4,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 37.737000,
    },
    {
      // edge_18
      .src = 4,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 42.189000,
    },
    {
      // edge_19
      .src = 5,
      .dst = 1,
      .capacity_bps = 1000000000,
      .delay_ms = 51.170000,
    },
    {
      // edge_20
      .src = 5,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 38.976000,
    },
    {
      // edge_21
      .src = 5,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 37.737000,
    },
    {
      // edge_22
      .src = 5,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 6.320000,
    },
    {
      // edge_23
      .src = 5,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 10.193000,
    },
    {
      // edge_24
      .src = 5,
      .dst = 8,
      .capacity_bps = 1000000000,
      .delay_ms = 11.659000,
    },
    {
      // edge_25
      .src = 5,
      .dst = 9,
      .capacity_bps = 1000000000,
      .delay_ms = 16.298000,
    },
    {
      // edge_26
      .src = 6,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 43.555000,
    },
    {
      // edge_27
      .src = 6,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 42.189000,
    },
    {
      // edge_28
      .src = 6,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 6.320000,
    },
    {
      // edge_29
      .src = 6,
      .dst = 8,
      .capacity_bps = 1000000000,
      .delay_ms = 10.204000,
    },
    {
      // edge_30
      .src = 6,
      .dst = 9,
      .capacity_bps = 1000000000,
      .delay_ms = 15.057000,
    },
    {
      // edge_31
      .src = 6,
      .dst = 12,
      .capacity_bps = 1000000000,
      .delay_ms = 44.079000,
    },
    {
      // edge_32
      .src = 7,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 10.193000,
    },
    {
      // edge_33
      .src = 7,
      .dst = 8,
      .capacity_bps = 1000000000,
      .delay_ms = 2.666000,
    },
    {
      // edge_34
      .src = 7,
      .dst = 9,
      .capacity_bps = 1000000000,
      .delay_ms = 6.202000,
    },
    {
      // edge_35
      .src = 7,
      .dst = 10,
      .capacity_bps = 1000000000,
      .delay_ms = 6.103000,
    },
    {
      // edge_36
      .src = 7,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 7.951000,
    },
    {
      // edge_37
      .src = 7,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 34.565000,
    },
    {
      // edge_38
      .src = 8,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 11.659000,
    },
    {
      // edge_39
      .src = 8,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 10.204000,
    },
    {
      // edge_40
      .src = 8,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 2.666000,
    },
    {
      // edge_41
      .src = 8,
      .dst = 9,
      .capacity_bps = 1000000000,
      .delay_ms = 4.912000,
    },
    {
      // edge_42
      .src = 8,
      .dst = 12,
      .capacity_bps = 1000000000,
      .delay_ms = 39.890000,
    },
    {
      // edge_43
      .src = 9,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 16.298000,
    },
    {
      // edge_44
      .src = 9,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 15.057000,
    },
    {
      // edge_45
      .src = 9,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 6.202000,
    },
    {
      // edge_46
      .src = 9,
      .dst = 8,
      .capacity_bps = 1000000000,
      .delay_ms = 4.912000,
    },
    {
      // edge_47
      .src = 9,
      .dst = 10,
      .capacity_bps = 1000000000,
      .delay_ms = 2.082000,
    },
    {
      // edge_48
      .src = 9,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 1.967000,
    },
    {
      // edge_49
      .src = 10,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 6.103000,
    },
    {
      // edge_50
      .src = 10,
      .dst = 9,
      .capacity_bps = 1000000000,
      .delay_ms = 2.082000,
    },
    {
      // edge_51
      .src = 10,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 2.357000,
    },
    {
      // edge_52
      .src = 10,
      .dst = 15,
      .capacity_bps = 1000000000,
      .delay_ms = 32.116000,
    },
    {
      // edge_53
      .src = 11,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 7.951000,
    },
    {
      // edge_54
      .src = 11,
      .dst = 9,
      .capacity_bps = 1000000000,
      .delay_ms = 1.967000,
    },
    {
      // edge_55
      .src = 11,
      .dst = 10,
      .capacity_bps = 1000000000,
      .delay_ms = 2.357000,
    },
    {
      // edge_56
      .src = 11,
      .dst = 12,
      .capacity_bps = 1000000000,
      .delay_ms = 36.556000,
    },
    {
      // edge_57
      .src = 11,
      .dst = 14,
      .capacity_bps = 1000000000,
      .delay_ms = 32.740000,
    },
    {
      // edge_58
      .src = 11,
      .dst = 15,
      .capacity_bps = 1000000000,
      .delay_ms = 33.090000,
    },
    {
      // edge_59
      .src = 12,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 44.079000,
    },
    {
      // edge_60
      .src = 12,
      .dst = 8,
      .capacity_bps = 1000000000,
      .delay_ms = 39.890000,
    },
    {
      // edge_61
      .src = 12,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 36.556000,
    },
    {
      // edge_62
      .src = 13,
      .dst = 14,
      .capacity_bps = 1000000000,
      .delay_ms = 3.735000,
    },
    {
      // edge_63
      .src = 13,
      .dst = 15,
      .capacity_bps = 1000000000,
      .delay_ms = 3.780000,
    },
    {
      // edge_64
      .src = 13,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 4.275000,
    },
    {
      // edge_65
      .src = 14,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 32.740000,
    },
    {
      // edge_66
      .src = 14,
      .dst = 13,
      .capacity_bps = 1000000000,
      .delay_ms = 3.735000,
    },
    {
      // edge_67
      .src = 14,
      .dst = 15,
      .capacity_bps = 1000000000,
      .delay_ms = 1.384000,
    },
    {
      // edge_68
      .src = 15,
      .dst = 10,
      .capacity_bps = 1000000000,
      .delay_ms = 32.116000,
    },
    {
      // edge_69
      .src = 15,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 33.090000,
    },
    {
      // edge_70
      .src = 15,
      .dst = 13,
      .capacity_bps = 1000000000,
      .delay_ms = 3.780000,
    },
    {
      // edge_71
      .src = 15,
      .dst = 14,
      .capacity_bps = 1000000000,
      .delay_ms = 1.384000,
    },
    {
      // edge_72
      .src = 15,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 1.454000,
    },
    {
      // edge_73
      .src = 15,
      .dst = 17,
      .capacity_bps = 1000000000,
      .delay_ms = 8.349000,
    },
    {
      // edge_74
      .src = 16,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 34.565000,
    },
    {
      // edge_75
      .src = 16,
      .dst = 13,
      .capacity_bps = 1000000000,
      .delay_ms = 4.275000,
    },
    {
      // edge_76
      .src = 16,
      .dst = 15,
      .capacity_bps = 1000000000,
      .delay_ms = 1.454000,
    },
    {
      // edge_77
      .src = 16,
      .dst = 17,
      .capacity_bps = 1000000000,
      .delay_ms = 6.935000,
    },
    {
      // edge_78
      .src = 17,
      .dst = 15,
      .capacity_bps = 1000000000,
      .delay_ms = 8.349000,
    },
    {
      // edge_79
      .src = 17,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 6.935000,
    },
  };
  return TestTopology{nodes, links};
}

}  // namespace routing_algos
