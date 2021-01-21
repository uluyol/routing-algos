#include "routing-algos/test-data.h"

namespace routing_algos {

namespace {

TestTopology RawTracedAkamaiNetwork() {
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
      {
          .name = "node_21",
          .transit_only = false,
      },
      {
          .name = "node_22",
          .transit_only = false,
      },
      {
          .name = "node_23",
          .transit_only = false,
      },
      {
          .name = "node_24",
          .transit_only = false,
      },
      {
          .name = "node_25",
          .transit_only = false,
      },
      {
          .name = "node_26",
          .transit_only = false,
      },
      {
          .name = "node_27",
          .transit_only = false,
      },
  };
  std::vector<Link> links{
      {
          // edge_0
          .src = 0,
          .dst = 1,
          .capacity_bps = 1000000000,
          .delay_ms = 2.846000,
      },
      {
          // edge_1
          .src = 0,
          .dst = 4,
          .capacity_bps = 1000000000,
          .delay_ms = 13.125000,
      },
      {
          // edge_2
          .src = 1,
          .dst = 0,
          .capacity_bps = 1000000000,
          .delay_ms = 2.846000,
      },
      {
          // edge_3
          .src = 1,
          .dst = 2,
          .capacity_bps = 1000000000,
          .delay_ms = 2.132000,
      },
      {
          // edge_4
          .src = 1,
          .dst = 3,
          .capacity_bps = 1000000000,
          .delay_ms = 10.217000,
      },
      {
          // edge_5
          .src = 1,
          .dst = 4,
          .capacity_bps = 1000000000,
          .delay_ms = 12.973000,
      },
      {
          // edge_6
          .src = 1,
          .dst = 7,
          .capacity_bps = 1000000000,
          .delay_ms = 17.438000,
      },
      {
          // edge_7
          .src = 1,
          .dst = 25,
          .capacity_bps = 1000000000,
          .delay_ms = 39.097000,
      },
      {
          // edge_8
          .src = 2,
          .dst = 1,
          .capacity_bps = 1000000000,
          .delay_ms = 2.132000,
      },
      {
          // edge_9
          .src = 2,
          .dst = 3,
          .capacity_bps = 1000000000,
          .delay_ms = 9.236000,
      },
      {
          // edge_10
          .src = 2,
          .dst = 4,
          .capacity_bps = 1000000000,
          .delay_ms = 12.782000,
      },
      {
          // edge_11
          .src = 2,
          .dst = 23,
          .capacity_bps = 1000000000,
          .delay_ms = 54.242000,
      },
      {
          // edge_12
          .src = 3,
          .dst = 1,
          .capacity_bps = 1000000000,
          .delay_ms = 10.217000,
      },
      {
          // edge_13
          .src = 3,
          .dst = 2,
          .capacity_bps = 1000000000,
          .delay_ms = 9.236000,
      },
      {
          // edge_14
          .src = 3,
          .dst = 4,
          .capacity_bps = 1000000000,
          .delay_ms = 5.282000,
      },
      {
          // edge_15
          .src = 3,
          .dst = 5,
          .capacity_bps = 1000000000,
          .delay_ms = 6.546000,
      },
      {
          // edge_16
          .src = 3,
          .dst = 6,
          .capacity_bps = 1000000000,
          .delay_ms = 9.031000,
      },
      {
          // edge_17
          .src = 3,
          .dst = 7,
          .capacity_bps = 1000000000,
          .delay_ms = 8.736000,
      },
      {
          // edge_18
          .src = 4,
          .dst = 0,
          .capacity_bps = 1000000000,
          .delay_ms = 13.125000,
      },
      {
          // edge_19
          .src = 4,
          .dst = 1,
          .capacity_bps = 1000000000,
          .delay_ms = 12.973000,
      },
      {
          // edge_20
          .src = 4,
          .dst = 2,
          .capacity_bps = 1000000000,
          .delay_ms = 12.782000,
      },
      {
          // edge_21
          .src = 4,
          .dst = 3,
          .capacity_bps = 1000000000,
          .delay_ms = 5.282000,
      },
      {
          // edge_22
          .src = 4,
          .dst = 5,
          .capacity_bps = 1000000000,
          .delay_ms = 4.629000,
      },
      {
          // edge_23
          .src = 4,
          .dst = 7,
          .capacity_bps = 1000000000,
          .delay_ms = 4.475000,
      },
      {
          // edge_24
          .src = 4,
          .dst = 8,
          .capacity_bps = 1000000000,
          .delay_ms = 5.633000,
      },
      {
          // edge_25
          .src = 5,
          .dst = 3,
          .capacity_bps = 1000000000,
          .delay_ms = 6.546000,
      },
      {
          // edge_26
          .src = 5,
          .dst = 4,
          .capacity_bps = 1000000000,
          .delay_ms = 4.629000,
      },
      {
          // edge_27
          .src = 5,
          .dst = 6,
          .capacity_bps = 1000000000,
          .delay_ms = 4.234000,
      },
      {
          // edge_28
          .src = 5,
          .dst = 7,
          .capacity_bps = 1000000000,
          .delay_ms = 3.438000,
      },
      {
          // edge_29
          .src = 6,
          .dst = 3,
          .capacity_bps = 1000000000,
          .delay_ms = 9.031000,
      },
      {
          // edge_30
          .src = 6,
          .dst = 5,
          .capacity_bps = 1000000000,
          .delay_ms = 4.234000,
      },
      {
          // edge_31
          .src = 6,
          .dst = 7,
          .capacity_bps = 1000000000,
          .delay_ms = 6.924000,
      },
      {
          // edge_32
          .src = 6,
          .dst = 9,
          .capacity_bps = 1000000000,
          .delay_ms = 31.772000,
      },
      {
          // edge_33
          .src = 7,
          .dst = 1,
          .capacity_bps = 1000000000,
          .delay_ms = 17.438000,
      },
      {
          // edge_34
          .src = 7,
          .dst = 3,
          .capacity_bps = 1000000000,
          .delay_ms = 8.736000,
      },
      {
          // edge_35
          .src = 7,
          .dst = 4,
          .capacity_bps = 1000000000,
          .delay_ms = 4.475000,
      },
      {
          // edge_36
          .src = 7,
          .dst = 5,
          .capacity_bps = 1000000000,
          .delay_ms = 3.438000,
      },
      {
          // edge_37
          .src = 7,
          .dst = 6,
          .capacity_bps = 1000000000,
          .delay_ms = 6.924000,
      },
      {
          // edge_38
          .src = 7,
          .dst = 8,
          .capacity_bps = 1000000000,
          .delay_ms = 1.664000,
      },
      {
          // edge_39
          .src = 7,
          .dst = 12,
          .capacity_bps = 1000000000,
          .delay_ms = 31.353000,
      },
      {
          // edge_40
          .src = 8,
          .dst = 4,
          .capacity_bps = 1000000000,
          .delay_ms = 5.633000,
      },
      {
          // edge_41
          .src = 8,
          .dst = 7,
          .capacity_bps = 1000000000,
          .delay_ms = 1.664000,
      },
      {
          // edge_42
          .src = 8,
          .dst = 11,
          .capacity_bps = 1000000000,
          .delay_ms = 27.367000,
      },
      {
          // edge_43
          .src = 9,
          .dst = 6,
          .capacity_bps = 1000000000,
          .delay_ms = 31.772000,
      },
      {
          // edge_44
          .src = 10,
          .dst = 12,
          .capacity_bps = 1000000000,
          .delay_ms = 5.121000,
      },
      {
          // edge_45
          .src = 10,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 6.970000,
      },
      {
          // edge_46
          .src = 11,
          .dst = 8,
          .capacity_bps = 1000000000,
          .delay_ms = 27.367000,
      },
      {
          // edge_47
          .src = 11,
          .dst = 12,
          .capacity_bps = 1000000000,
          .delay_ms = 2.439000,
      },
      {
          // edge_48
          .src = 11,
          .dst = 13,
          .capacity_bps = 1000000000,
          .delay_ms = 2.232000,
      },
      {
          // edge_49
          .src = 11,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 4.198000,
      },
      {
          // edge_50
          .src = 12,
          .dst = 7,
          .capacity_bps = 1000000000,
          .delay_ms = 31.353000,
      },
      {
          // edge_51
          .src = 12,
          .dst = 10,
          .capacity_bps = 1000000000,
          .delay_ms = 5.121000,
      },
      {
          // edge_52
          .src = 12,
          .dst = 11,
          .capacity_bps = 1000000000,
          .delay_ms = 2.439000,
      },
      {
          // edge_53
          .src = 12,
          .dst = 13,
          .capacity_bps = 1000000000,
          .delay_ms = 1.805000,
      },
      {
          // edge_54
          .src = 12,
          .dst = 15,
          .capacity_bps = 1000000000,
          .delay_ms = 2.807000,
      },
      {
          // edge_55
          .src = 12,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 2.195000,
      },
      {
          // edge_56
          .src = 13,
          .dst = 11,
          .capacity_bps = 1000000000,
          .delay_ms = 2.232000,
      },
      {
          // edge_57
          .src = 13,
          .dst = 12,
          .capacity_bps = 1000000000,
          .delay_ms = 1.805000,
      },
      {
          // edge_58
          .src = 13,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 2.272000,
      },
      {
          // edge_59
          .src = 13,
          .dst = 21,
          .capacity_bps = 1000000000,
          .delay_ms = 4.635000,
      },
      {
          // edge_60
          .src = 14,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 1.034000,
      },
      {
          // edge_61
          .src = 14,
          .dst = 17,
          .capacity_bps = 1000000000,
          .delay_ms = 1.093000,
      },
      {
          // edge_62
          .src = 15,
          .dst = 12,
          .capacity_bps = 1000000000,
          .delay_ms = 2.807000,
      },
      {
          // edge_63
          .src = 15,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 1.960000,
      },
      {
          // edge_64
          .src = 16,
          .dst = 10,
          .capacity_bps = 1000000000,
          .delay_ms = 6.970000,
      },
      {
          // edge_65
          .src = 16,
          .dst = 11,
          .capacity_bps = 1000000000,
          .delay_ms = 4.198000,
      },
      {
          // edge_66
          .src = 16,
          .dst = 12,
          .capacity_bps = 1000000000,
          .delay_ms = 2.195000,
      },
      {
          // edge_67
          .src = 16,
          .dst = 13,
          .capacity_bps = 1000000000,
          .delay_ms = 2.272000,
      },
      {
          // edge_68
          .src = 16,
          .dst = 14,
          .capacity_bps = 1000000000,
          .delay_ms = 1.034000,
      },
      {
          // edge_69
          .src = 16,
          .dst = 15,
          .capacity_bps = 1000000000,
          .delay_ms = 1.960000,
      },
      {
          // edge_70
          .src = 16,
          .dst = 17,
          .capacity_bps = 1000000000,
          .delay_ms = 1.588000,
      },
      {
          // edge_71
          .src = 16,
          .dst = 18,
          .capacity_bps = 1000000000,
          .delay_ms = 1.079000,
      },
      {
          // edge_72
          .src = 16,
          .dst = 19,
          .capacity_bps = 1000000000,
          .delay_ms = 1.279000,
      },
      {
          // edge_73
          .src = 16,
          .dst = 20,
          .capacity_bps = 1000000000,
          .delay_ms = 2.319000,
      },
      {
          // edge_74
          .src = 16,
          .dst = 21,
          .capacity_bps = 1000000000,
          .delay_ms = 5.014000,
      },
      {
          // edge_75
          .src = 17,
          .dst = 14,
          .capacity_bps = 1000000000,
          .delay_ms = 1.093000,
      },
      {
          // edge_76
          .src = 17,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 1.588000,
      },
      {
          // edge_77
          .src = 17,
          .dst = 19,
          .capacity_bps = 1000000000,
          .delay_ms = 1.211000,
      },
      {
          // edge_78
          .src = 18,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 1.079000,
      },
      {
          // edge_79
          .src = 18,
          .dst = 20,
          .capacity_bps = 1000000000,
          .delay_ms = 1.252000,
      },
      {
          // edge_80
          .src = 19,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 1.279000,
      },
      {
          // edge_81
          .src = 19,
          .dst = 17,
          .capacity_bps = 1000000000,
          .delay_ms = 1.211000,
      },
      {
          // edge_82
          .src = 20,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 2.319000,
      },
      {
          // edge_83
          .src = 20,
          .dst = 18,
          .capacity_bps = 1000000000,
          .delay_ms = 1.252000,
      },
      {
          // edge_84
          .src = 21,
          .dst = 13,
          .capacity_bps = 1000000000,
          .delay_ms = 4.635000,
      },
      {
          // edge_85
          .src = 21,
          .dst = 16,
          .capacity_bps = 1000000000,
          .delay_ms = 5.014000,
      },
      {
          // edge_86
          .src = 22,
          .dst = 23,
          .capacity_bps = 1000000000,
          .delay_ms = 13.085000,
      },
      {
          // edge_87
          .src = 22,
          .dst = 25,
          .capacity_bps = 1000000000,
          .delay_ms = 25.959000,
      },
      {
          // edge_88
          .src = 23,
          .dst = 2,
          .capacity_bps = 1000000000,
          .delay_ms = 54.242000,
      },
      {
          // edge_89
          .src = 23,
          .dst = 22,
          .capacity_bps = 1000000000,
          .delay_ms = 13.085000,
      },
      {
          // edge_90
          .src = 23,
          .dst = 25,
          .capacity_bps = 1000000000,
          .delay_ms = 13.536000,
      },
      {
          // edge_91
          .src = 24,
          .dst = 25,
          .capacity_bps = 1000000000,
          .delay_ms = 2.156000,
      },
      {
          // edge_92
          .src = 25,
          .dst = 1,
          .capacity_bps = 1000000000,
          .delay_ms = 39.097000,
      },
      {
          // edge_93
          .src = 25,
          .dst = 22,
          .capacity_bps = 1000000000,
          .delay_ms = 25.959000,
      },
      {
          // edge_94
          .src = 25,
          .dst = 23,
          .capacity_bps = 1000000000,
          .delay_ms = 13.536000,
      },
      {
          // edge_95
          .src = 25,
          .dst = 24,
          .capacity_bps = 1000000000,
          .delay_ms = 2.156000,
      },
      {
          // edge_96
          .src = 26,
          .dst = 27,
          .capacity_bps = 1000000000,
          .delay_ms = 4.159000,
      },
      {
          // edge_97
          .src = 27,
          .dst = 26,
          .capacity_bps = 1000000000,
          .delay_ms = 4.159000,
      },
  };
  return TestTopology{nodes, links};
}

}  // namespace

TestTopology TracedAkamaiNetwork() {
  return DedupLinks(RawTracedAkamaiNetwork());
}

}  // namespace routing_algos
