#include "test_data.h"

namespace routing_algos {

TestTopology TracedAWSNetwork() {
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
      .name = "transit_25",
      .transit_only = true,
    },
    {
      .name = "transit_26",
      .transit_only = true,
    },
    {
      .name = "transit_27",
      .transit_only = true,
    },
    {
      .name = "transit_28",
      .transit_only = true,
    },
    {
      .name = "transit_29",
      .transit_only = true,
    },
    {
      .name = "transit_30",
      .transit_only = true,
    },
    {
      .name = "transit_31",
      .transit_only = true,
    },
    {
      .name = "transit_32",
      .transit_only = true,
    },
    {
      .name = "transit_33",
      .transit_only = true,
    },
    {
      .name = "transit_34",
      .transit_only = true,
    },
    {
      .name = "transit_35",
      .transit_only = true,
    },
    {
      .name = "transit_36",
      .transit_only = true,
    },
    {
      .name = "transit_37",
      .transit_only = true,
    },
    {
      .name = "transit_38",
      .transit_only = true,
    },
    {
      .name = "transit_39",
      .transit_only = true,
    },
    {
      .name = "transit_40",
      .transit_only = true,
    },
  };
  std::vector<Link> links{
    {
      // edge_0
      .src = 0,
      .dst = 1,
      .capacity_bps = 1000000000,
      .delay_ms = 2.138000,
    },
    {
      // edge_1
      .src = 0,
      .dst = 20,
      .capacity_bps = 1000000000,
      .delay_ms = 50.861000,
    },
    {
      // edge_2
      .src = 0,
      .dst = 25,
      .capacity_bps = 1000000000,
      .delay_ms = 2.128000,
    },
    {
      // edge_3
      .src = 0,
      .dst = 28,
      .capacity_bps = 1000000000,
      .delay_ms = 6.551000,
    },
    {
      // edge_4
      .src = 1,
      .dst = 0,
      .capacity_bps = 1000000000,
      .delay_ms = 2.138000,
    },
    {
      // edge_5
      .src = 1,
      .dst = 2,
      .capacity_bps = 1000000000,
      .delay_ms = 1.541000,
    },
    {
      // edge_6
      .src = 1,
      .dst = 2,
      .capacity_bps = 1000000000,
      .delay_ms = 1.541000,
    },
    {
      // edge_7
      .src = 1,
      .dst = 24,
      .capacity_bps = 1000000000,
      .delay_ms = 61.366000,
    },
    {
      // edge_8
      .src = 1,
      .dst = 25,
      .capacity_bps = 1000000000,
      .delay_ms = 4.227000,
    },
    {
      // edge_9
      .src = 2,
      .dst = 1,
      .capacity_bps = 1000000000,
      .delay_ms = 1.541000,
    },
    {
      // edge_10
      .src = 2,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 12.634000,
    },
    {
      // edge_11
      .src = 2,
      .dst = 28,
      .capacity_bps = 1000000000,
      .delay_ms = 4.820000,
    },
    {
      // edge_12
      .src = 3,
      .dst = 2,
      .capacity_bps = 1000000000,
      .delay_ms = 12.634000,
    },
    {
      // edge_13
      .src = 3,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 3.057000,
    },
    {
      // edge_14
      .src = 3,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 2.643000,
    },
    {
      // edge_15
      .src = 3,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 4.992000,
    },
    {
      // edge_16
      .src = 3,
      .dst = 29,
      .capacity_bps = 1000000000,
      .delay_ms = 6.606000,
    },
    {
      // edge_17
      .src = 3,
      .dst = 31,
      .capacity_bps = 1000000000,
      .delay_ms = 3.314000,
    },
    {
      // edge_18
      .src = 4,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 3.057000,
    },
    {
      // edge_19
      .src = 4,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 1.266000,
    },
    {
      // edge_20
      .src = 4,
      .dst = 8,
      .capacity_bps = 1000000000,
      .delay_ms = 26.133000,
    },
    {
      // edge_21
      .src = 4,
      .dst = 10,
      .capacity_bps = 1000000000,
      .delay_ms = 29.889000,
    },
    {
      // edge_22
      .src = 4,
      .dst = 32,
      .capacity_bps = 1000000000,
      .delay_ms = 3.272000,
    },
    {
      // edge_23
      .src = 4,
      .dst = 33,
      .capacity_bps = 1000000000,
      .delay_ms = 5.129000,
    },
    {
      // edge_24
      .src = 5,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 2.643000,
    },
    {
      // edge_25
      .src = 5,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 1.266000,
    },
    {
      // edge_26
      .src = 5,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 3.585000,
    },
    {
      // edge_27
      .src = 5,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 38.680000,
    },
    {
      // edge_28
      .src = 5,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 38.680000,
    },
    {
      // edge_29
      .src = 6,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 4.992000,
    },
    {
      // edge_30
      .src = 6,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 3.585000,
    },
    {
      // edge_31
      .src = 6,
      .dst = 8,
      .capacity_bps = 1000000000,
      .delay_ms = 21.694000,
    },
    {
      // edge_32
      .src = 6,
      .dst = 8,
      .capacity_bps = 1000000000,
      .delay_ms = 21.694000,
    },
    {
      // edge_33
      .src = 6,
      .dst = 23,
      .capacity_bps = 1000000000,
      .delay_ms = 48.755000,
    },
    {
      // edge_34
      .src = 7,
      .dst = 5,
      .capacity_bps = 1000000000,
      .delay_ms = 38.680000,
    },
    {
      // edge_35
      .src = 7,
      .dst = 33,
      .capacity_bps = 1000000000,
      .delay_ms = 34.262000,
    },
    {
      // edge_36
      .src = 7,
      .dst = 33,
      .capacity_bps = 1000000000,
      .delay_ms = 34.262000,
    },
    {
      // edge_37
      .src = 7,
      .dst = 34,
      .capacity_bps = 1000000000,
      .delay_ms = 3.578000,
    },
    {
      // edge_38
      .src = 7,
      .dst = 34,
      .capacity_bps = 1000000000,
      .delay_ms = 3.578000,
    },
    {
      // edge_39
      .src = 8,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 26.133000,
    },
    {
      // edge_40
      .src = 8,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 21.694000,
    },
    {
      // edge_41
      .src = 8,
      .dst = 9,
      .capacity_bps = 1000000000,
      .delay_ms = 2.102000,
    },
    {
      // edge_42
      .src = 9,
      .dst = 8,
      .capacity_bps = 1000000000,
      .delay_ms = 2.102000,
    },
    {
      // edge_43
      .src = 9,
      .dst = 10,
      .capacity_bps = 1000000000,
      .delay_ms = 1.809000,
    },
    {
      // edge_44
      .src = 9,
      .dst = 14,
      .capacity_bps = 1000000000,
      .delay_ms = 47.268000,
    },
    {
      // edge_45
      .src = 10,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 29.889000,
    },
    {
      // edge_46
      .src = 10,
      .dst = 9,
      .capacity_bps = 1000000000,
      .delay_ms = 1.809000,
    },
    {
      // edge_47
      .src = 10,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 2.864000,
    },
    {
      // edge_48
      .src = 10,
      .dst = 12,
      .capacity_bps = 1000000000,
      .delay_ms = 2.501000,
    },
    {
      // edge_49
      .src = 10,
      .dst = 35,
      .capacity_bps = 1000000000,
      .delay_ms = 4.324000,
    },
    {
      // edge_50
      .src = 11,
      .dst = 10,
      .capacity_bps = 1000000000,
      .delay_ms = 2.864000,
    },
    {
      // edge_51
      .src = 11,
      .dst = 12,
      .capacity_bps = 1000000000,
      .delay_ms = 2.507000,
    },
    {
      // edge_52
      .src = 11,
      .dst = 14,
      .capacity_bps = 1000000000,
      .delay_ms = 43.753000,
    },
    {
      // edge_53
      .src = 11,
      .dst = 14,
      .capacity_bps = 1000000000,
      .delay_ms = 43.753000,
    },
    {
      // edge_54
      .src = 11,
      .dst = 15,
      .capacity_bps = 1000000000,
      .delay_ms = 19.635000,
    },
    {
      // edge_55
      .src = 11,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 30.262000,
    },
    {
      // edge_56
      .src = 11,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 30.262000,
    },
    {
      // edge_57
      .src = 11,
      .dst = 35,
      .capacity_bps = 1000000000,
      .delay_ms = 5.442000,
    },
    {
      // edge_58
      .src = 11,
      .dst = 36,
      .capacity_bps = 1000000000,
      .delay_ms = 3.406000,
    },
    {
      // edge_59
      .src = 11,
      .dst = 36,
      .capacity_bps = 1000000000,
      .delay_ms = 3.406000,
    },
    {
      // edge_60
      .src = 12,
      .dst = 10,
      .capacity_bps = 1000000000,
      .delay_ms = 2.501000,
    },
    {
      // edge_61
      .src = 12,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 2.507000,
    },
    {
      // edge_62
      .src = 12,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 30.322000,
    },
    {
      // edge_63
      .src = 14,
      .dst = 9,
      .capacity_bps = 1000000000,
      .delay_ms = 47.268000,
    },
    {
      // edge_64
      .src = 14,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 43.753000,
    },
    {
      // edge_65
      .src = 15,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 19.635000,
    },
    {
      // edge_66
      .src = 15,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 11.302000,
    },
    {
      // edge_67
      .src = 16,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 30.262000,
    },
    {
      // edge_68
      .src = 16,
      .dst = 12,
      .capacity_bps = 1000000000,
      .delay_ms = 30.322000,
    },
    {
      // edge_69
      .src = 16,
      .dst = 15,
      .capacity_bps = 1000000000,
      .delay_ms = 11.302000,
    },
    {
      // edge_70
      .src = 16,
      .dst = 17,
      .capacity_bps = 1000000000,
      .delay_ms = 18.752000,
    },
    {
      // edge_71
      .src = 16,
      .dst = 37,
      .capacity_bps = 1000000000,
      .delay_ms = 4.627000,
    },
    {
      // edge_72
      .src = 16,
      .dst = 38,
      .capacity_bps = 1000000000,
      .delay_ms = 4.702000,
    },
    {
      // edge_73
      .src = 16,
      .dst = 38,
      .capacity_bps = 1000000000,
      .delay_ms = 4.702000,
    },
    {
      // edge_74
      .src = 17,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 18.752000,
    },
    {
      // edge_75
      .src = 17,
      .dst = 19,
      .capacity_bps = 1000000000,
      .delay_ms = 12.775000,
    },
    {
      // edge_76
      .src = 17,
      .dst = 19,
      .capacity_bps = 1000000000,
      .delay_ms = 12.775000,
    },
    {
      // edge_77
      .src = 17,
      .dst = 23,
      .capacity_bps = 1000000000,
      .delay_ms = 26.001000,
    },
    {
      // edge_78
      .src = 17,
      .dst = 23,
      .capacity_bps = 1000000000,
      .delay_ms = 26.001000,
    },
    {
      // edge_79
      .src = 17,
      .dst = 38,
      .capacity_bps = 1000000000,
      .delay_ms = 14.077000,
    },
    {
      // edge_80
      .src = 19,
      .dst = 17,
      .capacity_bps = 1000000000,
      .delay_ms = 12.775000,
    },
    {
      // edge_81
      .src = 19,
      .dst = 20,
      .capacity_bps = 1000000000,
      .delay_ms = 1.060000,
    },
    {
      // edge_82
      .src = 20,
      .dst = 0,
      .capacity_bps = 1000000000,
      .delay_ms = 50.861000,
    },
    {
      // edge_83
      .src = 20,
      .dst = 19,
      .capacity_bps = 1000000000,
      .delay_ms = 1.060000,
    },
    {
      // edge_84
      .src = 20,
      .dst = 22,
      .capacity_bps = 1000000000,
      .delay_ms = 9.841000,
    },
    {
      // edge_85
      .src = 20,
      .dst = 23,
      .capacity_bps = 1000000000,
      .delay_ms = 13.048000,
    },
    {
      // edge_86
      .src = 20,
      .dst = 23,
      .capacity_bps = 1000000000,
      .delay_ms = 13.048000,
    },
    {
      // edge_87
      .src = 22,
      .dst = 20,
      .capacity_bps = 1000000000,
      .delay_ms = 9.841000,
    },
    {
      // edge_88
      .src = 22,
      .dst = 23,
      .capacity_bps = 1000000000,
      .delay_ms = 5.019000,
    },
    {
      // edge_89
      .src = 23,
      .dst = 6,
      .capacity_bps = 1000000000,
      .delay_ms = 48.755000,
    },
    {
      // edge_90
      .src = 23,
      .dst = 17,
      .capacity_bps = 1000000000,
      .delay_ms = 26.001000,
    },
    {
      // edge_91
      .src = 23,
      .dst = 20,
      .capacity_bps = 1000000000,
      .delay_ms = 13.048000,
    },
    {
      // edge_92
      .src = 23,
      .dst = 22,
      .capacity_bps = 1000000000,
      .delay_ms = 5.019000,
    },
    {
      // edge_93
      .src = 23,
      .dst = 24,
      .capacity_bps = 1000000000,
      .delay_ms = 39.793000,
    },
    {
      // edge_94
      .src = 23,
      .dst = 24,
      .capacity_bps = 1000000000,
      .delay_ms = 39.793000,
    },
    {
      // edge_95
      .src = 23,
      .dst = 25,
      .capacity_bps = 1000000000,
      .delay_ms = 39.714000,
    },
    {
      // edge_96
      .src = 24,
      .dst = 1,
      .capacity_bps = 1000000000,
      .delay_ms = 61.366000,
    },
    {
      // edge_97
      .src = 24,
      .dst = 23,
      .capacity_bps = 1000000000,
      .delay_ms = 39.793000,
    },
    {
      // edge_98
      .src = 24,
      .dst = 25,
      .capacity_bps = 1000000000,
      .delay_ms = 59.092000,
    },
    {
      // edge_99
      .src = 24,
      .dst = 39,
      .capacity_bps = 1000000000,
      .delay_ms = 16.390000,
    },
    {
      // edge_100
      .src = 24,
      .dst = 40,
      .capacity_bps = 1000000000,
      .delay_ms = 3.801000,
    },
    {
      // edge_101
      .src = 24,
      .dst = 40,
      .capacity_bps = 1000000000,
      .delay_ms = 3.801000,
    },
    {
      // edge_102
      .src = 25,
      .dst = 0,
      .capacity_bps = 1000000000,
      .delay_ms = 2.128000,
    },
    {
      // edge_103
      .src = 25,
      .dst = 1,
      .capacity_bps = 1000000000,
      .delay_ms = 4.227000,
    },
    {
      // edge_104
      .src = 25,
      .dst = 23,
      .capacity_bps = 1000000000,
      .delay_ms = 39.714000,
    },
    {
      // edge_105
      .src = 25,
      .dst = 24,
      .capacity_bps = 1000000000,
      .delay_ms = 59.092000,
    },
    {
      // edge_106
      .src = 25,
      .dst = 26,
      .capacity_bps = 1000000000,
      .delay_ms = 1.393000,
    },
    {
      // edge_107
      .src = 26,
      .dst = 25,
      .capacity_bps = 1000000000,
      .delay_ms = 1.393000,
    },
    {
      // edge_108
      .src = 26,
      .dst = 27,
      .capacity_bps = 1000000000,
      .delay_ms = 2.717000,
    },
    {
      // edge_109
      .src = 27,
      .dst = 26,
      .capacity_bps = 1000000000,
      .delay_ms = 2.717000,
    },
    {
      // edge_110
      .src = 27,
      .dst = 29,
      .capacity_bps = 1000000000,
      .delay_ms = 6.277000,
    },
    {
      // edge_111
      .src = 28,
      .dst = 0,
      .capacity_bps = 1000000000,
      .delay_ms = 6.551000,
    },
    {
      // edge_112
      .src = 28,
      .dst = 2,
      .capacity_bps = 1000000000,
      .delay_ms = 4.820000,
    },
    {
      // edge_113
      .src = 28,
      .dst = 29,
      .capacity_bps = 1000000000,
      .delay_ms = 4.333000,
    },
    {
      // edge_114
      .src = 28,
      .dst = 31,
      .capacity_bps = 1000000000,
      .delay_ms = 5.557000,
    },
    {
      // edge_115
      .src = 29,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 6.606000,
    },
    {
      // edge_116
      .src = 29,
      .dst = 27,
      .capacity_bps = 1000000000,
      .delay_ms = 6.277000,
    },
    {
      // edge_117
      .src = 29,
      .dst = 28,
      .capacity_bps = 1000000000,
      .delay_ms = 4.333000,
    },
    {
      // edge_118
      .src = 29,
      .dst = 30,
      .capacity_bps = 1000000000,
      .delay_ms = 2.182000,
    },
    {
      // edge_119
      .src = 29,
      .dst = 32,
      .capacity_bps = 1000000000,
      .delay_ms = 5.193000,
    },
    {
      // edge_120
      .src = 30,
      .dst = 29,
      .capacity_bps = 1000000000,
      .delay_ms = 2.182000,
    },
    {
      // edge_121
      .src = 30,
      .dst = 32,
      .capacity_bps = 1000000000,
      .delay_ms = 5.175000,
    },
    {
      // edge_122
      .src = 31,
      .dst = 3,
      .capacity_bps = 1000000000,
      .delay_ms = 3.314000,
    },
    {
      // edge_123
      .src = 31,
      .dst = 28,
      .capacity_bps = 1000000000,
      .delay_ms = 5.557000,
    },
    {
      // edge_124
      .src = 32,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 3.272000,
    },
    {
      // edge_125
      .src = 32,
      .dst = 29,
      .capacity_bps = 1000000000,
      .delay_ms = 5.193000,
    },
    {
      // edge_126
      .src = 32,
      .dst = 30,
      .capacity_bps = 1000000000,
      .delay_ms = 5.175000,
    },
    {
      // edge_127
      .src = 32,
      .dst = 33,
      .capacity_bps = 1000000000,
      .delay_ms = 4.238000,
    },
    {
      // edge_128
      .src = 33,
      .dst = 4,
      .capacity_bps = 1000000000,
      .delay_ms = 5.129000,
    },
    {
      // edge_129
      .src = 33,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 34.262000,
    },
    {
      // edge_130
      .src = 33,
      .dst = 32,
      .capacity_bps = 1000000000,
      .delay_ms = 4.238000,
    },
    {
      // edge_131
      .src = 34,
      .dst = 7,
      .capacity_bps = 1000000000,
      .delay_ms = 3.578000,
    },
    {
      // edge_132
      .src = 35,
      .dst = 10,
      .capacity_bps = 1000000000,
      .delay_ms = 4.324000,
    },
    {
      // edge_133
      .src = 35,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 5.442000,
    },
    {
      // edge_134
      .src = 36,
      .dst = 11,
      .capacity_bps = 1000000000,
      .delay_ms = 3.406000,
    },
    {
      // edge_135
      .src = 37,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 4.627000,
    },
    {
      // edge_136
      .src = 37,
      .dst = 38,
      .capacity_bps = 1000000000,
      .delay_ms = 7.044000,
    },
    {
      // edge_137
      .src = 38,
      .dst = 16,
      .capacity_bps = 1000000000,
      .delay_ms = 4.702000,
    },
    {
      // edge_138
      .src = 38,
      .dst = 17,
      .capacity_bps = 1000000000,
      .delay_ms = 14.077000,
    },
    {
      // edge_139
      .src = 38,
      .dst = 37,
      .capacity_bps = 1000000000,
      .delay_ms = 7.044000,
    },
    {
      // edge_140
      .src = 39,
      .dst = 24,
      .capacity_bps = 1000000000,
      .delay_ms = 16.390000,
    },
    {
      // edge_141
      .src = 39,
      .dst = 40,
      .capacity_bps = 1000000000,
      .delay_ms = 13.878000,
    },
    {
      // edge_142
      .src = 40,
      .dst = 24,
      .capacity_bps = 1000000000,
      .delay_ms = 3.801000,
    },
    {
      // edge_143
      .src = 40,
      .dst = 39,
      .capacity_bps = 1000000000,
      .delay_ms = 13.878000,
    },
  };
  return TestTopology{nodes, links};
}

}  // namespace routing_algos
