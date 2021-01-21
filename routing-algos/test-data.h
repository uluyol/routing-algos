#ifndef ROUTING_ALGOS_TEST_DATA_H_
#define ROUTING_ALGOS_TEST_DATA_H_

#include "routing-algos/types.h"

namespace routing_algos {

struct TestTopology {
  std::vector<Node> nodes;
  std::vector<Link> links;
};

// LinearNetwork returns a linear three-node topology.
//
// A → B → C
//
// All links have 100 bps capacity and 1 ms delay.
TestTopology LinearNetwork();

// TriangleNetwork returns a simple three-node topology.
//
// A  →  C
//  ↘   ↗
//    B
//
// All links have 100 bps capacity and 1 ms delay.
TestTopology TriangleNetwork();

// FourNodeNetwork returns a four-node topology used in the SIGCOMM'13 B4 paper.
//
//     D
//  ↗↙   ↘↖
// A   ↔   C
//  ↘↖   ↗↙
//     B
//
// All links have 10 Gbps capacity and 1 ms delay EXCEPT Links A↔D which have 10
// ms delay.
TestTopology FourNodeNetwork();

// ------
// Folowing network topologies are all "traced" from images and taken from
// https://github.com/uluyol/tracegeog
// ------

TestTopology TracedAkamaiNetwork();
TestTopology TracedAWSNetwork();
TestTopology TracedCloudflareNetwork();
TestTopology TracedB4Network();

// DedupLinks combines links between the same src and dst nodes.
// The capacities are summed together in the combined link and the delay is the
// max of the original links.
TestTopology DedupLinks(TestTopology input);

}  // namespace routing_algos

#endif  // ROUTING_ALGOS_TEST_DATA_H_