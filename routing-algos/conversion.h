#ifndef ROUTING_ALGOS_CONVERSION_H_
#define ROUTING_ALGOS_CONVERSION_H_

#include "routing-algos/types.h"

namespace routing_algos {

PathSplit BpsToFrac(const PathSplit& path_bps);

PerFG<PathSplit> BpsToFracAll(const PerFG<PathSplit>& fg_path_bps);

}  // namespace routing_algos

#endif  // ROUTING_ALGOS_CONVERSION_H_