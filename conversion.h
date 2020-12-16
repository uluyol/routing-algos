#ifndef ULUYOL_ROUTING_ALGOS_CONVERSION_H_
#define ULUYOL_ROUTING_ALGOS_CONVERSION_H_

#include "types.h"

namespace routing_algos {

PathSplit BpsToFrac(const PathSplit& path_bps);

PerFG<PathSplit> BpsToFracAll(const PerFG<PathSplit>& fg_path_bps);

}  // namespace routing_algos

#endif  // ULUYOL_ROUTING_ALGOS_CONVERSION_H_