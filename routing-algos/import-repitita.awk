#!/usr/bin/env awk -f

BEGIN {
    printf "#include \"routing-algos/test-data.h\"\n\nnamespace routing_algos {\n\nnamespace {\n\nTestTopology Raw%s() {\n", func_name;
}

/^NODES/ {
    print "  std::vector<Node> nodes{";
}

/^node_[0-9]+/ {
    printf "      {\n          .name = \"%s\",\n          .transit_only = false,\n      },\n", $1;
}

/^transit_[0-9]+/ {
    printf "      {\n          .name = \"%s\",\n          .transit_only = true,\n      },\n", $1;
}

/^EDGES/ {
    print "  };\n  std::vector<Link> links{";
}

/^edge/ {
    printf "      {\n          // %s\n          .src = %d,\n          .dst = %d,\n          .capacity_bps = %d,\n          .delay_ms = %f,\n      },\n", $1, $2, $3, $5*1000, $6/1000;
}

END {
    printf "  };\n  return TestTopology{nodes, links};\n}\n\n}  // namespace\n\nTestTopology %s() { return DedupLinks(Raw%s()); }\n\n}  // namespace routing_algos\n", func_name, func_name;
}
