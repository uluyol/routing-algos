#!/usr/bin/env awk -f

BEGIN {
    printf "#include \"test_data.h\"\n\nnamespace routing_algos {\n\nTestTopology %s() {\n", func_name;
}

/^NODES/ {
    print "  std::vector<Node> nodes{";
}

/^node_[0-9]+/ {
    printf "    {\n      .name = \"%s\",\n      .transit_only = false,\n    },\n", $1;
}

/^transit_[0-9]+/ {
    printf "    {\n      .name = \"%s\",\n      .transit_only = true,\n    },\n", $1;
}

/^EDGES/ {
    print "  };\n  std::vector<Link> links{";
}

/^edge/ {
    printf "    {\n      // %s\n      .src = %d,\n      .dst = %d,\n      .capacity_bps = %d,\n      .delay_ms = %f,\n    },\n", $1, $2, $3, $5*1000, $6/1000;
}

END {
    print "  };\n  return TestTopology{nodes, links};\n}\n\n}  // namespace routing_algos";
}
