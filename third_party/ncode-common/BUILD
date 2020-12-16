load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "ncode-common",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
        "@glog",
    ],
)
