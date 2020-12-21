load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "max_min_fairness",
    srcs = ["max_min_fairness.cc"],
    hdrs = ["max_min_fairness.h"],
    deps = [
        ":nth_element",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "max_min_fairness_test",
    srcs = ["max_min_fairness_test.cc"],
    deps = [
        ":max_min_fairness",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/strings",
        "@gtest//:gtest_main",
    ],
)

cc_binary(
    name = "max_min_fairness_bench",
    srcs = ["max_min_fairness_bench.cc"],
    deps = [
        ":max_min_fairness",
        "@com_google_absl//absl/functional:function_ref",
        "@com_google_absl//absl/random",
        "@com_google_benchmark//:benchmark_main",
    ],
)

cc_library(
    name = "nth_element",
    hdrs = ["nth_element.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//third_party/fastalg:hybrid_qsort",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "nth_element_test",
    srcs = ["nth_element_test.cc"],
    deps = [
        ":nth_element",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@gtest//:gtest_main",
    ],
)
