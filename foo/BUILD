load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:config.bzl", "aos_config")

flatbuffer_cc_library(
    name = "ping_fbs",
    srcs = ["ping.fbs"],
    gen_reflections = 1,
)

flatbuffer_cc_library(
    name = "pong_fbs",
    srcs = ["pong.fbs"],
    gen_reflections = 1,
)

aos_config(
    name = "pingpong_config",
    src = "pingpong.json",
    flatbuffers = [
        ":ping_fbs",
        ":pong_fbs",
    ],
    deps = ["//aos/events:aos_config"],
)

cc_library(
    name = "ping_lib",
    srcs = ["ping_lib.cc"],
    hdrs = ["ping_lib.h"],
    deps = [
        ":ping_fbs",
        ":pong_fbs",
        "//aos:json_to_flatbuffer",
        "//aos/events:event_loop",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log","@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "pong_lib",
    srcs = ["pong_lib.cc"],
    hdrs = ["pong_lib.h"],
    deps = [
        ":ping_fbs",
        ":pong_fbs",
        "//aos/events:event_loop",
    ],
)

cc_binary(
    name = "pong",
    srcs = ["pong.cc"],
    data = [":pingpong_config"],
    deps = [
        ":pong_lib",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "ping",
    srcs = ["ping.cc"],
    data = [":pingpong_config"],
    deps = [
        ":ping_lib",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_test(
    name = "ping_lib_test",
    srcs = ["ping_lib_test.cc"],
    data = [":pingpong_config.json"],
    deps = [
        ":ping_lib",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)