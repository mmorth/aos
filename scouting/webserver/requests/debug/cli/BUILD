load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_library")
load("@rules_python//python:defs.bzl", "py_test")

go_library(
    name = "cli_lib",
    srcs = ["main.go"],
    data = [
        "//scouting/webserver/requests/messages:fbs_files",
        "@com_github_google_flatbuffers//:flatc",
    ],
    importpath = "github.com/RealtimeRoboticsGroup/aos/scouting/webserver/requests/debug/cli",
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:private"],
    deps = [
        "//scouting/webserver/requests/debug",
        "@com_github_davecgh_go_spew//spew",
    ],
)

go_binary(
    name = "cli",
    embed = [":cli_lib"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    visibility = ["//visibility:public"],
)

py_test(
    name = "cli_test",
    size = "small",
    srcs = [
        "cli_test.py",
    ],
    args = [
        "-v",
    ],
    data = [
        ":cli",
    ],
    deps = [
        "//scouting/testing:scouting_test_servers",
    ],
)
