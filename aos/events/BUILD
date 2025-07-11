load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:autocxx.bzl", "autocxx_library")
load("//tools/rust:defs.bzl", "flatbuffer_rust_library", "rust_binary", "rust_library", "rust_test")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
)

cc_static_flatbuffer(
    name = "test_message_schema",
    function = "aos::TestMessageSchema",
    target = ":test_message_fbs_fbs_reflection_out",
)

static_flatbuffer(
    name = "event_loop_fbs",
    srcs = ["event_loop.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos:configuration_fbs"],
)

cc_static_flatbuffer(
    name = "timing_report_schema",
    function = "aos::timing::ReportSchema",
    target = ":event_loop_fbs_reflection_out",
)

static_flatbuffer(
    name = "ping_fbs",
    srcs = ["ping.fbs"],
)

flatbuffer_ts_library(
    name = "ping_ts_fbs",
    srcs = ["ping.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    ts_files = [
        "aos/examples/ping.ts",
        "aos/examples.ts",
    ],
)

flatbuffer_rust_library(
    name = "ping_rust_fbs",
    srcs = ["ping.fbs"],
)

flatbuffer_rust_library(
    name = "pong_rust_fbs",
    srcs = ["pong.fbs"],
)

static_flatbuffer(
    name = "pong_fbs",
    srcs = ["pong.fbs"],
)

cc_library(
    name = "epoll",
    srcs = ["epoll.cc"],
    hdrs = ["epoll.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/time",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "epoll_test",
    srcs = ["epoll_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":epoll",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "context",
    hdrs = [
        "context.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:flatbuffers",
        "//aos:uuid",
        "//aos/time",
    ],
)

cc_library(
    name = "event_loop",
    srcs = [
        "event_loop.cc",
        "event_loop_event.h",
        "event_loop_tmpl.h",
    ],
    hdrs = [
        "channel_preallocated_allocator.h",
        "event_loop.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":context",
        ":event_loop_fbs",
        ":timing_statistics",
        "//aos:configuration",
        "//aos:configuration_fbs",
        "//aos:flatbuffers",
        "//aos:ftrace",
        "//aos:realtime",
        "//aos:shared_span",
        "//aos:uuid",
        "//aos/flatbuffers:builder",
        "//aos/ipc_lib:data_alignment",
        "//aos/logging",
        "//aos/time",
        "//aos/util:phased_loop",
        "//aos/util:status",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_tartanllama_expected",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_library(
    name = "event_loop_runtime_cc",
    srcs = [
        "event_loop_runtime.cc",
    ],
    hdrs = [
        "event_loop_runtime.h",
    ],
    deps = [
        ":event_loop",
        "//aos:for_rust",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@crate_index//:cxx_cc",
    ],
)

autocxx_library(
    name = "event_loop_runtime",
    srcs = ["event_loop_runtime.rs"],
    crate_name = "aos_events_event_loop_runtime",
    doctest_deps = [
        ":pong_rust_fbs",
    ],
    gen_tests = False,
    libs = [
        ":event_loop_runtime_cc",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    rs_deps = [
        "@com_github_google_flatbuffers//rust",
        "@crate_index//:uuid",
        "@crate_index//:futures",
        "@crate_index//:thiserror",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration_rs",
        "//aos:uuid_rs",
    ],
)

autocxx_library(
    name = "event_loop_runtime_test_lib_rs",
    testonly = True,
    srcs = ["event_loop_runtime_test_lib.rs"],
    gen_docs = False,
    libs = [
        ":event_loop",
    ],
    rs_deps = [
        ":event_loop_runtime",
        ":ping_rust_fbs",
        ":pong_rust_fbs",
    ],
)

cc_test(
    name = "event_loop_runtime_test",
    srcs = ["event_loop_runtime_test.cc"],
    data = [":pingpong_config"],
    deps = [
        ":event_loop_runtime_test_lib_rs",
        ":ping_fbs",
        ":pong_fbs",
        ":simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_library(
    name = "ping_lib",
    srcs = [
        "ping_lib.cc",
    ],
    hdrs = [
        "ping_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop",
        ":ping_fbs",
        ":pong_fbs",
        "//aos:json_to_flatbuffer",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "ping",
    srcs = [
        "ping.cc",
    ],
    data = [":pingpong_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":ping_lib",
        ":shm_event_loop",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

rust_test(
    name = "pingpong_test_rs",
    srcs = [
        "pingpong_test.rs",
    ],
    data = [":pingpong_config"],
    # TODO(adam.snaider): Remove later. For now we need this because when
    # a rust test crashes inside of C++, the output gets captured which makes
    # it pretty much impossible to figure out what happened.
    env = {"RUST_TEST_NOCAPTURE": "1"},
    deps = [
        ":ping_lib_rs",
        ":ping_rust_fbs",
        ":pong_lib_rs",
        ":pong_rust_fbs",
        "//aos/testing:aos_rs",
        "@com_github_google_flatbuffers//rust",
        "@crate_index//:futures",
    ],
)

rust_binary(
    name = "ping_rs",
    srcs = [
        "ping.rs",
    ],
    data = [
        ":pingpong_config",
    ],
    deps = [
        ":ping_lib_rs",
        ":shm_event_loop_rs",
        "//aos:aos_rs",
        "@crate_index//:clap",
    ],
)

rust_library(
    name = "ping_lib_rs",
    srcs = [
        "ping_lib.rs",
    ],
    crate_name = "ping_lib",
    deps = [
        ":ping_rust_fbs",
        ":pong_rust_fbs",
        "//aos:aos_rs",
        "@com_github_google_flatbuffers//rust",
        "@crate_index//:futures",
        "@crate_index//:log",
    ],
)

rust_binary(
    name = "pong_rs",
    srcs = [
        "pong.rs",
    ],
    data = [
        ":pingpong_config",
    ],
    deps = [
        ":pong_lib_rs",
        ":shm_event_loop_rs",
        "//aos:aos_rs",
        "@crate_index//:clap",
    ],
)

rust_library(
    name = "pong_lib_rs",
    srcs = [
        "pong_lib.rs",
    ],
    crate_name = "pong_lib",
    deps = [
        ":ping_rust_fbs",
        ":pong_rust_fbs",
        "//aos:aos_rs",
        "@com_github_google_flatbuffers//rust",
        "@crate_index//:futures",
        "@crate_index//:log",
    ],
)

aos_config(
    name = "aos_config",
    src = "aos.json",
    flatbuffers = [
        ":event_loop_fbs",
        "//aos/logging:dynamic_log_command_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "pingpong_config",
    src = "pingpong.json",
    flatbuffers = [
        ":ping_fbs",
        ":pong_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":aos_config"],
)

[
    aos_config(
        name = config + "_config",
        src = config + ".json",
        flatbuffers = [
            ":ping_fbs",
            ":pong_fbs",
            "//aos/network:message_bridge_client_fbs",
            "//aos/network:remote_message_fbs",
            "//aos/network:timestamp_fbs",
            "//aos/network:message_bridge_server_fbs",
        ],
        target_compatible_with = ["@platforms//os:linux"],
        deps = [":aos_config"],
    )
    for config in [
        "multinode_pingpong_test_split",
        "multinode_pingpong",
        "multinode_pingpong_test_combined",
    ]
]

cc_library(
    name = "pong_lib",
    srcs = [
        "pong_lib.cc",
    ],
    hdrs = [
        "pong_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop",
        ":ping_fbs",
        ":pong_fbs",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "pong",
    srcs = [
        "pong.cc",
    ],
    data = [":pingpong_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":ping_fbs",
        ":pong_fbs",
        ":pong_lib",
        ":shm_event_loop",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "pingpong_test",
    srcs = ["pingpong_test.cc"],
    data = [
        ":multinode_pingpong_test_split_config",
        ":pingpong_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":ping_lib",
        ":pong_lib",
        ":simulated_event_loop",
        "//aos:configuration",
        "//aos:flatbuffers",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

cc_library(
    name = "timing_statistics",
    srcs = ["timing_statistics.cc"],
    hdrs = ["timing_statistics.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop_fbs",
        "//aos:configuration",
        "//aos/util:error_counter",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "timing_statistics_test",
    srcs = ["timing_statistics_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":timing_statistics",
        "//aos:configuration",
        "//aos:flatbuffers",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "shm_event_loop",
    srcs = ["shm_event_loop.cc"],
    hdrs = ["shm_event_loop.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_logging",
        ":epoll",
        ":event_loop",
        ":event_loop_fbs",
        ":test_message_fbs",
        ":timing_statistics",
        "//aos:init",
        "//aos:realtime",
        "//aos/ipc_lib:lockless_queue",
        "//aos/ipc_lib:shm_base",
        "//aos/ipc_lib:signalfd",
        "//aos/stl_mutex",
        "//aos/util:phased_loop",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "shm_event_loop_test",
    srcs = ["shm_event_loop_test.cc"],
    flaky = True,
    shard_count = 50,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop_param_test",
        ":shm_event_loop",
        ":test_message_fbs",
    ],
)

cc_library(
    name = "event_loop_param_test",
    testonly = True,
    srcs = ["event_loop_param_test.cc"],
    hdrs = ["event_loop_param_test.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop",
        ":test_message_fbs",
        ":test_message_schema",
        ":timing_report_schema",
        "//aos:realtime",
        "//aos/logging:log_message_fbs",
        "//aos/logging:log_message_schema",
        "//aos/network:message_bridge_client_schema",
        "//aos/network:message_bridge_server_schema",
        "//aos/network:timestamp_schema",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "simulated_event_loop_test",
    srcs = ["simulated_event_loop_test.cc"],
    data = [
        ":multinode_pingpong_test_combined_config",
        ":multinode_pingpong_test_split_config",
    ],
    shard_count = 16,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event_loop_param_test",
        ":function_scheduler",
        ":message_counter",
        ":ping_lib",
        ":pong_lib",
        ":simulated_event_loop",
        "//aos/network:remote_message_fbs",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
        "//aos/util:file",
    ],
)

cc_library(
    name = "simple_channel",
    srcs = ["simple_channel.cc"],
    hdrs = ["simple_channel.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration_fbs",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "simulated_event_loop",
    srcs = [
        "event_scheduler.cc",
        "simulated_event_loop.cc",
        "simulated_network_bridge.cc",
    ],
    hdrs = [
        "event_scheduler.h",
        "simulated_event_loop.h",
        "simulated_network_bridge.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_logging",
        ":epoll",
        ":event_loop",
        ":simple_channel",
        "//aos:init",
        "//aos:realtime",
        "//aos:sanitizers",
        "//aos/events/logging:boot_timestamp",
        "//aos/events/logging:logger_fbs",
        "//aos/ipc_lib:index",
        "//aos/network:message_bridge_client_status",
        "//aos/network:message_bridge_server_status",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_channel",
        "//aos/util:phased_loop",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:die_if_null",
        "@com_google_absl//absl/log:log_sink",
    ],
)

cc_library(
    name = "simulated_event_loop_for_rust",
    hdrs = ["simulated_event_loop_for_rust.h"],
    deps = [
        ":simulated_event_loop",
        "//aos:for_rust",
        "@crate_index//:cxx_cc",
    ],
)

cc_library(
    name = "shm_event_loop_for_rust",
    hdrs = ["shm_event_loop_for_rust.h"],
    deps = [
        ":event_loop",
        ":simulated_event_loop",
    ],
)

autocxx_library(
    name = "simulated_event_loop_rs",
    srcs = ["simulated_event_loop.rs"],
    crate_name = "aos_events_simulated_event_loop",
    libs = [
        ":simulated_event_loop",
        ":simulated_event_loop_for_rust",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    rs_deps = [
        "@com_github_google_flatbuffers//rust",
        "//aos:configuration_rust_fbs",
        "//aos:flatbuffers_rs",
        "@crate_index//:futures",
    ],
    test_data = [
        ":multinode_pingpong_test_combined_config",
    ],
    test_deps = [
        ":ping_rust_fbs",
        "//aos:test_init_rs",
        "//aos/testing:path_rs",
        "@crate_index//:futures",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop_runtime",
        "//aos:configuration_rs",
        "//aos:uuid_rs",
    ],
)

autocxx_library(
    name = "shm_event_loop_rs",
    srcs = ["shm_event_loop.rs"],
    crate_name = "aos_events_shm_event_loop",
    doctest_deps = [
        ":ping_rust_fbs",
        ":pong_rust_fbs",
    ],
    libs = [
        ":shm_event_loop",
        ":shm_event_loop_for_rust",
    ],
    rs_deps = [
        "@com_github_google_flatbuffers//rust",
        "@crate_index//:futures",
        "//aos:configuration_rust_fbs",
        "//aos:flatbuffers_rs",
    ],
    test_data = [":pingpong_config"],
    test_deps = [
        ":ping_rust_fbs",
        "//aos:test_init_rs",
        "//aos/testing:path_rs",
        "@crate_index//:futures",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop_runtime",
        "//aos:configuration_rs",
    ],
)

cc_test(
    name = "event_scheduler_test",
    srcs = ["event_scheduler_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":simulated_event_loop",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "aos_logging",
    srcs = [
        "aos_logging.cc",
    ],
    hdrs = [
        "aos_logging.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop",
        "//aos/logging",
        "//aos/logging:log_message_fbs",
    ],
)

cc_library(
    name = "message_counter",
    hdrs = ["message_counter.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":event_loop",
    ],
)

cc_library(
    name = "glib_main_loop",
    srcs = [
        "glib_main_loop.cc",
    ],
    hdrs = [
        "glib_main_loop.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:shm_event_loop",
        "//third_party:gstreamer",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "glib_main_loop_test",
    srcs = [
        "glib_main_loop_test.cc",
    ],
    data = [
        ":aos_config",
    ],
    deps = [
        ":glib_main_loop",
        "//aos:configuration",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//third_party:gstreamer",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "timing_report_dump_lib",
    srcs = ["timing_report_dump_lib.cc"],
    hdrs = ["timing_report_dump_lib.h"],
    deps = ["//aos/events:event_loop"],
)

cc_binary(
    name = "timing_report_dump",
    srcs = ["timing_report_dump.cc"],
    deps = [
        ":timing_report_dump_lib",
        "//aos:init",
        "//aos/events/logging:log_reader",
    ],
)

cc_library(
    name = "function_scheduler",
    srcs = [
        "function_scheduler.cc",
    ],
    hdrs = [
        "function_scheduler.h",
    ],
    deps = [":event_loop"],
)

cc_binary(
    name = "aos_timing_report_streamer",
    srcs = ["aos_timing_report_streamer.cc"],
    deps = [
        ":timing_report_dump_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)
