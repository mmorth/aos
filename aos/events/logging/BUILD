load("@rules_python//python:defs.bzl", "py_binary", "py_test")
load("//aos:config.bzl", "aos_config")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "logger_fbs",
    srcs = ["logger.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration_fbs",
    ],
)

static_flatbuffer(
    name = "replay_timing_fbs",
    srcs = ["replay_timing.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_static_flatbuffer(
    name = "replay_timing_schema",
    function = "aos::timing::ReplayTimingSchema",
    target = ":replay_timing_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "log_replayer_stats_fbs",
    srcs = ["log_replayer_stats.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":replay_config_fbs",
    ],
)

cc_static_flatbuffer(
    name = "log_replayer_stats_schema",
    function = "aos::LogReplayerStatsSchema",
    target = ":log_replayer_stats_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "replay_config_fbs",
    srcs = ["log_replayer_config.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration_fbs",
    ],
)

cc_library(
    name = "boot_timestamp",
    srcs = ["boot_timestamp.cc"],
    hdrs = ["boot_timestamp.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/time",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "log_replayer",
    srcs = [
        "log_replayer.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_reader",
        ":log_reader_utils",
        ":log_replayer_stats_fbs",
        ":log_replayer_stats_schema",
        ":replay_config_fbs",
        ":replay_timing_fbs",
        ":replay_timing_schema",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:shm_event_loop",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/log:die_if_null",
    ],
)

cc_library(
    name = "file_operations",
    srcs = ["file_operations.cc"],
    hdrs = ["file_operations.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "s3_file_operations",
    srcs = ["s3_file_operations.cc"],
    hdrs = ["s3_file_operations.h"],
    deps = [
        ":file_operations",
        ":s3_fetcher",
    ],
)

cc_library(
    name = "log_backend",
    srcs = ["log_backend.cc"],
    hdrs = ["log_backend.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_encoder",
        ":file_operations",
        ":logfile_decoder_options",
        "//aos/time",
        "//aos/util:file",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "log_backend_test",
    srcs = ["log_backend_test.cc"],
    shard_count = 4,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_backend",
        "//aos/containers:resizeable_buffer",
        "//aos/testing:googletest",
        "//aos/testing:tmpdir",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "logfile_decoder_options",
    srcs = ["logfile_decoder_options.cc"],
    hdrs = ["logfile_decoder_options.h"],
    copts = select({
        "//tools:cpu_k8": ["-DENABLE_S3=1"],
        "//conditions:default": ["-DENABLE_S3=0"],
    }),
    deps = [
        ":file_operations",
        ":snappy_encoder",
        "//aos:sanitizers",
    ] + select({
        "//tools:cpu_k8": [
            ":s3_fetcher",
            ":s3_file_operations",
        ],
        "//tools:cpu_k8_no_msan": [
            ":lzma_encoder",
            ":s3_fetcher",
            ":s3_file_operations",
        ],
        "//tools:cpu_arm64_no_msan": [
            ":lzma_encoder",
        ],
        "//tools:cpu_arm64": [":lzma_encoder"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "log_reader_utils",
    srcs = [
        "log_reader_utils.cc",
    ],
    hdrs = [
        "log_reader_utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_reader",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:uuid",
        "//aos/containers:resizeable_buffer",
        "//aos/events:event_loop",
        "//aos/util:file",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "logfile_utils",
    srcs = [
        "logfile_sorting.cc",
        "logfile_utils.cc",
    ],
    hdrs = [
        "logfile_sorting.h",
        "logfile_utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":boot_timestamp",
        ":buffer_encoder",
        ":file_operations",
        ":log_backend",
        ":logfile_decoder_options",
        ":logger_fbs",
        ":snappy_encoder",
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:flatbuffers",
        "//aos:sha256",
        "//aos:uuid",
        "//aos/containers:error_list",
        "//aos/containers:resizeable_buffer",
        "//aos/events:event_loop",
        "//aos/network:remote_message_fbs",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
    ] + select({
        "//tools:cpu_k8": [
            ":lzma_encoder",
            ":s3_fetcher",
            ":s3_file_operations",
        ],
        "//tools:cpu_arm64": [":lzma_encoder"],
        "//conditions:default": [],
    }),
)

cc_binary(
    name = "logfile_utils_out_of_space_test_runner",
    testonly = True,
    srcs = [
        "logfile_utils_out_of_space_test_runner.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":logfile_utils",
        "//aos:init",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

sh_test(
    name = "logfile_utils_out_of_space_test",
    srcs = [
        "logfile_utils_out_of_space_test.sh",
    ],
    data = [
        ":logfile_utils_out_of_space_test_runner",
    ],
    env = {
        "RUNNER": "$(rootpath :logfile_utils_out_of_space_test_runner)",
    },
    tags = [
        # This uses unclone to make a new mount namespace.  Turns out, this is
        # quite hard to figure out how to put in a remote cluster.  So just
        # give up and run it locally under linux-sandbox where it is easy.
        "no-remote-exec",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

cc_library(
    name = "buffer_encoder",
    srcs = [
        "buffer_encoder.cc",
    ],
    hdrs = [
        "buffer_encoder.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":logger_fbs",
        "//aos:configuration_fbs",
        "//aos:flatbuffers",
        "//aos/containers:resizeable_buffer",
        "//aos/time",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "buffer_encoder_test",
    srcs = [
        "buffer_encoder_test.cc",
    ],
    shard_count = 4,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":buffer_encoder",
        ":buffer_encoder_param_test",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "snappy_encoder",
    srcs = [
        "snappy_encoder.cc",
    ],
    hdrs = [
        "snappy_encoder.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_encoder",
        ":logger_fbs",
        "//aos:configuration_fbs",
        "//aos/containers:resizeable_buffer",
        "//aos/util:crc32",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
        "@snappy",
    ],
)

cc_test(
    name = "snappy_encoder_test",
    srcs = [
        "snappy_encoder_test.cc",
    ],
    shard_count = 4,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":buffer_encoder_param_test",
        ":snappy_encoder",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "lzma_encoder",
    srcs = [
        "lzma_encoder.cc",
    ],
    hdrs = [
        "lzma_encoder.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_encoder",
        ":logger_fbs",
        "//aos:configuration_fbs",
        "//aos/containers:resizeable_buffer",
        "//third_party:lzma",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "s3_fetcher",
    srcs = [
        "s3_fetcher.cc",
    ],
    hdrs = [
        "s3_fetcher.h",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":buffer_encoder",
        "//aos/containers:resizeable_buffer",
        "@aws_sdk//:core",
        "@aws_sdk//:s3",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "lzma_encoder_test",
    srcs = [
        "lzma_encoder_test.cc",
    ],
    shard_count = 8,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":buffer_encoder_param_test",
        ":lzma_encoder",
        "//aos/testing:googletest",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "buffer_encoder_param_test",
    testonly = True,
    srcs = [
        "buffer_encoder_param_test.cc",
    ],
    hdrs = [
        "buffer_encoder_param_test.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":logfile_utils",
        ":logger_fbs",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "log_namer",
    srcs = [
        "log_namer.cc",
    ],
    hdrs = [
        "log_namer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":logfile_utils",
        ":logger_fbs",
        "//aos:uuid",
        "//aos/containers:error_list",
        "//aos/containers:sized_array",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "log_writer",
    srcs = [
        "log_writer.cc",
    ],
    hdrs = [
        "log_writer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":log_namer",
        "//aos:configuration",
        "//aos:sha256",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/network:message_bridge_server_fbs",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "log_reader",
    srcs = [
        "log_reader.cc",
    ],
    hdrs = [
        "log_reader.h",
        "replay_channels.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":config_remapper",
        ":log_namer",
        ":log_writer",
        ":logfile_utils",
        ":logger_fbs",
        ":replay_timing_fbs",
        "//aos:condition",
        "//aos:uuid",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/mutex",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:multinode_timestamp_filter",
        "//aos/network:noncausal_timestamp_filter",
        "//aos/network:remote_message_fbs",
        "//aos/network:remote_message_schema",
        "//aos/network:team_number",
        "//aos/time",
        "//aos/util:file",
        "//aos/util:threaded_queue",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "config_remapper",
    srcs = [
        "config_remapper.cc",
    ],
    hdrs = [
        "config_remapper.h",
        "replay_channels.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":logfile_utils",
        ":logger_fbs",
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:multinode_timestamp_filter",
        "//aos/network:noncausal_timestamp_filter",
        "//aos/network:remote_message_fbs",
        "//aos/network:remote_message_schema",
        "//aos/network:team_number",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "config_remapper_test",
    srcs = ["config_remapper_test.cc"],
    copts = select({
        "//tools:cpu_k8": ["-DLZMA=1"],
        "//tools:cpu_arm64": ["-DLZMA=1"],
        "//conditions:default": [],
    }),
    data = [
        ":multinode_pingpong_combined_config",
        ":multinode_pingpong_split3_config",
        ":multinode_pingpong_split4_config",
        ":multinode_pingpong_split4_mixed1_config",
        ":multinode_pingpong_split4_mixed2_config",
        ":multinode_pingpong_split4_reliable_config",
        ":multinode_pingpong_split_config",
        ":multinode_pingpong_triangle_split_config",
        "//aos/events:pingpong_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":config_remapper",
        ":multinode_logger_test_lib",
    ],
)

cc_library(
    name = "node_printer",
    srcs = [
        "node_printer.cc",
    ],
    hdrs = [
        "node_printer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:aos_cli_utils",
        "//aos/events:event_loop",
        "//aos/events:simulated_event_loop",
    ],
)

cc_binary(
    name = "log_cat",
    srcs = [
        "log_cat.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_reader",
        ":node_printer",
        "//aos:aos_cli_utils",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos:sha256",
        "//aos/events:simulated_event_loop",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "logfile_validator",
    srcs = [
        "logfile_validator.cc",
    ],
    hdrs = ["logfile_validator.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":logfile_utils",
        "//aos:configuration",
        "//aos/events:simulated_event_loop",
        "//aos/network:multinode_timestamp_filter",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "timestamp_extractor",
    srcs = [
        "timestamp_extractor.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":logfile_utils",
        ":logfile_validator",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/network:multinode_timestamp_filter",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "single_node_merge",
    srcs = [
        "single_node_merge.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":logfile_utils",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/network:multinode_timestamp_filter",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "log_edit",
    srcs = [
        "log_edit.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/util:file",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "log_stats",
    srcs = [
        "log_stats.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/time",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "logger_main",
    srcs = [
        "logger_main.cc",
    ],
    copts = select({
        "//tools:cpu_k8": ["-DLZMA=1"],
        "//tools:cpu_arm64": ["-DLZMA=1"],
        "//conditions:default": [],
    }),
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_writer",
        ":snappy_encoder",
        "//aos:configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging:log_namer",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "multinode_logger_test_lib",
    testonly = True,
    srcs = [
        "multinode_logger_test_lib.cc",
    ],
    hdrs = ["multinode_logger_test_lib.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_reader",
        ":log_writer",
        ":logfile_validator",
        ":snappy_encoder",
        "//aos/events:message_counter",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:simulated_event_loop",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

aos_config(
    name = "multinode_pingpong_split_config",
    src = "multinode_pingpong_split.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_split3_config",
    src = "multinode_pingpong_split3.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_split4_config",
    src = "multinode_pingpong_split4.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_split4_mixed1_config",
    src = "multinode_pingpong_split4_mixed1.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_split4_mixed2_config",
    src = "multinode_pingpong_split4_mixed2.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_split4_reliable_config",
    src = "multinode_pingpong_split4_reliable.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_combined_config",
    src = "multinode_pingpong_combined.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_triangle_split_config",
    src = "multinode_pingpong_triangle_split.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_reboot_reliable_only_config",
    src = "multinode_pingpong_reboot_reliable_only.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_reboot_ooo_config",
    src = "multinode_pingpong_reboot_ooo.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_pingpong_pi3_pingpong_config",
    src = "multinode_pingpong_pi3_pingpong.json",
    flatbuffers = [
        "//aos/events:ping_fbs",
        "//aos/events:pong_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

aos_config(
    name = "multinode_single_node_config",
    src = "multinode_single_node.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:timestamp_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["//aos/events:aos_config"],
)

cc_test(
    name = "realtime_replay_test",
    srcs = ["realtime_replay_test.cc"],
    data = [
        ":multinode_pingpong_combined_config",
        "//aos/events:pingpong_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader",
        ":log_writer",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_test(
    name = "log_reader_utils_test",
    srcs = ["log_reader_utils_test.cc"],
    data = [
        ":multinode_pingpong_combined_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader_utils",
        ":multinode_logger_test_lib",
        "@com_google_absl//absl/log:die_if_null",
    ],
)

cc_test(
    name = "multinode_logger_test",
    srcs = ["multinode_logger_test.cc"],
    copts = select({
        "//tools:cpu_k8": ["-DLZMA=1"],
        "//tools:cpu_arm64": ["-DLZMA=1"],
        "//conditions:default": [],
    }),
    data = [
        ":multinode_pingpong_combined_config",
        ":multinode_pingpong_pi3_pingpong_config",
        ":multinode_pingpong_reboot_ooo_config",
        ":multinode_pingpong_reboot_reliable_only_config",
        ":multinode_pingpong_split3_config",
        ":multinode_pingpong_split4_config",
        ":multinode_pingpong_split4_mixed1_config",
        ":multinode_pingpong_split4_mixed2_config",
        ":multinode_pingpong_split4_reliable_config",
        ":multinode_pingpong_split_config",
        ":multinode_pingpong_triangle_split_config",
        ":multinode_single_node_config",
        "//aos/events:pingpong_config",
    ],
    shard_count = 10,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":multinode_logger_test_lib",
        "//aos/flatbuffers:base",
    ],
)

cc_test(
    name = "logger_test",
    srcs = ["logger_test.cc"],
    copts = select({
        "//tools:cpu_k8": ["-DLZMA=1"],
        "//tools:cpu_arm64": ["-DLZMA=1"],
        "//conditions:default": [],
    }),
    data = [
        "//aos/events:pingpong_config",
    ],
    shard_count = 10,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_reader",
        ":log_writer",
        ":snappy_encoder",
        "//aos/events:message_counter",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:simulated_event_loop",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_test(
    name = "logfile_utils_test",
    srcs = ["logfile_utils_test.cc"],
    data = [
        ":logger_fbs_reflection_out",
        "//aos/network:remote_message_fbs_reflection_out",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":logfile_utils",
        ":logger_fbs",
        ":test_message_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:random_seed",
        "//aos/testing:tmpdir",
        "@com_github_google_flatbuffers//src:flatc_library",
    ],
)

static_flatbuffer(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "timestamp_plot",
    srcs = ["timestamp_plot.cc"],
    deps = [
        "//aos:init",
        "//aos/analysis:in_process_plotter",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "log_config_extractor",
    srcs = [
        "log_config_extractor.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:aos_cli_utils",
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events/logging:log_reader",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "ssd_profiler",
    srcs = [
        "ssd_profiler.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":log_backend",
        "//aos:init",
        "//aos/containers:resizeable_buffer",
        "//aos/time",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

py_binary(
    name = "plot_logger_profile",
    srcs = [
        "plot_logger_profile.py",
    ],
    target_compatible_with = [
        # TODO(PRO-24640): Remove compatibility.
        "@platforms//cpu:x86_64",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@pip//bokeh",
        "@pip//numpy",
        "@pip//tabulate",
    ],
)

py_test(
    name = "plot_logger_profile_test",
    srcs = [
        "plot_logger_profile_test.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":plot_logger_profile",
        "@pip//numpy",
    ],
)
