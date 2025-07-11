load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("@rules_python//python:defs.bzl", "py_library", "py_test")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("config_validator_macro.bzl", "config_validator_test")

package(
    default_applicable_licenses = ["//:license"],
    default_visibility = ["//visibility:public"],
)

config_validator_test(
    name = "config_validator_test",
    config = "//aos/events:pingpong_config",
)

cc_library(
    name = "bitpacking",
    hdrs = [
        "bitpacking.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "bitpacking_test",
    srcs = [
        "bitpacking_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":bitpacking",
        "//aos/testing:googletest",
    ],
)

py_library(
    name = "py_trapezoid_profile",
    srcs = [
        "trapezoid_profile.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":python_init",
    ],
)

cc_library(
    name = "log_to_mcap_lib",
    srcs = ["log_to_mcap_lib.cc"],
    hdrs = ["log_to_mcap_lib.h"],
    implementation_deps = [
        ":clock_publisher",
        ":clock_timepoints_schema",
        ":mcap_logger",
        "//aos:configuration",
        "@com_google_absl//absl/flags:flag",
    ],
    deps = [
        "//aos:configuration_fbs",
        "//aos/events/logging:log_reader",
    ],
)

cc_binary(
    name = "log_to_mcap",
    srcs = ["log_to_mcap.cc"],
    deps = [
        ":log_to_mcap_lib",
        "//aos:init",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "error_counter",
    hdrs = ["error_counter.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
    ],
)

cc_test(
    name = "error_counter_test",
    srcs = ["error_counter_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":error_counter",
        "//aos:flatbuffers",
        "//aos/events:event_loop_fbs",
        "//aos/testing:googletest",
    ],
)

static_flatbuffer(
    name = "clock_timepoints_fbs",
    srcs = ["clock_timepoints.fbs"],
    deps = ["//aos:configuration_fbs"],
)

cc_static_flatbuffer(
    name = "clock_timepoints_schema",
    function = "aos::ClockTimepointsSchema",
    target = ":clock_timepoints_fbs_reflection_out",
)

cc_library(
    name = "clock_publisher",
    srcs = ["clock_publisher.cc"],
    hdrs = ["clock_publisher.h"],
    deps = [
        ":clock_timepoints_fbs",
        "//aos/events:simulated_event_loop",
    ],
)

static_flatbuffer(
    name = "log_conversion_metadata_fbs",
    srcs = ["log_conversion_metadata.fbs"],
)

cc_static_flatbuffer(
    name = "log_conversion_metadata_schema",
    function = "aos::LogConversionMetadataSchema",
    target = ":log_conversion_metadata_fbs_reflection_out",
)

static_flatbuffer(
    name = "live_metadata_fbs",
    srcs = ["live_metadata.fbs"],
)

cc_static_flatbuffer(
    name = "live_metadata_schema",
    function = "aos::LiveMetadataSchema",
    target = ":live_metadata_fbs_reflection_out",
)

cc_library(
    name = "mcap_logger",
    srcs = ["mcap_logger.cc"],
    hdrs = ["mcap_logger.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":log_conversion_metadata_fbs",
        ":log_conversion_metadata_schema",
        "//aos:configuration_fbs",
        "//aos:configuration_schema",
        "//aos:fast_string_builder",
        "//aos:flatbuffer_utils",
        "//aos/events:event_loop",
        "@com_github_lz4_lz4//:lz4",
        "@com_github_nlohmann_json//:nlohmann_json",
    ],
)

cc_binary(
    name = "generate_test_log",
    testonly = True,
    srcs = ["generate_test_log.cc"],
    data = ["//aos/events:pingpong_config"],
    deps = [
        "//aos:configuration",
        "//aos/events:ping_lib",
        "//aos/events:pong_lib",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/testing:path",
    ],
)

py_test(
    name = "log_to_mcap_test",
    srcs = ["log_to_mcap_test.py"],
    args = [
        "--log_to_mcap",
        "$(location :log_to_mcap)",
        "--mcap",
        "$(location @com_github_foxglove_mcap_mcap//file)",
        "--generate_log",
        "$(location :generate_test_log)",
    ],
    data = [
        ":generate_test_log",
        ":log_to_mcap",
        "@com_github_foxglove_mcap_mcap//file",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
)

cc_test(
    name = "mcap_logger_test",
    srcs = ["mcap_logger_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":mcap_logger",
        "//aos/testing:googletest",
        "@com_github_nlohmann_json//:nlohmann_json",
    ],
)

cc_library(
    name = "math",
    hdrs = ["math.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "math_test",
    srcs = ["math_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":math",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "death_test_log_implementation",
    hdrs = [
        "death_test_log_implementation.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
    ],
)

cc_library(
    name = "phased_loop",
    srcs = [
        "phased_loop.cc",
    ],
    hdrs = [
        "phased_loop.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/time",
        "@com_google_absl//absl/log:absl_check",
    ],
)

cc_library(
    name = "log_interval",
    hdrs = [
        "log_interval.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
        "//aos/time",
    ],
)

cc_library(
    name = "string_formatting",
    srcs = [
        "string_formatting.cc",
    ],
    hdrs = [
        "string_formatting.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_test(
    name = "string_formatting_test",
    srcs = ["string_formatting_test.cc"],
    deps = [
        ":string_formatting",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "trapezoid_profile",
    srcs = [
        "trapezoid_profile.cc",
    ],
    hdrs = [
        "trapezoid_profile.h",
    ],
    linkopts = [
        "-lm",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
        "//aos/time",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "trapezoid_profile_test",
    srcs = [
        "trapezoid_profile_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":trapezoid_profile",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "wrapping_counter",
    srcs = [
        "wrapping_counter.cc",
    ],
    hdrs = [
        "wrapping_counter.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "wrapping_counter_test",
    srcs = [
        "wrapping_counter_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":wrapping_counter",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "options",
    hdrs = [
        "options.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "options_test",
    srcs = [
        "options_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":options",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "compiler_memory_barrier",
    hdrs = [
        "compiler_memory_barrier.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "global_factory",
    hdrs = [
        "global_factory.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "global_factory_test",
    srcs = [
        "global_factory_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":global_factory",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "phased_loop_test",
    srcs = [
        "phased_loop_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":phased_loop",
        "//aos/testing:googletest",
        "//aos/time",
    ],
)

cc_library(
    name = "file",
    srcs = [
        "file.cc",
    ],
    hdrs = [
        "file.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:sanitizers",
        "//aos/scoped:scoped_fd",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "file_test",
    size = "small",
    srcs = [
        "file_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":file",
        "//aos:realtime",
        "//aos/testing:googletest",
        "//aos/testing:tmpdir",
    ],
)

static_flatbuffer(
    name = "process_info_fbs",
    srcs = ["process_info.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "process_info_ts_fbs",
    srcs = ["process_info.fbs"],
    namespace = "aos.util",
    tables = [
        "ThreadState",
        "ThreadInfo",
        "ProcessInfo",
        "TopProcessesFbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "top",
    srcs = ["top.cc"],
    hdrs = ["top.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":process_info_fbs",
        "//aos/containers:ring_buffer",
        "//aos/events:event_loop",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "top_test",
    srcs = ["top_test.cc"],
    data = [
        "//aos/events:pingpong_config",
    ],
    flaky = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":top",
        "//aos/events:shm_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:tmpdir",
    ],
)

cc_library(
    name = "scoped_pipe",
    srcs = ["scoped_pipe.cc"],
    hdrs = ["scoped_pipe.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "scoped_pipe_test",
    srcs = ["scoped_pipe_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":scoped_pipe",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "crc32",
    srcs = ["crc32.cc"],
    hdrs = ["crc32.h"],
    deps = [
        "@com_google_absl//absl/types:span",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//aos:python_init"],
)

cc_library(
    name = "threaded_queue",
    hdrs = [
        "threaded_queue.h",
        "threaded_queue_tmpl.h",
    ],
    deps = [
        "//aos:condition",
        "//aos/mutex",
    ],
)

cc_test(
    name = "threaded_queue_test",
    srcs = ["threaded_queue_test.cc"],
    deps = [
        ":threaded_queue",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "threaded_consumer",
    hdrs = [
        "threaded_consumer.h",
    ],
    deps = [
        "//aos:condition",
        "//aos:realtime",
        "//aos/containers:ring_buffer",
        "//aos/mutex",
    ],
)

cc_test(
    name = "threaded_consumer_test",
    srcs = ["threaded_consumer_test.cc"],
    deps = [
        ":threaded_consumer",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "foxglove_websocket_lib",
    srcs = ["foxglove_websocket_lib.cc"],
    hdrs = ["foxglove_websocket_lib.h"],
    deps = [
        ":live_metadata_fbs",
        ":live_metadata_schema",
        ":mcap_logger",
        "//aos:configuration_fbs",
        "//aos:configuration_schema",
        "//aos/events:event_loop",
        "@com_github_foxglove_ws-protocol",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:die_if_null",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "config_validator",
    testonly = True,
    srcs = ["config_validator.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":config_validator_lib",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_googletest//:gtest",
    ],
)

cc_binary(
    name = "foxglove_websocket",
    srcs = ["foxglove_websocket.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":foxglove_websocket_lib",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:usage",
    ],
)

cc_library(
    name = "simulation_logger",
    srcs = ["simulation_logger.cc"],
    hdrs = ["simulation_logger.h"],
    deps = [
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
    ],
)

static_flatbuffer(
    name = "config_validator_config_fbs",
    srcs = ["config_validator_config.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "config_validator_lib",
    testonly = True,
    srcs = ["config_validator_lib.cc"],
    hdrs = ["config_validator_lib.h"],
    deps = [
        ":config_validator_config_fbs",
        ":simulation_logger",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/events/logging:log_writer",
        "//aos/network:timestamp_channel",
        "//aos/testing:tmpdir",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "config_validator_lib_test",
    srcs = ["config_validator_lib_test.cc"],
    data = [
        "//aos/util/test_data:multinode_common_logger",
        "//aos/util/test_data:multinode_extraneous_timestamp",
        "//aos/util/test_data:multinode_invalid_timestamp_logger_list",
        "//aos/util/test_data:multinode_no_logged_timestamps",
        "//aos/util/test_data:multinode_no_statistics",
        "//aos/util/test_data:multinode_timestamp_typo",
        "//aos/util/test_data:valid_multinode_config",
        "//aos/util/test_data:valid_singlenode_config",
    ],
    deps = [
        ":config_validator_lib",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

static_flatbuffer(
    name = "filesystem_fbs",
    srcs = ["filesystem.fbs"],
)

cc_static_flatbuffer(
    name = "filesystem_schema",
    function = "aos::util::FilesystemStatusSchema",
    target = ":filesystem_fbs_reflection_out",
)

cc_binary(
    name = "filesystem_monitor",
    srcs = ["filesystem_monitor.cc"],
    deps = [
        ":filesystem_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "status",
    srcs = ["status.cc"],
    hdrs = ["status.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/containers:inlined_vector",
        "@com_github_tartanllama_expected",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_test(
    name = "status_test",
    srcs = ["status_test.cc"],
    deps = [
        ":status",
        "//aos:realtime",
        "//aos:sanitizers",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)
