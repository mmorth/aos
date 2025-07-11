load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:autocxx.bzl", "autocxx_library")
load("//tools/build_rules:select.bzl", "compiler_select")
load("//tools/rust:defs.bzl", "flatbuffer_rust_library", "rust_library")

package(default_applicable_licenses = ["//:license"])

exports_files(["aos_dump_autocomplete.sh"])

filegroup(
    name = "prime_binaries",
    srcs = [
        "//aos:aos_dump",
        "//aos:aos_dump_autocomplete.sh",
        "//aos:aos_send",
        "//aos/starter",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_start_binaries",
    srcs = [
        "//aos/events/logging:logger_main",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_binaries_stripped",
    srcs = [
        "//aos:aos_dump.stripped",
        "//aos:aos_dump_autocomplete.sh",
        "//aos:aos_send.stripped",
        "//aos/starter:starter_stripped",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "prime_start_binaries_stripped",
    srcs = [
        "//aos/events/logging:logger_main.stripped",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "math",
    hdrs = [
        "commonmath.h",
    ],
    visibility = ["//visibility:public"],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "macros",
    hdrs = [
        "macros.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "gtest_prod",
    hdrs = [
        "gtest_prod.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "unique_malloc_ptr",
    hdrs = [
        "unique_malloc_ptr.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "condition",
    srcs = [
        "condition.cc",
    ],
    hdrs = [
        "condition.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/ipc_lib:aos_sync",
        "//aos/mutex",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "condition_test",
    srcs = [
        "condition_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":condition",
        "//aos:die",
        "//aos/ipc_lib:aos_sync",
        "//aos/ipc_lib:core_lib",
        "//aos/logging",
        "//aos/mutex",
        "//aos/testing:googletest",
        "//aos/testing:prevent_exit",
        "//aos/testing:test_shm",
        "//aos/time",
    ],
)

cc_library(
    name = "die",
    srcs = [
        "die.cc",
    ],
    hdrs = [
        "die.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:macros",
        "//aos/libc:aos_strerror",
    ],
)

cc_test(
    name = "die_test",
    srcs = [
        "die_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":die",
        "//aos/testing:googletest",
    ],
)

cc_binary(
    name = "dump_rtprio",
    srcs = [
        "dump_rtprio.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/time",
        "//aos/util:top",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "init",
    srcs = [
        "init.cc",
    ],
    hdrs = [
        "init.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":realtime",
        ":uuid",
        "//aos:die",
        "//aos/logging",
        "@com_google_absl//absl/debugging:failure_signal_handler",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/log:flags",
        "@com_google_absl//absl/log:globals",
        "@com_google_absl//absl/log:initialize",
    ],
)

cc_library(
    name = "init_for_rust",
    srcs = [
        "init_for_rust.cc",
    ],
    hdrs = [
        "init_for_rust.h",
    ],
    deps = [
        ":for_rust",
        ":init",
        "//aos/logging",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:flags",
        "@crate_index//:cxx_cc",
    ],
)

autocxx_library(
    name = "init_rs",
    srcs = ["init.rs"],
    crate_name = "aos_init",
    libs = [
        ":init_for_rust",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    visibility = ["//visibility:public"],
    deps = [
        "@crate_index//:clap",
        "@crate_index//:env_logger",
    ],
)

autocxx_library(
    name = "test_init_rs",
    testonly = True,
    srcs = ["test_init.rs"],
    crate_name = "aos_test_init",
    gen_docs = False,
    libs = [
        "//aos/testing:tmpdir",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    visibility = ["//visibility:public"],
    deps = [
        ":init_rs",
        "@crate_index//:env_logger",
    ],
)

cc_test(
    name = "stacktrace_test",
    srcs = [
        "stacktrace_test.cc",
    ],
    copts = ["-Wno-error=attributes"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":realtime",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "realtime",
    srcs = [
        "realtime.cc",
    ],
    hdrs = [
        "realtime.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":sanitizers",
        ":uuid",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

static_flatbuffer(
    name = "configuration_fbs",
    srcs = ["configuration.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//aos/flatbuffers/reflection:reflection_fbs"],
)

cc_static_flatbuffer(
    name = "configuration_schema",
    function = "aos::ConfigurationSchema",
    target = ":configuration_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "configuration_ts_fbs",
    srcs = ["configuration.fbs"],
    include_reflection = True,
    namespace = "aos",
    tables = [
        "LoggerConfig",
        "Connection",
        "ReadMethod",
        "Channel",
        "Map",
        "Application",
        "Node",
        "Configuration",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_rust_library(
    name = "configuration_rust_fbs",
    srcs = ["configuration.fbs"],
    crate_name = "aos_configuration_fbs",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "configuration",
    srcs = [
        "configuration.cc",
    ],
    hdrs = [
        "configuration.h",
    ],
    copts = compiler_select({
        "clang": [],
        "gcc": ["-Wno-missing-field-initializers"],
    }),
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration_fbs",
        ":flatbuffer_merge",
        ":flatbuffers",
        ":json_to_flatbuffer",
        "//aos:unique_malloc_ptr",
        "//aos/ipc_lib:index",
        "//aos/network:team_number",
        "//aos/util:file",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "configuration_for_rust",
    srcs = [
        "configuration_for_rust.cc",
    ],
    hdrs = [
        "configuration_for_rust.h",
    ],
    deps = [
        ":configuration",
        ":for_rust",
        "@crate_index//:cxx_cc",
    ],
)

autocxx_library(
    name = "configuration_rs",
    srcs = ["configuration.rs"],
    crate_name = "aos_configuration",
    libs = [
        ":configuration",
        ":configuration_for_rust",
        ":configuration_fbs",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    test_data = [
        "//aos/testdata:test_configs",
    ],
    test_deps = [
        "//aos/testing:path_rs",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration_rust_fbs",
        ":flatbuffers_rs",
        "@crate_index//:thiserror",
    ],
)

flatbuffer_ts_library(
    name = "json_to_flatbuffer_fbs_ts",
    srcs = ["json_to_flatbuffer.fbs"],
    namespace = "aos.testing",
    tables = [
        "BaseType",
        "NonConsecutive",
        "Location",
        "Map",
        "Application",
        "VectorOfStrings",
        "VectorOfVectorOfString",
        "Configuration",
        "FooStructNested",
        "FooStruct",
        "ScalarSweepStruct",
        "StructEnum",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//aos:__subpackages__"],
)

flatbuffer_cc_library(
    name = "json_to_flatbuffer_fbs",
    srcs = ["json_to_flatbuffer.fbs"],
    gen_reflections = True,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//aos:__subpackages__"],
)

flatbuffer_rust_library(
    name = "json_to_flatbuffer_rust_fbs",
    srcs = ["json_to_flatbuffer.fbs"],
    crate_name = "aos_json_to_flatbuffer_fbs",
    visibility = ["//aos:__subpackages__"],
)

cc_library(
    name = "flatbuffer_utils",
    srcs = ["flatbuffer_utils.cc"],
    hdrs = ["flatbuffer_utils.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_library(
    name = "json_tokenizer",
    srcs = ["json_tokenizer.cc"],
    hdrs = ["json_tokenizer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "json_to_flatbuffer",
    srcs = [
        "flatbuffer_introspection.cc",
        "json_to_flatbuffer.cc",
    ],
    hdrs = ["json_to_flatbuffer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":fast_string_builder",
        ":flatbuffer_utils",
        ":flatbuffers",
        ":json_tokenizer",
        "//aos/flatbuffers:builder",
        "//aos/util:file",
        "//aos/util:string_formatting",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "json_to_flatbuffer_test",
    srcs = [
        "json_to_flatbuffer_test.cc",
    ],
    data = [
        ":json_to_flatbuffer_fbs_reflection_out",
        ":json_to_flatbuffer_test_spaces.json",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":flatbuffer_merge",
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

cc_test(
    name = "flatbuffer_introspection_test",
    srcs = [
        "flatbuffer_introspection_test.cc",
    ],
    data = [
        ":json_to_flatbuffer_fbs_reflection_out",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "flatbuffer_merge",
    srcs = ["flatbuffer_merge.cc"],
    hdrs = ["flatbuffer_merge.h"],
    copts = ["-Wno-cast-align"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":flatbuffer_utils",
        ":flatbuffers",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "flatbuffer_merge_test",
    srcs = [
        "flatbuffer_merge_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":flatbuffer_merge",
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_fbs",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "flatbuffers",
    srcs = [
        "flatbuffers.cc",
    ],
    hdrs = [
        "flatbuffers.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:macros",
        "//aos/containers:resizeable_buffer",
        "//aos/ipc_lib:data_alignment",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ] + select({
        "//tools:has_sanitizer": [":realtime"],
        "//conditions:default": [],
    }),
)

rust_library(
    name = "flatbuffers_rs",
    srcs = ["flatbuffers.rs"],
    crate_name = "aos_flatbuffers",
    test_deps = [
        ":json_to_flatbuffer_rust_fbs",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_flatbuffers//rust",
    ],
)

cc_test(
    name = "configuration_test",
    srcs = [
        "configuration_test.cc",
    ],
    data = [
        "//aos/events:ping_fbs_reflection_out",
        "//aos/events:pingpong_config",
        "//aos/events:pong_fbs_reflection_out",
        "//aos/testdata:test_configs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":configuration",
        "//aos/events:ping_fbs",
        "//aos/testing:flatbuffer_eq",
        "//aos/testing:googletest",
        "//aos/testing:path",
        "//aos/testing:test_logging",
    ],
)

cc_binary(
    name = "config_flattener",
    srcs = [
        "config_flattener.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        ":init",
        "//aos/util:file",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "aos_cli_utils",
    srcs = [
        "aos_cli_utils.cc",
    ],
    hdrs = [
        "aos_cli_utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/events:simulated_event_loop",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "aos_dump",
    srcs = [
        "aos_dump.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_cli_utils",
        ":configuration",
        ":json_to_flatbuffer",
        "//aos:init",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "aos_jitter",
    srcs = [
        "aos_jitter.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_cli_utils",
        ":configuration",
        ":init",
        ":json_to_flatbuffer",
        ":realtime",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "aos_send",
    srcs = [
        "aos_send.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_cli_utils",
        ":configuration",
        ":init",
        ":json_to_flatbuffer",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "aos_graph_nodes",
    srcs = [
        "aos_graph_nodes.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":configuration",
        ":json_to_flatbuffer",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "ftrace",
    srcs = [
        "ftrace.cc",
    ],
    hdrs = [
        "ftrace.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "fast_string_builder",
    srcs = [
        "fast_string_builder.cc",
    ],
    hdrs = [
        "fast_string_builder.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_test(
    name = "realtime_test",
    srcs = [
        "realtime_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":init",
        ":realtime",
        "//aos:sanitizers",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "flatbuffers_test",
    srcs = [
        "flatbuffers_test.cc",
    ],
    deps = [
        ":flatbuffers",
        ":json_to_flatbuffer",
        ":json_to_flatbuffer_fbs",
        ":realtime",
        "//aos/testing:googletest",
        "//aos/testing:tmpdir",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/strings",
    ],
)

py_binary(
    name = "flatbuffers_static",
    srcs = ["flatbuffers_static.py"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "uuid",
    srcs = ["uuid.cc"],
    hdrs = ["uuid.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "uuid_collision_test",
    timeout = "eternal",
    srcs = ["uuid_collision_test.cc"],
    shard_count = 2,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":uuid",
        "//aos/testing:googletest",
        "@com_google_absl//absl/container:btree",
    ],
)

cc_test(
    name = "uuid_test",
    srcs = ["uuid_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":uuid",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "uuid_for_rust",
    hdrs = ["uuid_for_rust.h"],
    deps = [
        ":uuid",
    ],
)

autocxx_library(
    name = "uuid_rs",
    srcs = ["uuid.rs"],
    crate_name = "aos_uuid",
    libs = [
        ":uuid",
        ":uuid_for_rust",
    ],
    override_cc_toolchain = "@llvm_toolchain//:cc-clang-x86_64-linux",
    rs_deps = [
        "@crate_index//:uuid",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "aos_graph_channels",
    srcs = [
        "aos_graph_channels.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/time",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "for_rust",
    hdrs = [
        "for_rust.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@crate_index//:cxx_cc",
    ],
)

cc_library(
    name = "sha256",
    srcs = [
        "sha256.cc",
    ],
    hdrs = ["sha256.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/util:file",
        "@boringssl//:crypto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "sha256_test",
    srcs = ["sha256_test.cc"],
    deps = [
        ":sha256",
        "//aos/testing:googletest",
        "//aos/testing:tmpdir",
    ],
)

rust_library(
    name = "aos_rs",
    srcs = ["aos.rs"],
    crate_name = "aos",
    visibility = ["//visibility:public"],
    deps = [
        ":configuration_rs",
        ":flatbuffers_rs",
        ":init_rs",
        ":uuid_rs",
        "//aos/events:event_loop_runtime",
        "//aos/events:shm_event_loop_rs",
        "//aos/events:simulated_event_loop_rs",
    ],
)

cc_library(
    name = "shared_span",
    hdrs = ["shared_span.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "sanitizers",
    hdrs = [
        "sanitizers.h",
    ],
    visibility = ["//:__subpackages__"],
)
