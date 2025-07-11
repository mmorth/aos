load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:clean_dep.bzl", "aos_repo_name")

cc_library(
    name = "static_flatbuffers",
    srcs = ["static_flatbuffers.cc"],
    hdrs = ["static_flatbuffers.h"],
    local_defines = ["AOS_REPO_NAME=\\\"" + aos_repo_name() + "\\\""],
    deps = [
        ":attributes",
        ":static_table",
        ":static_vector",
        "//aos:json_to_flatbuffer",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "base",
    srcs = [
        "base.cc",
    ],
    hdrs = [
        "base.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:shared_span",
        "//aos/containers:resizeable_buffer",
        "//aos/ipc_lib:data_alignment",
        "@aos//aos:realtime",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/types:span",
    ],
)

cc_test(
    name = "base_test",
    srcs = ["base_test.cc"],
    deps = [
        ":base",
        ":static_flatbuffers",
        ":test_fbs",
        "//aos:sanitizers",
        "//aos/testing:googletest",
        "@aos//aos:realtime",
    ],
)

cc_library(
    name = "static_table",
    hdrs = ["static_table.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":base",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
    ],
)

cc_library(
    name = "static_vector",
    hdrs = ["static_vector.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/containers:inlined_vector",
        "//aos/containers:sized_array",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "builder",
    hdrs = ["builder.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":static_table",
        "//aos:flatbuffers",
        "@com_google_absl//absl/log:absl_check",
    ],
)

cc_static_flatbuffer(
    name = "test_schema",
    function = "aos::fbs::testing::TestTableSchema",
    target = ":test_fbs_fbs_reflection_out",
)

cc_static_flatbuffer(
    name = "interesting_schemas",
    function = "aos::fbs::testing::UnsupportedSchema",
    target = ":interesting_schemas_fbs_reflection_out",
)

flatbuffer_cc_library(
    name = "interesting_schemas_fbs",
    srcs = ["interesting_schemas.fbs"],
    gen_reflections = True,
)

cc_test(
    name = "static_flatbuffers_test",
    srcs = ["static_flatbuffers_test.cc"],
    data = ["//aos/flatbuffers/test_dir:test_data"],
    deps = [
        ":binary_annotator",
        ":builder",
        ":interesting_schemas",
        ":static_flatbuffers",
        ":test_fbs",
        ":test_schema",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/flatbuffers/test_dir:include_reflection_fbs",
        "//aos/flatbuffers/test_dir:type_coverage_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

cc_test(
    name = "static_flatbuffers_fuzz_test",
    # This does a combinatoric sweep over a variety of things, and can end up taking a while.
    timeout = "eternal",
    srcs = ["static_flatbuffers_fuzz_test.cc"],
    data = ["//aos/flatbuffers/test_dir:test_data"],
    deps = [
        ":builder",
        ":static_flatbuffers",
        ":test_fbs",
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/flatbuffers/test_dir:type_coverage_fbs",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

cc_binary(
    name = "generate",
    srcs = ["generate.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":static_flatbuffers",
        "//aos:flatbuffers",
        "//aos:init",
        "//aos:json_to_flatbuffer",
        "//aos/util:file",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

sh_binary(
    name = "generate_wrapper",
    srcs = ["generate.sh"],
    data = [
        ":generate",
    ],
    visibility = ["//visibility:public"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)

static_flatbuffer(
    name = "test_fbs",
    srcs = ["test.fbs"],
    deps = ["//aos/flatbuffers/test_dir:include_fbs"],
)

filegroup(
    name = "test_static_file",
    srcs = ["test_static.h"],
    visibility = [":__subpackages__"],
)

cc_library(
    name = "attributes",
    hdrs = ["attributes.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
)

cc_library(
    name = "binary_annotator",
    srcs = ["binary_annotator.cc"],
    hdrs = ["binary_annotator.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_flatbuffers//src:flatc_library",
    ],
)

cc_test(
    name = "binary_annotator_test",
    srcs = ["binary_annotator_test.cc"],
    data = [":test_fbs_fbs_reflection_out"],
    deps = [
        ":binary_annotator",
        ":test_fbs",
        ":test_schema",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)
