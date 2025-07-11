load("@bazel_skylib//lib:selects.bzl", "selects")
load("//tools/build_rules:clean_dep.bzl", "clean_dep")

package(default_visibility = ["//visibility:public"])

exports_files(["test_sharding_compliant"])

alias(
    name = "gen_rust_project",
    actual = "@rules_rust//tools/rust_analyzer:gen_rust_project",
    target_compatible_with = select({
        "//tools/platforms/rust:has_support": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

# Don't use these directly! Use //tools/build_rules/*.bzl instead.
config_setting(
    name = "compiler_clang",
    flag_values = {
        "@bazel_tools//tools/cpp:compiler": "clang",
    },
)

config_setting(
    name = "compiler_gcc",
    flag_values = {
        "@bazel_tools//tools/cpp:compiler": "gcc",
    },
)

# TODO(phil): Get rid of these and just use the constraint values directly
# everywhere.
config_setting(
    name = "cpu_k8",
    constraint_values = ["@platforms//cpu:x86_64"],
)

config_setting(
    name = "cpu_roborio",
    constraint_values = [clean_dep("//tools/platforms/hardware:roborio")],
)

config_setting(
    name = "cpu_cortex_m4f",
    constraint_values = [clean_dep("//tools/platforms/hardware:cortex_m4f")],
)

config_setting(
    name = "cpu_cortex_m0plus",
    constraint_values = [clean_dep("//tools/platforms/hardware:cortex_m0plus")],
)

config_setting(
    name = "cpu_cortex-m4f-imu",
    constraint_values = [clean_dep("//tools/platforms/hardware:cortex-m4f-imu")],
)

# TODO(phil): Re-enable this with proper support.
#config_setting(
#    name = "cpu_cortex_m4f_k22",
#    constraint_values = ["@platforms//cpu:?"],
#)

config_setting(
    name = "cpu_armv7",
    constraint_values = [
        "@platforms//cpu:armv7",
    ],
)

config_setting(
    name = "cpu_arm64",
    constraint_values = ["@platforms//cpu:arm64"],
)

config_setting(
    name = "has_asan",
    values = {"define": "have_asan=true"},
)

config_setting(
    name = "has_tsan",
    values = {"define": "have_tsan=true"},
)

config_setting(
    name = "has_ubsan",
    values = {"define": "have_ubsan=true"},
)

config_setting(
    name = "has_msan",
    values = {"define": "have_msan=true"},
)

config_setting(
    name = "cpu_k8_msan",
    constraint_values = ["@platforms//cpu:x86_64"],
    values = {
        "define": "have_msan=true",
    },
)

config_setting(
    name = "cpu_k8_no_msan",
    constraint_values = ["@platforms//cpu:x86_64"],
    flag_values = {
        "@io_bazel_rules_go//go/config:msan": "false",
    },
)

config_setting(
    name = "cpu_arm64_no_msan",
    constraint_values = ["@platforms//cpu:arm64"],
    flag_values = {
        "@io_bazel_rules_go//go/config:msan": "false",
    },
)

selects.config_setting_group(
    name = "has_sanitizer",
    match_any = [
        ":has_msan",
        ":has_asan",
        ":has_ubsan",
    ],
)
