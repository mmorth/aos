# Incompatible library in case one isn't available for a specific architecture.

cc_library(
    name = "unavailable",
    target_compatible_with = ["@platforms//:incompatible"],
)

cc_library(
    name = "wpilib",
    linkstatic = True,
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = ["//frc/wpilib/ahal"],
)

cc_library(
    name = "wpilib_hal",
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = ["@com_github_wpilibsuite_allwpilib//hal:wpiHal.static"],
)

cc_library(
    name = "phoenix",
    linkopts = [
        "-Wl,-rpath",
        "-Wl,.",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "@ctre_phoenix6_tools_athena//:tools",
        "@ctre_phoenix6_tools_headers//:tools",
        "@ctre_phoenix_api_cpp_athena//:api-cpp",
        "@ctre_phoenix_api_cpp_headers//:api-cpp",
        "@ctre_phoenix_cci_athena//:cci",
        "@ctre_phoenix_cci_headers//:cci",
    ],
)

cc_library(
    name = "phoenix6",
    linkopts = [
        "-Wl,-rpath",
        "-Wl,.",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "//tools/platforms/hardware:roborio": [
            "@ctre_phoenix6_api_cpp_headers//:api-cpp",
            "@ctre_phoenix6_api_cpp_athena//:api-cpp",
            "@ctre_phoenix6_tools_athena//:tools",
            "@ctre_phoenix6_tools_headers//:tools",
            "//third_party:wpilib",
        ],
        "@platforms//cpu:arm64": [
            "@ctre_phoenix6_arm64//:headers",
            "@ctre_phoenix6_arm64//:shared_libraries_lib",
        ],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "opencv",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:opencv4"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:opencv4"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "gstreamer",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": [
            "@amd64_debian_sysroot//:gstreamer-1.0",
            "@amd64_debian_sysroot//:gstreamer-app-1.0",
            "@amd64_debian_sysroot//:gstreamer-webrtc-1.0",
        ],
        "//tools:cpu_arm64": [
            "@arm64_debian_sysroot//:gstreamer-1.0",
            "@arm64_debian_sysroot//:gstreamer-app-1.0",
            "@arm64_debian_sysroot//:gstreamer-webrtc-1.0",
        ],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "halide",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@halide_k8//:halide"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "halide_gengen",
    visibility = ["//visibility:public"],
    # It's the same file in either version, but we'll pick the native version
    # to minimize the chances of needing to download the other version unnecessarily.
    deps = select({
        "//tools:cpu_k8": ["@halide_k8//:gengen"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "halide_runtime",
    visibility = ["//visibility:public"],
    # It's the same file in either version, but we'll pick the native version
    # to minimize the chances of needing to download the other version unnecessarily.
    deps = select({
        "//tools:cpu_k8": ["@halide_k8//:runtime"],
        "//tools:cpu_arm64": ["@halide_arm64//:runtime"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "lzma",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:liblzma"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:liblzma"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "cudart",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:cudart"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:cudart-12.6"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "tensorrt",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@amd64_tensorrt//:tensorrt"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:tensorrt"],
        "//conditions:default": [":unavailable"],
    }),
)

cc_library(
    name = "libturbojpeg",
    visibility = ["//visibility:public"],
    deps = select({
        "//tools:cpu_k8": ["@amd64_debian_sysroot//:libturbojpeg"],
        "//tools:cpu_arm64": ["@arm64_debian_sysroot//:libturbojpeg"],
        "//conditions:default": [":unavailable"],
    }),
)
