load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

cc_library(
    name = "imu",
    srcs = [
        "imu.cc",
    ],
    hdrs = [
        "imu.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:epoll",
        "//aos/events:shm_event_loop",
        "//aos/util:crc32",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/wpilib:imu_fbs",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
    ],
)

static_flatbuffer(
    name = "imu_failures_fbs",
    srcs = [
        "imu_failures.fbs",
    ],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "imu_failures_ts_fbs",
    srcs = [
        "imu_failures.fbs",
    ],
    ts_files = [
        "frc/controls.ts",
        "frc/controls/imu-failures.ts",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "imu_watcher",
    srcs = ["imu_watcher.cc"],
    hdrs = ["imu_watcher.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":imu_failures_fbs",
        "//aos/events:event_loop",
        "//frc/control_loops/drivetrain:drivetrain_config",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/zeroing:imu_zeroer",
        "//frc/zeroing:wrap",
    ],
)
