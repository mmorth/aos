load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

cc_library(
    name = "utils",
    srcs = ["utils.cc"],
    hdrs = ["utils.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//frc/control_loops:pose",
        "//frc/control_loops/drivetrain:drivetrain_output_fbs",
        "//frc/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc/control_loops/drivetrain:hybrid_ekf",
        "//frc/control_loops/drivetrain:rio_localizer_inputs_fbs",
        "//frc/input:joystick_state_fbs",
        "//frc/vision:calibration_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "puppet_localizer",
    srcs = ["puppet_localizer.cc"],
    hdrs = ["puppet_localizer.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
        "//frc/control_loops/drivetrain:hybrid_ekf",
        "//frc/control_loops/drivetrain:localizer",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
    ],
)

cc_test(
    name = "puppet_localizer_test",
    srcs = ["puppet_localizer_test.cc"],
    data = ["//frc/control_loops/drivetrain:multinode_drivetrain_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":puppet_localizer",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_writer",
        "//aos/network:team_number",
        "//frc/control_loops:control_loop_test",
        "//frc/control_loops:team_number_test_environment",
        "//frc/control_loops/drivetrain:drivetrain_lib",
        "//frc/control_loops/drivetrain:drivetrain_test_lib",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        "//frc/control_loops/drivetrain/test_robot:drivetrain_base",
    ],
)

static_flatbuffer(
    name = "localizer_output_fbs",
    srcs = [
        "localizer_output.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "localizer_output_ts_fbs",
    srcs = ["localizer_output.fbs"],
    ts_files = [
        "frc/controls.ts",
        "frc/controls/quaternion.ts",
        "frc/controls/led-output.ts",
        "frc/controls/localizer-output.ts",
    ],
    visibility = ["//visibility:public"],
)
