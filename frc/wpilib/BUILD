load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:js.bzl", "ts_project")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "logging_fbs",
    srcs = [
        "logging.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "encoder_and_potentiometer",
    srcs = [
        "encoder_and_potentiometer.cc",
    ],
    hdrs = [
        "encoder_and_potentiometer.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":dma",
        ":dma_edge_counting",
        "//aos:init",
        "//aos/logging",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "dma_edge_counting",
    srcs = [
        "dma_edge_counting.cc",
    ],
    hdrs = [
        "dma_edge_counting.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":dma",
        "//aos/containers:sized_array",
        "//aos/logging",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "interrupt_edge_counting",
    srcs = [
        "interrupt_edge_counting.cc",
    ],
    hdrs = [
        "interrupt_edge_counting.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//aos:init",
        "//aos/logging",
        "//aos/stl_mutex",
        "//aos/time",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "buffered_pcm",
    srcs = [
        "buffered_pcm.cc",
        "buffered_solenoid.cc",
    ],
    hdrs = [
        "buffered_pcm.h",
        "buffered_solenoid.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//aos/logging",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "gyro_interface",
    srcs = [
        "gyro_interface.cc",
    ],
    hdrs = [
        "gyro_interface.h",
    ],
    # This library uses some deprecated parts of the SPI API.
    copts = ["-Wno-deprecated-declarations"],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//aos/logging",
        "//aos/time",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "gyro_sender",
    srcs = [
        "gyro_sender.cc",
    ],
    hdrs = [
        "gyro_sender.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":gyro_interface",
        "//aos:init",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/time",
        "//aos/util:phased_loop",
        "//frc/input:robot_state_fbs",
        "//frc/queues:gyro_fbs",
        "//frc/queues:gyro_uid_fbs",
        "//frc/zeroing:averager",
    ],
)

static_flatbuffer(
    name = "loop_output_handler_test_fbs",
    srcs = [
        "loop_output_handler_test.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "aos_config",
    src = "wpilib_config.json",
    flatbuffers = [
        ":pdp_values_fbs",
        ":logging_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "loop_output_handler_test_config",
    src = "loop_output_handler_test_config_source.json",
    flatbuffers = [
        ":loop_output_handler_test_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:aos_config",
    ],
)

cc_library(
    name = "loop_output_handler",
    srcs = [
        "loop_output_handler.cc",
    ],
    hdrs = [
        "loop_output_handler.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/events:event_loop",
        "//aos/scoped:scoped_fd",
        "//aos/time",
        "//aos/util:log_interval",
        "//frc/input:robot_state_fbs",
    ],
)

cc_test(
    name = "loop_output_handler_test",
    srcs = [
        "loop_output_handler_test.cc",
    ],
    data = [
        ":loop_output_handler_test_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":loop_output_handler",
        ":loop_output_handler_test_fbs",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
    ],
)

cc_library(
    name = "joystick_sender",
    srcs = [
        "joystick_sender.cc",
    ],
    hdrs = [
        "joystick_sender.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/network:team_number",
        "//frc/input:driver_station_data",
        "//frc/input:joystick_state_fbs",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    hdrs = [
        "wpilib_interface.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//aos/events:event_loop",
        "//frc/input:robot_state_fbs",
        "//third_party:wpilib",
    ],
)

static_flatbuffer(
    name = "pdp_values_fbs",
    srcs = [
        "pdp_values.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "pdp_fetcher",
    srcs = [
        "pdp_fetcher.cc",
    ],
    hdrs = [
        "pdp_fetcher.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":pdp_values_fbs",
        "//aos:init",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/util:phased_loop",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "wpilib_robot_base",
    hdrs = [
        "wpilib_robot_base.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//third_party:wpilib",
    ],
)

static_flatbuffer(
    name = "imu_fbs",
    srcs = [
        "imu.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "imu_batch_fbs",
    srcs = [
        "imu_batch.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":imu_fbs",
    ],
)

flatbuffer_ts_library(
    name = "imu_ts_fbs",
    srcs = ["imu.fbs"],
    ts_files = [
        "frc.ts",
        "frc/adis16470-diag-stat.ts",
        "frc/imuvalues.ts",
    ],
)

flatbuffer_ts_library(
    name = "imu_batch_ts_fbs",
    srcs = ["imu_batch.fbs"],
    ts_files = [
        "frc.ts",
        "frc/adis16470-diag-stat.ts",
        "frc/imuvalues.ts",
        "frc/imuvalues-batch.ts",
    ],
    deps = [
        ":imu_ts_fbs",
    ],
)

cc_library(
    name = "ADIS16470",
    srcs = [
        "ADIS16470.cc",
    ],
    hdrs = [
        "ADIS16470.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":fpga_time_conversion",
        ":imu_batch_fbs",
        ":imu_fbs",
        "//aos/containers:sized_array",
        "//aos/events:event_loop",
        "//aos/time",
        "//third_party:wpilib",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "ADIS16448",
    srcs = [
        "ADIS16448.cc",
    ],
    hdrs = [
        "ADIS16448.h",
    ],
    # This library uses some deprecated parts of the SPI API.
    copts = ["-Wno-deprecated-declarations"],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":fpga_time_conversion",
        ":imu_batch_fbs",
        ":imu_fbs",
        ":spi_rx_clearer",
        "//aos:init",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/time",
        "//frc/zeroing:averager",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "dma",
    srcs = [
        "dma.cc",
    ],
    hdrs = [
        "dma.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "spi_rx_clearer",
    srcs = [
        "spi_rx_clearer.cc",
    ],
    hdrs = [
        "spi_rx_clearer.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
    ],
)

cc_library(
    name = "sensor_reader",
    srcs = [
        "sensor_reader.cc",
    ],
    hdrs = [
        "sensor_reader.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":dma",
        ":dma_edge_counting",
        ":encoder_and_potentiometer",
        ":fpga_time_conversion",
        ":wpilib_interface",
        "//aos:init",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:phased_loop",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops/drivetrain:drivetrain_position_fbs",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "drivetrain_writer",
    srcs = [
        "drivetrain_writer.cc",
    ],
    hdrs = [
        "drivetrain_writer.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":loop_output_handler",
        "//aos:math",
        "//aos/logging",
        "//frc/control_loops/drivetrain:drivetrain_output_fbs",
        "//third_party:wpilib",
    ],
)

cc_library(
    name = "fpga_time_conversion",
    srcs = [
        "fpga_time_conversion.cc",
    ],
    hdrs = [
        "fpga_time_conversion.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        "//aos/time",
        "//aos/util:compiler_memory_barrier",
        "//third_party:wpilib",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "wpilib_utils",
    srcs = [
        "wpilib_utils.cc",
    ],
    hdrs = [
        "wpilib_utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc:constants",
    ],
)

cc_test(
    name = "wpilib_utils_test",
    srcs = [
        "wpilib_utils_test.cc",
    ],
    data = [
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/testing:flatbuffer_eq",
        "//aos/testing:test_logging",
        "//frc/wpilib:wpilib_utils",
    ],
)

ts_project(
    name = "imu_plot_utils",
    srcs = ["imu_plot_utils.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":imu_batch_ts_fbs",
        "//aos:configuration_ts_fbs",
        "//aos/network/www:aos_plotter",
        "//aos/network/www:plotter",
        "//aos/network/www:reflection_ts",
        "@com_github_google_flatbuffers//reflection:reflection_ts_fbs",
        "@com_github_google_flatbuffers//ts:flatbuffers_ts",
    ],
)

ts_project(
    name = "imu_plotter",
    srcs = ["imu_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":imu_plot_utils",
        "//aos/network/www:aos_plotter",
        "//aos/network/www:proxy",
    ],
)

cc_library(
    name = "talonfx",
    srcs = [
        "talonfx.cc",
    ],
    hdrs = [
        "talonfx.h",
    ],
    deps = [
        "//aos:init",
        "//aos:math",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//frc/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//third_party:phoenix6",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "can_drivetrain_writer",
    srcs = [
        "can_drivetrain_writer.cc",
    ],
    hdrs = [
        "can_drivetrain_writer.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":loop_output_handler",
        ":talonfx",
        "//frc:can_configuration_fbs",
        "//frc/control_loops/drivetrain:drivetrain_output_fbs",
    ],
)

cc_library(
    name = "generic_can_writer",
    srcs = [
        "generic_can_writer.cc",
    ],
    hdrs = [
        "generic_can_writer.h",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":loop_output_handler",
        ":talonfx",
        "//frc:can_configuration_fbs",
    ],
)

cc_library(
    name = "can_sensor_reader",
    srcs = ["can_sensor_reader.cc"],
    hdrs = ["can_sensor_reader.h"],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        ":talonfx",
        "//aos:realtime",
        "//aos/containers:sized_array",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
    ],
)

cc_binary(
    name = "joystick_republish",
    srcs = [
        "joystick_republish.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:configuration",
        "//aos:flatbuffer_merge",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//frc/input:joystick_state_fbs",
        "@com_google_absl//absl/flags:flag",
    ],
)
