load("@com_github_google_flatbuffers//:typescript.bzl", "flatbuffer_ts_library")
load("//aos:config.bzl", "aos_config")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")
load("//tools/build_rules:js.bzl", "ts_project")
load("//tools/build_rules:select.bzl", "cpu_select")
load("//tools/build_rules:template.bzl", "jinja2_template")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "drivetrain_can_position_fbs",
    srcs = ["drivetrain_can_position.fbs"],
    deps = ["//frc/control_loops:can_talonfx_fbs"],
)

static_flatbuffer(
    name = "spline_goal_fbs",
    srcs = ["spline_goal.fbs"],
    deps = ["//frc/control_loops:control_loops_fbs"],
)

static_flatbuffer(
    name = "drivetrain_goal_fbs",
    srcs = ["drivetrain_goal.fbs"],
    deps = [
        ":spline_goal_fbs",
        "//frc/control_loops:control_loops_fbs",
    ],
)

static_flatbuffer(
    name = "rio_localizer_inputs_fbs",
    srcs = ["rio_localizer_inputs.fbs"],
)

static_flatbuffer(
    name = "drivetrain_output_fbs",
    srcs = ["drivetrain_output.fbs"],
)

static_flatbuffer(
    name = "drivetrain_position_fbs",
    srcs = ["drivetrain_position.fbs"],
)

static_flatbuffer(
    name = "drivetrain_status_fbs",
    srcs = ["drivetrain_status.fbs"],
    deps = [
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:encoder_fault_status_fbs",
    ],
)

static_flatbuffer(
    name = "trajectory_fbs",
    srcs = ["trajectory.fbs"],
    deps = ["//frc/control_loops:control_loops_fbs"],
)

cc_static_flatbuffer(
    name = "trajectory_schema",
    function = "frc::control_loops::drivetrain::fb::TrajectorySchema",
    target = ":trajectory_fbs_reflection_out",
    visibility = ["//visibility:public"],
)

flatbuffer_ts_library(
    name = "drivetrain_status_ts_fbs",
    srcs = ["drivetrain_status.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    ts_files = [
        "frc.ts",
        "frc/index-position.ts",
        "frc/pot-and-index-position.ts",
        "frc/pot-and-absolute-position.ts",
        "frc/absolute-position.ts",
        "frc/absolute-and-absolute-position.ts",
        "frc/relative-position.ts",
        "frc/zeroing-error.ts",
        "frc/estimator-state.ts",
        "frc/pot-and-absolute-encoder-estimator-state.ts",
        "frc/absolute-encoder-estimator-state.ts",
        "frc/absolute-and-absolute-encoder-estimator-state.ts",
        "frc/relative-encoder-estimator-state.ts",
        "frc/index-estimator-state.ts",
        "frc/hall-effect-and-position-estimator-state.ts",
        "frc/pot-and-index-pair.ts",
        "frc/hall-effect-struct.ts",
        "frc/hall-effect-and-position.ts",
        "frc/hall-event-positions.ts",
        "frc/posedge-only-counted-hall-effect-struct.ts",
        "frc/profile-parameters.ts",
        "frc/range.ts",
        "frc/constraint-type.ts",
        "frc/constraint.ts",
        "frc/multi-spline.ts",
        "frc/control-loops.ts",
        "frc/control-loops/encoder-fault-status.ts",
        "frc/control-loops/drivetrain.ts",
        "frc/control-loops/drivetrain/gear-logging.ts",
        "frc/control-loops/drivetrain/cimlogging.ts",
        "frc/control-loops/drivetrain/poly-drive-logging.ts",
        "frc/control-loops/drivetrain/planning-state.ts",
        "frc/control-loops/drivetrain/spline-state.ts",
        "frc/control-loops/drivetrain/trajectory-logging.ts",
        "frc/control-loops/drivetrain/robot-side.ts",
        "frc/control-loops/drivetrain/line-follow-logging.ts",
        "frc/control-loops/drivetrain/localizer-state.ts",
        "frc/control-loops/drivetrain/down-estimator-state.ts",
        "frc/control-loops/drivetrain/imu-zeroer-state.ts",
        "frc/control-loops/drivetrain/faults.ts",
        "frc/control-loops/drivetrain/status.ts",
    ],
    deps = [
        "//frc/control_loops:control_loops_ts_fbs",
        "//frc/control_loops:encoder_fault_status_ts_fbs",
    ],
)

flatbuffer_ts_library(
    name = "drivetrain_position_ts_fbs",
    srcs = ["drivetrain_position.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    ts_files = [
        "frc/control-loops/drivetrain.ts",
        "frc/control-loops/drivetrain/position.ts",
    ],
)

flatbuffer_ts_library(
    name = "drivetrain_can_position_ts_fbs",
    srcs = ["drivetrain_can_position.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
    ts_files = [
        "frc/control-loops/drivetrain.ts",
        "frc/control-loops/drivetrain/canposition.ts",
        "frc/control-loops.ts",
        "frc/control-loops/cantalon-fx.ts",
    ],
    deps = ["//frc/control_loops:can_talonfx_ts_fbs"],
)

genrule(
    name = "drivetrain_goal_float_fbs_generated",
    srcs = ["drivetrain_goal.fbs"],
    outs = ["drivetrain_goal_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
)

genrule(
    name = "drivetrain_position_float_fbs_generated",
    srcs = ["drivetrain_position.fbs"],
    outs = ["drivetrain_position_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
)

genrule(
    name = "drivetrain_output_float_fbs_generated",
    srcs = ["drivetrain_output.fbs"],
    outs = ["drivetrain_output_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
)

genrule(
    name = "drivetrain_status_float_fbs_generated",
    srcs = ["drivetrain_status.fbs"],
    outs = ["drivetrain_status_float.fbs"],
    cmd = "cat $(SRCS) | sed 's/double/float/g' > $(OUTS)",
)

static_flatbuffer(
    name = "drivetrain_goal_float_fbs",
    srcs = ["drivetrain_goal_float.fbs"],
    deps = [
        ":spline_goal_fbs",
        "//frc/control_loops:control_loops_fbs",
    ],
)

static_flatbuffer(
    name = "drivetrain_output_float_fbs",
    srcs = ["drivetrain_output_float.fbs"],
)

static_flatbuffer(
    name = "drivetrain_position_float_fbs",
    srcs = ["drivetrain_position_float.fbs"],
)

static_flatbuffer(
    name = "drivetrain_status_float_fbs",
    srcs = ["drivetrain_status_float.fbs"],
    deps = [
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:encoder_fault_status_fbs",
    ],
)

aos_config(
    name = "multinode_drivetrain_config",
    testonly = True,
    src = "multinode_drivetrain_config_source.json",
    flatbuffers = [
        "//aos/events:event_loop_fbs",
        "//aos/logging:dynamic_log_command_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/starter:starter_fbs",
        "//aos/starter:starter_rpc_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//aos/network:remote_message_fbs",
        "//frc/control_loops/drivetrain/localization:localizer_output_fbs",
        ":drivetrain_goal_fbs",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
        ":trajectory_fbs",
        ":spline_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_can_position_fbs",
        ":localizer_fbs",
        "//frc/queues:gyro_fbs",
        "//frc/queues:gyro_uid_fbs",
        "//frc/wpilib:imu_fbs",
        "//frc/wpilib:imu_batch_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = [":__subpackages__"],
)

aos_config(
    name = "simulation_channels",
    src = "drivetrain_simulation_channels.json",
    flatbuffers = [
        ":drivetrain_status_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

aos_config(
    name = "simulation_config",
    src = "drivetrain_simulation_config.json",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_config",
        ":simulation_channels",
    ],
)

aos_config(
    name = "aos_config",
    src = "drivetrain_config.json",
    flatbuffers = [
        ":drivetrain_goal_fbs",
        ":trajectory_fbs",
        ":spline_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_can_position_fbs",
        ":localizer_fbs",
        "//frc/queues:gyro_fbs",
        "//frc/queues:gyro_uid_fbs",
        "//frc/wpilib:imu_fbs",
        "//frc/wpilib:imu_batch_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc/input:aos_config",
    ],
)

cc_library(
    name = "drivetrain_config",
    hdrs = [
        "drivetrain_config.h",
    ],
    deps = [
        ":drivetrain_config_fbs",
        "//frc:shifter_hall_effect",
        "//frc/control_loops:state_feedback_loop",
        "//frc/control_loops:state_feedback_loop_converters",
    ] + select({
        "@platforms//os:linux": [
            "//frc/control_loops:hybrid_state_feedback_loop",
            "//frc/control_loops:hybrid_state_feedback_loop_converters",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "hybrid_ekf",
    hdrs = ["hybrid_ekf.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        "//aos:math",
        "//aos/containers:priority_queue",
        "//aos/util:math",
        "//frc/control_loops:c2d",
        "//frc/control_loops:runge_kutta",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "hybrid_ekf_test",
    srcs = ["hybrid_ekf_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_test_lib",
        ":hybrid_ekf",
        ":trajectory",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "//aos/testing:test_shm",
    ],
)

static_flatbuffer(
    name = "localizer_fbs",
    srcs = ["localizer.fbs"],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "localizer",
    hdrs = ["localizer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_status_fbs",
        ":hybrid_ekf",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:pose",
    ],
)

cc_library(
    name = "gear",
    hdrs = [
        "gear.h",
    ],
)

cc_library(
    name = "splinedrivetrain",
    srcs = [
        "splinedrivetrain.cc",
    ],
    hdrs = [
        "splinedrivetrain.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":spline",
        ":spline_goal_fbs",
        ":trajectory",
        "//aos:condition",
        "//aos:init",
        "//aos/util:math",
        "//frc/control_loops:control_loops_fbs",
    ],
)

cc_library(
    name = "line_follow_drivetrain",
    srcs = [
        "line_follow_drivetrain.cc",
    ],
    hdrs = [
        "line_follow_drivetrain.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_status_fbs",
        ":localizer",
        ":spline_goal_fbs",
        "//aos:math",
        "//aos/util:math",
        "//frc/control_loops:c2d",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:dlqr",
        "//frc/control_loops:pose",
        "//frc/control_loops:profiled_subsystem_fbs",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "line_follow_drivetrain_test",
    srcs = ["line_follow_drivetrain_test.cc"],
    linkstatic = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_test_lib",
        ":line_follow_drivetrain",
        ":trajectory",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//third_party/matplotlib-cpp",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "ssdrivetrain",
    srcs = [
        "ssdrivetrain.cc",
    ],
    hdrs = [
        "ssdrivetrain.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_states",
        ":drivetrain_status_fbs",
        ":gear",
        ":localizer",
        ":spline_goal_fbs",
        "//aos:math",
        "//aos/util:log_interval",
        "//aos/util:trapezoid_profile",
        "//frc:shifter_hall_effect",
        "//frc/control_loops:coerce_goal",
        "//frc/control_loops:control_loop",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:polytope",
        "//frc/control_loops:state_feedback_loop",
        "//frc/input:robot_state_fbs",
    ],
)

cc_library(
    name = "polydrivetrain",
    srcs = [
        "polydrivetrain.cc",
    ],
    hdrs = [
        "polydrivetrain.h",
    ],
    copts = select({
        "@platforms//os:none": ["-Wno-type-limits"],
        "//conditions:default": [],
    }),
    deps = [
        ":drivetrain_config",
        ":drivetrain_states",
        ":gear",
        ":spline_goal_fbs",
        "//aos:math",
        "//frc/control_loops:coerce_goal",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:polytope",
        "//frc/control_loops:state_feedback_loop",
    ] + select({
        "@platforms//os:linux": [
            ":drivetrain_goal_fbs",
            ":drivetrain_output_fbs",
            ":drivetrain_position_fbs",
            ":drivetrain_status_fbs",
            "//aos/util:log_interval",
            "//frc/input:robot_state_fbs",
        ],
        "@platforms//os:none": [
            ":drivetrain_goal_float_fbs",
            ":drivetrain_output_float_fbs",
            ":drivetrain_position_float_fbs",
            ":drivetrain_status_float_fbs",
        ],
    }),
)

genrule(
    name = "genrule_down_estimator",
    outs = [
        "down_estimator.h",
        "down_estimator.cc",
    ],
    cmd = "$(location //frc/control_loops/python:down_estimator) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//frc/control_loops/python:down_estimator",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "down_estimator",
    srcs = [
        "down_estimator.cc",
    ],
    hdrs = [
        "down_estimator.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "drivetrain_states",
    hdrs = ["drivetrain_states.h"],
)

cc_library(
    name = "drivetrain_lib",
    srcs = [
        "drivetrain.cc",
    ],
    hdrs = [
        "drivetrain.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_states",
        ":drivetrain_status_fbs",
        ":gear",
        ":improved_down_estimator",
        ":line_follow_drivetrain",
        ":localizer",
        ":localizer_fbs",
        ":polydrivetrain",
        ":rio_localizer_inputs_fbs",
        ":spline_goal_fbs",
        ":splinedrivetrain",
        ":ssdrivetrain",
        "//aos/util:log_interval",
        "//frc/control_loops:control_loop",
        "//frc/control_loops:runge_kutta",
        "//frc/queues:gyro_fbs",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/wpilib:imu_fbs",
        "//frc/zeroing:imu_zeroer",
    ],
)

cc_library(
    name = "drivetrain_test_lib",
    testonly = True,
    srcs = ["drivetrain_test_lib.cc"],
    hdrs = ["drivetrain_test_lib.h"],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_output_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_status_fbs",
        ":trajectory",
        "//aos/events:event_loop",
        "//aos/testing:googletest",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:state_feedback_loop",
        "//frc/control_loops/drivetrain/test_robot:polydrivetrain_plants",
        "//frc/queues:gyro_fbs",
        "//frc/wpilib:imu_batch_fbs",
        "//frc/wpilib:imu_fbs",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)

cc_test(
    name = "drivetrain_lib_test",
    srcs = [
        "drivetrain_lib_test.cc",
    ],
    data = [":simulation_config"],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    linkstatic = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_goal_fbs",
        ":drivetrain_lib",
        ":drivetrain_output_fbs",
        ":drivetrain_position_fbs",
        ":drivetrain_status_fbs",
        ":drivetrain_test_lib",
        ":localizer_fbs",
        ":trajectory_generator",
        "//aos/events/logging:log_writer",
        "//aos/testing:googletest",
        "//frc/control_loops:control_loop_test",
        "//frc/queues:gyro_fbs",
        "//frc/wpilib:imu_fbs",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)

genrule(
    name = "genrule_haptic_wheel",
    outs = [
        "haptic_wheel.h",
        "haptic_wheel.cc",
        "integral_haptic_wheel.h",
        "integral_haptic_wheel.cc",
        "haptic_trigger.h",
        "haptic_trigger.cc",
        "integral_haptic_trigger.h",
        "integral_haptic_trigger.cc",
    ],
    cmd = "$(location //frc/control_loops/python:haptic_wheel) $(OUTS)",
    tools = [
        "//frc/control_loops/python:haptic_wheel",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "haptic_wheel",
    srcs = [
        "haptic_trigger.cc",
        "haptic_wheel.cc",
        "integral_haptic_trigger.cc",
        "integral_haptic_wheel.cc",
    ],
    hdrs = [
        "haptic_trigger.h",
        "haptic_wheel.h",
        "integral_haptic_trigger.h",
        "integral_haptic_wheel.h",
    ],
    deps = [
        "//frc/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "spline",
    srcs = ["spline.cc"],
    hdrs = ["spline.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc/control_loops:binomial",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_binary(
    name = "spline.so",
    srcs = ["libspline.cc"],
    linkshared = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        ":spline",
        ":trajectory",
        "//aos/logging",
        "//aos/network:team_number",
        "//frc/control_loops/drivetrain/test_robot:drivetrain_base",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "spline_test",
    srcs = [
        "spline_test.cc",
    ],
    linkstatic = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":spline",
        "//aos/analysis:in_process_plotter",
        "//aos/testing:googletest",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "distance_spline",
    srcs = ["distance_spline.cc"],
    hdrs = ["distance_spline.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":spline",
        ":trajectory_fbs",
        "//aos/containers:sized_array",
        "//aos/logging",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:fixed_quadrature",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "distance_spline_test",
    srcs = [
        "distance_spline_test.cc",
    ],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    linkstatic = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        "//aos:flatbuffers",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "@com_google_absl//absl/flags:flag",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)

cc_library(
    name = "trajectory",
    srcs = ["trajectory.cc"],
    hdrs = ["trajectory.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        ":drivetrain_config",
        ":spline_goal_fbs",
        ":trajectory_fbs",
        "//aos/util:math",
        "//frc/control_loops:c2d",
        "//frc/control_loops:dlqr",
        "//frc/control_loops:hybrid_state_feedback_loop",
        "//frc/control_loops:runge_kutta",
        "//frc/control_loops:state_feedback_loop",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "trajectory_generator",
    srcs = ["trajectory_generator.cc"],
    hdrs = ["trajectory_generator.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        ":drivetrain_config",
        ":spline_goal_fbs",
        ":trajectory",
        ":trajectory_fbs",
    ],
)

cc_binary(
    name = "trajectory_plot",
    srcs = [
        "trajectory_plot.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":distance_spline",
        ":trajectory",
        "//aos/logging",
        "//aos/network:team_number",
        "//frc/control_loops/drivetrain/test_robot:drivetrain_base",
        "//third_party/matplotlib-cpp",
        "@com_google_absl//absl/flags:flag",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = [
        "trajectory_test.cc",
    ],
    defines =
        cpu_select({
            "amd64": [
                "SUPPORT_PLOT=1",
            ],
            "arm": [],
        }),
    linkstatic = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_test_lib",
        ":trajectory",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
    ] + cpu_select({
        "amd64": [
            "//third_party/matplotlib-cpp",
        ],
        "arm": [],
    }),
)

cc_library(
    name = "improved_down_estimator",
    srcs = [
        "improved_down_estimator.cc",
    ],
    hdrs = [
        "improved_down_estimator.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_status_fbs",
        "//aos/events:event_loop",
        "//aos/time",
        "//aos/util:math",
        "//frc/control_loops:control_loops_fbs",
        "//frc/control_loops:quaternion_utils",
        "//frc/control_loops:runge_kutta",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "improved_down_estimator_test",
    srcs = [
        "improved_down_estimator_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_test_lib",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "//frc/control_loops:quaternion_utils",
        "//frc/control_loops/drivetrain:improved_down_estimator",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "camera",
    srcs = ["camera.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/containers:sized_array",
        "//frc/control_loops:pose",
    ],
)

cc_library(
    name = "drivetrain_encoder_fault_detector",
    srcs = ["drivetrain_encoder_fault_detector.cc"],
    hdrs = ["drivetrain_encoder_fault_detector.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:event_loop",
        "//frc/control_loops:encoder_fault_detector",
        "//frc/control_loops:encoder_fault_status_fbs",
        "//frc/control_loops/drivetrain:drivetrain_can_position_fbs",
        "//frc/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc/control_loops/drivetrain:drivetrain_status_fbs",
    ],
)

cc_test(
    name = "drivetrain_encoder_fault_detector_test",
    srcs = ["drivetrain_encoder_fault_detector_test.cc"],
    data = [":simulation_config"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":drivetrain_encoder_fault_detector",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "camera_test",
    srcs = ["camera_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":camera",
        "//aos/testing:googletest",
    ],
)

ts_project(
    name = "down_estimator_plotter",
    srcs = ["down_estimator_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
        "//frc/wpilib:imu_plot_utils",
    ],
)

ts_project(
    name = "spline_plotter",
    srcs = ["spline_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)

ts_project(
    name = "drivetrain_plotter",
    srcs = ["drivetrain_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
        "//frc/wpilib:imu_plot_utils",
    ],
)

ts_project(
    name = "robot_state_plotter",
    srcs = ["robot_state_plotter.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network/www:aos_plotter",
        "//aos/network/www:colors",
        "//aos/network/www:proxy",
    ],
)

static_flatbuffer(
    name = "drivetrain_config_fbs",
    srcs = ["drivetrain_config.fbs"],
    visibility = ["//visibility:public"],
    deps = ["//frc/control_loops:state_feedback_loop_fbs"],
)

cc_binary(
    name = "drivetrain_config_merge",
    srcs = ["drivetrain_config_merge.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":drivetrain_config_fbs",
        "//aos:flatbuffer_merge",
        "//aos:init",
        "//aos:json_to_flatbuffer",
    ],
)

cc_test(
    name = "drivetrain_config_test",
    srcs = ["drivetrain_config_test.cc"],
    data = [":drivetrain_test_config.json"],
    deps = [
        ":drivetrain_config_fbs",
        ":drivetrain_test_lib",
        "//aos/testing:googletest",
        "//aos/testing:path",
    ],
)

jinja2_template(
    name = "drivetrain_test_config.json",
    src = "drivetrain_test_config.jinja2.json",
    includes = [
        "//frc/control_loops/drivetrain/test_robot:drivetrain_config",
    ],
    parameters = {},
    visibility = ["//visibility:public"],
)
