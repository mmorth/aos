load(
    "@com_github_google_flatbuffers//:typescript.bzl",
    "flatbuffer_ts_library",
)
load("//aos:config.bzl", "aos_config")
load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "subsystem_simulator",
    testonly = True,
    hdrs = [
        "subsystem_simulator.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc:constants",
        "//frc/control_loops:capped_test_plant",
        "//frc/control_loops:control_loop_test",
        "//frc/control_loops:position_sensor_sim",
    ],
)

cc_library(
    name = "control_loop_test",
    testonly = True,
    srcs = [
        "control_loop_test.cc",
    ],
    hdrs = [
        "control_loop_test.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:flatbuffers",
        "//aos:json_to_flatbuffer",
        "//aos/events:simulated_event_loop",
        "//aos/network:testing_time_converter",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/time",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
    ],
)

cc_library(
    name = "polytope",
    hdrs = [
        "polytope.h",
    ],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ] + select({
        "@platforms//os:linux": [
            "//aos/logging",
            "//third_party/cddlib",
            "@com_google_absl//absl/log",
            "@com_google_absl//absl/log:check",
        ],
        "//conditions:default": [],
    }),
)

cc_test(
    name = "polytope_test",
    srcs = [
        "polytope_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":polytope",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "control_loop",
    srcs = [
        "control_loop.cc",
        "control_loop-tmpl.h",
    ],
    hdrs = [
        "control_loop.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/actions:action_lib",
        "//aos/events:event_loop",
        "//aos/events:shm_event_loop",
        "//aos/flatbuffers:static_table",
        "//aos/logging",
        "//aos/time",
        "//aos/util:log_interval",
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
    ],
)

cc_library(
    name = "quaternion_utils",
    srcs = [
        "quaternion_utils.cc",
    ],
    hdrs = [
        "quaternion_utils.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "quarternion_utils_test",
    srcs = [
        "quaternion_utils_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":jacobian",
        ":quaternion_utils",
        ":runge_kutta",
        "//aos/testing:googletest",
        "//aos/testing:random_seed",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "team_number_test_environment",
    testonly = True,
    srcs = [
        "team_number_test_environment.cc",
    ],
    hdrs = [
        "team_number_test_environment.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/network:team_number",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "hybrid_state_feedback_loop_test",
    srcs = [
        "hybrid_state_feedback_loop_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":hybrid_state_feedback_loop",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "pose",
    hdrs = ["pose.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/util:math",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "pose_test",
    srcs = ["pose_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":pose",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "encoder_fault_detector",
    srcs = ["encoder_fault_detector.cc"],
    hdrs = ["encoder_fault_detector.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":encoder_fault_status_fbs",
        "//aos/containers:sized_array",
        "//aos/time",
        "//frc/control_loops:can_talonfx_fbs",
    ],
)

cc_test(
    name = "encoder_fault_detector_test",
    srcs = ["encoder_fault_detector_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":encoder_fault_detector",
        "//aos:json_to_flatbuffer",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "hall_effect_tracker",
    hdrs = [
        "hall_effect_tracker.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
    ],
)

flatbuffer_ts_library(
    name = "control_loops_ts_fbs",
    srcs = [
        "control_loops.fbs",
    ],
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
    ],
)

static_flatbuffer(
    name = "control_loops_fbs",
    srcs = [
        "control_loops.fbs",
    ],
)

flatbuffer_ts_library(
    name = "encoder_fault_status_ts_fbs",
    srcs = [
        "encoder_fault_status.fbs",
    ],
    ts_files = [
        "frc/control-loops.ts",
        "frc/control-loops/encoder-fault-status.ts",
    ],
)

static_flatbuffer(
    name = "encoder_fault_status_fbs",
    srcs = [
        "encoder_fault_status.fbs",
    ],
)

flatbuffer_ts_library(
    name = "can_talonfx_ts_fbs",
    srcs = [
        "can_talonfx.fbs",
    ],
    ts_files = [
        "frc/control-loops.ts",
        "frc/control-loops/cantalon-fx.ts",
    ],
)

static_flatbuffer(
    name = "can_talonfx_fbs",
    srcs = [
        "can_talonfx.fbs",
    ],
)

cc_test(
    name = "position_sensor_sim_test",
    srcs = [
        "position_sensor_sim_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
        ":position_sensor_sim",
        "//aos/logging",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "position_sensor_sim",
    testonly = True,
    srcs = [
        "position_sensor_sim.cc",
    ],
    hdrs = [
        "position_sensor_sim.h",
    ],
    linkopts = [
        "-lm",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
        ":gaussian_noise",
        "//aos/testing:random_seed",
    ],
)

cc_library(
    name = "gaussian_noise",
    srcs = [
        "gaussian_noise.cc",
    ],
    hdrs = [
        "gaussian_noise.h",
    ],
    linkopts = [
        "-lm",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "coerce_goal",
    srcs = [
        "coerce_goal.cc",
    ],
    hdrs = [
        "coerce_goal.h",
    ],
    linkopts = select({
        "@platforms//os:linux": ["-lm"],
        "//conditions:default": [],
    }),
    deps = [
        "//frc/control_loops:polytope",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "coerce_goal_test",
    srcs = [
        "coerce_goal_test.cc",
    ],
    linkopts = [
        "-lm",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":coerce_goal",
        "//aos/testing:googletest",
        "//frc/control_loops:polytope",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "state_feedback_loop",
    hdrs = [
        "state_feedback_loop.h",
    ],
    deps = [
        "//aos:macros",
        "//frc/zeroing:wrap",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@org_tuxfamily_eigen//:eigen",
    ] + select({
        "@platforms//os:linux": ["//aos/logging"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "hybrid_state_feedback_loop",
    hdrs = [
        "hybrid_state_feedback_loop.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":c2d",
        ":state_feedback_loop",
        "//aos:macros",
        "//aos/logging",
        "//frc/control_loops:control_loop",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "simple_capped_state_feedback_loop",
    hdrs = [
        "simple_capped_state_feedback_loop.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":state_feedback_loop",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "runge_kutta",
    hdrs = [
        "runge_kutta.h",
        "runge_kutta_helpers.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "runge_kutta_test",
    srcs = [
        "runge_kutta_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":runge_kutta",
        "//aos/testing:googletest",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "fixed_quadrature",
    hdrs = [
        "fixed_quadrature.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "fixed_quadrature_test",
    srcs = [
        "fixed_quadrature_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":fixed_quadrature",
        "//aos/testing:googletest",
    ],
)

flatbuffer_ts_library(
    name = "profiled_subsystem_ts_fbs",
    srcs = [
        "profiled_subsystem.fbs",
    ],
    ts_files = [
        "frc/fbs.ts",
        "frc/fbs/storage-order.ts",
        "frc/fbs/matrix.ts",
        "frc/fbs/matrix-field.ts",
        "frc/fbs/field-error.ts",
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
        "frc/control-loops/profiled-joint-status.ts",
        "frc/control-loops/hall-profiled-joint-status.ts",
        "frc/control-loops/pot-and-absolute-encoder-profiled-joint-status.ts",
        "frc/control-loops/index-profiled-joint-status.ts",
        "frc/control-loops/absolute-encoder-profiled-joint-status.ts",
        "frc/control-loops/absolute-and-absolute-encoder-profiled-joint-status.ts",
        "frc/control-loops/relative-encoder-profiled-joint-status.ts",
        "frc/control-loops/static-zeroing-single-dofprofiled-subsystem-goal.ts",
        "frc/control-loops/static-zeroing-single-dofprofiled-subsystem-common-params.ts",
        "frc/control-loops/fbs.ts",
        "frc/control-loops/fbs/state-feedback-plant-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-hybrid-plant-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-controller-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-observer-coefficients.ts",
        "frc/control-loops/fbs/hybrid-kalman-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-loop-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-hybrid-loop-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-loop-coefficients-vector.ts",
        "frc/control-loops/fbs/state-feedback-hybrid-loop-coefficients-vector.ts",
    ],
    deps = [
        ":control_loops_ts_fbs",
        ":state_feedback_loop_ts_fbs",
    ],
)

static_flatbuffer(
    name = "profiled_subsystem_fbs",
    srcs = [
        "profiled_subsystem.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
        ":state_feedback_loop_fbs",
    ],
)

cc_library(
    name = "profiled_subsystem",
    srcs = [
        "profiled_subsystem.cc",
    ],
    hdrs = [
        "profiled_subsystem.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
        ":profiled_subsystem_fbs",
        ":simple_capped_state_feedback_loop",
        ":state_feedback_loop",
        ":state_feedback_loop_converters",
        "//aos:flatbuffer_merge",
        "//aos/util:trapezoid_profile",
        "//frc/control_loops:control_loop",
        "//frc/zeroing",
        "//frc/zeroing:pot_and_index",
    ],
)

cc_library(
    name = "jacobian",
    hdrs = [
        "jacobian.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "jacobian_test",
    srcs = [
        "jacobian_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":jacobian",
        "//aos/testing:googletest",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "c2d_test",
    srcs = [
        "c2d_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":c2d",
        ":runge_kutta",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "c2d",
    hdrs = [
        "c2d.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/time",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "dare",
    hdrs = [
        "dare.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_tartanllama_expected",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "dare_test",
    srcs = [
        "dare_test.cc",
    ],
    data = [
        "//third_party/python:python_runtime",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":dare",
        "//aos/testing:googletest",
        "//third_party/python",
        "@com_google_absl//absl/log",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

py_test(
    name = "dare_python_test",
    srcs = [
        "dare_python_test.py",
    ],
    data = [":cpp_dare.so"],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "@pip//absl_py",
        "@pip//numpy",
        "@pip//scipy",
    ],
)

cc_binary(
    name = "cpp_dare.so",
    srcs = ["dare_python_bindings.cc"],
    linkshared = True,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":dare",
        "//third_party/python",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "dlqr",
    hdrs = [
        "dlqr.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":dare",
        "@com_github_tartanllama_expected",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//frc:python_init"],
)

cc_library(
    name = "binomial",
    hdrs = ["binomial.h"],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_test(
    name = "binomial_test",
    srcs = [
        "binomial_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":binomial",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "capped_test_plant",
    testonly = True,
    srcs = [
        "capped_test_plant.cc",
    ],
    hdrs = [
        "capped_test_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":state_feedback_loop",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "static_zeroing_single_dof_profiled_subsystem",
    hdrs = [
        "static_zeroing_single_dof_profiled_subsystem.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//frc/control_loops:profiled_subsystem",
    ],
)

genrule(
    name = "genrule_static_zeroing_single_dof_profiled_subsystem_test",
    outs = [
        "static_zeroing_single_dof_profiled_subsystem_test_plant.h",
        "static_zeroing_single_dof_profiled_subsystem_test_plant.cc",
        "static_zeroing_single_dof_profiled_subsystem_test_plant.json",
        "static_zeroing_single_dof_profiled_subsystem_test_integral_plant.h",
        "static_zeroing_single_dof_profiled_subsystem_test_integral_plant.cc",
        "static_zeroing_single_dof_profiled_subsystem_test_integral_plant.json",
    ],
    cmd =
        "$(location //frc/control_loops/python:static_zeroing_single_dof_profiled_subsystem_test) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//frc/control_loops/python:static_zeroing_single_dof_profiled_subsystem_test",
    ],
)

genrule(
    name = "genrule_wrapped_subsystem_test",
    outs = [
        "wrapped_subsystem_test_plant.h",
        "wrapped_subsystem_test_plant.cc",
        "wrapped_subsystem_test_plant.json",
        "wrapped_subsystem_test_integral_plant.h",
        "wrapped_subsystem_test_integral_plant.cc",
        "wrapped_subsystem_test_integral_plant.json",
    ],
    cmd =
        "$(location //frc/control_loops/python:wrapped_subsystem_test) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        "//frc/control_loops/python:wrapped_subsystem_test",
    ],
)

cc_library(
    name = "wrapped_subsystem_test_plants",
    srcs = [
        "wrapped_subsystem_test_integral_plant.cc",
        "wrapped_subsystem_test_plant.cc",
    ],
    hdrs = [
        "wrapped_subsystem_test_integral_plant.h",
        "wrapped_subsystem_test_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":state_feedback_loop",
    ],
)

cc_library(
    name = "static_zeroing_single_dof_profiled_subsystem_test_plants",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test_integral_plant.cc",
        "static_zeroing_single_dof_profiled_subsystem_test_plant.cc",
    ],
    hdrs = [
        "static_zeroing_single_dof_profiled_subsystem_test_integral_plant.h",
        "static_zeroing_single_dof_profiled_subsystem_test_plant.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":state_feedback_loop",
    ],
)

static_flatbuffer(
    name = "static_zeroing_single_dof_profiled_subsystem_test_subsystem_goal_fbs",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test_subsystem_goal.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
        ":profiled_subsystem_fbs",
    ],
)

static_flatbuffer(
    name =
        "static_zeroing_single_dof_profiled_subsystem_test_subsystem_output_fbs",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test_subsystem_output.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name =
        "static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_position_fbs",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_position.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
    ],
)

static_flatbuffer(
    name =
        "static_zeroing_single_dof_profiled_subsystem_test_absolute_position_fbs",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test_absolute_position.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
    ],
)

static_flatbuffer(
    name =
        "static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_encoder_status_fbs",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_encoder_status.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
        ":profiled_subsystem_fbs",
    ],
)

static_flatbuffer(
    name =
        "static_zeroing_single_dof_profiled_subsystem_test_absolute_encoder_status_fbs",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test_absolute_encoder_status.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":control_loops_fbs",
        ":profiled_subsystem_fbs",
    ],
)

cc_test(
    name = "static_zeroing_single_dof_profiled_subsystem_test",
    srcs = [
        "static_zeroing_single_dof_profiled_subsystem_test.cc",
    ],
    data = [
        ":static_zeroing_single_dof_profiled_subsystem_test_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":capped_test_plant",
        ":position_sensor_sim",
        ":static_zeroing_single_dof_profiled_subsystem",
        ":static_zeroing_single_dof_profiled_subsystem_test_absolute_encoder_status_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_absolute_position_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_plants",
        ":static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_encoder_status_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_position_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_subsystem_goal_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_subsystem_output_fbs",
        ":wrapped_subsystem_test_plants",
        "//aos/testing:googletest",
        "//frc/control_loops:control_loop_test",
        "//frc/zeroing:absolute_and_absolute_encoder",
        "//frc/zeroing:absolute_encoder",
        "//frc/zeroing:continuous_absolute_encoder",
        "//frc/zeroing:pot_and_absolute_encoder",
    ],
)

aos_config(
    name = "static_zeroing_single_dof_profiled_subsystem_test_config",
    src = "static_zeroing_single_dof_profiled_subsystem_test_config_source.json",
    flatbuffers = [
        "//frc/input:joystick_state_fbs",
        "//frc/input:robot_state_fbs",
        "//aos/logging:log_message_fbs",
        "//aos/events:event_loop_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_subsystem_output_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_absolute_encoder_status_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_subsystem_goal_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_absolute_position_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_encoder_status_fbs",
        ":static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_position_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "state_feedback_loop_fbs",
    srcs = ["state_feedback_loop.fbs"],
    deps = ["//frc/math:matrix_fbs"],
)

flatbuffer_ts_library(
    name = "state_feedback_loop_ts_fbs",
    srcs = ["state_feedback_loop.fbs"],
    ts_files = [
        "frc/control-loops/fbs.ts",
        "frc/control-loops/fbs/state-feedback-plant-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-hybrid-plant-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-controller-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-observer-coefficients.ts",
        "frc/control-loops/fbs/hybrid-kalman-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-loop-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-hybrid-loop-coefficients.ts",
        "frc/control-loops/fbs/state-feedback-loop-coefficients-vector.ts",
        "frc/control-loops/fbs/state-feedback-hybrid-loop-coefficients-vector.ts",
        "frc/fbs.ts",
        "frc/fbs/storage-order.ts",
        "frc/fbs/matrix.ts",
        "frc/fbs/matrix-field.ts",
        "frc/fbs/field-error.ts",
    ],
    deps = ["//frc/math:matrix_ts_fbs"],
)

cc_library(
    name = "hybrid_state_feedback_loop_converters",
    srcs = ["hybrid_state_feedback_loop_converters.h"],
    deps = [
        ":hybrid_state_feedback_loop",
        ":state_feedback_loop_converters",
        ":state_feedback_loop_fbs",
        "//frc/math:flatbuffers_matrix",
    ],
)

cc_library(
    name = "state_feedback_loop_converters",
    srcs = ["state_feedback_loop_converters.h"],
    deps = [
        ":state_feedback_loop",
        ":state_feedback_loop_fbs",
        "//frc/math:flatbuffers_matrix",
    ],
)
