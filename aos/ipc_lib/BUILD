cc_library(
    name = "aos_sync",
    srcs = [
        "aos_sync.cc",
    ],
    hdrs = [
        "aos_sync.h",
    ],
    linkopts = [
        "-lpthread",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":shm_observers",
        "//aos:macros",
        "//aos/util:compiler_memory_barrier",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_library(
    name = "core_lib",
    srcs = [
        "core_lib.c",
    ],
    hdrs = [
        "core_lib.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_sync",
        ":shared_mem_types",
    ],
)

cc_library(
    name = "shared_mem",
    srcs = [
        "shared_mem.cc",
    ],
    hdrs = [
        "shared_mem.h",
    ],
    linkopts = [
        "-lrt",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_sync",
        ":core_lib",
        ":shared_mem_types",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    # TODO(Brian): This should be shared_mem{,.h}, and the other one should be
    # shared_mem_init{,.cc,.h}.
    name = "shared_mem_types",
    hdrs = [
        "shared_mem_types.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":aos_sync",
    ],
)

cc_test(
    name = "ipc_stress_test",
    srcs = [
        "ipc_stress_test.cc",
    ],
    tags = [
        "manual",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":core_lib",
        "//aos:die",
        "//aos/libc:aos_strsignal",
        "//aos/libc:dirname",
        "//aos/logging",
        "//aos/mutex",
        "//aos/testing:googletest",
        "//aos/testing:test_shm",
        "//aos/time",
    ],
)

cc_binary(
    name = "ipc_comparison",
    srcs = [
        "ipc_comparison.cc",
    ],
    linkopts = ["-lrt"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event",
        "//aos:condition",
        "//aos:init",
        "//aos/logging",
        "//aos/mutex",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "signalfd",
    srcs = [
        "signalfd.cc",
    ],
    hdrs = [
        "signalfd.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos:sanitizers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "signalfd_test",
    srcs = [
        "signalfd_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":signalfd",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "index32",
    srcs = ["index.cc"],
    hdrs = ["index.h"],
    defines = [
        "AOS_QUEUE_ATOMIC_SIZE=32",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":shm_observers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "index32_test",
    srcs = ["index_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":index32",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "index",
    srcs = ["index.cc"],
    hdrs = ["index.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":shm_observers",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
    ],
)

cc_test(
    name = "index_test",
    srcs = ["index_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":index",
        "//aos/testing:googletest",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_library(
    name = "lockless_queue",
    srcs = [
        "lockless_queue.cc",
        "lockless_queue_memory.h",
        "memory_mapped_queue.cc",
        "robust_ownership_tracker.cc",
    ],
    hdrs = [
        "lockless_queue.h",
        "memory_mapped_queue.h",
        "robust_ownership_tracker.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":aos_sync",
        ":data_alignment",
        ":index",
        "//aos:configuration",
        "//aos:realtime",
        "//aos:uuid",
        "//aos/events:context",
        "//aos/time",
        "//aos/util:compiler_memory_barrier",
        "//aos/util:top",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "queue_racer",
    testonly = True,
    srcs = [
        "queue_racer.cc",
    ],
    hdrs = [
        "queue_racer.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event",
        ":lockless_queue",
        "//aos/testing:googletest",
    ],
)

cc_library(
    name = "lockless_queue_stepping",
    testonly = True,
    srcs = [
        "lockless_queue_stepping.cc",
    ],
    hdrs = ["lockless_queue_stepping.h"],
    deps = [
        ":lockless_queue",
        ":shm_observers",
        "//aos/libc:aos_strsignal",
        "//aos/testing:prevent_exit",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "robust_ownership_tracker_test",
    srcs = ["robust_ownership_tracker_test.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":lockless_queue",
        "//aos/testing:googletest",
    ],
)

cc_test(
    name = "lockless_queue_test",
    timeout = "eternal",
    srcs = ["lockless_queue_test.cc"],
    # We don't want all the signal handlers registered, otherwise we can't fork.
    args = ["--nobacktrace"],
    shard_count = 10,
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event",
        ":lockless_queue",
        ":lockless_queue_stepping",
        ":queue_racer",
        ":signalfd",
        "//aos/events:epoll",
        "//aos/testing:googletest",
        "//aos/util:phased_loop",
    ],
)

cc_test(
    name = "lockless_queue_death_test",
    srcs = ["lockless_queue_death_test.cc"],
    # We don't want all the signal handlers registered, otherwise we can't fork.
    args = ["--nobacktrace"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event",
        ":lockless_queue",
        ":lockless_queue_stepping",
        ":queue_racer",
        ":shm_observers",
        ":signalfd",
        "//aos/events:epoll",
        "//aos/libc:aos_strsignal",
        "//aos/testing:googletest",
        "//aos/testing:prevent_exit",
        "//aos/testing:test_logging",
    ],
)

cc_library(
    name = "data_alignment",
    hdrs = [
        "data_alignment.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/log:absl_check",
    ],
)

cc_library(
    name = "latency_lib",
    srcs = ["latency_lib.cc"],
    hdrs = ["latency_lib.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:realtime",
        "//aos/logging",
        "//aos/time",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_binary(
    name = "signal_stress",
    srcs = [
        "signal_stress.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":latency_lib",
        "//aos:init",
        "//aos/events:epoll",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "futex_latency",
    srcs = [
        "futex_latency.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":latency_lib",
        "//aos:condition",
        "//aos:init",
        "//aos/mutex",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "named_pipe_latency",
    srcs = [
        "named_pipe_latency.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":latency_lib",
        "//aos:init",
        "//aos/events:epoll",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "eventfd_latency",
    srcs = [
        "eventfd_latency.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":latency_lib",
        "//aos:init",
        "//aos/events:epoll",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_binary(
    name = "print_lockless_queue_memory",
    srcs = [
        "print_lockless_queue_memory.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":lockless_queue",
    ],
)

cc_library(
    name = "event",
    srcs = [
        "event.cc",
    ],
    hdrs = [
        "event.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/ipc_lib:aos_sync",
        "//aos/time",
        "//aos/type_traits",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)

cc_test(
    name = "event_test",
    srcs = [
        "event_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":event",
        "//aos/testing:googletest",
        "//aos/testing:test_logging",
        "//aos/time",
    ],
)

cc_library(
    name = "shm_observers",
    srcs = [
        "shm_observers.cc",
    ],
    hdrs = [
        "shm_observers.h",
    ],
)

cc_library(
    name = "shm_base",
    srcs = ["shm_base.cc"],
    hdrs = ["shm_base.h"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_library(
    name = "memory_estimation",
    srcs = ["memory_estimation.cc"],
    hdrs = ["memory_estimation.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lockless_queue",
        "//aos:configuration",
    ],
)
