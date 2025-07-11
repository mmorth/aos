load("//motors:macros.bzl", "hex_from_elf")

cc_binary(
    name = "drivers_station.elf",
    srcs = [
        "drivers_station.cc",
    ],
    target_compatible_with = ["@platforms//os:none"],
    deps = [
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:can",
        "//motors/print:usb",
        "//motors/usb",
        "//motors/usb:cdc",
        "//motors/usb:hid",
        "//motors/usb:interrupt_out",
    ],
)

hex_from_elf(
    name = "drivers_station",
    target_compatible_with = ["@platforms//os:none"],
)

cc_binary(
    name = "controller.elf",
    srcs = [
        "controller.cc",
        "controller_adc.cc",
        "controller_adc.h",
        "vtable_trigger0.cc",
        "vtable_trigger1.cc",
        "vtable_wheel0.cc",
        "vtable_wheel1.cc",
    ],
    target_compatible_with = ["@platforms//os:none"],
    deps = [
        ":motor_controls",
        "//frc/control_loops/drivetrain:haptic_wheel",
        "//motors:motor",
        "//motors:util",
        "//motors/core",
        "//motors/peripheral:adc",
        "//motors/peripheral:can",
        "//motors/print:usb",
    ],
)

hex_from_elf(
    name = "controller",
    target_compatible_with = ["@platforms//os:none"],
)

cc_binary(
    name = "usb_forward_linux",
    srcs = [
        "usb_forward.cc",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        # Don't add anything else here. :usb_forward_windows still has to build it
        # without any other dependencies.
        "@amd64_debian_sysroot//:libusb-1.0",
        "@amd64_debian_sysroot//:libusb",
    ],
)

cc_library(
    name = "motor_controls",
    srcs = [
        "motor_controls.cc",
    ],
    hdrs = [
        "motor_controls.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        "//motors:math",
        "//motors:motor",
        "//motors/peripheral:configuration",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
