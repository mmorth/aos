load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "generate_compile_command",
    srcs = [
        "generate_compile_command.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//third_party/bazel:extra_actions_proto_py",
    ],
)

action_listener(
    name = "generate_compile_commands_listener",
    extra_actions = [":generate_compile_commands_action"],
    mnemonics = [
        "CppCompile",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

extra_action(
    name = "generate_compile_commands_action",
    cmd = "$(location :generate_compile_command) $(EXTRA_ACTION_FILE)" +
          " $(output $(ACTION_ID)_compile_command)",
    out_templates = [
        "$(ACTION_ID)_compile_command",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        ":generate_compile_command",
    ],
)
