load("@rules_python//python:defs.bzl", "py_binary")

_COMMON_TARGETS = [
    ":aos_demo_config",
    ":variable_size_message_sender",
    "//aos:aos_dump",
    "//aos/network:message_bridge_client",
    "//aos/network:message_bridge_server",
    "//aos/events/logging:logger_main",
    "//aos/starter:starterd",
    "//aos/starter:aos_starter",
    "//aos/util:foxglove_websocket",
]

def py_demo_runner_binary(name, data = [], deps = [], **kwargs):
    py_binary(
        name = name,
        srcs = [
            "runner.py",
        ],
        main = "runner.py",
        data = data + _COMMON_TARGETS,
        deps = deps + [
            "@rules_python//python/runfiles",
        ],
        **kwargs
    )
