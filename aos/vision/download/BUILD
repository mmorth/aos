load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "downloader",
    srcs = [
        "downloader.py",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
