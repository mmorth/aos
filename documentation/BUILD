load("@rules_python//python:defs.bzl", "py_binary")

exports_files(["mkdocs_bin.py"])

py_binary(
    name = "mkdocs_bin",
    srcs = ["mkdocs_bin.py"],
    visibility = ["//visibility:public"],
    deps = ["@pip//mkdocs"],
)
