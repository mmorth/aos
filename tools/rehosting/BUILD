load("@rules_python//python:defs.bzl", "py_binary", "py_test")

py_test(
    name = "rehost_test",
    srcs = ["rehost_test.py"],
    deps = [":rehost"],
)

py_binary(
    name = "rehost",
    srcs = ["rehost.py"],
    deps = ["@pip//validators"],
)
