load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "scouting_test_servers",
    testonly = True,
    srcs = [
        "scouting_test_servers.py",
    ],
    data = [
        "//scouting",
        "//scouting/db/testdb_server",
        "//scouting/scraping:test_data",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@rules_python//python/runfiles",
    ],
)
