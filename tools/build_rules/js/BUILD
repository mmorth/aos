load("@aspect_bazel_lib//lib:copy_file.bzl", "copy_file")
load("@npm//:@angular/compiler-cli/package_json.bzl", angular_compiler_cli = "bin")
load("@rules_python//python:defs.bzl", "py_binary")
load(":ts.bzl", "ts_project")

exports_files([
    "cypress.config.js",
])

# Define the @angular/compiler-cli ngc bin binary as a target
angular_compiler_cli.ngc_binary(
    name = "ngc",
    visibility = ["//visibility:public"],
)

# ESBuild plugin to run the Angular linker
ts_project(
    name = "ngc.esbuild",
    srcs = ["ngc.esbuild.ts"],
    tsconfig = "//:tsconfig.node",
    deps = [
        "//:node_modules/@angular/compiler-cli",
        "//:node_modules/@babel/core",
        #"//:node_modules/@types/babel__core",
        "//:node_modules/@types/node",
    ],
)

copy_file(
    name = "ngc_esbuild_js_copy",
    src = ":ngc.esbuild.js",
    out = "ngc.esbuild.mjs",
    visibility = ["//visibility:public"],
)

py_binary(
    name = "assemble_service_worker_files",
    srcs = [
        "assemble_service_worker_files.py",
    ],
    visibility = ["//visibility:public"],
)
