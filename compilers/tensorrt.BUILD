load("@bazel_skylib//rules:write_file.bzl", "write_file")

cc_library(
    name = "tensorrt",
    srcs = [
        "targets/x86_64-linux-gnu/lib/libnvinfer.so.10.9.0",
    ],
    hdrs = [
        "include/NvInfer.h",
        "include/NvInferImpl.h",
        "include/NvInferLegacyDims.h",
        "include/NvInferPlugin.h",
        "include/NvInferPluginBase.h",
        "include/NvInferPluginUtils.h",
        "include/NvInferRuntime.h",
        "include/NvInferRuntimeBase.h",
        "include/NvInferRuntimeCommon.h",
        "include/NvInferRuntimePlugin.h",
        "include/NvInferVersion.h",
        "include/NvOnnxConfig.h",
        "include/NvOnnxParser.h",
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
)

sh_binary(
    name = "trtexec",
    srcs = ["trtexec.sh"],
    data = [
        "bin/trtexec",
    ] + glob(["lib/**"]),
)

write_file(
    name = "trtexec_sh",
    out = "trtexec.sh",  # The desired name of the output file
    content = [
        # Content can be a list of strings, they will be joined by newlines
        "#!/bin/bash",
        'LD_LIBRARY_PATH="external/amd64_tensorrt/lib" external/amd64_tensorrt/bin/trtexec "$@"',
    ],
    # You can optionally make it executable
    is_executable = True,
)
