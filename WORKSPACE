workspace(name = "aos")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/jdk:remote_java_repository.bzl", "remote_java_repository")
load("//tools/ci:repo_defs.bzl", "ci_configure")

ci_configure(name = "ci_configure")

load("@ci_configure//:ci.bzl", "RUNNING_IN_CI")

http_archive(
    name = "platforms",
    sha256 = "29742e87275809b5e598dc2f04d86960cc7a55b3067d97221c9abbc9926bff0f",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/platforms/releases/download/0.0.11/platforms-0.0.11.tar.gz",
        "https://github.com/bazelbuild/platforms/releases/download/0.0.11/platforms-0.0.11.tar.gz",
    ],
)

http_archive(
    name = "bazel_skylib",
    sha256 = "bc283cdfcd526a52c3201279cda4bc298652efa898b10b4db0837dc51652756f",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.7.1/bazel-skylib-1.7.1.tar.gz",
        "https://github.com/bazelbuild/bazel-skylib/releases/download/1.7.1/bazel-skylib-1.7.1.tar.gz",
    ],
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

http_archive(
    name = "aspect_bazel_lib",
    sha256 = "40ba9d0f62deac87195723f0f891a9803a7b720d7b89206981ca5570ef9df15b",
    strip_prefix = "bazel-lib-2.14.0",
    url = "https://github.com/bazel-contrib/bazel-lib/releases/download/v2.14.0/bazel-lib-v2.14.0.tar.gz",
)

load("@aspect_bazel_lib//lib:repositories.bzl", "aspect_bazel_lib_dependencies", "aspect_bazel_lib_register_toolchains", "register_jq_toolchains")

aspect_bazel_lib_dependencies()

aspect_bazel_lib_register_toolchains()

register_jq_toolchains()

http_archive(
    name = "rules_python",
    patch_args = ["-p1"],
    patches = [
        "//third_party:rules_python/0001-Support-overriding-individual-packages.patch",
        "//third_party:rules_python/0002-Allow-user-to-patch-wheels.patch",
    ],
    sha256 = "497ca47374f48c8b067d786b512ac10a276211810f4a580178ee9b9ad139323a",
    strip_prefix = "rules_python-0.16.1",
    url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.16.1.tar.gz",
)

load("@rules_python//python:repositories.bzl", "python_register_toolchains")

python_register_toolchains(
    name = "python3_9",
    python_version = "3.9",
    register_toolchains = False,
)

load(
    "@python3_9//:defs.bzl",
    python_interpreter = "interpreter",
)
load("@rules_python//python:pip.bzl", "pip_parse")
load(
    "//tools/python:package_annotations.bzl",
    PYTHON_ANNOTATIONS = "ANNOTATIONS",
)

pip_parse(
    name = "pip_deps",
    annotations = PYTHON_ANNOTATIONS,
    enable_implicit_namespace_pkgs = True,
    overrides = "//tools/python:whl_overrides.json",
    patch_spec = "//tools/python:patches.json",
    python_interpreter_target = python_interpreter,
    require_overrides = RUNNING_IN_CI,
    requirements_lock = "//tools/python:requirements.lock.txt",
)

# Load the starlark macro which will define your dependencies.
load(
    "@pip_deps//:requirements.bzl",
    install_pip_deps = "install_deps",
)

install_pip_deps()

load("//tools/python:repo_defs.bzl", "pip_configure")

pip_configure(
    name = "pip",
)

http_archive(
    name = "rules_pkg",
    sha256 = "cad05f864a32799f6f9022891de91ac78f30e0fa07dc68abac92a628121b5b11",
    urls = [
        "https://github.com/bazelbuild/rules_pkg/releases/download/1.0.0/rules_pkg-1.0.0.tar.gz",
    ],
)

load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")

rules_pkg_dependencies()

load("//debian:packages.bzl", "generate_repositories_for_debs")
load(
    "//debian:phoenix6.bzl",
    phoenix6_debs = "files",
)

generate_repositories_for_debs(phoenix6_debs)

local_repository(
    name = "com_grail_bazel_toolchain",
    path = "third_party/bazel-toolchain",
)

local_repository(
    name = "com_github_wpilibsuite_allwpilib",
    path = "third_party/allwpilib",
)

# Download toolchains
http_archive(
    name = "rules_bzlmodrio_toolchains",
    patch_args = ["-p1"],
    patches = [
        "//third_party/rules_bzlmodrio_toolchains:0001-Make-platform-constraints-more-flexible.patch",
    ],
    sha256 = "ff25b5f9445cbd43759be4c6582b987d1065cf817c593eedc7ada1a699298c84",
    url = "https://github.com/wpilibsuite/rules_bzlmodRio_toolchains/releases/download/2025-1.bcr2/rules_bzlmodRio_toolchains-2025-1.bcr2.tar.gz",
)

http_archive(
    name = "bzlmodrio-ni",
    sha256 = "fff62c3cb3e83f9a0d0a01f1739477c9ca5e9a6fac05be1ad59dafcd385801f7",
    url = "https://github.com/wpilibsuite/bzlmodRio-ni/releases/download/2025.2.0/bzlmodRio-ni-2025.2.0.tar.gz",
)

load("@bzlmodrio-ni//:maven_cpp_deps.bzl", "setup_legacy_bzlmodrio_ni_cpp_dependencies")

setup_legacy_bzlmodrio_ni_cpp_dependencies()

http_archive(
    name = "RangeHTTPServer",
    sha256 = "98a8e4980f91d048dc9159cfc5f115280d0b5ec59a5b01df0422b887212fa4f0",
    strip_prefix = "RangeHTTPServer-9070394508a135789238a33259793f3c6f3c127a",
    url = "https://github.com/jkuszmaul/RangeHTTPServer/archive/9070394508a135789238a33259793f3c6f3c127a.zip",
)

load("@com_grail_bazel_toolchain//toolchain:rules.bzl", "llvm", "llvm_toolchain")

llvm_version = "19.1.7"

llvm(
    name = "llvm_k8",
    distribution = "clang+llvm-%s-x86_64-linux-gnu.tar.zst" % llvm_version,
    llvm_version = llvm_version,
)

llvm(
    name = "llvm_aarch64",
    distribution = "clang+llvm-%s-aarch64-linux-gnu-ubuntu-22.04.tar.zst" % llvm_version,
    llvm_version = llvm_version,
)

llvm_conlyopts = [
    "-std=gnu99",
]

llvm_copts = [
    "-D__STDC_FORMAT_MACROS",
    "-D__STDC_CONSTANT_MACROS",
    "-D__STDC_LIMIT_MACROS",
    "-D_FILE_OFFSET_BITS=64",
    "-fmessage-length=100",
    "-fmacro-backtrace-limit=0",
    "-ggdb3",
    # Too many core libraries have these right now.
    # TODO(austin): Turn this off later.
    "-Wno-deprecated-declarations",
]

llvm_cxxopts = [
    "-std=gnu++20",
]

llvm_opt_copts = [
    "-DAOS_DEBUG=0",
]

llvm_fastbuild_copts = [
    "-DAOS_DEBUG=0",
]

llvm_dbg_copts = [
    "-DAOS_DEBUG=1",
]

llvm_toolchain(
    name = "llvm_toolchain",
    additional_target_compatible_with = {},
    conlyopts = {
        "linux-x86_64": llvm_conlyopts,
        "linux-aarch64": llvm_conlyopts,
    },
    copts = {
        "linux-x86_64": llvm_copts,
        "linux-aarch64": llvm_copts,
    },
    cxxopts = {
        "linux-x86_64": llvm_cxxopts,
        "linux-aarch64": llvm_cxxopts,
    },
    dbg_copts = {
        "linux-x86_64": llvm_dbg_copts,
        "linux-aarch64": llvm_dbg_copts,
    },
    fastbuild_copts = {
        "linux-x86_64": llvm_fastbuild_copts,
        "linux-aarch64": llvm_fastbuild_copts,
    },
    llvm_version = llvm_version,
    opt_copts = {
        "linux-x86_64": llvm_opt_copts,
        "linux-aarch64": llvm_opt_copts,
    },
    standard_libraries = {
        "linux-x86_64": "libstdc++-12",
        "linux-aarch64": "libstdc++-14.2.0",
    },
    static_libstdcxx = False,
    sysroot = {
        "linux-x86_64": "@amd64_debian_sysroot//:sysroot_files",
        "linux-aarch64": "@arm64_debian_sysroot//:sysroot_files",
    },
    target_toolchain_roots = {
        "linux-x86_64": "@llvm_k8//",
        "linux-aarch64": "@llvm_aarch64//",
    },
    toolchain_roots = {
        "linux-x86_64": "@llvm_k8//",
        "linux-aarch64": "@llvm_aarch64//",
    },
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")

llvm_register_toolchains()

register_toolchains(
    "//tools/cpp:cc-toolchain-roborio",
    "//tools/cpp:cc-toolchain-cortex-m4f",
    "//tools/cpp:cc-toolchain-rp2040",
    "//tools/cpp:cc-toolchain-cortex-m4f-imu",
    # Find a good way to select between these two M4F toolchains.
    #"//tools/cpp:cc-toolchain-cortex-m4f-k22",
    "//tools/python:python_toolchain",
    "//tools/go:noop_go_toolchain",
    "//tools/rust:rust-toolchain-x86",
    "//tools/rust:rust-toolchain-armv7",
    "//tools/rust:rust-toolchain-arm64",
    # TODO(Brian): Make this work. See the comment on
    # //tools/platforms:linux_roborio for details.
    #"//tools/rust:rust-toolchain-roborio",
    "//tools/rust:noop_rust_toolchain",
    "//tools/ts:noop_node_toolchain",
)

http_archive(
    name = "com_google_absl",
    patch_args = ["-p1"],
    patches = [
        "//third_party/abseil:0001-Add-hooks-for-using-abseil-with-AOS.patch",
        "//third_party/abseil:0002-Suppress-the-stack-trace-on-SIGABRT.patch",
    ],
    sha256 = "733726b8c3a6d39a4120d7e45ea8b41a434cdacde401cba500f14236c49b39dc",
    strip_prefix = "abseil-cpp-20240116.2",
    url = "https://github.com/abseil/abseil-cpp/archive/refs/tags/20240116.2.tar.gz",
)

local_repository(
    name = "org_tuxfamily_eigen",
    path = "third_party/eigen",
)

local_repository(
    name = "com_github_rawrtc_re",
    path = "third_party/rawrtc/re",
)

local_repository(
    name = "com_github_rawrtc_rew",
    path = "third_party/rawrtc/rew",
)

local_repository(
    name = "com_github_rawrtc_usrsctp",
    path = "third_party/rawrtc/usrsctp",
)

local_repository(
    name = "com_github_rawrtc_rawrtc_common",
    path = "third_party/rawrtc/rawrtc-common",
)

local_repository(
    name = "com_github_rawrtc_rawrtc_data_channel",
    path = "third_party/rawrtc/rawrtc-data-channel",
)

local_repository(
    name = "com_github_rawrtc_rawrtc",
    path = "third_party/rawrtc/rawrtc",
)

# C++ rules for Bazel.
http_archive(
    name = "rules_cc",
    sha256 = "d75a040c32954da0d308d3f2ea2ba735490f49b3a7aa3e4b40259ca4b814f825",
    urls = ["https://github.com/bazelbuild/rules_cc/releases/download/0.0.10-rc1/rules_cc-0.0.10-rc1.tar.gz"],
)

new_local_repository(
    name = "usr_repo",
    build_file = "@//debian:usr.BUILD",
    path = "/usr",
)

# TODO(Ravago, Max, Alex): https://github.com/wpilibsuite/opensdk
http_archive(
    name = "arm_frc_linux_gnueabi_repo",
    build_file = "@//tools/cpp/arm-frc-linux-gnueabi:arm-frc-linux-gnueabi.BUILD",
    patches = ["//debian:fts.patch"],
    sha256 = "0ed7c454eab947166c41177f72f795a408f4b7c5fc0f7fd6ee35dba1871fbfac",
    strip_prefix = "roborio-academic",
    url = "https://github.com/wpilibsuite/opensdk/releases/download/v2025-1/cortexa9_vfpv3-roborio-academic-2025-x86_64-linux-gnu-Toolchain-12.1.0.tgz",
)

# The main partition packaged with //compilers/buildify_yocto_image.py
# Packaging the yocto image built from https://github.com/frc4646/meta-frc4646
# To rebuild, follow the instructions in meta-frc4646, then, cd compilers and
# run buildify_yocto_image.py /path/to/git/checkout/meta-frc4646
http_archive(
    name = "arm64_debian_sysroot",
    build_file = "@//:compilers/orin_debian_rootfs.BUILD",
    sha256 = "d1eeb1224a726cc9a8bb0eb55171872edea90bb0564f639b5c310b97d5cc7001",
    url = "https://realtimeroboticsgroup.org/build-dependencies/2025-04-06-walnascar-arm64-nvidia-rootfs.tar.zst",
)

# Sysroot generated using //frc/amd64/build_rootfs.py
http_archive(
    name = "amd64_debian_sysroot",
    build_file = "@//:compilers/amd64_debian_rootfs.BUILD",
    sha256 = "e94dec03e19d88cd428964f1e4a430e6bc4a2dd2f4f7342f56b75efa9c75a761",
    url = "https://realtimeroboticsgroup.org/build-dependencies/2025-04-20-bookworm-amd64-nvidia-rootfs.tar.zst",
)

# Originally from: https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.9.0/tars/TensorRT-10.9.0.34.Linux.x86_64-gnu.cuda-11.8.tar.gz
# Recompressed for faster extraction.
http_archive(
    name = "amd64_tensorrt",
    build_file = "@//:compilers/tensorrt.BUILD",
    sha256 = "5b5b828be725077d13a23d296a5ae56ce42f785ad8258dc79d34f326b34cc783",
    strip_prefix = "TensorRT-10.9.0.34",
    url = "https://realtimeroboticsgroup.org/build-dependencies/developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.9.0/tars/TensorRT-10.9.0.34.Linux.x86_64-gnu.cuda-11.8.tar.zst",
)

http_archive(
    name = "april_tag_test_image",
    build_file_content = """
filegroup(
    name = "april_tag_test_image",
    srcs = ["test.bfbs", "expected.jpeg", "expected.png"],
    visibility = ["//visibility:public"],
)""",
    sha256 = "5312c79b19e9883b3cebd9d65b4438a2bf05b41da0bcd8c35e19d22c3b2e1859",
    urls = ["https://realtimeroboticsgroup.org/build-dependencies/test_image_frc971.vision.CameraImage_2023.01.28.tar.gz"],
)

http_archive(
    name = "ffmpeg",
    build_file_content = """
load("@bazel_skylib//rules:native_binary.bzl", "native_binary")

native_binary(
  name = "ffmpeg",
  src = "ffmpeg-6.0.1-amd64-static/ffmpeg",
  out = "ffmpeg",
  visibility = ["//visibility:public"],
  target_compatible_with = ["@platforms//cpu:x86_64", "@platforms//os:linux"],
)
    """,
    sha256 = "28268bf402f1083833ea269331587f60a242848880073be8016501d864bd07a5",
    url = "https://www.johnvansickle.com/ffmpeg/old-releases/ffmpeg-6.0.1-amd64-static.tar.xz",
)

# Downloaded from
# From https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
http_archive(
    name = "gcc_arm_none_eabi",
    build_file = "@//:compilers/gcc_arm_none_eabi.BUILD",
    sha256 = "6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb",
    strip_prefix = "arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi",
    url = "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz",
)

# Java11 JDK.
remote_java_repository(
    name = "openjdk_linux_archive",
    prefix = "openjdk",
    sha256 = "60e65d32e38876f81ddb623e87ac26c820465b637e263e8bed1acdecb4ca9be2",
    strip_prefix = "zulu11.54.25-ca-jdk11.0.14.1-linux_x64",
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    urls = [
        "https://realtimeroboticsgroup.org/build-dependencies/zulu11.54.25-ca-jdk11.0.14.1-linux_x64.tar.gz",
    ],
    version = "11",
)

remote_java_repository(
    name = "openjdk_linux_archive_aarch64",
    prefix = "openjdk",
    sha256 = "b0fb0bc303bb05b5042ef3d0939b9489f4a49a13a2d1c8f03c5d8ab23099454d",
    strip_prefix = "zulu11.54.25-ca-jdk11.0.14.1-linux_aarch64",
    target_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    urls = [
        "https://realtimeroboticsgroup.org/build-dependencies/zulu11.54.25-ca-jdk11.0.14.1-linux_aarch64.tar.gz",
    ],
    version = "11",
)

http_archive(
    name = "com_google_googletest",
    patch_args = ["-p1"],
    patches = ["//third_party/googletest:googletest.patch"],
    sha256 = "8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7",
    strip_prefix = "googletest-1.14.0",
    urls = ["https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz"],
)

http_archive(
    name = "com_github_google_benchmark",
    patch_args = ["-p1"],
    patches = ["//third_party/google-benchmark:benchmark.patch"],
    sha256 = "3e7059b6b11fb1bbe28e33e02519398ca94c1818874ebed18e504dc6f709be45",
    strip_prefix = "benchmark-1.8.4",
    urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.8.4.tar.gz"],
)

http_archive(
    name = "com_google_ceres_solver",
    patch_args = ["-p1"],
    patches = ["//third_party:ceres.patch"],
    sha256 = "5fef6cd0ed744a09e20d1c341a15b0f94ed0c8df43537e198a869e6c242c99d5",
    strip_prefix = "ceres-solver-bd323ce698748bef0686eb27cb6cea4f88bb4f44",
    urls = ["https://github.com/ceres-solver/ceres-solver/archive/bd323ce698748bef0686eb27cb6cea4f88bb4f44.zip"],
)

http_archive(
    name = "ctre_phoenix6_api_cpp_headers",
    build_file_content = """
cc_library(
    name = 'api-cpp',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/phoenix6/**/*.hpp', 'ctre/unit/**/*.h']),
    includes = ["."],
    deps = [
        "@com_github_wpilibsuite_allwpilib//wpimath:wpimath.static",
        "@ctre_phoenix6_tools_headers//:tools",
    ],
)
""",
    sha256 = "67fdd9a4bf275c69666bbc8bf38312eea8a85bf9fda3901d02af3a18136ffb3e",
    urls = [
        "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/api-cpp/24.50.0-alpha-2/api-cpp-24.50.0-alpha-2-headers.zip",
    ],
)

http_archive(
    name = "ctre_phoenix6_api_cpp_athena",
    build_file_content = """
filegroup(
    name = 'shared_libraries',
    srcs = [
        'linux/athena/shared/libCTRE_Phoenix6.so',
    ],
    visibility = ['//visibility:public'],
)

cc_library(
    name = 'api-cpp',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/shared/libCTRE_Phoenix6.so'],
    target_compatible_with = ['@//tools/platforms/hardware:roborio'],
)
""",
    sha256 = "00da14f437cfeb2c344674dee59fe70477a3a0d612eb93a1441188dc94a5136f",
    urls = [
        "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/api-cpp/24.50.0-alpha-2/api-cpp-24.50.0-alpha-2-linuxathena.zip",
    ],
)

http_archive(
    name = "ctre_phoenix6_tools_headers",
    build_file_content = """
cc_library(
    name = 'tools',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/**/*.h', 'ctre/phoenix/**/*.hpp', 'ctre/phoenix6/**/*.hpp']),
)
""",
    sha256 = "77624291ec19a03c9e068347ef800e435782444722793d56c9e39f6108da33a8",
    urls = [
        "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/tools/24.50.0-alpha-2/tools-24.50.0-alpha-2-headers.zip",
    ],
)

http_archive(
    name = "ctre_phoenix6_tools_athena",
    build_file_content = """
filegroup(
    name = 'shared_libraries',
    srcs = [
        'linux/athena/shared/libCTRE_PhoenixTools.so',
    ],
    visibility = ['//visibility:public'],
)

cc_library(
    name = 'tools',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/shared/libCTRE_PhoenixTools.so'],
    target_compatible_with = ['@//tools/platforms/hardware:roborio'],
)
""",
    sha256 = "217bcd6aecb71224fd32725f857da58e61a6ea451ca07f85fb55e49f83dbf113",
    urls = [
        "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/tools/24.50.0-alpha-2/tools-24.50.0-alpha-2-linuxathena.zip",
    ],
)

http_archive(
    name = "ctre_phoenix_api_cpp_headers",
    build_file_content = """
cc_library(
    name = 'api-cpp',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/phoenix/**/*.h', 'ctre/unit/**/*.h']),
    includes = ["."]
)
""",
    sha256 = "4fba20441ea1d61f8487897682c723a48ee2c2741946eab4062b4248434c0afc",
    urls = [
        "https://maven.ctr-electronics.com/release/com/ctre/phoenix/api-cpp/5.34.0-alpha-1/api-cpp-5.34.0-alpha-1-headers.zip",
    ],
)

http_archive(
    name = "ctre_phoenix_api_cpp_athena",
    build_file_content = """
filegroup(
    name = 'shared_libraries',
    srcs = [
        'linux/athena/shared/libCTRE_Phoenix.so',
    ],
    visibility = ['//visibility:public'],
)

cc_library(
    name = 'api-cpp',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/shared/libCTRE_Phoenix.so'],
    target_compatible_with = ['@//tools/platforms/hardware:roborio'],
)
""",
    sha256 = "e67348b0db1e61797700b5c4e50b38b745f91899665b740b09fde879e4d8ad7a",
    urls = [
        "https://maven.ctr-electronics.com/release/com/ctre/phoenix/api-cpp/5.33.0/api-cpp-5.33.0-linuxathena.zip",
    ],
)

http_archive(
    name = "ctre_phoenix_cci_headers",
    build_file_content = """
cc_library(
    name = 'cci',
    visibility = ['//visibility:public'],
    hdrs = glob(['ctre/phoenix/**/*.h']),
)
""",
    sha256 = "cd39f32341037a7ec1074710044b332db6ca607dfd548b5f951c75f7a8506ab5",
    urls = [
        "https://maven.ctr-electronics.com/release/com/ctre/phoenix/cci/5.34.0-alpha-1/cci-5.34.0-alpha-1-headers.zip",
    ],
)

http_archive(
    name = "ctre_phoenix_cci_athena",
    build_file_content = """
filegroup(
    name = 'shared_libraries',
    srcs = [
        'linux/athena/shared/libCTRE_PhoenixCCI.so',
    ],
    visibility = ['//visibility:public'],
)

cc_library(
    name = 'cci',
    visibility = ['//visibility:public'],
    srcs = ['linux/athena/shared/libCTRE_PhoenixCCI.so'],
    target_compatible_with = ['@//tools/platforms/hardware:roborio'],
)
""",
    sha256 = "7bd8e9950c3e62a3c6ce967e3491c438ba08effde51d434f85b756fa004fbec9",
    urls = [
        "https://maven.ctr-electronics.com/release/com/ctre/phoenix/cci/5.33.0/cci-5.33.0-linuxathena.zip",
    ],
)

http_archive(
    name = "ctre_phoenix6_arm64",
    build_file_content = """
filegroup(
    name = 'shared_libraries',
    srcs = [
        'usr/lib/phoenix6/libCTRE_PhoenixTools.so',
        'usr/lib/phoenix6/libCTRE_Phoenix6.so',
    ],
    visibility = ['//visibility:public'],
    target_compatible_with = ['@platforms//cpu:arm64'],
)


# TODO(max): Use cc_import once they add a defines property.
# See: https://github.com/bazelbuild/bazel/issues/19753
cc_library(
    name = "shared_libraries_lib",
    visibility = ['//visibility:public'],
    srcs = [
        'usr/lib/phoenix6/libCTRE_PhoenixTools.so',
        'usr/lib/phoenix6/libCTRE_Phoenix6.so',
    ],
    target_compatible_with = ['@platforms//cpu:arm64'],
)

cc_library(
    name = 'headers',
    visibility = ['//visibility:public'],
    hdrs = glob(['usr/include/phoenix6/**/*.hpp', 'usr/include/phoenix6/**/*.h']),
    includes = ["usr/include/phoenix6/"],
    target_compatible_with = ['@platforms//cpu:arm64'],
    defines = [
        "UNIT_LIB_DISABLE_FMT",
        "UNIT_LIB_ENABLE_IOSTREAM"
    ],
)
""",
    sha256 = "0f1312f39eacc490fb253198c2d0e61e48ae00eff6a87cfd362358b1ad36a930",
    urls = [
        "https://realtimeroboticsgroup.org/build-dependencies/phoenix6_24.50.0-alpha-2_arm64-2024.10.26.tar.gz",
    ],
)

http_archive(
    name = "aspect_rules_js",
    patch_args = [
        "-p1",
    ],
    patches = [
        "//third_party:rules_js/0001-Fix-package-permissions.patch",
    ],
    sha256 = "bc9b4a01ef8eb050d8a7a050eedde8ffb1e45a56b0e4094e26f06c17d5fcf1d5",
    strip_prefix = "rules_js-1.41.2",
    url = "https://github.com/aspect-build/rules_js/releases/download/v1.41.2/rules_js-v1.41.2.tar.gz",
)

load("@aspect_rules_js//js:repositories.bzl", "rules_js_dependencies")

rules_js_dependencies()

load("@bazel_features//:deps.bzl", "bazel_features_deps")

bazel_features_deps()

load("@aspect_rules_js//npm:npm_import.bzl", "npm_translate_lock", "pnpm_repository")

pnpm_repository(name = "pnpm")

http_archive(
    name = "aspect_rules_esbuild",
    sha256 = "999349afef62875301f45ec8515189ceaf2e85b1e67a17e2d28b95b30e1d6c0b",
    strip_prefix = "rules_esbuild-0.18.0",
    url = "https://github.com/aspect-build/rules_esbuild/releases/download/v0.18.0/rules_esbuild-v0.18.0.tar.gz",
)

load("@aspect_rules_esbuild//esbuild:dependencies.bzl", "rules_esbuild_dependencies")

rules_esbuild_dependencies()

load("@rules_nodejs//nodejs:repositories.bzl", "DEFAULT_NODE_VERSION", "nodejs_register_toolchains")

nodejs_register_toolchains(
    name = "nodejs",
    node_version = DEFAULT_NODE_VERSION,
)

npm_translate_lock(
    name = "npm",
    data = [
        "//aos/analysis/foxglove_extension:package.json",
        "//control_loops/swerve/spline_ui/www:package.json",
        "@//:package.json",
        "@//:pnpm-workspace.yaml",
        "@//scouting/webserver/requests/messages:package.json",
        "@//scouting/www:package.json",
        "@//scouting/www/driver_ranking:package.json",
        "@//scouting/www/entry:package.json",
        "@//scouting/www/match_list:package.json",
        "@//scouting/www/notes:package.json",
        "@//scouting/www/pipes:package.json",
        "@//scouting/www/rpc:package.json",
        "@//scouting/www/scan:package.json",
        "@//scouting/www/shift_schedule:package.json",
        "@//scouting/www/test/authorize:package.json",
        "@//scouting/www/view:package.json",
    ],

    # Running lifecycle hooks on npm package fsevents@2.3.2 fails in a dramatic way:
    # ```
    # SyntaxError: Unexpected strict mode reserved word
    # at ESMLoader.moduleStrategy (node:internal/modules/esm/translators:117:18)
    # at ESMLoader.moduleProvider (node:internal/modules/esm/loader:337:14)
    # at async link (node:internal/modules/esm/module_job:70:21)
    # ```
    lifecycle_hooks_no_sandbox = False,
    npmrc = "@//:.npmrc",
    pnpm_lock = "//:pnpm-lock.yaml",
    quiet = False,
    update_pnpm_lock = False,
    verify_node_modules_ignored = "//:.bazelignore",
)

load("@aspect_rules_esbuild//esbuild:repositories.bzl", "LATEST_ESBUILD_VERSION", "esbuild_register_toolchains")

esbuild_register_toolchains(
    name = "esbuild",
    esbuild_version = LATEST_ESBUILD_VERSION,
)

http_archive(
    name = "aspect_rules_rollup",
    patch_args = [
        "-p1",
    ],
    patches = [
        "//third_party:rules_rollup/0001-Fix-resolving-files.patch",
    ],
    sha256 = "a0433a0b0206a45d362749d71bc1e4e0dacf5ca2a572b059328f9753392bca80",
    strip_prefix = "rules_rollup-1.0.0",
    url = "https://github.com/aspect-build/rules_rollup/releases/download/v1.0.0/rules_rollup-v1.0.0.tar.gz",
)

http_archive(
    name = "aspect_rules_terser",
    sha256 = "8424b4c064d0e490e5b6f215b993712ef641b77e03b68fdc64221edf48d14add",
    strip_prefix = "rules_terser-1.0.0",
    url = "https://github.com/aspect-build/rules_terser/releases/download/v1.0.0/rules_terser-v1.0.0.tar.gz",
)

load("@aspect_rules_terser//terser:dependencies.bzl", "rules_terser_dependencies")

rules_terser_dependencies()

http_archive(
    name = "aspect_rules_ts",
    sha256 = "6ad28b5bac2bb5a74e737925fbc3f62ce1edabe5a48d61a9980c491ef4cedfb7",
    strip_prefix = "rules_ts-2.1.1",
    url = "https://github.com/aspect-build/rules_ts/releases/download/v2.1.1/rules_ts-v2.1.1.tar.gz",
)

load("@aspect_rules_ts//ts:repositories.bzl", "rules_ts_dependencies")

rules_ts_dependencies(
    ts_version_from = "//:package.json",
)

load("@npm//:repositories.bzl", "npm_repositories")

npm_repositories()

http_archive(
    name = "aspect_rules_cypress",
    patch_args = ["-p1"],
    patches = [
        "//third_party:rules_cypress/0001-fix-incorrect-linux-checksums.patch",
        "//third_party:rules_cypress/0002-Add-support-for-cypress-13.6.6.patch",
    ],
    sha256 = "76947778d8e855eee3c15931e1fcdc1c2a25d56d6c0edd110b2227c05b794d08",
    strip_prefix = "rules_cypress-0.3.2",
    urls = [
        "https://github.com/aspect-build/rules_cypress/archive/refs/tags/v0.3.2.tar.gz",
    ],
)

load("@aspect_rules_cypress//cypress:dependencies.bzl", "rules_cypress_dependencies")
load("@aspect_rules_cypress//cypress:repositories.bzl", "cypress_register_toolchains")

rules_cypress_dependencies()

cypress_register_toolchains(
    name = "cypress",
    cypress_version = "13.3.1",
)

# Copied from:
# https://github.com/aspect-build/rules_cypress/blob/3db1b74818ac4ce1b9d489a6e0065b36c1076761/internal_deps.bzl#L47
#
# To update CHROME_REVISION, use the below script
#
# LASTCHANGE_URL="https://www.googleapis.com/download/storage/v1/b/chromium-browser-snapshots/o/Linux_x64%2FLAST_CHANGE?alt=media"
# CHROME_REVISION=$(curl -s -S $LASTCHANGE_URL)
# echo "latest CHROME_REVISION_LINUX is $CHROME_REVISION"
CHROME_REVISION_LINUX = "1264932"

http_archive(
    name = "chrome_linux",
    build_file_content = """filegroup(
name = "all",
srcs = glob(["**"]),
visibility = ["//visibility:public"],
)""",
    sha256 = "4de54f43b2fc4812b9fad4145e44df6ed3063969174a8883ea42ed4c1ee58301",
    strip_prefix = "chrome-linux",
    urls = [
        "https://www.googleapis.com/download/storage/v1/b/chromium-browser-snapshots/o/Linux_x64%2F" + CHROME_REVISION_LINUX + "%2Fchrome-linux.zip?alt=media",
    ],
)

http_archive(
    name = "chromedriver_linux",
    build_file_content = """
filegroup(
    name = "chromedriver",
    srcs = ["chromedriver-linux64/chromedriver"],
    visibility = ["//visibility:public"],
)""",
    sha256 = "527b81f8aaf94344af4103c1166ce5e65037e7ad071c773fe354c215d547ef73",
    urls = [
        "https://storage.googleapis.com/chrome-for-testing-public/124.0.6367.8/linux64/chromedriver-linux64.zip",
    ],
)

http_archive(
    name = "rules_rust_tinyjson",
    build_file = "@rules_rust//util/process_wrapper:BUILD.tinyjson.bazel",
    sha256 = "1a8304da9f9370f6a6f9020b7903b044aa9ce3470f300a1fba5bc77c78145a16",
    strip_prefix = "tinyjson-2.3.0",
    type = "tar.gz",
    url = "https://crates.io/api/v1/crates/tinyjson/2.3.0/download",
)

# Flatbuffers
local_repository(
    name = "com_github_google_flatbuffers",
    path = "third_party/flatbuffers",
)

load("@com_github_google_flatbuffers//ts:repositories.bzl", "flatbuffers_npm")

flatbuffers_npm(
    name = "flatbuffers_npm",
)

load("@flatbuffers_npm//:repositories.bzl", fbs_npm_repositories = "npm_repositories")

fbs_npm_repositories()

local_repository(
    name = "rules_rust",
    path = "third_party/rules_rust",
)

load("@rules_rust//rust:repositories.bzl", "rust_analyzer_toolchain_repository", "rust_repository_set")

RUST_VERSION = "1.70.0"

rust_repository_set(
    name = "rust",
    allocator_library = "@//tools/rust:forward_allocator",
    edition = "2021",
    exec_triple = "x86_64-unknown-linux-gnu",
    extra_target_triples = [
        "arm-unknown-linux-gnueabi",
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
    register_toolchain = False,
    rustfmt_version = RUST_VERSION,
    versions = [RUST_VERSION],
)

load("@rules_rust//crate_universe:repositories.bzl", "crate_universe_dependencies")

crate_universe_dependencies()

load("@rules_rust//crate_universe:defs.bzl", "crate", "crates_repository")

# Run `CARGO_BAZEL_REPIN=1 bazel sync --only=crate_index` to update the lock file
# after adding or removing any dependencies.
crates_repository(
    name = "crate_index",
    annotations = {
        "link-cplusplus": [
            # Bazel toolchains take care of linking the C++ standard library, so don't add
            # an extra flag via Rust by enabling the `nothing` feature. I'm not even sure
            # it would end up on the link command line, but this crate's build.rs attempts
            # to find a C++ compiler itself otherwise which definitely doesn't work.
            crate.annotation(
                crate_features = ["nothing"],
            ),
        ],
        "cxx": [
            crate.annotation(
                additive_build_file = "@//third_party/cargo:cxx/include.BUILD.bazel",
                extra_aliased_targets = ["cxx_cc"],
                gen_build_script = False,
            ),
        ],
        "log": [
            crate.annotation(
                rustc_flags = ["--cfg=atomic_cas"],
            ),
        ],
    },
    cargo_lockfile = "//:Cargo.lock",
    generator_sha256s = {"x86_64-unknown-linux-gnu": "1987a00e7ae12c705fa010b340410230ae8a47d7d95c02900191968b2e745649"},
    generator_urls = {
        "x86_64-unknown-linux-gnu": "https://realtimeroboticsgroup.org/build-dependencies/cargo-bazel-x86_64-unknown-linux-gnu",
    },
    lockfile = "//:Cargo.Bazel.lock",
    manifests = [
        "//:Cargo.toml",
        "//third_party/autocxx:Cargo.toml",
        "//third_party/autocxx:engine/Cargo.toml",
        "//third_party/autocxx:parser/Cargo.toml",
        "//third_party/autocxx:gen/cmd/Cargo.toml",
        "//third_party/autocxx:macro/Cargo.toml",
        "//third_party/autocxx:integration-tests/Cargo.toml",
        "@com_github_google_flatbuffers//rust:flatbuffers/Cargo.toml",
    ],
    rust_toolchain_cargo_template = "@rust__{triple}__{channel}_tools//:bin/{tool}",
    rust_toolchain_rustc_template = "@rust__{triple}__{channel}_tools//:bin/{tool}",
    rust_version = RUST_VERSION,
    supported_platform_triples = [
        "x86_64-unknown-linux-gnu",
        "arm-unknown-linux-gnueabi",
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
)

load("@crate_index//:defs.bzl", "crate_repositories")

crate_repositories()

load("@rules_rust//tools/rust_analyzer:deps.bzl", "rust_analyzer_dependencies")

rust_analyzer_dependencies()

register_toolchains(rust_analyzer_toolchain_repository(
    name = "rust_analyzer_toolchain",
    version = RUST_VERSION,
))

http_archive(
    name = "cxxbridge-cmd",
    build_file = "//third_party/cargo:cxxbridge-cmd/include.BUILD.bazel",
    sha256 = "df13eece12ed9e7bd4fb071a6af4c44421bb9024d339d029f5333bcdaca00000",
    strip_prefix = "cxxbridge-cmd-1.0.100",
    type = "tar.gz",
    urls = ["https://crates.io/api/v1/crates/cxxbridge-cmd/1.0.100/download"],
)

crates_repository(
    name = "cxxbridge_cmd_deps",
    cargo_lockfile = "//third_party/cargo:cxxbridge-cmd/Cargo.lock",
    generator_sha256s = {"x86_64-unknown-linux-gnu": "1987a00e7ae12c705fa010b340410230ae8a47d7d95c02900191968b2e745649"},
    generator_urls = {
        "x86_64-unknown-linux-gnu": "https://realtimeroboticsgroup.org/build-dependencies/cargo-bazel-x86_64-unknown-linux-gnu",
    },
    lockfile = "//third_party/cargo:cxxbridge-cmd/Cargo.Bazel.lock",
    manifests = ["@cxxbridge-cmd//:Cargo.toml"],
    rust_toolchain_cargo_template = "@rust__{triple}__{channel}_tools//:bin/{tool}",
    rust_toolchain_rustc_template = "@rust__{triple}__{channel}_tools//:bin/{tool}",
    rust_version = "1.70.0",
    supported_platform_triples = [
        "x86_64-unknown-linux-gnu",
        "arm-unknown-linux-gnueabi",
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
)

load("@cxxbridge_cmd_deps//:defs.bzl", cxxbridge_cmd_deps = "crate_repositories")

cxxbridge_cmd_deps()

http_file(
    name = "sample_logfile",
    downloaded_file_path = "log.fbs",
    sha256 = "45d1d19fb82786c476d3f21a8d62742abaeeedf4c16a00ec37ae350dcb61f1fc",
    urls = ["https://realtimeroboticsgroup.org/build-dependencies/small_sample_logfile2.fbs"],
)

http_file(
    name = "coral_image_thriftycam_2025",
    downloaded_file_path = "image.bfbs",
    sha256 = "b746bda7db8a6233a74c59c35f3c9d5e343cd9f9c580c897013e8dff7c492eed",
    urls = ["https://realtimeroboticsgroup.org/build-dependencies/coral_image_thriftycam_2025.bfbs"],
)

http_archive(
    name = "drivetrain_replay",
    build_file_content = """
filegroup(
    name = "drivetrain_replay",
    srcs = glob(["**/*.bfbs"]),
    visibility = ["//visibility:public"],
)
    """,
    sha256 = "115dcd2fe005cb9cad3325707aa7f4466390c43a08555edf331c06c108bdf692",
    url = "https://realtimeroboticsgroup.org/build-dependencies/2021-03-20_drivetrain_spin_wheels.tar.gz",
)

http_archive(
    name = "superstructure_replay",
    build_file_content = """
filegroup(
    name = "superstructure_replay",
    srcs = glob(["**/*.bfbs"]),
    visibility = ["//visibility:public"],
)
    """,
    sha256 = "2b9a3ecc83f2aba89a1909ae38fe51e6718a5b4d0e7c131846dfb2845df9cd19",
    url = "https://realtimeroboticsgroup.org/build-dependencies/2021-10-03_superstructure_shoot_balls.tar.gz",
)

http_file(
    name = "opencv_wasm",
    sha256 = "447244d0e67e411f91e7c225c07f104437104e3e753085248a0c527a25bd8807",
    urls = [
        "https://docs.opencv.org/4.9.0/opencv.js",
    ],
)

http_archive(
    name = "halide_k8",
    build_file = "@//debian:halide.BUILD",
    sha256 = "be3bdd067acb9ee0d37d0830821113cd69174bee46da466a836d8829fef7cf91",
    strip_prefix = "Halide-14.0.0-x86-64-linux/",
    url = "https://github.com/halide/Halide/releases/download/v14.0.0/Halide-14.0.0-x86-64-linux-6b9ed2afd1d6d0badf04986602c943e287d44e46.tar.gz",
)

http_archive(
    name = "halide_arm64",
    build_file = "@//debian:halide.BUILD",
    sha256 = "cdd42411bcbba682f73d7db0af69837c4857ee90f1727c6feb37fc9a98132385",
    strip_prefix = "Halide-14.0.0-arm-64-linux/",
    url = "https://github.com/halide/Halide/releases/download/v14.0.0/Halide-14.0.0-arm-64-linux-6b9ed2afd1d6d0badf04986602c943e287d44e46.tar.gz",
)

local_repository(
    name = "snappy",
    path = "third_party/snappy",
)

http_archive(
    name = "io_bazel_rules_go",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@//third_party:rules_go/0001-Disable-warnings-for-external-repositories.patch",
    ],
    sha256 = "af47f30e9cbd70ae34e49866e201b3f77069abb111183f2c0297e7e74ba6bbc0",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_go/releases/download/v0.47.0/rules_go-v0.47.0.zip",
        "https://github.com/bazelbuild/rules_go/releases/download/v0.47.0/rules_go-v0.47.0.zip",
    ],
)

http_archive(
    name = "bazel_gazelle",
    patch_args = [
        "-p1",
    ],
    patches = [
        "@//third_party:bazel-gazelle/0001-Fix-visibility-of-gazelle-runner.patch",
    ],
    sha256 = "75df288c4b31c81eb50f51e2e14f4763cb7548daae126817247064637fd9ea62",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-gazelle/releases/download/v0.36.0/bazel-gazelle-v0.36.0.tar.gz",
        "https://github.com/bazelbuild/bazel-gazelle/releases/download/v0.36.0/bazel-gazelle-v0.36.0.tar.gz",
    ],
)

load("@io_bazel_rules_go//go:deps.bzl", "go_download_sdk", "go_register_toolchains", "go_rules_dependencies")

go_download_sdk(
    name = "go_sdk",
    goarch = "amd64",
    goos = "linux",
    sdks = {
        # Pulled from https://go.dev/dl/ to avoid the external dependency.
        "linux_amd64": ("go1.19.5.linux-amd64.tar.gz", "36519702ae2fd573c9869461990ae550c8c0d955cd28d2827a6b159fda81ff95"),
    },
    version = "1.19.5",
)

go_rules_dependencies()

go_register_toolchains()

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")
load("//:go_deps.bzl", "go_dependencies")
load("//tools/go:mirrored_go_deps.bzl", "mirrored_go_dependencies")

mirrored_go_dependencies()

# gazelle:repository_macro go_deps.bzl%go_dependencies
go_dependencies()

gazelle_dependencies()

http_archive(
    name = "com_google_protobuf",
    sha256 = "4fc5ff1b2c339fb86cd3a25f0b5311478ab081e65ad258c6789359cd84d421f8",
    strip_prefix = "protobuf-26.1",
    url = "https://github.com/protocolbuffers/protobuf/archive/refs/tags/v26.1.tar.gz",
)

http_archive(
    name = "com_github_grpc_grpc",
    patch_args = ["-p1"],
    patches = ["//debian:grpc.patch"],
    sha256 = "493d9905aa09124c2f44268b66205dd013f3925a7e82995f36745974e97af609",
    strip_prefix = "grpc-1.63.0",
    url = "https://github.com/grpc/grpc/archive/refs/tags/v1.63.0.tar.gz",
)

load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")

grpc_deps()

load("@com_github_grpc_grpc//bazel:grpc_extra_deps.bzl", "grpc_extra_deps")

grpc_extra_deps()

http_archive(
    name = "com_github_bazelbuild_buildtools",
    sha256 = "44a6e5acc007e197d45ac3326e7f993f0160af9a58e8777ca7701e00501c0857",
    strip_prefix = "buildtools-4.2.4",
    urls = [
        "https://github.com/bazelbuild/buildtools/archive/refs/tags/4.2.4.tar.gz",
    ],
)

# https://curl.haxx.se/download/curl-7.69.1.tar.gz
http_archive(
    name = "com_github_curl_curl",
    build_file = "//debian:curl.BUILD",
    sha256 = "01ae0c123dee45b01bbaef94c0bc00ed2aec89cb2ee0fd598e0d302a6b5e0a98",
    strip_prefix = "curl-7.69.1",
    url = "https://realtimeroboticsgroup.org/build-dependencies/curl-7.69.1.tar.gz",
)

http_archive(
    name = "com_github_nghttp2_nghttp2",
    build_file = "//debian:BUILD.nghttp2.bazel",
    sha256 = "7da19947b33a07ddcf97b9791331bfee8a8545e6b394275a9971f43cae9d636b",
    strip_prefix = "nghttp2-1.58.0",
    url = "https://github.com/nghttp2/nghttp2/archive/refs/tags/v1.58.0.tar.gz",
)

http_archive(
    # No official name exists.  Names used in our external dependencies include
    # zlib, madler_zlib, com_github_madler_zlib.
    name = "zlib",
    build_file = "//debian:BUILD.zlib.bazel",
    sha256 = "629380c90a77b964d896ed37163f5c3a34f6e6d897311f1df2a7016355c45eff",
    strip_prefix = "zlib-1.2.11",
    urls = [
        "https://github.com/madler/zlib/archive/v1.2.11.tar.gz",
    ],
)

# This one is tricky to get an archive because it has recursive submodules. These semi-automated steps do work though:
# git clone -b 1.11.321 --recurse-submodules --depth=1 https://github.com/aws/aws-sdk-cpp
# cd aws-sdk-cpp
# echo bsdtar -a -cf aws_sdk-version.tar.gz --ignore-zeros @\<\(git archive HEAD\) $(git submodule foreach --recursive --quiet 'echo @\<\(cd $displaypath \&\& git archive HEAD --prefix=$displaypath/\)')
# Now run the command that printed, and the output will be at aws_sdk-version.tar.gz.
http_archive(
    name = "aws_sdk",
    build_file = "//debian:aws_sdk.BUILD",
    sha256 = "08856b91139d209f7423e60dd8f74a14ab6d053ca40088fcb42fd02484003e95",
    url = "https://realtimeroboticsgroup.org/build-dependencies/aws_sdk-1.11.321.tar.gz",
)

# Source code of LZ4 (files under lib/) are under BSD 2-Clause.
# The rest of the repository (build information, documentation, etc.) is under GPLv2.
# We only care about the lib/ subfolder anyways, and strip out any other files.
http_archive(
    name = "com_github_lz4_lz4",
    build_file = "//debian:BUILD.lz4.bazel",
    sha256 = "0b0e3aa07c8c063ddf40b082bdf7e37a1562bda40a0ff5272957f3e987e0e54b",
    strip_prefix = "lz4-1.9.4/lib",
    url = "https://github.com/lz4/lz4/archive/refs/tags/v1.9.4.tar.gz",
)

http_file(
    name = "com_github_foxglove_mcap_mcap",
    executable = True,
    sha256 = "e87895e9af36db629ad01c554258ec03d07b604bc61a0a421449c85223357c71",
    urls = ["https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.51/mcap-linux-amd64"],
)

http_archive(
    name = "com_github_zaphoyd_websocketpp",
    build_file = "//third_party/websocketpp:websocketpp.BUILD",
    patch_args = ["-p1"],
    patches = ["//third_party/websocketpp:websocketpp.patch"],
    sha256 = "6ce889d85ecdc2d8fa07408d6787e7352510750daa66b5ad44aacb47bea76755",
    strip_prefix = "websocketpp-0.8.2",
    url = "https://github.com/zaphoyd/websocketpp/archive/refs/tags/0.8.2.tar.gz",
)

http_archive(
    name = "com_github_foxglove_ws-protocol",
    build_file = "//third_party/foxglove/ws_protocol:foxglove_ws_protocol.BUILD",
    patch_args = ["-p1"],
    patches = ["//third_party/foxglove/ws_protocol:foxglove_ws_protocol.patch"],
    sha256 = "eee484aefe4cb08dcef9ec52df5f904e017e15517865dcfa2462ff8070c9d906",
    strip_prefix = "ws-protocol-releases-typescript-ws-protocol-examples-v0.8.1",
    url = "https://github.com/foxglove/ws-protocol/archive/refs/tags/releases/typescript/ws-protocol-examples/v0.8.1.tar.gz",
)

http_archive(
    name = "asio",
    build_file_content = """
cc_library(
    name = "asio",
    hdrs = glob(["include/asio/**/*.hpp", "include/asio/**/*.ipp", "include/asio.hpp"]),
    visibility = ["//visibility:public"],
    defines = ["ASIO_STANDALONE"],
    includes = ["include/"],
)""",
    sha256 = "8976812c24a118600f6fcf071a20606630a69afe4c0abee3b0dea528e682c585",
    strip_prefix = "asio-1.24.0",
    url = "https://downloads.sourceforge.net/project/asio/asio/1.24.0%2520%2528Stable%2529/asio-1.24.0.tar.bz2",
)

http_archive(
    name = "com_github_foxglove_schemas",
    build_file = "//third_party/foxglove/schemas:schemas.BUILD",
    sha256 = "c0d08365eb8fba0af7773b5f0095fb53fb53f020bde46edaa308af5bb939fc15",
    strip_prefix = "schemas-7a3e077b88142ac46bb4e2616f83dc029b45352e",
    url = "https://github.com/foxglove/schemas/archive/7a3e077b88142ac46bb4e2616f83dc029b45352e.tar.gz",
)

#
# https://www.st.com/en/embedded-software/stsw-img009.html#overview
http_archive(
    name = "vl53l1x_ultra_lite_driver_api",
    build_file = "//third_party/vl53l1x:vl53l1x.BUILD",
    patch_args = ["-p1"],
    patches = ["//third_party/vl53l1x:vl53l1x.patch"],
    sha256 = "06a66254ab7a8b061f93ff0f65abb6088c3ea50335475bb6ac11087beb65d174",
    strip_prefix = "en.STSW-IMG009_v3.5.2/API",
    url = "https://realtimeroboticsgroup.org/build-dependencies/en.STSW-IMG009.zip",
)

http_archive(
    name = "apriltag_test_bfbs_images",
    build_file_content = """
filegroup(
    name = "apriltag_test_bfbs_images",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)""",
    sha256 = "2356b9d0b3be59d01e837bfbbee21de55b16232d5e00c66701c20b64ff3272e3",
    url = "https://realtimeroboticsgroup.org/build-dependencies/2023_arducam_apriltag_test_images.tar.gz",
)

http_file(
    name = "orin_image_apriltag",
    downloaded_file_path = "orin_image_apriltag.bfbs",
    sha256 = "c86604fd0b1301b301e299b1bba2573af8c586413934a386a2bd28fd9b037b84",
    url = "https://realtimeroboticsgroup.org/build-dependencies/orin_image_apriltag.bfbs",
)

http_file(
    name = "orin_large_image_apriltag",
    downloaded_file_path = "orin_large_gs_apriltag.bfbs",
    sha256 = "d933adac0d6c205c574791060be73701ead05977ff5dd9f6f4eadb45817c3ccb",
    url = "https://realtimeroboticsgroup.org/build-dependencies/orin_large_gs_apriltag.bfbs",
)

http_file(
    name = "orin_capture_24_04",
    downloaded_file_path = "orin_capture_24_04.bfbs",
    sha256 = "719edb1d1394c13c1b55d02cf35c277e1d4c2111f4eb4220b28addc08634488a",
    url = "https://realtimeroboticsgroup.org/build-dependencies/orin-capture-24-04-2024.02.14.bfbs",
)

http_file(
    name = "orin_capture_24_04_side",
    downloaded_file_path = "orin_capture_24_04_side.bfbs",
    sha256 = "4747cc98f8794d6570cb12a3171d7984e358581914a28b43fb6bb8b9bd7a10ac",
    url = "https://realtimeroboticsgroup.org/build-dependencies/orin-capture-24-04-side-2024.02.17.bfbs",
)

http_archive(
    name = "julia",
    build_file = "//third_party:julia/julia.BUILD",
    patch_cmds = [
        "echo 'LIB_SYMLINKS = {' > files.bzl",
        """find lib/ -type l -exec bash -c 'echo "\\"{}\\": \\"$(readlink {})\\","' \\; | sort >> files.bzl""",
        "echo '}' >> files.bzl",
        "echo 'LIBS = [' >> files.bzl",
        """find lib/ -type f -exec bash -c 'echo "\\"{}\\","' \\; | sort >> files.bzl""",
        "echo ']' >> files.bzl",
    ],
    sha256 = "e71a24816e8fe9d5f4807664cbbb42738f5aa9fe05397d35c81d4c5d649b9d05",
    strip_prefix = "julia-1.8.5",
    url = "https://julialang-s3.julialang.org/bin/linux/x64/1.8/julia-1.8.5-linux-x86_64.tar.gz",
)

http_archive(
    name = "com_github_nvidia_cccl",
    build_file = "//third_party/cccl:cccl.BUILD",
    sha256 = "38160c628a9e32b7cd55553f299768f72b24074cc9c1a993ba40a177877b3421",
    strip_prefix = "cccl-931dc6793482c61edbc97b7a19256874fd264313",
    url = "https://github.com/NVIDIA/cccl/archive/931dc6793482c61edbc97b7a19256874fd264313.zip",
)

# Hedron's Compile Commands Extractor for Bazel
# https://github.com/hedronvision/bazel-compile-commands-extractor
http_archive(
    name = "hedron_compile_commands",

    # Replace the commit hash (daae6f40adfa5fdb7c89684cbe4d88b691c63b2d) in both places (below) with the latest (https://github.com/hedronvision/bazel-compile-commands-extractor/commits/main), rather than using the stale one here.
    # Even better, set up Renovate and let it do the work for you (see "Suggestion: Updates" in the README).
    sha256 = "43451a32bf271e7ba4635a07f7996d535501f066c0fe8feab04fb0c91dd5986e",
    strip_prefix = "bazel-compile-commands-extractor-daae6f40adfa5fdb7c89684cbe4d88b691c63b2d",
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/daae6f40adfa5fdb7c89684cbe4d88b691c63b2d.tar.gz",
    # When you first run this tool, it'll recommend a sha256 hash to put here with a message like: "DEBUG: Rule 'hedron_compile_commands' indicated that a canonical reproducible form can be obtained by modifying arguments sha256 = ..."
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()

http_archive(
    name = "calibrate_multi_cameras_data",
    build_file_content = """
filegroup(
    name = "calibrate_multi_cameras_data",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)""",
    sha256 = "b106b3b975d3cf3ad3fcd5e4be7409f6095e1d531346a90c4ad6bdb7da1d08a5",
    url = "https://realtimeroboticsgroup.org/build-dependencies/2023_calibrate_multi_cameras_data.tar.gz",
)

http_archive(
    name = "com_github_tartanllama_expected",
    build_file_content = """
cc_library(
  name = "com_github_tartanllama_expected",
  srcs = ["include/tl/expected.hpp"],
  includes = ["include"],
  visibility = ["//visibility:public"],
)""",
    sha256 = "1db357f46dd2b24447156aaf970c4c40a793ef12a8a9c2ad9e096d9801368df6",
    strip_prefix = "expected-1.1.0",
    url = "https://github.com/TartanLlama/expected/archive/refs/tags/v1.1.0.tar.gz",
)

http_archive(
    name = "com_github_storypku_bazel_iwyu",
    integrity = "sha256-R/rVwWn3SveoC8lAcicw6MOfdTqLLkubpaljT4qHjJg=",
    strip_prefix = "bazel_iwyu-bb102395e553215abd66603bcdeb6e93c66ca6d7",
    urls = [
        "https://github.com/storypku/bazel_iwyu/archive/bb102395e553215abd66603bcdeb6e93c66ca6d7.zip",
    ],
)

load("@com_github_storypku_bazel_iwyu//bazel:dependencies.bzl", "bazel_iwyu_dependencies")

bazel_iwyu_dependencies()

http_archive(
    name = "symengine",
    build_file = "@//debian:symengine.BUILD",
    sha256 = "1b5c3b0bc6a9f187635f93585649f24a18e9c7f2167cebcd885edeaaf211d956",
    strip_prefix = "symengine-0.12.0",
    url = "https://github.com/symengine/symengine/releases/download/v0.12.0/symengine-0.12.0.tar.gz",
)

http_archive(
    name = "com_google_tcmalloc",
    sha256 = "f11a004d7361ac6cd3e41fd573b08a92db28220934e8d4c82344ce0aeb20d1e4",
    strip_prefix = "tcmalloc-6c3e8bf43de02934525b3760571ca8781dca1869",
    url = "https://github.com/google/tcmalloc/archive/6c3e8bf43de02934525b3760571ca8781dca1869.zip",
)

http_archive(
    name = "intrinsic_calibration_test_images",
    build_file_content = """
filegroup(
    name = "intrinsic_calibration_test_images",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)""",
    sha256 = "0359e5c19117835c6ec336233a3bbfe2b273797afe9460bf224496802b8f4055",
    url = "https://realtimeroboticsgroup.org/build-dependencies/intrinsic_calibration_test_images.tar.gz",
)

http_archive(
    name = "rules_m4",
    # Obtain the package checksum from the release page:
    # https://github.com/jmillikin/rules_m4/releases/tag/v0.2.4
    sha256 = "e2ada6a8d6963dc57fa56ef15be1894c37ddd85f6195b21eb290a835b1cef03a",
    urls = ["https://github.com/jmillikin/rules_m4/releases/download/v0.2.4/rules_m4-v0.2.4.tar.zst"],
)

load("@rules_m4//m4:m4.bzl", "m4_register_toolchains")

m4_register_toolchains(version = "1.4.18")

http_file(
    name = "frc2025_field_map_welded",
    downloaded_file_path = "frc2025r2.fmap",
    sha256 = "20b7621bf988a6e378a252576d43d2bfbd17d4f38ea2cbb2e7f2cfc82a17732a",
    urls = ["https://downloads.limelightvision.io/models/frc2025r2.fmap"],
)
