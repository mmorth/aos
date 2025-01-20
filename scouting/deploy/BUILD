load("@rules_pkg//pkg:mappings.bzl", "pkg_files")
load("@rules_pkg//pkg:pkg.bzl", "pkg_deb", "pkg_tar")

pkg_files(
    name = "systemd_files",
    srcs = [
        "scouting.service",
    ],
    prefix = "etc/systemd/system",
)

pkg_tar(
    name = "server_files",
    srcs = [
        "//scouting",
    ],
    include_runfiles = True,
    package_dir = "opt/aos/scouting_server",
    strip_prefix = ".",
)

pkg_tar(
    name = "deploy_tar",
    srcs = [
        ":systemd_files",
    ],
    deps = [
        ":server_files",
    ],
)

pkg_deb(
    name = "aos-scouting-server",
    architecture = "amd64",
    data = ":deploy_tar",
    depends = [
        "aos-scouting-julia",
    ],
    description = "The scouting web server.",
    # TODO(phil): What's a good email address for this?
    maintainer = "gerrit@realtimeroboticsgroup.org",
    package = "aos-scouting-server",
    postinst = "postinst",
    predepends = [
        "systemd",
    ],
    prerm = "prerm",
    version = "1",
)

py_binary(
    name = "deploy",
    srcs = [
        "deploy.py",
    ],
    args = [
        "--deb",
        "$(location :aos-scouting-server)",
    ],
    data = [
        ":aos-scouting-server",
    ],
    visibility = ["//scouting/DriverRank:__pkg__"],
)
