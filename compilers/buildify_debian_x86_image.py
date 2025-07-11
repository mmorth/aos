#!/usr/bin/env python3

from __future__ import annotations
import sys
import unicodedata
import datetime
import os
import subprocess
from absl import app
from absl import flags

from rootfs_utils import scoped_tmpdir_tarball, Filesystem, check_buildifier, generate_build_file


def main(argv):
    if len(argv) != 2:
        print("Usage:", file=sys.stderr)
        print(" |buildify_debian_x86_image /path/to/image.tar.zst",
              file=sys.stderr)
        return 1

    if not check_buildifier():
        print(
            "ERROR: Need to have buildifier in the path.  Please resolve this.",
            file=sys.stderr)
        exit()

    full_image = argv[1]

    if not os.path.exists(full_image):
        print("ERROR: Point to a valid image", file=sys.stderr)
        exit()

    with scoped_tmpdir_tarball(full_image) as rootfs:
        filesystem = Filesystem(
            rootfs,
            ["sudo", "chroot", "--userspec=0:0", rootfs, "/bin/bash"],
        )

        # TODO(austin):
        #  I need tests for all this...
        #  This is getting too complicated to trust that it keeps working otherwise.
        # I can't wrap a cc_library around a linker script.
        #  I need to either rewrite the linker script to use sandboxed paths, or I need to inline it into the BUILD file.  Either way, I need to explode if someone tries to do another one.
        # libusb-0.1-4

        packages_to_eval = [
            filesystem.packages['libopencv-dev'],
            filesystem.packages['libc6-dev'],
            filesystem.packages['libusb-dev'],
            filesystem.packages['libusb-1.0-0-dev'],
            filesystem.packages['libstdc++-dev'],
            filesystem.packages['libturbojpeg0-dev'],
            filesystem.packages['nvidia-cuda-dev'],
            filesystem.packages['ssh'],
            filesystem.packages['nvidia-cudnn'],
            filesystem.packages['rsync'],
            filesystem.packages['libgstreamer-plugins-bad1.0-dev'],
            filesystem.packages['libgstreamer-plugins-base1.0-dev'],
            filesystem.packages['libgstreamer1.0-dev'],
        ]

        with open("amd64_debian_rootfs.BUILD", "w") as file:
            file.write(
                generate_build_file(filesystem, packages_to_eval,
                                    "amd64_debian_rootfs.BUILD.template"))

        subprocess.run(['buildifier', "amd64_debian_rootfs.BUILD"])


if __name__ == '__main__':
    app.run(main)
