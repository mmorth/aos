# This file is run by shell scripts generated by the aos_downloader Skylark
# macro. Everything before the first -- is a hard-coded list of files to
# download.

from __future__ import print_function

import argparse
import sys
from tempfile import TemporaryDirectory
import subprocess
import re
import stat
import os
import shutil


def call(args, **kwargs):
    # Make sure the environmental variables for the ssh agent get through.
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "external/amd64_debian_sysroot/usr/lib/x86_64-linux-gnu/:external/amd64_debian_sysroot/lib/x86_64-linux-gnu/"
    subprocess.check_call(
        args,
        **kwargs,
        env=env,
    )


def install(ssh_target, pkg, channel, ssh_path, scp_path):
    """Installs a package from NI on the ssh target."""
    print("Installing", pkg)
    PKG_URL = f"http://download.ni.com/ni-linux-rt/feeds/academic/2023/arm/{channel}/cortexa9-vfpv3/{pkg}"
    subprocess.check_call(["wget", PKG_URL, "-O", pkg])
    try:
        call([scp_path, "-S", ssh_path, pkg, ssh_target + ":/tmp/" + pkg])
        call([ssh_path, ssh_target, "opkg", "install", "/tmp/" + pkg])
        call([ssh_path, ssh_target, "rm", "/tmp/" + pkg])
    finally:
        subprocess.check_call(["rm", pkg])


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--target",
                        type=str,
                        default="roborio-4646-frc.local",
                        help="Target to deploy code to.")
    parser.add_argument("--type",
                        type=str,
                        choices=["roborio", "pi", "orin"],
                        required=True,
                        help="Target type for deployment")
    parser.add_argument("srcs",
                        type=str,
                        nargs='+',
                        help="List of files to copy over")
    args = parser.parse_args(argv[1:])

    srcs = args.srcs

    destination = args.target

    result = re.match("(?:([^:@]+)@)?([^:@]+)(?::([^:@]+))?", destination)
    if not result:
        print("Not sure how to parse destination \"%s\"!" % destination,
              file=sys.stderr)
        return 1
    user = None
    if result.group(1):
        user = result.group(1)
    hostname = result.group(2)

    if result.group(3):
        target_dir = result.group(3)

    if user is None:
        if args.type == "pi" or args.type == "orin":
            user = "pi"
        elif args.type == "roborio":
            user = "admin"
    target_dir = "/home/" + user + "/bin"

    ssh_target = "%s@%s" % (user, hostname)

    ssh_path = "external/amd64_debian_sysroot/usr/bin/ssh"
    scp_path = "external/amd64_debian_sysroot/usr/bin/scp"

    # install jq
    try:
        subprocess.check_call([ssh_path, ssh_target, "jq", "--version"],
                              stdout=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        if e.returncode == 127:
            print("Didn't find jq on roboRIO, installing jq.")
            install(ssh_target, "jq-lic_1.5-r0.35_cortexa9-vfpv3.ipk", 'extra',
                    ssh_path, scp_path)
            install(ssh_target, "libonig-lic_5.9.6-r0.27_cortexa9-vfpv3.ipk",
                    'extra', ssh_path, scp_path)
            install(ssh_target, "libonig2_5.9.6-r0.27_cortexa9-vfpv3.ipk",
                    'extra', ssh_path, scp_path)
            install(ssh_target, "jq_1.5-r0.35_cortexa9-vfpv3.ipk", 'extra',
                    ssh_path, scp_path)

            call(
                [ssh_path, ssh_target, "jq", "--version"],
                stdout=subprocess.DEVNULL,
            )

    # Since rsync is pretty fixed in what it can do, build up a temporary
    # directory with the exact contents we want the target to have.  This
    # is faster than multiple SSH connections.
    with TemporaryDirectory() as temp_dir:
        pwd = os.getcwd()
        # Bazel gives us the same file twice, so dedup here rather than
        # in starlark
        copied = set()
        for s in srcs:
            if ":" in s:
                folder = os.path.join(temp_dir, s[s.find(":") + 1:])
                os.makedirs(folder, exist_ok=True)
                s = os.path.join(pwd, s[:s.find(":")])
                destination = os.path.join(folder, os.path.basename(s))
            else:
                s = os.path.join(pwd, s)
                destination = os.path.join(temp_dir, os.path.basename(s))

            if s in copied:
                continue
            copied.add(s)
            if s.endswith(".stripped"):
                destination = destination[:destination.find(".stripped")]
            shutil.copy2(s, destination)
        # Make sure the folder that gets created on the roboRIO has open
        # permissions or the executables won't be visible to init.
        os.chmod(temp_dir, 0o775)
        # Starter needs to be SUID so we transition from lvuser to admin.
        if args.type != "pi" and args.type != "orin":
            os.chmod(os.path.join(temp_dir, "starterd"), 0o775 | stat.S_ISUID)

        rsync_cmd = ([
            "external/amd64_debian_sysroot/usr/bin/rsync",
            "-e",
            ssh_path,
            "-c",
            "-r",
            "-v",
            "--perms",
            "-l",
            temp_dir + "/",
        ])

        # If there is only 1 file to transfer, we would overwrite the destination
        # folder.  In that case, specify the full path to the target.
        if len(srcs) == 1:
            rsync_cmd += ["%s:%s/%s" % (ssh_target, target_dir, srcs[0])]
        else:
            rsync_cmd += ["%s:%s" % (ssh_target, target_dir)]

        try:
            call(rsync_cmd)
        except subprocess.CalledProcessError as e:
            if e.returncode == 127 or e.returncode == 12:
                print("Unconfigured roboRIO, installing rsync.")
                install(ssh_target, "libacl1_2.2.52-r0.310_cortexa9-vfpv3.ipk",
                        'main', ssh_path, scp_path)
                install(ssh_target, "rsync-lic_3.1.3-r0.23_cortexa9-vfpv3.ipk",
                        'extra', ssh_path, scp_path)
                install(ssh_target, "rsync_3.1.3-r0.23_cortexa9-vfpv3.ipk",
                        'extra', ssh_path, scp_path)
                call(rsync_cmd)
            elif e.returncode == 11:
                # Directory wasn't created, make it and try again.  This keeps the happy path fast.
                call([ssh_path, ssh_target, "mkdir", "-p", target_dir])
                call(rsync_cmd)
            else:
                raise e


if __name__ == "__main__":
    main(sys.argv)
