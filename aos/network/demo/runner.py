import argparse
import os
import shutil
import signal
import sys
import subprocess
from pathlib import Path

from python.runfiles import runfiles

from aos.network.demo import hostnames

# Prevent manifest-based runfiles lookup.
RUNFILES = runfiles.CreateDirectoryBased(
    runfiles.Create().EnvVars()["RUNFILES_DIR"])

ROOT_DIR = RUNFILES.Rlocation("aos")
STARTER = RUNFILES.Rlocation("aos/aos/starter/starterd")
CONFIG = RUNFILES.Rlocation("aos/aos/network/demo/aos_demo_config.bfbs")

SHM_BASE = Path("/dev/shm/aos_network_demo")
LOGGING_FOLDER = Path("/tmp/aos_network_demo/logs")


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("direction", type=str, help="Either 'ping' or 'pong'.")
    args, starter_args = parser.parse_known_args(argv[1:])

    # Set up the logging directory.
    LOGGING_FOLDER.mkdir(parents=True, exist_ok=True)

    # Clean up the shared memory queues. The channel definitions may have changed.
    shutil.rmtree(SHM_BASE, ignore_errors=True)

    def ignore_sigint(signum, frame):
        pass

    # starterd is handling the CTRL-C signal already. Let it do that and return. The
    # corresponding exit code will be propagated up via `check_call`.
    signal.signal(signal.SIGINT, ignore_sigint)

    print(f"ROOT_DIR: {ROOT_DIR}")
    print(f"STARTER: {STARTER}")
    print(f"CONFIG: {CONFIG}")
    print(f"cwd: {Path.cwd()}")

    subprocess.check_call([
        STARTER,
        f"--shm_base={SHM_BASE}",
        f"--config={CONFIG}",
        f"--override_hostname={hostnames.HOSTNAMES[args.direction]}",
    ] + starter_args,
                          cwd=ROOT_DIR)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
