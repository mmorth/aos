"""Starts up Apache to provide HTTPS + LDAP for another web server.

This script is used by the apache_wrapper() rule as the main entrypoint for its
"executable". This script sets up a minimal Apache environment in a directory
in /tmp.

Both Apache and the wrapped server binary are started by this script. The
wrapped server should bind to the port specified by the APACHE_WRAPPED_PORT
environment variable.

See the documentation for apache_wrapper() for more information.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

import jinja2

DUMMY_CERT_ANSWERS = """\
US
California
Mountain View
Realtime Robotics Group
Software
realtimeroboticsgroup.org
dummy@realtimeroboticsgroup.org
"""


def wait_for_server(port: int):
    """Waits for the server at the specified port to respond to TCP connections."""
    while True:
        try:
            connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connection.connect(("localhost", port))
            connection.close()
            break
        except ConnectionRefusedError:
            connection.close()
            time.sleep(0.01)


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=str, required=True)
    parser.add_argument("--https_port", type=int, default=7000)
    parser.add_argument("--wrapped_port", type=int, default=7500)
    parser.add_argument(
        "--ldap_info",
        type=str,
        help=
        "JSON file containing 'ldap_bind_dn', 'ldap_url', and 'ldap_password' entries.",
        default="",
    )
    args, unknown_args = parser.parse_known_args(argv[1:])

    if not args.ldap_info:
        args.ldap_info = os.path.join(os.environ["BUILD_WORKSPACE_DIRECTORY"],
                                      "ldap.json")

    with open("tools/build_rules/apache_template.conf", "r") as file:
        template = jinja2.Template(file.read())

    with open(args.ldap_info, "r") as file:
        substitutions = json.load(file)

    for key in ("ldap_bind_dn", "ldap_url", "ldap_password"):
        if key not in substitutions:
            raise KeyError(
                f"The ldap_info JSON file must contain key '{key}'.")

    substitutions.update({
        "https_port": args.https_port,
        "wrapped_port": args.wrapped_port,
    })

    config_text = template.render(substitutions)

    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir = Path(temp_dir)
        with open(temp_dir / "apache2.conf", "w") as file:
            file.write(config_text)

        # Create a directory for error logs and such.
        logs_dir = temp_dir / "logs"
        os.mkdir(logs_dir)

        print("-" * 60)
        print(f"Logs are in {logs_dir}/")
        print("-" * 60)

        # Make modules available.
        modules_path = Path(
            "external/amd64_debian_sysroot/usr/lib/apache2/modules")
        os.symlink(modules_path.resolve(), temp_dir / "modules")

        # Generate a testing cert.
        subprocess.run(
            [
                "openssl",
                "req",
                "-x509",
                "-nodes",
                "-days=365",
                "-newkey=rsa:2048",
                "-keyout=" + str(temp_dir / "apache-selfsigned.key"),
                "-out=" + str(temp_dir / "apache-selfsigned.crt"),
            ],
            check=True,
            input=DUMMY_CERT_ANSWERS,
            text=True,
        )

        # Start the wrapped binary in the background.
        # Tell it via the environment what port to listen on.
        env = os.environ.copy()
        env["APACHE_WRAPPED_PORT"] = str(args.wrapped_port)
        wrapped_binary = subprocess.Popen([args.binary] + unknown_args,
                                          env=env)

        # Start the apache server.
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = "external/amd64_debian_sysroot/usr/lib/x86_64-linux-gnu"
        apache = subprocess.Popen(
            [
                "external/amd64_debian_sysroot/usr/sbin/apache2", "-X", "-d",
                str(temp_dir)
            ],
            env=env,
        )

        wait_for_server(args.https_port)
        wait_for_server(args.wrapped_port)
        # Sleep to attempt to get the HTTPS message after the webserver message.
        time.sleep(1)
        print(f"Serving HTTPS on port {args.https_port}")

        # Wait until we see a request to shut down.
        signal.signal(signal.SIGINT, lambda signum, frame: None)
        signal.signal(signal.SIGTERM, lambda signum, frame: None)
        signal.pause()

        print("\nShutting down apache and wrapped binary.")
        apache.terminate()
        wrapped_binary.terminate()
        apache.wait()
        wrapped_binary.wait()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
