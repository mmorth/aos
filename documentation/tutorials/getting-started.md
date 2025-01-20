# Getting Started
## Locally

These steps assume you are on the most up to date Debian bookworm, however they can be adapted to most other distributions.

1. Updating your system
    1. This can be done by running `sudo apt update && sudo apt upgrade`
2. Installing packages needed for development
    1. `sudo apt install git python3`
3. Installing bazel
    1. Follow the steps [here](https://bazel.build/install/ubuntu)

## Running the Code

To run the code you'll need to download the repository from [gerrit](https://realtimeroboticsgroup.org/gerrit/admin/repos/RealtimeRoboticsGroup/aos), make sure to select ssh and not http. Click on SSH, and clone with commit message hook. Copy the command, and run it locally on terminal.
To learn more about git, open a terminal and run `man git`, or see [git(1)](https://manpages.debian.org/buster/git-man/git.1.en.html) (especially the NOTES section).

Once the repositoy is selected you'll want to make sure to configure your name, email on git. This is required to ensure you're following the [contributing guidelines above](#contributing). You can do this by running these following commands:
```bash
cd aos
git config user.email "<YOUR_EMAIL_HERE>"
git config user.name "<YOUR_NAME>"
```
We use [Bazel](http://bazel.io) to build the code. Bazel has [extensive docs](https://docs.bazel.build/versions/master/build-ref.html), including a nice [build encyclopedia reference](https://docs.bazel.build/versions/master/be/overview.html), and does a great job with fast, correct incremental rebuilds.

### Bazel commands for building, testing, and deploying the code
  * Build and test everything (on the host system, for the roborio target-- note, this may take a while):
```
bazel test //...
bazel build --config=roborio -c opt //...
```
  * Build the code for a specific robot:
```console
# For the roborio:
bazel build --config=roborio -c opt //aos/...
```

  * Configuring a roborio: Freshly imaged roboRIOs need to be configured to run aos code
at startup.  This is done by using the setup_roborio.sh script.
```console
bazel run -c opt //frc/config:setup_roborio -- roboRIO-XXX-frc.local
```

  * Download code to a robot:
```console
# For the roborio
bazel run --config=roborio -c opt //y2020:download_stripped -- roboRIO-XXX-frc.local
```
This assumes the roborio is reachable at `roboRIO-XXX-frc.local`.  If that does not work, you can try with a static IP address like `10.XX.YY.2` (see troubleshooting below)
