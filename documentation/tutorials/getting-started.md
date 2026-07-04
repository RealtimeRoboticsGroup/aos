# Getting Started

Install bazel, follow the steps [here](https://bazel.build/install/ubuntu).

## Recommended for Debian and Ubuntu

The simplest setup is to use a bazelisk installation. Select the latest release from "https://github.com/bazelbuild/bazelisk/releases/" Then use curl to pull it down and run dpkg to install it.

```console
curl -L -o bazelisk-amd64.deb https://github.com/bazelbuild/bazelisk/releases/download/v1.25.0/bazelisk-amd64.deb
dpkg -i bazelisk-amd64.deb
```

Bazel has a large caching system, so the first build can be a little slow. Let's run that step now so all subsequent build are fast:

```console
bazel build -c opt //aos/...
```

Note that the `-c opt` argument is not required here, but if you use it here then you should use it in all future builds to preserve the cache. If you run into issue building here, please have a look at the [Build Trouble Shooting](#build-trouble-shooting) section below.

Lets make sure things are running correctly by running the complete set of AOS tests:<br>

```console
bazel test -c opt //aos/...
```

## For roboRIOs

Build for the [roboRIO](https://www.ni.com/en-us/shop/model/roborio.html) target:

```console
bazel build --config=roborio -c opt //aos/...
```

Configuring a roborio: Freshly imaged roboRIOs need to be configured to run aos code at startup. This is done by using the setup_roborio.sh script.

```console
bazel run -c opt //frc/config:setup_roborio -- roboRIO-XXX-frc.local
```

Download code to a roborio:

```console
# For the roborio
bazel run --config=roborio -c opt //y2020:download_stripped -- roboRIO-XXX-frc.local
```

This assumes the roborio is reachable at `roboRIO-XXX-frc.local`. If that does not work, you can try with a static IP address like `10.XX.YY.2`.

## Build Trouble Shooting

#### Additional Bazel Info

If you are having issues increase the log output by running the following:

```console
bazel build --sandbox_debug --verbose_failures -c opt //aos/...
```

#### Debugging Segfaults

**<span style="color:red;">SIGABRT received at time=1741544825 on cpu 0 </span>**<br> Any type of segfault or abort will result in a backtrace, but this is not in the most useful format.<br> Instead re-run the command using gdb. To do this we will need to run the binary directly.<br> For example if we got a segfault running `bazel run //aos/events:ping` we would now need to run `gdb ./bazel-bin/aos/events/ping`<br> This will drop us into a gdb terminal. This program requires the config argument so we would run `run --config bazel-bin/aos/events/pingpong_config.json` which should result in our segfault again.<br> From here we execute the `bt` command to get a full backtrace and begin the search for our problem.

#### Realtime System Configuration

**<span style="color:red;">Check failed: setrlimit64(resource, &rlim) == 0</span>**<br> Adding the `--skip_locking_memory --skip_realtime_scheduler` options should get you running, but this is less than ideal. Follow the instructions in [Realtime System Configuration](https://github.com/RealtimeRoboticsGroup/aos/blob/main/documentation/aos/docs/build_ping_pong.md#realtime-system-configuration) to fix this problem permanently.

#### Less Common Issues

**<span style="color:red;">I/O exception during sandboxed execution: (No space left on device)</span>**<br> Note: AOS can be quite large, building `bazel build //...` is not recommended. Try building only `//aos/..`<br> Another solution might be to edit the `/etc/fstab` file and add the following line and then rebooting the machine.

```bash
tmpfs   /dev/shm    tmpfs   defaults,size=10G   0  0
```
