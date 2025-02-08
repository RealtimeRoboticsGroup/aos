# Getting Started

Install bazel, follow the steps [here](https://bazel.build/install/ubuntu).

Build:
```console
bazel build -c opt //...
```

Build and test:
```console
bazel test -c opt //...
```

## For roboRIOs

Build for the [roboRIO](https://www.ni.com/en-us/shop/model/roborio.html) target:
```console
bazel build --config=roborio -c opt //...
```

Configuring a roborio: Freshly imaged roboRIOs need to be configured to run aos code
at startup.  This is done by using the setup_roborio.sh script.
```console
bazel run -c opt //frc/config:setup_roborio -- roboRIO-XXX-frc.local
```
Download code to a roborio:
```console
# For the roborio
bazel run --config=roborio -c opt //y2020:download_stripped -- roboRIO-XXX-frc.local
```
This assumes the roborio is reachable at `roboRIO-XXX-frc.local`.  If that does not work, you can try with a static IP address like `10.XX.YY.2`.