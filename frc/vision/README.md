## Hardware

The vision coprocessor is made of three parts:

- **SOM**: [NVIDIA Jetson Orin Nano 8GB module,
  `900-13767-0030-000`](https://www.arrow.com/en/products/900-13767-0030-000/nvidia.html).
  This is the compute module only, with no I/O connectors of its own.
  This production module is not the same as the module that ships on the
  NVIDIA Jetson Orin Nano developer kit, and it needs a different firmware
  image.
- **Carrier board**: [Seeed reComputer J401 carrier
  board](https://www.seeedstudio.com/reComputer-J401-Carrier-Board-for-Jetson-Orin-NX-Orin-Nano-without-Power-Adapter-p-5637.html)
  (sold without a power adapter). It provides the USB ports for the
  cameras, gigabit Ethernet, an M.2 Key M slot for the NVMe drive, and the
  USB-C port used for flashing.
- **SSD**: [Samsung 990 PRO 1TB NVMe, `MZ-V9P1T0B/AM`](https://www.amazon.com/dp/B0BHJF2VRN),
  in the J401's M.2 Key M slot. Write performance matters here: the image
  logger needs to _sustain_ about 1 GB/s of writes for a whole match. Many
  cheaper drives (QLC or DRAM-less) only hit their advertised speed until
  their SLC cache fills, then drop well below that, and the log falls behind.
  If you substitute a different drive, check its sustained write speed, not
  just the peak number on the box.
- **Power/interface board**: [NX-J401-Adapter
  v7](https://github.com/RealtimeRoboticsGroup/electrical/tree/main/circuit-boards/NX-J401-Adapter-v7),
  which mounts to the J401 and connects it to the robot. It has:
  - An LM5176 buck-boost converter that makes a regulated 15V supply for the
    J401 from the FRC battery. It accepts 4.5V-35V in, so the Orin stays up
    when the battery voltage sags under load.
    The input has LM74700 ideal-diode reverse-polarity protection, and the
    output has back-feed protection.
  - A [CANable 2.0](https://canable.io/)-compatible USB-to-CAN adapter
    (STM32G431 and TJA1051), so the Orin can talk on the robot's CAN bus.
  - An isolated USB-C serial console (FT232R behind an ISO6721) wired to the
    Orin's UART2, for debugging without the network.
  - Reset and recovery buttons, and light pipes for the power LEDs, wired
    through the J401's 15-pin control header. Use the recovery button when
    flashing (see below).

Older revisions of the adapter board are in the same repository; use v7 for
new builds.

## Bringing up a vision system

Instructions for bringing up a vision system running the
apriltag detection in this folder:

1. Follow [the instructions](../orin/README.md) to flash a rootfs image to an orin.
2. The current default image will cause the Orin to have an IP address of
   `10.18.68.101` and a hostname of `orin-1868-1`.
3. To SSH to the device, do `ssh pi@10.18.68.101`. The default password will be
   `raspberry`, similar to any raspberry pi device.
4. In order to update the hostname and IP address to match that of your FRC
   team, use the `change_hostname.sh` script in `/root/bin`. It takes a
   hostname of the form `orin-[team#]-[orin#]`, and sets the IP address to
   `10.TE.AM.(100 + orin#)`. For example, for team 4646:

```
$ ssh pi@10.18.68.101
pi[1] orin-1868-1 ~
$ sudo su
root[1] orin-1868-1 /home/pi
# /root/bin/change_hostname.sh orin-4646-1
root[2] orin-1868-1 /home/pi
# reboot
```

5. Set the robot name. The hostname gives the team number, and
   `/home/pi/bin/robotname` names which robot of that team this is. Together
   they pick the matching entry (`team` and `robot_name`) in
   [`constants.jinja2.json`](constants.jinja2.json), which holds that robot's
   camera calibrations. `vision_constants_sender` won't start if the file is
   missing, and nothing will find its constants if there's no matching entry.
   The file isn't overwritten by deploys, so this only needs to be done once:

```
$ echo bot1 > /home/pi/bin/robotname   # team 1868's "bot1" entry
```

6. To deploy code, run:
   `bazel run -c opt --config=arm64 //frc/vision:download_stripped -- 10.TE.AM.101`
   for a team number `TEAM`.
7. Once code is running, view the cameras in Foxglove. See [Viewing live
   camera feeds](#viewing-live-camera-feeds) below.
8. Depending on your cameras, you may need to alter the `/etc/modprobe.d/uvc.conf`.
   By default, most MJPEG cameras we have encountered grossly overestimate the
   bandwidth that they will typically require. This means that v4l2 refuses to
   stream all of your cameras because it could theoretically consume more
   bandwidth than the USB device on the Orin has available. The
   `bandwidth_quirk_divisor` setting in `/etc/modprobe.d/uvc.conf` will be used
   to divide the reported bandwidth. Note that any time a camera attempts to
   send more data than the kernel alots to it, it will end up truncated and
   will cause an invalid image to be read. The image defaults to a divisor of
   `8`. 4646 is using a divisor of `8`,
   1868 is using a divisor of `2`.

## Viewing live camera feeds

1. Get access to [Foxglove](https://foxglove.dev)---for working at events, downloading
   the desktop application is encouraged.
2. Forward the Foxglove websocket port from the Orin over SSH (substituting
   your team number as appropriate), and leave that SSH session open:

   ```
   ssh -L 8765:localhost:8765 pi@10.18.68.101
   ```

   This puts the websocket on `localhost`. Foxglove running in a browser
   refuses to connect to an insecure (`ws://`) websocket on another machine,
   but allows one on `localhost`.

3. Select "Open a new connection", and use the "Foxglove WebSocket" option
   with a URL of `ws://localhost:8765`.
4. [Import](https://docs.foxglove.dev/docs/visualization/layouts#import-and-export) the Foxglove layout that we have at
   [`foxglove_camera_feeds.json`](foxglove_camera_feeds.json).

You may need to close some of the camera feeds to reduce bandwidth load.
You can also zoom in on feeds, and do any variety of things to ease in looking
at the images.

## Viewing the localizer

The Orin also serves a webpage showing what the localizer is doing at
http://10.18.68.101:1180/field.html (substituting your team number as
appropriate). It draws the robot's estimated position on the field, along
with its X, Y, and heading, how many camera images the localizer has
accepted, and whether it is receiving chassis speeds from the robot.

## Robot code (Java) side

The roboRIO and the Orin talk to each other over UDP, plus NetworkTables. On
the Orin side, this is `network_tables_swerve_client`
([`swerve_localizer/network_tables_swerve_client.cc`](swerve_localizer/network_tables_swerve_client.cc)).
On the roboRIO side, use 4646's
[`OrinVision.java`](https://github.com/frc4646/2025-Robot-Public/blob/e853b90ee0815684757cfb01439a986384d29dcc/src/main/java/org/team4646/lib/vision/OrinVision.java)
as the starting point for a new team's robot code.

All messages are packed arrays of little-endian doubles:

| Port | Direction      | Contents                                                                                                                                                     |
| ---- | -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 4647 | roboRIO → Orin | Drivetrain pose x, y, theta, chassis speeds vx, vy, omega, and the timestamp in FPGA time (seconds)                                                          |
| 4648 | Orin → roboRIO | Localizer pose x, y, theta (meters and radians, field coordinates), the timestamp (microseconds), and the number of camera images the localizer has accepted |
| 4649 | Orin → roboRIO | The closest game piece detection: confidence, x, y, width, height, and the timestamp (microseconds)                                                          |

The roboRIO code sends its drivetrain state to the Orin's IP address
(`10.TE.AM.101`). The Orin sends back to the hostname `roborio`, which
`change_hostname.sh` points at `10.TE.AM.2` in `/etc/hosts`.

`network_tables_swerve_client` also connects to the roboRIO's NetworkTables
server. It reads the match state (enabled, alliance, match time, ...) from the
AdvantageKit `/AdvantageKit/DriverStation/...` topics, and uses the
NetworkTables time offset to convert the pose timestamps to the roboRIO's
clock. It doesn't send poses until NetworkTables is connected, and
`OrinUdp` ignores poses whose timestamp is more than a second off from the
roboRIO's clock. So if the robot isn't getting poses, check that
NetworkTables is connected first.

## Checking on the Orin

SSH in with `ssh pi@10.TE.AM.101` and use these tools to see what is running.
They live in `~/bin`, which is on the `PATH` for interactive shells.

### aos_starter

`aos_starter` talks to starterd, which starts and supervises every AOS
application. `aos_starter status` lists them all:

```
$ aos_starter status
Name                           Node       State    PID    Uptime    Last Exit Code
apriltag_detector0             orin       RUNNING  612    312s      -
apriltag_detector1             orin       RUNNING  608    312s      -
apriltag_detector2             orin       RUNNING  610    312s      -
apriltag_detector3             orin       RUNNING  620    312s      -
camera_reader0                 orin       RUNNING  617    312s      -
camera_reader1                 orin       RUNNING  624    312s      -
camera_reader2                 orin       RUNNING  615    312s      -
camera_reader3                 orin       RUNNING  622    312s      -
field_map_constants_sender     orin       STOPPED                   0
field_side_exposure_adjuster   orin       RUNNING  616    312s      -
filesystem_monitor             orin       RUNNING  623    312s      -
foxglove_websocket             orin       RUNNING  621    312s      -
game_piece_mapper              orin       RUNNING  625    312s      -
hardware_monitor               orin       RUNNING  619    312s      -
image_logger                   orin       RUNNING  607    312s      -
irq_affinity                   orin       RUNNING  606    312s      -
localizer_logger               orin       RUNNING  605    312s      -
localizer_main                 orin       RUNNING  626    312s      -
network_tables_publisher       orin       RUNNING  627    312s      -
network_tables_swerve_client   orin       RUNNING  603    312s      -
turbojpeg_decoder0             orin       RUNNING  614    312s      -
turbojpeg_decoder1             orin       RUNNING  628    312s      -
turbojpeg_decoder2             orin       RUNNING  602    312s      -
turbojpeg_decoder3             orin       RUNNING  601    312s      -
vision_constants_sender        orin       STOPPED                   0
web_proxy                      orin       RUNNING  609    312s      -
yolo                           orin       RUNNING  618    312s      -
```

`vision_constants_sender` and `field_map_constants_sender` send their message
once and exit, so `STOPPED` with an exit code of `0` is normal for them.
Anything else that is not `RUNNING` is a problem. An application that keeps
crashing shows up as `WAITING` with a short uptime and a non-zero exit code
(for example, `camera_readerN` exits with `6` when its camera isn't plugged
in), and starterd restarts it every few seconds. Look in `journalctl` (below)
for why.

```
$ aos_starter status camera_reader0             # details for one application
$ aos_starter restart vision_constants_sender   # also start / stop
```

To restart all of the robot code, including starterd itself, restart the
`aos` systemd service:

```
$ sudo systemctl restart aos.service
```

### aos_dump

`aos_dump` prints the messages being sent on a channel, which is the quickest
way to check that a camera or detector is actually producing data. With no
arguments, it lists every channel:

```
$ aos_dump
Channels:
/aos aos.logging.LogMessageFbs
/aos aos.starter.Status
/aos aos.timing.Report
...
/camera0 frc.vision.CameraImage
/camera0 frc.vision.CameraStreamSettings
/camera0/gray foxglove.ImageAnnotations
/camera0/gray frc.vision.CameraImage
/camera0/gray frc.vision.TargetMap
...
/constants frc.vision.CameraConstants
/hardware_monitor frc.orin.HardwareStats
/localizer frc.vision.swerve_localizer.Status
```

Give it a channel (and optionally a type) to print messages. `--count 1`
prints one message and exits; without it, `aos_dump` keeps printing until you
hit Ctrl-C. Long arrays like image data are summarized:

```
$ aos_dump /camera0 frc.vision.CameraImage --count 1
2026-10-06_03-37-31.630108418 (677615.678753443sec) /camera0 frc.vision.CameraImage: { "rows": 1304, "cols": 1600, "data": [ "... 63011 elements ..." ], "monotonic_timestamp_ns": 677615662691000, "format": "MJPEG" }

$ aos_dump /camera0/gray frc.vision.TargetMap --count 1
2026-10-06_03-37-31.700679051 (677615.749324076sec) /camera0/gray frc.vision.TargetMap: { "target_poses": [  ], "monotonic_timestamp_ns": 677615726698000, "rejections": 0 }
```

An empty `target_poses` means that camera isn't seeing any AprilTags. Use
`--fetch` to print the most recent message even if nothing new is being sent,
which is useful for channels that are only sent once, like `/constants`:

```
$ aos_dump --fetch /constants frc.vision.CameraConstants --count 1
```

Run `aos_dump --help` for more options (`--pretty`, `--json`,
`--max_vector_size`, ...).

### journalctl

AOS is started at boot by the `aos` systemd service, which runs starterd. The
output of starterd and every application it starts ends up in the journal, so
this is where to look when something is crashing. A good default is to follow
everything, which also shows what the kernel and the rest of the system are
doing (cameras disconnecting, the network going down, ...):

```
$ sudo journalctl -f
```

To narrow it down:

```
$ systemctl status aos                    # is AOS running, plus its last few log lines
$ journalctl -u aos -f                    # follow only AOS
$ journalctl -u aos -b --no-pager | less  # everything from AOS since boot
$ journalctl -k                           # kernel log, e.g. USB camera or uvcvideo errors
```

The `pi` user is in the `adm` and `system-journal` groups, so all of these
work without `sudo` too.

When an application crashes, starterd only logs that it exited and that it
will restart it. The actual reason is in the application's own output,
further up. Each line is tagged with the PID that printed it: starterd's lines
have starterd's PID (`598` below), and the application's lines have the PID
from the `Starting` line (`641` below). Here is one full crash of `yolo`:

```
Oct 06 03:48:12 orin-1868-1 starter.sh[598]: I1006 03:48:12.432151  598 subprocess.cc:452] Starting 'yolo' pid 641
Oct 06 03:48:12 orin-1868-1 starter.sh[598]: I1006 03:48:12.442546  598 subprocess.cc:452] Starting 'localizer_main' pid 642
Oct 06 03:48:12 orin-1868-1 starter.sh[598]: I1006 03:48:12.442987  598 subprocess.cc:452] Starting 'network_tables_publisher' pid 643
Oct 06 03:48:12 orin-1868-1 starter.sh[598]: W1006 03:48:12.456069  598 subprocess.cc:870] Failed to start 'localizer_main' on pid 642 : Exited with status 6 and starter version 'unknown'
Oct 06 03:48:12 orin-1868-1 starter.sh[598]: I1006 03:48:12.456117  598 subprocess.cc:667] Restarting localizer_main in 3 seconds
Oct 06 03:48:12 orin-1868-1 starter.sh[598]: W1006 03:48:12.456358  598 subprocess.cc:870] Failed to start 'network_tables_publisher' on pid 643 : Exited with status 6 and starter version 'unknown'
Oct 06 03:48:12 orin-1868-1 starter.sh[598]: I1006 03:48:12.456371  598 subprocess.cc:667] Restarting network_tables_publisher in 3 seconds
Oct 06 03:48:12 orin-1868-1 starter.sh[641]: E1006 03:48:12.804594  641 file.cc:43] Failed to open /home/pi/bin/with_auto.engine: No such file or directory [2]
Oct 06 03:48:12 orin-1868-1 starter.sh[641]: F1006 03:48:12.804734  641 file.cc:34] Check failed: r.has_value() Failed to read /home/pi/bin/with_auto.engine to string: No such file or directory [2]
Oct 06 03:48:12 orin-1868-1 starter.sh[641]: [symbolize_elf.inc : 401] RAW: Unable to get high fd: rc=0, limit=1024
Oct 06 03:48:12 orin-1868-1 starter.sh[641]: *** Check failure stack trace: ***
Oct 06 03:48:12 orin-1868-1 starter.sh[641]:     @     0xaaaacc6f8f40  absl::lts_20250512::log_internal::LogMessage::SendToLog()
Oct 06 03:48:12 orin-1868-1 starter.sh[641]:     @     0xaaaacc6f8eb4  absl::lts_20250512::log_internal::LogMessage::Flush()
Oct 06 03:48:12 orin-1868-1 starter.sh[641]:     @     0xaaaacc6e9280  aos::util::ReadFileToStringOrDie[abi:cxx11]()
Oct 06 03:48:12 orin-1868-1 starter.sh[641]:     @     0xaaaacc67abf8  yolo::ModelInference::InitializeEngine()
Oct 06 03:48:12 orin-1868-1 starter.sh[641]:     @     0xaaaacc679c10  yolo::YoloApplication::YoloApplication()
Oct 06 03:48:12 orin-1868-1 starter.sh[641]:     @     0xaaaacc6799c8  yolo::Main()
Oct 06 03:48:12 orin-1868-1 starter.sh[641]:     @     0xaaaacc67a14c  main
Oct 06 03:48:12 orin-1868-1 starter.sh[641]:     @     0xffff38db2104  (unknown)
Oct 06 03:48:12 orin-1868-1 starter.sh[641]:     @     0xffff38db21e4  __libc_start_main
Oct 06 03:48:12 orin-1868-1 starter.sh[641]: *** SIGABRT received at time=1791258492 on cpu 4 ***
Oct 06 03:48:12 orin-1868-1 starter.sh[598]: W1006 03:48:12.822835  598 subprocess.cc:870] Failed to start 'yolo' on pid 641 : Exited with status 6 and starter version 'unknown'
Oct 06 03:48:12 orin-1868-1 starter.sh[598]: I1006 03:48:12.822896  598 subprocess.cc:667] Restarting yolo in 3 seconds
```

Reading from the bottom up: starterd reports that `yolo` (PID 641) exited
with status 6, which is `SIGABRT` from a failed `CHECK`. Above the stack trace,
the `F` (fatal) line has the reason: the `/home/pi/bin/with_auto.engine` YOLO
model file is missing. Other applications' output is mixed in (here,
`localizer_main` and `network_tables_publisher` crashing at the same time), so
follow the PID. Since crashing applications restart every few seconds, this
repeats over and over. To find the `CHECK` messages quickly:

```
$ journalctl -u aos -b --no-pager | grep -B2 -A14 'Check failed'
```

## Setting up constants for a new robot

All the camera constants for every robot live in
[`constants.jinja2.json`](constants.jinja2.json). Each entry is selected by
the team number (from the hostname) and the robot name (from
`/home/pi/bin/robotname`, see the bring-up steps above), and holds a
calibration for each camera plus the stream settings for all of them. A new
robot needs an entry before anything that uses calibrations will run, so set
one up with a base calibration first, and replace it with real intrinsics and
extrinsics as you calibrate (see below).

1. Make a base calibration file for each of the four cameras in
   `frc/vision/constants/`. The applications look up a camera's calibration by
   `node_name` (`orin`) and `camera_number` (`0`-`3`, matching `/camera0` to
   `/camera3`), so each camera number must appear exactly once. Each file
   needs:
   - `intrinsics` and `dist_coeffs` from an already calibrated camera of the
     same model, at the same resolution. These are close enough to get
     started, and intrinsics calibration will replace them.
   - `fixed_extrinsics`, the camera's position on the robot.
     `localizer_main`, `game_piece_mapper`, and `network_tables_publisher`
     all `CHECK` that this exists and will crash-loop without it, so even a
     placeholder is better than nothing.

   Start by copying an existing calibration file for each camera, and set
   `team_number` and `camera_number`. Keep the `camera_id` of the camera the
   intrinsics came from.

   Get `fixed_extrinsics` from the robot's Onshape CAD using the
   [`MateConnectorTransform.featurescript`](MateConnectorTransform.featurescript)
   custom feature:
   - Add a Feature Studio to the Onshape document, paste the script into it,
     and the "Mate Connector Transform" feature becomes available in the Part
     Studio's custom features.
   - Put a mate connector on each camera's lens with Z pointing out of the
     lens along the optical axis, X to the right in the image, and Y down in
     the image (the OpenCV camera convention). Put another at the robot's
     origin: the center of the robot at floor level, X forward, Y left, and Z
     up.
   - Add a Mate Connector Transform feature with the camera's mate connector
     as "From" and the robot origin as "To". It prints the row-major 4x4
     camera-to-robot transform, in meters, in the feature info and the
     FeatureScript console. Paste that in as `fixed_extrinsics.data`.

2. Add an entry for the robot to `constants.jinja2.json`. `robot_name` must
   match what is in `/home/pi/bin/robotname` on that robot, and the image size
   in `default_camera_stream_settings` must match the resolution the base
   intrinsics were calibrated at. Every file you `include` must exist in
   `frc/vision/constants/`, even if it is just a placeholder copy for a camera
   you haven't calibrated yet. Otherwise the build fails with
   `jinja2.exceptions.TemplateNotFound`:

   ```
   {
     "team": 9999,
     "robot_name": "newbot",
     "data": {
       "calibration": [
         {% include 'frc/vision/constants/calibration_orin-9999-0_cam-....json' %},
         {% include 'frc/vision/constants/calibration_orin-9999-1_cam-....json' %},
         {% include 'frc/vision/constants/calibration_orin-9999-2_cam-....json' %},
         {% include 'frc/vision/constants/calibration_orin-9999-3_cam-....json' %}
       ],
       "default_camera_stream_settings": {
         "image_width": 1600,
         "image_height": 1304,
         "exposure_100us": 15,
         "gain": 0
       }
     }
   }
   ```

   `default_camera_stream_settings` can also set `frame_period` (as a
   `numerator`/`denominator` fraction of a second). See
   [Adjusting exposure settings](#adjusting-exposure-settings) for exposure.

3. Check that it builds, which also validates the JSON against the schema:

   ```
   $ bazel build -c opt //frc/vision:constants.json
   ```

4. Deploy, then restart the robot code on the Orin. The applications only
   read the constants when they start, and deploying doesn't restart them:

   ```
   $ sudo systemctl restart aos.service
   ```

   Then check that the constants are being sent and that nothing is
   crash-looping:

   ```
   $ aos_dump --fetch /constants frc.vision.CameraConstants --count 1
   $ aos_starter status
   ```

## Intrinsics Calibration

Intrinsics calibration runs on the Orin. It first collects 50 images of the
board, then runs a solver against them. The solve is slow on the Orin's CPU,
around 5 minutes, so start it and leave it running; keep the SSH
session open until it finishes.

The recommended calibration board is the calib.io [600x400mm ChArUco target
with the coarse pattern](https://calib.io/products/charuco-targets?variant=9400454938671)
(9x14 squares, 40mm checkers, 30mm markers, `DICT_5x5`). This is the board
`--dict5x5_9x14_board` describes, and that flag is on by default. It is made
to order, so order it well before you need it. It needs to be flat and rigid,
so use the real target rather than a printout.

On the Orin, run:

```
pi[83] orin-1868-1 ~
$ intrinsics_calibration --base_intrinsics /home/pi/bin/base_intrinsics/calibration_orin1-1868-1-fake.json --channel /camera0/gray --calibration_folder intrinsics_images/ --camera_id 25-99 --grayscale --image_save_path intrinsics_images/ --dict5x5_9x14_board --visualize
```

Notes to be aware of:

- Use a base intrinsics for a camera that _matches_ the resolution of your
  camera. Otherwise the calibration will tend to overly aggressively reject
  board detections.
- Set the `--channel` based on what camera you are calibrating.
- Set `--camera_id` to an ID you will use for the _physical_ camera you are
  calibrating.
- Move around/rotate the board to persuade it to automatically capture images.
- Once 50 images have been captured (see `--min_images_to_calibrate`), it will
  automatically quit and start the intrinsics calibration.
- `--visualize` opens two windows, so you need X11 forwarding (`ssh -X`):
  - "Display" is the live image from the camera, with the detected board drawn
    on it and a count of how many images have been captured. It is mirrored, so
    it moves like a mirror while you are holding the board in front of the
    camera. "Invalid Estimated Pose" means the board was seen but its pose
    couldn't be estimated, so that frame won't be used.
  - "Captured Point Visualization" shows every board corner from every image
    captured so far. You want these points spread evenly across the whole
    image, all the way into the corners and edges, where lens distortion is
    strongest. If there are gaps, move the board into them.
- The `--twenty_inch_large_board` corresponds to [this etsy
  listing](https://etsy.com/listing/1820746969/charuco-calibration-target).
- The `--dict5x5_9x14_board` corresponds to the recommended calib.io board
  above (9x14, 40mm, 30mm, DICT5x5).
- To use a different board, change the definition of `board_` in
  `charuco_lib.cc`.

When the solve finishes, it writes a JSON file to the `--calibration_folder`.
Copy it off the Orin into `frc/vision/constants/`, and update
`frc/vision/constants.jinja2.json` to point the camera at the new file in
place of its base calibration.

## Extrinsics calibration

```
bazel run -c opt //frc/vision:calibrate_multi_cameras -- `realpath pre-champs-calibration2/` --team_number 4646 --vmodule=calibrate_multi_cameras_lib=0 --visualize
```

Do the same, copy the constants over and update `constants.jinja.json`.

## Replaying the localizer

```
rm -rf /tmp/replayed/ && bazel run -c opt //frc/vision/swerve_localizer:localizer_replay -- /home/austin/local/aos/frc/vision/logs/image_log-060_1970-01-01_00-02-15-MOSE-e11/ --vmodule=simulated_event_loop=0
```

## Adjusting exposure settings

To adjust exposure settings, the `exposure_100us` setting should be adjusted in
the `constants.jinja2.json`. Setting this value to `0` will enable
auto-exposure.

The "proper" way to change and deploy changes to exposure is to modify the file
in the code-base and then redeploy code. However, at events where there may not
be anyone available who _can_ deploy code, the following procedure may be
followed:

1. SSH onto the device: `ssh pi@10.18.68.101`.
2. Edit the `constants.json`: `vim bin/constants.json`
3. Edit the `exposure_100us` setting to the value you want (note: there may be
   multiple entries for each robot listed in the constants; there is no harm in
   just altering all of the exposure settings).
4. Save & exit editing the constants file.
5. Restart the constants sender to cause the changed constants to take effect:
   `aos_starter restart vision_constants_sender`
6. Note: Sometimes camera drivers have difficulty doing things right and you may
   need to restart the whole device to have the changes take effect.

## YOLO

`yolo.cc` listens to images and publishes bounding box detections of those images.
It assumes a 1600x1304 image, downsizing by a factor of 3 and cropping off the bottom and left side of the image.
The crop and resize and normalize is done in halide on the CPU to save GPU bandwidth.

Export the model to onnx with:

```
yolo export model=~/local/frc4646/yolo/runs/detect/train21/weights/best.pt format="onnx" imgsz=[416,512] data=../../coral.yaml
```

Then convert it to the required tensorrt engine on the ORIN by runing:

```
/usr/src/tensorrt/bin/trtexec --fp16 --onnx=best.onnx --saveEngine=best416x.engine --useSpinWait --noDataTransfer --useCudaGraph
```
