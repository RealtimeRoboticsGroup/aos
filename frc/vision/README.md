Instructions for bringing up a vision system running the
apriltag detection in this folder:

1. Follow [the instructions](../orin/README.md) to flash a rootfs image to an orin.
2. The current default image will cause the Orin to have an IP address of `10.46.46.101` and a hostname of `orin-4646-1`.
3. To SSH to the device, do `ssh pi@10.46.46.101`. The default password will be
   `raspberry`, similar to any raspberry pi device.
4. In order to update the hostname and IP address to match that of your FRC
   team, use the `change_hostname.sh` script, e.g.:

```
$ ssh 10.46.46.101
pi[1] orin-4646-1 ~
$ sudo su
root[1] orin-4646-1 /home/pi
# change_hostname.sh orin-1868-1
root[2] orin-4646-1 /home/pi
# reboot
```

5. To deploy code, run:
   `bazel run -c opt --config=arm64 //frc/vision:download_stripped -- 10.TE.AM.101`
   for a team number `TEAM`.
6. Depending on your cameras, you may need to alter the `/etc/modprobe.d/uvc.conf`.
   By default, most MJPEG cameras we have encountered grossly overestimate the
   bandwidth that they will typically require. This means that v4l2 refuses to
   stream all of your cameras because it could theoretically consume more
   bandwidth than the USB device on the Orin has available. The
   `bandwidth_quirk_divisor` setting in `/etc/modprobe.d/uvc.conf` will be used
   to divide the reported bandwidth. Note that any time a camera attempts to
   send more data than the kernel alots to it, it will end up truncated and
   will cause an invalid image to be read. 4646 is using a divisor of `8`,
   1868 is using a divisor of `2`.

## Intrinsics Calibration

Intrinsics calibration *can* be run just on the Orin. However, it tends to
be significantly faster to run it first on the Orin, then scp the calibration
images off and let the actual *solve* run on a host laptop (the
calibration consists first of collecting 50 images, then of running a solver
against said images; the Orin CPU tends to be very slow at the solve portion).

On the Orin, run:

```
pi[83] orin-1868-1 ~
$ intrinsics_calibration --base_intrinsics bin/base_intrinsics/calibration_orin1-1868-1-fake.json --channel /camera0/gray --calibration_folder intrinsics_images/ --camera_id 25-99 --grayscale --image_save_path intrinsics_images/ --twenty_inch_large_board
```

Notes to be aware of:
* Use a base intrinsics for a camera that *matches* the resolution of your
  camera. Otherwise the calibration will tend to overly aggressively reject
  board detections.
* Set the `--channel` based on what camera you are calibrating.
* Set `--camera_id` to an ID you will use for the *physical* camera you are
  calibrating.
* Move around/rotate the board to persuade it to automatically capture images.
* Once 50 images have been captured, press `q` to quit.
* You need to turn on X11 forwarding (`-X` passed to `ssh`) to get a
  visualization.
* The `--twenty_inch_large_board` corresponds to [this etsy
  listing](https://etsy.com/listing/1820746969/charuco-calibration-target).

If you wish to wait for the orin to complete calibration on its own (on the
order of ~20 minutes), you may. This will produce a JSON file that can be placed
in `frc/vision/constants/` and referenced in `frc/vision/constants.jinja2.json`.

If you want to copy the images off and run intrinsics calibration on your
machine, `scp` them back to your device and run:
```
$ bazel run -c opt //frc/vision:intrinsics_calibration  -- --twenty_inch_large_board --grayscale --override_hostname orin-1868-1 --base_intrinsics ~/aos/frc/vision/constants/calibration_orin1-1868-0_cam-25-11_1970-01-01_03-17-57.json --camera_id 25-99 --channel /camera0/gray --image_load_path ~/logs/2025/intrinsics/test_cal0/ --config frc/vision/aos_config.json
```

With similar notes about the flags to before; update the `--override_hostname`
to match your team number. Make sure you are using the same `--base_intrinsics`
that you used in the live capture.

## Adjusting exposure settings

To adjust exposure settings, the `exposure_100us` setting should be adjusted in
the `constants.jinja2.json`. Setting this value to `0` will enable
auto-exposure.

The "proper" way to change and deploy changes to exposure is to modify the file
in the code-base and then redeploy code. However, at events where there may not
be anyone available who *can* deploy code, the following procedure may be
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

## Viewing live camera feeds

1. Get access to [Foxglove](foxglove.dev)---for working at events, downloading
   the desktop application is encouraged.
2. Select "Open a new connection".
3. Use the "Foxglove WebSocket" option with a URL of `ws://10.18.68.101:8765`
   (substitution your team number as appropriate).
4.  [Import](https://docs.foxglove.dev/docs/visualization/layouts#import-and-export) the Foxglove layout that we have at
   [`foxglove_camera_feeds.json`](foxglove_camera_feeds.json).

You may need to close some of the camera feeds to reduce bandwidth load.
You can also zoom in on feeds, and do any variety of things to ease in looking
at the images.

