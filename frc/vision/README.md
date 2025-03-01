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
