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
