# Flashing the Orin Nano 8GB

## Getting an image

The current image is
[`demo-image-base-p3768-0000-p3767-0003.rootfs.tegraflash.tar.zst`](https://mirror.spacecookies.dev/dependencies/yocto_images/2026.01.17/demo-image-base-p3768-0000-p3767-0003.rootfs.tegraflash.tar.zst)
(2026.01.17, about 2.6GB). It is built from the `frc1868-walnascar` branch of
https://github.com/frc4646/meta-frc4646 (Yocto walnascar, CUDA 12.6, OpenCV
4.11), which matches the sysroot AOS is currently compiled against.

`p3768-0000-p3767-0003` is the build target for the production Orin Nano 8GB
module on a J401 carrier (see the [hardware list](../vision/README.md#hardware)).
The production module needs a different image than the NVIDIA Orin Nano
developer kit, which uses `jetson-orin-nano-devkit-nvme` instead. Don't flash a
dev kit image onto the production module, or vice versa.

To build your own image instead, follow the instructions in meta-frc4646.

## Flashing

You will have much better luck flashing from a Linux PC. The flash script is a
Linux shell script, and the Orin disconnects and reconnects over USB several
times while flashing, which VMs and WSL tend to lose track of. If you don't
have a Linux machine, boot a laptop from an
[Ubuntu Desktop live USB](https://ubuntu.com/tutorials/try-ubuntu-before-you-install)
and flash from there. Note that a live session keeps files in RAM, and the
extracted image is around 9GB, so either use a machine with plenty of RAM or
download and extract onto a separate USB drive or the laptop's disk.

1. Install the tools the flash scripts need. On Ubuntu or Debian:

   ```
   sudo apt update
   sudo apt install zstd python3 python3-yaml device-tree-compiler cpp \
       gdisk parted udev udisks2 usbutils bmap-tools
   ```

   `initrd-flash` and the scripts it calls use `dtc` and `cpp` to build the
   boot configuration, `python3` with `yaml` for NVIDIA's flashing tools,
   `udisksctl` (from `udisks2`) to mount the Orin's storage when it shows up as
   a USB drive, and `sgdisk` (from `gdisk`), `partprobe` (from `parted`), and
   `udevadm` to partition and find it. `bmaptool` is optional, but makes
   writing the root filesystem faster. If the flash fails with a "command not
   found" error, install whatever is missing and try again.

2. Download and extract the image:

   ```
   mkdir orin-image && cd orin-image
   wget https://mirror.spacecookies.dev/dependencies/yocto_images/2026.01.17/demo-image-base-p3768-0000-p3767-0003.rootfs.tegraflash.tar.zst
   tar --zstd -xf demo-image-base-p3768-0000-p3767-0003.rootfs.tegraflash.tar.zst
   ```

3. Plug a USB cable from your computer into the USB-C port on the reComputer
   J401 carrier board. Then hold down the "RECOVERY" button on the
   NX-J401-Adapter board while powering on. Check that the Orin is in recovery
   mode with `lsusb`. You should see an NVIDIA `APX` device with ID
   `0955:7523` (the Orin Nano 8GB in recovery mode):

   ```
   $ lsusb -d 0955:
   Bus 001 Device 011: ID 0955:7523 NVIDIA Corp. APX
   ```

   If nothing shows up, the Orin booted normally instead of into recovery.
   Power it off and try again, holding RECOVERY until it is powered on. Also
   make sure your cable carries data and isn't a charge-only cable.

4. Flash it with `sudo ./initrd-flash` from the extracted directory. During
   flashing, the Orin reboots and reconnects as a USB drive, which the script
   then writes to; leave the cable plugged in until the script says it is done.

Once it boots, continue with the [vision bring-up
instructions](../vision/README.md#bringing-up-a-vision-system).
