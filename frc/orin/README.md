# Building the rootfs for the Orin Nano 8g

Check out and follow the instructions on https://github.com/frc4646/meta-frc4646
to create a working image for the Orin.  You will need this to install.

If you have an image in a tarball that someone else built (e.g., something like
`demo-image-base-jetson-orin-nano-som.rootfs.tegraflash.tar.zst`), then you must
extract it first.

In order to connect the Orin, hold down the "recovery"
button while powering on, with a USB cable attached to
your computer and to the USB-C port on the Orin module.
You can confirm that the orin is detected on your computer
using `lsusb`, which should show an NVIDIA device of some sort.

Flash it with `sudo ./initrd-flash`
