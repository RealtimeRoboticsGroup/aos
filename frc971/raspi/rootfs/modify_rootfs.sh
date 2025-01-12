#!/bin/bash

set -xe

# Full path to Raspberry Pi Bullseye disk image
IMAGE="2022-01-28-raspios-bullseye-arm64-lite.img"
MOD_IMAGE_NAME=`echo ${IMAGE} | sed s/.img/-frc-mods.img/`

if [ ! -f "$IMAGE" ]; then
    echo "Attempting to use already modified image"
    if [ ! -f "$MOD_IMAGE_NAME" ]; then
        echo "Must provide image filename."
        echo "Couldn't find $IMAGE or $MOD_IMAGE_NAME"
        exit 1
    fi
    echo "Using already modified image: ${MOD_IMAGE_NAME}"
    IMAGE=$MOD_IMAGE_NAME
fi

# Kernel built with build_kernel.sh
KERNEL="kernel_5.10.tar.gz"
BOOT_PARTITION="${IMAGE}.boot_partition"
PARTITION="${IMAGE}.partition"

function target() {
  HOME=/root/ USER=root sudo proot -0 -q qemu-aarch64-static -w / -r "${PARTITION}" "$@"
}

function user_pi_target() {
  USER=root sudo proot -0 -q qemu-aarch64-static -w / -r "${PARTITION}" sudo -h 127.0.0.1 -u pi "$@"
}


mkdir -p "${PARTITION}"
mkdir -p "${BOOT_PARTITION}"

if mount | grep "${BOOT_PARTITION}" >/dev/null ;
then
  echo "Already mounted"
else
  OFFSET="$(/sbin/fdisk -lu "${IMAGE}" | grep "${IMAGE}1" | awk '{print 512*$2}')"
  sudo mount -o loop,offset=${OFFSET} "${IMAGE}" "${BOOT_PARTITION}"
fi

# Enable the camera on boot.
if ! grep "start_x=1" "${BOOT_PARTITION}/config.txt"; then
  echo "start_x=1" | sudo tee -a "${BOOT_PARTITION}/config.txt"
fi
if ! grep "gpu_mem=128" "${BOOT_PARTITION}/config.txt"; then
  echo "gpu_mem=128" | sudo tee -a "${BOOT_PARTITION}/config.txt"
fi
if ! grep "enable_uart=1" "${BOOT_PARTITION}/config.txt"; then
  echo "enable_uart=1" | sudo tee -a "${BOOT_PARTITION}/config.txt"
fi
# For now, disable the new libcamera driver in favor of legacy ones
sudo sed -i s/^camera_auto_detect=1/#camera_auto_detect=1/ "${BOOT_PARTITION}/config.txt"
# Enable SPI.
sudo sed -i s/^.*dtparam=spi=on/dtparam=spi=on/ "${BOOT_PARTITION}/config.txt"

sudo tar -zxvf "${KERNEL}" --strip-components 2 -C ${BOOT_PARTITION}/ ./fat32

# Seeing a race condition with umount, so doing lazy umount
sudo umount -l "${BOOT_PARTITION}"
rmdir "${BOOT_PARTITION}"

if mount | grep "${PARTITION}" >/dev/null ;
then
  echo "Already mounted"
else
  OFFSET="$(/sbin/fdisk -lu "${IMAGE}" | grep "${IMAGE}2" | awk '{print 512*$2}')"

  if [[ "$(stat -c %s "${IMAGE}")" < 2000000000 ]]; then
    echo "Growing image"
    dd if=/dev/zero bs=1G count=1 >> "${IMAGE}"
    START="$(/sbin/fdisk -lu "${IMAGE}" | grep "${IMAGE}2" | awk '{print $2}')"

    sed -e 's/\s*\([\+0-9a-zA-Z]*\).*/\1/' << EOF | /sbin/fdisk "${IMAGE}"
  d # remove old partition
  2
  n # new partition
  p # primary partition
  2 # partion number 2
  532480 # start where the old one starts
    # To the end
  p # print the in-memory partition table
  w # Flush
  q # and we're done
EOF

    sudo losetup -o "${OFFSET}" -f "${IMAGE}"
    LOOPBACK="$(sudo losetup --list | grep "${IMAGE}" | awk '{print $1}')"
    sudo e2fsck -f "${LOOPBACK}"
    sudo resize2fs "${LOOPBACK}"
    sudo losetup -d "${LOOPBACK}"
  fi

  echo "Mounting"
  sudo mount -o loop,offset=${OFFSET} "${IMAGE}" "${PARTITION}"
fi

if [[ ! -e wiringpi-2.70-1.deb ]]; then
  wget --continue https://realtimeroboticsgroup.org/build-dependencies/wiringpi-2.70-1.deb
fi

sudo cp target_configure.sh "${PARTITION}/tmp/"
sudo cp wiringpi-2.70-1.deb "${PARTITION}/tmp/"
sudo cp dhcpcd.conf "${PARTITION}/tmp/dhcpcd.conf"
sudo cp sctp.conf "${PARTITION}/etc/sysctl.d/sctp.conf"
sudo cp logind.conf "${PARTITION}/etc/systemd/logind.conf"
sudo cp change_hostname.sh "${PARTITION}/tmp/change_hostname.sh"
sudo cp enable_imu.sh "${PARTITION}/tmp/"
sudo cp frc971.service "${PARTITION}/etc/systemd/system/frc971.service"
sudo cp frc971chrt.service "${PARTITION}/etc/systemd/system/frc971chrt.service"
sudo cp rt.conf "${PARTITION}/etc/security/limits.d/rt.conf"
sudo cp usb-mount@.service "${PARTITION}/etc/systemd/system/usb-mount@.service"
sudo cp 99-usb-mount.rules "${PARTITION}/etc/udev/rules.d/99-usb-mount.rules"

target /bin/mkdir -p /home/pi/.ssh/
cat ~/.ssh/id_rsa.pub | target tee /home/pi/.ssh/authorized_keys

sudo rm -rf "${PARTITION}/lib/modules/"*
sudo tar -zxvf "${KERNEL}" --strip-components 4 -C "${PARTITION}/lib/modules/" ./ext4/lib/modules/
sudo cp adis16505.ko "${PARTITION}/lib/modules/5.10.78-rt55-v8+/kernel/"
target /usr/sbin/depmod 5.10.78-rt55-v8+

# Downloads and installs our target libraries
target /bin/bash /tmp/target_configure.sh

# Add a file to show when this image was last modified and by whom
TIMESTAMP_FILE="${PARTITION}/home/pi/.ImageModifiedDate.txt"
echo "Date modified:"`date` > "${TIMESTAMP_FILE}"
echo "Image file: ${IMAGE}"  >> "${TIMESTAMP_FILE}"
echo "Git tag: "`git rev-parse HEAD` >> "${TIMESTAMP_FILE}"
echo "User: "`whoami` >> "${TIMESTAMP_FILE}"

# Run a prompt as root inside the target to poke around and check things.
target /bin/bash --rcfile /root/.bashrc

sudo umount -l "${PARTITION}"
rmdir "${PARTITION}"

# Move the image to a different name, to indicate we've modified it
if [ ${IMAGE} != ${MOD_IMAGE_NAME} ]; then
  mv ${IMAGE} ${MOD_IMAGE_NAME}
fi
