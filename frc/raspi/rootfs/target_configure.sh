#!/bin/bash

set -ex

mkdir -p /root/bin
mkdir -p /home/pi/bin

# Give it a static IP
cp /tmp/dhcpcd.conf /etc/

# And provide a script to change it.
cp /tmp/change_hostname.sh /root/bin/
cp /tmp/enable_imu.sh /root/bin/
chmod a+x /root/bin/change_hostname.sh
chmod a+x /root/bin/enable_imu.sh

chown -R pi.pi /home/pi/.ssh
chown -R pi.pi /home/pi/bin

echo 'deb [trusted=yes] https://realtimeroboticsgroup.org/build-dependencies/gstreamer_bullseye_arm64_deps ./' >> /etc/apt/sources.list

apt-get update

apt-get install -y vim-nox \
  git \
  cpufrequtils \
  libopencv-calib3d4.5 \
  libopencv-contrib4.5 \
  libopencv-core4.5 \
  libopencv-features2d4.5 \
  libopencv-flann4.5 \
  libopencv-highgui4.5 \
  libopencv-imgcodecs4.5 \
  libopencv-imgproc4.5 \
  libopencv-ml4.5 \
  libopencv-objdetect4.5 \
  libopencv-photo4.5 \
  libopencv-shape4.5 \
  libopencv-stitching4.5 \
  libopencv-superres4.5 \
  libopencv-video4.5 \
  libopencv-videoio4.5 \
  libopencv-videostab4.5 \
  libopencv-viz4.5 \
  libnice10 \
  pmount \
  libnice-dev \
  feh \
  libgstreamer1.0-0 \
  libgstreamer-plugins-base1.0-0 \
  libgstreamer-plugins-bad1.0-0 \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-nice

PATH=$PATH:/sbin dpkg -i /tmp/wiringpi-2.70-1.deb

echo 'GOVERNOR="performance"' > /etc/default/cpufrequtils

# Add a .bashrc and friends for root.
if [[ ! -e "/root/.dotfiles" ]]; then
  cd /root/
  git clone --separate-git-dir=/root/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles
  rsync --recursive --verbose --exclude '.git' tmpdotfiles/ /root/
  rm -r tmpdotfiles
  git --git-dir=/root/.dotfiles/ --work-tree=/root/ config --local status.showUntrackedFiles no
  # Run the vundle installer which installs plugins on the first run of vim.
  vim -c ":qa!"
fi

# Add a .bashrc and friends for pi.
if [[ ! -e "/home/pi/.dotfiles" ]]; then
  cd /home/pi/
  su -c "git clone --separate-git-dir=/home/pi/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles" pi
  su -c "rsync --recursive --verbose --exclude '.git' tmpdotfiles/ /home/pi/" pi
  su -c "rm -r tmpdotfiles" pi
  su -c "git --git-dir=/home/pi/.dotfiles/ --work-tree=/home/pi/ config --local status.showUntrackedFiles no" pi
  # Run the vundle installer which installs plugins on the first run of vim.
  su -c "vim -c ':qa!'" pi
fi

# Make SSH work and not complain about pi being the username and pw still.
rm -f /etc/profile.d/sshpwd.sh
rm -f /etc/profile.d/wifi-check.sh

systemctl enable ssh.service
systemctl enable aos.service
systemctl enable aoschrt.service

# Default us to pi-4646-1
/root/bin/change_hostname.sh pi-4646-1
