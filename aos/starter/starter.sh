#!/bin/bash

set -x

STARTERD_FLAGS=()

if [[ "$(hostname)" == "roboRIO"* ]]; then
  /usr/local/natinst/etc/init.d/systemWebServer stop

  ROBOT_CODE="/home/admin/bin"
  cd "${ROBOT_CODE}"

  ln -s /var/local/natinst/log/FRC_UserProgram.log /tmp/FRC_UserProgram.log
  ln -s /var/local/natinst/log/FRC_UserProgram.log "${ROBOT_CODE}/FRC_UserProgram.log"

  # The roboRIO uses cgroups v1, but doesn't mount the memory controller.
  STARTERD_FLAGS+=(--nomemory_cgroups)
elif [[ "$(hostname)" == "pi-"* ]]; then
  # We have systemd configured to handle restarting, so just exec.
  export PATH="${PATH}:/home/pi/bin"
  exec starterd --user=pi --purge_shm_base
elif [[ "$(hostname)" == "orin-"* ]]; then
  # We have systemd configured to handle restarting, so just exec.
  export PATH="${PATH}:/home/pi/bin"

  # Turn the fans up.
  echo 255 > /sys/devices/platform/pwm-fan/hwmon/hwmon?/pwm1

  # Give read permissions for INA3221 electrical readings sysfs
  chmod -R a+r /sys/devices/platform/c240000.i2c/i2c-1/1-0040/hwmon/hwmon?/

  exec starterd --user=pi --purge_shm_base
else
  ROBOT_CODE="${HOME}/bin"
fi

cd "${ROBOT_CODE}"
export PATH="${PATH}:${ROBOT_CODE}"
while true; do
  starterd --purge_shm_base "${STARTERD_FLAGS[@]}" 2>&1
done
