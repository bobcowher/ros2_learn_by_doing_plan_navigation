#!/bin/bash
conda deactivate

echo "Cleaning up project..."
rm -f compile_commands.json

echo "Building project..."

PATH="/usr/bin:/home/robertcowher/rosprojects/bumperbot_ws/install/bumperbot_localization/bin:/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin:/snap/bin"

. install/setup.bash

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
	     --merge-install

cp build/compile_commands.json .

# Disable if robot isn't physically connected
arduino-cli compile --fqbn arduino:avr:uno ./src/bumperbot_firmware_v2/firmware/
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328old ./src/bumperbot_firmware_v2/firmware/
