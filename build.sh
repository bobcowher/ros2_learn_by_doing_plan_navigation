#!/bin/bash

echo "Cleaning up project..."
rm -f compile_commands.json

echo "Building project..."

PATH="/usr/bin:/home/robertcowher/rosprojects/bumperbot_ws/install/bumperbot_localization/bin:/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin:/snap/bin"

. install/setup.bash

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
	     --merge-install

cp build/compile_commands.json .

