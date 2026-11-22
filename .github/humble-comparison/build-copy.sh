#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/verify/ros-logs
mkdir -p /tmp/duel-source/ros2/src /tmp/duel-source/simulation /tmp/duel-colcon
for package in alohamini_core alohamini_bridge alohamini_description alohamini_navigation alohamini_sensors alohamini_duel; do
    cp -a "/checkout/ros2/src/$package" /tmp/duel-source/ros2/src/
done
cp -a /checkout/simulation/navigation /tmp/duel-source/simulation/
cp -a /checkout/ros2/vendor/humble /tmp/duel-source/vendor
cp -a /checkout/LICENSE /checkout/NOTICE /checkout/models /checkout/src /tmp/duel-source/
