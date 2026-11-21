cd /tmp/duel-colcon
colcon --log-base /verify/colcon-logs build \
    --base-paths /tmp/duel-source/ros2/src /tmp/duel-source/simulation/navigation /tmp/duel-source/vendor \
    --packages-select tf2 alohamini_core alohamini_bridge alohamini_description \
        alohamini_navigation alohamini_sensors alohamini_nav_sim alohamini_duel \
    --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
