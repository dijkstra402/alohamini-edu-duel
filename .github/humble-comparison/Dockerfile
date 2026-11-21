FROM ros:humble-ros-base-jammy@sha256:a18afd4a42ad603579d03b9012385dfeaf2e848423f0a6c41699766e7849c86c
RUN apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-humble-nav2-amcl ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-collision-monitor ros-humble-robot-localization \
    ros-humble-robot-state-publisher ros-humble-rmw-cyclonedds-cpp \
    ros-humble-nav2-controller ros-humble-nav2-smoother ros-humble-nav2-planner \
    ros-humble-nav2-behaviors ros-humble-nav2-bt-navigator ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-navfn-planner ros-humble-dwb-core ros-humble-dwb-critics ros-humble-dwb-plugins \
    ros-humble-control-msgs python3-zmq python3-pytest python3-numpy python3-yaml \
    && rm -rf /var/lib/apt/lists/*
COPY build-copy.sh build-install.sh guest-probe.py repeat-validation.sh /runner/
