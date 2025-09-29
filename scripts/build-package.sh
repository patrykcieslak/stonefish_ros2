#!/bin/bash
set -e
ROS_WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"

log_info() {
    echo -e "$LOG_PREFIX [INFO] $1"
}

log_error() {
    echo -e "$LOG_PREFIX [ERROR] $1" >&2
}

# ----------------------------- BUILD ROS 2 PACKAGES -----------------------------
build_ros_workspace() {
    log_info "Setting up ROS 2 workspace..."
    cd "$ROS_WORKSPACE"

    log_info "Sourcing ROS 2 setup..."
    . /opt/ros/humble/setup.sh

    log_info "Building stonefish_ros2 package..."
    colcon build --packages-select stonefish_ros2 --symlink-install

    log_info "Stonefish ROS 2 package successfully built."
}

build_ros_workspace

log_info "Build script completed."

