#!/bin/bash
set -e

envsubst < dependencies.repos | vcs import src --recursive
sudo apt-get update
rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO
