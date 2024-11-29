#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/jazzy/setup.bash"
source "/our-workspace/install/setup.bash"
exec "$@"
