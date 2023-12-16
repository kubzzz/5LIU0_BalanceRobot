#!/bin/bash

# enable error signals
set -e

# source the ros installation
source /opt/ros/humble/setup.bash

# print argument
echo "Provided arguments: $@"

# execute the provide argument
exec $@