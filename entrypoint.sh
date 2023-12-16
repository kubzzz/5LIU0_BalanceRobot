#!/bin/bash

# enable error messages
set -e

source /opt/ros/humble/setup.bash

# print the provided arguments
echo "Provided arguments: $@"

# execute the provided arguments
exec$@