#!/bin/bash
source ~/.bashrc

_CUR_PATH=$(pwd)

rosdep install -i --from-path src --rosdistro foxy -y
colcon build
cd $_CUR_PATH

source install/local_setup.bash

$@