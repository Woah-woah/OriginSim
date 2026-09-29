#!/bin/zsh

# Set user camera follow the robot in Gazebo

gz service -s /gui/follow \
  -r "data: 'red_standard_robot1'" \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  --timeout 1000

gz service -s /gui/follow/offset \
  -r "x: -0.8, y: 0.0, z: 1.2" \
  --reqtype gz.msgs.Vector3d \
  --reptype gz.msgs.Boolean \
  --timeout 1000
