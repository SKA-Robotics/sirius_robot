#!/bin/bash
# Navigate to point 5

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: {frame_id: 'map'} pose: {position: {x: 19.034, y: 5.339, z: -0.259}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
