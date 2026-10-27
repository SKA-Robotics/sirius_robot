#!/bin/bash
# Navigate to point 7

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: {frame_id: 'map'}, pose: {position: {x: 7.682, y: 2.108, z: -0.163}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
