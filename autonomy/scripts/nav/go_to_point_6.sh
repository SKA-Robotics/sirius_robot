#!/bin/bash
# Navigate to point 6

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: {frame_id: 'map'} pose: {position: {x: 8.197, y: 7.675, z: -0.179}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
