#!/bin/bash
# Navigate to point 3

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: {frame_id: 'map'} pose: {position: {x: 23.149, y: -7.013, z: 0.205}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
