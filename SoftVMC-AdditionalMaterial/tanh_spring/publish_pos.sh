#!/bin/bash

# Helper function to publish and hold
publish_position() {
  local x=$1 y=$2 z=$3 duration=$4
  echo "Publishing [$x, $y, $z] for ${duration}s..."
  ros2 topic pub /VC_ref_pos std_msgs/Float64MultiArray "{data: [$x, $y, $z]}" &
  PID=$!
  sleep $duration
  kill $PID 2>/dev/null
  sleep 1  # ADD THIS - let ROS2 context cleanup
}

# Sequence
publsih_position -0.05 0.0 0.4 4
publish_position -0.15 0.0 0.4 4
publish_position -0.3 0.0 0.4 4
publish_position -0.6 0.0 0.4 4

echo "Done!"
