#!/bin/bash

# Helper function to publish and hold
publish_position() {
  local x=$1 y=$2 z=$3 duration=$4
  echo "Publishing [$x, $y, $z] for ${duration}s..."
  ros2 topic pub /VC_ref_pos std_msgs/Float64MultiArray "{data: [$x, $y, $z]}" &
  PID=$!
  sleep $duration
  kill $PID 2>/dev/null
}

# Sequence
publish_position 0.2 0.2 0.4 2
publish_position 0.4 0.4 0.4 2
publish_position 0.6 0.6 0.3 2

echo "Done!"
