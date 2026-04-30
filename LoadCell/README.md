# LoadCell — Force Sensor Interface

Interface to the Arduino-based load cell used as ground truth for force measurement
in finger experiments (proprioceptive sensing validation, efficiency identification).

## Files

### `arduino_force.py`

Reads force measurements from the load cell over USB serial and publishes them
to a ROS2 topic or logs them to file for later analysis.
