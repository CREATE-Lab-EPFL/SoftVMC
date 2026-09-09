# LoadCell — Force Sensor Interface

Interface to the Arduino-based load cell used as ground truth for force
measurement in finger experiments (proprioceptive sensing validation,
efficiency identification). `Real_res1_twoLoadCells_HW.ino` is the Arduino
sketch; `arduino_force.py` is the ROS 2 side.

## `arduino_force.py`

Reads `shear,normal` CSV lines from the Arduino over USB serial (raw
**grams**, ×`9.80665e-3` for N — see `ButtonPressing`'s notebooks for the
conversion) and publishes them as a ROS 2 node, always taking the freshest
line available rather than draining the serial buffer in order:

| Topic | Contents |
|---|---|
| `/force_shear` | **negated** shear force (`-force_shear` from the wire) |
| `/force_normal` | normal force, as read |
| `/force` | total magnitude, `sqrt(shear² + normal²)` |

Check the sign before trusting `/force_shear` in an analysis — it is flipped
relative to what the Arduino sends.

Hardcodes the serial port (`/dev/ttyACM0`) and baud rate (115200 baud) at the
top of `ForcePublisher.__init__`; edit those if the load cell enumerates
elsewhere.

## Running

```bash
conda activate finger
python3 arduino_force.py
```

No arguments. Run it alongside a torque node that expects a live load cell
(e.g. `ButtonPressing`'s button sweep) — the topics above are what
`GUI_MultiVMC_Torque.py` subscribes to for the `force`/`force_age` fields it
logs.
