# LoadCell — Force Sensor Interface

Interface to the Arduino-based load cell used as ground truth for force
measurement in finger experiments (proprioceptive sensing validation,
efficiency identification). `Real_res1_twoLoadCells_HW/Real_res1_twoLoadCells_HW.ino`
is the Arduino sketch (two HX711 load cells; it prints `Force1(g),Force2(g)`,
which `arduino_force.py` reads as `shear,normal`); `arduino_force.py` is the
ROS 2 side.

## `arduino_force.py`

Reads `shear,normal` CSV lines from the Arduino over USB serial (raw
**grams**, ×`9.80665e-3` for N — see `ButtonPressing`'s notebooks for the
conversion) and publishes them as a ROS 2 node, always taking the freshest
line available rather than draining the serial buffer in order (a 200 Hz timer;
it warns and flushes if data piles up):

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
(e.g. `ButtonPressing`'s button sweep, or `TanhSpring`'s jump tests).
`GUI_MultiVMC_Torque.py` subscribes to **`/force_shear` only** — not `/force`
or `/force_normal` — and logs it as `force` (with its age as `force_age`). That
is the **negated** shear in raw grams: multiply by `9.80665e-3` for newtons; the
sign is the opposite of the Arduino's. Without this node running, `force` is
identically 0.

(`ButtonPressing/sensor/fdc1004_single_taxel/` is a different sensor's sketch — a
capacitive taxel — and nothing here or there reads it.)
