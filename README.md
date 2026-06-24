<div align="center">

# TORQEA Joint Actuator — Python SDK

**Control TORQEA J-series robotic joint actuators over standard CANopen (CiA 402)**

[![License: MIT](https://img.shields.io/badge/License-MIT-C9A96E.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)
[![CANopen](https://img.shields.io/badge/protocol-CANopen%20CiA%20402-0E0E10.svg)](docs/CANopen_Protocol_Manual_EN.md)

No vendor GUI required · Open source · Works with any `python-can` adapter

</div>

---

A lightweight Python control library for **TORQEA J-series robotic joint
actuators** (J68H / J83H / J120H). Because the actuators speak **standard
CANopen (CiA 402)**, you are never locked into a proprietary tool — control
them directly from Python, or use any CANopen-compliant master.

```python
from torqea import TorqeaActuator

joint = TorqeaActuator(node_id=1, channel="can0", bustype="socketcan")
joint.connect()
joint.enable()
joint.set_position(90.0)        # move output shaft to 90 degrees
print(joint.get_position())     # read actual position
joint.disable()
joint.disconnect()
```

---

## Installation

```bash
# Option A — install directly from GitHub
pip install git+https://github.com/torqea/torqea-joint-sdk.git

# Option B — clone and install editable
git clone https://github.com/torqea/torqea-joint-sdk.git
cd torqea-joint-sdk
pip install -e .
```

The only dependency is [`python-can`](https://python-can.readthedocs.io/).

---

## Hardware setup

1. Wire **CAN_H / CAN_L / GND** between your CAN adapter and the actuator(s).
2. Add a **120 Ω terminator** at each physical end of the bus.
3. Power the actuator with **48 V DC**.
4. Default bitrate is **1 Mbps**. Each actuator needs a unique **Node-ID**.

On Linux with SocketCAN:
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

Works with SocketCAN, PEAK PCAN, Kvaser, USB-CAN/slcan — anything `python-can`
supports.

---

## Quick start

Context-manager style (auto-disable + disconnect on exit):

```python
from torqea import TorqeaActuator

with TorqeaActuator(node_id=1, channel="can0", bustype="socketcan") as joint:
    joint.enable()
    joint.set_position(45, wait=True)
    print("position:", joint.get_position())
```

---

## API overview

| Method | Description |
|---|---|
| `connect()` / `disconnect()` | Open / close the CAN bus |
| `enable(mode=...)` / `disable()` | CiA 402 enable sequence / de-energize |
| `fault_reset()` | Clear an active fault |
| `set_position(deg, speed_rpm=, wait=)` | Profile Position move |
| `set_velocity(rpm)` | Profile Velocity spin |
| `set_torque(per_mille)` | Torque mode |
| `get_position()` / `get_velocity()` / `get_torque()` | Read feedback |
| `get_statusword()` / `is_fault()` / `get_error_register()` | Status & faults |
| `save_parameters()` / `restore_defaults()` | Flash persistence |
| `sdo_read()` / `sdo_write()` | Raw object-dictionary access |

For force-compliant / legged control, use `MitController` (see `examples/04`).

---

## Examples

| File | What it shows |
|---|---|
| [`examples/01_basic_spin.py`](examples/01_basic_spin.py) | Constant-speed spin (velocity mode) |
| [`examples/02_position_control.py`](examples/02_position_control.py) | Absolute position moves + readback |
| [`examples/03_read_telemetry.py`](examples/03_read_telemetry.py) | Live telemetry polling loop |
| [`examples/04_mit_impedance.py`](examples/04_mit_impedance.py) | MIT / impedance streaming (channel family) |
| [`examples/05_multi_axis.py`](examples/05_multi_axis.py) | Multiple actuators on one bus |
| [`examples/06_mit_object_impedance.py`](examples/06_mit_object_impedance.py) | Object-based MIT / impedance (0x3001-0x3006) |

```bash
cd examples
python 01_basic_spin.py
```

---

## Documentation

- 📖 **[CANopen Protocol Manual (EN)](docs/CANopen_Protocol_Manual_EN.md)** —
  full object dictionary, control sequences (position / velocity / torque / MIT),
  PDO mapping, encoder conversion, and fault codes.

---

## Units & conversion

- Position is exposed in **degrees of the output shaft** (after the reducer).
- Defaults: 19-bit absolute encoder (524288 counts/rev), 101:1 gear ratio.
  Override per model:
  ```python
  TorqeaActuator(node_id=1, counts_per_rev=524288, gear_ratio=101.0)
  ```
- Torque is reported in **per-mille of rated torque** (1000 = rated).

---

## Use any standard CANopen tool

Because TORQEA actuators follow CiA 402, you can also use:
PEAK PCAN-View · BUSMASTER · CANopenNode · Lely CANopen · ROS / ROS 2 `canopen_402`.

---

## Support

For datasheets, CAD models, EDS files, and integration help, open an
[Issue](https://github.com/torqea/torqea-joint-sdk/issues) or contact your
TORQEA representative.

## License

[MIT](LICENSE) — use and modify freely.
© 2026 TORQEA Robotics Co., Ltd.
