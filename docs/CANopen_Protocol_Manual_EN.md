# TORQEA Joint Actuator — CANopen Protocol Manual

> **Document No.** TQ-PR-0001　**Revision** Rev. 1.0 (EN)
> **Applicable Products** TORQEA Harmonic Joint Actuators — J68H / J83H / J120H
> **Protocol Base** CANopen (CiA 301 / CiA 402, DS402 Motion Control Profile)
> © TORQEA Robotics Co., Ltd. All rights reserved.

---

## Disclaimer

This manual is provided by TORQEA Robotics Co., Ltd. ("TORQEA") for use with TORQEA
joint actuator modules. The information herein is believed to be accurate at the time
of publication. TORQEA reserves the right to update specifications and protocol details
without prior notice. Operating the actuator outside its rated parameters may cause
damage and will void the warranty. For the latest documentation, contact your TORQEA
sales representative.

---

## Table of Contents

1. Overview
2. Physical Layer & Wiring
3. CANopen Fundamentals (NMT / SDO / PDO / Heartbeat)
4. Object Dictionary — Key Objects
5. Operating Modes (Position / Velocity / Current / MIT / Torque)
6. Control Procedure (State Machine & Enable Sequence)
7. PDO Mapping & Configuration
8. Encoder & Unit Conversion
9. Advanced Communication Features
10. Fault Codes & Diagnostics
11. Per-Model Connector Definitions
12. Quick-Start Checklist

---

## 1. Overview

The TORQEA J-series joint actuators are fully integrated robotic actuator modules,
combining a BLDC motor, harmonic reducer, dual absolute encoders, and a FOC driver
in a single housing. Each unit communicates over a **standard CANopen** interface
conforming to the **CiA 402 (DS402) motion control profile**.

Because the actuator follows the standard CANopen protocol, it can be commissioned
and controlled with **any standard CANopen tooling** — including PEAK PCAN-View,
BUSMASTER, python-can / CANopenNode, or ROS / ROS 2 CANopen drivers — **without
requiring any vendor-specific GUI software**. This manual provides everything needed
to integrate the actuator into your own control system.

### Supported Operating Modes
| Mode | DS402 Code | Use Case |
|---|---|---|
| Profile Position (PP) | 1 | Point-to-point positioning |
| Profile Velocity (PV) | 3 | Constant speed control |
| Current / Torque (CST/CST-like) | 4 / 10 | Force / current control |
| Cyclic Sync Position (CSP) | 8 | Real-time trajectory streaming |
| MIT Mode (impedance) | — (vendor extension) | Legged / force-compliant control |

---

## 2. Physical Layer & Wiring

| Item | Specification |
|---|---|
| Bus type | CAN 2.0B (11-bit / 29-bit) — CANopen |
| Default bitrate | 1 Mbps |
| Supported bitrates | 125k / 250k / 500k / 1M bps |
| Termination | 120 Ω resistor required at both ends of the bus |
| Wiring | CAN_H, CAN_L, GND (twisted pair recommended) |
| Power | 48 V DC nominal |

**Bus topology:** multi-drop. Each actuator has a unique Node-ID. Connect CAN_H to
CAN_H, CAN_L to CAN_L across all nodes, with a 120 Ω terminator at each physical end.

> ⚠️ Always power off before changing wiring. Ensure a common ground reference between
> the host controller and all actuators.

---

## 3. CANopen Fundamentals

### 3.1 COB-ID Allocation (default, per CiA 301)
| Object | COB-ID (function code + Node-ID) | Direction |
|---|---|---|
| NMT control | 0x000 | Host → Node |
| SYNC | 0x080 | Host → Node |
| Emergency (EMCY) | 0x080 + Node-ID | Node → Host |
| TPDO1 | 0x180 + Node-ID | Node → Host |
| RPDO1 | 0x200 + Node-ID | Host → Node |
| TPDO2 | 0x280 + Node-ID | Node → Host |
| RPDO2 | 0x300 + Node-ID | Host → Node |
| SDO (tx) | 0x580 + Node-ID | Node → Host |
| SDO (rx) | 0x600 + Node-ID | Host → Node |
| Heartbeat | 0x700 + Node-ID | Node → Host |

### 3.2 NMT — Network Management

To bring a node into operation, the host sends an NMT command (COB-ID 0x000, 2 bytes):

| Command | Byte 0 | Byte 1 (Node-ID, 0 = all) | Effect |
|---|---|---|---|
| Start (Operational) | 0x01 | nn | Enables PDO communication |
| Stop | 0x02 | nn | Stops PDO |
| Pre-Operational | 0x80 | nn | SDO only, no PDO |
| Reset Node | 0x81 | nn | Full reset |
| Reset Communication | 0x82 | nn | Comm reset |

Example — start all nodes:
```
COB-ID 0x000  Data: 01 00
```

### 3.3 SDO — Service Data Object (read/write object dictionary)

SDO is used for configuration (set Node-ID, PID, limits, etc.). Request via
COB-ID `0x600 + Node-ID`, response via `0x580 + Node-ID`.

| Operation | Command byte (CCS) |
|---|---|
| Write 1 byte | 0x2F |
| Write 2 bytes | 0x2B |
| Write 4 bytes | 0x23 |
| Read (request) | 0x40 |
| Read response (4 bytes) | 0x43 |
| Write success response | 0x60 |
| Abort (error) | 0x80 |

**SDO frame layout (8 bytes):**
```
[CCS] [Index LSB] [Index MSB] [Sub-index] [Data0] [Data1] [Data2] [Data3]
```

Example — read Device Name (0x1008):
```
Request : 40 08 10 00 00 00 00 00
```

### 3.4 Heartbeat

Each node periodically transmits a heartbeat (COB-ID `0x700 + Node-ID`, 1 byte state):
| Value | State |
|---|---|
| 0x00 | Boot-up |
| 0x04 | Stopped |
| 0x05 | Operational |
| 0x7F | Pre-Operational |

Producer heartbeat time is set via object `0x1017` (ms).

---

## 4. Object Dictionary — Key Objects

The following objects cover the most common configuration and control tasks. All
objects follow the CiA 301 / CiA 402 standard unless marked "vendor".

### 4.1 Device & Identity
| Index:Sub | Name | Type | Access | Notes |
|---|---|---|---|---|
| 0x1000:00 | Device Type | U32 | RO | DS402 device profile |
| 0x1008:00 | Device Name | String | RO | See note below ★ |
| 0x1009:00 | Hardware Version | String | RO | e.g. V2.0.0 |
| 0x100A:00 | Software Version | String | RO | e.g. V2.0.1 |
| 0x1018 | Identity Object | — | RO | Vendor/Product/Serial |

### 4.2 Communication Configuration
| Index:Sub | Name | Type | Access | Notes |
|---|---|---|---|---|
| 0x1017:00 | Producer Heartbeat Time | U16 | RW | ms |
| Node-ID | Node-ID | U8 | RW | Vendor object — requires save + reset (see datasheet / SDK) |
| CAN Bitrate (0x2540) | CAN Bitrate | U8 | RW | Requires save + reset (see §9.1) |

### 4.3 CiA 402 Control Objects (core)
| Index:Sub | Name | Type | Access | Notes |
|---|---|---|---|---|
| 0x6040:00 | Controlword | U16 | RW | State machine control |
| 0x6041:00 | Statusword | U16 | RO | State feedback |
| 0x6060:00 | Modes of Operation | I8 | RW | Select operating mode |
| 0x6061:00 | Modes of Operation Display | I8 | RO | Active mode |
| 0x6064:00 | Position Actual Value | I32 | RO | Encoder counts |
| 0x606C:00 | Velocity Actual Value | I32 | RO | |
| 0x6077:00 | Torque Actual Value | I16 | RO | per-mille of rated |
| 0x607A:00 | Target Position | I32 | RW | PP mode |
| 0x60FF:00 | Target Velocity | I32 | RW | PV mode |
| 0x6071:00 | Target Torque | I16 | RW | Torque mode |
| 0x6081:00 | Profile Velocity | U32 | RW | PP speed limit |
| 0x6083:00 | Profile Acceleration | U32 | RW | |
| 0x6084:00 | Profile Deceleration | U32 | RW | |

### 4.4 Limits & Protection
| Index:Sub | Name | Notes |
|---|---|---|
| 0x607D:01/02 | Min/Max Position Limit | Soft limits |
| 0x6072:00 | Max Torque | Torque limit |
| 0x6073:00 | Max Current | Current limit |

### 4.5 PID Loop Gains (vendor objects)
Current / Velocity / Position loop Kp & Ki are exposed as vendor objects and can be
tuned over SDO, then saved to Flash.

### 4.6 Save / Restore
| Action | Object | Value |
|---|---|---|
| Save all parameters to Flash | 0x1010:01 | "save" = 0x65766173 |
| Restore factory defaults | 0x1011:01 | "load" = 0x64616F6C |

> After writing Node-ID or bitrate, write 0x1010:01 to save, then reset the node.

---

## 5. Operating Modes

To select a mode, write the mode code to **0x6060**, then read **0x6061** to confirm.

### 5.1 Profile Position (PP) — code 1
1. Set `0x6060 = 1`
2. Set `0x6081` (profile velocity), `0x6083`/`0x6084` (accel/decel)
3. Set `0x607A` (target position)
4. In Controlword, trigger new set-point (bit 4 rising edge): `0x6040 = 0x0F` → `0x1F`

### 5.2 Profile Velocity (PV) — code 3
1. Set `0x6060 = 3`
2. Set `0x60FF` (target velocity)
3. Enable: `0x6040 = 0x0F`

### 5.3 Current / Torque Mode — code 4 / 10
1. Set `0x6060 = 4` (or 10)
2. Set `0x6071` (target torque, per-mille of rated) or current target
3. Enable: `0x6040 = 0x0F`

### 5.4 MIT / Impedance Mode (vendor extension) — mode 6

MIT mode is used for legged robots and force-compliant control. It uses a dedicated
fixed channel for the 5-tuple payload.

**MIT 5-tuple payload (8 bytes, fixed channel `0x500 + Node-ID`):**
The payload packs target position, target velocity, Kp, Kd, and target torque/current.
The host must **continuously stream** this frame at **1–10 ms** intervals; longer gaps
trigger MIT command-timeout protection (≈50 ms → safe damping, ≈500 ms → quick stop).

**MIT entry sequence (example, Node-ID = 1):**
| Step | CAN-ID | DLC | Data (HEX) | Description |
|---|---|---|---|---|
| 1 | 0x000 | 2 | `01 01` | NMT start node 1 |
| 2 | 0x601 | 8 | `2F 60 60 00 06 00 00 00` | Write 0x6060 = 6 (MIT impedance mode) |
| 3 | 0x601 | 8 | `2B 40 60 00 06 00 00 00` | 0x6040 = 0x06 — Ready to switch on |
| 4 | 0x601 | 8 | `2B 40 60 00 07 00 00 00` | 0x6040 = 0x07 — Switch on |
| 5 | 0x601 | 8 | `2B 40 60 00 0F 00 00 00` | 0x6040 = 0x0F — Operation enabled |
| 6 | 0x501 | 8 | `80 00 80 03 E8 14 18 00` | Stream MIT payload periodically |

If the device is in a fault state, first clear it (`0x6040 = 0x80`), then re-run the
enable sequence above.

> Additional fast-control channels are available: single-axis `0x110 + Node-ID` (9 B,
> with control byte) and multi-axis sync `0x210` (up to 6 sub-frames). See §9.

---

## 6. Control Procedure — CiA 402 State Machine

Standard DS402 enable sequence (applies to PP/PV/Current modes):

```
Power on
  → Statusword shows "Switch on disabled"
Controlword 0x06  → "Ready to switch on"
Controlword 0x07  → "Switched on"
Controlword 0x0F  → "Operation enabled"   ← motor now energized
(run your mode)
Controlword 0x80  → fault reset (if faulted)
```

**Statusword (0x6041) key bits:**
| Bits (masked) | State |
|---|---|
| 0x_ _40 | Switch on disabled |
| 0x_ _21 | Ready to switch on |
| 0x_ _23 | Switched on |
| 0x_ _27 | Operation enabled |
| 0x_ _08 | Fault |

---

## 7. PDO Mapping & Configuration

PDOs allow cyclic, low-latency exchange without SDO overhead. Default mappings:

| PDO | COB-ID | Typical Content |
|---|---|---|
| RPDO1 | 0x200 + Node-ID | Controlword + Modes of Operation |
| RPDO2 | 0x300 + Node-ID | Target Position / Velocity |
| TPDO1 | 0x180 + Node-ID | Statusword + Mode display |
| TPDO2 | 0x280 + Node-ID | Position actual + Velocity actual |

**To remap a PDO:**
1. Set node to Pre-Operational (NMT 0x80)
2. Disable PDO (set bit 31 of its COB-ID object)
3. Set mapping count to 0, write new mapping entries, restore count
4. Re-enable PDO, set transmission type (0x_ _02 = on SYNC, 0xFF = async)
5. Return to Operational (NMT 0x01)

---

## 8. Encoder & Unit Conversion

| Item | Value |
|---|---|
| Encoder resolution | 19-bit absolute (input & output side) |
| Counts per revolution | 524,288 (2^19) |
| Gear ratio | 101:1 (model-dependent) |

**Position conversion:**
```
output_degrees = (raw_counts / 524288) × 360 / gear_ratio
```
**Velocity** is reported in encoder counts/s (or RPM, depending on object) — see 0x606C.

---

## 9. Advanced Communication Features

### 9.1 CAN Bitrate Configuration (0x2540)
| Value | Bitrate |
|---|---|
| 0 | 1 Mbps (default) |
| 1 | 500 kbps |
| 2 | 250 kbps |
| 3 | 125 kbps |

Write the desired value to `0x2540`, save (0x1010:01), then reset the node. The new
bitrate takes effect only after reset.

### 9.2 Restore Default Parameters (0x1011:03)
Write `"load"` (0x64616F6C) to `0x1011:03` to restore factory defaults.
**Gate condition:** the request is **rejected while the servo is in Operation-enabled**.
Disable the servo first, then restore.

### 9.3 Electrical Angle Calibration (0x2654:00)
Triggers motor electrical-angle (commutation) calibration. Required only after motor
or encoder service; normally pre-calibrated at the factory.
**Gate condition:** rejected while servo is enabled. Read back `0x2654` for result.

> ⚠️ Do not run angle calibration on an installed, load-bearing joint — the rotor must
> be free to move. Calibrate only on a bench with the output unloaded.

### 9.4 Fault Item Read & Masking (0x4500)
A 32-bit command interface to read active faults and selectively mask/unmask fault
items. Use with care — masking a protective fault can lead to hardware damage.

### 9.5 Historical Fault Read
Historical fault codes can be read back to diagnose intermittent issues (see fault
read objects in §10).

### 9.6 NMT Safe Reset
A reset request (NMT 0x81 / 0x82) enters a **safe reset flow** — it does NOT reboot
immediately:
1. Reset command received
2. If servo is Operation-enabled → reset is **refused**
3. If not enabled → request accepted
4. Pre-reset prep: PWM off; engage brake (if configured)
5. 100 ms delayed reset task starts
6. After 100 ms, re-check enable state; if re-enabled meanwhile, cancel reset
7. Otherwise perform MCU soft reset

After reset, wait for the boot-up heartbeat `0x700 + Node-ID` with data `00`. The node
powers up in **Pre-Operational** — send NMT start (0x01) to resume PDO operation.

---

## 10. Fault Codes & Diagnostics

TORQEA J-series firmware (V2.0+) uses a **unified fault-code scheme**: the same code
means the same fault type across all models. Some models may mask certain faults or
add a few custom ones, but defined codes keep consistent meaning.

### 10.1 Reading Faults
| Object | Meaning |
|---|---|
| 0x603F:00 | Current active vendor-compatible error code (for legacy masters / generic CANopen error display) |
| 0x1001:00 | Error Register (standard CiA 301) |
| 0x1003 | Pre-defined Error Field (history) |

### 10.2 Common Fault Categories
| Category | Typical Cause | Action |
|---|---|---|
| Over-voltage | Bus voltage too high / regen | Add brake resistor; check supply |
| Under-voltage | Supply sag / wiring | Check 48 V supply & cabling |
| Over-current | Mechanical jam / short | Remove obstruction; check wiring |
| Over-temperature | Continuous overload | Reduce duty; improve cooling |
| Encoder error | Cable / EMI | Check encoder cable, shielding |
| Following / velocity error too large | Aggressive command / load | Re-tune PID; lower accel |
| Communication timeout | Bus fault / host stall | Check bus, termination, host loop |

---

## 11. Per-Model Connector Definitions & Parameters

| TORQEA Model | Outer Dia. | Peak Torque | Gear Ratio | Weight |
|---|---|---|---|---|
| **J68H** | 68 mm | 82 Nm (60.5 lb·ft) | 101:1 | 830 g |
| **J83H** | 83 mm | 120 Nm (88.5 lb·ft) | 101:1 | 1,100 g |
| **J120H** | 120 mm | 300 Nm (221 lb·ft) | 101:1 | 2,500 g |

**Connector pinout** (per model) — see the model-specific datasheet, or contact your
TORQEA representative for mechanical drawings and the connector interface specification.

---

## 12. Quick-Start Checklist

```
□ 1. Wire CAN_H / CAN_L / GND. Add 120 Ω terminators at both bus ends.
□ 2. Power on 48 V. Confirm boot-up heartbeat 0x700+ID = 00.
□ 3. (Optional) Set Node-ID and bitrate; save (0x1010:01); reset.
□ 4. NMT Start: send 0x000 = [01 00] to enable all nodes.
□ 5. Select mode: write 0x6060 (1=PP, 3=PV, 4=Current, 6=MIT).
□ 6. Enable: Controlword 0x6040 → 0x06 → 0x07 → 0x0F.
□ 7. Command motion (0x607A / 0x60FF / 0x6071, or MIT stream).
□ 8. Read feedback: 0x6064 (pos), 0x606C (vel), 0x6077 (torque).
□ 9. On fault: read 0x1001 / 0x603F; clear via 0x6040 = 0x80.
```

**Recommended tools (all standard, no vendor software needed):**
- PEAK PCAN-View / BUSMASTER — raw CAN frames
- python-can + canopen (Python library)
- CANopenNode / Lely CANopen (C/C++)
- ROS / ROS 2 canopen_402 driver

> A TORQEA Python SDK with ready-to-use examples is available — contact your TORQEA
> representative or see the Developer Resources section of the product page.

---

*Document prepared by TORQEA Robotics Co., Ltd. for integration support.
For the latest revision and SDK, contact your TORQEA representative.*
