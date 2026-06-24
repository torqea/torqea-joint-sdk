"""
TORQEA Joint Actuator — Python SDK
==================================

A lightweight Python control library for TORQEA J-series robotic joint
actuators (J68H / J83H / J120H) over standard CANopen (CiA 402).

Built on top of `python-can`. Works with any SocketCAN, PEAK PCAN,
Kvaser, or USB-CAN adapter supported by python-can.

Quick start
-----------
    from torqea import TorqeaActuator

    joint = TorqeaActuator(node_id=1, channel="can0", bustype="socketcan")
    joint.connect()
    joint.enable()
    joint.set_position(90.0)        # rotate output to 90 degrees
    print(joint.get_position())     # read actual position
    joint.disable()
    joint.disconnect()

Requirements
------------
    pip install python-can

License: MIT (you may freely use and modify this SDK).
(c) TORQEA Robotics Co., Ltd.
"""

import time
import struct
import logging

try:
    import can  # python-can
except ImportError as e:
    raise ImportError(
        "python-can is required. Install it with:  pip install python-can"
    ) from e

log = logging.getLogger("torqea")

# ---------------------------------------------------------------------------
# CiA 402 standard object dictionary indices
# ---------------------------------------------------------------------------
OD_CONTROLWORD        = 0x6040
OD_STATUSWORD         = 0x6041
OD_MODES_OF_OPERATION = 0x6060
OD_MODES_DISPLAY      = 0x6061
OD_POSITION_ACTUAL    = 0x6064
OD_VELOCITY_ACTUAL    = 0x606C
OD_TORQUE_ACTUAL      = 0x6077
OD_TARGET_POSITION    = 0x607A
OD_TARGET_VELOCITY    = 0x60FF
OD_TARGET_TORQUE      = 0x6071
OD_PROFILE_VELOCITY   = 0x6081
OD_PROFILE_ACCEL      = 0x6083
OD_PROFILE_DECEL      = 0x6084
OD_MAX_TORQUE         = 0x6072
OD_ERROR_REGISTER     = 0x1001
OD_STORE_PARAMS       = 0x1010   # sub 1 = "save"
OD_RESTORE_PARAMS     = 0x1011   # sub 1 = "load"
OD_DEVICE_NAME        = 0x1008

# Operating modes (0x6060 values)
# NOTE: TORQEA actuators come from more than one production line. The MIT/impedance
#       implementation differs by family:
#         - "Channel" family: MIT mode code 6, streamed 5-tuple on 0x500+ID
#                              (use MitController)
#         - "Object"  family: MIT mode code 0x0B, written to objects 0x3001-0x3006
#                              (use MitObjectController)
#       Position / velocity / torque modes are identical (standard CiA 402) across all.
MODE_PROFILE_POSITION = 1
MODE_PROFILE_VELOCITY = 3
MODE_TORQUE           = 4
MODE_MIT              = 6      # channel-family MIT (streamed)
MODE_MIT_OBJECT       = 0x0B   # object-family MIT (0x3001-0x3006)
MODE_CSP              = 8

# ---------------------------------------------------------------------------
# Vendor objects — communication & configuration (confirmed V1.0 protocol)
# ---------------------------------------------------------------------------
OD_NODE_ID            = 0x2530   # Node-ID setting (save + reset required)
OD_CANFD_BAUD         = 0x2540   # CANFD data-segment bitrate (1=5M 2=4M 3=2M 4=1M)
OD_ZERO_CALIB         = 0x2531   # Zero calibration: sub1=set zero, sub2=reset
OD_WATCHDOG_OFF       = 0x2650   # Write 0x01 to disable 500 ms watchdog + soft limits
OD_FLASH_SAVE_PID     = 0x2539   # Write 0x01 to save PID + current limit to flash
# PID gains (vendor, all sub 0, unit TBD — write then save via OD_FLASH_SAVE_PID)
OD_CUR_KP             = 0x2532
OD_CUR_KI             = 0x2533
OD_VEL_KP             = 0x2534
OD_VEL_KI             = 0x2535
OD_POS_KP             = 0x2536
OD_POS_KI             = 0x2537
OD_MAX_CURRENT_CFG    = 0x2538   # max output current config (mA)
# Temperature readback (vendor)
OD_TEMP_MOS           = 0x2662   # MOS temperature, unit 0.1°C
OD_TEMP_MOTOR         = 0x2663   # Motor coil temperature, unit 0.1°C

# Fault code bit-mask (from 0x603F, confirmed V1.0 protocol §4.3)
FAULT_CODES = {
    0x0001: "Over-voltage",
    0x0002: "Under-voltage",
    0x0004: "Over-temperature",
    0x0008: "Motor stall / locked rotor",
    0x0010: "Overload",
    0x0020: "Current sampling error",
    0x0040: "Positive software limit reached",
    0x0080: "Negative software limit reached",
    0x0100: "Encoder communication timeout",
    0x0200: "Motor over-speed",
    0x0400: "Electrical angle init failed (power-on)",
    0x1000: "Position following error too large",
    0x2000: "Encoder fault",
}

# Object-family MIT objects (vendor extension)
OD_MIT_FF_CURRENT     = 0x3001   # feed-forward torque current, 0.001 A
OD_MIT_TARGET_POS     = 0x3002   # target position, pulse
OD_MIT_MAX_CURRENT    = 0x3003   # max output current, 0.001 A
OD_MIT_TARGET_VEL     = 0x3004   # target velocity, pulse/s
OD_MIT_STIFFNESS      = 0x3005   # position stiffness Kp, 0.001 A/rad
OD_MIT_DAMPING        = 0x3006   # velocity damping Kd, 0.001 A/(rad/s)

# Controlword command sequence (CiA 402 state machine)
CW_SHUTDOWN        = 0x06   # -> Ready to switch on
CW_SWITCH_ON       = 0x07   # -> Switched on
CW_ENABLE          = 0x0F   # -> Operation enabled
CW_NEW_SETPOINT    = 0x1F   # 0x0F | bit4 (PP new set-point trigger)
CW_FAULT_RESET     = 0x80   # clear fault

# NMT commands
NMT_START = 0x01
NMT_STOP  = 0x02
NMT_PREOP = 0x80
NMT_RESET = 0x81

# Save / restore magic
MAGIC_SAVE = 0x65766173  # "save"
MAGIC_LOAD = 0x64616F6C  # "load"

# ---------------------------------------------------------------------------
# Encoder / unit conversion
# NOTE: counts-per-rev and gear ratio are per-model. Defaults below match the
#       J-series 19-bit absolute encoder + 101:1 harmonic reducer. Override at
#       construction time if your model uses a different ratio (see the model
#       datasheet or the CANopen Protocol Manual §8 for exact values).
# ---------------------------------------------------------------------------
COUNTS_PER_REV_DEFAULT = 65536    # 16-bit output-shaft encoder (65536 cnt/rev)
GEAR_RATIO_DEFAULT     = 1.0      # 0x6064 already reflects output shaft; no further ratio needed


class TorqeaError(Exception):
    """Base exception for TORQEA SDK."""


class SdoError(TorqeaError):
    """Raised when an SDO transfer is aborted by the node."""


class TorqeaActuator:
    """Control a single TORQEA joint actuator over CANopen."""

    def __init__(self, node_id, channel="can0", bustype="socketcan",
                 bitrate=1_000_000, counts_per_rev=COUNTS_PER_REV_DEFAULT,
                 gear_ratio=GEAR_RATIO_DEFAULT, sdo_timeout=0.5):
        if not (1 <= node_id <= 127):
            raise ValueError("node_id must be 1..127")
        self.node_id = node_id
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.counts_per_rev = counts_per_rev
        self.gear_ratio = gear_ratio
        self.sdo_timeout = sdo_timeout
        self.bus = None

    # -- connection ---------------------------------------------------------
    def connect(self):
        """Open the CAN bus."""
        self.bus = can.interface.Bus(
            channel=self.channel, bustype=self.bustype, bitrate=self.bitrate
        )
        log.info("Connected to %s (%s) node %d", self.channel, self.bustype, self.node_id)
        return self

    def disconnect(self):
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        try:
            self.disable()
        finally:
            self.disconnect()

    # -- low level: raw CAN -------------------------------------------------
    def _send(self, cob_id, data):
        msg = can.Message(arbitration_id=cob_id, data=bytes(data), is_extended_id=False)
        self.bus.send(msg)

    def _recv(self, expected_cob_id, timeout):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.bus.recv(timeout=end - time.time())
            if msg is not None and msg.arbitration_id == expected_cob_id:
                return msg
        return None

    # -- NMT ----------------------------------------------------------------
    def nmt(self, command):
        """Send an NMT command to this node."""
        self._send(0x000, [command, self.node_id])

    # -- SDO ----------------------------------------------------------------
    def sdo_write(self, index, subindex, value, size):
        """Write `size` bytes (1/2/4) to object dictionary index:subindex."""
        ccs = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
        payload = list(struct.pack("<i", value)[:size]) + [0] * (4 - size)
        frame = [ccs, index & 0xFF, (index >> 8) & 0xFF, subindex] + payload
        self._send(0x600 + self.node_id, frame)
        resp = self._recv(0x580 + self.node_id, self.sdo_timeout)
        if resp is None:
            raise SdoError(f"SDO write timeout (0x{index:04X}:{subindex})")
        if resp.data[0] == 0x80:
            code = struct.unpack("<I", bytes(resp.data[4:8]))[0]
            raise SdoError(f"SDO write aborted 0x{index:04X}:{subindex} code=0x{code:08X}")
        return True

    def sdo_read(self, index, subindex, signed=True):
        """Read a numeric object. Returns int."""
        frame = [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
        self._send(0x600 + self.node_id, frame)
        resp = self._recv(0x580 + self.node_id, self.sdo_timeout)
        if resp is None:
            raise SdoError(f"SDO read timeout (0x{index:04X}:{subindex})")
        if resp.data[0] == 0x80:
            code = struct.unpack("<I", bytes(resp.data[4:8]))[0]
            raise SdoError(f"SDO read aborted 0x{index:04X}:{subindex} code=0x{code:08X}")
        raw = bytes(resp.data[4:8])
        return struct.unpack("<i" if signed else "<I", raw)[0]

    # -- state machine ------------------------------------------------------
    def enable(self, mode=MODE_PROFILE_POSITION):
        """Bring the actuator to Operation-enabled in the given mode."""
        self.nmt(NMT_START)
        time.sleep(0.05)
        self.set_mode(mode)
        self.sdo_write(OD_CONTROLWORD, 0, CW_SHUTDOWN, 2)
        self.sdo_write(OD_CONTROLWORD, 0, CW_SWITCH_ON, 2)
        self.sdo_write(OD_CONTROLWORD, 0, CW_ENABLE, 2)
        log.info("Node %d enabled in mode %d", self.node_id, mode)

    def disable(self):
        """Disable the actuator (motor de-energized)."""
        if self.bus is None:
            return
        try:
            self.sdo_write(OD_CONTROLWORD, 0, CW_SHUTDOWN, 2)
        except TorqeaError:
            pass

    def fault_reset(self):
        """Clear an active fault."""
        self.sdo_write(OD_CONTROLWORD, 0, CW_FAULT_RESET, 2)
        time.sleep(0.05)

    def set_mode(self, mode):
        self.sdo_write(OD_MODES_OF_OPERATION, 0, mode, 1)

    def get_mode(self):
        return self.sdo_read(OD_MODES_DISPLAY, 0)

    # -- conversions --------------------------------------------------------
    def deg_to_counts(self, deg):
        return int(deg / 360.0 * self.counts_per_rev * self.gear_ratio)

    def counts_to_deg(self, counts):
        return counts / (self.counts_per_rev * self.gear_ratio) * 360.0

    # -- profile position ---------------------------------------------------
    def set_position(self, degrees, speed_rpm=10, accel=1000, decel=1000, wait=False):
        """Move output shaft to absolute `degrees` (Profile Position mode)."""
        self.set_mode(MODE_PROFILE_POSITION)
        self.sdo_write(OD_PROFILE_VELOCITY, 0, int(speed_rpm), 4)
        self.sdo_write(OD_PROFILE_ACCEL, 0, int(accel), 4)
        self.sdo_write(OD_PROFILE_DECEL, 0, int(decel), 4)
        self.sdo_write(OD_TARGET_POSITION, 0, self.deg_to_counts(degrees), 4)
        # rising edge on bit4 to latch the new set-point
        self.sdo_write(OD_CONTROLWORD, 0, CW_ENABLE, 2)
        self.sdo_write(OD_CONTROLWORD, 0, CW_NEW_SETPOINT, 2)
        if wait:
            self.wait_target_reached(degrees)

    def wait_target_reached(self, target_deg, tol_deg=0.5, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            if abs(self.get_position() - target_deg) <= tol_deg:
                return True
            time.sleep(0.05)
        return False

    # -- profile velocity ---------------------------------------------------
    def set_velocity(self, rpm):
        """Spin continuously at `rpm` (Profile Velocity mode)."""
        self.set_mode(MODE_PROFILE_VELOCITY)
        self.sdo_write(OD_TARGET_VELOCITY, 0, int(rpm), 4)
        self.sdo_write(OD_CONTROLWORD, 0, CW_ENABLE, 2)

    # -- torque -------------------------------------------------------------
    def set_torque(self, per_mille):
        """Apply torque target in per-mille of rated torque (Torque mode).
        Example: 100 = 10.0% of rated torque."""
        self.set_mode(MODE_TORQUE)
        self.sdo_write(OD_TARGET_TORQUE, 0, int(per_mille), 2)
        self.sdo_write(OD_CONTROLWORD, 0, CW_ENABLE, 2)

    # -- feedback -----------------------------------------------------------
    def get_position(self):
        """Actual output position in degrees."""
        return self.counts_to_deg(self.sdo_read(OD_POSITION_ACTUAL, 0))

    def get_velocity(self):
        """Actual velocity (raw counts/s or RPM, model-dependent)."""
        return self.sdo_read(OD_VELOCITY_ACTUAL, 0)

    def get_torque(self):
        """Actual torque in per-mille of rated torque."""
        return self.sdo_read(OD_TORQUE_ACTUAL, 0)

    def get_statusword(self):
        return self.sdo_read(OD_STATUSWORD, 0, signed=False)

    def get_error_register(self):
        return self.sdo_read(OD_ERROR_REGISTER, 0, signed=False)

    def is_fault(self):
        return bool(self.get_statusword() & 0x0008)

    def decode_fault(self, fault_code=None):
        """Return list of active fault strings from 0x603F bit-mask.
        If fault_code is None, reads 0x603F automatically."""
        if fault_code is None:
            fault_code = self.sdo_read(0x603F, 0, signed=False)
        return [msg for bit, msg in FAULT_CODES.items() if fault_code & bit]

    def get_temperature(self, sensor="mos"):
        """Read MOS or motor coil temperature in degrees Celsius.
        sensor: 'mos' | 'motor'"""
        idx = OD_TEMP_MOS if sensor == "mos" else OD_TEMP_MOTOR
        raw = self.sdo_read(idx, 0)
        return raw / 10.0  # unit is 0.1°C

    def get_device_name(self):
        """Read the device name string object (0x1008)."""
        frame = [0x40, OD_DEVICE_NAME & 0xFF, (OD_DEVICE_NAME >> 8) & 0xFF, 0, 0, 0, 0, 0]
        self._send(0x600 + self.node_id, frame)
        resp = self._recv(0x580 + self.node_id, self.sdo_timeout)
        if resp is None:
            return ""
        return bytes(resp.data[4:8]).split(b"\x00")[0].decode("ascii", "ignore")

    # -- persistence --------------------------------------------------------
    def save_parameters(self):
        """Persist current parameters to flash. Reset required for some params."""
        self.sdo_write(OD_STORE_PARAMS, 1, MAGIC_SAVE, 4)

    def restore_defaults(self):
        """Restore factory defaults (servo must be disabled first)."""
        self.disable()
        time.sleep(0.05)
        self.sdo_write(OD_RESTORE_PARAMS, 1, MAGIC_LOAD, 4)


# ---------------------------------------------------------------------------
# MIT / impedance mode helper
# ---------------------------------------------------------------------------
class MitController:
    """
    MIT (impedance) mode streaming helper for legged / force-compliant control.

    The 5-tuple (position, velocity, kp, kd, torque) is packed into an 8-byte
    payload and streamed on the fixed channel 0x500 + node_id at 1-10 ms.
    Longer gaps trigger the on-board timeout protection.

    The bit-width packing below follows the common MIT motor convention
    (16/12/12/12/12 bits). Set the field ranges (P_MIN/P_MAX etc.) to match
    your model — see the CANopen Protocol Manual §5.4 for the values.
    """

    # Field ranges — set these to match your model (see Protocol Manual §5.4).
    P_MIN, P_MAX = -12.5, 12.5      # rad
    V_MIN, V_MAX = -45.0, 45.0      # rad/s
    KP_MIN, KP_MAX = 0.0, 500.0
    KD_MIN, KD_MAX = 0.0, 5.0
    T_MIN, T_MAX = -18.0, 18.0      # Nm

    def __init__(self, actuator: TorqeaActuator):
        self.act = actuator

    @staticmethod
    def _float_to_uint(x, lo, hi, bits):
        x = max(lo, min(hi, x))
        span = hi - lo
        return int((x - lo) * ((1 << bits) - 1) / span)

    def enter(self):
        """Enter MIT mode via the standard CANopen control plane."""
        self.act.nmt(NMT_START)
        time.sleep(0.05)
        self.act.set_mode(MODE_MIT)
        self.act.sdo_write(OD_CONTROLWORD, 0, CW_SHUTDOWN, 2)
        self.act.sdo_write(OD_CONTROLWORD, 0, CW_SWITCH_ON, 2)
        self.act.sdo_write(OD_CONTROLWORD, 0, CW_ENABLE, 2)

    def pack(self, p, v, kp, kd, t):
        """Pack the 5-tuple into the 8-byte MIT payload."""
        pi = self._float_to_uint(p, self.P_MIN, self.P_MAX, 16)
        vi = self._float_to_uint(v, self.V_MIN, self.V_MAX, 12)
        kpi = self._float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kdi = self._float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        ti = self._float_to_uint(t, self.T_MIN, self.T_MAX, 12)
        return bytes([
            (pi >> 8) & 0xFF,
            pi & 0xFF,
            (vi >> 4) & 0xFF,
            ((vi & 0xF) << 4) | ((kpi >> 8) & 0xF),
            kpi & 0xFF,
            (kdi >> 4) & 0xFF,
            ((kdi & 0xF) << 4) | ((ti >> 8) & 0xF),
            ti & 0xFF,
        ])

    def send(self, p, v, kp, kd, t):
        """Send one MIT command frame (call this every 1-10 ms)."""
        payload = self.pack(p, v, kp, kd, t)
        self.act._send(0x500 + self.act.node_id, payload)


# ---------------------------------------------------------------------------
# Object-family MIT / impedance controller
# ---------------------------------------------------------------------------
class MitObjectController:
    """
    Object-based MIT (impedance / force-position hybrid) controller.

    For TORQEA models whose MIT mode is selected with operating-mode code 0x0B
    and driven through object dictionary entries 0x3001-0x3006 (rather than a
    streamed 0x500 channel). The on-board driver computes output current as:

        I = Kp * (target_pos - actual_pos)
          + Kd * (target_vel - actual_vel)
          + I_feedforward

    Units (per the protocol manual):
        feed-forward current : 0.001 A   (1000 = 1.000 A)
        target position      : pulse
        max output current   : 0.001 A
        target velocity      : pulse/s
        stiffness  Kp        : 0.001 A/rad
        damping    Kd        : 0.001 A/(rad/s)

    Always set the zero position before running MIT. Avoid large position steps
    relative to the current position — they can cause over-current.
    """

    def __init__(self, actuator: TorqeaActuator):
        self.act = actuator

    def enter(self):
        """Select object-family MIT mode (0x0B) and run the enable sequence."""
        self.act.nmt(NMT_START)
        time.sleep(0.05)
        self.act.set_mode(MODE_MIT_OBJECT)
        self.act.sdo_write(OD_CONTROLWORD, 0, CW_SHUTDOWN, 2)
        self.act.sdo_write(OD_CONTROLWORD, 0, CW_SWITCH_ON, 2)
        self.act.sdo_write(OD_CONTROLWORD, 0, CW_ENABLE, 2)

    def set_command(self, target_pos_pulse, target_vel_pulse_s=0,
                    kp_milli=0, kd_milli=0, ff_current_milli=0,
                    max_current_milli=None):
        """
        Write one MIT command set via objects 0x3001-0x3006.

        All values are in the raw integer units listed in the class docstring
        (milli-units where noted). Unlike the channel family, this does not need
        a continuous high-rate stream — write when the target changes.
        """
        if max_current_milli is not None:
            self.act.sdo_write(OD_MIT_MAX_CURRENT, 0, int(max_current_milli), 4)
        self.act.sdo_write(OD_MIT_FF_CURRENT, 0, int(ff_current_milli), 4)
        self.act.sdo_write(OD_MIT_STIFFNESS, 0, int(kp_milli), 4)
        self.act.sdo_write(OD_MIT_DAMPING, 0, int(kd_milli), 4)
        self.act.sdo_write(OD_MIT_TARGET_VEL, 0, int(target_vel_pulse_s), 4)
        # target position written last so it latches with the gains above
        self.act.sdo_write(OD_MIT_TARGET_POS, 0, int(target_pos_pulse), 4)


__all__ = [
    "TorqeaActuator", "MitController", "MitObjectController",
    "TorqeaError", "SdoError",
]
