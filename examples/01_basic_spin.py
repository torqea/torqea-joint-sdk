"""
Example 01 — Basic spin (Profile Velocity mode)
================================================
Connect to a single TORQEA actuator and spin it at a constant speed.

Edit `CHANNEL` / `BUSTYPE` / `NODE_ID` to match your setup, then run:
    python 01_basic_spin.py
"""
import time
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from torqea import TorqeaActuator

CHANNEL = "can0"          # e.g. "can0" (SocketCAN), "PCAN_USBBUS1" (PEAK)
BUSTYPE = "socketcan"     # e.g. "socketcan", "pcan", "kvaser", "slcan"
NODE_ID = 1

with TorqeaActuator(node_id=NODE_ID, channel=CHANNEL, bustype=BUSTYPE) as joint:
    joint.enable()
    print("Spinning at 20 RPM for 3 seconds...")
    joint.set_velocity(20)
    time.sleep(3)

    print("Reversing at -20 RPM for 3 seconds...")
    joint.set_velocity(-20)
    time.sleep(3)

    print("Stopping.")
    joint.set_velocity(0)
    time.sleep(0.5)
# `with` block auto-disables and disconnects on exit.
print("Done.")
