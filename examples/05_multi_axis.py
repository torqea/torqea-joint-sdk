"""
Example 05 — Multi-axis bus (read all joints on the bus)
========================================================
Enumerate several actuators sharing one CAN bus and read their state.
Each actuator must have a unique Node-ID (set via SDO + save + reset).
"""
import time
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from torqea import TorqeaActuator, SdoError

CHANNEL = "can0"
BUSTYPE = "socketcan"
NODE_IDS = [1, 2, 3]      # the joints on your bus

# Open one bus and share it across all actuator objects
joints = []
first = TorqeaActuator(node_id=NODE_IDS[0], channel=CHANNEL, bustype=BUSTYPE)
first.connect()
joints.append(first)
for nid in NODE_IDS[1:]:
    j = TorqeaActuator(node_id=nid, channel=CHANNEL, bustype=BUSTYPE)
    j.bus = first.bus          # reuse the same bus instance
    joints.append(j)

print("Scanning bus...")
for j in joints:
    try:
        pos = j.get_position()
        sw = j.get_statusword()
        print(f"  Node {j.node_id}: pos={pos:8.2f} deg  statusword=0x{sw:04X}")
    except SdoError:
        print(f"  Node {j.node_id}: no response (not present?)")

first.disconnect()
print("Done.")
