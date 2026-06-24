"""
Example 02 — Absolute position control (Profile Position mode)
=============================================================
Move the output shaft to a series of absolute angles and read back position.
"""
import time
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from torqea import TorqeaActuator

joint = TorqeaActuator(node_id=1, channel="can0", bustype="socketcan")
joint.connect()
joint.enable()

for target in (0, 90, 180, 90, 0):
    print(f"Moving to {target} deg ...")
    joint.set_position(target, speed_rpm=15, wait=True)
    print(f"  actual position = {joint.get_position():.2f} deg")
    time.sleep(0.3)

joint.disable()
joint.disconnect()
print("Done.")
