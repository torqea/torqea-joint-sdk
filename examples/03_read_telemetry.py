"""
Example 03 — Read live telemetry
================================
Continuously poll position, velocity, torque, and fault state.
Useful for monitoring or building a simple dashboard.
"""
import time
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from torqea import TorqeaActuator

joint = TorqeaActuator(node_id=1, channel="can0", bustype="socketcan")
joint.connect()

# Optional: read device identity (see manual §3.3)
print("Device name:", joint.get_device_name())

print(f"{'time':>6} | {'pos(deg)':>10} | {'vel':>8} | {'torque(‰)':>10} | fault")
print("-" * 55)
t0 = time.time()
try:
    while True:
        pos = joint.get_position()
        vel = joint.get_velocity()
        tor = joint.get_torque()
        flt = joint.is_fault()
        print(f"{time.time()-t0:6.1f} | {pos:10.2f} | {vel:8d} | {tor:10d} | {flt}")
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    joint.disconnect()
