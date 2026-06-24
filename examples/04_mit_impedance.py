"""
Example 04 — MIT / impedance mode streaming
===========================================
Stream the MIT 5-tuple (position, velocity, kp, kd, torque) for
force-compliant / legged-robot control.

The host MUST stream continuously at 1-10 ms. Longer gaps trigger
the on-board MIT timeout protection (safe damping -> quick stop).

Note: set the MIT field ranges (P_MIN/P_MAX etc.) in torqea.py to match
your model before applying high gains. See the Protocol Manual §5.4.
"""
import time
import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from torqea import TorqeaActuator, MitController

joint = TorqeaActuator(node_id=1, channel="can0", bustype="socketcan")
joint.connect()

mit = MitController(joint)
mit.enter()

print("Streaming a gentle sine-wave position target for 5 s (Ctrl-C to stop)...")
t0 = time.time()
try:
    while time.time() - t0 < 5.0:
        t = time.time() - t0
        target_rad = 0.3 * math.sin(2 * math.pi * 0.5 * t)   # +/-0.3 rad, 0.5 Hz
        # position-hold impedance: moderate kp, small kd, zero feed-forward torque
        mit.send(p=target_rad, v=0.0, kp=30.0, kd=1.0, t=0.0)
        time.sleep(0.002)   # 2 ms loop
except KeyboardInterrupt:
    pass
finally:
    joint.disable()
    joint.disconnect()
print("Done.")
