"""
Example 06 — Object-based MIT / impedance mode
==============================================
For TORQEA models whose MIT mode is driven through object dictionary entries
0x3001-0x3006 (operating-mode code 0x0B), rather than a streamed 0x500 channel.

Unlike the streamed channel family (see example 04), this family does not need
a continuous high-rate loop — write a command set whenever the target changes.

>> Set the zero position before running MIT, and avoid large position steps
   relative to the current position (can cause over-current). See Protocol Manual.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
from torqea import TorqeaActuator, MitObjectController

joint = TorqeaActuator(node_id=1, channel="can0", bustype="socketcan")
joint.connect()

mit = MitObjectController(joint)
mit.enter()

# Move to a target with a soft impedance:
#   target 10000 pulse, gentle stiffness, small damping, no feed-forward,
#   max output current limited to 20.000 A for safety.
print("Commanding object-based MIT move...")
mit.set_command(
    target_pos_pulse=10000,
    target_vel_pulse_s=1000,
    kp_milli=1,            # 0.001 A/rad
    kd_milli=0,            # 0.000 A/(rad/s)
    ff_current_milli=0,    # no feed-forward
    max_current_milli=20000,  # 20.000 A limit
)

time.sleep(2)
print("Position:", joint.get_position())

joint.disable()
joint.disconnect()
print("Done.")
