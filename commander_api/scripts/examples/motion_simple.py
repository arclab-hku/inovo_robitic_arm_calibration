#!/usr/bin/env python3

import rospy
import math
from commander_api.motion_control_client import MotionControlClient, Waypoint, Motion
#from commander_api.gripper_client import GripperControlClient
#gripper_msgs/GripperBasicCommandActionGoal
# from inovopy.robot import InovoRobot

rospy.init_node("motion_simple")

mc = MotionControlClient("default_move_group")

# mc.run(Motion(Waypoint(0.55, -0.25, 0.5, math.pi, 0, 0)))
# mc.run(Motion(Waypoint(0.55, -0.15, 0.5, math.pi, 0, 0)))

# mc.run(Motion(Waypoint(0.0197, 0.407, 0.507, math.pi, 0, (math.pi / 2))))
# mc.run(Motion(Waypoint(-0.03377, 0.338, 0.385, math.pi, 0, (math.pi / 2))))



# mc.run(Motion(Waypoint(0.020, 0.260, 0.550, math.pi, 0, 0)))
#rotate according to the down edge as center

#home point
# mc.run(Motion(Waypoint(0.007, 0.363, 0.340, math.pi, 0, 0.5 * math.pi)))
mc.run(Motion(Waypoint(0.007, 0.363, 0.400, math.pi, 0, 0.5 * math.pi)))

#higher home point for large flower
# mc.run(Motion(Waypoint(0.007, 0.363, 0.440, math.pi, 0, 0.5 * math.pi)))

#Way point
# mc.run(Motion(Waypoint(-0.122, 0.410, 0.077, 0.667 * math.pi, -0.458 * math.pi, 0.344 * math.pi)))

#grasp target
# mc.run(Motion(Waypoint(-0.147, 0.425, 0.056, 0.822*math.pi, -0.420 * math.pi, 0.198 * math.pi)))



print("Done!")
