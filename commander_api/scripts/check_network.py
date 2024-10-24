#!/usr/bin/env python

# A simple script that attempts to connect to the "move" action server on an Inovo robot and reports whether it could connect or not.

import rospy
import actionlib
from commander_msgs.msg import MotionAction
print("1")

rospy.init_node('check_network')
print("2")

client = actionlib.SimpleActionClient('/default_move_group/move', MotionAction)

if client.wait_for_server(rospy.Duration(1.0)):
    print("Connected to the 'move' action server. Network is configured correctly!")
else:
    print("Timed out waiting for server to connect. Your network is NOT configured properly.")
