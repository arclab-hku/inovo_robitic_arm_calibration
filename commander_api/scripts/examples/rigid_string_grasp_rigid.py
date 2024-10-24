#!/usr/bin/env python3

import rospy
import math

# from commander_api.motion_control_client import MotionControlClient, Waypoint, Motion
import time
import socket
import binascii

from inovopy.robot import InovoRobot

bot = InovoRobot.default_iva("192.168.1.7")

bot.gripper_activate()

bot.sleep(5)
bot.linear(HOME)

bot.gripper_set("open")

# bot.gripper_set("open/close")


# rigid gripper test
rospy.init_node("string_grasp")
print("Node initialized")



mc = MotionControlClient("default_move_group")
print("MotionControlClient initialized")

ACCEL = 0.7
VEL = 0.7
BLENDL = 0.1

m1 = Motion()
m2 = Motion()
m3 = Motion()

# home point
m1.add(Waypoint(0.007, 0.363, 0.400, math.pi, 0, 0.5 * math.pi) \
    .constrain_joint_acceleration(0.3) \
    .constrain_joint_velocity(0.2) \
    .set_blend(BLENDL, 0.5) \
    .set_linear())
mc.run(m1)
print("Arrived m1")

time.sleep(0.5)
print("sleep 0.5s")

m2.add(Waypoint(0.007, 0.363, 0.290, math.pi, 0, 0.5 * math.pi) \
    .constrain_joint_acceleration(0.2) \
    .constrain_joint_velocity(0.2) \
    .set_blend(BLENDL, 0.5) \
    .set_linear())

mc.run(m2)
print("Arrived m2")

time.sleep(1)
print("sleep 1s")

bot.gripper_set("close")

time.sleep(1)
print("sleep 1s")


# Back to home point
m3.add(Waypoint(0.007, 0.363, 0.400, math.pi, 0, 0.5 * math.pi) \
    .constrain_joint_acceleration(0.3) \
    .constrain_joint_velocity(0.3) \
    .set_blend(BLENDL, 0.5) \
    .set_linear())
mc.run(m3)
print("Arrived m4, back to home point")


print("Finished")
