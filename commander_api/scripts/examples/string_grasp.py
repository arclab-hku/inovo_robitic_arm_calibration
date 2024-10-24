#!/usr/bin/env python3

import rospy
import math

from commander_api.motion_control_client import MotionControlClient, Waypoint, Motion
import time
import socket
import binascii


rospy.init_node("string_grasp")
print("Node initialized")

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("Socket created")
addr = ("192.168.131.127", 3333)
# addr = ("10.70.24.91", 3333)
print("Socket connected")

mc = MotionControlClient("default_move_group")
print("MotionControlClient initialized")

ACCEL = 0.7
VEL = 0.7
BLENDL = 0.1

m1 = Motion()
# m2 = Motion()
m3 = Motion()
m4 = Motion()


data = b'\x00\x35\x00' #open
s.sendto(data, addr)
response, addr = s.recvfrom(1024)
print("string_grasp: open", response)

# # Down edge -> grasp point
# m1.add(Waypoint(0.007, 0.363, 0.340, math.pi, 0, 0) \
#     .constrain_joint_acceleration(0.3) \
#     .constrain_joint_velocity(0.2) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# mc.run(m1)
# print("Arrived m1")

# # rospy.sleep(3.)
# time.sleep(0.5)
# print("sleep 0.5s")




# # pink T table parameters
# m2.add(Waypoint(-0.091, 0.399, 0.134, math.pi, -0.203* math.pi, 0) \
#     .constrain_joint_acceleration(ACCEL) \
#     .constrain_joint_velocity(VEL) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# mc.run(m2)

# print("Arrived m2")


# # paper grasping trajectory
# m3.add(Waypoint(-0.125, 0.425, 0.171, 0.822*math.pi, -0.420 * math.pi, 0.198 * math.pi) \
#     .constrain_joint_acceleration(0.95) \
#     .constrain_joint_velocity(0.95) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# m3.add(Waypoint(-0.147, 0.425, 0.106, 0.822*math.pi, -0.420 * math.pi, 0.198 * math.pi) \
#     .constrain_joint_acceleration(0.95) \
#     .constrain_joint_velocity(0.95) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())


# # pen grasping trajectory
# m3.add(Waypoint(0.080, 0.385, 0.164, math.pi, 0, 0) \
#     .constrain_joint_acceleration(0.95) \
#     .constrain_joint_velocity(0.95) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# m3.add(Waypoint(0.014, 0.390, 0.162, math.pi, 0, 0) \
#     .constrain_joint_acceleration(0.95) \
#     .constrain_joint_velocity(0.95) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# m3.add(Waypoint(0.002, 0.395, 0.139, math.pi, 0, 0) \
#     .constrain_joint_acceleration(0.95) \
#     .constrain_joint_velocity(0.95) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())

# # coke bottle grasping trajectory
# m3.add(Waypoint(0.007, 0.363, 0.340, math.pi, 0, 0) \
#     .constrain_joint_acceleration(0.95) \
#     .constrain_joint_velocity(0.95) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# m3.add(Waypoint(-0.104, 0.364, 0.250, 0, -0.5 * math.pi, math.pi) \
#     .constrain_joint_acceleration(0.95) \
#     .constrain_joint_velocity(0.95) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# m3.add(Waypoint(-0.104, 0.380, 0.140, 0, -0.5 * math.pi, math.pi) \
#     .constrain_joint_acceleration(0.55) \
#     .constrain_joint_velocity(0.55) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# m3.add(Waypoint(-0.122, 0.410, 0.130, 0.667 * math.pi, -0.458 * math.pi, 0.344 * math.pi) \
#     .constrain_joint_acceleration(0.95) \
#     .constrain_joint_velocity(0.95) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())

# mc.run(m3)
# print("Arrived m3")

time.sleep(1)
print("sleep 1s")


# data = b'\x00\x00\x00' #open
data = b'\x00\x00\x01' #close
# data = b'\x00\x00\x02' #stop
# print(data)
s.sendto(data, addr)
response, addr = s.recvfrom(1024)
# print(response)
print("string_grasp: close", response)

# # Back to home point
# m4.add(Waypoint(0.007, 0.363, 0.180, math.pi, 0, 0) \
#     .constrain_joint_acceleration(0.6) \
#     .constrain_joint_velocity(0.6) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# mc.run(m4)
# m4.add(Waypoint(0.007, 0.363, 0.340, math.pi, 0, 0) \
#     .constrain_joint_acceleration(0.6) \
#     .constrain_joint_velocity(0.6) \
#     .set_blend(BLENDL, 0.5) \
#     .set_linear())
# mc.run(m4)
# print("Arrived m4")

time.sleep(0.2)
print("sleep 0.2s")

data = b'\x00\x00\x02' #add force
s.sendto(data, addr)
response, addr = s.recvfrom(1024)
print("string_grasp: add force", response)


time.sleep(5.0)
print("sleep 5.0s")

data = b'\x00\x00\x03' #stop
s.sendto(data, addr)
response, addr = s.recvfrom(1024)
print("string_grasp: stop", response)


s.close()

print("Finished")
