#!/usr/bin/env python3

import rospy
import math

from commander_api.motion_control_client import MotionControlClient, Waypoint, Motion
import time
import socket
import binascii

# for cat and hohrse
rospy.init_node("string_grasp")
print("Node initialized")

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("Socket created")
addr = ("192.168.137.253", 3333)
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

# Down edge -> grasp point
m1.add(Waypoint(0.007, 0.363, 0.340, math.pi, 0, 0.5 * math.pi) \
    .constrain_joint_acceleration(0.3) \
    .constrain_joint_velocity(0.2) \
    .set_blend(BLENDL, 0.5) \
    .set_linear())
mc.run(m1)
print("Arrived m1")

# rospy.sleep(3.)
time.sleep(0.5)
print("sleep 0.5s")




# coke tin grasping trajectory
m3.add(Waypoint(-0.189, 0.331, 0.185, 0.772 * math.pi, 0.0395 * math.pi, 0.667 * math.pi) \
    .constrain_joint_acceleration(0.8) \
    .constrain_joint_velocity(0.8) \
    .set_blend(BLENDL, 0.5) \
    .set_linear())
m3.add(Waypoint(-0.153, 0.359, 0.153, 0.789 * math.pi, 0.011 * math.pi, 0.685 * math.pi) \
    .constrain_joint_acceleration(0.8) \
    .constrain_joint_velocity(0.8) \
    .set_blend(BLENDL, 0.5) \
    .set_linear())


mc.run(m3)
print("Arrived m3")

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

time.sleep(0.2)
print("sleep 0.2s")

data = b'\x00\x00\x02' #add force
s.sendto(data, addr)
response, addr = s.recvfrom(1024)
print("string_grasp: add force", response)

time.sleep(0.2)
print("sleep 0.2s")

# Back to home point
m4.add(Waypoint(0.007, 0.363, 0.340, math.pi, 0, 0.5 * math.pi) \
    .constrain_joint_acceleration(0.3) \
    .constrain_joint_velocity(0.3) \
    .set_blend(BLENDL, 0.5) \
    .set_linear())
mc.run(m4)
print("Arrived m4, back to home point")


time.sleep(5.0)
print("sleep 5.0s")

data = b'\x00\x00\x03' #stop
s.sendto(data, addr)
response, addr = s.recvfrom(1024)
print("string_grasp: stop", response)


s.close()

print("Finished")
