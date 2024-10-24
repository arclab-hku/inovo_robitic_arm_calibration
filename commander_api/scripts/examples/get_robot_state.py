#!/usr/bin/env python3
import rospy
from commander_api.motion_control_client import MotionControlClient
import tf

rospy.init_node("get_robot_state")

mc = MotionControlClient("default_move_group")

# 获取机器人的 TCP 姿态
tcp_pose = mc.get_tcp_pose()

position = [tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z]
quaternion = [
    tcp_pose.orientation.x,
    tcp_pose.orientation.y,
    tcp_pose.orientation.z,
    tcp_pose.orientation.w
]

# 将四元数转换为旋转矩阵
rotation_matrix = tf.transformations.quaternion_matrix(quaternion)

transformation_matrix = rotation_matrix
transformation_matrix[0:3, 3] = position  # 将位置添加到变换矩阵的最后一列

# 打印变换矩阵
print("Transformation Matrix:")
print(transformation_matrix)


# print("Robot's TCP pose :", mc.get_tcp_pose())

# rotation_matrix = tf.transformations.quaternion_matrix(quaternion)

# print("Robot's joint angles:", mc.get_joint_angles())
