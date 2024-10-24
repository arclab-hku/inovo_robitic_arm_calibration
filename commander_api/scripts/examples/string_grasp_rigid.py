#!/usr/bin/env python3

from inovopy.robot import InovoRobot
from inovopy.geometry.transform import Transform
from inovopy.logger import Logger
print("setp 1")

# 假设末端位置和方向如下

x = -21.9  # mm
y = 262.6 # mm
z = 355.4 # mm
rx = -180  # 度
ry = 0  # 度
rz = 90  # 度

# 创建 Transform 实例
HOME = Transform(vec_mm=(x, y, z), euler_deg=(rx, ry, rz))

# 打印结果
print(HOME.to_homogenous())

bot = InovoRobot.default_iva("192.168.1.7")
# bot = InovoRobot.default_iva("psu020")
print("setp 2")
bot.gipper_activate()
bot.sleep(2)
bot.gripper_set("close")
bot.sleep(2)
bot.gripper_set("open")
# bot.gripper_set("open/close")

bot.linear(HOME)

print("setp 3")
print("Finished")
