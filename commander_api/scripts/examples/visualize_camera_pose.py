import os
import json
import numpy as np
from scipy.spatial.transform import Rotation
import glob
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

R_c2g = np.array([
    [-0.9140669, -0.27372591, -0.2992588],
    [0.25605588, 0.18271938, -0.94923602],
    [0.31451088, -0.9442922, -0.09692861]
])

T_c2g = np.array([[9.76377104e-05],
                  [-3.99840801e-01],
                  [-1.02993708e-04]])
exchangei = [[-1, 0, 0], [0, 0, 1], [0, 1, 0]]
# exchangei = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
camera_to_gripper = np.eye(4)
camera_to_gripper[:3, :3] = R_c2g
camera_to_gripper[:3, 3] = T_c2g.flatten()
input_path = "/home/qy/inovo_ws/src/inovo_ros/commander_api/scripts/examples/exp1_10041542"
txt_files = glob.glob(os.path.join(input_path, '*.txt'))

# 创建 3D 图形
fig = plt.figure(figsize=(12, 10))
# fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制箭头的起点
origin = np.array([0, 0, 0])
arrow_vector = np.array([0, 0, 1])

# start_point = np.array([0, 0, 0])
# end_point = np.array([0, 0, 1])
# original_vector = end_point - start_point


for i in range(55):
    file_nmae = f"exp_{i}.txt"
    txt_file = os.path.join(input_path, file_nmae)
    try:
        with open(txt_file, 'r') as f:
            lines = f.readlines()

        # print(f"Content of {txt_file}:")
        # for line in lines:
        #     print(line.strip())

        position = []
        orientation = []
        for line in lines:
            parts = line.split(':')
            if len(parts) >= 3:
                if 'Position' in parts[0]:
                    position.append(float(parts[2].strip()))
                elif 'Orientation' in parts[0]:
                    orientation.append(float(parts[2].strip()))
            else:
                if orientation == []:
                    position.append(float(parts[1].strip()))
                else:
                    orientation.append(float(parts[1].strip()))


        position = np.array(position)
        orientation = np.array(orientation)

        rotation_matrix = Rotation.from_quat(orientation).as_matrix()

        transform_matrix = np.eye(4) #gripper to base
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = position

        final_transform_matrix = camera_to_gripper @ transform_matrix
        print(final_transform_matrix)

        # 应用变换
        # original_start_point = np.append(start_point, 1)  # 转换为齐次坐标
        # transformed_start_point = np.dot(final_transform_matrix, original_start_point)[:3]

        # original_end_point = np.append(end_point, 1)  # 转换为齐次坐标
        # transformed_end_point = np.dot(final_transform_matrix, original_end_point)[:3]

        # arrow_tip = final_transform_matrix @ np.append(arrow_vector, 1)

        # 绘制箭头
        # ax.quiver(position[0], position[1], position[2],
        #            arrow_vector[0], arrow_vector[1], arrow_vector[2],
        #            length=0.1, normalize=True)
        # ax.quiver(transformed_start_point[0], transformed_start_point[1], transformed_start_point[2],
        #            transformed_end_point[0], transformed_end_point[1], transformed_end_point[2],
        #            length=0.1, normalize=True)

        
        # 计算箭头的旋转后的方向
        rotated_arrow = exchangei @ rotation_matrix @  arrow_vector
        # rotated_arrow = R_c2g @ arrow_vector
        # position = T_c2g

        # 绘制箭头
        ax.quiver(position[0], position[1], position[2],
                   rotated_arrow[0]+position[0], rotated_arrow[1]+position[1], rotated_arrow[2]+position[2],
                   length=0.05, normalize=True)

    except Exception as e:
        print(f"Error processing {txt_file}: {str(e)}")

# 设置坐标轴范围
ax.set_xlim([-0.5, 0.5])
ax.set_ylim([0, 1])
ax.set_zlim([0, 0.5])

# ax.set_xlim([-1, 1])
# ax.set_ylim([-1, 1])
# ax.set_zlim([-1, 1])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title("3D Arrow Representation")
plt.show()