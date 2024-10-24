import numpy as np

def generate_hemisphere_samples(x0, y0, z0, r):
    # 生成上半球上的100个采样点
    num_samples = 2
    latitude = 2 #layers
    longitude = 2
    threshold = 0
    
    theta = np.linspace((threshold * 2 * np.pi), ((1-threshold) * 2 * np.pi), num_samples)
    phi = np.linspace((threshold * np.pi / 2), ((1-threshold) * np.pi / 2), num_samples)
    
    # 生成网格
    theta_grid, phi_grid = np.meshgrid(theta, phi)
    
    # 计算球面坐标
    x = x0 + r * np.sin(phi_grid) * np.cos(theta_grid)
    y = y0 + r * np.sin(phi_grid) * np.sin(theta_grid)
    z = z0 + r * np.cos(phi_grid)
    
    # 初始化姿态数组
    poses = []
    
    for i in range(num_samples):
        for j in range(num_samples):
            # 计算每个点的坐标
            point_x = x[i, j]
            point_y = y[i, j]
            point_z = z[i, j]
            
            # 计算面向球心的方向向量
            direction_vector = np.array([x0 - point_x, y0 - point_y, z0 - point_z])
            direction_vector = direction_vector / np.linalg.norm(direction_vector)
            
            # 计算俯仰角（pitch）和偏航角（yaw）
            pitch = np.arctan2(direction_vector[1], direction_vector[2])
            yaw = np.arctan2(direction_vector[0], np.sqrt(direction_vector[1]**2 + direction_vector[2]**2))
            
            # 横滚角（roll）设为0，因为我们假设没有横滚
            roll = 0
            
            # 将姿态添加到列表中
            pose = [point_x, point_y, point_z, pitch, roll, yaw]
            poses.append(pose)
    
    return poses

# 示例使用
x0, y0, z0 = 0, 0, 0
r = 1
samples = generate_hemisphere_samples(x0, y0, z0, r)
for pose in samples:
    print(pose)