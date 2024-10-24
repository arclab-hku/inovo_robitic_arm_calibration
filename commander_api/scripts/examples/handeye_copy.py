#!/usr/bin/env python3
# import rospy
# from commander_api.motion_control_client import MotionControlClient
# import tf
#  注释掉ros相关的内容
import cv2
import os
import yaml
import numpy as np
# import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

def quaternion_matrix(quaternion):
    """
    Convert a quaternion to a rotation matrix.
    
    Args:
    quaternion (array-like): A 4-element array representing the quaternion (w, x, y, z)
    
    Returns:
    numpy.ndarray: A 4x4 homogeneous rotation matrix
    """
    q = np.array(quaternion, dtype=np.float64)
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        return np.identity(4)
    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]
    ])

def xyzw_to_wxyz(q):
    return [q[3], q[0], q[1], q[2]]

def wxyz_to_xyzw(q):
    return [q[1], q[2], q[3], q[0]]

def draw_coordinate_system(image, rvec, tvec, camera_matrix, dist_coeffs, length=0.1):
    """
    Draw a 3D coordinate system on the image.
    """
    axis = np.float32([[0,0,0], [length,0,0], [0,length,0], [0,0,length]]).reshape(-1,3)
    imgpts, jac = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    
    origin = tuple(imgpts[0].ravel().astype(int))
    image = cv2.line(image, origin, tuple(imgpts[1].ravel().astype(int)), (0,0,255), 3)  # X-axis (red)
    image = cv2.line(image, origin, tuple(imgpts[2].ravel().astype(int)), (0,255,0), 3)  # Y-axis (green)
    image = cv2.line(image, origin, tuple(imgpts[3].ravel().astype(int)), (255,0,0), 3)  # Z-axis (blue)
    
    return image

intrinsic_path = "/home/qy/workspace/hand_eye_calibration/aruco_example_cv_4.8.0/config/camera_calibration.yaml"
# 我们还使用之前的这个内参，因为D455相机还是那个

with open(intrinsic_path) as file:
    cam_calib_dist = yaml.load(file, Loader=yaml.Loader)
    print('load yaml')
    print(cam_calib_dist.keys())
    print("cam_calib_dist['mtx']")
    print(cam_calib_dist['mtx'])
    mtx = cam_calib_dist['mtx']
    dist = cam_calib_dist['dist']
# dist = np.zeros_like(dist)

# 读取了内参，接下来还要想办法在循环里面读取外参

# calibrate_path = "/home/qy/inovo_ws/src/inovo_ros/commander_api/scripts/examples/calibration_data"
# os.makedirs(calibrate_path, exist_ok=True)

# # 初始化 RealSense 相机，这里不需要了
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
# profile = pipeline.start(config)

dict_aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dict_aruco, parameters)
squareLength = 4.1625   # Here, our measurement unit is centimetre.
markerLength = 3.1219  # Here, our measurement unit is centimetre.
board = cv2.aruco.CharucoBoard((5,7), squareLength,markerLength,dict_aruco)

#  注释掉ros相关的内容
# rospy.init_node("get_robot_state")
# mc = MotionControlClient("default_move_group")
R_gripper2base = []
t_gripper2base = []
R_target2camera = []
t_target2camera = []


raw_data_path = "/home/qy/inovo_ws/src/inovo_ros/commander_api/scripts/examples/exp1_10041611"

for i in range(46):
    print(i)
    if i in (35, 36, 37) :
    # if i > 20:
        print("skip", i)
        continue
    image_file_name = f"exp_{i}.jpg"
    text_file_name = f"exp_{i}.txt"
    image_full_path = os.path.join(raw_data_path, image_file_name)
    text_full_path = os.path.join(raw_data_path, text_file_name)
    # 读取图像
    image = cv2.imread(image_full_path)
    if image is not None:  # 检查图像是否成功读取
        image_array = np.array(image)  # 转换为 NumPy 数组
    else:
        print(f"Error reading image {image_file_name}.")
        continue

    # 将图像转换为numpy数组
    # color_image = np.asanyarray(color_frame.get_data())
    color_image = image_array
    h, w = color_image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(color_image, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]

    # newcameramtx = mtx
    # dst = color_image

    # dst = color_image
    # cv2.imshow('RealSense raw', dst)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(dst)
    image = dst

    if markerIds is not None:
        rvecs, tvecs, trash = my_estimatePoseSingleMarkers(markerCorners, 3.1219, newcameramtx, dist) #Also use cm unit #changed into m
        # 在图像上绘制检测到的标记
        cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)
        # 细化角点位置
        _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            markerCorners, markerIds, dst, board)
        
        # 估计位姿
        if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 3:
            # 绘制 Charuco 角点
            cv2.aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids, (0, 255, 0))
            valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, board, mtx, dist, None, None)
            
            if valid:
                print("Rotation Vector:", rvec)
                print("Translation Vector:", tvec)
                
                # 可选:将旋转向量转换为旋转矩阵
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                print("Rotation Matrix:", rotation_matrix)
                
                # 绘制坐标轴
                cv2.drawFrameAxes(image, mtx, dist, rvec, tvec, 5)
                
                # cv2.imshow('Pose Estimation', image)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
            else:
                print("无法估计位姿")
        else:
            print("未检测到足够的 Charuco 角点")

        for idx in range(len(markerIds)):
            if markerIds[idx] == 8:
                # cv2.drawFrameAxes(dst, mtx, dist, rvecs[idx], tvecs[idx], 5)
                # print('marker id:%d, pos_x = %f,pos_y = %f, pos_z = %f' % (markerIds[idx],tvecs[idx][0],tvecs[idx][1],tvecs[idx][2]))
                dst = draw_coordinate_system(dst, rvecs[idx], tvecs[idx], newcameramtx, dist)
                Rt2c, _ = cv2.Rodrigues(rvecs[idx])
                # Pt2c = [tvecs[idx][0],tvecs[idx][1],tvecs[idx][2]]
                Pt2c = np.concatenate([tvecs[idx][0],tvecs[idx][1],tvecs[idx][2]]).reshape(1, 3)



                Rt2c, _ = cv2.Rodrigues(rvec)
                # Pt2c = [tvecs[idx][0],tvecs[idx][1],tvecs[idx][2]]
                Pt2c = np.concatenate([tvec[0],tvec[1],tvec[2]]).reshape(1, 3)



                Pt2c = Pt2c * 0.01 # from cm to m
                Tt2c = np.zeros((4, 4))
                Tt2c[0:3, 0:3] = Rt2c
                Tt2c[0:3, 3] = Pt2c
                Tt2c[3, 3] = 1.0
                R_target2camera.append(Rt2c)
                t_target2camera.append(Pt2c)
                print("Camera to target:")
                print(Tt2c)
                
        # cv2.aruco.drawDetectedMarkers(dst, markerCorners, mar_to_end=nkerIds)
        # cv2.imshow('detect', dst)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # 读取文本文件

    tcp_pose = {
        'position': {},
        'orientation': {}
    }

    with open(text_full_path, 'r') as file:
        for line in file:
            # 去掉开头和结尾的空白字符
            line = line.strip()
            
            if line.startswith("Position:"):
                # 读取位置数据
                tcp_pose['position']['x'] = float(line.split('x:')[1].strip())
                tcp_pose['position']['y'] = float(next(file).split('y:')[1].strip())
                tcp_pose['position']['z'] = float(next(file).split('z:')[1].strip())
            
            elif line.startswith("Orientation:"):
                # 读取方向数据
                tcp_pose['orientation']['x'] = float(line.split('x:')[1].strip())
                tcp_pose['orientation']['y'] = float(next(file).split('y:')[1].strip())
                tcp_pose['orientation']['z'] = float(next(file).split('z:')[1].strip())
                tcp_pose['orientation']['w'] = float(next(file).split('w:')[1].strip())
    Pg2b = [tcp_pose['position']['x'], tcp_pose['position']['y'], tcp_pose['position']['z']]
    quaternion = [
        tcp_pose['orientation']['x'],
        tcp_pose['orientation']['y'],
        tcp_pose['orientation']['z'],
        tcp_pose['orientation']['w']
    ]

    quaternion = xyzw_to_wxyz(quaternion)
    Rg2b = quaternion_matrix(quaternion)
    # 将四元数转换为旋转矩阵
    # Rg2b = tf.transformations.quaternion_matrix(quaternion)
    Tg2b = Rg2b
    Tg2b[0:3, 3] = Pg2b  # 将位置添加到变换矩阵的最后一列
    R_gripper2base.append(Rg2b[0:3, 0:3])
    t_gripper2base.append(Pg2b)

    print("Gripper to base:")
    print(Tg2b)
    
        


R_gripper2base = np.array(R_gripper2base)
t_gripper2base = np.array(t_gripper2base)
R_target2camera = np.array(R_target2camera)
t_target2camera = np.array(t_target2camera)

# print("R_gripper2base, t_gripper2base, R_target2camera, t_target2camera") 
# print(R_gripper2base, t_gripper2base, R_target2camera, t_target2camera)

# 调用 OpenCV 的 calibrateHandEye 函数
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base, R_target2camera, t_target2camera, cv2.CALIB_HAND_EYE_TSAI) #the method do not influence
# 打印结果
print("Rotation matrix from camera to gripper:")
print(R_cam2gripper)

print("Translation vector from camera to gripper:")
print(t_cam2gripper)

# print(R_gripper2base, t_gripper2base, R_target2camera, t_target2camera)


# 计算欧拉角
# 使用 scipy 的 Rotation 类来处理不同的顺序
rotation = R.from_matrix(R_cam2gripper)

# 计算不同顺序的欧拉角
euler_angles_xyz = rotation.as_euler('xyz', degrees=True)  # XYZ 顺序
euler_angles_zyx = rotation.as_euler('zyx', degrees=True)  # ZYX 顺序
euler_angles_yxz = rotation.as_euler('yxz', degrees=True)  # YXZ 顺序
euler_angles_xzy = rotation.as_euler('xzy', degrees=True)  # XZY 顺序
euler_angles_zxy = rotation.as_euler('zxy', degrees=True)  # ZXY 顺序
euler_angles_yzx = rotation.as_euler('yzx', degrees=True)  # YZX 顺序

# 打印结果
print("Euler angles (XYZ):", euler_angles_xyz)
print("Euler angles (ZYX):", euler_angles_zyx)
print("Euler angles (YXZ):", euler_angles_yxz)
print("Euler angles (XZY):", euler_angles_xzy)
print("Euler angles (ZXY):", euler_angles_zxy)
print("Euler angles (YZX):", euler_angles_yzx)

