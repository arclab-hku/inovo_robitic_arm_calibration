#!/usr/bin/env python3
import rospy
from commander_api.motion_control_client import MotionControlClient
import tf
import cv2
import os
import yaml
import numpy as np
import pyrealsense2 as rs

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



intrinsic_path = "/home/qy/workspace/hand_eye_calibration/aruco_example_cv_4.8.0/config/camera_calibration.yaml"
with open(intrinsic_path) as file:
    cam_calib_dist = yaml.load(file, Loader=yaml.Loader)
    print('load yaml')
    print(cam_calib_dist.keys())
    mtx = cam_calib_dist['mtx']
    dist = cam_calib_dist['dist']
calibrate_path = "/home/qy/inovo_ws/src/inovo_ros/commander_api/scripts/examples/calibration_data"
os.makedirs(calibrate_path, exist_ok=True)

# 初始化 RealSense 相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
profile = pipeline.start(config)

dict_aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dict_aruco, parameters)
# squareLength = 4.1625   # Here, our measurement unit is centimetre.
# markerLength = 3.1219  # Here, our measurement unit is centimetre.
# board = cv2.aruco.CharucoBoard((5,7), squareLength,markerLength,dict_aruco)

rospy.init_node("get_robot_state")
mc = MotionControlClient("default_move_group")
R_gripper2base = []
t_gripper2base = []
R_target2camera = []
t_target2camera = []
iteration = 0


try:
    while True:
        # 等待一组连贯的帧: 彩色
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        # 将图像转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())
        h, w = color_image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        dst = cv2.undistort(color_image, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        # dst = color_frame
        cv2.imshow('RealSense raw', dst)

        #按下去捕获，把当前的Tg2b Tt2c都存下来，用作下一步计算
        if cv2.waitKey(2) & 0xFF == ord('c'):
            print('captured')
            iteration = iteration + 1
            markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(dst)
            if markerIds is not None:
                rvecs, tvecs, trash = my_estimatePoseSingleMarkers(markerCorners, 3.1219, newcameramtx, dist) #Also use cm unit
                for idx in range(len(markerIds)):
                    if markerIds[idx] == 0:
                        # cv2.drawFrameAxes(dst, mtx, dist, rvecs[idx], tvecs[idx], 5)
                        # print('marker id:%d, pos_x = %f,pos_y = %f, pos_z = %f' % (markerIds[idx],tvecs[idx][0],tvecs[idx][1],tvecs[idx][2]))
                        Rt2c, _ = cv2.Rodrigues(rvecs[idx])
                        # Pt2c = [tvecs[idx][0],tvecs[idx][1],tvecs[idx][2]]
                        Pt2c = np.concatenate([tvecs[idx][0],tvecs[idx][1],tvecs[idx][2]]).reshape(1, 3)
                        Pt2c = Pt2c * 0.01 # from cm to m
                        Tt2c = np.zeros((4, 4))
                        Tt2c[0:3, 0:3] = Rt2c
                        Tt2c[0:3, 3] = Pt2c
                        R_target2camera.append(Rt2c)
                        t_target2camera.append(Pt2c)
                        print("Camera to target:")
                        print(Tt2c)
            # cv2.aruco.drawDetectedMarkers(dst, markerCorners, markerIds)
            # cv2.imshow('detect', dst)

            # 获取机器人的 TCP 姿态
            tcp_pose = mc.get_tcp_pose()

            Pg2b = [tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z]
            quaternion = [
                tcp_pose.orientation.x,
                tcp_pose.orientation.y,
                tcp_pose.orientation.z,
                tcp_pose.orientation.w
            ]
            # 将四元数转换为旋转矩阵
            Rg2b = tf.transformations.quaternion_matrix(quaternion)
            Tg2b = Rg2b
            Tg2b[0:3, 3] = Pg2b  # 将位置添加到变换矩阵的最后一列
            R_gripper2base.append(Rg2b[0:3, 0:3])
            t_gripper2base.append(Pg2b)

            print("Gripper to base:")
            print(Tg2b)
            path_picture = os.path.join(calibrate_path, 'img_%d.jpg' % iteration)
            cv2.imwrite(path_picture, dst)
            
            header = f"Rt2c/Pt2c/Rg2b/Pg2b\n"
            array_path = os.path.join(calibrate_path, 'Rt2c','array_%d.txt' % iteration)
            np.savetxt(array_path, Rt2c, delimiter=',',header=header)
            array_path = os.path.join(calibrate_path,'Pt2c', 'array_%d.txt' % iteration)
            np.savetxt(array_path, Pt2c, delimiter=',',header=header)
            array_path = os.path.join(calibrate_path,'Rg2b', 'array_%d.txt' % iteration)
            np.savetxt(array_path, Rg2b[0:3, 0:3], delimiter=',',header=header)
            array_path = os.path.join(calibrate_path,'Pg2b', 'array_%d.txt' % iteration)
            np.savetxt(array_path, Pg2b, delimiter=',',header=header)
            

        if cv2.waitKey(2) & 0xFF == ord('r'):
            R_gripper2base = np.array(R_gripper2base)
            t_gripper2base = np.array(t_gripper2base)
            R_target2camera = np.array(R_target2camera)
            t_target2camera = np.array(t_target2camera)

            print("R_gripper2base, t_gripper2base, R_target2camera, t_target2camera") 
            print(R_gripper2base, t_gripper2base, R_target2camera, t_target2camera)

            # 调用 OpenCV 的 calibrateHandEye 函数
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base, R_target2camera, t_target2camera)
            # 打印结果
            print("Rotation matrix from camera to gripper:")
            print(R_cam2gripper)

            print("Translation vector from camera to gripper:")
            print(t_cam2gripper)

            # print(R_gripper2base, t_gripper2base, R_target2camera, t_target2camera)

        if cv2.waitKey(2) & 0xFF == ord('q'):
            break
        # board_detector = cv2.aruco.CharucoDetector(board)
        # arucoParams = board_detector.getDetectorParameters
        # char_corners, char_ids, markerCorners, markerIds = board_detector.detectBoard(color_image)
        # cv2.aruco.drawDetectedMarkers(color_image, markerCorners, markerIds)
        # cv2.imshow('plot', color_image)

finally:
    # 停止流
    pipeline.stop()
    # 关闭所有窗口
    cv2.destroyAllWindows()


