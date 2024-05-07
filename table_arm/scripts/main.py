# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
from ultralytics import YOLO
import rospy
from plumbing_pub_sub.msg import target_pose

''' 
设置
'''
pipeline = rs.pipeline()  # 定义流程pipeline，创建一个管道
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 配置depth流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 配置color流
pipe_profile = pipeline.start(config)  # streaming流开始
# 创建对齐对象与color流对齐
align = rs.align(rs.stream.color)

''' 
获取对齐图像帧与相机参数
'''

def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧，获取颜色和深度的框架集
    aligned_frames = align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐

    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
    aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧

    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参

    img_color = np.asanyarray(aligned_color_frame.get_data())  # RGB图
    img_depth = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）

    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame


''' 
获取随机点三维坐标
'''

def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = int(depth_pixel[0])
    y = int(depth_pixel[1])
    dis = aligned_depth_frame.get_distance(x, y)  # 获取该像素点对应的深度
    #camera_coordinate[x,y,depth]-->[x,y,z]
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    z = math.sqrt(camera_coordinate[2] ** 2 - camera_coordinate[0] ** 2 - camera_coordinate[1] ** 2)
    camera_coordinate[2] = z
    # print('camera_coordinate: ', camera_coordinate)
    return camera_coordinate


if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("talker_p")
    #3.实例化 发布者 对象
    pub = rospy.Publisher("real_coordinate",target_pose,queue_size=10)
    #4.组织被发布的数据，并编写逻辑发布数据
    msg = target_pose()  #创建 msg 对象
    # 加载yolov8模型
    model = YOLO('/home/mzh/ros_project/src/plumbing_pub_sub/scripts/holo_delet.pt')
    while True:
        # 获取对齐图像帧与相机参数
        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()  # 获取对齐图像与相机参数

        # 获取海参框像素坐标
        results = model(img_color, conf=0.8, device=0)
        annotated_frame = results[0].plot()
        centerpoint = [0, 0]
        for result in results:
            if len(result) > 0:
                boxes = result.boxes.xywh.cpu().numpy().astype(int)
                for box in boxes:
                    centerpoint[0] = box[0]+box[2]/2
                    centerpoint[1] = box[1]+box[3]/2
                    camera_coordinate = get_3d_camera_coordinate(centerpoint, aligned_depth_frame, depth_intrin)
                    '''发布话题的代码写在这儿里,real_x,y,z是真实世界的坐标，把这三个数据发给机械臂规划端'''
                    real_x = camera_coordinate[0]
                    real_y = camera_coordinate[1]
                    real_z = camera_coordinate[2]
                    position = np.array([[real_x],[real_y],[real_z]])
                    T1 = np.array([[1,0,0],
                                   [0, -1*np.sqrt(2)/2, -1*np.sqrt(2)/2],
                                   [0, np.sqrt(2)/2, -1*np.sqrt(2)/2]])
                    T2 = np.array([[0,1,0],
                                  [-1,0,0],
                                  [0,0,1]])
                    P1 = np.dot(T1,position)
                    P2 = np.dot(T2, P1)

                    rospy.loginfo("发出的数据:%f,%f,%f",P2[0,0],P2[1,0],P2[2,0])
                    msg.x = P2[0,0]+0.78
                    msg.y = P2[1,0]
                    msg.z = P2[2,0]+0.258


                    pub.publish(msg)
                    rospy.loginfo("发出的数据:%f,%f,%f",msg.x,msg.y,msg.z)






        # 显示画面
        cv2.imshow('RealSence', annotated_frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
