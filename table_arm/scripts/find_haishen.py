#!/usr/bin/env python
# coding=utf-8

import rospy
from math import *
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
from geometry_msgs.msg import Twist
from table_arm.msg import position_color as PositionMsg
from table_arm.msg import color_ik_result as color_ik_result_Msg
from std_msgs.msg import Int8
from std_msgs.msg import String as StringMsg
from ultralytics import YOLO
last_erro=0
col_blue = (80,33,12,127,255,255)# blue
col_green= (50,46,24,80,255,255)# green
col_yellow = (20,90,110,39,255,255)# yellow
haishen_model = YOLO('/home/mzh/ros_project/src/plumbing_pub_sub/scripts/holo_delet.pt')
def nothing(s):
    pass

class Find_Haishen:
    def __init__(self):
        self.m=1            #更换检测颜色的标志位
        self.count=0       #每个色块检测次数的计数值
        self.bridge = cv_bridge.CvBridge()
        self.i=0
        #cv2.namedWindow("window", 1)
        # 订阅usb摄像头
        self.pictureHeight= 480
        self.pictureWidth = 640
        horizontalAngle =0.5235987755982988
        vertAngle =0.43196898986859655
        self.tanHorizontal = np.tan(horizontalAngle)
        self.tanVertical = np.tan(vertAngle)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)#订阅图像话题
        # self.image_sub = rospy.Subscriber("cv_bridge_image", Image, self.image_callback)
 
        self.positionPublisher = rospy.Publisher('/color_position', PositionMsg, queue_size=10) #发布色块的位置（原始数据）
        self.arm_ik_angle_Publisher = rospy.Publisher('/color_ik_result', color_ik_result_Msg, queue_size=10)#发布根据色块位置求解的机械臂关节目标弧度话题
        self.visual_flagPublisher = rospy.Publisher('/visual_func_flag', Int8, queue_size =1)
        self.twist = Twist()
        self.link_a=rospy.get_param('link_a',0.105)   #机械参数
        self.link_b=rospy.get_param('link_b',0.100)    #机械参数
        self.link_c=rospy.get_param('link_c',0.170)    #机械参数
        self.link_h=rospy.get_param('link_h',0.105)   #机械参数
        self.x_offset=rospy.get_param('color_x_offset',0.000)      #x轴和y轴夹取位置微调的偏差参数，单位（米）
        self.y_offset=rospy.get_param('color_y_offset',0.001)
        self.auxiliary_angle=rospy.get_param('auxiliary_angle',0.20)

        self.basic_angle= acos( (self.link_c -self.link_h)/self.link_a ) #计算机械臂夹爪可触底的关节基础角度
        #self.basic_angle= 0.5732
        rospy.loginfo('basic_angle is %s' ,self.basic_angle)
        rospy.loginfo('find_color_node is init successful')

     #色块分拣标志位发布函数
    def publish_flag(self):
        visual_func_flag=Int8()
        visual_func_flag.data=1
        rospy.sleep(1.)
        self.visual_flagPublisher.publish(visual_func_flag)
        rospy.loginfo('a=%d',visual_func_flag.data)
        print("1111111111111111111111111111111111111111111111111111111111111")

    def image_callback(self, msg):
        global last_erro
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.resize(image, (320,240), interpolation=cv2.INTER_AREA)#提高帧率
        results = haishen_model(image, conf=0.8, device=0)
        annotated_frame = results[0].plot()
        centerpoint = [0, 0]
        for result in results:
            if len(result) > 0:
                boxes = result.boxes.xywh.cpu().numpy().astype(int)
                for box in boxes:
                
                    centerpoint[0] = box[0]+10
                    centerpoint[1] = box[1]
                    ##########################################################################################################################3
                    cropped_region = annotated_frame[(box[1]-box[3]//2):(box[1]+box[3]//2),(box[0]-box[2]//2):(box[0]+box[2]//2)]
                    ###########################################################################################################################33
                    cv2.imwrite("/home/mzh/lala.png",cropped_region)
                    cv2.imwrite("/home/mzh/lalas.png",annotated_frame)
                    _, cropped_region = cv2.threshold(cropped_region, 2, 255, cv2.THRESH_BINARY)
                    cropped_region = cv2.cvtColor(cropped_region, cv2.COLOR_BGR2GRAY)
                    contour = cv2.findContours(cropped_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  #获取色块的轮廓

                    centerRaw, size, rotation = cv2.minAreaRect(contour[0])  #输入色块的轮廓，获得:centerraw是色块的坐标，size是色块的面积，rotation是色块的旋转角
                    print(centerRaw,centerpoint)
                    angleX = self.calculateAngleX(centerpoint) #做数学转换获取色块在画幅中的坐标
                    angleY = self.calculateAngleY(centerpoint)
                    self.publishPosition(angleX,angleY,rotation) #发布话题：色块的位置（原始数据）
                    self.publishArm_Angle(angleX,angleY,rotation)	 #发布话题：根据色块位置求解的机械臂关节目标弧度话题
                    cv2.imshow('RealSence', annotated_frame)
                    key = cv2.waitKey(1)
                    return
        #如果画幅中没有出现色块，则发布话题，色块的位置信息为999
        angleX = 999
        angleY = 999
        rotation=999
        self.publishPosition(angleX,angleY,rotation)
        self.publishArm_Angle(angleX,angleY,rotation)	

        #cv2.imshow("window", image)
        #cv2.waitKey(3)
        cv2.imshow('RealSence', annotated_frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
    

        
       

    def calculateAngleX(self, pos):
        '''calculates the X angle of displacement from straight ahead'''
        centerX = pos[0]
        displacement = 2*centerX/self.pictureWidth-1
        angle = -1*np.arctan(displacement*self.tanHorizontal)
        return angle

    def calculateAngleY(self, pos):
        centerY = pos[1]
        displacement = 2*centerY/self.pictureHeight-1
        angle = -1*np.arctan(displacement*self.tanVertical)
        return angle

    def publishPosition(self, x,y,rotate):
        posMsg = PositionMsg(x, y, rotate)
        self.positionPublisher.publish(posMsg)

    def publishArm_Angle(self, x,y,rotate):

        if x > 0.5 or y >0.4: # 如果色块的位置太偏，则认为数据有误
             return 
        #print(x)
        true_x= x*(-0.354)+0.105+self.x_offset  #色块的原始坐标在（左右方向）从左开始是0到0.56，因此减0.28是将0点坐标移到整个画幅的中间

        #色块的原始坐标（前后方向）转换成实际距离
        true_y= y * 0.307 +0.17+self.y_offset
      
        #rospy.loginfo('x is %s' ,x)
        #计算云台的目标运动角度
        #print(x)
        pedestal_angle=degrees(atan(abs(true_x / true_y)))
        if  true_x >0 :#角度正反（左右）关系转换
             pedestal_angle  = pedestal_angle
        else :
             pedestal_angle  = -pedestal_angle
        #arm_angle=degrees( atan( (self.link_a*sin(self.basic_angle) +  self.link_c )/self.link_a*cos(self.basic_angle) ) )
        caculate_A=self.link_a*sin(self.basic_angle) +sin(self.auxiliary_angle)*self.link_c
        caculate_B=self.link_a*cos(self.basic_angle) +cos(self.auxiliary_angle)*self.link_c
        caculate_C=(sqrt(pow(true_x,2)+ pow(true_y,2))-self.link_b)
        caculate_D= acos( caculate_C/(sqrt(pow(caculate_A,2)+ pow(caculate_B,2) ) ) )
        #kk=sqrt(pow(x,2)+ pow(y,2))
        #rospy.loginfo('DD is %s' ,DD)
        caculate_E= atan(caculate_B/caculate_A) 
        caculate_G=(caculate_E-caculate_D)
        #print(GG)
        #rospy.loginfo('DD is %s,EE is %s,GG is %s' ,DD,EE,GG)
        #rospy.loginfo('GG is %s' ,GG)

        hand_angle = 80+rotate #角度正反关系转换
        if  hand_angle >45 :
             hand_angle  = hand_angle -90
        #hand_angle = 90-rotate + 90
        pedestal_angle = radians(pedestal_angle) #云台的目标运动角度,radians函数是弧度转角度
        #pedestal_angle=pedestal_angle
        #rospy.loginfo('pedestal_angle is %s' ,pedestal_angle)
        arm_angle      = (caculate_G)  #控制机械臂臂长的目标角度,radians函数是弧度转角度
        #rospy.loginfo('arm_angle is %s' ,arm_angle)
        hand_angle     = radians(hand_angle) #控制夹取色块旋转的目标角度,radians函数是弧度转角度
        #rospy.loginfo('hand_angle is %s' ,hand_angle)
        ikMsg=color_ik_result_Msg(pedestal_angle,arm_angle,hand_angle)
        self.arm_ik_angle_Publisher.publish(ikMsg)

rospy.init_node("find_color")
find_color = Find_Haishen()
rospy.spin()

