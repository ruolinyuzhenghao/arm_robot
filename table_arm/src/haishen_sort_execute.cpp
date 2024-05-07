#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <stdbool.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "color_sort_execute.h"

int color_sequence=1;  //夹取顺序
int sequence[4]={0};  //夹取顺序排序数组
int color_confirmed_flag =0; //启动夹取的标志位
float auxiliary_angle; 
int red_count=0,blue_count=0,green_count=0,yellow_count=0; //接收到该数据帧的回调函数次数
int red_ready=0,blue_ready=0,green_ready=0,yellow_ready=0; //色块位置确认标志位
int red_done=0,yellow_done=0,blue_done=0,green_done=0;  //色块是否已经夹取标志位
float red_target_data[3]={0},blue_target_data[3]={0},green_target_data[3]={0},yellow_target_data[3]={0};//色块的目标位置对应的关节运动弧度数组
float joint_target1=0,joint_target2=0,joint_target3=0; //赋值给moveit做正解的目标关节值
float link_a,link_b,link_c,link_h; //机械参数
float base_angle;
std::string arm_state="none",target_color="noon";  //机械臂状态，夹取目标色块的颜色

bool arm_success,hand_close_success,hand_open_success; //moveit正解计算（返回值）是否成功的标志位

std::vector<double> joint_group_positions(5); //机械臂正解的目标关节位置的数组


//目标色块对应逆解的目标关节角度回调函数
void color_ik_result_callback(const table_arm::color_ik_result &msg)
{
    joint_target1=msg.pedestal_angle;  //云台的目标角度
    joint_target2=msg.arm_angle;       //控制机械臂臂长的目标角度
    joint_target3=msg.hand_angle;      //控制夹取色块旋转的目标角度
    arm_state="ready";
    std::cout<<("#############################33");
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "color_sort_execute");
    ros::NodeHandle n;
    ros::NodeHandle nprivate("~");

    nprivate.param<float>("/auxiliary_angle", auxiliary_angle, 0.157);
    nprivate.param<float>("/link_a", link_a, 0.105);
    nprivate.param<float>("/link_b", link_b, 0.100);
    nprivate.param<float>("/link_c", link_c, 0.175);
    nprivate.param<float>("/link_h", link_h, 0.105);

    base_angle=acos((link_c-link_h)/link_a);  //计算机械臂夹爪可触底的关节基础角度

    ros::AsyncSpinner spinner(1);
    spinner.start();
   
    moveit::planning_interface::MoveGroupInterface arm("arm");  
    moveit::planning_interface::MoveGroupInterface hand("hand"); 
    moveit::planning_interface::MoveGroupInterface::Plan my_plan; 

    //arm.setGoalJointTolerance(0.01);
    //arm.setMaxAccelerationScalingFactor(0.2);
    //arm.setMaxVelocityScalingFactor(0.5);
    arm.setNamedTarget("arm_look"); arm.move(); sleep(1);    //机械臂运动到观测色块的位置
    hand.setNamedTarget("hand_open"); hand.move(); sleep(1);  //机械爪张开
    ros::Subscriber color_ik_result_sub=n.subscribe("color_ik_result",10,color_ik_result_callback); //订阅色块目标位置对应的关节角度信息

    ROS_INFO("color_sort_execute_node init successful");
    ROS_INFO("finding color ,waitting....");

    while(ros::ok())
   {
      if( arm_state=="ready" )
      {
        arm_state="working";
        ROS_INFO("joint_target_is    :(%4.2f)-(%4.2f)-(%4.2f)",joint_target1,joint_target2,joint_target3);
        ROS_INFO("color_sequence_is  :(%d)",color_sequence);
        //关节的目标旋转角度赋值
        joint_group_positions[0] =  joint_target1;
        joint_group_positions[1] =  -1.57-joint_target2+base_angle;
        joint_group_positions[2] =  joint_target2-base_angle;
        joint_group_positions[3] =  -1.57+joint_target2+auxiliary_angle;
        joint_group_positions[4] =  joint_target3;
        arm.setJointValueTarget(joint_group_positions); //输入目标关节数组
       // while(!arm_success)
        //{
          arm_success = ((arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)); //规划路径

          //ROS_INFO_NAMED("fk_demo_plan",success ? "plan_success" : "plan_False");
          arm.execute(my_plan),sleep(1); //如果规划成功则执行
        //}
        //arm_success=false;
                
        hand.setNamedTarget("hand_close");  //机械爪夹取
        while( !hand_close_success ) //判断是否规划成功，如果不成功则继续规划
        { 
           hand_close_success = ((hand.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)); //规划路径
           sleep(1);
        }
        hand_close_success=false;
 
        arm.setNamedTarget("arm_look");   arm.move();  sleep(1);    //机械臂运动到观测色块的位置
        hand.setNamedTarget("hand_open"); hand.move(); sleep(1);  //机械爪张开  //机械爪张开
        //arm.setNamedTarget("color_put_interval");  arm.move();  sleep(1); //机械臂臂身运动到放置色块的预位置后，再放置色块
        //arm_put(target_color); //根据颜色将色块放置到对应位置
      }
    ros::spinOnce();
  }
  ros::shutdown(); 
  return 0;
}

//一个完整的放置动作
void arm_put(std::string color)
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");
    //arm.setGoalJointTolerance(0.01);
    //arm.setMaxAccelerationScalingFactor(0.2);
    //arm.setMaxVelocityScalingFactor(0.6);
    //根据色块的颜色判断放置位置
    arm.setNamedTarget("yellow_put");  
    arm.move();  
    sleep(1);
   
    hand.setNamedTarget("hand_half_open");   //机械爪张开
    while( !hand_open_success )  //判断是否规划成功，如果不成功则继续规划
    { 
       hand_open_success = ((hand.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)); //规划路径
       sleep(1);
    }
    hand_open_success=false;

    arm.setNamedTarget("color_put_interval");   arm.move();  sleep(1);  //机械臂臂身运动到放置色块的预位置
    arm.setNamedTarget("arm_look");   arm.move();  sleep(1);      //机械臂运动到观测色块的位置
    hand.setNamedTarget("hand_open");  //机械爪张开
    while( !hand_open_success ) //判断是否规划成功，如果不成功则继续规划
    { 
       hand_open_success = ((hand.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)); //规划路径
       sleep(1);
    }
    hand_open_success=false;

    arm_state="ready";
}


