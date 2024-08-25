#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
from moveit_msgs.msg import RobotTrajectory
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, atan
# 角度转弧度
DE2RA = pi / 180

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_test')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')

        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('gripper')

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        print('This is moveit_ik_test.py!')

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
	      
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.005)
        arm.set_goal_orientation_tolerance(0.005)

        # 解析接收到的JSON数据
        json_file = ("/home/tbs/le_arm/src/le_arm_planning/scripts/target_pose.json")
        json_data = json.load(open(json_file)) #load loads**
        car_x = json_data['car_x']
        car_y = json_data['car_y']
        car_theta = json_data['car_theta']
        is_arrive = json_data['is_arrive']
        

        # 创建一个tf广播器
        tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        if is_arrive == 1:
            # 创建一个TransformStamped消息
            transform_stamped = TransformStamped()

            # 设置消息的元数据
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = reference_frame  # 世界坐标系
            transform_stamped.child_frame_id = "root_link"  # 机身坐标系

            # 设置变换关系
            transform_stamped.transform.translation.x = car_x  # 在x轴上的平移
            transform_stamped.transform.translation.y = car_y  # 在y轴上的平移
            transform_stamped.transform.translation.z = 0.0  # 在z轴上的平移
            quaternion = quaternion_from_euler(0.0, 0.0, car_theta * DE2RA)  # 欧拉角转换为四元数
            transform_stamped.transform.rotation.x = quaternion[0]
            transform_stamped.transform.rotation.y = quaternion[1]
            transform_stamped.transform.rotation.z = quaternion[2]
            transform_stamped.transform.rotation.w = quaternion[3]

            # 发布变换关系
            tf_broadcaster.sendTransform(transform_stamped)


        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        gripper.set_named_target('open')
        gripper.go()
        rospy.sleep(1)

        #计算目标位姿
        goal_x = json_data['goal_x']
        goal_y = json_data['goal_y']
        goal_z = json_data['goal_z']
        goal_roll = json_data['goal_roll']
        goal_pitch = json_data['goal_pitch']
        if (goal_x-car_x) > 0:
            goal_yaw = atan((goal_y-car_y)/(goal_x-car_x))
        elif (goal_x-car_x) ==0 and (goal_y-car_y) > 0:
            goal_yaw = pi/2
        elif (goal_x-car_x) ==0 and (goal_y-car_y) < 0:
            goal_yaw = -pi/2
        elif (goal_x-car_x) < 0 :
            goal_pitch = -180 - goal_pitch
            goal_yaw = atan((goal_y-car_y)/(goal_x-car_x))

        # RPY转四元素, 'rzyx'绕非定轴
        q = quaternion_from_euler(goal_yaw, goal_pitch * DE2RA, goal_roll * DE2RA, 'rzyx')

        # 设置机械臂工作空间中的抓取目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于world坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = goal_x
        target_pose.pose.position.y = goal_y
        target_pose.pose.position.z = goal_z
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]   
        target_pose.pose.orientation.w = q[3]

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        arm.set_pose_target(target_pose, end_effector_link)
        #arm.set_joint_value_target(target_pose, end_effector_link,True)

        # 规划运动路径
        traj = arm.plan()
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        gripper.set_named_target('close')
        gripper.go()
        rospy.sleep(5)
           
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        gripper.set_named_target('open')
        gripper.go()
        rospy.sleep(2)
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
	try:    
		MoveItIkDemo()
	except rospy.ROSInterruptException:
		pass
    
    
