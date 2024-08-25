#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
# 角度转弧度
DE2RA = pi / 180

class MovePickMove:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('move_pick_move')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')

        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('gripper')
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'root_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
	      
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.05)
        #gripper.set_goal_joint_tolerance(0.05)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        gripper.set_named_target('close')
        gripper.go()
        rospy.sleep(1)
	
        # 解析接收到的JSON数据
        json_file = ("/home/tbs/le_arm/src/le_arm_planning/scripts/target_pose.json")
        json_data = json.load(open(json_file)) #load loads***
        x = json_data['x']
        y = json_data['y']
        z = json_data['z']
        roll = json_data['roll']
        pitch = json_data['pitch']
        yaw = json_data['yaw']
        # RPY转四元素
        q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)

     
        # 设置机械臂工作空间中的抓取目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于root_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 四元数转RPY
        q = euler_from_quaternion([target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w])
        print(q)

        arm.set_pose_target(target_pose, end_effector_link)
        #arm.set_joint_value_target(target_pose, end_effector_link,True)

        # 规划运动路径
        traj = arm.plan()
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        # 控制夹爪闭合
        gripper.set_named_target('close')
        gripper.go()
        rospy.sleep(2)


        #中间以forward姿态保持抓取状态
        arm.set_named_target('forward')
        arm.go()
        rospy.sleep(2)

           
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        # 控制夹闭合
        gripper.set_named_target('close')
        gripper.go()
        rospy.sleep(2)
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
	try:    
		MovePickMove()
	except rospy.ROSInterruptException:
		pass
    
    
