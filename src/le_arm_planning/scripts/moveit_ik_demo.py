#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')

        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('gripper')
        
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        print(end_effector_link)          
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'root_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
	      
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.05)
        gripper.set_goal_joint_tolerance(0.05)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        gripper.set_named_target('open')
        gripper.go()
        rospy.sleep(2)
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于root_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = -0.213674
        target_pose.pose.position.y = 0.244203
        target_pose.pose.position.z = 0.0856214

		
        '''# RPY的单位是角度值
        roll = 90.0
        pitch = 0.0
        yaw = 0.0
    	  # RPY转四元数
        q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)
        print(q)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        '''


        target_pose.pose.orientation.x = 0.664344
        target_pose.pose.orientation.y = -0.305322
        target_pose.pose.orientation.z = -0.619889
        target_pose.pose.orientation.w = -0.284892
        # 四元数转RPY
        q = euler_from_quaternion([target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w])
        print(q)
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        #arm.set_joint_value_target(target_pose, end_effector_link,True)
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()

        plan = arm.plan()
        if plan.joint_trajectory.points:  # True if trajectory contains points
          print('yes')
        else:
          rospy.logerr("Trajectory is empty. Planning was unsuccessful.")


        '''# 多次执行,提高成功率
        for i in range(5):
        # 运动规划
        traj = arm.plan()
        if len(traj.joint_trajectory.points) != 0:
          print("plan success")
          # 规划成功后运行
          arm.execute(traj)
          rospy.sleep(3)
          break
        else:
          print("plan error")'''

        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        # 控制夹爪闭合
        gripper.set_named_target('close')
        gripper.go()
        rospy.sleep(2)

        # 控制机械臂终端向右移动5cm
        '''arm.shift_pose_target(4, 1.57, end_effector_link)
        arm.go()
        rospy.sleep(1)'''
  
        # 控制机械臂终端反向旋转90度
        #arm.shift_pose_target(5, -0.06, end_effector_link)
        #arm.go()
        #rospy.sleep(1)
           
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        # 控制夹开启
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
    
    
