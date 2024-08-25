#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from math import pi
from sensor_msgs.msg import JointState

# 弧度转角度
RA2DE = 180 / pi


def topic(msg):
    # 如果不是该话题的数据直接返回
    if not isinstance(msg, JointState): return
    # 定义关节角度容器,最后一个是夹爪的角度,默认夹爪不动为90.
    joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # 将接收到的弧度[-1.57,1.57]转换成角度[0,180]
    for i in range(6): print msg.position[i]
    # joints[i] = (msg.position[i] * RA2DE) +90
    # 调驱动函数for i in range(6):print joints[i]


if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node("rad2pwm")
    # 创建一个订阅者
    subscriber = rospy.Subscriber("/joint_states", JointState, topic)
    # 设置循环的频率
    #rate = rospy.Rate(20)
    # 按照循环频率延时
    rospy.spin()

