#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/JointState.h>

#define scaler 423
#define offset 1500

using namespace std;

// 接收到订阅的消息后，会进入消息回调函数
void PWMInfoCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int pwm[6];
    // 将接收到的消息打印出来
    ROS_INFO("Subcribe sensor_msgs/JointState Info: position:%lf", 
			 msg->position[0]);
    pwm[0]=msg->position[0] * scaler + offset;
    pwm[1]=msg->position[1] * scaler + offset;
    pwm[2]=msg->position[2] * scaler + offset;
    pwm[3]=msg->position[3] * scaler + offset;
    pwm[4]=msg->position[4] * scaler + offset;
    pwm[5]=msg->position[5] * scaler + offset;
    for(int i=0;i<5;i++)
    {
        if(pwm[i]>2500)
        {
            pwm[i]=2500;
        }
        else if(pwm[i]<500)
        {
            pwm[i]=500;
        }
    }
    if(pwm[5]>1900)
    {
        pwm[5]=1900;
    }
    else if(pwm[5]<1500)
    {
        pwm[5]=1500;
    }

    string pwm_send="#005P1900T1000!";
    //pwm_send="{#000P"+1500+"T1000!#00"1P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!}"

}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "rad2pwm");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/joint_states 的topic，注册回调函数pPWMInfoCallback
    ros::Subscriber rad2pwm_info_sub = n.subscribe("/joint_states", 10, PWMInfoCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
