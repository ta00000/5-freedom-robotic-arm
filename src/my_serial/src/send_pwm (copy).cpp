#include <ros/ros.h> 
#include <sstream>
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <string>

#define scaler 423
#define offset 1500
using namespace std;

serial::Serial ser; //声明串口对象 

//回调函数 
void PWMInfoCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int pwm[6];
   
    pwm[0]=-msg->position[2] * scaler*(-1) + offset;
    pwm[1]=-msg->position[4] * scaler + offset;
    pwm[2]=msg->position[1] * scaler + offset;
    pwm[3]=msg->position[3] * scaler*(-1) + offset;
    pwm[4]=-msg->position[5] * scaler*(-1) + offset;
    pwm[5]=msg->position[0] * scaler*(-1) + offset;
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
    if(pwm[5]>1950)
    {
        pwm[5]=1950;
    }
    else if(pwm[5]<1000)
    {
        pwm[5]=1000;
    }
	 // 将接收到的消息打印出来
    ROS_INFO("Subcribe sensor_msgs/JointState Info: position:%d %d %d %d %d %d", pwm[0],pwm[1],pwm[2], pwm[3],pwm[4],pwm[5]);
	
    //string pwm_send="#004P1900T1000!";
	stringstream pwm0;
	pwm0 << pwm[0];
	string ss0=pwm0.str();
	string pwm_send0="#000P"+ss0+"T0100!";
	stringstream pwm1;
	pwm1 << pwm[1];
	string ss1=pwm1.str();
	string pwm_send1="#001P"+ss1+"T0100!";
	stringstream pwm2;
	pwm2 << pwm[2];
	string ss2=pwm2.str();
	string pwm_send2="#002P"+ss2+"T0100!";
	stringstream pwm3;
	pwm3 << pwm[3];
	string ss3=pwm3.str();
	string pwm_send3="#003P"+ss3+"T0100!";	
	stringstream pwm4;
	pwm4 << pwm[4];
	string ss4=pwm4.str();
	string pwm_send4="#004P"+ss4+"T0100!";
    stringstream pwm5;
	pwm5 << pwm[5];
	string ss5=pwm5.str();
	string pwm_send5="#005P"+ss5+"T0100!";
    string pwm_send="{"+pwm_send0+pwm_send1+pwm_send2+pwm_send3+pwm_send4+pwm_send5+"}";
	//ROS_INFO("Subcribe sensor_msgs/JointState Info: position:%s", &pwm_send);
    ser.write(pwm_send); 
	ROS_INFO("ok");

}



int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "send_pwm"); 
    //声明节点句柄
    ros::NodeHandle nh; 

    //订阅主题，并配置回调函数 
    ros::Subscriber rad2pwm_info_sub = nh.subscribe("/joint_states", 10, PWMInfoCallback);

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    //指定循环的频率 
    ros::Rate loop_rate(50); 
    while(ros::ok()) 
    { 

        if(ser.available())
        { 
            ROS_INFO_STREAM("Reading from serial port\n"); 
            std_msgs::String result; 
            result.data = ser.read(ser.available()); 
            ROS_INFO_STREAM("Read: " << result.data); 
        } 

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spin(); 
        loop_rate.sleep(); 

    } 
} 
