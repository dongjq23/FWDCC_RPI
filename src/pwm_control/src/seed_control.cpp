#include "ros/ros.h" 
#include "std_msgs/Float64.h"
#include <wiringPi.h>
// #include "std_msgs/Float64MultiArray.h"
// #include <std_msgs/Int32.h>
#include <softPwm.h>
/*
GPIO 10 正反转（低电平-反转）
GPIO 14 急停（低电平-急停；HIGH-不停）
GPIO 12 PWM（0-3.3v 软件pwm 调速）
*/

//排种器控制电机 软件PWM
const int seed_soft_pwm = 12; 
//排种器急停引脚
const int seed_brack = 14; 
//排种器正反转
// const int seed_rotation = 10; 
//排种器测速
// const int seed_vel = 15; 

// seed_soft_pwm = 0; // 初始化为0

void seed_pwmCallback(const std_msgs::Float64::ConstPtr& msg) {
    // 回调函数，用于处理从话题 "pwm_control" 接收到的数据
    // seed_soft_pwm = msg->data;
    softPwmWrite(seed_soft_pwm, msg->data); 
    ROS_INFO("Subscribe PWM value: %f", msg->data);
}

int main(int argc, char *argv[ ])
{
    setlocale(LC_ALL,""); //中文显示
    ros::init(argc, argv, "seed_pwm_node"); 
    ros::NodeHandle nh; //ros句柄

    //使用wiring编码初始化GPIO序号
    wiringPiSetup();
    //seed_soft_pwm 控制范围
    softPwmCreate(seed_soft_pwm, 0, 100);
    
    pinMode(seed_brack, OUTPUT);
    // pinMode(seed_rotation, OUTPUT);
    // pinMode(seed_vel, INPUT);

    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(2.5); //19.2Mhz/1024/2.5=7.5Khz
    pwmSetRange(1024);

    //关闭电机急停
    digitalWrite(seed_brack, HIGH);
    //设置为 反转
    // digitalWrite(seed_rotation, HIGH);
    
    ros::Subscriber sub = nh.subscribe("seed_pwm_control", 1, seed_pwmCallback);
    
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
