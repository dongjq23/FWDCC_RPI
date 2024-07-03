#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"


// const int seed_brack = 14;


double pwm_value = 100;
void seedValueCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // 更新 pwm_value 的值

    pwm_value = msg->data;
}

int main(int argc, char **argv)
{

    // 初始化 ROS 节点
    ros::init(argc, argv, "seed_pwm_publisher");
    // 创建 ROS 句柄
    ros::NodeHandle nh;
    // 创建一个 publisher，用于发布消息到 "pwm_control" 话题
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("seed_pwm_control", 1);

    ros::Subscriber sub = nh.subscribe("seed_control_value", 1, seedValueCallback);
    // 设置发布频率
    ros::Rate rate(100); // 假设发布频率为10Hz
    // 循环发布消息

    nh.getParam("seed_pwm_value", pwm_value);
  
    while (ros::ok())
    {
        std_msgs::Float64 msg;
        msg.data = pwm_value; // 设置消息的数据
        pub.publish(msg);
        ROS_INFO("Published PWM value: %f", msg.data);
        rate.sleep();
        ros::spinOnce();
    }

  
    return 0;
}
