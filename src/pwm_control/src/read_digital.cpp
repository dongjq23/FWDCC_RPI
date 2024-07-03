#include <ros/ros.h>
// #include <std_msgs/Bool.h>
#include <wiringPi.h>
#include "std_msgs/Int32.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "gpio_publisher");
    ros::NodeHandle nh;
    ros::Publisher gpio_pub = nh.advertise<std_msgs::Int32>("gpio_state", 1);

    wiringPiSetup();  // 初始化WiringPi库

    const int gpioPin = 25;  // 你的GPIO引脚号

    pinMode(gpioPin, INPUT);
    // pullUpDnControl(gpioPin, PUD_UP);
    ros::Rate rate(10);  // 假设500Hz的发布频率
 
    while (ros::ok())
    {
        std_msgs::Int32 msg;
        msg.data = digitalRead(gpioPin);
        // ROS_INFO("引脚电平为：%d",msg.data);
        gpio_pub.publish(msg);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
