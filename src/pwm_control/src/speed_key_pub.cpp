#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "seed_value_publisher");

    // 创建 ROS 句柄
    ros::NodeHandle nh;

    // 创建一个 publisher，用于发布消息到 "seed_value" 话题
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("seed_control_value", 1);

    // 设置发布频率
    ros::Rate rate(100); // 假设发布频率为1Hz

    char input;
    struct termios oldt, newt;

    // 保存旧的终端设置
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // 设置终端为不回显和无缓冲模式
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    double pwm_value = 100;
    nh.getParam("seed_pwm_value", pwm_value);

    // 循环发布消息
    while (ros::ok())
    {
        // 创建一个 Float64 消息
        std_msgs::Float64 msg;
        msg.data = pwm_value;

        // 发布消息
        pub.publish(msg);

        // 输出发布的消息
        ROS_INFO("Published seed value: %f", msg.data);
        input = getchar();
        // 根据输入调整 pwm_value 的值
            if (input == 'a')
            {
                pwm_value = std::min(pwm_value + 0.03 * pwm_value, 100.0);
            }
            else if (input == 's')
            {
                pwm_value = std::max(pwm_value - 0.03 * pwm_value, 30.0);
            }

        // 等待一段时间
        rate.sleep();
    }
    // 恢复旧的终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);


    return 0;
}

