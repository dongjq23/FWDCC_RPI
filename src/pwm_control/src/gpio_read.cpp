#include <ros/ros.h>
#include <wiringPi.h>

const int gpioPin = 7;


int main(int argc, char** argv) {
    ros::init(argc, argv, "gpio_reader_node");
    ros::NodeHandle nh;

    // 初始化WiringPi库
    wiringPiSetup();

    // 设置GPIO引脚为输入模式
    pinMode(gpioPin, INPUT);

    ros::Rate loop_rate(10); // 设置循环频率

    while (ros::ok()) {
        // 读取GPIO引脚的电平值
        int value = digitalRead(gpioPin);

        ROS_INFO("GPIO Pin %d Value: %d", gpioPin, value);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
