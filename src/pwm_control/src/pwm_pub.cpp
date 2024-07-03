#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include <vector>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"control_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::UInt16MultiArray>("pwm_control",1);

    //从参数服务器中读取数据
    std::vector<int> pwmValues;
    if (nh.getParam("pwm_values", pwmValues))  // 注意：使用参数服务器的全局路径
    {
        ROS_INFO("Loaded PWM values from parameter server");
    }
    else
    {
        ROS_ERROR("Failed to load PWM values from parameter server. Using default values.");
        pwmValues = {0, 0, 0, 0};  // 默认值
    }

    ros::Rate rate(100);
    while (ros::ok())
    {
        std_msgs::UInt16MultiArray msg;
        msg.data.resize(pwmValues.size());
        std::transform(pwmValues.begin(), pwmValues.end(), msg.data.begin(),
                       [](int value) { return static_cast<uint16_t>(value); });
        pub.publish(msg);
        rate.sleep();
        ros::spinOnce();
        // ROS_INFO("msg.data=%.2f",msg.data);
    }
    return 0;
}