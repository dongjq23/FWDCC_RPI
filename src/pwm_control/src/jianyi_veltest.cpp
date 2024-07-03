#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <wiringPi.h>
#include <cmath>


const int motorPin_br = 5;


const int K = 20;
const float r = 0.175f;

const unsigned long timeoutDuration = 60000;  // 设置超时时间，单位为纳秒
double speed_bl = 0.0, speed_br = 0.0 , speed_fl = 0.0 , speed_fr = 0.0;
unsigned long startTime_bl, startTime_br, startTime_fl, startTime_fr;
unsigned long endTime_bl, endTime_br, endTime_fl, endTime_fr;
unsigned long period_bl, period_br, period_fl, period_fr;
unsigned int count_bl = 1, count_br = 1, count_fl = 1, count_fr = 1;

ros::Time lastPulseTime;


void interruptHandler_br() {
    lastPulseTime = ros::Time::now();
    if (count_br == 1) {
    // isRisingEdgeDetected = true;
    // 记录起始时间
        ROS_INFO("==========================================================================");
        startTime_br = micros();
        ROS_INFO("startTime_br = %lu", startTime_br);
        
    } else if (count_br == 13) {
        // 记录终止时间，计算周期
        endTime_br = micros();
        ROS_INFO("endTime_br = %lu", endTime_br);
        period_br = endTime_br - startTime_br;
        ROS_INFO("period_br = %lu", period_br);
        double frequency_br = 1000000.0 / (period_br / 12);
        double rpm_br = 20.0 * frequency_br / 5.0;
        speed_br = 2 * M_PI * r * rpm_br / (60 * K);
        ROS_INFO("speed_br = %f", speed_br);
        ROS_INFO("==========================================================================");
        count_br = 0;
        // isRisingEdgeDetected = false;
    }
count_br++;
}


// void timerCallback(const ros::TimerEvent& event) {
//     if (isLow) {
//         // 如果在定时器到期之前引脚一直为低电平，将速度赋值为0
//         speed_br = 0.0;
//     }
// }


int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "motor_speed_test");
    ros::NodeHandle nh;

    wiringPiSetup();
    pinMode(motorPin_br, INPUT);

    wiringPiISR(motorPin_br,  INT_EDGE_BOTH, &interruptHandler_br);

    ros::Publisher speed_pub = nh.advertise<std_msgs::Float64MultiArray>("/current_velocities", 1);  
    // ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCallback);  // 定时器周期可以根据实际情况调整

    // timer.stop();  // 初始时停止定时器
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::Duration elapsedTime = ros::Time::now() - lastPulseTime;

        if (elapsedTime.toSec() > 0.1) {  // 0.1秒是一个示例阈值，可以根据需要调整
            speed_br = 0.0;
        }

        std_msgs::Float64MultiArray msg;

        msg.data.push_back(speed_bl);
        msg.data.push_back(speed_br);
        msg.data.push_back(speed_fl);
        msg.data.push_back(speed_fr);
        ROS_INFO(" Back Right Speed: %f m/s", speed_br);
        speed_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
