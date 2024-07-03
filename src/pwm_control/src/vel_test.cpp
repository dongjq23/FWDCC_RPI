#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <wiringPi.h>
#include <cmath>

const int motorPin_bl = 25;
const int motorPin_br = 5;
const int motorPin_fl = 3;
const int motorPin_fr = 2;

const int K = 20;
const float r = 0.175f;

// const unsigned long timeoutDuration = 60000;  // 设置超时时间，单位为纳秒
double speed_bl = 0.0, speed_br = 0.0 , speed_fl = 0.0 , speed_fr = 0.0;
unsigned long startTime_bl, startTime_br, startTime_fl, startTime_fr;
unsigned long endTime_bl, endTime_br, endTime_fl, endTime_fr;
unsigned long period_bl, period_br, period_fl, period_fr;
unsigned int count_bl = 1, count_br = 1, count_fl = 1, count_fr = 1;

ros::Time lastPulseTime_bl, lastPulseTime_br, lastPulseTime_fl, lastPulseTime_fr;
ros::Duration elapsedTime_bl, elapsedTime_br, elapsedTime_fl, elapsedTime_fr;


void interruptHandler_bl() {
    lastPulseTime_bl = ros::Time::now();
  
    if (count_bl == 1) {
        // 记录起始时间
        startTime_bl = micros();
        // ROS_INFO("startTime_bl = %lu", startTime_bl);

    } else if (count_bl == 13) {

        // 记录终止时间，计算周期
        endTime_bl = micros();
        // ROS_INFO("endTime_bl = %lu", endTime_bl);

        period_bl = endTime_bl - startTime_bl;
        // ROS_INFO("period_bl: %lu", period_bl);

        double frequency_bl = 1000000.0 / (period_bl / 12);
        double rpm_bl = 20.0 * frequency_bl / 5.0;
        speed_bl = 2 * M_PI * r * rpm_bl / (60 * K);
        // ROS_INFO("speed_br = %f", speed_bl);

        count_bl = 0;
    }
    count_bl++;


}


void interruptHandler_br() {
  

    lastPulseTime_br = ros::Time::now();

    if (count_br == 1) {
        startTime_br = micros();
        // ROS_INFO("startTime_br = %lu", startTime_br);  

    } else if (count_br == 13) {
        // 记录终止时间，计算周期
        endTime_br = micros();
        // ROS_INFO("endTime_br = %lu", endTime_br);
        period_br = endTime_br - startTime_br;
        // ROS_INFO("period_br = %lu", period_br);

        double frequency_br = 1000000.0 / (period_br / 12);
        double rpm_br = 20.0 * frequency_br / 5.0;
        speed_br = 2 * M_PI * r * rpm_br / (60 * K);
        // ROS_INFO("speed_br = %f", speed_br);

        count_br = 0;
    }
count_br++;
 
}

void interruptHandler_fl() {
    lastPulseTime_fl = ros::Time::now();

    if (count_fl == 1) {
        // 记录起始时间
        startTime_fl = micros();
    } else if (count_fl == 13) {
        // 记录终止时间，计算周期
        endTime_fl = micros();
        period_fl = endTime_fl - startTime_fl;
        // ROS_INFO("period_fl: %lu", period_fl);
        double frequency_fl = 1000000.0 / (period_fl / 12);
        double rpm_fl = 20.0 * frequency_fl / 5.0;
        speed_fl = 2 * M_PI * r * rpm_fl / (60 * K);
        count_fl = 0;
    }
    count_fl++;
}

void interruptHandler_fr() {
    lastPulseTime_fr = ros::Time::now();

    if (count_fr == 1) {
        // 记录起始时间
        startTime_fr = micros();
    } else if (count_fr == 13) {
        // 记录终止时间，计算周期
        endTime_fr = micros();
        period_fr = endTime_fr - startTime_fr;
        // ROS_INFO("period_fr: %lu", period_fr);
        double frequency_fr = 1000000.0 / (period_fr / 12);
        double rpm_fr = 20.0 * frequency_fr / 5.0;
        speed_fr = 2 * M_PI * r * rpm_fr / (60 * K);
        count_fr = 0;
    }
    count_fr++;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "motor_speed_test");
    ros::NodeHandle nh;

    wiringPiSetup();

    pinMode(motorPin_bl, INPUT);
    pinMode(motorPin_br, INPUT);
    pinMode(motorPin_fl, INPUT);
    pinMode(motorPin_fr, INPUT);

    // wiringPiISR(motorPin_br, INT_EDGE_RISING, &interruptHandler_br);
    wiringPiISR(motorPin_bl, INT_EDGE_RISING, &interruptHandler_bl);
    wiringPiISR(motorPin_br,  INT_EDGE_RISING, &interruptHandler_br);
    wiringPiISR(motorPin_fl, INT_EDGE_RISING, &interruptHandler_fl);
    wiringPiISR(motorPin_fr, INT_EDGE_RISING, &interruptHandler_fr);

    ros::Publisher speed_pub = nh.advertise<std_msgs::Float64MultiArray>("/current_velocities", 1);  

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        elapsedTime_br = ros::Time::now() - lastPulseTime_br;
        elapsedTime_bl = ros::Time::now() - lastPulseTime_bl;
        elapsedTime_fl = ros::Time::now() - lastPulseTime_fl;
        elapsedTime_fr = ros::Time::now() - lastPulseTime_fr;

        if (elapsedTime_br.toSec() > 0.1) {  
            speed_br = 0.0;
        }
        if (elapsedTime_bl.toSec() > 0.1) { 
            speed_bl = 0.0;
        }
        if (elapsedTime_fl.toSec() > 0.1) {  
            speed_fl = 0.0;
        }
        if (elapsedTime_fr.toSec() > 0.1) { 
            speed_fr = 0.0;
        }

        std_msgs::Float64MultiArray msg;

        msg.data.push_back(speed_bl);
        msg.data.push_back(speed_br);
        msg.data.push_back(speed_fl);
        msg.data.push_back(speed_fr);

        ROS_INFO("Forward Left Speed: %f m/s, Forward Right Speed: %f m/s, Back Left Speed: %f m/s, Back Right Speed: %f m/s",speed_fl, speed_fr, speed_bl, speed_br);
        ROS_INFO(" Back Right Speed: %f m/s", speed_br);

        speed_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
