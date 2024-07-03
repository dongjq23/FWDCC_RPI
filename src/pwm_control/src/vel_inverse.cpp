// 已知角速度和线速度，根据四轮差速运动学模型，求单侧左右轮逆解
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include <wiringPi.h>

const int pwmPin_fwd_l = 30;  //向前转
const int pwmPin_rev_l = 31;  //向后转
const int pwmPin_brack_l = 9;  //急停

const int pwmPin_fwd_r = 4;  //向前转
const int pwmPin_rev_r = 6;  //向后转
const int pwmPin_brack_r = 8;  //急停

double v, w, c;

void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
    // 提取线速度和角速度
    v = twist_msg->linear.x;  // x方向线速度
    w = twist_msg->angular.z; // 绕z轴角速度

    // ROS_INFO("Received Twist message - Linear Velocity (v): %.2f, Angular Velocity (w): %.2f", v, w);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    wiringPiSetup();
    pinMode(pwmPin_fwd_l, OUTPUT);
    pinMode(pwmPin_rev_l, OUTPUT);
    pinMode(pwmPin_brack_l, OUTPUT);

    pinMode(pwmPin_fwd_r, OUTPUT);
    pinMode(pwmPin_rev_r, OUTPUT);
    pinMode(pwmPin_brack_r, OUTPUT);

    // 初始化ROS节点
    ros::init(argc, argv, "wheel_speed_calculator");
    ros::NodeHandle nh;
    // 获取车辆参数
    ros::Publisher wheels_publisher = nh.advertise<std_msgs::Float64MultiArray>("wheel_velocities", 1);
    std_msgs::Float64MultiArray wheels_msg;

    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, twistCallback);

    ros::Rate loop_rate(100);

    while (ros::ok()){
        

        if(nh.getParam("/wheel_distance", c))
        {
            //  ROS_INFO("Loaded wheel_distance from parameter server");
        }else{
            // ROS_ERROR("Failed to load wheel_distance from parameter server. Using default values.");
            c = 0.5; 
        }

        double Vl = v - (c * w) / 2;
        double Vr = v + (c * w) / 2;

        ROS_INFO("Left Wheel Speed: %.3f m/s ,Right Wheel Speed: %.3f m/s", Vl,Vr);
        
        //左侧正反转判断
        if (Vl > 0) {
            // ROS_INFO("此时电机正转，向前走");
            digitalWrite(pwmPin_fwd_l, LOW);
            digitalWrite(pwmPin_rev_l, HIGH);
            digitalWrite(pwmPin_brack_l, HIGH);

        } else {
  
            if (Vl < 0) {
            // ROS_INFO("此时电机反转，向后走");
            Vl = abs(Vl);
            digitalWrite(pwmPin_fwd_l, HIGH);
            digitalWrite(pwmPin_rev_l, LOW);
            digitalWrite(pwmPin_brack_l, HIGH);

            } else {
            // ROS_INFO("此时为急停状态");
           
            Vl = abs(Vl);
            // 当 Vl 为零时引脚置位为零
            digitalWrite(pwmPin_fwd_l, HIGH);
            digitalWrite(pwmPin_rev_l, HIGH);
            digitalWrite(pwmPin_brack_l, LOW);
        }

    }
        //右侧轮判断
        if (Vr > 0) {
            // ROS_INFO("此时电机正转，向前走");
            digitalWrite(pwmPin_fwd_r, HIGH);
            digitalWrite(pwmPin_rev_r, LOW);
            digitalWrite(pwmPin_brack_r, HIGH);

        } else {
  
            if (Vr < 0) {
            // ROS_INFO("此时电机反转，向后走");
            // Vl = abs(Vl);
            Vr = abs(Vr);
            digitalWrite(pwmPin_fwd_r, LOW);
            digitalWrite(pwmPin_rev_r, HIGH);
            digitalWrite(pwmPin_brack_r, HIGH);

            } else {
            // ROS_INFO("此时为急停状态");
            // Vl = abs(Vl);
            Vr = abs(Vr);
            // 当 Vl 为零时引脚置位为零
            digitalWrite(pwmPin_fwd_r, HIGH);
            digitalWrite(pwmPin_rev_r, HIGH);
            digitalWrite(pwmPin_brack_r, LOW);
        }

    }
        // 将结果发布
        // vl_msg.data = Vl;
        // vr_msg.data = Vr;
        
        wheels_msg.data.clear();
        wheels_msg.data.push_back(Vl);
        wheels_msg.data.push_back(Vr);
        wheels_publisher.publish(wheels_msg);
  
        // ROS_INFO("Left Wheel Speed: %.3f m/s ,Right Wheel Speed: %.3f m/s", Vl,Vr);
       
        ros::spinOnce();
        // 按指定频率休眠，以维持循环频率
        loop_rate.sleep();
    }
    return 0;
}
