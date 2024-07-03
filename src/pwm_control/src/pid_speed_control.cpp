#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <algorithm> 

//变量定义
const int K = 20;
const double r = 0.175;

double current_speed_bl, current_speed_br, current_speed_fl, current_speed_fr;
double rpm_bl, rpm_br, rpm_fl, rpm_fr;
double pwm_bl, pwm_br, pwm_fl, pwm_fr;

double integral = 0.0;
double previous_error = 0.0;
double v_l;
double v_r;

double Kp;
double Ki;
double Kd;
bool flag;

ros::Publisher pwm_pub;
ros::Subscriber wheel_subscriber;
ros::Subscriber speed_sub;

//限制PWM值的范围
uint16_t limitToRange(uint16_t value, uint16_t min_val, uint16_t max_val) {
    return std::min(std::max(value, min_val), max_val);
}

//PID计算达到目标速度需要的转速
double calculateRPM(double target_speed, double speed_output) {
  
    // ROS_INFO("开始进行PID计算~");
    // 计算误差
    ROS_INFO("============================================================");
  
    double error = target_speed - speed_output;
    ROS_INFO("target_speed = %f ", target_speed);
    ROS_INFO("speed_output = %f ", speed_output);
    ROS_INFO("error = %f ", error);

    // 比例、积分和微分项
    double P = Kp * error;
    ROS_INFO("P = %f ", P);

    // double I = Ki * integral;
    // ROS_INFO("I = %f ", I);

    double D = Kd * (error - previous_error);
    ROS_INFO("D = %f ", D);
    speed_output = std::min(std::max(speed_output, 0.0), 1.8);
    // PID控制输出
    // speed_output += (P + I + D);
    speed_output += (P + D);

    if(speed_output < 0 ){
      // flag = false;
      integral = 0;
    }
    ROS_INFO("new_speed_output = %f ", speed_output);

    // 更新积分以供下一次迭代
    speed_output = std::min(std::max(speed_output, 0.0), 1.8);
    // integral += error;

    // 保存当前误差以供下一次迭代
    previous_error = error;

    double rpm = (60 * K * speed_output) / (2 * M_PI * r);

    // 限制 rpm 为非负值
    rpm = std::min(std::max(rpm, 0.0), 3300.0);

    //计算转速，从而获取对应的pwm值
    ROS_INFO("rpm = %f ", rpm);
    ROS_INFO("============================================================");
    return rpm;
}

//各轮速度订阅
void speedCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  std::vector<double> current_speed = msg->data;
  current_speed_bl = current_speed[0];
  current_speed_br = current_speed[1];
  current_speed_fl = current_speed[2];
  current_speed_fr = current_speed[3];


  current_speed_bl = std::min(std::max(current_speed_bl, 0.0), 1.8);
  current_speed_br = std::min(std::max(current_speed_br, 0.0), 1.8);
  current_speed_fl = std::min(std::max(current_speed_fl, 0.0), 1.8);
  current_speed_fr = std::min(std::max(current_speed_fr, 0.0), 1.8);
  // ROS_INFO("已经订阅到各个轮子的转速！！");


  // ROS_INFO("pid_speed_current_bl = %.3f m/s , pid_speed_current_br = %.3f m/s , pid_speed_current_fl = %.3f , pid_speed_current_fr = %.3f", current_speed_bl,current_speed_br, current_speed_fl, current_speed_fr);
}

void wheel_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // ROS_INFO("已经订阅到目标转速！！");
  std::vector<double> wheel_velocities = msg->data;
  v_l = wheel_velocities[0];
  v_r = wheel_velocities[1];
}

int main(int argc, char **argv) {

  setlocale(LC_ALL,"");
  ros::init(argc, argv, "pid_speed_controller");
  ros::NodeHandle nh;
  // double previous_pwm_fr = 0.0;

  //PID赋值-----------------------------------------------------------------------------------------------------
  //PID赋值-----------------------------------------------------------------------------------------------------
  Kp = 0.7;
  if(nh.getParam("Kp", Kp)){ 
        ROS_INFO("Loaded Kp values from parameter server");
    }else{
        ROS_ERROR("Failed to load Kp values from parameter server. Using default values.");
    } 

  Ki = 0.01;
  if(nh.getParam("Ki", Ki)){  
        ROS_INFO("Loaded Ki values from parameter server");
    }else{
        ROS_ERROR("Failed to load Ki values from parameter server. Using default values.");
      } 

  Kd = 0.05;
  if(nh.getParam("Kd", Kd)){ 
        ROS_INFO("Loaded Kd values from parameter server");
      }else{
        ROS_ERROR("Failed to load Kd values from parameter server. Using default values.");
    } 

  //PID赋值完毕-----------------------------------------------------------------------------------------------------
  //PID赋值完毕-----------------------------------------------------------------------------------------------------
  

  //发布pwm的话题句柄
  pwm_pub = nh.advertise<std_msgs::Float64MultiArray>("/pwm_control", 1);

  //订阅获取各轮当前转速
  speed_sub = nh.subscribe("/current_velocities", 1, speedCallback);
  ros::spinOnce();
  wheel_subscriber = nh.subscribe<std_msgs::Float64MultiArray>("wheel_velocities", 1, wheel_Callback);
  ros::Duration(0.8).sleep();
  ros::spinOnce();
  // ros::spinOnce();
  ros::Rate loop_rate(100);


  while (ros::ok()) {

    std_msgs::Float64MultiArray pwm_msg;
    //订阅获取目标转速
    rpm_bl = calculateRPM(v_l,current_speed_bl);
    pwm_bl= (134.71 + rpm_bl) / 2.1005;
    pwm_bl = limitToRange(pwm_bl, 0, 1023);
    pwm_msg.data.push_back(pwm_bl);


    rpm_br = calculateRPM(v_r,current_speed_br);
    pwm_br= (134.71 + rpm_br) / 2.1005;
    pwm_br = limitToRange(pwm_br, 0, 1023);
    pwm_msg.data.push_back(pwm_br);


    rpm_fl = calculateRPM(v_l,current_speed_fl);
    pwm_fl= (161.66 + rpm_fl) / 21.835;
    pwm_fl = limitToRange(pwm_fl, 0, 1023);
    pwm_msg.data.push_back(pwm_fl);


    rpm_fr = calculateRPM(v_r,current_speed_fr);
    pwm_fr= (161.66 + rpm_fr) / 21.835;
    pwm_fr = limitToRange(pwm_fr, 0, 1023);
    pwm_msg.data.push_back(pwm_fr);

    // ROS_INFO("rpm_bl = %.3f m/s , rpm_br = %.3f m/s , rpm_fl = %.3f , rpm_fr = %.3f ", rpm_bl, rpm_br, rpm_fl, rpm_fr);


    pwm_pub.publish(pwm_msg);

    // if (std::abs(pwm_fr - previous_pwm_fr) > 2.0) {
    // ros::Duration(0.2).sleep();
    // }

    // previous_pwm_fr = pwm_fr;

    ros::Duration(0.8).sleep(); //避免软件速度太快，硬件发布pwm跟不上
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

