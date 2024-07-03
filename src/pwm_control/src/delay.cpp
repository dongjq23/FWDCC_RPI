#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "your_node");
  ros::NodeHandle nh;

  // 获取延时参数
  double delay_duration;
  nh.getParam("/pwm_control/delay_duration", delay_duration);

  // 进行延时
  ros::Duration(delay_duration).sleep();

  // 其他节点启动代码

  ros::spin();
  return 0;
}
