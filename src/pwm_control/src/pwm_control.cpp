#include "ros/ros.h" 
#include "std_msgs/Float64.h"
#include <wiringPi.h>
#include "std_msgs/Float64MultiArray.h"
#include <softPwm.h>

//PWM0
const int pwmPin_left = 26;  //orange 12 back left
//pwm1
const int pwmPin_right = 23;  // red ros

// //软件PWM4
const int softPwm_fl = 27; //fl

// //软件PWM4
const int softPwm_fr = 0; //fr

// const int pwmPin_fwd_r = 4;  //向前转
// const int pwmPin_rev_r = 6;  //向后转
// const int pwmPin_brack_r = 8;  //急停

// const int pwmPin_fwd_l = 30;  //向前转
// const int pwmPin_rev_l = 31;  //向后转
// const int pwmPin_brack_l = 9;  //急停


// const int defaultPWM_left = 300;
// const int defaultPWM_right = 300;
// const int defaultSoftPWM_fl = 20;
// const int defaultSoftPWM_fr = 20;

void pwmCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    pwmWrite(pwmPin_left,msg -> data[0]);
    pwmWrite(pwmPin_right,msg -> data[1]);
    softPwmWrite(softPwm_fl, msg -> data[2]);
    softPwmWrite(softPwm_fr, msg -> data[3]);
}

int main(int argc, char *argv[ ])
{
    setlocale(LC_ALL,""); //中文显示
    ros::init(argc, argv, "pwm_node"); 
    ros::NodeHandle nh; //ros句柄

    //使用wiring编码初始化GPIO序号
    wiringPiSetup();
  
    // pinMode(pwmPin_fwd_l, OUTPUT);
    // pinMode(pwmPin_rev_l, OUTPUT);
    // pinMode(pwmPin_brack_l, OUTPUT);

    // pinMode(pwmPin_fwd_r, OUTPUT);
    // pinMode(pwmPin_rev_r, OUTPUT);
    // pinMode(pwmPin_brack_r, OUTPUT);

    pinMode(pwmPin_left, PWM_OUTPUT);
    pinMode(pwmPin_right, PWM_OUTPUT);
    softPwmCreate(softPwm_fl, 0, 100);
    softPwmCreate(softPwm_fr, 0, 100);
    
    // // 设定电机正转
    // digitalWrite(pwmPin_fwd_l, HIGH);
    // digitalWrite(pwmPin_rev_l, LOW);
    // digitalWrite(pwmPin_brack_l, HIGH);

    // digitalWrite(pwmPin_fwd_r, LOW);
    // digitalWrite(pwmPin_rev_r, HIGH);
    // digitalWrite(pwmPin_brack_r, HIGH);

    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(2.5); //19.2Mhz/1024/2.5=7.5Khz
    pwmSetRange(1024);

    // pwmWrite(pwmPin_left,defaultPWM_left);//初始转速
    // pwmWrite(pwmPin_right,defaultPWM_right);
    // softPwmWrite(softPwm_fl,defaultSoftPWM_fl);
    // softPwmWrite(softPwm_fr,defaultSoftPWM_fr);

    ros::Subscriber sub = nh.subscribe("pwm_control", 1, pwmCallback);
    
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
