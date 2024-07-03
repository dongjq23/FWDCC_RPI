#include "ros/ros.h" 
#include "std_msgs/Float64.h"
#include <wiringPi.h>
// #include "std_msgs/Float64MultiArray.h"
#include <softPwm.h>

//PWM0
const int pwmPin_left = 26;  //orange 12 back left
const int pwmPin_fwd = 4;  //orange 12 back left
const int pwmPin_rev = 6;  //orange 12 back left
const int pwmPin_brack = 9;  //orange 12 back left

const int defaultPWM_left = 300;

int main(int argc, char *argv[ ])
{
    setlocale(LC_ALL,""); //中文显示
    ros::init(argc, argv, "pwm_node"); 
    ros::NodeHandle nh; //ros句柄

    //使用wiring编码初始化GPIO序号
    wiringPiSetup();
    pinMode(pwmPin_left, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);

    pwmSetClock(2.5); //19.2Mhz/1024/2.5=7.5Khz
    pwmSetRange(1024);
    pinMode(pwmPin_fwd, OUTPUT);
    pinMode(pwmPin_rev, OUTPUT);
    pinMode(pwmPin_brack, OUTPUT);

    // 置位GPIO引脚4、6和18为高电平
    digitalWrite(pwmPin_fwd, LOW);
    digitalWrite(pwmPin_rev, HIGH);
    digitalWrite(pwmPin_brack, HIGH);


    while(ros::ok())
    {
        pwmWrite(pwmPin_left,defaultPWM_left);//初始转速
    }
    ros::spin();
    return 0;
}
