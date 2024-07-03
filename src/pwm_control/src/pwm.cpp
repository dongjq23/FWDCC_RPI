#include <wiringPi.h>
#define pwmpin 26
#define mode PWM_MODE_MS
 
int main(void)
{
    wiringPiSetup();
    pinMode(pwmpin,PWM_OUTPUT);
    pwmSetMode(mode);
    
    while(1){
        pwmWrite(pwmpin,512);
        delay(1);
    // pwmWrite(pwmpin,0);
    }
    
   
    
    return 0;
 
}