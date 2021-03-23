 #ifndef CHASSISCONTROL_H
#define CHASSISCONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

//说明：
//1 在使用底盘控制之前，必须先调用chassisInit()
//2 引脚编码采用wiringpi编码，对应引脚功能名为wiringpi编码下的功能名
//3 小轿车电机控制方式：pwm占空比控速，电源换相反转
//4 公交车电机控制方式：pwm占空比控速控向

//创建软件控制的PWM脚:int softPwmCreate(int pin,int initialValue,int pwmRange);

/*
+-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
|     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
|   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
|   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
|   4 |   7 | GPIO. 7 | ALT4 | 1 |  7 || 8  | 1 | ALT5 | TxD     | 15  | 14  |
|     |     |      0v |      |   |  9 || 10 | 1 | ALT5 | RxD     | 16  | 15  |
|  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
|  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
|  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
|     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
|  10 |  12 |    MOSI |   IN | 0 | 19 || 20 |   |      | 0v      |     |     |
|   9 |  13 |    MISO | ALT4 | 1 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
|  11 |  14 |    SCLK |   IN | 0 | 23 || 24 | 1 | ALT4 | CE0     | 10  | 8   |
|     |     |      0v |      |   | 25 || 26 | 1 | IN   | CE1     | 11  | 7   |
|   0 |  30 |   SDA.0 | ALT4 | 1 | 27 || 28 | 1 | ALT4 | SCL.0   | 31  | 1   |
|   5 |  21 | GPIO.21 | ALT4 | 1 | 29 || 30 |   |      | 0v      |     |     |
|   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 1 | ALT4 | GPIO.26 | 26  | 12  |
|  13 |  23 | GPIO.23 | ALT4 | 1 | 33 || 34 |   |      | 0v      |     |     |
|  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
|  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
|     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
+-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
*/

#include "wiringPi.h"
#include <softPwm.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

//---------------------------------------------------------------------------------------------------------------------声明
//----------------------------------------------宏定义与变量预定义
#define ON 1
#define OFF 0

//------参数标定
#define CALIBRATION_ANGLE_SWPWM 15
#define CALIBRATION_ANGLE_HWPWM 162

//-----编码器
volatile long long encoderValA = 0L; //编码器A相计数值
bool encoderDir = true;              //编码器A相上升沿时，B相的电平高低决定旋转方向

//----------------------------------------------函数定义
//初始化底层控制
void chassisInit();
//电机控制
void setSpeed(int dir = 1, int speed = 0);
void stop();
//舵机控制
void setAngle(int dir = 1, int angle = 0);
void closeHwPwm();
//编码器反馈
void encoderValAISR();
long long getVal();
bool getDir();
void resetVal();
//
void testAllPin(int value);


//----------------------------------------------引脚定义
//车辆控制相关引脚
//电机控制
int ctrl_motor_dir = 6;     //小车电机换向：物理引脚 22，引脚功能名 GPIO.6，wiringpi编码 6口
int ctrl_motor_brk = 22;    //小车电机刹车：物理引脚 31，引脚功能名 GPIO.22，wiringpi编码 22口
int ctrl_motor_pwm = 29;    //小车电机速度pwm：物理引脚 40，引脚功能名 GPIO.29，wiringpi编码 29口
int ctrl_motor_pwm_bus = 26;//公交电机速度pwm：物理引脚 32，引脚功能名 GPIO.26，wiringpi编码 26口

//编码器
int ctrl_encoder_a = 23;    //小车编码器A：物理引脚 33，引脚功能名 GPIO.23，wiringpi编码 23口
int ctrl_encoder_b = 24;    //小车编码器B：物理引脚 35，引脚功能名 GPIO.24，wiringpi编码 24口

//舵机控制
int ctrl_servo_pwm_sw = 25; //软件pwm：物理引脚 37，引脚功能名 GPIO.25，wiringpi编码 25口
int ctrl_servo_pwm_hw = 1;  //硬件pwm：物理引脚 12，引脚功能名 GPIO.1，wiringpi编码 1口

//备用与调试相关引脚
int pwm_spare_5_0 = 28;     //备用pwm：物理引脚 38，引脚功能名 GPIO.28，wiringpi编码 28口
int debug_1 = 2;            //debug_1：物理引脚 13，引脚功能名 GPIO.2，wiringpi编码 2口
int debug_2 = 3;            //debug_2：物理引脚 15，引脚功能名 GPIO.3，wiringpi编码 3口
int debug_3 = 4;            //debug_3：物理引脚 16，引脚功能名 GPIO.4，wiringpi编码 4口
int debug_4 = 5;            //debug_4：物理引脚 18，引脚功能名 GPIO.5，wiringpi编码 5口

//---------------------------------------------------------------------------------------------------------------------实现
//初始化底层控制
void chassisInit()
{
//wiringPi初始化
    wiringPiSetup();

//车辆控制
    //电机
    pinMode(ctrl_motor_dir, OUTPUT);
    pinMode(ctrl_motor_brk, OUTPUT);
    softPwmCreate(ctrl_motor_pwm,0,200);        //软件控制pwm
    //softPwmCreate(ctrl_motor_pwm_bus,0,200);

    //编码器
    pinMode(ctrl_encoder_a, INPUT);
    pinMode(ctrl_encoder_b, INPUT);
    pullUpDnControl(ctrl_encoder_a,PUD_UP);     //输入引脚上拉
    pullUpDnControl(ctrl_encoder_b,PUD_UP);     //输入引脚上拉
    wiringPiISR(ctrl_encoder_a,INT_EDGE_RISING,&encoderValAISR);     //设编码器A相位上升沿中断,中断函数:encoderValAISR

    //舵机
    //softPwmCreate(ctrl_servo_pwm_sw,0,200);     //软件控制pwm
    pinMode(ctrl_servo_pwm_sw,INPUT);
    pinMode(ctrl_servo_pwm_hw,PWM_OUTPUT);          //硬件控制PWM
    pwmSetRange(2048);  //脉宽设置
    pwmSetClock(512);   //分频系数
    pwmSetMode(PWM_MODE_MS);//占空比输出

//备用与调试相关
    softPwmCreate(pwm_spare_5_0,0,200);
    pinMode(debug_1, OUTPUT);
    pinMode(debug_2, OUTPUT);
    pinMode(debug_3, INPUT);
    pinMode(debug_4, INPUT);

//初始状态：刹车且回正
    stop();
    setAngle(1,0);
}

void testAllPin(int value)
{

}

//------------------------------------------------------控制:速度、转向、制动
//设置小车速度
//dir==1表示前进，dir==0表示后退，速度范围小车0~200，大车0～5
void setSpeed(int dir, int speed)
{
    //小车电机控制方式
    digitalWrite(ctrl_motor_brk, LOW);  //松刹车

    if(speed>200||speed<0)
        return;

    if(dir==1||dir==0)
    {
        digitalWrite(ctrl_motor_dir, dir==1 ? HIGH : LOW);
        softPwmWrite(ctrl_motor_pwm, speed);
    }

    //公交车电机控制方式
/*
    if(speed>5||speed<0)
        return;

    if(dir==1||dir==0)
    {
        softPwmWrite(ctrl_bus_motor_pwm, 15);                           //先停转再换速度
        softPwmWrite(ctrl_bus_motor_pwm, dir==1 ? 15+speed : 15-speed);
    }*/
}

//设置小车转向
//dir==1表示左转，dir==0表示右转
void setAngle(int dir, int angle)
{
/*
    //软件pwm转向：低分辨率 0.5%
    if(dir==1 && (angle>4||angle<0))
        return;

    if(dir==0 && (angle>3||angle<0))
        return;

    if(dir==1||dir==0)
        softPwmWrite(ctrl_servo_pwm_sw, dir==1 ? CALIBRATION_ANGLE_SWPWM-angle : CALIBRATION_ANGLE_SWPWM+angle);
//*/

    //硬件pwm转向：高分辨率  0.05%
//*
    if(dir==1 && (angle>40||angle<0))
        return;

    if(dir==0 && (angle>30||angle<0))
        return;

    if(dir==1||dir==0)
        pwmWrite(ctrl_servo_pwm_hw, dir==1 ? CALIBRATION_ANGLE_HWPWM-angle : CALIBRATION_ANGLE_HWPWM+angle);
//*/
}

//关闭硬件pwm输出
void closeHwPwm()
{
    pinMode(ctrl_servo_pwm_hw,INPUT);
    return;
}

//刹车
void stop()
{
    //小车刹车
    digitalWrite(ctrl_motor_brk, HIGH);
    softPwmWrite(ctrl_motor_pwm, 0);

    //公交刹车
    //softPwmWrite(ctrl_motor_pwm_bus, 15);
}

//------------------------------------------------------反馈：编码器
//编码器中断计数
void encoderValAISR()
{
    encoderDir=digitalRead(ctrl_encoder_b);
    encoderValA++;
}

//提取编码器计数值
long long getVal()
{
    return encoderValA;
}

//提取编码器方向
bool getDir()
{
    return encoderDir;
}

//重置编码器计数值
void resetVal()
{
    encoderValA=0;
}

#ifdef __cplusplus
}
#endif
#endif
