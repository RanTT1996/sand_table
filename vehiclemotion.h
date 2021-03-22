#ifndef VEHICLEMOTION_H
#define VEHICLEMOTION_H

#include <QObject>
#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QQueue>
#include <QTimer>

#include "vehiclesensor.h"

struct PidControl
{
    PidControl(){}
    PidControl(double _p, double _i, double _d, double _limit)
    {
        p = _p;
        i = _i;
        d = _d;
        limit = _limit;
    }

    double p, i, d;                 //PID控制量
    int limit;                      //限制
    double currentError = 0;        //当前时刻的误差
    double lastError = 0;           //上一时刻的误差
    double previousError = 0;       //上上时刻的误差

};

class VehicleMotion : public QObject
{
    Q_OBJECT
//------------------------------------------------------------------------------------------初始化
public:
    explicit VehicleMotion(QObject *parent = nullptr);
    void initMotion(QMainWindow* _vehicle, Pose* _posePtr);                     //初始化底盘控制
    void initBaseVariable();                                                    //初始化基本变量

private:
    QMainWindow* vehicleConsole;                                                //车辆主控指针
    Pose* posePtr;                                                              //传感器线程位姿指针

    bool debugPID;
    int flag;
    int time_encoder_pid_ms;                                                    //输出速度和控制速度定时器的周期

public slots:
    void on_toStartMotion();
    void on_toStopMotion();

//------------------------------------------------------------------------------------------编码器
private:
    QTimer *encoderTimer;                                                       //求解车辆实时速度: 定时一定时间,通过滑移滤波计算速度值和方向
    QQueue<double> speeds_r_s;                                                  //求解车辆实时速度: 选取最近的几个速度值组成一个队列
    QQueue<bool> dirs;                                                          //求解车辆实时方向（前进/后退）: 选取最近的几个方向值组成一个队列
    double curSpeed_r_s;                                                        //求解车辆实时速度: 滑移滤波后的实时速度
    bool curDirection;                                                          //求解车辆实时方向（前进/后退）: 滑移滤波后的实时方向

public slots:
    void slipFilterAveSpeed();                                                  //求解车辆实时速度: 滑移滤波,求取速度平均值和方向

signals:
    void curSpeedDir(double speed_r_s, bool dir);                                //发送给另外的两个线程的实时速度及转向

//------------------------------------------------------------------------------------------控制
private:
    double targetSpeed_r_s;                                                     //目标速度大小
    int targetDir;                                                              //目标速度方向
    double lastControl;                                                            //上一时刻控制量
    PidControl* speedControl;                                                   //速度pid控制器
    QTimer *pidTimer;                                                           //定时启动pid

    double getIncrement(PidControl* controlV, double curV_r_s, double targetV_r_s);     //计算速度控制量增量

public slots:
    void on_run(int dir, double speed_r_s);                                     //车辆运动
    void on_turn(int dir,int angle);                                            //车辆转向
    void on_stop();                                                             //车辆刹车
    void on_test(int value);                                                             //物理引脚测试
    void on_setPID(double p, double i, double d);                               //设置PID值
    void realtimeSpeedControl();                                                //定时更新速度控制量
};

#endif // VEHICLEMOTION_H
