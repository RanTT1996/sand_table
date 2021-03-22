#ifndef VEHICLESENSOR_H
#define VEHICLESENSOR_H

#include <QObject>
#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QSerialPort>
#include <QSerialPortInfo>

#include <cmath>

#include "rplidar.h"
#include "librealsense2/rs.hpp"
#include "infomation_static.h"
#include "opencv4/opencv2/opencv.hpp"

using namespace cv;

/*class: Pose
 * 车辆位姿
 * 1 包括IMU/NFC数据的获取等
 * 2 定位融合
 *
 */
class Pose : public QObject
{
    Q_OBJECT
//------------------------------------------------------------------------------------------初始化
public:
    explicit Pose(QObject *parent = nullptr);
    void initPose(QMainWindow* _vehicle);                               //初始化位姿对象

private:
    QMainWindow* vehicleConsole;                                        //车辆主控指针
    void initBaseVariable();                                            //初始化基本变量
    bool debugPose;
    bool debugNFC;

private slots:
    void on_toStartSensor();                                            //接收车辆主线程信号，启动串口，开始接收IMU和NFC数据
    void on_toStopSensor();                                             //接收车辆主线程信号，关闭串口，停止接收IMU和NFC数据

//------------------------------------------------------------------------------------------串口
private:
    bool enableSerial(QSerialPort* _serial,
                      QString _portName, int _baudRate,
                      int _dataBits=8, int _parity=0, int _StopBits=1); //使能串口，开始接收IMU/NFC数据
    void disableSerial(QSerialPort* _serial);                           //关闭串口，停止接收IMU/NFC数据
    void imuSetting(unsigned char* commond,int argc);                   //串口配置IMU

    QSerialPort* serialIMU;                                             //IMU串口指针
    QSerialPort* serialNFC;                                             //NFC串口指针

//------------------------------------------------------------------------------------------基本数据
private:
    //电子地图
    StaticInfo staticInfo;

    //标定IMU初值
    bool isCalibration;                                                 //IMU标定标志，定时三秒解除
    double wzAverage_rad_s,wxAverage_rad_s;                             //标定平均角速度
    double rollAverage_rad,pitchAverage_rad;                            //标定平均角
    double ayAverage_g;                                                 //标定平均加速度
    int averageCount;                                                   //累计采样数

    //处理IMU数据
    QString completeIMUFrame;                                           //完整的一帧IMU数据

    long long lastTimeStamp;                                            //上一帧的时间戳
    double wzLast_rad_s;                                                //上一帧的横摆角速度
    double wxLast_rad_s;                                                //上一帧的俯仰角速度
    double ayCaliLast_g;                                                //上一帧标定的y向加速度

    double curSpeed_m_s;                                                //当前车辆纵向速度，单位m/s，imu积分的速度
    double caliAngle(double raw_angle_rad);                             //转换角度为0～360

    //车辆位姿
    double poseX_m,poseY_m,poseYaw_rad,poseRoll_rad;                    //当前车辆位置、偏航角、俯仰角，单位m，rad

    //校准值输入
    double curSpeedRaw_r_s;                                            //编码器校准：速度，单位r/s，
    bool curDirection;                                                  //编码器校准：当前车辆运动方向（前进/后退）

    double poseXnfc_m,poseYnfc_m,IDnfc;                                 //NFC校准：当前车辆最近经过的NFC位置（位置校准值），单位m

    double poseYawCorrect_rad;                                          //相机校准：当前车辆最近的偏航角校准值，单位rad

    //定时器
    QTimer* timerPose;                                                  //定时给主线程发送当前计算的车辆位置

signals:
    void curPose(double x_m, double y_m, double yaw_rad);               //给主线程发送当前计算的车辆位置

private slots:
    void sendCurPose2VehicleConsole();                                  //定时给主线程发送当前计算的车辆位置
    void on_readyread_IMU();                                            //处理接收的IMU数据
    void on_readyread_NFC();                                            //处理接收的NFC数据
    void on_curSpeedDir(double _curSpeed_r_s, bool _curDirection);      //接收motion线程传来的实时速度及其转向
    void on_calibration();                                              //imu标定，单次触发

};

/*class: Lidar
 * 激光雷达
 * 1 包括Lidar连接创建和断开、扫描的起止等
 *
 */
class Lidar : public QObject
{
    Q_OBJECT
public:
    explicit Lidar(QObject *parent = nullptr);
    void initLidar(QMainWindow* _vehicle);      //初始化雷达对象
    void connect2Lidar();                       //创建雷达连接
    void disconnect2Lidar();                    //断开雷达连接
    void startScan();                           //开始扫描输出
    void stopScan();                            //停止扫描输出

private:
    QMainWindow* vehicleConsole;                //车辆主控指针
    rp::standalone::rplidar::RPlidarDriver *u;  //雷达驱动
    QTimer* timerLidar;                              //定时检测障碍物

signals:
    void obstacleInfo(bool isStop, double obstacleDir, double obstacleDis_mm, QString info);

private slots:
    void on_toStartSensor();                    //接收车辆主线程信号，启动串口，开始接收激光数据
    void on_toStopSensor();                     //接收车辆主线程信号，关闭串口，停止接收激光数据

    void obstacleDetection();                   //定时检测障碍物
};

/*class: Camera
 * 深度相机
 * 1 包括realsense D435 的基本使用
 *
 */
class Camera : public QObject
{
    Q_OBJECT
public:
    explicit Camera(QObject *parent = nullptr);
    void initCamera(QMainWindow* _vehicle);     //初始化相机对象

    void realsenseTest();                       //D435基本用法
    Mat getImage();                             //获取原始图像
    void trans(Mat oriImage);                   //车道线识别函数
    void calibrate();                           //车道线识别预处理：校正

private:
    QMainWindow* vehicleConsole;                //车辆主控指针
    Mat cali_matrix;                            //透视矫正矩阵
    Mat image_read;                             //实时原始图像

private slots:
    void on_toStartSensor();                    //接收车辆主线程信号
    void on_toStopSensor();                     //接收车辆主线程信号
};

#endif // VEHICLESENSOR_H
