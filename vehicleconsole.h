#ifndef VEHICLECONSOLE_H
#define VEHICLECONSOLE_H

#include <QMainWindow>
#include <QThread>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QTimer>

#include "vehiclesensor.h"
#include "vehiclemotion.h"
#include "infomation_static.h"
#include "form.h"

#define STRAIGHT 1
#define LEFT 2
#define RIGHT 3

QT_BEGIN_NAMESPACE
namespace Ui { class VehicleConsole; }
QT_END_NAMESPACE


//---------------------------------------------------------------------车辆控制器
class VehicleConsole : public QMainWindow
{
    Q_OBJECT
//车载控制台初始化--------------------------------------------------------------------------------------------------------------
public:
    VehicleConsole(QWidget *parent = nullptr);
    ~VehicleConsole();
    void initBasicVariable();               //初始化基本变量
private:
    //调试信息输出
    bool debugRouteFollow;                  //循迹
    bool debugInterpolation;                //路径插值
    bool debugPathPlan;                     //路径规划
    bool debugTL;                           //信号灯相关
    bool debugLidar;                        //激光避障

    //车辆基本信息
    int carID;                              //车辆ID
    double L_mm;                            //车辆轴距，单位 mm

    double curPoseX_m, curPoseY_m, curPoseYaw_rad;  //车辆当前车辆位姿，单位m，rad
    int leavingNode;                        //车辆最近经过的节点
    int arrivingNode;                       //车辆即将抵达的节点
    int locatedEdge;                        //车辆当前所在边

    double curSpeed_r_s;                    //车辆当前纵向速度,单位 r/s
    bool curDirection;                      //车辆当前车辆方向（前进/后退）
    double setSpeed_m_s;                    //车辆控制目标速度：由速度控制指令修改，循迹时依据此速度循迹，单位 m/s
    bool setSpeedDir;                       //车辆控制目标速度方向：这个暂时禁止用户修改

    QString curOrder;                       //车辆当前执行的指令

//单车功能模块--------------------------------------------------------------------------------------------------------------
private:
    double disClose2Node_mm;                //阈值：车辆位置与节点位置距离，单位 mm
    void initFuncSingleVehicle();           //初始化单车功能

//规划跟踪
private:
    bool enableFollow;                      //使能循迹

    //规划-----------------------------------
    QList<int> path;                        //节点路线：既定或规划得到
    QList<QList<double>> densePath;         //稠密坐标路线：对既定或规划路线插值得到

    void pathPreprocess(const int source, const int target);     //规划主函数
    QList<int> pathPlanning(const int source, const int target);    //路径规划
    void interpolation(const QList<int>& _route, QList<QList<double>>& _denseRoute,bool isAP = false);//路径插值函数
    void calcuCircleCenter(double x1, double y1, double x2, double y2,
                           double r, int dir, double& x0, double& y0); //插值：计算圆弧的圆心坐标


    //跟踪-----------------------------------
    int firstPoint;                         //第一个待跟踪点
    int finishPoint;                        //最后一个待跟踪点
    int nextPoint;                          //下一个待跟踪点
    int lastRouteType;                      //循迹时，减速过弯的路段类型标志
    int disFinish_mm;                       //循迹结束时，车辆于目标点的偏差范围

    QTimer timerPosCheck;                   //定时器：定时检测位置，并发出控制指令
    int timePC_ms;                          //定时器：定时时间
    void realtimeControl();                 //定时器：实时控制
    int getGrade(double theta_rad);         //根据计算的前轮转角，计算控制量

private slots:
    void on_PCtimeout();                    //定时检测位置，并发出指令 即函数realtimeControl()

//车路协同
private:
    bool enableTL;                          //功能使能
    QList<int> crossingTLInfo;              //前方信号灯信息，列表一般有3个值，分别是信号灯编号、所属控制器编号、控制器组内编号

    QTimer timerTL;                         //定时器：定时更新前方红绿灯编号
    int timeTL_ms;                          //定时器：定时时间

private slots:
    void on_TLtimeout();                    //定时更新前方红绿灯编号

//自动泊车
private:
    bool enableAP;                          //使能功能
    QList<int> parkingPath;                 //节点路线：既定
    bool isParkIn;                          //入库还是出库
    int apFlag;                             //AP的临时目标
    int carportID;                          //最终停车的车位编号
    int outTarget;                          //从车库驶出的目的地

    QTimer timerAP;                         //定时器：定时检查车辆是否到达目标
    int timeAP_ms;                          //定时时间

    void autoParking(int parkingID,bool isParkIn_,int outID = 0);//泊车主函数，泊入目标车位，或从目标车位驶离
    void parkPathPreprocess(int parkingID);       //泊车路径预处理

private slots:
    void on_APtimeout();                    //定时检查车辆是否到达目标

//激光避障
private:
    bool enableLidarAvoid;

//多车协同模块--------------------------------------------------------------------------------------------------------------
private:
    bool belong2Formation;                  //车辆是否属于编队
    int formationIndex;                     //所属编队的车辆编号

//车云通信模块--------------------------------------------------------------------------------------------------------------
private:
    QTcpSocket  *tcpClient;                 //通信socket
    QString masterIP;
    quint16 masterPort;

    QTimer timerState2Master;               //定时器：定时给主控发送车辆自身状态
    int timeS2M_ms;                         //定时时间

    void initTCPClient();                   //初始化与主控的通信
    void connect2master();                  //连接到主控
    void disconnect2master();               //断开与主控连接
    void send2master(QString _msg);         //给总控发送消息

private slots:
    void on_Connected();                    //成功建立连接后触发的槽
    void on_Disconnected();                 //成功断开连接后触发的槽
    void on_SocketStateChange(QAbstractSocket::SocketState socketState);//socket连接状态
    void on_SocketReadyRead();              //读取主控传入的数据
    void on_state2MasterTimeout();          //定时给主控发送车辆自身状态

//车路通信模块--------------------------------------------------------------------------------------------------------------
private:
    QUdpSocket *udpSocket;                  //socket:接收信号灯的信号
    quint16 tlPort;                         //绑定的信号灯广播端口

    void connect2Road();                    //连接到路
    void disconnect2Road();                 //断开与路的连接

private slots:
    void on_udpSocketReadyRead();           //读取信号灯传入的数据

//车载传感器模块--------------------------------------------------------------------------------------------------------------
private:
    QThread sensor_pose;                    //车载传感器,线程管理器
    QThread sensor_lidar;
    QThread sensor_camera;
    Pose pose;                              //接收IMU/NFC数据,并预处理
    Lidar lidar;                            //接收激光雷达数据,并预处理
    Camera camera;                          //接收深度相机数据,并预处理

    void connect2Sensor();                  //初始化传感器

signals:
    void toStartSensor();                   //开始接收传感器数据
    void toStopSensor();                    //停止接收传感器数据

private slots:
    void on_curPose(double poseX_m,double poseY_m,double poseYaw_rad);//接收sensor线程传来的实时位姿
    void on_obstacleInfo(bool isStop, double obstacleDir, double obstacleDis_mm, QString info);//接收sensor线程传来的障碍物信息

//车辆底盘控制模块--------------------------------------------------------------------------------------------------------------
private:
    QThread motion;                         //底盘控制，线程管理器
    VehicleMotion motioner;                 //接收底盘控制信号，控制底盘

public:
    void connect2Motion();                  //初始化底盘控制

signals:
    void toStartMotion();                   //启动运动反馈
    void toStopMotion();                    //停止运动反馈
    void run(int dir,double speed_r_s);     //车辆运动，速度单位 r/s
    void turn(int dir,int angle);           //车辆转向
    void stop();                            //车辆刹车
    void setPID(double p,double i,double d);

private slots:
    void on_curSpeedDir(double _curSpeed_r_s, bool _curDirection);//接收motion线程传来的实时速度及其转向

//电子地图--------------------------------------------------------------------------------------------------------------
private:
    StaticInfo staticInfo;

//TEST-----------------------------------------------------------------------------------------------------------------
private:
    Form *displayPath;
    QTimer* testTimer;
signals:
    void showP(QList<QList<double>> * p);
    void appendP(double x, double y);

private slots:
    void on_btnTestWheelRun_clicked();
    void on_btnTestWheelTurn_clicked();
    void on_test_clicked();
    void on_park_clicked();
    void on_test2_clicked();
    void on_stop_clicked();
    void on_test3_clicked();
    void on_testTimerTimeout();

    void on_test4_clicked();

signals:
    void test(int);

private:
    Ui::VehicleConsole *ui;
};


#endif // VEHICLECONSOLE_H
