extern "C" {
#include "chassiscontrol.h"
}

#include "vehiclemotion.h"

VehicleMotion::VehicleMotion(QObject *parent) : QObject(parent)
{
    initBaseVariable();
}

//------------------------------------------------------------------------------------------初始化
/* Function: VehicleMotion::initMotion
 * 初始化底盘控制
 */
void VehicleMotion::initMotion(QMainWindow* _vehicle, Pose* _posePtr)
{
    vehicleConsole = _vehicle;
    posePtr =  _posePtr;

    //使能
    connect(vehicleConsole,SIGNAL(toStartMotion()),this,SLOT(on_toStartMotion()),Qt::QueuedConnection);
    connect(vehicleConsole,SIGNAL(toStopMotion()),this,SLOT(on_toStopMotion()),Qt::BlockingQueuedConnection);

    //速度反馈
    connect(this,SIGNAL(curSpeedDir(double,bool)),posePtr,SLOT(on_curSpeedDir(double,bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(curSpeedDir(double,bool)),vehicleConsole,SLOT(on_curSpeedDir(double,bool)),Qt::QueuedConnection);

    //车辆控制
    connect(vehicleConsole,SIGNAL(run(int,double)),this,SLOT(on_run(int,double)),Qt::QueuedConnection);
    connect(vehicleConsole,SIGNAL(turn(int,double)),this,SLOT(on_turn(int,double)),Qt::QueuedConnection);
    connect(vehicleConsole,SIGNAL(stop()),this,SLOT(on_stop()),Qt::QueuedConnection);
    connect(vehicleConsole,SIGNAL(test(int)),this,SLOT(on_test(int)),Qt::QueuedConnection);
    connect(vehicleConsole,SIGNAL(setPID(double,double,double)),this,SLOT(on_setPID(double,double,double)),Qt::QueuedConnection);


    //底层初始化
    chassisInit();
}

/* Function: VehicleMotion::initBaseVariable
 *
 */
void VehicleMotion::initBaseVariable()
{
    time_encoder_pid_ms = 10;
    flag = 0;

    //PID
    lastControl = 0;
    debugPID = false;
    targetSpeed_r_s = 0;
    targetDir = 1;
}

/* Function: VehicleMotion::on_toStartMotion
 *
 */
void VehicleMotion::on_toStartMotion()
{
    qDebug()<<"I: connect & start motion...";

    //编码器
        speeds_r_s.clear();
        dirs.clear();
        //定时器：速度采集与预处理
        encoderTimer = new QTimer(this);
        connect(encoderTimer,SIGNAL(timeout()),this,SLOT(slipFilterAveSpeed()));
        encoderTimer->start(time_encoder_pid_ms);
        resetVal();

    //PID
        speedControl = new PidControl(0,0,0,190);
        //定时器：速度控制
        pidTimer = new QTimer(this);
        connect(pidTimer,SIGNAL(timeout()),this,SLOT(realtimeSpeedControl()));
        pidTimer->start(time_encoder_pid_ms);
}

/* Function: VehicleMotion::on_toStopMotion
 *
 */
void VehicleMotion::on_toStopMotion()
{
    qDebug()<<"I: stop & disconnect motion...";

    //PID
        pidTimer->stop();

    //刹车，回正，停止硬件pwm输出
        stop();
        setAngle(1,0);
        closeHwPwm();

    //编码器
        encoderTimer->stop();


    delete pidTimer;
    delete encoderTimer;
}

//------------------------------------------------------------------------------------------控制
/* Function: VehicleMotion::on_run
 * 车辆运动
 */
void VehicleMotion::on_run(int dir,double speed_r_s)
{
    if((dir == 0) || (dir == 1))
        targetDir = dir;

    if(speed_r_s > -1)
        targetSpeed_r_s = speed_r_s;
}

/* Function: VehicleMotion::on_turn
 * 车辆转向
 */
void VehicleMotion::on_turn(int dir, double angle_rad)
{
    int angleGrade = 0;
    angle_rad = fabs(angle_rad);

/*软件pwm
    //左转
    if(angle_rad>0)
    {
        if(angle_rad>(15.0/180.0*3.14159))
            angleGrade = 4;
        else if(angle_rad>(11.0/180.0*3.14159))//15.0
            angleGrade = 3;
        else if(angle_rad>(7.0/180.0*3.14159))//9.0
            angleGrade = 2;
        else if(angle_rad>(2.0/180.0*3.14159))//3.0
            angleGrade = 1;
        else
            angleGrade = 0;
    }
    else//右转
    {
        angle_rad = fabs(angle_rad);
        if(angle_rad>(14.0/180.0*3.14159))
            angleGrade = 3;
        else if(angle_rad>(7.0/180.0*3.14159))
            angleGrade = 2;
        else if(angle_rad>(2.0/180.0*3.14159))
            angleGrade = 1;
        else
            angleGrade = 0;
    }
//*/

//*硬件pwm
    //左转：转向极限是grade==40，angle==18
    if(1 == dir)
    {
        angleGrade = round(40*(angle_rad/(18/180.0*3.14159)));
        angleGrade = angleGrade>40 ? 40 : angleGrade;
    }
    else if(0 == dir)//右转：转向极限是grade==30，angle==22
    {
        angleGrade = round(30*(angle_rad/(22/180.0*3.14159)));
        angleGrade = angleGrade>30 ? 30 : angleGrade;
    }

//*/

    setAngle(dir,angleGrade);
}

/* Function: VehicleMotion::on_stop
 * 车辆刹车
 */
void VehicleMotion::on_stop()
{
    targetSpeed_r_s = 0;
}

/* Function: VehicleMotion::on_test
 * 物理引脚测试
 */
void VehicleMotion::on_test(int value)
{
    testAllPin(value);
}

/* Function: VehicleMotion::on_setPID
 *
 */
void VehicleMotion::on_setPID(double p, double i, double d)
{
    speedControl->p = p;
    speedControl->i = i;
    speedControl->d = d;
}

/* Function: VehicleMotion::realtimeSpeedControl
 * 定时更新速度控制量
 */
void VehicleMotion::realtimeSpeedControl()
{
    //使用增量式PID，计算控制量增量
    double inc = getIncrement(speedControl,curSpeed_r_s,targetSpeed_r_s);

    //计算当前需要输出的控制量
    double curControl = lastControl + inc;

    if(curControl > speedControl->limit)
        curControl = speedControl->limit;
    if(curControl < 0)
        curControl = 0;

    //控制
    setSpeed(targetDir,round(curControl));//round(curControl)

    if(curSpeed_r_s > 0.001)
        qDebug()<<flag<<"\t"<<curSpeed_r_s<<"\t"<<targetSpeed_r_s<<"\t"<<inc<<"\t"<<lastControl<<"\t"<<curControl<<round(curControl);

    //更新上一时刻控制量
    lastControl = curControl;
    flag++;

    if(debugPID)
    {
        qDebug()<<"D: PID2: curSpeed_"<<curSpeed_r_s<<",targetSpeed_"<<targetSpeed_r_s;
        qDebug()<<"D: PID3: inc_"<<inc<<",curControl_"<<curControl;
    }
}

/* Function: VehicleMotion::getIncrement
 * 计算速度控制量增量
 */
double VehicleMotion::getIncrement(PidControl *controlV, double curV_r_s, double targetV_r_s)
{
    //计算当前误差
    double curError = targetV_r_s - curV_r_s;
    //计算PID增量
    double inc = (controlV->p)*(curError - (controlV->lastError))
               + (controlV->i)*(curError)
               + (controlV->d)*(curError - 2*(controlV->lastError) + (controlV->previousError));

    if(debugPID)
    {
        qDebug()<<"D: PID0: curError_"<<curError<<",lastError_"<<controlV->lastError<<",previousError_"<<controlV->previousError;
        qDebug()<<"D: PID1: p_"<<controlV->p<<",i_"<<controlV->i<<",d_"<<controlV->d;
    }

    //更新误差值
    controlV->previousError = controlV->lastError;
    controlV->lastError = curError;

    return inc;
}

//------------------------------------------------------------------------------------------编码器
/* Function: VehicleMotion::slipFilterAveSpeed
 * 求解车辆实时速度: 滑移滤波,求取速度平均值和方向
 */
void VehicleMotion::slipFilterAveSpeed()
{
    double curV = 0, curDir = 0;

    //采集数据:采集了50ms，则每秒的脉冲数要*20，电机转一圈360脉冲，减速比34
    long long val = getVal();
    resetVal();
    bool dir = getDir();

    speeds_r_s.enqueue(val*(1000.0/time_encoder_pid_ms)/360.0/34.0);

    dirs.enqueue(dir);

    //队列:存储最近x个速度值和最近y个方向, 注：50%占空比的速度（6V以下），方向的9个最近观测值是足够的，随着速度的增加，还要继续增加滤波的观测值
    if(speeds_r_s.size()>1)
        speeds_r_s.dequeue();
    if(dirs.size()>20)
        dirs.dequeue();

    //求均值
        //速度
        for(int i=0;i<speeds_r_s.size();i++)
        {
            if(!speeds_r_s.isEmpty())
                curV+=speeds_r_s[i];
        }
        curV=curV/speeds_r_s.size();

        //方向
        int flag=0;
        for(int i=0;i<dirs.size();i++)
        {
            if(!dirs.isEmpty())
                flag+=dirs[i];
        }

        curDir=(flag>((dirs.size()-1)/2));

    //submit
    emit curSpeedDir(curV*2,curDir);

    curSpeed_r_s = curV*2;
    curDirection = curDir;
}
