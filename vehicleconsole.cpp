#include "vehicleconsole.h"
#include "ui_vehicleconsole.h"

//-------------------------------------------------------------------------------------------车载控制台初始化
VehicleConsole::VehicleConsole(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::VehicleConsole)
{
    ui->setupUi(this);
    qDebug()<<"I: "<<QThread::currentThread();

    initBasicVariable();

//底盘控制模块
    connect2Motion();

//传感器模块
    connect2Sensor();

//车路通信模块
    connect2Road();

//车云通信模块
    connect2master();

//单车功能模块
    initFuncSingleVehicle();
}

VehicleConsole::~VehicleConsole()
{
    emit toStopSensor();
    if(sensor_camera.isRunning())
    {
        sensor_camera.quit();
        sensor_camera.wait();
    }

    if(sensor_lidar.isRunning())
    {
        sensor_lidar.quit();
        sensor_lidar.wait();
    }

    if(sensor_pose.isRunning())
    {
        sensor_pose.quit();
        sensor_pose.wait();
    }

    emit toStopMotion();
    if(motion.isRunning())
    {  
        motion.quit();
        motion.wait();
    }

    qDebug()<<"I: stop & disconnect done.";

    disconnect2master();
    disconnect2Road();

    delete ui;
}

/* Function: VehicleConsole::initBasicVariable
 * 初始化基本变量
 */
void VehicleConsole::initBasicVariable()
{
//调试
    debugPathPlan = false;
    debugInterpolation = false;
    debugRouteFollow = false;
    debugTL = false;
    debugLidar = false;

    testTimer = new QTimer(this);
    connect(testTimer,SIGNAL(timeout()),this,SLOT(on_testTimerTimeout()));

//通信
    masterIP = "192.168.11.100";
    masterPort = 4000;
    tlPort = 6000;

//车辆基本信息
    carID = 1;                  //车辆编号
    L_mm = 123;                 //车辆轴距,mm

    curPoseX_m = 0;             //车辆当前位置X
    curPoseY_m = 0;             //车辆当前位置Y
    curPoseYaw_rad = 0;         //车辆当前偏航角
    curSpeed_r_s = 0;           //车辆当前速度
    curDirection = 1;           //车辆当前速度方向
    leavingNode = 0;            //车辆最近经过的节点
    arrivingNode = 0;           //车辆即将抵达的节点
    locatedEdge = 0;            //车辆当前所在边

    curOrder = "000";           //车辆当前执行的指令

    timeS2M_ms = 50;

//车辆基本功能
    disClose2Node_mm = 300;     //阈值：车辆位置与节点位置距离，单位 mm
    disFinish_mm = 50;          //循迹结束时，车辆于目标点的偏差范围，单位 mm
    setSpeed_m_s = 0;           //车辆控制目标速度：由速度控制指令改变，循迹时依据此速度循迹，单位m/s
    setSpeedDir = 1;            //车辆控制目标速度方向：这个暂时禁止用户修改


    //路径规划
    path.clear();               //节点路线：既定或规划得到
    densePath.clear();          //稠密坐标路线：对既定或规划路线插值得到
    timePC_ms = 50;             //定时时间：定时检测位置，并发出控制指令
    lastRouteType = 1;          //循迹时，减速过弯的路段类型标志

    //车路协同
    crossingTLInfo.clear();     //前方信号灯信息
    crossingTLInfo<<0<<0;
    timeTL_ms = 100;            //定时时间：定时更新前方红绿灯编号

    //自动泊车
    isParkIn = true;            //入库还是出库
    apFlag = 0;                 //AP的临时目标，包括行驶到车库入口的目标96、车门打开目标、到达指定车位的目标等
    carportID = 0;              //最终停车的车位编号
    outTarget = 0;              //从车库驶出的目的地
    parkingPath.clear();        //节点路线：既定
    timeAP_ms = 100;            //定时时间：定时检查车辆是否到达泊车各阶段目标

//编队
    belong2Formation = false;   //车辆是否属于编队
    formationIndex = 0;         //所属编队的车辆编号
}

//-------------------------------------------------------------------------------------------单车功能模块
/* Function: VehicleConsole::initFuncSingleVehicle
 * 初始化单车功能
 */
void VehicleConsole::initFuncSingleVehicle()
{
    enableTL = false;           //使能单车车路协同
    enableFollow = false;       //使能循迹
    enableAP = false;           //使能泊车
    enableLidarAvoid = false;   //使能激光避障

    connect(&timerPosCheck,SIGNAL(timeout()),this,SLOT(on_PCtimeout()));
    connect(&timerAP,SIGNAL(timeout()),this,SLOT(on_APtimeout()));
}

//规划跟踪
/* Function: VehicleConsole::pathPreprocessing
 * 路径规划与跟踪：数据预处理，对规划的路线进行插值处理得到一条稠密的路径
 */
void VehicleConsole::pathPreprocess(const int source, const int target)
{
    qDebug()<<"I: Path peprocessing: \tRun, start...";

//如果车辆处于行驶中，则首先刹车，更换路线
    timerPosCheck.stop();
    emit stop();

//路径规划
    qDebug()<<"I: \t\tPath planning, start...";

    path.clear();
    path.append(pathPlanning(source,target));
    qDebug()<<"I: final path:"<<path;

    qDebug()<<"I: \t\tPath planning, done.\n\t\tInterpolation, start...";

    QList<QList<double>> dp;
    dp.clear();
    if(!densePath.isEmpty())//这里首先将当前边走完，再续上下一段
    {
        QList<int> p;
        p.clear();
        p<<leavingNode<<arrivingNode;
        if(leavingNode!=0 && arrivingNode!=0)
            interpolation(p,dp);//插值，得到稠密路径
    }
    interpolation(path,dp);//插值，得到稠密路径
    densePath.clear();
    densePath = dp;

    qDebug()<<"I: \t\tInterpolation, done.";

//当前位置与最近点
    qDebug()<<"I: \t\tGet the closest point, start...";

    double curX = curPoseX_m*1000.0;
    double curY = curPoseY_m*1000.0;

    int minID = 0;
    double minDis = pow(curX-densePath[0].at(0),2) + pow(curY-densePath[0].at(1),2);

    if(debugInterpolation)
        qDebug()<<"D: [debugInterpolation] Interpolation result: "<<0<<densePath[0].at(0)<<densePath[0].at(1);

    //循环找到最近点
    for(int k = 1; k < densePath.count(); k++)
    {
        double d = pow(curX-densePath[k].at(0),2) + pow(curY-densePath[k].at(1),2);
        if(d < minDis)
        {
            minDis = d;
            minID = k;
        }

        if(debugInterpolation)
            qDebug()<<"D: [debugInterpolation] Interpolation result: "<<k<<densePath[k].at(0)<<densePath[k].at(1);
    }

    firstPoint = minID;
    nextPoint = ((minID + 2) < (densePath.count()-1)) ? (minID + 2) : (densePath.count()-1);
    finishPoint = densePath.count()-1;
    qDebug()<<"I: \t\tCurrent position:"<<curX<<curY;
    qDebug()<<"I: \t\t"<<"Get the first point to follow:\tID:"<<QString::number(firstPoint)
            <<"\tx:"<<QString::number(densePath[firstPoint].at(0))
            <<"\ty:"<<QString::number(densePath[firstPoint].at(1));
    qDebug()<<"I: \t\t"<<"Get the next point to follow:\tID:"<<QString::number(nextPoint)
            <<"\tx:"<<QString::number(densePath[nextPoint].at(0))
            <<"\ty:"<<QString::number(densePath[nextPoint].at(1));
    qDebug()<<"I: \t\t"<<"Get the finish point to follow:\tID:"<<QString::number(finishPoint)
            <<"\tx:"<<QString::number(densePath[finishPoint].at(0))
            <<"\ty:"<<QString::number(densePath[finishPoint].at(1));

//启动定时器，实时修改指令
    if(enableFollow)
    {
        timerPosCheck.start(timePC_ms);
        qDebug()<<"D:[debugRouteFollow]"
                  "\t\tPose x y yaw speed"
                  "\tGoal x y next/node"
                  "\t\t\tControl:turn theta (grade)"
                  "\t\tControl:speed"
                  "\ttype routeType";
    }
}

/* Function: VehicleConsole::pathPlanning
 * 路径规划与跟踪：路径规划
 */
QList<int> VehicleConsole::pathPlanning(const int source, const int target)
{
//定义可能的路径及其代价值
    QList<QList<int>> allPath;          //定义所有的可能路径
    allPath.clear();
    QList<int> cost;                    //对应每条路径的代价值
    cost.clear();

    QList<int> firstPath;               //添加一条新的路径
    firstPath.clear();
    firstPath.append(source);           //添加起点
    allPath.append(firstPath);
    cost.append(0);                     //添加新路径的代价值

//定义一些标志
    bool finishPlan = false;            //规划完成标志
    int finalCost = 60000;              //预定义最终代价值
    int hasReachTarget = false;         //定义是否已经有路线找到了终点

    QList<int> endingNode;              //断路预处理
    endingNode = {0,16,21};

//遍历规划：
//  不断遍历所有的可能路线，直到到达目标位置
//  当某一条可能的路径到达目标位置后，开始与其他可能的路径比较长度，并删除超过(到达终点的路径中最短路径长度)的路径
    while(!finishPlan)
    {
        if(debugPathPlan)
            qDebug()<<"D: [debugPathPlan]****************************allPath.size():"<<allPath.size();

        int outNum = 0; //定义每次遍历时淘汰的路径数
        for(int m = 0; m < allPath.size(); m++)
        {
            int i = m-outNum;
        //1、路径淘汰
            QList<int> oriPath;
            oriPath.clear();
            QList<int> T = allPath.at(i);
            for(int k = 0; k < T.size(); k++)
                oriPath.append(T.at(k));

            if(debugPathPlan)
                qDebug()<<"D: [debugPathPlan]oriPath:"<<oriPath;

            int fromNodeIndex = oriPath.last();
            if(target == fromNodeIndex)                 //1.1、对于达到目标点的路径：比较产生最小的代价值，淘汰较大代价值的路径
            {
                hasReachTarget = true;

                if(1 == allPath.size())                 //1.1.1、当到达目标点，同时经过淘汰环节，只剩一条可能的路径时，完成规划
                {
                    if(debugPathPlan)
                        qDebug()<<"D: [debugPathPlan]finishPlan";

                    finishPlan = true;
                    break;
                }

                if(cost.at(i)<(finalCost+1))                //1.1.2、到达目标点，但还有未淘汰的路径时，进行最小代价值更新或淘汰操作
                {
                    finalCost = cost.at(i);
                }
                else
                {//同时移除路径及其代价
                    allPath.removeAt(i);
                    cost.removeAt(i);
                    outNum++;
                    if(debugPathPlan)
                        qDebug()<<"D: [debugPathPlan] remove route type 1";
                }
                continue;
            }
            else                                        //1.2、对于未达到目标点的路径：淘汰较大代价值的路径
            {
                if(hasReachTarget)
                    if(cost.at(i)>finalCost)
                    {//同时移除路径及其代价
                        allPath.removeAt(i);
                        cost.removeAt(i);
                        outNum++;
                        if(debugPathPlan)
                            qDebug()<<"D: [debugPathPlan] remove route type 2";
                        continue;
                    }
            }

        //2、路径延伸
            Node_ fromNode = staticInfo.getMapNodeInfoS()->value(fromNodeIndex);    //2.1、取出当前路径最后一个点
            QList<int> nextNodes = fromNode.relatedNodes;
            QList<int> nextEdges = fromNode.relatedEdges;
            int curCost = cost.at(i);

            if(endingNode.contains(nextNodes.at(0)))                                //2.2、淘汰断路
            {//同时移除路径及其代价
                allPath.removeAt(i);
                cost.removeAt(i);
                outNum++;
                if(debugPathPlan)
                    qDebug()<<"D: [debugPathPlan] remove route type 3";
                continue;
            }

            for(int j = 0; j < nextNodes.size(); j++)                               //2.3、确定分叉情况
            {
                //更新路径
                QList<int> newOne;
                newOne.clear();
                newOne.append(oriPath);
                newOne.append(nextNodes.at(j));
                //更新代价
                int incCost = staticInfo.getMapEdgeInfoS()->value(nextEdges.at(j)).attr_cost;

                if(0 == j)                                                          //2.3.1、对于第一个子节点，顺延当前路径
                {
                    allPath.replace(i,newOne);
                    cost.replace(i,curCost+incCost);
                }
                else                                                                //2.3.2、对于第2/3个子节点，则新建路径
                {
                    allPath.append(newOne);
                    cost.append(curCost+incCost);
                }
            }

        }
    }

    QList<int> p;
    p.clear();
    p = allPath.at(0);

    return p;
}

/* Function: VehicleConsole::interpolation
 * 路径规划与跟踪：路径插值函数
 */
void VehicleConsole::interpolation(const QList<int> &_route, QList<QList<double>> &_denseRoute, bool isAP)
{
    int resolution = 50;//插值的分辨率，单位：mm，下述距离单位均为 mm
    QList<int> repairEdge;//修复一些边
    repairEdge = {12,18,39,50,52,53,55,104,109,125,136,149,163,167,168};

    for(int i = 0; i < _route.count()-1; i++)
    {
        //确定该节点坐标
        Node_ rNode= isAP ? staticInfo.getParkNodeInfoS()->value(_route[i])
                         : staticInfo.getMapNodeInfoS()->value(_route[i]);
        double x1 = rNode.nodePos.x;
        double y1 = rNode.nodePos.y;

        //确定相邻下一节点
        int nextNode = _route[i+1];
        Node_ nNode = isAP ? staticInfo.getParkNodeInfoS()->value(nextNode)
                          : staticInfo.getMapNodeInfoS()->value(nextNode);
        double x2 = nNode.nodePos.x;
        double y2 = nNode.nodePos.y;

        //确定两节点之间的路线属性
        int index = rNode.relatedNodes.indexOf(nextNode);
        int edgeIndex = rNode.relatedEdges.at(index);
        Edge rEdge = isAP ? staticInfo.getParkEdgeInfoS()->value(edgeIndex)
                          : staticInfo.getMapEdgeInfoS()->value(edgeIndex);
        double dis = rEdge.attr_dis;
        double angle = rEdge.attr_angle;
        int edgeType = rEdge.attr_type;

        if(rEdge.attr_isReverse && (edgeType!=1)) //表示此时为泊车路径插值，要考虑是否是倒车：若是倒车，在插值算法下，需要将左右互换
            edgeType = (2 == edgeType) ? 3 : 2;

        //添加该路段首节点
        QList<double> add0 = {x1,y1,double(_route[i]),double(_route[i+1]),double(edgeIndex)};
        _denseRoute.append(add0);

        QList<double> add1;
        add1.clear();

        //开始插值
        if(edgeType == STRAIGHT)   //直行
        {          
            int cnt = int(dis)/resolution;
            for(int j = 0; j < cnt; j++)
            {
                double xx = x1 + (x2 - x1)*(j+1)/cnt;
                double yy = y1 + (y2 - y1)*(j+1)/cnt;

                add1 = {xx,yy,double(_route[i]),double(_route[i+1]),double(edgeIndex)};
                _denseRoute.append(add1);
                add1.clear();

                if(debugInterpolation)
                    qDebug()<<"D: [debugInterpolation] "<<rNode.nodePos.id<<"\t\t\t\t\t\t"<<xx<<"\t"<<yy;
            }
        }
        else if(edgeType == LEFT)  //左转
        {
            //计算v圆心坐标
            double x0=0,y0=0;
            calcuCircleCenter(x1,y1,x2,y2,dis,edgeType,x0,y0);

            if(debugInterpolation)
                qDebug()<<"D: [debugInterpolation] \t\tcalcuCircleCenter:"<<x0<<y0;

            //分段
            double radAngle = angle * 3.14159 / 180;        //角度用弧度表示
            double curveDis = dis * radAngle;               //弧长
            int cnt = int(curveDis)/resolution;             //按弧长分段
            double aveAngle = radAngle/cnt;                 //按角度分段

            if(i == 0 && debugInterpolation)
            {
                qDebug()<<"D: [debugInterpolation] \t"<<curveDis<<"\t"<<cnt<<"\t"<<aveAngle;
                //qDebug()<<"D: [debugInterpolation] \t"<<atan((y2-y1)/(x2-x1))/3.14159*180;
            }

            //插值
            for(int j = 0; j < cnt-1; j++)
            {

                double angle = (j+1)*aveAngle;               //圆心角
                double isoscelesAngle = (3.14159 - angle)/2; //等腰底角
                double chord = 2 * dis * sin(angle/2);       //弦长

                double a = atan((y0-y1)/(x0-x1));  
                double b = 3.14159 - (isoscelesAngle +a);

                int flag = (y1>y2) ? 1 : -1;
                if(repairEdge.contains(edgeIndex))          //修复一些边
                    flag = flag *(-1);

                double xx = x1 + flag*chord*cos(b);
                double yy = y1 - flag*chord*sin(b);

                add1 = {xx,yy,double(_route[i]),double(_route[i+1]),double(edgeIndex)};
                _denseRoute.append(add1);
                add1.clear();

                if(debugInterpolation)
                    qDebug()<<"D: [debugInterpolation] "<<rNode.nodePos.id<<"\t"<<angle/3.14159*180<<"\t"<<isoscelesAngle/3.14159*180
                                              <<"\t"<<chord
                                              <<"\t"<<a/3.14159*180<<"\t"<<b/3.14159*180
                                              <<"\t"<<xx<<"\t"<<yy;
            }
        }
        else if(edgeType == RIGHT)  //右转
        {
            //计算v圆心坐标
            double x0=0,y0=0;
            calcuCircleCenter(x1,y1,x2,y2,dis,edgeType,x0,y0);
            if(debugInterpolation)
                qDebug()<<"D: [debugInterpolation] \t\tcalcuCircleCenter:"<<x0<<y0;

            //分段
            double radAngle = angle * 3.14159 / 180;        //角度用弧度表示
            double curveDis = dis * radAngle;               //弧长
            int cnt = int(curveDis)/resolution;             //按弧长分段
            double aveAngle = radAngle/cnt;                 //按角度分段

            if(i == 0 && debugInterpolation)
            {
                qDebug()<<"D: [debugInterpolation] \t"<<curveDis<<"\t"<<cnt<<"\t"<<aveAngle;
                //qDebug()<<"D: [debugInterpolation] \t"<<atan((y2-y1)/(x2-x1))/3.14159*180;
            }

            //插值
            for(int j = 0; j < cnt-1; j++)
            {

                double angle = (j+1)*aveAngle;               //圆心角
                double isoscelesAngle = (3.14159 - angle)/2; //等腰底角
                double chord = 2 * dis * sin(angle/2);       //弦长

                double a = atan((y0-y1)/(x0-x1));
                double b = 3.14159 - (isoscelesAngle - a);

                int flag = (y1>y2) ? 1 : -1;
                if(repairEdge.contains(edgeIndex))          //修复一些边
                    flag = flag *(-1);

                double xx = x1 - flag*chord*cos(b);
                double yy = y1 - flag*chord*sin(b);

                add1 = {xx,yy,double(_route[i]),double(_route[i+1]),double(edgeIndex)};
                _denseRoute.append(add1);
                add1.clear();

                if(debugInterpolation)
                    qDebug()<<"D: [debugInterpolation] "<<rNode.nodePos.id<<"\t"<<angle/3.14159*180<<"\t"<<isoscelesAngle/3.14159*180
                                              <<"\t"<<chord
                                              <<"\t"<<a/3.14159*180<<"\t"<<b/3.14159*180
                                              <<"\t"<<xx<<"\t"<<yy;
            }  
    }
  }
}

/* Function: VehicleConsole::calcuCircleCenter
 * 路径规划与跟踪：计算圆弧的圆心坐标
 */
void VehicleConsole::calcuCircleCenter(double x1, double y1, double x2, double y2,
                                       double r, int dir, double &x0, double &y0)
{
    double c1 = (x2*x2 - x1*x1 + y2*y2 - y1*y1) / (2*(x2 - x1));
    double c2 = (y2 - y1) / (x2 - x1);
    double A = c2*c2 + 1;
    double B = 2*x1*c2 - 2*c1*c2 - 2*y1;
    double C = x1*x1 - 2*x1*c1 + c1*c1 + y1*y1 - r*r;

    if(dir==LEFT)
    {
        if(x1>x2)
            y0 = (-B + sqrt(B*B - 4*A*C)) / (2*A);
        else
            y0 = (-B - sqrt(B*B - 4*A*C)) / (2*A);
    }
    else if(dir==RIGHT)
    {
        if(x1>x2)
            y0 = (-B - sqrt(B*B - 4*A*C)) / (2*A);
        else
            y0 = (-B + sqrt(B*B - 4*A*C)) / (2*A);
    }
    else
        return;

    x0 = c1 - c2*y0;
}

/* Function: VehicleConsole::on_PCtimeout
 * 定时检测位置，并发出指令
 */
void VehicleConsole::on_PCtimeout()
{
    realtimeControl();
}

/* Function: VehicleConsole::realtimeControl
 * 路径规划与跟踪：实时控制
 */
void VehicleConsole::realtimeControl()
{
//根据当前位置与目标位置计算距离
    double curX_mm = curPoseX_m*1000.0;
    double curY_mm = curPoseY_m*1000.0;
    double curYaw_rad = curPoseYaw_rad;
    double nextX_mm = densePath.at(nextPoint).at(0);
    double nextY_mm = densePath.at(nextPoint).at(1);
    double d_mm = sqrt(pow(curX_mm-nextX_mm,2)+pow(curY_mm-nextY_mm,2));

//记录车辆所在路段信息
    leavingNode = densePath.at(nextPoint-2).at(2);
    arrivingNode = densePath.at(nextPoint-2).at(3);
    locatedEdge = densePath.at(nextPoint-2).at(4);

//当接近目标位置时，刹车或更换待跟踪的节点
    if(d_mm < disClose2Node_mm)
    {
        //到达终点，刹车，关定时器
        if(nextPoint == finishPoint && d_mm < disFinish_mm)
        {
            qDebug()<<"I: Path follow ending, finish dis(mm) ="<<d_mm;

            timerPosCheck.stop();
            emit stop();

            densePath.clear();  //这里要清空：解决循迹过程中改变目标的情况，若未清空说明上一次循迹未结束
            curOrder = "000";

            return;
        }

        //未到达终点，更新待跟踪节点
        nextPoint = ((nextPoint+2) > (densePath.count()-1)) ? nextPoint : nextPoint+2;
        return;
    }


//计算控制量——角度
    //统一角度基准：x轴正向为起始，向y轴负方向旋转0～360度，在imu数据处理中也有统一基准部分
        double dy_mm = (nextY_mm-curY_mm);
        double dx_mm = (nextX_mm-curX_mm);
        double beta_rad = fabs(atan(dy_mm/dx_mm));

        if(dy_mm<0 && dx_mm>0)
            beta_rad = beta_rad;
        else if(dy_mm<0 && dx_mm<0)
            beta_rad = 3.14159 - beta_rad;
        else if(dy_mm>0 && dx_mm<0)
            beta_rad = 3.14159 + beta_rad;
        else
            beta_rad = -beta_rad;

        double alpha_rad = beta_rad - curYaw_rad;
        double theta_rad = atan(2*L_mm*sin(alpha_rad)/d_mm);

    //计算转角
        if(theta_rad > 0)
            emit turn(1,theta_rad);//向左转
        else
            emit turn(0,theta_rad);//向右转

//计算控制量——速度
        int dir = 1;
        int v_r_s = setSpeed_m_s/(2.0*3.14159*0.03);

        //倒车判断
            if(locatedEdge > 200)
            {
                //倒车判断：位于车库，进行泊车中: 入库且该边在入库时属于倒车边，或出库且该边在入库时不属于倒车边, 则倒车
                //逻辑如下表：
                //isReserve\isEntering    true   false
                //   true                 true   false
                //   false                false   true
                bool isReserve = staticInfo.getParkEdgeInfoS()->value(locatedEdge).attr_isReverse;
                dir = (isReserve == isParkIn) ? 0 : 1;
            }

        //减速过弯
            int routeType = 0;//根据路段类型，遇到转弯减速行驶
            if(locatedEdge > 200)
                routeType = staticInfo.getParkEdgeInfoS()->value(locatedEdge).attr_type;
            else
                routeType = staticInfo.getMapEdgeInfoS()->value(locatedEdge).attr_type;

            if(routeType != lastRouteType)//减速这一行为仅发生在入弯和出弯
            {
                if(routeType==LEFT || routeType==RIGHT)
                    v_r_s /= 2.0;

                lastRouteType = routeType;
            }

         //控制
            emit run(dir,v_r_s);

    if(debugRouteFollow)
        qDebug()<<"D:[debugRouteFollow]"
                <<"\tPose"<<curX_mm<<curY_mm<<curYaw_rad/3.14159*180<<curSpeed_r_s*2.0*3.14159*0.03
                <<"\tGoal"<<nextX_mm<<nextY_mm<<beta_rad<<nextPoint<<"/"<<arrivingNode
                <<"\t\tControl:turn"<<theta_rad/3.14159*180<<"d\tControl:speed"<<setSpeed_m_s<<"\ttype"<<routeType<<curDirection<<dir<<locatedEdge;

}

//车路协同
/* Function: VehicleConsole::on_TLtimeout
 * 定时更新前方红绿灯编号
 */
void VehicleConsole::on_TLtimeout()
{
    //判断是否位于信号灯影响区
    const QList<QList<double>> *TLInfoS = staticInfo.getTLInfoS();

    int tlID = 0;
    for(int i=0;i<TLInfoS->size();i++)
    {
        QList<double> tl = TLInfoS->at(i);
        double xmin = tl.at(0);
        double xmax = tl.at(1);
        double ymin = tl.at(2);
        double ymax = tl.at(3);
        if((curPoseX_m*1000.0 > xmin) && (curPoseX_m*1000.0 < xmax) && (curPoseY_m*1000.0 > ymin) && (curPoseY_m*1000.0 < ymax))
            tlID = i+1;
    }

    crossingTLInfo.clear();     //前方信号灯信息，列表一般有3个值，分别是信号灯编号1~11、所属控制器编号1~5、控制器组内编号
    switch (tlID)
    {
        case 1: crossingTLInfo<<1<<1<<1;break;
        case 2: crossingTLInfo<<2<<1<<2;break;
        case 3: crossingTLInfo<<3<<1<<3;break;
        case 4: crossingTLInfo<<4<<2<<1;break;
        case 5: crossingTLInfo<<5<<2<<2;break;
        case 6: crossingTLInfo<<6<<3<<1;break;
        case 7: crossingTLInfo<<7<<3<<2;break;
        case 8: crossingTLInfo<<8<<4<<1;break;
        case 9: crossingTLInfo<<9<<4<<2;break;
        case 10: crossingTLInfo<<10<<5<<1;break;
        case 11: crossingTLInfo<<11<<5<<2;break;
        default: crossingTLInfo<<0<<0<<0;
    }

    if(debugTL)
        qDebug()<<"D: [debugTL] crossingTLInfo: "<<crossingTLInfo;
}

//自动泊车
/* Function: VehicleConsole::autoParking
 * 泊车主函数，输入的是目标车位
 */
void VehicleConsole::autoParking(int parkingID, bool isParkIn_, int outID)
{
    if(enableAP)
    {
        //传进来的车位编号一定是空闲的...

        //1 更新泊车状态
        isParkIn = isParkIn_;

        //2 入库准备
        if(isParkIn)
        {
            if(parkingID>7 || parkingID<1)//排除错误车位
            {
                qDebug()<<"W: Input wrong carport ID.";
                return;
            }

            carportID = parkingID;

            //2.1 将车辆导航到泊车起始位置：节点150
            pathPreprocess(arrivingNode, staticInfo.getMapGateInfoS()->parkIn);//arrivingNode

            //2.2 等待车辆到达指定位置
            apFlag = staticInfo.getMapGateInfoS()->parkIn;
        }
        //3 出库准备
        else
        {
            parkPathPreprocess(carportID);
            outTarget = outID;
            apFlag = -4;
        }

        timerAP.start(timeAP_ms);
    }
}

/* Function: VehicleConsole::parkingInto
 * 泊车入位或出位
 */
void VehicleConsole::parkPathPreprocess(int parkingID)
{
    qDebug()<<"I: Park path peprocessing: \tRun, start...";
    qDebug()<<"I: \t\tPark path planning, start...";
    //路径规划
        parkingPath.clear();

    //根据泊车位，选择路线
        switch (parkingID)
        {
            case 1: parkingPath<<201<<202<<203<<204;break;
            case 2: parkingPath<<201<<202<<205<<206<<207<<208;break;
            case 3: parkingPath<<201<<202<<205<<206<<209<<210<<211;break;
            case 4: parkingPath<<201<<202<<205<<212<<213;break;
            case 5: parkingPath<<201<<214<<215;break;
            case 6: parkingPath<<201<<202<<205<<206<<209<<216<<217<<218<<219;break;
            case 7: parkingPath<<201<<202<<205<<206<<209<<216<<220<<221;break;
            default :parkingPath.clear();
        }

        if(parkingPath.isEmpty())
            return;

        //判断是入库还是出库：若是出库，则在路线的基础上，增加节点222
        if(!isParkIn)
            parkingPath.insert(0,222);

    qDebug()<<"I: final parkingPath:"<<parkingPath;

    qDebug()<<"I: \t\tPath planning, done.\n\t\tInterpolation, start...";

    //插值，得到稠密路径
        densePath.clear();
        interpolation(parkingPath,densePath,true);

        //根据出入的不同，路径要选择顺序还是倒序：若是出库，则将插值的稠密路径倒序
        if(!isParkIn)
        {
            int halfCount = densePath.size()/2;
            int count = densePath.size();
            for(int i=0;i<halfCount;i++)
            {
                densePath.swap(i,count-1-i);
            }
        }

    qDebug()<<"I: \t\tInterpolation, done.";
    qDebug()<<"I: \t\tGet the closest point, start...";

    //循环找到最近点
        double curX = curPoseX_m*1000.0;
        double curY = curPoseY_m*1000.0;
        int minID = 0;
        double minDis = pow(curX-densePath[0].at(0),2) + pow(curY-densePath[0].at(1),2);
        for(int k = 1; k < densePath.count(); k++)
    {
        double d = pow(curX-densePath[k].at(0),2) + pow(curY-densePath[k].at(1),2);
        if(d<minDis)
        {
            minDis = d;
            minID = k;
        }
    }

    //依次确定出发点，第一个待跟踪点和最终点
        firstPoint = minID;
        nextPoint = ((minID + 2) < (densePath.count()-1)) ? (minID + 2) : (densePath.count()-1);
        finishPoint = densePath.count()-1;

        qDebug()<<"I: \t\tCurrent position:"<<curX<<curY;
        qDebug()<<"I: \t\t"<<"Get the first point to follow:\tID:"<<QString::number(firstPoint)
                <<"\tx:"<<QString::number(densePath[firstPoint].at(0))
                <<"\ty:"<<QString::number(densePath[firstPoint].at(1));
        qDebug()<<"I: \t\t"<<"Get the next point to follow:\tID:"<<QString::number(nextPoint)
                <<"\tx:"<<QString::number(densePath[nextPoint].at(0))
                <<"\ty:"<<QString::number(densePath[nextPoint].at(1));
        qDebug()<<"I: \t\t"<<"Get the finish point to follow:\tID:"<<QString::number(finishPoint)
                <<"\tx:"<<QString::number(densePath[finishPoint].at(0))
                <<"\ty:"<<QString::number(densePath[finishPoint].at(1));


    //循迹定时器启动
    if(enableFollow && enableAP)
    {
        timerPosCheck.start(timePC_ms);
        qDebug()<<"D:[debugRouteFollow]"
                  "\t\tPose x y yaw speed"
                  "\tGoal x y next/node"
                  "\t\t\tControl:turn theta (grade)"
                  "\t\tControl:speed"
                  "\ttype routeType";
    }
}

/* Function: VehicleConsole::on_APtimeout
 * 定时检查车辆是否到达目标
 */
void VehicleConsole::on_APtimeout()
{
    //入库
    if(isParkIn)
    {
        //判断车辆是否到达泊车起始位置，到达后开始泊车
        if(apFlag == staticInfo.getMapGateInfoS()->parkIn)
        {
            double goalX = staticInfo.getMapNodeInfoS()->value(staticInfo.getMapGateInfoS()->parkIn).nodePos.x;
            double goalY = staticInfo.getMapNodeInfoS()->value(staticInfo.getMapGateInfoS()->parkIn).nodePos.y;
            double d = sqrt(pow((goalX-curPoseX_m*1000.0),2)+pow((goalY-curPoseY_m*1000.0),2));
            if(d < disFinish_mm)
            {
                qDebug()<<"I: Path follow ending by AP_in, finish dis(mm) ="<<d;

                timerPosCheck.stop();
                emit stop();

                parkPathPreprocess(carportID);
                apFlag = carportID;
            }
        }
        //泊车完成，告知主控
        else if(apFlag == carportID)
        {
            double goalX = staticInfo.getParkNodeInfoS()->value(204).nodePos.x;
            double goalY = staticInfo.getParkNodeInfoS()->value(204).nodePos.y;
            double d = sqrt(pow((goalX-curPoseX_m*1000.0),2)+pow((goalY-curPoseY_m*1000.0),2));
            if(d < disFinish_mm)
            {
                send2master("$R,"+QString::number(carID)+",162,"+QString::number(carportID)+",*BB\r\n");//泊车完成，告知主控

                //关定时器、刹车
                timerAP.stop();
                timerPosCheck.stop();
                emit stop();

                //初始化变量
                apFlag = 0;
                curOrder = "000";
                densePath.clear();
                qDebug()<<"AP ending";
            }
        }
    }
    //出库
    else
    {
        //判断车辆是否到达出库闸口（222点附近），达到后再出发去指定位置
        if(-4 == apFlag)
        {
            double goalX = staticInfo.getParkNodeInfoS()->value(222).nodePos.x;
            double goalY = staticInfo.getParkNodeInfoS()->value(222).nodePos.y;
            double d = sqrt(pow((goalX-curPoseX_m*1000.0),2)+pow((goalY-curPoseY_m*1000.0),2));
            if(d < disFinish_mm)
            {
                qDebug()<<"I: Path follow ending by AP_out, finish dis(mm) ="<<d;

                send2master("$R,"+QString::number(carID)+",163,"+QString::number(carportID)+",*BB\r\n");//出库完成，告知主控

                timerPosCheck.stop();
                emit stop();

                apFlag = -5;
                densePath.clear();
            }
        }
        //车辆出库完成，行驶到规定位置
        else if(-5 == apFlag)
        {
            if(outTarget>0)
                pathPreprocess(151, outTarget);
            else
                pathPreprocess(151, staticInfo.getMapGateInfoS()->parkOut);

            apFlag = 0;
            curOrder = "000";
            timerAP.stop();
        }
    }
}

//------------------------------------------------------------------------------------------车云通信模块
/* Function: VehicleConsole::initTCPClient
 * 初始化与主控的通信
 */
void VehicleConsole::initTCPClient()
{
    tcpClient=new QTcpSocket(this);

    connect(tcpClient,SIGNAL(connected()),this,SLOT(on_Connected()));
    connect(tcpClient,SIGNAL(disconnected()),this,SLOT(on_Disconnected()));
    connect(tcpClient,SIGNAL(stateChanged(QAbstractSocket::SocketState)),this,SLOT(on_SocketStateChange(QAbstractSocket::SocketState)));
    connect(tcpClient,SIGNAL(readyRead()),this,SLOT(on_SocketReadyRead()));
}

/* Function: VehicleConsole::connect2master
 * 连接到主控
 */
void VehicleConsole::connect2master()
{
    initTCPClient();
    tcpClient->connectToHost(masterIP,masterPort);

    connect(&timerState2Master,SIGNAL(timeout()),this,SLOT(on_state2MasterTimeout()));
    timerState2Master.start(timeS2M_ms);
}

/* Function: VehicleConsole::disconnect2master
 * 断开与主控的连接
 */
void VehicleConsole::disconnect2master()
{
    timerState2Master.stop();

    if(tcpClient->state()==QAbstractSocket::ConnectedState)
        tcpClient->disconnectFromHost();
    delete tcpClient;

}

/* Function: VehicleConsole::send2master
 * 给总控发送消息
 */
void VehicleConsole::send2master(QString _msg)
{
    if(tcpClient->state()!=QAbstractSocket::ConnectedState)
        return;

    if(_msg.isEmpty())
        return;

    QByteArray  str=_msg.toUtf8();
    tcpClient->write(str);
}

/* Function: VehicleConsole::on_Connected
 * 成功建立连接后触发的槽
 */
void VehicleConsole::on_Connected()
{
    qDebug()<<"I: succeed to connnect to master";
}

/* Function: VehicleConsole::on_Disconnected
 * 成功断开连接后触发的槽
 */
void VehicleConsole::on_Disconnected()
{
    emit stop();        //一旦在行驶途中与主控断开连接，立即刹车
    qDebug()<<"I: succeed to disconnnect to master";
}

/* Function: VehicleConsole::on_SocketStateChange
 * socket连接状态
 */
void VehicleConsole::on_SocketStateChange(QAbstractSocket::SocketState socketState)
{
    Q_UNUSED(socketState)
}

/* Function: VehicleConsole::on_SocketReadyRead
 * 读取主控传入的数据
 */
void VehicleConsole::on_SocketReadyRead()
{
    QByteArray byte;
    byte=tcpClient->readAll();
    QString commond=QString::fromLatin1(byte);

    QStringList msgs = commond.split(",");
    QString id = msgs.at(1);
    if(id.toInt()!=carID)
        return;

//指令解析
    qDebug()<<"I: commond:  "<<commond;
    if(commond.contains("$C"))
    {
        QString order = msgs.at(2);
//循迹
        if(order.contains("150"))
        {            
            QString target = msgs.at(5);
            int source = arrivingNode;//车辆当前所在点

            if(curOrder.contains("151") || 0 == source)//排除泊车时循迹、车辆之前还没进行任何的运动（没有产生对应的前目标节点，不执行或先动一动再执行）等情况
                return;

            curOrder = order;
            if(arrivingNode > 200)//判断车是否在车库中，在车库则先出库
                autoParking(0, false, target.toInt());
            else
                pathPreprocess(source, target.toInt());
        }
//泊车
        else if(order.contains("151"))
        {                
            QString target = msgs.at(3);
            if(curOrder.contains("151") || target.toInt()>7 || target.toInt()<1 || arrivingNode>200)//排除泊车时再控制泊车、错误车位、已在车位等情况
                return;

            curOrder = order;
            autoParking(target.toInt(), true);
        }
//速度控制
        else if(order.contains("152"))
        {
            QString controlSpeed = msgs.at(3);

            if(0 == controlSpeed.toInt())
            {
                timerPosCheck.stop();
                emit stop();
            }
            else if(1 == controlSpeed.toInt())
            {
                QString goalSpeed = msgs.at(4);
                if((goalSpeed.toInt()>0) && (goalSpeed.toInt()<1000))//暂时不允许用户修改车辆前进或后退
                    setSpeed_m_s = setSpeed_m_s - goalSpeed.toInt()/1000.0;
            }
            else if(2 == controlSpeed.toInt())
            {
                QString goalSpeed = msgs.at(4);
                if((goalSpeed.toInt()>0) && (goalSpeed.toInt()<1000))//暂时不允许用户修改车辆前进或后退
                    setSpeed_m_s = setSpeed_m_s + goalSpeed.toInt()/1000.0;
            }
            else if(3 == controlSpeed.toInt())
            {
                timerPosCheck.start(timePC_ms);
            }
        }
//编队控制
        else if(order.contains("153"))
        {
            QString isBelong2Formation = msgs.at(3);
            if(0 == isBelong2Formation.toInt())
                belong2Formation = false;
            else if(1 == isBelong2Formation.toInt())
                belong2Formation = true;

            QString index = msgs.at(4);
                formationIndex = index.toInt();
        }
        else
        {
            curOrder = "000";
            return;
        }
    }
    else if(commond.contains("$A"))
    {
        QString order = msgs.at(2);
    }
}

/* Function: VehicleConsole::on_state2MasterTimeout
 * 定时给主控发送车辆自身状态
 */
void VehicleConsole::on_state2MasterTimeout()
{
    QString state = "$S,"+QString::number(carID)+","
                         +QString::number(curPoseX_m*1000)+","+QString::number(curPoseY_m*1000)+","+QString::number(curPoseYaw_rad*1000)+","
                         +QString::number(locatedEdge)+","+QString::number((curSpeed_r_s*2*3.14159*0.03*1000))+","
                         +QString::number(0)+","+curOrder+",*BB\r\n";
    send2master(state);
}

//------------------------------------------------------------------------------------------车路通信模块
/* Function: VehicleConsole::connect2Road
 * 连接到路
 */
void VehicleConsole::connect2Road()
{
    udpSocket = new QUdpSocket(this);
    connect(udpSocket,SIGNAL(readyRead()),this,SLOT(on_udpSocketReadyRead()));
    udpSocket->bind(tlPort);

    connect(&timerTL,SIGNAL(timeout()),this,SLOT(on_TLtimeout()));
    timerTL.start(timeTL_ms);
}

/* Function: VehicleConsole::disconnect2Road
 * 断开与路的连接
 */
void VehicleConsole::disconnect2Road()
{
    udpSocket->abort();

    timerTL.stop();

    delete udpSocket;
}

/* Function: VehicleConsole::on_udpSocketReadyRead
 * 读取红绿灯传入的数据，只保留距离最近的红绿灯信息
 */
void VehicleConsole::on_udpSocketReadyRead()
{
    //无效信号灯信息
    if(3 != crossingTLInfo.size())
        return;

    //对于前方没有红绿灯的情况，不解析红绿灯广播数据
    if(0 == crossingTLInfo.first())
        return;

    while(udpSocket->hasPendingDatagrams())
    {
        QByteArray      datagram;
        QHostAddress    peerAddr;
        quint16         peerPort;
        datagram.resize(udpSocket->pendingDatagramSize());
        udpSocket->readDatagram(datagram.data(),datagram.size(),&peerAddr,&peerPort);
//仅考虑某个控制器下的信号灯
        int ip = peerAddr.toString().right(3).toInt();
        if((ip-199) != crossingTLInfo.at(1))
            return;

//读取数据
        bool ok;
        QString tfData = QString(datagram.toHex());

        int greenIndex = 20+(crossingTLInfo.last()-1)*6;
        int yellowIndex = greenIndex + 2;
        int redIndex = greenIndex + 4;

        int green = tfData.left(greenIndex).right(2).toInt(&ok,16);
        int yellow = tfData.left(yellowIndex).right(2).toInt(&ok,16);
        int red = tfData.left(redIndex).right(2).toInt(&ok,16);

//判断是否刹车
        if(enableTL&&enableFollow)    //只有功能使能，才能红灯停
        {
            if(belong2Formation && 1 == formationIndex)
            {
                if(red>0||yellow>0||green<10)//对于队列的头车，需要的绿灯时间长才能完全通过
                {
                    //刹车
                    timerPosCheck.stop();
                    emit stop();
                }
            }
            else if(belong2Formation && 1 != formationIndex)
            {
                //对于队列中，非头车，忽视信号灯
            }
            else if(red>0||yellow>0||green<2)
            {
                //刹车
                timerPosCheck.stop();
                emit stop();
            }
            else
            {
                //重新启动
                timerPosCheck.start(timePC_ms);
            }
        }

//展示
        int id = ip - 199;
        QString labelName = "tl"+QString::number(id);
        QString rButtonGName = "light"+QString::number(id)+"G";
        QString rButtonYName = "light"+QString::number(id)+"Y";
        QString rButtonRName = "light"+QString::number(id)+"R";
        QLabel *label = ui->groupTL->findChild<QLabel *>(labelName);
        QPushButton * rButtonG = ui->groupTL->findChild<QPushButton *>(rButtonGName);
        QPushButton * rButtonY = ui->groupTL->findChild<QPushButton *>(rButtonYName);
        QPushButton * rButtonR = ui->groupTL->findChild<QPushButton *>(rButtonRName);


        if(green > 0)
        {
            label->setText(QString::number(green));
            rButtonG->setStyleSheet("background-color:rgb(0,255,0);border:none;");
            rButtonY->setStyleSheet("background-color:rgb(240,240,240)");
            rButtonR->setStyleSheet("background-color:rgb(240,240,240)");
        }
        else if(yellow > 0)
        {
            label->setText(QString::number(yellow));
            rButtonG->setStyleSheet("background-color:rgb(240,240,240)");
            rButtonY->setStyleSheet("background-color:rgb(255,255,0);border:none;");
            rButtonR->setStyleSheet("background-color:rgb(240,240,240)");
        }
        else if(red > 0)
        {
            label->setText(QString::number(red));
            rButtonG->setStyleSheet("background-color:rgb(240,240,240)");
            rButtonY->setStyleSheet("background-color:rgb(240,240,240)");
            rButtonR->setStyleSheet("background-color:rgb(255,0,0);border:none;");
        }
        else
            label->setText(QString::number(0));
    }
}

//------------------------------------------------------------------------------------------车载传感器模块:IMU/NFC、激光、相机
/* Function: VehicleConsole::connect2Sensor
 * 初始化传感器
 */
void VehicleConsole::connect2Sensor()
{
    pose.initPose(this);                        //初始化位姿对象
    pose.moveToThread(&sensor_pose);            //将位姿传感器的数据处理过程移至sensor线程
    lidar.initLidar(this);                      //初始化激光雷达对象
    lidar.moveToThread(&sensor_lidar);          //将激光雷达的数据处理过程移至sensor线程
    camera.initCamera(this);                    //初始化激光雷达对象
    camera.moveToThread(&sensor_camera);        //将相机的数据处理过程移至sensor线程

    sensor_pose.start();                        //启动sensor线程
    sensor_lidar.start();
    sensor_camera.start();
    emit toStartSensor();                       //开始接收传感器数据
}

/* Function: VehicleConsole::on_curPose
 * 接收sensor线程传来的实时位姿
 */
void VehicleConsole::on_curPose(double poseX_m, double poseY_m, double poseYaw_rad)
{
    curPoseX_m = poseX_m;
    curPoseY_m = poseY_m;
    curPoseYaw_rad = poseYaw_rad;
    ui->displayPosition->setText("\tx: "+QString::number(curPoseX_m)
                                        +" m\ty: "+QString::number(curPoseY_m)
                                        +" m\tyaw: "+QString::number(curPoseYaw_rad)
                                        +" rad\t"+QString::number(curPoseYaw_rad/3.14159*180.0)
                                        +" degree\t");
}

/* Function: VehicleConsole::on_obstacleInfo
 * 接收sensor线程传来的障碍物信息
 */
void VehicleConsole::on_obstacleInfo(bool isStop, double obstacleDir, double obstacleDis_mm, QString info)
{
    QString msgs = QString::number(obstacleDir) + "? " + QString::number(obstacleDis_mm) + "mm ";

    if(debugLidar)
        qDebug()<<"D: [debugLidar] Test Lidar obstacle: "<<msgs;

    //激光避障
    if(enableLidarAvoid)
    {
        int type = staticInfo.getMapEdgeInfoS()->value(locatedEdge).attr_type;
        bool breakFlag = false;
        switch (type)
        {
        case 1: if((obstacleDir<22) || (obstacleDir > 338)) breakFlag = true; break;
        case 2: if(obstacleDir > 315) breakFlag = true; break;
        case 3: if(obstacleDir < 45) breakFlag = true; break;
        default:   emit run(2,-2);break;//维持运动
        }

        if(breakFlag)
        {
            timerPosCheck.stop();
            emit stop();
        }
        else if(!breakFlag && (curSpeed_r_s < 0.1))
            timerPosCheck.start(timePC_ms);
    }
}

//------------------------------------------------------------------------------------------车辆运动控制模块
/* Function: VehicleConsole::connect2Motion
 * 初始化底盘控制
 */
void VehicleConsole::connect2Motion()
{
    motioner.initMotion(this,&pose);            //初始化底盘控制对象
    motioner.moveToThread(&motion);             //将底盘控制的过程移至motion线程

    motion.start();                             //启动motion线程
    emit toStartMotion();                       //启动运动反馈
}

/* Function: VehicleConsole::on_curSpeedDir
 * 接收motion线程传来的实时速度及其转向
 */
void VehicleConsole::on_curSpeedDir(double _curSpeed_r_s, bool _curDirection)
{
    curSpeed_r_s = _curSpeed_r_s;
    curDirection = _curDirection;
    ui->displaySpeed->setText("\t"+QString::number(curSpeed_r_s)+" r/s\t"+QString::number(curDirection));
}

//------------------------------------------------------------------------------------------TEST
//运动测试
void VehicleConsole::on_btnTestWheelRun_clicked()//手动变速
{
    int index=ui->testRunComboBox->currentIndex();
    double value=ui->testRunValue->value();
    if(value<0.1)
        return;

    if(index==0)
        emit run(1,value);//向前
    else if(index==1)
        emit run(0,value);//向后
    else
        emit stop();

    //emit setPID(ui->pid_p->value(),ui->pid_i->value(),ui->pid_d->value());
    emit setPID(2,1,0.5);
}
void VehicleConsole::on_btnTestWheelTurn_clicked()//手动转向
{
    int index=ui->testTurnComboBox->currentIndex();
    int value=ui->testTurnValue->value();

    double angle_rad = value/180.0*3.14159;

    if(index==0)
        emit turn(1,angle_rad);//左转
    else if(index==1)
        emit turn(0,angle_rad);//右转
}
void VehicleConsole::on_stop_clicked()//手动刹车
{
    emit stop();
}
//功能测试
void VehicleConsole::on_test_clicked()//规划功能测试
{
    pathPreprocess(ui->startNode->value(),ui->finalNode->value());


    QString display;
    for(int i=0;i<path.size();i++)
        display.append(QString::number(path.at(i))+" ");
    ui->path->setText(display);
/*
    displayPath = new Form();
    connect(this,SIGNAL(showP(QList<QList<double>> *)),displayPath,SLOT(showPath(QList<QList<double>> *)));
    displayPath->show();
    emit showP(&densePath);
/*/
}
void VehicleConsole::on_park_clicked()//泊车功能测试
{
    //配速
    double v_r_s = 2;
    setSpeed_m_s = v_r_s*(2.0*3.14159*0.03);
    emit setPID(1,1,0);

    //直接泊车
    isParkIn = true;
    carportID = 1;

    parkPathPreprocess(carportID);
    apFlag = carportID;

    timerAP.start(timeAP_ms);

    emit run(0,v_r_s);

    //路径规划
    /*parkingPath.clear();
    switch (ui->parkID->value())
    {
        case 1: parkingPath<<201<<202<<203<<204;break;
        case 2: parkingPath<<201<<202<<205<<206<<207<<208;break;
        case 3: parkingPath<<201<<202<<205<<206<<209<<210<<211;break;
        case 4: parkingPath<<201<<202<<205<<212<<213;break;
        case 5: parkingPath<<201<<214<<215;break;
        case 6: parkingPath<<201<<202<<205<<206<<209<<216<<217<<218<<219;break;
        case 7: parkingPath<<201<<202<<205<<206<<209<<216<<220<<221;break;
        default :parkingPath.clear();
    }

    parkingPath.insert(0,222);

    if(parkingPath.isEmpty())
        return;

    //插值，得到稠密路径
    densePath.clear();
    interpolation(parkingPath,densePath,true);
    //qDebug()<<densePath;

    int halfCount = densePath.size()/2;
    int count = densePath.size();
    for(int i=0;i<halfCount;i++)
    {
        densePath.swap(i,count-1-i);
    }
    //Debug()<<densePath;

//*
    displayPath = new Form();
    connect(this,SIGNAL(showP(QList<QList<double>> *)),displayPath,SLOT(showPath(QList<QList<double>> *)));
    displayPath->show();
    emit showP(&densePath);
//*/
}
void VehicleConsole::on_test2_clicked()//循迹功能测试
{
    //配速
    double v_r_s = 1.5;
    setSpeed_m_s = v_r_s*(2.0*3.14159*0.03);
    emit setPID(1,1,0);

    //规划并开始循迹
    pathPreprocess(8,41);

    emit run(1,v_r_s);

}
//辅助测试定时器
void VehicleConsole::on_test3_clicked()//测试定时器
{
    if(testTimer->isActive())
    {
        testTimer->stop();
        return;
    }
    else
        testTimer->start(20);

    /*
    displayPath = new Form();
    connect(this,SIGNAL(showP(QList<QList<double>> *)),displayPath,SLOT(showPath(QList<QList<double>> *)));
    connect(this,SIGNAL(appendP(double,double)),displayPath,SLOT(appendPath(double,double)));
    displayPath->show();//*/
}
void VehicleConsole::on_testTimerTimeout()//测试用定时器触发函数
{
    if(curSpeed_r_s < -0.1)
        return;

    //定位测试
    QString str = "\tx: "+QString::number(curPoseX_m)
                        +" m\ty: "+QString::number(curPoseY_m)
                        +" m\tyaw: "+QString::number(curPoseYaw_rad)
                        +" rad\t"+QString::number(curPoseYaw_rad/3.14159*180.0)
                        +" degree\t";
    emit appendP(curPoseX_m,curPoseY_m);

    //速度测试
    //QString speed = QString::number(curSpeed_r_s);
    //qDebug()<<curSpeed_r_s;
}

void VehicleConsole::on_test4_clicked()
{
    int value = ui->spinBox->value();
    emit test(value);
}
