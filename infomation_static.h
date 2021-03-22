#ifndef INFO_STATIC_H
#define INFO_STATIC_H

#include <QObject>
#include <QMap>
#include <QHash>

/*struct: Position
 * 表示位置的结构体
 * 1 左上角为原点，横向为x，纵向为y
 * 2 用于表示NFC卡位置
 */
struct Position
{
    Position(){}
    Position(double _x, double _y){x=_x; y=_y;}
    Position(double _x, double _y, int _id){x=_x; y=_y; id=_id;}

    double x=0;     //位置x
    double y=0;     //位置y
    int id=0;       //NFC卡的id
};

/*struct: Node/Edge
 * 表示道路相关信息的结构体
 * 1 用于表示道路的节点、边
 */
struct Node_
{
    Node_(){}
    Node_(QList<int> _relatedNodes, QList<int> _relatedEdges, Position _nodePos)
    {
        relatedNodes=_relatedNodes;
        relatedEdges=_relatedEdges;
        nodePos=_nodePos;
    }

    QList<int> relatedNodes;        //与该节点关联的所有节点和边的ID，所谓关联即是根据车道标志（左转等）能够移动到的节点及其对应的边
    QList<int> relatedEdges;
    Position nodePos;               //该节点的位置和ID
};

struct Edge
{
    Edge(){}
    Edge(QList<int> _relatedNodes, int _attr_type,
                double _attr_dis, double _dattr_angle, int _attr_cost, bool _attr_isReverse = false)
    {
        relatedNodes.clear();
        relatedNodes=_relatedNodes;
        attr_type = _attr_type;
        attr_dis = _attr_dis;
        attr_angle = _dattr_angle;
        attr_cost = _attr_cost;
        attr_isReverse = _attr_isReverse;
    }

    QList<int> relatedNodes;        //该道路两端的节点ID
    int attr_type;                  //边属性1：1代表直行，2代表左转弯，3代表右转弯
    double attr_dis;                //边属性2：直行时代表路段长度，转弯时代表路段曲率，单位：mm
    double attr_angle;              //边属性3：直行时取0，转弯时取转弯的角度，单位：度
    int attr_cost;                  //边属性4：计算最短路径时使用的代价值
    bool attr_isReverse;            //边属性5：入库时，是否属于倒车边，车库外的边默认false
};

/*struct: Gate
 * 表示地图中闸口
 */
struct Gate
{
    Gate(){}
    Gate(int _parkIn, int _parkOut, int _chargeIn, int _chargeOut, int _ETCIn, int _ETCOut)
    {
        parkIn = _parkIn;
        parkOut = _parkOut;
        chargeIn = _chargeIn;
        chargeOut = _chargeOut;
        ETCIn = _ETCIn;
        ETCOut = _ETCOut;
    }
    int parkIn = 0;
    int parkOut = 0;
    int chargeIn = 0;
    int chargeOut = 0;
    int ETCIn = 0;
    int ETCOut = 0;
};

/*class: StaticInfo
 * 静态信息分中心
 * 1 车辆的静态信息：车辆ID、车型等
 * 2 道路的静态信息：节点和边组成的拓扑地图
 * 3 NFC_ID和对应位置的映射关系
 */
class StaticInfo : public QObject
{
    Q_OBJECT
public:
    StaticInfo();

//-----------------------映射总览-----------------------
//------|变量|----------|KEY|--------------|VALUE|-----
//   vehicleInfoS      车辆ID              车辆类型
//     NFCInfoS        NFCID              NFC 位置
//   mapNodeInfoS      地图节点ID       地图节点的相关描述
//   mapEdgeInfoS      地图边ID         地图边的相关描述
//----------------------------------------
//所有ID,除NFC外,ID均从1开始,ID==0视为无效值,仅用于判断
//----------------------------------------
private:
    QMap<int,QString> vehicleInfoS;                 //定义车辆基本信息
    QList<QList<double>> TLInfoS;                   //定义信号灯相关信息
    QHash<QByteArray,Position> NFCInfoS;            //定义NFC的相关存储变量,相比map,Hash对于复杂key查找速度更快,同时占用内存更高

    QMap<int,Node_> mapNodeInfoS;                    //定义拓扑地图的节点
    QMap<int,Edge> mapEdgeInfoS;                    //定义拓扑地图的边

    QMap<int,Node_> parkNodeInfoS;                   //定义停车场拓扑地图的节点
    QMap<int,Edge> parkEdgeInfoS;                   //定义停车场拓扑地图的边

    Gate *gate;                                     //定义闸口位置

    void initVehicleInfoS();                        //初始化车辆基本信息
    void initTLInfoS();                             //初始化信号灯相关信息
    void initNFCInfoS();                            //初始化NFC
    void initMapInfoS();                            //初始化拓扑地图

public:
    const QMap<int, QString> *getVehicleInfoS();           //获取车辆基本信息
    const QList<QList<double>> *getTLInfoS();              //获取信号灯相关信息
    const QHash<QByteArray, Position> *getNFCInfoS();      //获取NFC数据

    const QMap<int, Node_> *getMapNodeInfoS();              //获取道路节点数据
    const QMap<int, Edge> *getMapEdgeInfoS();              //获取道路边数据

    const QMap<int, Node_> *getParkNodeInfoS();             //获取停车场节点数据
    const QMap<int, Edge> *getParkEdgeInfoS();             //获取停车场边数据

    const Gate* getMapGateInfoS();                         //获取闸口信息

};

#endif // INFO_STATIC_H
