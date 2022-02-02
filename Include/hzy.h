#ifndef HZY_H
#define HZY_H
#include "stdafx.h"
#include <QThread>
#include <QDebug>
#include <QMessageBox>
#include <QVector>
#include <QStandardItemModel>
#include <QTableWidget>
#include <Eigen/Dense>
#include "QSerialPort" //串口访问
#include "QSerialPortInfo" //串口端口信息访问
#include <QTimer>
#define pi 3.14159265358979323846264338328

struct EulerAngles {
    double x, y, z;
};


//************手套参数****************/

//int numofsensor[2];//保存传感器数量
//char sn[2][32];    //保存序列号
//unsigned int validdata[2]; //数据有效，当数据无效时值为0， 有效时值为数据包的时间戳
//bool opensuccess[2] = { false, false };  //访问手套成功标志
////调节拇指CMC角度的参数，当使用小手时，此值为-28， 大手模式时，-39
//float adjthumb = -36.0f;
//float dispangle[2][19];  //用于屏幕显示传感器角度
//double FlexionAngle[2][15];//屈曲角度
//double AbductionAngle[2][4];//外展角度
///************************************/

///*************手臂参数****************/
//EulerAngles Uparm[2], Forearm[2], Wrist[2];
//float quat[2][12];
//float m_quat[2][16]; //手臂传感器值位于m_quat[0]-m_quat[2]
///***********************************



class GainAngles:public QObject
{
    Q_OBJECT
public:
    explicit GainAngles(QObject *parent = nullptr);



    bool GetQuat(Eigen::Quaternionf & bluetooth); //从蓝牙得到四元数
signals:
    void sendArray(QVector<float>* Angles);
private:
    QSerialPort* SerialPort = nullptr ;
    QTimer* timer = nullptr;

    WiseGlove *g_pGlove0= nullptr;
    QTableWidget* tableWidget = nullptr;
    Eigen::Quaternionf uparm,forarm, hand, bluetooth;
public slots:
    void onCreateTimer(WiseGlove *g_pGlove0,QTableWidget* tableWidget);
    void onTimeout();
    void working();

};




/**************四元数相关函数******************/
EulerAngles ToEulerAngles(QUAT q); //四元数转换欧拉角
Eigen::Quaternionf quatmul(Eigen::Quaternionf a, Eigen::Quaternionf b); //四元数乘法
Eigen::Quaternionf quatconj(Eigen::Quaternionf a); //四元数共轭
void QuattoEuler(Eigen::Quaternionf quat, float eular[3]);

/*****************************************/
#endif // HZY_H
