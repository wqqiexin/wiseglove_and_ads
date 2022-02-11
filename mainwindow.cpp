#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "hzy.h"
#include "stdafx.h"
#include <QMessageBox>
#include <QString>
#include <QThread>
#include <QVector>
#include <QString>
#include <QStandardItemModel>
#include <iostream>
/************指针变量***************/
//wiseglove指针变量，并初始化
WiseGlove *g_pGlove0 = NULL;
/***********************************/

int glovetotal = 0; //保存手套实际数量
char sn[2][32];    //保存序列号
bool opensuccess[2] = { false, false };  //访问手套成功标志
int numofsensor[2];//保存传感器数量



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    QString thumb[47] = {"拇指MCP","拇指IP","食指MCP","食指PIP","食指DIP","中指MCP","中指PIP","中指DIP","无名指MCP","无名指PIP","无名指DIP",
                         "小指MCP","小指PIP","小指DIP","拇指CMC","拇指与食指夹角","食指与中指夹角","中指与环指夹角","环指与小指夹角",
                         "uparm.x","uparm.y","uparm.z","uparm.w","forearm.x","forearm.y","forearm.z","forearm.w",
                         "hand.x","hand.y","hand.z","hand.w","forearmzero.x","forearmzero.y","forearmzero.z","forearmzero.w",
                        "handzero.x","handzero.y","handzero.z","handzero.w","bluetooth.x","bluetooth.y","bluetoothmodified.z","bluetooth.w","bluetoothmodified.x","bluetoothmodified.y","bluetoothmodified.z","bluetoothmodified.w"};
    ui->setupUi(this);

    ui->tableWidget->setRowCount(47);
    ui->tableWidget->setColumnCount(3);
    for(int i= 0; i<43; i++){
        ui->tableWidget->setItem(i,0,new QTableWidgetItem(thumb[i]));
    }
    Serial_Init();


    // 1. 创建子线程对象
    QThread* t1 = new QThread;
    QThread* t2 = new QThread;


    // 2. 创建任务类的对象
    GainAngles* ga = new GainAngles;


    // 3. 将任务对象移动到某个子线程中
    ga->moveToThread(t1);

    connect(ga,&GainAngles::sendArray, this, &MainWindow::displayAngles);
    connect(this, &MainWindow::starting, ga, &GainAngles::onCreateTimer);
    connect(ui->StartGain, &QPushButton::clicked, this, [=]()
    {
        emit starting(g_pGlove0,ui->tableWidget);
        t1->start();
    });
    qDebug() << " work thread id:" << QThread::currentThreadId();
    RecordQuat.w() =1; RecordQuat.x()=0;RecordQuat.y() = 0;RecordQuat.z() = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_Connect_glove_clicked()
{
    /*******************************连接串口**************************************/


    /****************************************************************************/

   /*******************************连接手套**************************************/
   g_pGlove0 = GetWiseGlove();
   QString flag = "手套数量: ";
    if (g_pGlove0 != NULL)
    {
        glovetotal = g_pGlove0->Scan();
    }
    else{
         QMessageBox::information(this, tr("连接失败"), tr("请重试！！！！"));
    }
    if (glovetotal > 0)
   {
       QMessageBox::information(this, tr("连接成功"), flag+QString::number(glovetotal));
       unsigned int comport = g_pGlove0->GetPort(0);          //取得选定序号的端口号
       unsigned int baudrate = g_pGlove0->GetPortBaudrate(0); //取得选定序号的端口波特率

       opensuccess[0] = g_pGlove0->Open(comport, baudrate);   //打开串口
       if (opensuccess[0])
       {
           g_pGlove0->GetSn(sn[0]);//获得序列号
           //序列号例子 DG05S001R
           if (sn[0][8] == 'R')
           {}
           else
           {}//end_if(sn[0][8] == 'R')
           numofsensor[0] = g_pGlove0->GetNumOfSensor();
       }//end_if (opensuccess[0])
    }// end_if (glovetotal > 0)
    else
    {
        QMessageBox::information(this, tr("连接失败"), tr("请重试！！！！"));
        return;
    }
   /***************************************************************************/

}


void MainWindow::on_StartGain_clicked()
{

}


void MainWindow::on_pushButton_clicked()
{
    if(g_pGlove0 != NULL){
        g_pGlove0->SetCalibMode(CALIB_AUTO);
        g_pGlove0->ResetQuat();
    }
//    RecordQuat.w() = bluetooth.w();RecordQuat.x() = bluetooth.x();RecordQuat.y() = bluetooth.y();RecordQuat.z() = bluetooth.z();


}
void MainWindow::Serial_Init()
{
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {   //在portBox中显示可用串口
        ui->SerialList->addItem(info.portName());
    }


}

void MainWindow::displayAngles(QVector<float>* Angles)
{
    uparm.x() = (*Angles)[19]; //x
    uparm.y() = (*Angles)[20]; //y
    uparm.z() = (*Angles)[21]; //z
    uparm.w() = (*Angles)[22]; //w


    //肘关节
    forarm.x() = (*Angles)[23]; //x
    forarm.y() = (*Angles)[24]; //y
    forarm.z() = (*Angles)[25]; //z
    forarm.w() = (*Angles)[26]; //w


    //腕关节
    hand.x() = (*Angles)[27];  //x
    hand.y() = (*Angles)[28];  //y
    hand.z() = (*Angles)[29]; //z
    hand.w() = (*Angles)[30]; //w

    forearmzero.x() = (*Angles)[31];  //x
    forearmzero.y() = (*Angles)[32];  //y
    forearmzero.z() = (*Angles)[33]; //z
    forearmzero.w() = (*Angles)[34]; //w

    handzero.x() = (*Angles)[35];  //x
    handzero.y() = (*Angles)[36];  //y
    handzero.z() = (*Angles)[37];    //z
    handzero.w() = (*Angles)[38]; //w


    bluetooth.x() = (*Angles)[39];  //x
    bluetooth.y() = (*Angles)[40];  //y
    bluetooth.z() = (*Angles)[41]; //z
    bluetooth.w() = (*Angles)[42]; //w


    bluetoothmodified = quatmul(RecordQuat,bluetooth);
//    std::cout << bluetooth.normalized().toRotationMatrix();
    for(int i= 0; i<43; i++){
        ui->tableWidget->setItem(i,1,new QTableWidgetItem(QString("%1").arg((*Angles)[i])));

    }
}

