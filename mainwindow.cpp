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
    QString thumb[39] = {"拇指MCP","拇指IP","食指MCP","食指PIP","食指DIP","中指MCP","中指PIP","中指DIP","无名指MCP","无名指PIP","无名指DIP",
                         "小指MCP","小指PIP","小指DIP","拇指CMC","拇指与食指夹角","食指与中指夹角","中指与环指夹角","环指与小指夹角",
                         "uparm.x","uparm.y","uparm.z","uparm.w","forearm.x","forearm.y","forearm.z","forearm.w",
                         "hand.x","hand.y","hand.z","hand.w","forearmzero.x","forearmzero.y","forearmzero.z","forearmzero.w",
                        "handzero.x","handzero.y","handzero.z","handzero.w"};
    ui->setupUi(this);

    ui->tableWidget->setRowCount(39);
    ui->tableWidget->setColumnCount(3);
    for(int i= 0; i<39; i++){
        ui->tableWidget->setItem(i,0,new QTableWidgetItem(thumb[i]));
    }
    Serial_Init();


    // 1. 创建子线程对象
    QThread* t1 = new QThread;


    // 2. 创建任务类的对象
    GainAngles* ga = new GainAngles;


    // 3. 将任务对象移动到某个子线程中
    ga->moveToThread(t1);

    connect(this, &MainWindow::starting, ga, &GainAngles::working);
    connect(ui->StartGain, &QPushButton::clicked, this, [=]()
    {
        emit starting(g_pGlove0,ui->tableWidget, SerialPort);
        t1->start();
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_Connect_glove_clicked()
{
    /*******************************连接串口**************************************/
    SerialPort = new QSerialPort();
    SerialPort->setPortName(ui->SerialList->currentText());


    if(SerialPort->open(QIODevice::ReadWrite))
    {
        SerialPort->setBaudRate(115200);
        //设置数据位
        SerialPort->setDataBits(QSerialPort::Data8);
        //设置校验位
        SerialPort->setParity(QSerialPort::NoParity);
        //设置流控制
        SerialPort->setFlowControl(QSerialPort::NoFlowControl);
        //设置停止位
        SerialPort->setStopBits(QSerialPort::OneStop);
    }
    else
    {
        QMessageBox::about(NULL, "提示", "串口没有打开！");
        return;
    }

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

}
void MainWindow::Serial_Init()
{
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {   //在portBox中显示可用串口
        ui->SerialList->addItem(info.portName());
    }


}

