#include "hzy.h"

EulerAngles ToEulerAngles(Eigen::Quaternionf q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles.y = std::copysign(pi / 2, sinp); // use 90 degrees if out of range
    else
        angles.y = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles.z = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Eigen::Quaternionf quatmul(Eigen::Quaternionf a, Eigen::Quaternionf b)
{
    float xn, yn, zn, wn;
    wn = a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z();
    xn = a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y();
    yn = a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x();
    zn = a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w();

    //Quaternion q1 = q0 * _qOrigin;
    Eigen:: Quaternionf temp;
    temp.w() = wn;
    temp.x() = xn;
    temp.y() = yn;
    temp.z() = zn;
    float mo = sqrt(temp.w() * temp.w() + temp.x() * temp.x() + temp.y() * temp.y() + temp.z() * temp.z());

    if (mo > 0.8)
    {
        temp.w() /= mo;
        temp.x() /= mo;
        temp.y() /= mo;
        temp.z() /= mo;
    }
    else
    {
        temp.w() = 1.0;
        temp.x() = 0.0;
        temp.y() = 0.0;
        temp.z() = 0.0;
    }

    return temp;
}

Eigen::Quaternionf quatconj(Eigen::Quaternionf a)
{
    Eigen::Quaternionf temp;
    temp.w() = a.w();
    temp.x() = -a.x();
    temp.y() = -a.y();
    temp.z() = -a.z();

    return temp;
}

void QuattoEuler(Eigen::Quaternionf quat, float eular[3])  //x y z
{

    double sqw = quat.w()*quat.w();
    double sqx = quat.x()*quat.x();
    double sqy = quat.y()*quat.y();
    double sqz = quat.z()*quat.z();

    eular[0] = atan2(2.0*(quat.x()*quat.y() + quat.z()*quat.w()), (sqx - sqy - sqz + sqw));  //x
    eular[1] = asin(-2.0*(quat.x()*quat.z() - quat.y()*quat.w()) / (sqx + sqy + sqz + sqw)); //y
    eular[2] = atan2(2.0*(quat.y()*quat.z() + quat.x()*quat.w()), (-sqx - sqy + sqz + sqw)); //z

}

GainAngles::GainAngles(QObject *parent ): QObject(parent)
{
    SerialPort = new QSerialPort(this);
    SerialPort->setPortName("COM12");


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
}


CacluateAngles::CacluateAngles(QObject *parent ): QObject(parent)
{

}


void GainAngles::working()
{
    QVector<float>* Angles = new QVector<float>(43);
    unsigned int validdata[2];
    float m_quat[16],quat[16]; //手臂传感器值位于m_quat[0]-m_quat[2]
    char FrameState[1];
    char Q[9];
    FrameState[0] = 0;
    short Q0L,Q0H,Q1L,Q1H,Q2L,Q2H,Q3L,Q3H;
    EulerAngles Uparm, Forearm, Wrist;
    float angle[19];  //临时存储角度变量
    float dispangle[19];  //用于屏幕显示传感器角度
    while(1)
    {
        while(!SerialPort->getChar(FrameState));
        if(FrameState[0] == 0x55){
           break;

        }

    }
    while(1)
    {
        while(!SerialPort->getChar(FrameState));
        if(FrameState[0] == 0x59){
            while(!SerialPort->read(Q,9));

            Q0L = (short)Q[0]&0x00ff ;Q0H = (short)Q[1]&0x00ff;Q1L = (short)Q[2]&0x00ff;Q1H= (short)Q[3]&0x00ff;Q2L= (short)Q[4]&0x00ff;Q2H= (short)Q[5]&0x00ff;Q3L= (short)Q[6]&0x00ff;Q3H= (short)Q[7]&0x00ff;


            bluetooth.w() = (short)((Q0H << 8) | Q0L)/32768.0 ;
            bluetooth.x() = (short)((Q1H << 8) | Q1L)/32768.0 ;
            bluetooth.y() = (short)((Q2H << 8) | Q2L)/32768.0;
            bluetooth.z() = (short)((Q3H << 8) | Q3L) /32768.0;

            (*Angles)[39]  = bluetooth.x();  //x
            (*Angles)[40]  = bluetooth.y();  //y
            (*Angles)[41]  = bluetooth.z(); //z
            (*Angles)[42]  = bluetooth.w(); //w
            break;
        }
    }

        SerialPort->readAll();
        validdata[0] = g_pGlove0->GetAngle(dispangle);

        g_pGlove0->GetQuat(m_quat);
        if(validdata[0] > 0){
            dispangle[2] = 50.0 - dispangle[2] > 0 ? 50.0 - dispangle[2] : 0;//拇指与食指夹角
            dispangle[6] = 25.0 - dispangle[6] > 0 ? 25.0 - dispangle[6] : 0;//食指与中指夹角
            dispangle[10] = 25.0 - dispangle[10] > 0 ? 25.0 - dispangle[10] : 0;//中指与环指夹角
            dispangle[14] = 25.0 - dispangle[14] > 0 ? 25.0 - dispangle[14] : 0;//环指与小指夹角

            (*Angles)[0] = dispangle[0];//拇指MCP
            (*Angles)[1] = dispangle[1];//拇指IP
            (*Angles)[2] = dispangle[3];//食指MCP
            (*Angles)[3] = dispangle[4];//食指PIP
            (*Angles)[4] = dispangle[5];//食指DIP
            (*Angles)[5] = dispangle[7];//中指MCP
            (*Angles)[6] = dispangle[8];//中指PIP
            (*Angles)[7] = dispangle[9];//中指DIP
            (*Angles)[8] = dispangle[11];//无名指MCP
            (*Angles)[9] = dispangle[12];//无名指PIP
            (*Angles)[10] = dispangle[13];//无名指DIP
            (*Angles)[11] = dispangle[15];//小指MCP
            (*Angles)[12] = dispangle[16];//小指PIP
            (*Angles)[13] = dispangle[17];//小指DIP
            (*Angles)[14] = angle[18];//拇指CMC

            (*Angles)[15] = dispangle[2];//拇指与食指夹角
            (*Angles)[16] = dispangle[6];//食指与中指夹角
            (*Angles)[17] = dispangle[10];//中指与环指夹角
            (*Angles)[18] = dispangle[14];//环指与小指夹角

            quat[0] = m_quat[9]; //x
            quat[1] = -m_quat[11]; //y
            quat[2] = m_quat[10]; //z
            quat[3] = m_quat[8]; //w

           //肘关节
            quat[4] = m_quat[5]; //x
            quat[5] = -m_quat[7]; //y
            quat[6] = m_quat[6]; //z
            quat[7] = m_quat[4]; //w

            //腕关节
            quat[8] = m_quat[1];  //x
            quat[9] = -m_quat[3];  //y
            quat[10] = m_quat[2]; //z
            quat[11] = m_quat[0]; //w


            hand.w() = quat[3];
            hand.x() = quat[0];
            hand.y() = quat[1];
            hand.z() = quat[2];

            forarm.w() = quat[7];
            forarm.x() = quat[4];
            forarm.y() = quat[5];
            forarm.z() = quat[6];

            uparm.w() = quat[11];
            uparm.x() = quat[8];
            uparm.y() = quat[9];
            uparm.z() = quat[10];
            Eigen::Quaternionf uparmconj = quatconj(uparm);
            Eigen::Quaternionf forearmconj = quatconj(forarm);



            Eigen::Quaternionf forearmzero = quatmul(uparmconj, forarm);
            Eigen::Quaternionf handzero = quatmul(forearmconj, hand);

            std::cout << forearmzero.toRotationMatrix() <<std::endl;
            std::cout <<std::endl;
            //xy zy
            //xz
            //肩膀关节
            (*Angles)[19] = quat[0] = uparm.x(); //x
            (*Angles)[20] = quat[1] = uparm.y(); //y
            (*Angles)[21] = quat[2] = uparm.z(); //z
            (*Angles)[22] = quat[3] = uparm.w(); //w
            Uparm = ToEulerAngles(uparm);

            //肘关节
            (*Angles)[23] = quat[4] = forarm.x(); //x
            (*Angles)[24] = quat[5] = forarm.y(); //y
            (*Angles)[25] = quat[6] = forarm.z(); //z
            (*Angles)[26] = quat[7] = forarm.w(); //w
            Forearm = ToEulerAngles(forearmzero);

            //腕关节
            (*Angles)[27] = quat[8] = hand.x();  //x
            (*Angles)[28] = quat[9] = hand.y();  //y
            (*Angles)[29] = quat[10] = hand.z(); //z
            (*Angles)[30] = quat[11] = hand.w(); //w

            (*Angles)[31] = quat[8] = forearmzero.x();  //x
            (*Angles)[32] = quat[9] = forearmzero.y();  //y
            (*Angles)[33] = quat[10] = forearmzero.z(); //z
            (*Angles)[34] = quat[11] = forearmzero.w(); //w

            (*Angles)[35] = quat[8] = handzero.x();  //x
            (*Angles)[36] = quat[9] = handzero.y();  //y
            (*Angles)[37] = quat[10] = handzero.z(); //z
            (*Angles)[38] = quat[11] = handzero.w(); //w
        }
        emit sendArray(Angles);





}

void CacluateAngles::working()
{

}

bool GainAngles::GetQuat(Eigen::Quaternionf& bluetooth)
{

    qDebug() <<"已进入";
    char FrameState[1];
    FrameState[0] = 0;
    char Q[9];
    short Q0L,Q0H,Q1L,Q1H,Q2L,Q2H,Q3L,Q3H;
    static int x =0;
    while(1)
    {
            while(!SerialPort->getChar(FrameState));

    }
//    while(1)
//    {
//        if (SerialPort->waitForReadyRead(10))
//        {
//            SerialPort->getChar(FrameState);
//        }
//        if(FrameState[0] == 0x55)
//            break;
//        else
//        {
//            qDebug() <<"读取帧头失败";
//            qDebug("FrameState: %x", FrameState[0]);
//        }

//    }
//    while (FrameState[0] != 0x55)
//    {
//        qDebug() <<"读取帧头";
//        while(!SerialPort->waitForReadyRead()){qDebug() <<"读取帧头2";}
//        SerialPort->getChar(FrameState);
//        qDebug() <<"读取帧头3";
//    }//如果接受失败或者接收的不是0x55则退出循环
//    qDebug() <<"成功读取帧头";
//    while (!SerialPort->waitForReadyRead(10)){qDebug() <<"读取标识符";}
//    SerialPort->getChar(FrameState);
//    switch (FrameState[0])
//    {
//        case 0x59:
//            qDebug() <<"IMU采集成功";
//            while (!SerialPort->waitForReadyRead(10)){}
//            if( -1 ==SerialPort->read(Q,9))
//            {
//                return false;
//            }

//            bluetooth.w() = ((Q[1] << 8) | Q[0]) /32768;
//            bluetooth.x() = ((Q[3] << 8) | Q[2]) /32768;
//            bluetooth.y() = ((Q[5] << 8) | Q[4]) /32768;
//            bluetooth.z() = ((Q[7] << 8) | Q[6]) /32768;
//        return true;
//        break;
//    default:
//        return false;

//}



//    return true;
    return true;
}


void GainAngles::onCreateTimer(WiseGlove *g_pGlove0,QTableWidget* tableWidget)
{
    //关键点：在子线程中创建QTimer的对象
    timer = new QTimer(this);
    timer->setInterval(200);
    this->g_pGlove0 = g_pGlove0;

    connect(timer, SIGNAL(timeout()), this, SLOT(working()));
    timer->start();
}

void GainAngles::onTimeout()
{
    qDebug() << " work thread id:" << QThread::currentThreadId(); //打印出线程ID，看看是否UI线程的ID不同
}
