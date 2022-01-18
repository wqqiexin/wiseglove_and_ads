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

}

void GainAngles::working(WiseGlove *g_pGlove0,QTableWidget* tableWidget, QSerialPort* SerialPort)
{
    unsigned int validdata[2];
    Eigen::Quaternionf uparm,forarm, hand, bluetooth;
    EulerAngles Uparm, Forearm, Wrist;
    float angle[19];  //临时存储角度变量
    float m_quat[16],quat[16]; //手臂传感器值位于m_quat[0]-m_quat[2]
    float dispangle[19];  //用于屏幕显示传感器角度
    SerialPort->readAll();

    QVector<float>* Angles = new QVector<float>(39);
    while(1){
        validdata[0] = g_pGlove0->GetAngle(angle);
        g_pGlove0->GetQuat(m_quat); //w,x,y,z
//       while(!GetQuat(SerialPort,bluetooth));
        while(!SerialPort->write("hzy"));

        if (validdata[0] > 0)  //数据有效
        {
            for (int i = 0; i < 19; i++)
            {
                dispangle[i] = angle[i];
            }
            //指间角度取反
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

            Wrist = ToEulerAngles(handzero);
            for(int i = 0; i < 39; i++)
            {
                tableWidget->setItem(i,1,new QTableWidgetItem(QString("%1").arg((*Angles)[i])));
            }
            QThread::msleep(200);

        }//end_if (validdata[0] > 0)  //数据有效
    }//end_while(1)
    emit sendArray(Angles);

}


bool GetQuat(QSerialPort* SerialPort, Eigen::Quaternionf& bluetooth)
{

    char FrameState[1];
    char Q[9];
    short Q0L,Q0H,Q1L,Q1H,Q2L,Q2H,Q3L,Q3H;
    while(!(SerialPort->read(FrameState,1)) )
    {
        qDebug() <<"IMU读取失败";
        qDebug("FrameState[0]:%x",FrameState[0]);
    }; //如果接受失败或者接收的不是0x55则退出循环
    while(!(SerialPort->getChar(FrameState)));
    switch (*FrameState)
    {
        case 0x59:
            if( -1 ==SerialPort->read(Q,9))
            {
                return false;
            }
            Q0L = Q[0];
            bluetooth.w() = ((Q[1] << 8) | Q[0]) /32768;
            bluetooth.x() = ((Q[3] << 8) | Q[2]) /32768;
            bluetooth.y() = ((Q[5] << 8) | Q[4]) /32768;
            bluetooth.z() = ((Q[7] << 8) | Q[6]) /32768;
    default:
        return false;

    }
    qDebug() <<"IMU采集成功";
    return true;

}
