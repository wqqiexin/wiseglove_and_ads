#include "vrepsim.h"
#include <QMessageBox>
VrepSim::VrepSim(QObject *parent ): QObject(parent)
{
    simxFinish(-1);
    clientID = simxStart((simxChar*)"127.0.0.1", 3000, true, true, 2000, 5);
    if(clientID == -1)
        qDebug() <<   "Vrep连接失败";
    else
        qDebug() <<   "Vrep连接成功";




}

void VrepSim::StartVrep()
{
    simxSynchronous(clientID,1);
    simxSetFloatingParameter(clientID,sim_floatparam_simulation_time_step,dt,simx_opmode_oneshot_wait);
    simxStartSimulation(clientID, simx_opmode_oneshot);
    simxGetObjectHandle(clientID, "LBR_iiwa_7_R800_joint1", &LBR_iiwa_7_R800_joint[0], simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "LBR_iiwa_7_R800_joint2", &LBR_iiwa_7_R800_joint[1], simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "LBR_iiwa_7_R800_joint3", &LBR_iiwa_7_R800_joint[2], simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "LBR_iiwa_7_R800_joint4", &LBR_iiwa_7_R800_joint[3], simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "LBR_iiwa_7_R800_joint5", &LBR_iiwa_7_R800_joint[4], simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "LBR_iiwa_7_R800_joint6", &LBR_iiwa_7_R800_joint[5], simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "LBR_iiwa_7_R800_joint7", &LBR_iiwa_7_R800_joint[6], simx_opmode_oneshot_wait);
    for(int i = 0; i < 7; i++)
    {
        simxSetJointTargetPosition(clientID, LBR_iiwa_7_R800_joint[i], 0, simx_opmode_oneshot);

    }
    while (simxGetConnectionId(clientID) != -1)
    {
        for(int i = 0; i < 7; i++)
        {
            simxSetJointTargetPosition(clientID, LBR_iiwa_7_R800_joint[i], Angles[i], simx_opmode_oneshot);

        }

        simxSynchronousTrigger(clientID);
        QApplication::processEvents();
    }


}


void VrepSim::updateAngles(QVector<double>* theta)
{
    for(int i = 0; i < 7; i++)
    {
        Angles[i] = (*theta)[i];
    }
}
