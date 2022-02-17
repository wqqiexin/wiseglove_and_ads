#ifndef VREPSIM_H
#define VREPSIM_H

#include <QObject>
#include<Windows.h>
#include <iostream>
#include <QtDebug>
#include <QApplication>
extern "C" {
    #include "extApi.h"
}


class VrepSim : public QObject
{
    Q_OBJECT
public:
    explicit VrepSim(QObject *parent = nullptr);
private:
    simxFloat dt = 0.01;
    simxInt LBR_iiwa_7_R800_joint[7] = {0,0,0,0,0,0,0};
    int clientID{0};
    QVector<double> Angles = {0,0,0,0,0,0,0};

public slots:
    void StartVrep();
    void updateAngles(QVector<double>* theta);
};

#endif // VREPSIM_H
