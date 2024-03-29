#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidget>
#include "stdafx.h"
#include "vrepsim.h"
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void Serial_Init();//串口初始化主函数中调用

private slots:
    void on_Connect_glove_clicked();

    void on_StartGain_clicked();

    void on_pushButton_clicked();

    void displayAngles(QVector<double>* Angles);

    void on_updateAngles_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort* SerialPort;
    Eigen::Quaterniond uparm,forarm, hand, bluetooth,handzero,forearmzero,bluetoothmodified;
    Eigen::Quaterniond RecordQuat;
    const Eigen::Quaterniond QuatInit = Eigen::Quaterniond (1.0,0,0,0);
    QVector<double> theta = {0,0,0,0,0,0,0};

signals:
    void starting(WiseGlove * g_pGlove0, QTableWidget * tableWidget);
    void sendName(QString serialName);
    void ResetQuat();
    void updateAngles(QVector<double>& theta);

};
#endif // MAINWINDOW_H
