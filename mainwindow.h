#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidget>
#include "stdafx.h"

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

    void displayAngles(QVector<float>* Angles);

private:
    Ui::MainWindow *ui;
    QSerialPort* SerialPort;
    Eigen::Quaternionf uparm,forarm, hand, bluetooth,handzero,forearmzero,bluetoothmodified;
    Eigen::Quaternionf RecordQuat;
    const Eigen::Quaternionf QuatInit = Eigen::Quaternionf (1.0,0,0,0);
signals:
    void starting(WiseGlove * g_pGlove0, QTableWidget * tableWidget);
};
#endif // MAINWINDOW_H
