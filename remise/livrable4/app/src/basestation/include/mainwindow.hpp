#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QTimer>
#include <QTime>
#include <iostream>
#include <string>
#include <math.h>

#include "../../build/basestation/ui_mainwindow.h"
#include "BaseStation.hpp"

const int OBSTACLE_RADIUS = 7;
const int TABLE_X = 231;
const int TABLE_Y = 114;

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(int argc, char **argv, QWidget *parent = 0);

    ~MainWindow();

private slots:

    void updateROSSlot();

    void on_StartSequenceButton_clicked();

    void on_calibrateKinectButton_clicked();

    void on_calibrateKinectManualButton_clicked();

    void on_saveParametersbtn_clicked();

    void showSolvedSudocubeSlot(QString, int, int);

    void showMessage(QString);

    void updateTableImage(QImage);

    void timerSlot();

    void endLoop(QString);

private:
    Ui::MainWindow *ui;
    BaseStation baseStation;
    QTimer *updateROSTimer;
    int loopNumber;

    Position obstacle1;
    Position obstacle2;
    Position actualPosition;
    std::vector<Position> plannedPath;
    std::vector<Position> kinoctoPositionUpdates;

    QTimer *applicationTimer;
    QTime *timeValue;
    QTime *timeOfLoop;

    void initSudocubeImage();

    void showRedCase(int);

    void showSolvedSudocubeInImage(QString);
};

#endif // MAINWINDOW_H
