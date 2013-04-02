#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QTimer>
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

class MainWindow: public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

private slots:
    void updateROSSlot();
    void on_StartSequenceButton_clicked();
    void showSolvedSudocubeSlot(QString, int);
    void UpdatingRobotPositionSlot(float, float);
    void showMessage(QString);
    void traceRealTrajectory(vector<Position>);
    void updateObstaclesPositions(int,int,int,int);

private:
    Ui::MainWindow *ui;
    BaseStation baseStation;
    QTimer *updateROSTimer;

    Position obstacle1;
    Position obstacle2;
    Position actualPosition;
    vector<Position> plannedPath;
    vector<Position> kinoctoPositionUpdates;

    //PRINTING RELATED METHODS
    void printTable();
    void setObstaclesInMatrixTable();
    // In array,
    // 1 = normal area
    // 2 = node
    // 9 = wall or obstacle
    int table[TABLE_X + 1][TABLE_Y + 1];

    cv::Scalar white;
    cv::Scalar blue;
    cv::Scalar black;
    cv::Scalar red;
    void showWindowWith(const char*, const cv::Mat &);
    void colorPixel(cv::Mat&, cv::Scalar, int, int);
    void drawLine(cv::Mat, cv::Point, cv::Point, cv::Scalar);
};

#endif // MAINWINDOW_H
