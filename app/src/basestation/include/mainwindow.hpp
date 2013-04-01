#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QTimer>
#include <iostream>
#include <string>

#include "../../build/basestation/ui_mainwindow.h"
#include "BaseStation.hpp"

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

private:
    Ui::MainWindow *ui;
    BaseStation baseStation;
    QTimer *updateROSTimer;
};

#endif // MAINWINDOW_H
