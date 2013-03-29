#include "mainwindow.hpp"

using namespace basestation;
using namespace std;
using namespace boost;
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow), baseStation(argc, argv) {
    ui->setupUi(this);

    QObject::connect(&baseStation, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&baseStation, SIGNAL(showSolvedSudocubeSignal(QString,int)), this, SLOT(showSolvedSudocubeSlot(QString,int)));
    QObject::connect(&baseStation, SIGNAL(loopEndedSignal(QString)), this, SLOT(loopEndedSlot(QString)));
    QObject::connect(&baseStation, SIGNAL(UpdatingRobotPositionSignal(float,float)), this, SLOT(UpdatingRobotPositionSlot(float,float)));
    QObject::connect(&baseStation, SIGNAL(showConfirmStartRobotSignal(QString)), this, SLOT(showConfirmStartRobotSlot(QString)));
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_StartSequenceButton_clicked() {
    baseStation.init();
}

void MainWindow::showSolvedSudocubeSlot(QString solvedSudocube, int redCaseValue) {
    stringstream strs;
    strs << redCaseValue;
    string temp_str = strs.str();
    char* redCaseChar = (char*) temp_str.c_str();

    ui->consoleText->append("Kinocto : Solution du sudocube");

    QString redCase("Valeur de la case rouge : ");
    redCase.append(redCaseChar);
    ui->consoleText->append(redCase);

    ui->consoleText->append("Solution :");
    ui->consoleText->append(solvedSudocube);
    ui->consoleText->append("");
}

void MainWindow::loopEndedSlot(QString loopEndedMessage) {
    ui->consoleText->append(loopEndedMessage);
}

void MainWindow::UpdatingRobotPositionSlot(float x, float y) {
    ui->consoleText->append("Kinocto : Mise a jour de la position du robot : ");

    stringstream strs;
    strs << "(";
    strs << x;
    strs << ",";
    strs << y;
    strs << ")";
    string temp_str = strs.str();
    char* position = (char*) temp_str.c_str();

    ui->consoleText->append(position);
}

void MainWindow::showConfirmStartRobotSlot(QString showConfirmStartRobot) {
    ui->consoleText->append(showConfirmStartRobot);
}
