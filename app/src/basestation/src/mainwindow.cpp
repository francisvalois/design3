#include "mainwindow.hpp"

using namespace basestation;
using namespace std;
using namespace boost;
using namespace Qt;
using namespace cv;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow), baseStation(argc, argv) {
    ui->setupUi(this);

    updateROSTimer = new QTimer(this);
    QObject::connect(updateROSTimer, SIGNAL(timeout()), SLOT(updateROSSlot()));
    updateROSTimer->start(200);

    applicationTimer = new QTimer(this);
    timeValue = new QTime(0, 10, 0); //10 minutes
    ui->TimeBeforeEnd->setPalette(Qt::black);
    ui->TimeBeforeEnd->display(timeValue->toString());
    QObject::connect(applicationTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    QObject::connect(&baseStation, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&baseStation, SIGNAL(showSolvedSudocubeSignal(QString,int)), this, SLOT(showSolvedSudocubeSlot(QString,int)));
    QObject::connect(&baseStation, SIGNAL(message(QString)), this, SLOT(showMessage(QString)));
    QObject::connect(&baseStation, SIGNAL(updateTableImage(QImage)), this, SLOT(updateTableImage(QImage)));
    QObject::connect(&baseStation, SIGNAL(endLoop(QString)), this, SLOT(endLoop(QString)));
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::updateROSSlot () {
    baseStation.loop();
}

void MainWindow::on_StartSequenceButton_clicked() {
    baseStation.setStateToSendStartLoopMessage();
    applicationTimer->start(1000);
}

void MainWindow::on_calibrateKinectButton_clicked() {
    ui->consoleText->append("Calibration Automatique de la Kinect");
    KinectCapture kinectCapture;
    KinectCalibrator kinectCalibrator;

    kinectCapture.openCapture();
    kinectCalibrator.calibrate(kinectCapture.captureRGBMatrix(), kinectCapture.captureDepthMatrix());
    kinectCapture.closeCapture();
}

void MainWindow::on_calibrateKinectManualButton_clicked() {
    ui->consoleText->append("Calibration Manuelle de la Kinect");
    KinectCalibrator kinectCalibrator;
    
    kinectCalibrator.calibratev2();
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

void MainWindow::showMessage(QString message) {
    ui->consoleText->append(message);
}

void MainWindow::updateTableImage(QImage image) {

    QImage largeImage = image.scaled(Workspace::TABLE_X * 2, Workspace::TABLE_Y * 2, Qt::KeepAspectRatio);
    ui->tableImage->setPixmap(QPixmap::fromImage(largeImage));
    ui->tableImage->show();
}

void MainWindow::endLoop(QString message) {
    applicationTimer->stop();
    showMessage(message);
}

void MainWindow::timerSlot() {
    if(timeValue->minute() == 0 && timeValue->second() == 0) {
        ui->TimeBeforeEnd->setPalette(Qt::red);
        endLoop("Kinocto : Loop Ended, Time Expired");
    } else {
        timeValue->setHMS(0, timeValue->addSecs(-1).minute(), timeValue->addSecs(-1).second());
        ui->TimeBeforeEnd->display(this->timeValue->toString());
    }

}
