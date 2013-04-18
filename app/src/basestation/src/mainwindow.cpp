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

    loopNumber = 0;
    timeOfLoop->start();

    applicationTimer = new QTimer(this);
    timeValue = new QTime(0, 10, 0); //10 minutes
    ui->TimeBeforeEnd->setPalette(Qt::black);
    ui->TimeBeforeEnd->display(timeValue->toString());
    QObject::connect(applicationTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    QImage imageObject;
    if(imageObject.load("sudocubeVide.jpg")) {
        imageObject.scaledToHeight(ui->sudocube->height());
        imageObject.scaledToWidth(ui->sudocube->width());
        ui->sudocube->setPixmap(QPixmap::fromImage(imageObject));
        ui->sudocube->show();
    }else {
        ui->consoleText->append("could not load sudocube image");
    }

    initSudocubeImage();
    ui->roue0x->setValue(92.0f);
    ui->roue0y->setValue(19.5f);
    ui->roue1x->setValue(83.0f);
    ui->roue1y->setValue(35.8f);

    QObject::connect(&baseStation, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&baseStation, SIGNAL(showSolvedSudocubeSignal(QString,int,int)), this, SLOT(showSolvedSudocubeSlot(QString,int,int)));
    QObject::connect(&baseStation, SIGNAL(message(QString)), this, SLOT(showMessage(QString)));
    QObject::connect(&baseStation, SIGNAL(updateTableImage(QImage)), this, SLOT(updateTableImage(QImage)));
    QObject::connect(&baseStation, SIGNAL(endLoop(QString)), this, SLOT(endLoop(QString)));
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::initSudocubeImage() {
    QPalette* palette = new QPalette();
    palette->setColor(QPalette::WindowText,Qt::black);
    ui->sudocubeCase1->setText("");
    ui->sudocubeCase2->setText("");
    ui->sudocubeCase3->setText("");
    ui->sudocubeCase4->setText("");
    ui->sudocubeCase5->setText("");
    ui->sudocubeCase6->setText("");
    ui->sudocubeCase7->setText("");
    ui->sudocubeCase8->setText("");
    ui->sudocubeCase9->setText("");
    ui->sudocubeCase10->setText("");
    ui->sudocubeCase11->setText("");
    ui->sudocubeCase12->setText("");
    ui->sudocubeCase13->setText("");
    ui->sudocubeCase14->setText("");
    ui->sudocubeCase15->setText("");
    ui->sudocubeCase16->setText("");
    ui->sudocubeCase17->setText("");
    ui->sudocubeCase18->setText("");
    ui->sudocubeCase19->setText("");
    ui->sudocubeCase20->setText("");
    ui->sudocubeCase21->setText("");
    ui->sudocubeCase22->setText("");
    ui->sudocubeCase23->setText("");
    ui->sudocubeCase24->setText("");
    ui->sudocubeCase25->setText("");
    ui->sudocubeCase26->setText("");
    ui->sudocubeCase27->setText("");
    ui->sudocubeCase28->setText("");
    ui->sudocubeCase29->setText("");
    ui->sudocubeCase30->setText("");
    ui->sudocubeCase31->setText("");
    ui->sudocubeCase32->setText("");
    ui->sudocubeCase33->setText("");
    ui->sudocubeCase34->setText("");
    ui->sudocubeCase35->setText("");
    ui->sudocubeCase36->setText("");
    ui->sudocubeCase37->setText("");
    ui->sudocubeCase38->setText("");
    ui->sudocubeCase39->setText("");
    ui->sudocubeCase40->setText("");
    ui->sudocubeCase41->setText("");
    ui->sudocubeCase42->setText("");
    ui->sudocubeCase43->setText("");
    ui->sudocubeCase44->setText("");
    ui->sudocubeCase45->setText("");
    ui->sudocubeCase46->setText("");
    ui->sudocubeCase47->setText("");
    ui->sudocubeCase48->setText("");
    ui->sudocubeCase1->setPalette(*palette);
    ui->sudocubeCase2->setPalette(*palette);
    ui->sudocubeCase3->setPalette(*palette);
    ui->sudocubeCase4->setPalette(*palette);
    ui->sudocubeCase5->setPalette(*palette);
    ui->sudocubeCase6->setPalette(*palette);
    ui->sudocubeCase7->setPalette(*palette);
    ui->sudocubeCase8->setPalette(*palette);
    ui->sudocubeCase9->setPalette(*palette);
    ui->sudocubeCase10->setPalette(*palette);
    ui->sudocubeCase11->setPalette(*palette);
    ui->sudocubeCase12->setPalette(*palette);
    ui->sudocubeCase13->setPalette(*palette);
    ui->sudocubeCase14->setPalette(*palette);
    ui->sudocubeCase15->setPalette(*palette);
    ui->sudocubeCase16->setPalette(*palette);
    ui->sudocubeCase17->setPalette(*palette);
    ui->sudocubeCase18->setPalette(*palette);
    ui->sudocubeCase19->setPalette(*palette);
    ui->sudocubeCase20->setPalette(*palette);
    ui->sudocubeCase21->setPalette(*palette);
    ui->sudocubeCase22->setPalette(*palette);
    ui->sudocubeCase23->setPalette(*palette);
    ui->sudocubeCase24->setPalette(*palette);
    ui->sudocubeCase25->setPalette(*palette);
    ui->sudocubeCase26->setPalette(*palette);
    ui->sudocubeCase27->setPalette(*palette);
    ui->sudocubeCase28->setPalette(*palette);
    ui->sudocubeCase29->setPalette(*palette);
    ui->sudocubeCase30->setPalette(*palette);
    ui->sudocubeCase31->setPalette(*palette);
    ui->sudocubeCase32->setPalette(*palette);
    ui->sudocubeCase33->setPalette(*palette);
    ui->sudocubeCase34->setPalette(*palette);
    ui->sudocubeCase35->setPalette(*palette);
    ui->sudocubeCase36->setPalette(*palette);
    ui->sudocubeCase37->setPalette(*palette);
    ui->sudocubeCase38->setPalette(*palette);
    ui->sudocubeCase39->setPalette(*palette);
    ui->sudocubeCase40->setPalette(*palette);
    ui->sudocubeCase41->setPalette(*palette);
    ui->sudocubeCase42->setPalette(*palette);
    ui->sudocubeCase43->setPalette(*palette);
    ui->sudocubeCase44->setPalette(*palette);
    ui->sudocubeCase45->setPalette(*palette);
    ui->sudocubeCase46->setPalette(*palette);
    ui->sudocubeCase47->setPalette(*palette);
    ui->sudocubeCase48->setPalette(*palette);
}

void MainWindow::updateROSSlot () {
    baseStation.loop();
}

void MainWindow::on_StartSequenceButton_clicked() {
    baseStation.setStateToSendStartLoopMessage();
    applicationTimer->start(1000);

    initSudocubeImage();

    loopNumber++;
    stringstream loopNumberSS;
    loopNumberSS << loopNumber;
    ui->loopNumber->setText((char*) loopNumberSS.str().c_str());

    QString loopText;
    loopText.append("\n");
    loopText.append(ui->loopTime->text());
    ui->loopTime->setText(loopText);

    timeOfLoop->restart();
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

void MainWindow::on_saveParametersbtn_clicked() {
    float roue1x = ui->roue1x->value();
    float roue1y = ui->roue1y->value();
    float roue0x = ui->roue0x->value();
    float roue0y = ui->roue0y->value();
    float robotRadius = 10;

    Position robotPosition;
    float coteOppose;
    float coteAdjacent;
    float angleRad;
    float angle;
    float xOffset;
    float yOffset;

    if(roue0x <= roue1x && roue0y >= roue1y) { //CADRAN 3
        coteOppose = roue0y - roue1y;
        coteAdjacent = roue1x - roue0x;
        angleRad = atan(coteOppose/coteAdjacent);
        angle = angleRad * 180/CV_PI;
        angle = angle - 180;
        xOffset = -1 * sin(angleRad) * robotRadius;
        yOffset = -1 * cos(angleRad) * robotRadius;
    } else if(roue0x <= roue1x && roue0y <= roue1y) { // CADRAN 4
        coteOppose = roue1y - roue0y;
        coteAdjacent = roue1x - roue0x;
        angleRad = atan(coteOppose/coteAdjacent);
        angle = angleRad * 180/CV_PI;
        angle = 180 - angle;
        xOffset = sin(angleRad) * robotRadius;
        yOffset = -1 * cos(angleRad) * robotRadius;
    } else if(roue0x >= roue1x && roue0y <= roue1y) { //CADRAN 1
        coteOppose = roue0x - roue1x;
        coteAdjacent = roue1y - roue0y;
        angleRad = atan(coteOppose/coteAdjacent);
        angle = angleRad * 180/CV_PI;
        angle = 90 - angle;
        xOffset = cos(angleRad) * robotRadius;
        yOffset = sin(angleRad) * robotRadius;
    } else if(roue0x >= roue1x && roue0y >= roue1y) { //CADRAN 2
        coteOppose = roue0y - roue1y;
        coteAdjacent = roue0x - roue1x;
        angleRad = atan(coteOppose/coteAdjacent);
        angle = angleRad * 180/CV_PI;
        angle = -angle;
        xOffset = -1 * sin(angleRad) * robotRadius;
        yOffset = cos(angleRad) * robotRadius;
    }

    robotPosition.x = (roue0x + roue1x)/2 + xOffset;
    robotPosition.y = (roue0y + roue1y)/2 + yOffset;

    stringstream consoleoutput;
    consoleoutput << "Initial position set to :  ";
    consoleoutput << "(" << robotPosition.x << "," << robotPosition.y << ") with angle " << angle;

    baseStation.sendRobotPosAndAngle(robotPosition.x, robotPosition.y, angle);

    ui->consoleText->append((char*) consoleoutput.str().c_str());
}

void MainWindow::showSolvedSudocubeSlot(QString solvedSudocube, int redCaseValue, int redCasePosition) {
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

    showRedCase(redCasePosition);
    showSolvedSudocubeInImage(solvedSudocube);
}

void MainWindow::showRedCase(int redCasePosition) {
    QPalette* palette = new QPalette();
    palette->setColor(QPalette::WindowText,Qt::red);
    switch (redCasePosition)
    {
    case 1: ui->sudocubeCase1->setPalette(*palette);
        break;
    case 2: ui->sudocubeCase2->setPalette(*palette);
        break;
    case 3: ui->sudocubeCase3->setPalette(*palette);
        break;
    case 4: ui->sudocubeCase4->setPalette(*palette);
        break;
    case 5: ui->sudocubeCase5->setPalette(*palette);
        break;
    case 6: ui->sudocubeCase6->setPalette(*palette);
        break;
    case 7: ui->sudocubeCase7->setPalette(*palette);
        break;
    case 8: ui->sudocubeCase8->setPalette(*palette);
        break;
    case 9: ui->sudocubeCase9->setPalette(*palette);
        break;
    case 10: ui->sudocubeCase10->setPalette(*palette);
        break;
    case 11: ui->sudocubeCase11->setPalette(*palette);
        break;
    case 12: ui->sudocubeCase12->setPalette(*palette);
        break;
    case 13: ui->sudocubeCase13->setPalette(*palette);
        break;
    case 14: ui->sudocubeCase14->setPalette(*palette);
        break;
    case 15: ui->sudocubeCase15->setPalette(*palette);
        break;
    case 16: ui->sudocubeCase16->setPalette(*palette);
        break;
    case 17: ui->sudocubeCase17->setPalette(*palette);
        break;
    case 18: ui->sudocubeCase18->setPalette(*palette);
        break;
    case 19: ui->sudocubeCase19->setPalette(*palette);
        break;
    case 20: ui->sudocubeCase20->setPalette(*palette);
        break;
    case 21: ui->sudocubeCase21->setPalette(*palette);
        break;
    case 22: ui->sudocubeCase22->setPalette(*palette);
        break;
    case 23: ui->sudocubeCase23->setPalette(*palette);
        break;
    case 24: ui->sudocubeCase24->setPalette(*palette);
        break;
    case 25: ui->sudocubeCase25->setPalette(*palette);
        break;
    case 26: ui->sudocubeCase26->setPalette(*palette);
        break;
    case 27: ui->sudocubeCase27->setPalette(*palette);
        break;
    case 28: ui->sudocubeCase28->setPalette(*palette);
        break;
    case 29: ui->sudocubeCase29->setPalette(*palette);
        break;
    case 30: ui->sudocubeCase30->setPalette(*palette);
        break;
    case 31: ui->sudocubeCase31->setPalette(*palette);
        break;
    case 32: ui->sudocubeCase32->setPalette(*palette);
        break;
    case 33: ui->sudocubeCase33->setPalette(*palette);
        break;
    case 34: ui->sudocubeCase34->setPalette(*palette);
        break;
    case 35: ui->sudocubeCase35->setPalette(*palette);
        break;
    case 36: ui->sudocubeCase36->setPalette(*palette);
        break;
    case 37: ui->sudocubeCase37->setPalette(*palette);
        break;
    case 38: ui->sudocubeCase38->setPalette(*palette);
        break;
    case 39: ui->sudocubeCase39->setPalette(*palette);
        break;
    case 40: ui->sudocubeCase40->setPalette(*palette);
        break;
    case 41: ui->sudocubeCase41->setPalette(*palette);
        break;
    case 42: ui->sudocubeCase42->setPalette(*palette);
        break;
    case 43: ui->sudocubeCase43->setPalette(*palette);
        break;
    case 44: ui->sudocubeCase44->setPalette(*palette);
        break;
    case 45: ui->sudocubeCase45->setPalette(*palette);
        break;
    case 46: ui->sudocubeCase46->setPalette(*palette);
        break;
    case 47: ui->sudocubeCase47->setPalette(*palette);
        break;
    case 48: ui->sudocubeCase48->setPalette(*palette);
        break;
    default: cout << "Erreur sur la position de la case rouge";
        break;
    }
}

void MainWindow::showSolvedSudocubeInImage(QString solvedSudocube) {
    solvedSudocube.replace(" ","");
    solvedSudocube.replace("\n","");
    ui->sudocubeCase1->setText(solvedSudocube.mid(0,1));
    ui->sudocubeCase2->setText(solvedSudocube.mid(1,1));
    ui->sudocubeCase3->setText(solvedSudocube.mid(2,1));
    ui->sudocubeCase4->setText(solvedSudocube.mid(3,1));
    ui->sudocubeCase5->setText(solvedSudocube.mid(4,1));
    ui->sudocubeCase6->setText(solvedSudocube.mid(5,1));
    ui->sudocubeCase7->setText(solvedSudocube.mid(6,1));
    ui->sudocubeCase8->setText(solvedSudocube.mid(7,1));
    ui->sudocubeCase9->setText(solvedSudocube.mid(8,1));
    ui->sudocubeCase10->setText(solvedSudocube.mid(9,1));
    ui->sudocubeCase11->setText(solvedSudocube.mid(10,1));
    ui->sudocubeCase12->setText(solvedSudocube.mid(11,1));
    ui->sudocubeCase13->setText(solvedSudocube.mid(12,1));
    ui->sudocubeCase14->setText(solvedSudocube.mid(13,1));
    ui->sudocubeCase15->setText(solvedSudocube.mid(14,1));
    ui->sudocubeCase16->setText(solvedSudocube.mid(15,1));
    ui->sudocubeCase17->setText(solvedSudocube.mid(16,1));
    ui->sudocubeCase18->setText(solvedSudocube.mid(17,1));
    ui->sudocubeCase19->setText(solvedSudocube.mid(18,1));
    ui->sudocubeCase20->setText(solvedSudocube.mid(19,1));
    ui->sudocubeCase21->setText(solvedSudocube.mid(20,1));
    ui->sudocubeCase22->setText(solvedSudocube.mid(21,1));
    ui->sudocubeCase23->setText(solvedSudocube.mid(22,1));
    ui->sudocubeCase24->setText(solvedSudocube.mid(23,1));
    ui->sudocubeCase25->setText(solvedSudocube.mid(24,1));
    ui->sudocubeCase26->setText(solvedSudocube.mid(25,1));
    ui->sudocubeCase27->setText(solvedSudocube.mid(26,1));
    ui->sudocubeCase28->setText(solvedSudocube.mid(27,1));
    ui->sudocubeCase29->setText(solvedSudocube.mid(28,1));
    ui->sudocubeCase30->setText(solvedSudocube.mid(29,1));
    ui->sudocubeCase31->setText(solvedSudocube.mid(30,1));
    ui->sudocubeCase32->setText(solvedSudocube.mid(31,1));
    ui->sudocubeCase33->setText(solvedSudocube.mid(32,1));
    ui->sudocubeCase34->setText(solvedSudocube.mid(33,1));
    ui->sudocubeCase35->setText(solvedSudocube.mid(34,1));
    ui->sudocubeCase36->setText(solvedSudocube.mid(35,1));
    ui->sudocubeCase37->setText(solvedSudocube.mid(36,1));
    ui->sudocubeCase38->setText(solvedSudocube.mid(37,1));
    ui->sudocubeCase39->setText(solvedSudocube.mid(38,1));
    ui->sudocubeCase40->setText(solvedSudocube.mid(39,1));
    ui->sudocubeCase41->setText(solvedSudocube.mid(40,1));
    ui->sudocubeCase42->setText(solvedSudocube.mid(41,1));
    ui->sudocubeCase43->setText(solvedSudocube.mid(42,1));
    ui->sudocubeCase44->setText(solvedSudocube.mid(43,1));
    ui->sudocubeCase45->setText(solvedSudocube.mid(44,1));
    ui->sudocubeCase46->setText(solvedSudocube.mid(45,1));
    ui->sudocubeCase47->setText(solvedSudocube.mid(46,1));
    ui->sudocubeCase48->setText(solvedSudocube.mid(47,1));
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

    QTime* timeElapsed = new QTime();
    timeElapsed->setHMS(0, timeElapsed->addMSecs(timeOfLoop->elapsed()).minute(), timeElapsed->addMSecs(timeOfLoop->elapsed()).second());

    QString loopText;
    loopText = timeElapsed->toString("mm:ss");
    loopText.append(ui->loopTime->text());
    ui->loopTime->setText(loopText);
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
