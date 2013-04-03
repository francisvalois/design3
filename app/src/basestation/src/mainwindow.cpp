#include "mainwindow.hpp"

using namespace basestation;
using namespace std;
using namespace boost;
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow), baseStation(argc, argv) {
    ui->setupUi(this);

    updateROSTimer = new QTimer(this);
    QObject::connect(updateROSTimer, SIGNAL(timeout()), SLOT(updateROSSlot()));
    updateROSTimer->start(200);

    QObject::connect(&baseStation, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&baseStation, SIGNAL(showSolvedSudocubeSignal(QString,int)), this, SLOT(showSolvedSudocubeSlot(QString,int)));
    QObject::connect(&baseStation, SIGNAL(UpdatingRobotPositionSignal(float,float)), this, SLOT(UpdatingRobotPositionSlot(float,float)));
    QObject::connect(&baseStation, SIGNAL(message(QString)), this, SLOT(showMessage(QString)));
    QObject::connect(&baseStation, SIGNAL(traceRealTrajectorySignal(vector<Position>)), this, SLOT(traceRealTrajectory(vector<Position>)));
    QObject::connect(&baseStation, SIGNAL(updateObstaclesPositions(int,int,int,int)), this, SLOT(updateObstaclesPositions(int,int,int,int)));
    QObject::connect(&baseStation, SIGNAL(updateTableImage(QImage)), this, SLOT(updateTableImage(QImage)));

    white = Scalar(255, 255, 255);
    blue = Scalar(255, 0, 0);
    black = Scalar(0, 0, 0);
    red = Scalar(0, 0, 255);

    obstacle1.x = 0;
    obstacle1.y = 0;
    obstacle2.x = 0;
    obstacle2.y = 0;

    actualPosition.x = 0;
    actualPosition.y = 0;

    for (int y = 0; y <= TABLE_Y; y++) {
        for (int x = 0; x <= TABLE_X; x++) {
            table[x][y] = 0;
        }
    }
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::updateROSSlot () {
    baseStation.loop();
}

void MainWindow::on_StartSequenceButton_clicked() {
    baseStation.setStateToSendStartLoopMessage();
}

void MainWindow::on_calibrateKinectButton_clicked() {
    ui->consoleText->append("Calibration de la Kinect");
    KinectCapture kinectCapture;
    KinectCalibrator kinectCalibrator;
    kinectCalibrator.calibrate(kinectCapture.captureDepthMatrix());
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

void MainWindow::UpdatingRobotPositionSlot(float x, float y) {
    Position position(x,y);
    kinoctoPositionUpdates.push_back(position);

    actualPosition.set(x,y);

    printTable();
}

void MainWindow::showMessage(QString message) {
    ui->consoleText->append(message);
}

void MainWindow::traceRealTrajectory(vector<Position> positions) {
    if(plannedPath.size() > 0) {
        plannedPath.clear();
    }
    plannedPath = positions;

    printTable();
}

void MainWindow::updateObstaclesPositions(int o1x, int o1y, int o2x, int o2y) {
    obstacle1.set(o1x,o1y);
    obstacle2.set(o2x,o2y);
    printTable();
}


void MainWindow::printTable() {
    setObstaclesInMatrixTable();

    Mat workspace = Mat(TABLE_X + 1, TABLE_Y + 1, CV_8UC3, white);

    //Drawing obstacles
    for (int y = TABLE_Y; y >= 0; y--) {
        for (int x = 0; x <= TABLE_X; x++) {
            if (table[x][y] == 9) {
                colorPixel(workspace, black, x, TABLE_Y - y);
            }
        }
    }

    //Drawing actual Kinocto position
    if (actualPosition.x != 0 && actualPosition.y != 0) {
        for (int y = (actualPosition.y - 3); y <= (actualPosition.y + 3); y++) {
            for (int x = (actualPosition.x - 3); x <= (actualPosition.x + 3); x++) {
                colorPixel(workspace, red, x, TABLE_Y - y);
            }
        }
    }

    transpose(workspace, workspace);

    //Drawing kinoctoPositionUpdates
    if(kinoctoPositionUpdates.size() > 0) {
        for(unsigned int i = 0; i < kinoctoPositionUpdates.size() - 1; i++) {
            Point currentPoint(kinoctoPositionUpdates[i].x, TABLE_Y - kinoctoPositionUpdates[i].y);
            Point nextPoint(kinoctoPositionUpdates[i+1].x, TABLE_Y - kinoctoPositionUpdates[i+1].y);
            drawLine(workspace, currentPoint, nextPoint, red);
        }
    }

    //Drawing plannedPath
    if(plannedPath.size() > 0) {
        for(unsigned int i = 0; i < plannedPath.size() - 1; i++) {
            Point currentPoint(plannedPath[i].x, TABLE_Y - plannedPath[i].y);
            Point nextPoint(plannedPath[i+1].x, TABLE_Y - plannedPath[i+1].y);
            drawLine(workspace, currentPoint, nextPoint, blue);
        }
    }

    showWindowWith("Workspace", workspace);
}

void MainWindow::setObstaclesInMatrixTable() {
//	OBSTACLES IN BLACK
    if (obstacle1.x != 0 && obstacle1.y != 0) {
        for (int y = (obstacle1.y - OBSTACLE_RADIUS); y <= (obstacle1.y + OBSTACLE_RADIUS); y++) {
            for (int x = (obstacle1.x - OBSTACLE_RADIUS); x <= (obstacle1.x + OBSTACLE_RADIUS); x++) {
                table[x][y] = 9;
            }
        }
    }
    if (obstacle2.x != 0 && obstacle2.y != 0) {
        for (int y = (obstacle2.y - OBSTACLE_RADIUS); y <= (obstacle2.y + OBSTACLE_RADIUS); y++) {
            for (int x = (obstacle2.x - OBSTACLE_RADIUS); x <= (obstacle2.x + OBSTACLE_RADIUS); x++) {
                table[x][y] = 9;
            }
        }
    }
}

void MainWindow::colorPixel(Mat &mat, Scalar color, int x, int y) {
    mat.at<cv::Vec3b>(x, y)[0] = color[0];
    mat.at<cv::Vec3b>(x, y)[1] = color[1];
    mat.at<cv::Vec3b>(x, y)[2] = color[2];
}

void MainWindow::showWindowWith(const char* name, const Mat &mat) {
    namedWindow(name, CV_WINDOW_AUTOSIZE);
    imshow(name, mat);
    waitKey(0);
}

void MainWindow::drawLine(Mat img, Point start, Point end, Scalar color) {
    int thickness = 2;
    int lineType = 8;
    line(img, start, end, color, thickness, lineType);
}

void MainWindow::updateTableImage(QImage image) {

    QImage largeImage = image.scaled(Workspace::TABLE_X * 2, Workspace::TABLE_Y * 2, Qt::KeepAspectRatio);
    ui->tableImage->setPixmap(QPixmap::fromImage(largeImage));
    ui->tableImage->show();
}
