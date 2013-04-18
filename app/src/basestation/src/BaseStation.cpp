#include "BaseStation.hpp"

using namespace basestation;
using namespace std;
using namespace boost;
using namespace cv;

bool BaseStation::isUpdatingShit = false;

BaseStation::BaseStation(int argc, char** argv) :
        init_argc(argc), init_argv(argv) {

    white = Scalar(255, 255, 255);
    blue = Scalar(255, 0, 0);
    black = Scalar(0, 0, 0);
    red = Scalar(0, 0, 255);
    darkRed = Scalar(0, 0, 155);
    green = Scalar(0, 255, 0);

    obstacle1.x = 0;
    obstacle1.y = 0;
    obstacle2.x = 0;
    obstacle2.y = 0;

    actualPosition.x = 0;
    actualPosition.y = 0;

    state = LOOP;

    kinectCapture = new KinectCapture();

    init();
}

BaseStation::~BaseStation() {
    delete kinectCapture;

    if (ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();

    ROS_INFO("Ros shutdown, proceeding to close the gui.");
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool BaseStation::init() {
    ros::init(init_argc, init_argv, "basestation");
    if (!ros::master::check()) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    ROS_INFO("Creating services handler for Basestation");
    initHandlers(n);

    ROS_INFO("Basestation initiated1");

    start(); // QT THREAD

    return true;
}

void BaseStation::initHandlers(ros::NodeHandle & node) {
    getObstaclesPositionService = node.advertiseService("basestation/getObstaclesPosition", &BaseStation::getObstaclesPosition, this);
    findRobotPositionAndAngleService = node.advertiseService("basestation/findRobotPositionAndAngle", &BaseStation::findRobotPositionAndAngle, this);
    showSolvedSudocubeService = node.advertiseService("basestation/showSolvedSudocube", &BaseStation::showSolvedSudocube, this);
    traceRealTrajectoryService = node.advertiseService("basestation/traceRealTrajectory", &BaseStation::traceRealTrajectory, this);
    loopEndedService = node.advertiseService("basestation/loopEnded", &BaseStation::loopEnded, this);

    startLoopClient = node.serviceClient<kinocto::StartLoop>("kinocto/startLoop");
    setRobotPositionAndAngleClient = node.serviceClient<kinocto::SetRobotPositionAndAngle>("kinocto/setRobotPositionAndAngle");

    updateRobotSubscriber = node.subscribe("basestation/updateRobotPosition", 1, &BaseStation::updateRobotPosition, this);
}

void BaseStation::loop() {
    if (ros::ok()) {
        switch (state) {
        case LOOP:
            //cout << "looping" << endl;
            break;
        case SEND_START_LOOP_MESSAGE:
            sendStartLoopMessage();
            state = LOOP;
            break;
        }

        if (BaseStation::isUpdatingShit == true) {
            BaseStation::isUpdatingShit = false;
            updateShizzle();
        }

        ros::spinOnce();
    }
}

void BaseStation::setStateToSendStartLoopMessage() {
    state = SEND_START_LOOP_MESSAGE;
}

void BaseStation::sendStartLoopMessage() {
    kinocto::StartLoop srv;
    if (startLoopClient.call(srv) == true) {
        ROS_INFO("Showing Confirmation of Start Robot");
        state = LOOP;
        emit message("Kinocto : Start");
    } else {
        ROS_ERROR("Failed to call service kinocto/startLoop");
    }
}

void BaseStation::sendRobotPosAndAngle(double x, double y, double angle) {
    kinocto::SetRobotPositionAndAngle srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.angle = angle;

    if (setRobotPositionAndAngleClient.call(srv) == true) {
        ROS_INFO("Setting Robot angle and position");
        state = LOOP;
    } else {
        ROS_ERROR("Failed to call service kinocto/setRobotPositionAndAngle");
    }
}

bool BaseStation::getObstaclesPosition(GetObstaclesPosition::Request & request, GetObstaclesPosition::Response & response) {
    int const AVERAGECOUNT = 5;
    int obstacle1AverageCount = 0;
    int obstacle2AverageCount = 0;

    //Average the measure for a better precision when the kinect is doing obscure things
    kinectCapture->openCapture();
    for (int i = 0; i < AVERAGECOUNT; i++) {
        Mat depthMatrix = kinectCapture->captureDepthMatrix();
        if (!depthMatrix.data) {
            return false;
        }

        obstaclesDetection.findCenteredObstacle(depthMatrix);
        Vec2f obs1 = obstaclesDetection.getObstacle1();
        Vec2f obs2 = obstaclesDetection.getObstacle2();

        if ((obs1[0] < 0.10 || obs1[1] < 0.20) || obs2[0] < 0.10 || obs2[1] < 0.20) {
            continue;
        }

        if (obs1[0] > 0.10 || obs1[1] > 0.20) {
            response.x1 += obs1[1] * 100;
            response.y1 += obs1[0] * 100;
            obstacle1AverageCount++;
        }

        if (obs2[0] > 0.10 || obs2[1] > 0.20) {
            response.x2 += obs2[1] * 100;
            response.y2 += obs2[0] * 100;
            obstacle2AverageCount++;
        }
    }
    kinectCapture->closeCapture();

    if (obstacle1AverageCount > 0) {
        response.x1 /= obstacle1AverageCount;
        response.y1 /= obstacle1AverageCount;
    }

    if (obstacle2AverageCount > 0) {
        response.x2 /= obstacle2AverageCount;
        response.y2 /= obstacle2AverageCount;
    }

    ROS_INFO( "%s x:%f y:%f  x:%f y:%f", "Request Find Obstacles Position. Sending values ", response.x1, response.y1, response.x2, response.y2);

    stringstream info;
    info << "Kinocto : Demande de la position des obstacles \n";
    info << "Envoi des positions : ";
    info << " (" << response.x1 << "," << response.y1 << ") et ";
    info << " (" << response.x2 << "," << response.y2 << ")";
    QString infoQ((char*) info.str().c_str());

    emit message(infoQ);

    obstacle1.set(response.x1, response.y1);
    obstacle2.set(response.x2, response.y2);

    QImage image = Mat2QImage(createMatrix());
    emit updateTableImage(image);

    return true;
}

bool BaseStation::findRobotPositionAndAngle(FindRobotPositionAndAngle::Request & request, FindRobotPositionAndAngle::Response & response) {
    int const AVERAGECOUNT = 5;
    int robotPositionAverageCount = 0;
    float positionX = 0.0f;
    float positionY = 0.0f;
    float robotAngle = 0.0f;

    kinectCapture->openCapture();
    for (int i = 0; i < AVERAGECOUNT; i++) {
        Mat depthMatrix = kinectCapture->captureDepthMatrix();
        Mat rgbMatrix = kinectCapture->captureRGBMatrix();
        if (!rgbMatrix.data || !depthMatrix.data) {
            return false;
        }

        robotDetection.findRobotWithAngle(depthMatrix, rgbMatrix);
        Vec2f robot = robotDetection.getRobotPosition();
        float angle = robotDetection.getRobotAngle();
        cout << angle << endl;
        if (robot[0] > 0.10 || robot[1] > 0.20) {
            positionX += robot[1] * 100;
            positionY += robot[0] * 100;
            robotAngle += angle;
            robotPositionAverageCount++;
        }
    }
    kinectCapture->closeCapture();

    if (robotPositionAverageCount > 0) {
        positionX /= robotPositionAverageCount;
        positionY /= robotPositionAverageCount;
        robotAngle /= robotPositionAverageCount;
        robotAngle = robotAngle * 180 / M_PI;
    }

    //Met a jour la position du robot dans l'interface
    if (positionX != 0 && positionY != 0) {
        actualPosition.set(positionX, positionY);
        kinoctoPositionUpdates.push_back(actualPosition);
    }

    QImage image = Mat2QImage(createMatrix());
    emit updateTableImage(image);

    response.x = positionX;
    response.y = positionY;
    response.angle = robotAngle;

    ROS_INFO( "%s x:%f y:%f angle:%f", "Request Find Robot Position. Sending Values ", positionX, positionY, robotAngle);

    stringstream info;
    info << "Kinocto : Demande de la position du robot \n";
    info << "Envoi de la position : ";
    info << " (" << positionX << "," << positionY << ")";
    info << "Envoi de l'angle : ";
    info << robotAngle;
    QString infoQ((char*) info.str().c_str());

    emit message(infoQ);

    return true;
}

bool BaseStation::showSolvedSudocube(ShowSolvedSudocube::Request & request, ShowSolvedSudocube::Response & response) {
    ROS_INFO("Show Solved sudocube");

    stringstream buff;
    buff << request.solvedSudocube;
    ROS_INFO( "%s\n red square value:%d\n solved sudocube:\n%s", "Show Solved Sudocube", request.redCaseValue, buff.str().c_str());

    emit showSolvedSudocubeSignal(QString(buff.str().c_str()), request.redCaseValue, request.redCasePosition);

    return true;
}

bool BaseStation::traceRealTrajectory(TraceRealTrajectory::Request & request, TraceRealTrajectory::Response & response) {
    ROS_INFO("Tracing Tracjectory");
    if (request.y.size() != request.x.size()) {
        ROS_ERROR("THE TRACJECTORY IS NOT WELL FORMATTED");

        return false;
    }

    stringstream buff;

    if (request.x.size() > 0) {
        plannedPath.clear();
        for (int i = 0; i < request.x.size(); i++) {
            buff << "(" << request.x[i] << "," << request.y[i] << ")" << endl;
            Position position(request.x[i], request.y[i]);
            plannedPath.push_back(position);
        }
    }

    while (!positionsForWhenThatDamnKinectDoesntReturnADamnPosition.empty()) {
        positionsForWhenThatDamnKinectDoesntReturnADamnPosition.pop();
    }

    for(int i = plannedPath.size(); i > 0; i--) {
        positionsForWhenThatDamnKinectDoesntReturnADamnPosition.push(plannedPath[i]);
    }

    ROS_INFO("%s %s", "Points of the trajectory :\n", buff.str().c_str());

    QString infoQ("Kinocto : Points de la trajectoire : \n");
    infoQ.append((char*) buff.str().c_str());

    emit message(infoQ);

    QImage image = Mat2QImage(createMatrix());
    emit updateTableImage(image);

    return true;
}

bool BaseStation::loopEnded(LoopEnded::Request & request, LoopEnded::Response & response) {
    ROS_INFO("Show Loop Ended Message");

    if (plannedPath.size() > 0) {
        plannedPath.clear();
    }
    if (kinoctoPositionUpdates.size() > 0) {
        kinoctoPositionUpdates.clear();
    }
    actualPosition.set(0, 0);

    emit endLoop("Kinocto : Loop Ended");

    return true;
}

void BaseStation::updateShizzle() {
    int const AVERAGECOUNT = 1;
    int robotPositionAverageCount = 0;
    float positionX = 0.0f;
    float positionY = 0.0f;
    float robotAngle = 0.0f;
    cout << "UPDATE ROBOT POS" << endl;
    kinectCapture->openCapture();
    for (int i = 0; i < AVERAGECOUNT; i++) {
        Mat depthMatrix = kinectCapture->captureDepthMatrix();
        Mat rgbMatrix = kinectCapture->captureRGBMatrix();
        if (!rgbMatrix.data || !depthMatrix.data) {
            return;
        }

        robotDetection.findRobotWithAngle(depthMatrix, rgbMatrix);
        Vec2f robot = robotDetection.getRobotPosition();
        float angle = robotDetection.getRobotAngle();
        cout << angle << endl;
        cout << robot << endl;
        if (robot[0] > 0.10 || robot[1] > 0.20) {
            positionX += robot[1] * 100;
            positionY += robot[0] * 100;
            robotAngle += angle;
            robotPositionAverageCount++;
        }
    }
    kinectCapture->closeCapture();

    if (robotPositionAverageCount > 0) {
        positionX /= robotPositionAverageCount;
        positionY /= robotPositionAverageCount;
        robotAngle /= robotPositionAverageCount;
        robotAngle = robotAngle * 180 / M_PI;
    }

    //Met a jour la position du robot dans l'interface
    if (positionX != 0 && positionY != 0) {
        actualPosition.set(positionX, positionY);
        kinoctoPositionUpdates.push_back(actualPosition);
    } else {
        actualPosition.set(positionsForWhenThatDamnKinectDoesntReturnADamnPosition.top().x, positionsForWhenThatDamnKinectDoesntReturnADamnPosition.top().y);
        kinoctoPositionUpdates.push_back(actualPosition);
    }
    positionsForWhenThatDamnKinectDoesntReturnADamnPosition.pop();

    QImage image = Mat2QImage(createMatrix());
    emit updateTableImage(image);

    ROS_INFO( "%s x:%f y:%f", "UPDATING Robot Position. Sending Values ", positionX, positionY);
    stringstream info;
    info << "Kinocto : Update de la position du robot \n";
    info << " (" << positionX << "," << positionY << ")";
    QString infoQ((char*) info.str().c_str());

    emit message(infoQ);
}

void BaseStation::updateRobotPosition(const basestation::UpdateRobotPos& str) {
    isUpdatingShit = true;
}

Mat3b BaseStation::createMatrix() {
    Mat3b tableWorkspace = Mat(Workspace::TABLE_X + 1, Workspace::TABLE_Y + 1, CV_8UC3, white);

    //GREEN SQUARE
    for (int i = 22; i <= 25; i++) {
        for (int j = 22; j <= 88; j++) {
            colorPixel(tableWorkspace, green, i, j);
            colorPixel(tableWorkspace, green, j, i);
        }
    }
    for (int i = 85; i <= 88; i++) {
        for (int j = 22; j <= 88; j++) {
            colorPixel(tableWorkspace, green, i, j);
            colorPixel(tableWorkspace, green, j, i);
        }
    }

    //REDLINE
    for (int i = 150; i <= 152; i++) {
        for (int j = 0; j <= Workspace::TABLE_Y; j++) {
            colorPixel(tableWorkspace, red, i, j);
        }
    }

    //OBSTACLE 1
    if (obstacle1.x != 0 && obstacle1.y != 0) {
        for (int y = (obstacle1.y - Workspace::OBSTACLE_RADIUS); y <= (obstacle1.y + Workspace::OBSTACLE_RADIUS); y++) {
            for (int x = (obstacle1.x - Workspace::OBSTACLE_RADIUS); x <= (obstacle1.x + Workspace::OBSTACLE_RADIUS); x++) {
                colorPixel(tableWorkspace, black, x, Workspace::TABLE_Y - y);
            }
        }
    }

    //OBSTACLE 2
    if (obstacle2.x != 0 && obstacle2.y != 0) {
        for (int y = (obstacle2.y - Workspace::OBSTACLE_RADIUS); y <= (obstacle2.y + Workspace::OBSTACLE_RADIUS); y++) {
            for (int x = (obstacle2.x - Workspace::OBSTACLE_RADIUS); x <= (obstacle2.x + Workspace::OBSTACLE_RADIUS); x++) {
                colorPixel(tableWorkspace, black, x, Workspace::TABLE_Y - y);
            }
        }
    }

    //Drawing actual Kinocto position
    if (actualPosition.x != 0 && actualPosition.y != 0) {
        for (int y = (actualPosition.y - 3); y <= (actualPosition.y + 3); y++) {
            for (int x = (actualPosition.x - 3); x <= (actualPosition.x + 3); x++) {
                colorPixel(tableWorkspace, red, x, Workspace::TABLE_Y - y);
            }
        }
    }

    transpose(tableWorkspace, tableWorkspace);

    //Drawing kinoctoPositionUpdates
    if (kinoctoPositionUpdates.size() > 0) {
        for (unsigned int i = 0; i < kinoctoPositionUpdates.size() - 1; i++) {
            Point currentPoint(kinoctoPositionUpdates[i].x, Workspace::TABLE_Y - kinoctoPositionUpdates[i].y);
            Point nextPoint(kinoctoPositionUpdates[i + 1].x, Workspace::TABLE_Y - kinoctoPositionUpdates[i + 1].y);
            drawLine(tableWorkspace, currentPoint, nextPoint, darkRed);
        }
    }

    //Drawing plannedPath
    if (plannedPath.size() > 0) {
        for (unsigned int i = 0; i < plannedPath.size() - 1; i++) {
            Point currentPoint(plannedPath[i].x, Workspace::TABLE_Y - plannedPath[i].y);
            Point nextPoint(plannedPath[i + 1].x, Workspace::TABLE_Y - plannedPath[i + 1].y);
            drawLine(tableWorkspace, currentPoint, nextPoint, blue);
        }
    }

    return tableWorkspace;
}

QImage BaseStation::Mat2QImage(const Mat3b &src) {
    QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
    for (int y = 0; y < src.rows; ++y) {
        const cv::Vec3b *srcrow = src[y];
        QRgb *destrow = (QRgb*) dest.scanLine(y);
        for (int x = 0; x < src.cols; ++x) {
            destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
        }
    }
    return dest;
}

void BaseStation::colorPixel(Mat &mat, Scalar color, int x, int y) {
    mat.at<cv::Vec3b>(x, y)[0] = color[0];
    mat.at<cv::Vec3b>(x, y)[1] = color[1];
    mat.at<cv::Vec3b>(x, y)[2] = color[2];
}

void BaseStation::drawLine(Mat img, Point start, Point end, Scalar color) {
    int thickness = 1;
    int lineType = 8;
    line(img, start, end, color, thickness, lineType);
}
