#include "BaseStation.hpp"

using namespace basestation;
using namespace std;
using namespace boost;
using namespace cv;

BaseStation::BaseStation(int argc, char** argv) :
        init_argc(argc), init_argv(argv) {

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

    state = LOOP;

    init();
}

BaseStation::~BaseStation() {
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
    startLoopPublisher = node.advertise<std_msgs::String>("basestation/startLoop", 1);

    getObstaclesPositionService = node.advertiseService("basestation/getObstaclesPosition", &BaseStation::getObstaclesPosition, this);
    findRobotPositionAndAngleService = node.advertiseService("basestation/findRobotPositionAndAngle", &BaseStation::findRobotPositionAndAngle, this);
    showSolvedSudocubeService = node.advertiseService("basestation/showSolvedSudocube", &BaseStation::showSolvedSudocube, this);
    traceRealTrajectoryService = node.advertiseService("basestation/traceRealTrajectory", &BaseStation::traceRealTrajectory, this);
    updateRobotPositionService = node.advertiseService("basestation/updateRobotPosition", &BaseStation::updateRobotPosition, this);
    loopEndedService = node.advertiseService("basestation/loopEnded", &BaseStation::loopEnded, this);
    showConfirmStartRobotMessageService = node.advertiseService("basestation/showConfirmStartRobotMessage",
            &BaseStation::showConfirmStartRobotdMessage, this);
}

void BaseStation::loop() {
    if (ros::ok()) {
        switch (state) {
        case LOOP:
            cout << "looping" << endl;
            break;
        case SEND_START_LOOP_MESSAGE:
            sendStartLoopMessage();
            state = LOOP;
            break;
        }

        ros::spinOnce();
    }
}

void BaseStation::setStateToSendStartLoopMessage() {
    state = SEND_START_LOOP_MESSAGE;
}

void BaseStation::sendStartLoopMessage() {
    std_msgs::String msg;
    msg.data = "Démarrage de la loop";

    ROS_INFO("%s", msg.data.c_str());
    startLoopPublisher.publish(msg);
}

bool BaseStation::showConfirmStartRobotdMessage(ShowConfirmStartRobot::Request & request, ShowConfirmStartRobot::Response & response) {
    ROS_INFO("Showing Confirmation of Start Robot");
    state = LOOP;

    emit message("Kinocto : Start");

    return true;
}

bool BaseStation::getObstaclesPosition(GetObstaclesPosition::Request & request, GetObstaclesPosition::Response & response) {
    int const AVERAGECOUNT = 3;
    int obstacle1AverageCount = 0;
    int obstacle2AverageCount = 0;

    //Average the measure for a better precision when the kinect is doing obscure things

    kinectCapture.openCapture();
    for (int i = 0; i < AVERAGECOUNT; i++) {
        Mat depthMatrix = kinectCapture.captureDepthMatrix();
        if (!depthMatrix.data) {
            return false;
        }

        obstaclesDetection.findCenteredObstacle(depthMatrix);
        Vec2f obs1 = obstaclesDetection.getObstacle1();
        Vec2f obs2 = obstaclesDetection.getObstacle2();

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
    kinectCapture.closeCapture();

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

    //emit updateObstaclesPositions(response.x1, response.y1, response.x2, response.y2);

    obstacle1.set(response.x1,response.y1);
    obstacle2.set(response.x2,response.y2);

    QImage image = Mat2QImage(createMatrix());
    emit updateTableImage(image);

    return true;
}

bool BaseStation::findRobotPositionAndAngle(FindRobotPositionAndAngle::Request & request, FindRobotPositionAndAngle::Response & response) {
    int const AVERAGECOUNT = 3;
    int robotPositionAverageCount = 0;

    kinectCapture.openCapture();
    for (int i = 0; i < AVERAGECOUNT; i++) {
        Mat depthMatrix = kinectCapture.captureDepthMatrix();
        Mat rgbMatrix = kinectCapture.captureRGBMatrix();
        if (!rgbMatrix.data || !depthMatrix.data) {
            return false;
        }

        robotDetection.findRobotWithAngle(depthMatrix, rgbMatrix);
        Vec2f robot = robotDetection.getRobotPosition();
        float angle = robotDetection.getRobotAngle();
        if (robot[0] > 0.10 || robot[1] > 0.20) {
            response.x += robot[1] * 100;
            response.y += robot[0] * 100;
            response.angle += angle;
            robotPositionAverageCount++;
        }
    }
    kinectCapture.closeCapture();


    if (robotPositionAverageCount > 0) {
        response.x /= robotPositionAverageCount;
        response.y /= robotPositionAverageCount;
        response.angle /= robotPositionAverageCount;
        response.angle = response.angle * 180 / M_PI;
    }

    //TODO TEMPORAIRE PR TESTS
    response.x = 35.5;
    response.y = 76.5;
    response.angle = 0.0f;

    ROS_INFO( "%s x:%f y:%f angle:%f", "Request Find Robot Position. Sending Values ", response.x, response.y, response.angle);

    stringstream info;
    info << "Kinocto : Demande de la position du robot \n";
    info << "Envoi de la position : ";
    info << " (" << response.x << "," << response.y << ")";
    QString infoQ((char*) info.str().c_str());

    emit message(infoQ);
    emit UpdatingRobotPositionSignal(response.x, response.y);

    return true;
}

bool BaseStation::showSolvedSudocube(ShowSolvedSudocube::Request & request, ShowSolvedSudocube::Response & response) {
    ROS_INFO("Show Solved sudocube");

    stringstream buff;
    buff << request.solvedSudocube;
    ROS_INFO( "%s\n red square value:%d\n solved sudocube:\n%s", "Show Solved Sudocube", request.redCaseValue, buff.str().c_str());

    emit showSolvedSudocubeSignal(QString(buff.str().c_str()), request.redCaseValue);

    return true;
}

bool BaseStation::traceRealTrajectory(TraceRealTrajectory::Request & request, TraceRealTrajectory::Response & response) {
    ROS_INFO("Tracing Tracjectory");
    if (request.y.size() != request.x.size()) {
        ROS_ERROR("THE TRACJECTORY IS NOT WELL FORMATTED");

        return false;
    }

    stringstream buff;

    if(request.x.size() > 0) {
        plannedPath.clear();

        for (int i = 0; i < request.x.size(); i++) {
            buff << "(" << request.x[i] << "," << request.y[i] << ")" << endl;
            Position position(request.x[i],request.y[i]);
            plannedPath.push_back(position);
        }
    }

    ROS_INFO("%s %s", "Points of the trajectory :\n", buff.str().c_str());

    QString infoQ("Kinocto : Points de la trajectoire : \n");
    infoQ.append((char*) buff.str().c_str());

    emit message(infoQ);
    //emit traceRealTrajectorySignal(positions);

    QImage image = Mat2QImage(createMatrix());
    emit updateTableImage(image);

    return true;
}

bool BaseStation::loopEnded(LoopEnded::Request & request, LoopEnded::Response & response) {
    ROS_INFO("Show Loop Ended Message");

    if(plannedPath.size() > 0) {
        plannedPath.clear();
    }
    if(kinoctoPositionUpdates.size() > 0) {
        kinoctoPositionUpdates.clear();
    }

    emit message("Kinocto : Loop Ended");

    return true;
}

bool BaseStation::updateRobotPosition(UpdateRobotPosition::Request & request, UpdateRobotPosition::Response & response) {
    ROS_INFO( "%s\n x:%f\n y:%f", "Updating Robot Position", request.x, request.y);

    stringstream info;
    info << "Kinocto : Mise a jour de la position du robot :  \n";
    info << " (" << request.x << "," << request.y << ")";
    QString infoQ((char*) info.str().c_str());

    emit message(infoQ);

    //emit UpdatingRobotPositionSignal(request.x, request.y);

    actualPosition.set(request.x, request.y);
    kinoctoPositionUpdates.push_back(actualPosition);

    QImage image = Mat2QImage(createMatrix());
    emit updateTableImage(image);

    return true;
}

Mat3b BaseStation::createMatrix() {
    Mat3b tableWorkspace = Mat(Workspace::TABLE_X + 1, Workspace::TABLE_Y + 1, CV_8UC3, white);

    //OBSTACLE 1
    if (obstacle1.x != 0 && obstacle1.y != 0) {
        for (int y = (obstacle1.y - Workspace::OBSTACLE_RADIUS); y <= (obstacle1.y + Workspace::OBSTACLE_RADIUS); y++) {
            for (int x = (obstacle1.x - Workspace::OBSTACLE_RADIUS); x <= (obstacle1.x + Workspace::OBSTACLE_RADIUS); x++) {
                colorPixel(tableWorkspace, black, x, Workspace::TABLE_Y -  y);
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
    if(kinoctoPositionUpdates.size() > 0) {
        for(unsigned int i = 0; i < kinoctoPositionUpdates.size() - 1; i++) {
            Point currentPoint(kinoctoPositionUpdates[i].x, Workspace::TABLE_Y - kinoctoPositionUpdates[i].y);
            Point nextPoint(kinoctoPositionUpdates[i+1].x, Workspace::TABLE_Y - kinoctoPositionUpdates[i+1].y);
            drawLine(tableWorkspace, currentPoint, nextPoint, red);
        }
    }

    //Drawing plannedPath
    if(plannedPath.size() > 0) {
        for(unsigned int i = 0; i < plannedPath.size() - 1; i++) {
            Point currentPoint(plannedPath[i].x, Workspace::TABLE_Y - plannedPath[i].y);
            Point nextPoint(plannedPath[i+1].x, Workspace::TABLE_Y - plannedPath[i+1].y);
            drawLine(tableWorkspace, currentPoint, nextPoint, blue);
        }
    }

    return tableWorkspace;
}

QImage BaseStation::Mat2QImage(const Mat3b &src) {
    QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
    for (int y = 0; y < src.rows; ++y) {
            const cv::Vec3b *srcrow = src[y];
            QRgb *destrow = (QRgb*)dest.scanLine(y);
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
