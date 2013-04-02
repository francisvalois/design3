#include "BaseStation.hpp"

using namespace basestation;
using namespace std;
using namespace boost;
using namespace cv;

BaseStation::BaseStation(int argc, char** argv) :
        init_argc(argc), init_argv(argv) {

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
    startLoopPublisher = node.advertise<std_msgs::String>("kinocto/startLoop", 1);

    findObstaclesPositionService = node.advertiseService("basestation/findObstaclesPosition", &BaseStation::findObstaclesPosition, this);
    findRobotPositionService = node.advertiseService("basestation/findRobotPosition", &BaseStation::findRobotPosition, this);
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

bool BaseStation::findObstaclesPosition(FindObstaclesPosition::Request & request, FindObstaclesPosition::Response & response) {
    int const AVERAGECOUNT = 3;
    int obstacle1AverageCount = 0;
    int obstacle2AverageCount = 0;
    
    //Average the measure for a better precision when the kinect is doing obscure things
    /*TODO: Check with Philippe if response is always 0 when that function is being called and clear that initializer if it's
    the case*/
    
    /*TODO: Check if there is a problem with creating the capture instance everytimes instead of storing it in a singleton 
     and check if there is a Kinect Frame buffer that could create a error when getting robot position because we are X frames
     behind*/
    response.x1 = 0;
    response.y1 = 0;
    response.x2 = 0;
    response.y2 = 0;    
    
    for(int i  = 0; i < AVERAGECOUNT; i++){
        Mat depthMatrix = kinectCapture.captureDepthMatrix();
        if (!depthMatrix.data) {
            return false;
        }
        
        obstaclesDetection.findCenteredObstacle(depthMatrix);
        Vec2f obs1 = obstaclesDetection.getObstacle1();
        Vec2f obs2 = obstaclesDetection.getObstacle2();
        
        if(obs1[0] > 0.10 || obs1[1] > 0.20){
            response.x1 += obs1[1] * 100;
            response.y1 += obs1[0] * 100;
            obstacle1AverageCount++;
        }
        
        if(obs2[0] > 0.10 || obs2[1] > 0.20){
            response.x2 += obs2[1] * 100;
            response.y2 += obs2[0] * 100;
            obstacle2AverageCount++;
        }       
    }
    
    if(obstacle1AverageCount > 0){
        response.x1 /= obstacle1AverageCount;
        response.y1 /= obstacle1AverageCount;
    }
    
    if(obstacle2AverageCount > 0){
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
    emit updateObstaclesPositions(response.x1, response.y1, response.x2, response.y2);

    return true;
}

bool BaseStation::findRobotPosition(FindRobotPosition::Request & request, FindRobotPosition::Response & response) {
    int const AVERAGECOUNT = 3;
    int robotPositionAverageCount;
    
    response.x = 0;
    response.y = 0;
    
    //Average the measure for a better precision when the kinect is doing obscure things
    /*TODO: Check with Philippe if response is always 0 when that function is being called and clear that initializer if it's
     the case*/
    
    /*TODO: Check if there is a lag when creating the capture instance everytimes instead of storing it in a singleton
     and check if there is a Kinect Frame buffer that could create a error when getting robot position because we are X frames
     behind*/
    
    for(int i  = 0; i < AVERAGECOUNT; i++){
        Mat depthMatrix = kinectCapture.captureDepthMatrix();
        Mat rgbMatrix = kinectCapture.captureRGBMatrix();
        if (!rgbMatrix.data || !depthMatrix.data) {
            return false;
        }
        
        robotDetection.findRobotWithAngle(depthMatrix, rgbMatrix);
        Vec2f robot = robotDetection.getRobotPosition(); //TEST TEMPORAIRE
        
        if(robot[0] > 0.10 || robot[1] > 0.20){
            response.x += robot[1] * 100;
            response.y += robot[0] * 100;
            robotPositionAverageCount++;
        }
    }
    
    
    response.x = 36.5; // TODO À supprimer quand la kinect fonctionnera
    response.y = 79.0;

    ROS_INFO( "%s x:%f y:%f", "Request Find Robot Position. Sending Values ", response.x, response.y);

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
    vector<Position> positions;
    for (int i = 0; i < request.x.size(); i++) {
        buff << "(" << request.x[i] << "," << request.y[i] << ")" << endl;
        Position position(request.x[i],request.y[i]);
        positions.push_back(position);
    }

    //TODO Afficher les points dans l'interface graphique

    ROS_INFO("%s %s", "Points of the trajectory :\n", buff.str().c_str());

    QString infoQ("Kinocto : Points de la trajectoire : \n");
    infoQ.append((char*) buff.str().c_str());

    emit message(infoQ);
    emit traceRealTrajectorySignal(positions);

    return true;
}

bool BaseStation::loopEnded(LoopEnded::Request & request, LoopEnded::Response & response) {
    ROS_INFO("Show Loop Ended Message");

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
    emit UpdatingRobotPositionSignal(request.x, request.y);

    return true;
}
