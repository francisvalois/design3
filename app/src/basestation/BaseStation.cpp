#include "BaseStation.h"

using namespace basestation;
using namespace std;
using namespace boost;

BaseStation::BaseStation() {
}

BaseStation::~BaseStation() {
}

void BaseStation::loop() {
    while (ros::ok()) {
        usleep(500);
        ros::spinOnce();
    }
}

bool BaseStation::findObstaclesPosition(FindObstaclesPosition::Request & request, FindObstaclesPosition::Response & response) {
    array<float, 2> posX; //TODO Hardcoded value
    posX[0] = 2.0;
    posX[1] = 20.0;

    array<float, 2> posY; //TODO Hardcoded value
    posY[0] = 24.0;
    posY[1] = 23.0;

    response.x = posX;
    response.y = posY;

    ROS_INFO( "%s x1:%f y1:%f  x2:%f y2:%f", "Request Find Obstacles Position. Sending values ", posX[0], posY[0], posX[1], posY[1]);

    return true;
}

bool BaseStation::findRobotPosition(FindRobotPosition::Request & request, FindRobotPosition::Response & response) {
    response.x = 20.0f; //TODO Hardcoded value
    response.y = 30.0f;

    ROS_INFO( "%s x:%f y:%f", "Request Find Robot Position. Sending Values ", response.x, response.y);

    return true;
}

bool BaseStation::showSolvedSudocube(ShowSolvedSudocube::Request & request, ShowSolvedSudocube::Response & response) {
    stringstream buff;
    buff << request.solvedSudocube;
    ROS_INFO( "%s\n red square value:%d\n solved sudocube:\n%s", "Show Solved Sudocube", request.redCaseValue, buff.str().c_str());

    return true;
}

bool BaseStation::traceRealTrajectory(TraceRealTrajectory::Request & request, TraceRealTrajectory::Response & response) {
    if (request.y.size() != request.x.size()) {
        cout << "THE TRACJECTORY IS NOT WELL FORMATTED" << endl;
        ROS_INFO( "%s", "Trace Real Trajectory. THE TRACJECTORY IS NOT WELL FORMATTED ");

        return false;
    }

    stringstream buff;
    for (int i = 0; i < request.x.size(); i++) {
        buff << "(" << request.x[i] << "," << request.y[i] << ")" << endl;
    }

    ROS_INFO("%s %s", "Trace Real Trajectory. Values :\n", buff.str().c_str());

    return true;
}

bool BaseStation::loopEnded(LoopEnded::Request & request, LoopEnded::Response & response) {
    ROS_INFO("%s", "LOOP JUST ENDED!!! CLEAN WORKSPACE");

    return true;
}

bool BaseStation::updateRobotPosition(UpdateRobotPosition::Request & request, UpdateRobotPosition::Response & response) {
    ROS_INFO( "%s\n x:%f\n y:\n%f", "Update Robot Position", request.x, request.y);

    return true;
}

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    ros::init(argc, argv, "basestation");
    ros::NodeHandle n;

    BaseStation baseStation;

    ROS_INFO("%s", "Creating services");
    ros::ServiceServer service = n.advertiseService("basestation/findObstaclesPosition", &BaseStation::findObstaclesPosition, &baseStation);
    ros::ServiceServer service2 = n.advertiseService("basestation/findRobotPosition", &BaseStation::findRobotPosition, &baseStation);
    ros::ServiceServer service3 = n.advertiseService("basestation/showSolvedSudocube", &BaseStation::showSolvedSudocube, &baseStation);
    ros::ServiceServer service4 = n.advertiseService("basestation/traceRealTrajectory", &BaseStation::traceRealTrajectory, &baseStation);
    ros::ServiceServer service5 = n.advertiseService("basestation/updateRobotPosition", &BaseStation::updateRobotPosition, &baseStation);
    ros::ServiceServer service6 = n.advertiseService("basestation/loopEnded", &BaseStation::loopEnded, &baseStation);

    ROS_INFO("%s", "Basestation Initiated");

    cout << "DATA ARE GOING TO BE SEND" << endl;

    ros::Publisher startKinoctoPub = n.advertise<std_msgs::String>("kinocto/start", 1);
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();

    int count = 0;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ROS_INFO("%s", msg.data.c_str());

        startKinoctoPub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    cout << "DATA HAVE BEEN SENT" << endl;

    /*ros::ServiceClient client = n.serviceClient<kinocto::StartKinocto>("kinocto/start");

     kinocto::StartKinocto srv;
     if (client.call(srv)) {
     ROS_INFO("Received response from service StartKinocto");
     } else {
     ROS_ERROR("Failed to call service StartKinocto");
     return 1;
     }*/

    baseStation.loop();

    //ros::spin();
    return 0;
    //return a.exec();
}
