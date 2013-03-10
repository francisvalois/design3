#include <QtGui/QApplication>
#include "MainWindow.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "kinocto/StartKinocto.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<kinocto::StartKinocto>("start_kinocto");

    kinocto::StartKinocto srv;
    if (client.call(srv)) {
        ROS_INFO("Received response from service StartKinocto");
    } else {
        ROS_ERROR("Failed to call service StartKinocto");
        return 1;
    }

    ros::spinOnce();

    return a.exec();
}

