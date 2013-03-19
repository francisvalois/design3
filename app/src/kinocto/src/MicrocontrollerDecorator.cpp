#include "MicrocontrollerDecorator.h"

MicrocontrollerDecorator::MicrocontrollerDecorator(ros::NodeHandle & nodeHandle) {
    this->nodeHandle = nodeHandle;
}

MicrocontrollerDecorator::~MicrocontrollerDecorator() {
}

AntennaParam MicrocontrollerDecorator::decodeAntenna() {
    ROS_INFO("Requesting the microcontroller to decode the antenna");

    AntennaParam antennaParam;
    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::DecodeAntenna>("microcontroller/decodeAntenna");
    microcontroller::DecodeAntenna srv;

    if (client.call(srv)) {
        ROS_INFO("The decoded information are number:%d isBig:%d orientation:%d", srv.response.number, srv.response.isBig, srv.response.orientation);

        antennaParam.isBig = srv.response.isBig;
        antennaParam.number = srv.response.number;
        antennaParam.orientation = srv.response.orientation;
    } else {
        ROS_ERROR("Failed to call service microcontroller/decodeAntenna");
    }

    return antennaParam;
}
void MicrocontrollerDecorator::drawNumber(int number, bool isBig) {
    ROS_INFO("Requesting the microcontroller to draw number:%d and isBig:%d", number, isBig);

    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::DrawNumber>("microcontroller/drawNumber");
    microcontroller::DrawNumber srv;
    srv.request.number = number;
    srv.request.isBig = isBig;

    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/drawNumber");
    }
}

void MicrocontrollerDecorator::move(float distance) {
    ROS_INFO("Requesting the microcontroller to move:%f", distance);

    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::Move>("microcontroller/move");
    microcontroller::Move srv;
    srv.request.distance = distance;

    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/move");
    }
}

void MicrocontrollerDecorator::rotate(float angle) {
    ROS_INFO("Requesting the microncontroller to rotate of :%f", angle);

    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::Rotate>("microcontroller/rotate");
    microcontroller::Rotate srv;
    srv.request.angle = angle;

    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/rotate");
    }
}

void MicrocontrollerDecorator::putPen(bool down) {
    ROS_INFO("Requesting the microcontroller to put the pen down?:%d", down);

    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::PutPen>("microcontroller/putPen");
    microcontroller::PutPen srv;
    srv.request.down = down;

    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/putPen");
    }
}

void MicrocontrollerDecorator::turnLED(bool on) {
    ROS_INFO("Requesting the microcontroller to turn the LED on?:%d", on);

    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::TurnLED>("microcontroller/turnLED");
    microcontroller::TurnLED srv;
    srv.request.on = on;

    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/turnLED");
    }
}

void MicrocontrollerDecorator::writeToLCD(std::string message) {
    ROS_INFO("Requesting the microcontroller to write on the LCD the message:%s", message.c_str());

    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::WriteToLCD>("microcontroller/writeToLCD");
    microcontroller::WriteToLCD srv;
    srv.request.message = message;

    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/writeToLCD");
    }
}

void MicrocontrollerDecorator::translate(Position position) {
    ROS_INFO("Requesting the microcontroller to translate the robot of x:%f y:%f", position.x, position.y);

    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::Translate>("microcontroller/translate");
    microcontroller::Translate srv;
    srv.request.x = position.x;
    srv.request.y = position.y;

    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/translate");
    }
}

void MicrocontrollerDecorator::rotateVerticallyCam(int angle) {
    ROS_INFO("Requesting the microcontroller to rotate webcam vertically of angle:%d", angle);

    ros::ServiceClient client = nodeHandle.serviceClient<microcontroller::RotateVerticallyCam>("microcontroller/rotateVerticallyCam");
    microcontroller::RotateVerticallyCam srv;
    srv.request.angle = angle;

    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/rotateVerticallyCam");
    }
}
