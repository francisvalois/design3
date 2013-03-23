#include "MicrocontrollerDecorator.h"

using namespace microcontroller;

MicrocontrollerDecorator::MicrocontrollerDecorator(ros::NodeHandle & nodeHandle) {
    this->nodeHandle = nodeHandle;

    decodeAntennaClient = nodeHandle.serviceClient<DecodeAntenna>("microcontroller/decodeAntenna");
    drawNumberClient = nodeHandle.serviceClient<DrawNumber>("microcontroller/drawNumber");
    moveClient = nodeHandle.serviceClient<Move>("microcontroller/move");
    putPenClient = nodeHandle.serviceClient<Rotate>("microcontroller/rotate");
    rotateClient = nodeHandle.serviceClient<PutPen>("microcontroller/putPen");
    turnLEDClient = nodeHandle.serviceClient<TurnLED>("microcontroller/turnLED");
    writeToLCDClient = nodeHandle.serviceClient<WriteToLCD>("microcontroller/writeToLCD");
    rotateVerticallyCamClient = nodeHandle.serviceClient<RotateVerticallyCam>("microcontroller/rotateVerticallyCam");
    translateClient = nodeHandle.serviceClient<Translate>("microcontroller/translate");
}

MicrocontrollerDecorator::~MicrocontrollerDecorator() {
}

AntennaParam MicrocontrollerDecorator::decodeAntenna() {
    ROS_INFO("Requesting the microcontroller to decode the antenna");

    AntennaParam antennaParam;
    DecodeAntenna srv;

    if (decodeAntennaClient.call(srv) == true) {
        ROS_INFO("The decoded information are number:%d isBig:%d orientation:%d", srv.response.number, srv.response.isBig, srv.response.orientation);
        antennaParam.set(srv.response.number, srv.response.isBig, srv.response.orientation);
    } else {
        ROS_ERROR("Failed to call service microcontroller/decodeAntenna");
    }

    return antennaParam;
}
void MicrocontrollerDecorator::drawNumber(int number, bool isBig) {
    ROS_INFO("Requesting the microcontroller to draw number:%d and isBig:%d", number, isBig);

    DrawNumber srv;
    srv.request.number = number;
    srv.request.isBig = isBig;

    if (drawNumberClient.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/drawNumber");
    }
}

void MicrocontrollerDecorator::move(float distance) {
    ROS_INFO("Requesting the microcontroller to move:%f", distance);

    Move srv;
    srv.request.distance = distance;

    if (distance < -1 || distance > 1) { // TODO vérifier conditions
        if (moveClient.call(srv) == false) {
            ROS_ERROR("Failed to call service microcontroller/move");
        }
    }
}

void MicrocontrollerDecorator::rotate(float angle) {
    ROS_INFO("Requesting the microncontroller to rotate of :%f", angle);

    Rotate srv;
    srv.request.angle = angle;

    if (angle < -1 || angle > 1) {
        if (rotateClient.call(srv) == false) {
            ROS_ERROR("Failed to call service microcontroller/rotate");
        }
    }

}

void MicrocontrollerDecorator::putPen(bool down) {
    ROS_INFO("Requesting the microcontroller to put the pen down?:%d", down);

    PutPen srv;
    srv.request.down = down;

    if (putPenClient.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/putPen");
    }
}

void MicrocontrollerDecorator::turnLED(bool on) {
    ROS_INFO("Requesting the microcontroller to turn the LED on?:%d", on);

    TurnLED srv;
    srv.request.on = on;

    if (turnLEDClient.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/turnLED");
    }
}

void MicrocontrollerDecorator::writeToLCD(std::string message) {
    ROS_INFO("Requesting the microcontroller to write on the LCD the message:%s", message.c_str());

    WriteToLCD srv;
    srv.request.message = message;

    if (writeToLCDClient.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/writeToLCD");
    }
}

void MicrocontrollerDecorator::translate(Position position) {
    ROS_INFO("Requesting the microcontroller to translate the robot of x:%f y:%f", position.x, position.y);

    Translate srv;
    srv.request.x = position.x;
    srv.request.y = position.y;

    if (translateClient.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/translate");
    }
}

void MicrocontrollerDecorator::rotateVerticallyCam(int angle) {
    ROS_INFO("Requesting the microcontroller to rotate webcam vertically of angle:%d", angle);

    RotateVerticallyCam srv;
    srv.request.angle = angle;

    if (rotateVerticallyCamClient.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/rotateVerticallyCam");
    }
}
