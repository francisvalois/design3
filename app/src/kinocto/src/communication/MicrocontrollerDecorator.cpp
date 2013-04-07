#include "communication/MicrocontrollerDecorator.h"

using namespace microcontroller;

MicrocontrollerDecorator::MicrocontrollerDecorator(ros::NodeHandle & nodeHandle) {
    this->nodeHandle = nodeHandle;

    decodeAntennaClient = nodeHandle.serviceClient<DecodeAntenna>("microcontroller/decodeAntenna");
    drawNumberClient = nodeHandle.serviceClient<DrawNumber>("microcontroller/drawNumber");
    moveClient = nodeHandle.serviceClient<Move>("microcontroller/move");
    putPenClient = nodeHandle.serviceClient<PutPen>("microcontroller/putPen");
    rotateClient = nodeHandle.serviceClient<Rotate>("microcontroller/rotate");
    turnLEDClient = nodeHandle.serviceClient<TurnLED>("microcontroller/turnLED");
    writeToLCDClient = nodeHandle.serviceClient<WriteToLCD>("microcontroller/writeToLCD");
    rotateCamClient = nodeHandle.serviceClient<RotateCam>("microcontroller/rotateCam");
    translateClient = nodeHandle.serviceClient<Translate>("microcontroller/translate");
    sonarXDistanceClient = nodeHandle.serviceClient<GetSonarXDistance>("microcontroller/getSonarXDistance");
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

    if (distance < -0.5f || distance > 0.5f) { // TODO vérifier conditions
        if (moveClient.call(srv) == false) {
            ROS_ERROR("Failed to call service microcontroller/move");
        }
    }
}

void MicrocontrollerDecorator::rotate(float angle) {
    ROS_INFO("Requesting the microncontroller to rotate of :%f", angle);

    if (angle > 180) {
        angle = -(360 - angle);
    } else if (angle < -180) {
        angle = 360 + angle;
    }

    Rotate srv;
    srv.request.angle = angle;

    if (angle < -1.0f || angle > 1.0f) {
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

void MicrocontrollerDecorator::writeToLCD(AntennaParam & antennaParam) {
    ROS_INFO("Requesting the microcontroller to write the antennaParam to the LCD screen");

    WriteToLCD srv;
    srv.request.sudocubeNo = antennaParam.getNumber();
    srv.request.orientation = antennaParam.getOrientationLetter();
    srv.request.size = antennaParam.getIsBigLetter();

    if (writeToLCDClient.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/writeToLCD");
    }
}

void MicrocontrollerDecorator::translate(Position position) {
    ROS_INFO("Requesting the microcontroller to translate the robot of x:%f y:%f", position.x, position.y);

    Translate srv;
    srv.request.x = position.x;
    srv.request.y = position.y;

    if (((position.x < 0.1) && (position.x > -0.1)) && ((position.y < 0.1) && (position.y > -0.1))) {
        ROS_ERROR("Translation impossible à effectuer");
    } else {
        if (translateClient.call(srv) == false) {
            ROS_ERROR("Failed to call service microcontroller/translate");
        }
    }
}

void MicrocontrollerDecorator::rotateCam(int vAngle, int hAngle) {
    ROS_INFO("Requesting the microcontroller to rotate webcam vertically of vAngle:%d and hAngle:%d", vAngle, hAngle);

    RotateCam srv;
    srv.request.vAngle = vAngle;
    srv.request.hAngle = hAngle;

    if (rotateCamClient.call(srv) == false) {
        ROS_ERROR("Failed to call service microcontroller/rotateCam");
    }
}

float MicrocontrollerDecorator::getSonarDistance(int sonarNo) {
    GetSonarXDistance srv;
    srv.request.sonarNo = sonarNo;

    if (sonarXDistanceClient.call(srv) == true) {
        ROS_INFO("Requesting the microcontroller to send the distance of the sonar no:%d value%f", sonarNo, srv.response.distance);
        return srv.response.distance;
    } else {
        ROS_ERROR("Failed to call service microcontroller/getSonarXDistance");
    }

    return 0.0f;
}
