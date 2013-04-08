#ifndef MICROCONTROLLERDECORATOR_H_
#define MICROCONTROLLERDECORATOR_H_

#include "ros/ros.h"

#include "AntennaParam.h"
#include "pathPlanning/Position.h"

#include "microcontroller/DecodeAntenna.h"
#include "microcontroller/DrawNumber.h"
#include "microcontroller/Move.h"
#include "microcontroller/PutPen.h"
#include "microcontroller/Rotate.h"
#include "microcontroller/TurnLED.h"
#include "microcontroller/WriteToLCD.h"
#include "microcontroller/RotateCam.h"
#include "microcontroller/Translate.h"
#include "microcontroller/GetSonarXDistance.h"

class MicrocontrollerDecorator {
public:
    MicrocontrollerDecorator(ros::NodeHandle & nodeHandle);
    virtual ~MicrocontrollerDecorator();

    AntennaParam decodeAntenna();
    void drawNumber(int number, bool isBig);
    void move(float distance);
    void rotate(float angle);
    void putPen(bool down);
    void turnLED(bool on);
    void writeToLCD(AntennaParam & antennaParam);
    void translate(Position position);
    void rotateCam(int vAngle, int hAngle);
    float getSonarDistance(int sonarNo);

private:
    ros::NodeHandle nodeHandle;

    ros::ServiceClient decodeAntennaClient;
    ros::ServiceClient drawNumberClient;
    ros::ServiceClient moveClient;
    ros::ServiceClient putPenClient;
    ros::ServiceClient rotateClient;
    ros::ServiceClient turnLEDClient;
    ros::ServiceClient writeToLCDClient;
    ros::ServiceClient rotateCamClient;
    ros::ServiceClient translateClient;
    ros::ServiceClient sonarXDistanceClient;
};

#endif /* MICROCONTROLLERDECORATOR_H_ */
