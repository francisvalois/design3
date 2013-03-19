#ifndef MICROCONTROLLERDECORATOR_H_
#define MICROCONTROLLERDECORATOR_H_

#include "ros/ros.h"

#include "AntennaParam.h"
#include "Position.h"

#include "microcontroller/DecodeAntenna.h"
#include "microcontroller/DrawNumber.h"
#include "microcontroller/Move.h"
#include "microcontroller/PutPen.h"
#include "microcontroller/Rotate.h"
#include "microcontroller/TurnLED.h"
#include "microcontroller/WriteToLCD.h"
#include "microcontroller/RotateVerticallyCam.h"
#include "microcontroller/Translate.h"

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
    void writeToLCD(std::string message);
    void translate(Position position);
    void rotateVerticallyCam(int angle);

private:
    ros::NodeHandle nodeHandle;
};

#endif /* MICROCONTROLLERDECORATOR_H_ */
