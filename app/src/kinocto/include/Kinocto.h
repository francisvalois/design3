#ifndef KINOCTO_H
#define KINOCTO_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "kinocto/StartKinocto.h"

#define INITIATED 1
#define START_LOOP 2

class Kinocto {

private:
	int state;
	ros::Timer loopTimer;

	void loop();
	void chatterCallback(const std_msgs::String::ConstPtr& msg);
public:
	Kinocto();
	~Kinocto();
	void start();

	bool startLoop(kinocto::StartKinocto::Request & request, kinocto::StartKinocto::Response & response);
};

#endif
