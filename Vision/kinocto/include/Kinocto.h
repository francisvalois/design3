#ifndef KINOCTO_H
#define KINOCTO_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#define INITIATED 1
#define START_LOOP 2


class Kinocto {

private :
    int state;
    
    void loop();
public :
    Kinocto();
};

#endif
