#include "AntennaParam.h"

AntennaParam::AntennaParam() {
    big = false;
    number = 1;
    orientation = AntennaParam::ORIENTATION_NORTH;
}

AntennaParam::~AntennaParam() {
}

bool AntennaParam::isBig() {
    return big;
}

int AntennaParam::getNumber() {
    return number;
}

int AntennaParam::getOrientation() {
    return orientation;
}

