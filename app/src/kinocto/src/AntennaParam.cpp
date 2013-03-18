#include "AntennaParam.h"

AntennaParam::AntennaParam() {
    isBig = false;
    number = 1;
    orientation = AntennaParam::ORIENTATION_NORTH;
}

AntennaParam::~AntennaParam() {
}

void AntennaParam::set(int number, bool isBig, int orientation) {
    this->number = number;
    this->isBig = isBig;
    this->orientation = orientation;
}
