#include "AntennaParam.h"

using namespace std;

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

string AntennaParam::getOrientationLetter() {
    switch (orientation) {
    case 1:
        return "N";
        break;
    case 2:
        return "S";
        break;
    case 3:
        return "E";
        break;
    case 4:
        return "W";
        break;
    default:
        return "N";
    }
}

string AntennaParam::getIsBigLetter() {
    if (isBig == true) {
        return "B";
    }

   return "P";
}
