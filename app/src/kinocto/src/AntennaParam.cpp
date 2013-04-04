    #include "AntennaParam.h"

using namespace std;

const string AntennaParam::ORIENTATION_LETTER[] = { "N", "S", "E", "O" };

AntennaParam::AntennaParam() {
    big = false;
    number = 1;
    orientation = AntennaParam::ORIENTATION_NORTH;
}

AntennaParam::~AntennaParam() {
}

void AntennaParam::set(AntennaParam & antennaParam) {
    number = antennaParam.getNumber();
    big = antennaParam.isBig();
    orientation = antennaParam.getOrientation();
}

void AntennaParam::set(int number, bool isBig, int orientation) {
    this->number = number;
    this->big = isBig;
    this->orientation = orientation;
}

string AntennaParam::getOrientationLetter() {
    return ORIENTATION_LETTER[orientation - 1];
}

string AntennaParam::getIsBigLetter() {
    if (big == true) {
        return "G";
    }

    return "P";
}

int AntennaParam::getNumber() {
    return number;
}

bool AntennaParam::isBig() {
    return big;
}

int AntennaParam::getOrientation() {
    return orientation;
}

void AntennaParam::setOrientation(int orientation) {
    if (orientation >= 1 && orientation <= 4) {
        this->orientation = orientation;
    }
}

void AntennaParam::setNumber(int sudocubeNumber) {
    number = sudocubeNumber;
}

void AntennaParam::setIsBig(bool isBig) {
    big = isBig;
}
