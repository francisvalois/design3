//
// Created by francisvalois on 2013-02-28.
//
// To change the template use AppCode | Preferences | File Templates.
//


#include "Utility.h"

void Utility::saveToFile(Mat matrix, string fileName){
    FileStorage fs(fileName, FileStorage::WRITE);
    fs << "cameraMatrix" << matrix;

    fs.release();
}

Mat Utility::readFromFile(string fileName){
    FileStorage f;
    Mat matrix;
    f.open(fileName, FileStorage::READ);
    f["cameraMatrix"] >> matrix;
    f.release();

    return matrix;
}
