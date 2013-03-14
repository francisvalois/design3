/* 
 * File:   Kinocto.h
 * Author: immou4
 *
 * Created on March 10, 2013, 2:40 PM
 */

#ifndef KINOCTO_H
#define	KINOCTO_H

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "Kinocto.h"
#include "Vision.h"
#include "Camera.h"
#include "CONSTANTS.h"
#include <iostream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

class Kinocto {
public:
	Kinocto();
	virtual ~Kinocto();
	int InitCapture();
	void ReleaseCapture();
	float GetAngleRobot();
	void PrendreImage();

	CvCapture* capture;
	Vision* mVision;
private:

	Camera* mCamera;
};

#endif	/* KINOCTO_H */

