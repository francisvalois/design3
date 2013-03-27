/* 
 * File:   Vision.h
 * Author: immou4
 *
 * Created on March 14, 2013, 2:21 PM
 */

#ifndef VISION_H
#define	VISION_H

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "Zhang.h"
#include <vector>
#include <string>
#include <iostream>
#include "CONSTANTS.h"

class Vision {
public: 
        Vision();
	virtual ~Vision();	
	IplImage* mImage;
	int getCoinOrange(CvPoint* Gauche, CvPoint* Droit);
        int getCoinbleu(CvPoint* Gauche, CvPoint* Droit);
        void setZhangParameters(int inIndexFile);
	float getAngle( int index);
        Zhang* zhang;
private:
    int IndexParam;
};

#endif	/* VISION_H */

