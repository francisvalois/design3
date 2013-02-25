#ifndef REDLINE_H_
#define REDLINE_H_

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "Zhang/Zhang.h"
#include <vector>
#include <string>
#include <iostream>


class RedLine {
public:
	RedLine();
	virtual ~RedLine();
	void setZhangParameters(int inIndexFile);
	float getAngle( int index);

	IplImage* mImage;
	Zhang* zhang;

private:
	int IndexParam;
};

#endif 
