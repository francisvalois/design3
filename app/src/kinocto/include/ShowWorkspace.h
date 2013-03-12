#ifndef SHOWWORKSPACE_H_
#define SHOWWORKSPACE_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

const int tableX = 231 + 1;
const int tableY = 114 +1;

class ShowWorkspace {
public:
	ShowWorkspace();

	void showWorkspace(int[tableX][tableY]);

private:
	cv::Scalar white;
	cv::Scalar blue;
	cv::Scalar black;

	void showWindowWith(const char*, const cv::Mat &);
	void colorPixel(cv::Mat&, cv::Scalar, int, int);
};

#endif /* SHOWWORKSPACE_H_ */
