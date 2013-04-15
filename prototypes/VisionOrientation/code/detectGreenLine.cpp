void Detector::detectGreenLine(Mat & srcHSV, vector<Vec2f> & greenLines) {

	Mat blur;
	Size sf;
	sf.width = 11;
	sf.height = 11;
	double sigmaX = 1;

	double d[] = {5.3049382516541385e-02, -8.3096662051120498e-02,-1.1345776472333211e-03, 2.5208106546648732e-03,-1.2073151061566005e-01};
	double m[3][3] = {{1.3225939376373308e+03, 0., 7.8950053275576806e+02},{0.,1.3197235387739179e+03, 5.2292007793085895e+02},{0., 0., 1.}};

	Mat undistorted;
	Mat intrinsic = Mat(3,3,CV_64F, m);
	Mat distMat = Mat(1,5, CV_64F, d);

	undistort(srcHSV, undistorted, intrinsic, distMat);
	cvtColor(undistorted, srcHSV, CV_RGB2HSV);
	GaussianBlur(srcHSV, blur, sf, sigmaX);

	Mat segmentedFrame;
	inRange(blur, Scalar(30, 30, 0), Scalar(80, 255, 255), segmentedFrame);

	int size = 7;
	Point erodePoint(size, size);
	Mat erodeElem = getStructuringElement(MORPH_ELLIPSE, Size(2 * size + 1, 2 * size + 1), erodePoint);
	erode(segmentedFrame, segmentedFrame, erodeElem);

	int size2 = 12;
	Point dilatePoint(size2, size2);
	Mat dilateElem = getStructuringElement(MORPH_ELLIPSE, Size(2 * size2 + 1, 2 * size2 + 1), dilatePoint);
	dilate(segmentedFrame, segmentedFrame, dilateElem);

	int size3 = 2;
	Point erodePoint2(size3, size3);
	Mat erodeElem2 = getStructuringElement(MORPH_ELLIPSE, Size(2 * size3 + 1, 2 * size3 + 1), erodePoint2);
	erode(segmentedFrame, segmentedFrame, erodeElem2);

	GaussianBlur(segmentedFrame, blur, sf, sigmaX);

	Mat edges;
	Canny(blur,edges,50, 200, 3);

	Mat cdst;
	cvtColor(blur, cdst, CV_GRAY2BGR);

	HoughLines(edges,greenLines, 1, CV_PI/150, 200, 0, 0);
}
