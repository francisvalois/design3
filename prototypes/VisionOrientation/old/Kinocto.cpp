/* 
 * File:   Kinocto.cpp
 * Author: immou4
 * 
 * Created on March 10, 2013, 2:40 PM
 */

#include "Kinocto.h"

bool VTri(int i, int j) {
	return (i < j);
}

Kinocto::Kinocto() {

	mVision = new Vision();
	mCamera = new Camera();
}

Kinocto::~Kinocto() {

	ReleaseCapture();
}

int Kinocto::InitCapture() {

	if (!(capture = cvCaptureFromCAM(-1)))
		printf("Ne peut pas initialiser la caméra\n");

	cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, 50);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 1600);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 1200);

	return 0;
}

void Kinocto::ReleaseCapture() {
	cvReleaseCapture(&capture);
}

void Kinocto::PrendreImage() {
	cvGrabFrame(capture);
	cvGrabFrame(capture);
	cvGrabFrame(capture);
	cvGrabFrame(capture);
	mVision->mImage = cvQueryFrame(capture);
	cvSaveImage("/home/design3/Images/ImagePrise.jpg", mVision->mImage);
}

float Kinocto::GetAngleRobot() {
	cout << "Avoir l angle du robot " << endl;

	int i;
	float tabAngle[12] = { 0 };
	vector<float> VAngle;
	Camera* CamPositionCourante;

	mCamera->SetPositionCourante(1, 1300, 2, 1200);
	i = mCamera->getindexOfParamsCalibFile();

	CamPositionCourante = mCamera->getPositionCourante();

	PrendreImage();
	while (CamPositionCourante->getHoriz() < 1700) {
		PrendreImage();
		i = mCamera->getindexOfParamsCalibFile();
		float temp = mVision->getAngle(i);
		if (temp != -1)
			tabAngle[i] = temp;
		CamPositionCourante = mCamera->getPositionCourante();
		mCamera->SetPositionCourante(1,
				CamPositionCourante->getHoriz() + 200, 2, 1200);

	}

	cout << "Angles trouves : " << endl;

	VAngle.clear();
	for (int i = 0; i < 12; i++) {
		cout << tabAngle[i] << "   ";
		if (tabAngle[i] > 0)
			VAngle.push_back(tabAngle[i]);
	}
	cout << endl;
	if (VAngle.size() > 0) {
		sort(VAngle.begin(), VAngle.end(), VTri);
		int mil = VAngle.size();
		cout << "Angle = " << VAngle[mil / 2] << endl;

		cout << "Angle du robot " << endl;

		return VAngle[mil / 2];
	} else
		return -1;
}


