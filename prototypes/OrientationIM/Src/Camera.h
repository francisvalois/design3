#ifndef CAMERA_H_
#define CAMERA_H_

#include "Camera.h"
#include <time.h>
#include <string>

using namespace std;

class Camera {
public:
	Camera();
	virtual ~Camera();
	void setVert( float inVert);
	void setHoriz( float inHorizontal);
	float getVert( );
	float getHoriz();
	void SetPositionCourante(int NumServoHorizontal, int PosHorizontal,int NumServoVertical,int PosVertical);
	int getIndexOfParamsCalibFile();
	Camera* PositionCourante;
	Camera* getPositionCourante();

private:
	float Vert;
	float Horiz;

};

#endif 