
#include "Camera.h"

Camera::Camera() :
	Vert(1000), Horiz(1000) {
	PositionCourante = new Camera();
	SetPositionCourante(1, 1500, 2, 1500);
}

Camera::~Camera() {
}

void Camera::setVert(float inVert) {
	Vert = inVert;
}
void Camera::setHoriz(float inHoriz) {
	Horiz = inHoriz;
}
float Camera::getVert(){
	return Vert;
}
float Camera::getHoriz(){
	return Horiz;
}

Camera* Camera::getPositionCourante() {
	return PositionCourante;
}

void Camera::SetPositionCourante(int NumServoHorizontal,int PosHorizontal, int NumServoVertical,int PosVertical) {
	
	//à ajouter
}

int Camera::getIndexOfParamsCalibFile() {
	int temp=1;
		 if (PositionCourante->getHoriz() == 1500 && PositionCourante->getVert() == 1200) {temp = 6;}
	else if (PositionCourante->getHoriz() == 1000 && PositionCourante->getVert() == 1200) {temp = 1;}
	else if (PositionCourante->getHoriz() == 1100 && PositionCourante->getVert() == 1200) {temp = 2;}
	else if (PositionCourante->getHoriz() == 1200 && PositionCourante->getVert() == 1200) {temp = 3;}
	else if (PositionCourante->getHoriz() == 1300 && PositionCourante->getVert() == 1200) {temp = 4;}
	else if (PositionCourante->getHoriz() == 1400 && PositionCourante->getVert() == 1200) {temp = 5;}
	else if (PositionCourante->getHoriz() == 1600 && PositionCourante->getVert() == 1200) {temp = 7;}
	else if (PositionCourante->getHoriz() == 1700 && PositionCourante->getVert() == 1200) {temp = 8;}
	else if (PositionCourante->getHoriz() == 1800 && PositionCourante->getVert() == 1200) {temp = 8;} 
	else if (PositionCourante->getHoriz() == 1900 && PositionCourante->getVert() == 1200) {temp = 10;} 
	else if (PositionCourante->getHoriz() == 2000 && PositionCourante->getVert() == 1200) {temp = 11;}

	return temp;
}


