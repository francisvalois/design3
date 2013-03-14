#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H


#include "IntrinsicParameters.h"
#include "ExtrinsicParameters.h"

#include "DistorsionConverter.h"


class CameraModel{

public:

	CameraModel();
	virtual ~CameraModel();

   void PixelDist2PixelUndist(const double Ud, const double Vd, double &Uu, double &Vu);

   void PixelDist2ImgPlaneUndist(const double Ud, const double Vd, double &Xu, double &Yu, double &Zu);


   IntrinsicParameters TheIntrinsicParam;
   ExtrinsicParameters TheExtrinsicParam;

   DistorsionConverter TheDistorsionConv;

};

#endif