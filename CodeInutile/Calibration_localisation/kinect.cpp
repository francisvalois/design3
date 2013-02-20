#include "kinect.h"


kinect::kinect()
{

}

kinect::~kinect()
{

}

////////////////////////////////////////////////////////////////////////////////
//  
//  (Ud, Vd) : Pixel coordinate of the distorted point
//  (Uu, Vu) : Pixel coordinate of the undistorted point
//  
////////////////////////////////////////////////////////////////////////////////
void kinect::PixelDist2PixelUndist(const double Ud, const double Vd, double &Uu, double &Vu){

   double k1 = TheIntrinsicParam.GetK1();
   double k2 = TheIntrinsicParam.GetK2();

   Matrix A    = TheIntrinsicParam.GetIntrinsicParamMatrix();
   Matrix AInv = TheIntrinsicParam.GetIntrinsicParamMatrixInv();
      
   Matrix DistImgPt(3,1);
   Matrix DistImgPlanePt(3,1);
   Matrix UndistImgPlanePt(3,1);
   Matrix UndistImgPt(3,1);

   DistImgPt(0,0) = Ud;
   DistImgPt(1,0) = Vd;
   DistImgPt(2,0) = 1.0;

   DistImgPlanePt = AInv*DistImgPt;

   TheDistorsionConv.RemoveRadialDistortion(k1, k2, DistImgPlanePt(0,0), DistImgPlanePt(1,0), UndistImgPlanePt(0,0), UndistImgPlanePt(1,0));

   UndistImgPlanePt(2,0) = 1.0;

   UndistImgPt = A*UndistImgPlanePt;

   Uu = UndistImgPt(0,0);
   Vu = UndistImgPt(1,0);

}

void kinect::PixelDist2ImgPlaneUndist(const double Ud, const double Vd, double &Xu, double &Yu, double &Zu){

   double k1 = TheIntrinsicParam.GetK1();
   double k2 = TheIntrinsicParam.GetK2();

   Matrix AInv = TheIntrinsicParam.GetIntrinsicParamMatrixInv();
      
   Matrix DistImgPt(3,1);
   Matrix DistImgPlanePt(3,1);

   DistImgPt(0,0) = Ud;
   DistImgPt(1,0) = Vd;
   DistImgPt(2,0) = 1.0;

   DistImgPlanePt = AInv*DistImgPt;

   TheDistorsionConv.RemoveRadialDistortion(k1, k2, DistImgPlanePt(0,0), DistImgPlanePt(1,0), Xu, Yu);
   //Xu = DistImgPlanePt(0,0);
   //Yu = DistImgPlanePt(1,0);

   Zu = 1.0;

}
