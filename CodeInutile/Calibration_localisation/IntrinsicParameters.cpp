

#include "IntrinsicParameters.h"
#include <math.h>
#include <float.h>


#ifndef PI
#define PI 3.141592653589793
#endif



IntrinsicParameters::IntrinsicParameters(){


   CameraType = TOSHIBA_1K_SM43H;

   CCDImageWidth  = TOSHIBA_SM43H_CCD_IMAGE_WIDTH;
   CCDImageHeight = TOSHIBA_SM43H_CCD_IMAGE_HEIGHT;

   ImageWidth  = 640;
   ImageHeight = 480;

   AlphaU   = 0.0;
   AlphaV   = 0.0;
   Gamma    = 1.0;
   u0       = 0.0;
   v0       = 0.0;

   f        = 0.0;
   Theta    = 0.0;
   Sx       = 0.0;
   k1       = 0.0;
   k2       = 0.0;


   IntrinsicParamMatrix.SetSize(3, 3);
   IntrinsicParamMatrixInv.SetSize(3, 3);
   SamplingTransformMatrix.SetSize(3, 3);
   SamplingTransformMatrixInv.SetSize(3, 3);
   ProjectionMatrix.SetSize(3, 3);
   ProjectionMatrixInv.SetSize(3, 3);

   IntrinsicParamMatrix.I();
   IntrinsicParamMatrixInv.I();
   SamplingTransformMatrix.I();
   SamplingTransformMatrixInv.I();
   ProjectionMatrix.I();
   ProjectionMatrixInv.I();

   IntrinsicParamUpToDate = 0;
   PixelSizeUpToDate = 0;

}

IntrinsicParameters::~IntrinsicParameters(){


}

void IntrinsicParameters::ComputeEffectivePixelSize(){

   if(ImageWidth > 0){
      PixelWidth  = CCDImageWidth /(double)ImageWidth;
   }
   if(ImageHeight > 0){
      PixelHeight = CCDImageHeight/(double)ImageHeight;
   }

   PixelSizeUpToDate = 1;
}


void IntrinsicParameters::ComputeIntrinsicParameters(){

   if(!PixelSizeUpToDate){
      ComputeEffectivePixelSize();
   }

   //Theta
   double TanTheta;
   if( (Gamma>0.0?Gamma:-Gamma) > DBL_EPSILON ){
      TanTheta = -AlphaU/Gamma;
   }else{
      TanTheta = 1.0e+300;
   }

   Theta = atan(TanTheta);
   if(Theta<0.0) Theta += PI;

   //f
   f = AlphaU*PixelWidth;

   //Sx
   if( ((AlphaV     >0.0?AlphaV     :-AlphaV)      > DBL_EPSILON ) &&
       ((PixelHeight>0.0?PixelHeight:-PixelHeight) > DBL_EPSILON ) &&
       ((Theta      >0.0?Theta      :-Theta)       > DBL_EPSILON ) ){

      Sx = (AlphaU/AlphaV)*(PixelWidth/PixelHeight)*(1.0/sin(Theta));
   }else{
      Sx = 1.0; //This case should never happen.
   }

   //Sx, expressed in pixel
   Sx = (Sx-1.0)*ImageWidth;

   IntrinsicParamMatrix(0,0) = AlphaU;
   IntrinsicParamMatrix(0,1) = Gamma;
   IntrinsicParamMatrix(0,2) = u0;
   IntrinsicParamMatrix(1,1) = AlphaV;
   IntrinsicParamMatrix(1,2) = v0;

   if( ((AlphaU>0.0?AlphaU:-AlphaU) > DBL_EPSILON ) &&
       ((AlphaV>0.0?AlphaV:-AlphaV) > DBL_EPSILON ) ){

      IntrinsicParamMatrixInv(0,0) = 1.0/AlphaU;
      IntrinsicParamMatrixInv(0,1) = -Gamma/(AlphaU*AlphaV);
      IntrinsicParamMatrixInv(0,2) = -(u0*AlphaV - v0*Gamma)/(AlphaU*AlphaV);
      IntrinsicParamMatrixInv(1,1) = 1.0/AlphaV;
      IntrinsicParamMatrixInv(1,2) = -v0/AlphaV;

      SamplingTransformMatrixInv(0,0) = f*IntrinsicParamMatrixInv(0,0);
      SamplingTransformMatrixInv(0,1) = f*IntrinsicParamMatrixInv(0,1);
      SamplingTransformMatrixInv(0,2) = f*IntrinsicParamMatrixInv(0,2);
      SamplingTransformMatrixInv(1,1) = f*IntrinsicParamMatrixInv(1,1);
      SamplingTransformMatrixInv(1,2) = f*IntrinsicParamMatrixInv(1,2);

   }else{

      IntrinsicParamMatrixInv.I();
      SamplingTransformMatrixInv.I();
   }


   if( (f>0.0?f:-f) > DBL_EPSILON ){

      SamplingTransformMatrix(0,0) = AlphaU/f;
      SamplingTransformMatrix(0,1) = Gamma/f;
      SamplingTransformMatrix(0,2) = u0;
      SamplingTransformMatrix(1,1) = AlphaV/f;
      SamplingTransformMatrix(1,2) = v0;

      ProjectionMatrix(2,2)    = 1.0/f;
      ProjectionMatrixInv(2,2) = f;

   }else{
      
      SamplingTransformMatrix.I();
      SamplingTransformMatrixInv.I();

      ProjectionMatrix.I();
      ProjectionMatrixInv.I();
   }
      
   IntrinsicParamUpToDate = 1;
}

void IntrinsicParameters::SetCameraType(int Type){

   switch(Type){
      case TOSHIBA_1K_SM43H : {
         CameraType = Type;
         CCDImageWidth  = TOSHIBA_SM43H_CCD_IMAGE_WIDTH;
         CCDImageHeight = TOSHIBA_SM43H_CCD_IMAGE_HEIGHT;
         break;
      }
      case PANASONIC_WW_CD50 : {
         CameraType = Type;
         CCDImageWidth  = PANASONIC_CD50_CCD_IMAGE_WIDTH;
         CCDImageHeight = PANASONIC_CD50_CCD_IMAGE_HEIGHT;
         break;
      }
      default:{
         CameraType = CUSTOM_CAMERA;
      }
   }

   PixelSizeUpToDate = 0;
   IntrinsicParamUpToDate = 0;

}


void IntrinsicParameters::SetCCDImageWidth(double CCDIW){

   CCDImageWidth = CCDIW;
   PixelSizeUpToDate = 0;
   IntrinsicParamUpToDate = 0;
   
}

void IntrinsicParameters::SetCCDImageHeight(double CCDIH){

   CCDImageHeight = CCDIH;
   PixelSizeUpToDate = 0;
   IntrinsicParamUpToDate = 0;
   
}


void IntrinsicParameters::SetImageWidth(unsigned int IW){

   ImageWidth = IW;
   PixelSizeUpToDate = 0;
   IntrinsicParamUpToDate = 0;

}
void IntrinsicParameters::SetImageHeight(unsigned int IH){

   ImageHeight = IH;
   PixelSizeUpToDate = 0;
   IntrinsicParamUpToDate = 0;

}

void IntrinsicParameters::SetU0(double NewU0){

   u0 = NewU0;
   IntrinsicParamUpToDate = 0;

}
void IntrinsicParameters::SetV0(double NewV0){

   v0 = NewV0;
   IntrinsicParamUpToDate = 0;

}

void IntrinsicParameters::SetAlphaU(double NewAlphaU){

   AlphaU = NewAlphaU;
   IntrinsicParamUpToDate = 0;

}

void IntrinsicParameters::SetAlphaV(double NewAlphaV){

   AlphaV = NewAlphaV;
   IntrinsicParamUpToDate = 0;

}

void IntrinsicParameters::SetGamma(double NewGamma){

   Gamma = NewGamma;
   IntrinsicParamUpToDate = 0;

}

void IntrinsicParameters::SetK1(double NewK1){

   k1 = NewK1;

}

void IntrinsicParameters::SetK2(double NewK2){

   k2 = NewK2;

}

double IntrinsicParameters::GetF(){

   if(!IntrinsicParamUpToDate){
      ComputeIntrinsicParameters();
   }
   return f;

}

double IntrinsicParameters::GetTheta(){

   if(!IntrinsicParamUpToDate){
      ComputeIntrinsicParameters();
   }
   return Theta;

}

double IntrinsicParameters::GetSx(){

   if(!IntrinsicParamUpToDate){
      ComputeIntrinsicParameters();
   }
   return Sx;

}

double IntrinsicParameters::GetPixelWidth(){

   if(!PixelSizeUpToDate){
      ComputeEffectivePixelSize();
   }
   return PixelWidth;

}
double IntrinsicParameters::GetPixelHeight(){

   if(!PixelSizeUpToDate){
      ComputeEffectivePixelSize();
   }
   return PixelHeight;
}



Matrix &IntrinsicParameters::GetIntrinsicParamMatrix(){

   if(!IntrinsicParamUpToDate){
      ComputeIntrinsicParameters();
   }
   return IntrinsicParamMatrix;

}

Matrix &IntrinsicParameters::GetSamplingTransformMatrix(){

   if(!IntrinsicParamUpToDate){
      ComputeIntrinsicParameters();
   }
   return SamplingTransformMatrix;

}

Matrix &IntrinsicParameters::GetProjectionMatrix(){

   if(!IntrinsicParamUpToDate){
      ComputeIntrinsicParameters();
   }
   return ProjectionMatrix;

}


Matrix &IntrinsicParameters::GetIntrinsicParamMatrixInv(){

   if(!IntrinsicParamUpToDate){
      ComputeIntrinsicParameters();
   }
   return IntrinsicParamMatrixInv;

}

Matrix &IntrinsicParameters::GetSamplingTransformMatrixInv(){

   if(!IntrinsicParamUpToDate){
      ComputeIntrinsicParameters();
   }
   return SamplingTransformMatrixInv;

}

Matrix &IntrinsicParameters::GetProjectionMatrixInv(){

   if(!IntrinsicParamUpToDate){
      ComputeIntrinsicParameters();
   }
   return ProjectionMatrixInv;

}

