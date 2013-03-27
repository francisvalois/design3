#if !defined(AFX_INTRINSICPARAMETERS_H__7BC8B854_E539_4E9C_89E9_CE2C55DA31CF__INCLUDED_)
#define AFX_INTRINSICPARAMETERS_H__7BC8B854_E539_4E9C_89E9_CE2C55DA31CF__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif 

#include "Matrix.h"

#define TOSHIBA_SM43H_CCD_IMAGE_WIDTH   0.00365
#define TOSHIBA_SM43H_CCD_IMAGE_HEIGHT  0.00274
#define PANASONIC_CD50_CCD_IMAGE_WIDTH  0.0088
#define PANASONIC_CD50_CCD_IMAGE_HEIGHT 0.0066

#define CUSTOM_CAMERA     0
#define TOSHIBA_1K_SM43H  1
#define PANASONIC_WW_CD50 2



class IntrinsicParameters{

public:

	IntrinsicParameters();
	virtual ~IntrinsicParameters();

   void SetCameraType(int Type);

   void SetCCDImageWidth(double CCDIW);
   void SetCCDImageHeight(double CCDIH);

   void SetImageWidth(unsigned int IW);
   void SetImageHeight(unsigned int IH);

   void SetU0(double NewU0);
   void SetV0(double NewV0);
   void SetAlphaU(double NewAlphaU);
   void SetAlphaV(double NewAlphaV);
   void SetGamma(double NewGamma);
   void SetK1(double NewK1);
   void SetK2(double NewK2);

   int GetCameraType() { return CameraType; }

   double GetCCDImageWidth() { return CCDImageWidth;  }
   double GetCCDImageHeight(){ return CCDImageHeight; }

   unsigned int GetImageWidth()  { return ImageWidth;  }
   unsigned int GetImageHeight() { return ImageHeight; }

   double GetU0()     { return u0;    }
   double GetV0()     { return v0;    }
   double GetAlphaU() { return AlphaU;}
   double GetAlphaV() { return AlphaV;}
   double GetGamma()  { return Gamma; }
   double GetK1()     { return k1;    }
   double GetK2()     { return k2;    }

   double GetF();
   double GetTheta();
   double GetSx();

   double GetPixelWidth();
   double GetPixelHeight();

   Matrix &GetIntrinsicParamMatrix();
   Matrix &GetSamplingTransformMatrix();
   Matrix &GetProjectionMatrix();

   Matrix &GetIntrinsicParamMatrixInv();
   Matrix &GetSamplingTransformMatrixInv();
   Matrix &GetProjectionMatrixInv();

private:

   int CameraType;

   double CCDImageWidth;
   double CCDImageHeight;

   unsigned int ImageWidth;
   unsigned int ImageHeight;

   double PixelWidth;
   double PixelHeight;

   double Gamma;
   double AlphaU;
   double AlphaV;
   double u0;
   double v0;

   double f;
   double Theta;
   double Sx;
   double k1;
   double k2;

   Matrix IntrinsicParamMatrix;
   Matrix SamplingTransformMatrix;
   Matrix ProjectionMatrix;

   Matrix IntrinsicParamMatrixInv;
   Matrix SamplingTransformMatrixInv;
   Matrix ProjectionMatrixInv;

   int IntrinsicParamUpToDate;
   int PixelSizeUpToDate;
   
   void ComputeEffectivePixelSize();

   void ComputeIntrinsicParameters();

};

#endif 