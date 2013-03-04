// Zhang.h
//
// Loic Tanon 21/02/2012
//////////////////////////////////////////////////////////////////////
#if !defined(AFX_ZHANG_H__8B10705D_BAD0_41E7_8F03_58A8586C86BA__INCLUDED_)
#define AFX_ZHANG_H__8B10705D_BAD0_41E7_8F03_58A8586C86BA__INCLUDED_



#if _MSC_VER > 1000
#pragma once
#endif 

#define FICHIER_PARAMS_LU  0
#define FICHIER_PARAMS_NON_LU 1

#include <vector>
#include "CameraModel.h"
#include <string>
#include<fstream>
using namespace std;

class Zhang  
{
public:
	Zhang();
	virtual ~Zhang();
	int SetParameters(int inIndexFile);
	int Apply(double u, double v, double &x, double &y, double &z);
private:
	CameraModel* camera;
	int ErrorIDZHANG;
};

#endif 
