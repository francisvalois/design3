#if !defined(AFX_DISTORSIONCONVERTER_H__1CC9A098_DA68_4733_AF78_ADA09A2265AF__INCLUDED_)
#define AFX_DISTORSIONCONVERTER_H__1CC9A098_DA68_4733_AF78_ADA09A2265AF__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class DistorsionConverter  
{
public:
	DistorsionConverter();
	virtual ~DistorsionConverter();

   int AddRadialDistortion(double k1, double k2, const double Xu, const double Yu, double &Xd, double &Yd);
   int RemoveRadialDistortion(double k1, double k2, const double Xd, const double Yd, double &Xu, double &Yu, double Precision=0.0);


private:

   double CalculateRuMax(double k1, double k2);
};

#endif // !defined(AFX_DISTORSIONCONVERTER_H__1CC9A098_DA68_4733_AF78_ADA09A2265AF__INCLUDED_)
