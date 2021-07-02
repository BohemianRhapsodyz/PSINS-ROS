#ifndef _KFAPP_H
#define _KFAPP_H

#include "PSINS.h"
class CSINSOD;
typedef struct {
	CVect3 wm, vm;
	double t;
	double odom;
} ImuGpsData;

class CSINSOD :public CSINSTDKF	// sizeof(CSINSGPSOD)~=30k bytes
{
public:
	CVect3 posDR, vnRes, posRes;
	CVect3 lvOD;
	CMat3 Cbo, MpkD;			// Cbo: from body-frame to OD-frame
	double Kod, tODInt;
	BOOL measGPSvnValid, measGPSposValid, measODValid, measMAGyawValid;

	CSINSOD(void);
	virtual void Init(const CSINS &sins0, int grade = -1);
	virtual void SetFt(int nnq);
	virtual void SetMeas(void);
	virtual void Feedback(double fbts);
	void SetMeasGPS(const CVect3 &pgps = O31, const CVect3 &vgps = O31);
	void SetMeasOD(double dSod, double ts);
	void SetMeasYaw(double ymag);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);
	int TDUpdate(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nStep);
};

CVect3 lla2enu(double latitude, double longtitude, double altitude, double latitude0, double longtitude0, double altitude0);
#endif


