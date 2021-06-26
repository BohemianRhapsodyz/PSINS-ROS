#include "KFApp.h"
#include "PSINS.h"
/***************************  class CKFApp  *********************************/
//CKFApp::CKFApp(double ts) :CSINSTDKF(34, 6)
//{
//	lvGPS = O31;
//}
//
//void CKFApp::Init(const CSINS &sins0, int grade)
//{
//	CSINSKF::Init(sins0);
//	sins.lever(-lvGPS);
//	sins.pos = sins.posL;
//	Pk.SetDiag2(0.1*DEG, 0.1*DEG, 1.0*DEG, 1.0, 1.0, 1.0, 10.0 / RE, 10.0 / RE, 10.0,
//		0.01*DPH, 0.01*DPH, 0.01*DPH, 100.0*UG, 100.0*UG, 500.0*UG,
//		1.0, 1.0, 1.0, 0.001,
//		1000.0*PPM, 10.0*SEC, 10.0*SEC, 10.0*SEC, 1000.0*PPM, 10.0*SEC, 10.0*SEC, 10.0*SEC, 1000.0*PPM,
//		1000.0*PPM, 10.0*SEC, 10.0*SEC, 1000.0*PPM, 10.0*SEC, 10.0*PPM
//	);
//	Pmin.Set2(1.1*SEC, 1.1*SEC, 10.0*SEC, 0.001, 0.001, 0.001, 0.001 / RE, 0.001 / RE, 0.001,
//		0.001*DPH, 0.001*DPH, 0.001*DPH, 10.0*UG, 10.0*UG, 20.0*UG,
//		0.01, 0.01, 0.01, 0.0001,
//		0.0*PPM, 0.0*SEC, 0.0*SEC, 0.0*SEC, 0.0*PPM, 0.0*SEC, 0.0*SEC, 0.0*SEC, 0.0*PPM,
//		0.0*PPM, 0.0*SEC, 0.0*SEC, 0.0*PPM, 0.0*SEC, 0.0*PPM
//	);
//	Qt.Set2(0.01*glv.dpsh, 0.01*glv.dpsh, 0.01*glv.dpsh, 10.0*glv.ugpsHz, 10.0*glv.ugpsHz, 20.0*glv.ugpsHz, 0.0, 0.0, 0.0,
//		0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//		0.0, 0.0, 0.0, 0.0,
//		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//		0.0, 0.0, 0.0, 0.0, 0.0, 0.0
//	);
//	FBTau.Set(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//		1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
//		INF, INF, INF, INF,
//		INF, INF, INF, INF, INF, INF, INF, INF, INF,
//		INF, INF, INF, INF, INF, INF
//	);
//	Rt.Set2(0.10, 0.10, 0.10, .10 / RE, .10 / RE, .10);
//	Rmax = Rt * 100.;  Rmin = Rt * 0.01;  Rb = 0.9;
//	rts = 1.0;
//}
//
//void CKFApp::SetMeasGPS(const CVect3 &posGPS, const CVect3 &vnGPS, double dt)
//{
//	if (!IsZero(posGPS, EPS) && sins.wnb.k<30.0 * DPS && sins.wnb.k>-30.0 * DPS)
//	{
//		sins.lever(lvGPS);
//		if (!IsZero(vnGPS, EPS))
//		{
//			*(CVect3*)&Zk.dd[0] = sins.vnL - sins.an*dt - vnGPS;
//			SetMeasFlag(00007);
//		}
//		*(CVect3*)&Zk.dd[3] = sins.posL - sins.Mpv*sins.vn*dt - posGPS;
//		SetMeasFlag(00070);
//	}
//}
//
//int CKFApp::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
//{
//	int res = TDUpdate(pwm, pvm, nSamples, ts, 5);
//	CVect3 lv = (lvGPS + *(CVect3*)&Xk.dd[15]);
//	vnRes = sins.vn + sins.Cnb*askew(sins.web)*lv + sins.an*Xk.dd[18];
//	posRes = sins.pos + sins.MpvCnb*lv + sins.Mpv*sins.vn*Xk.dd[18];
//	xpt.att = sins.att; xpt.vn = vnRes; xpt.pos = posRes;
//	double *p = &xpt.Patt.i, *p1 = &Pk.dd[0];
//	for (int i = 0; i < 9; i++, p++, p1 += nq + 1) *p = *p1;
//	return res;
//}
//
//void CKFApp::Reverse(void)
//{
//	sins.eth.wie = -sins.eth.wie;  sins.vn = -sins.vn; sins.eb = -sins.eb;
//	vnRes = -vnRes; xpt.vn = -xpt.vn;
//	int idx[] = { 3,4,5, 9,10,11, 18 };
//	for (int k = 0; k < (sizeof(idx) / sizeof(int)); k++) Xk.dd[idx[k]] = -Xk.dd[idx[k]];
//	TDReset();
//	int idxp[] = { 0,1, 3,4,5, 6,7,8, 14 };
//	Pset = diag(Pk);
//	for (int p = 0; p < (sizeof(idxp) / sizeof(int)); p++) Pset.dd[idxp[p]] = 10.0*Pset.dd[idxp[p]];
//}
/***************************  class CSINSOD  *********************************/
CSINSOD::CSINSOD(void) :CSINSTDKF(21, 10)
{
	lvOD = O31;
	tODInt = 0.0;  	Kod = 1.0;
	Cbo = I33;  MpkD = O33;
	Hk.SetMat3(6, 6, I33);   Hk.SetMat3(6, 18, -I33);
	Hk(9, 2) = 1.0;
	measGPSvnValid = measGPSposValid = measODValid = measMAGyawValid = 0;
}

void CSINSOD::Init(const CSINS &sins0, int grade)
{
	CSINSKF::Init(sins0);
	sins.lever(-lvOD);
	sins.pos = sins.posL;   //IMUת����̼�λ��
	//posDR = sins0.pos;
	posDR = sins.posL;
	//posDR = sins.pos;
	/*Pmax.Set2(10.0*glv.deg, 10.0*glv.deg, 30.0*glv.deg, 50.0, 50.0, 50.0, 1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4,
		10.0*glv.dph, 10.0*glv.dph, 10.0*glv.dph, 10.0*glv.mg, 10.0*glv.mg, 10.0*glv.mg, 1 * glv.deg, 0.01, 1 * glv.deg, 1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4);
	Pmin.Set2(0.01*glv.min, 0.01*glv.min, 0.1*glv.min, 0.01, 0.01, 0.1, 1.0 / glv.Re, 1.0 / glv.Re, 0.1,
		0.001*glv.dph, 0.001*glv.dph, 0.001*glv.dph, 5.0*glv.ug, 5.0*glv.ug, 15.0*glv.ug, 1 * glv.min, 0.0001, 0.2*glv.min, 1.0 / glv.Re, 1.0 / glv.Re, 1.0);
	Pk.SetDiag2(1.0*glv.deg, 1.0*glv.deg, 1.0*glv.deg, 1.0, 1.0, 1.0, 100.0 / glv.Re, 100.0 / glv.Re, 100.0,
		0.01*glv.dph, 0.01*glv.dph, 0.01*glv.dph, .10*glv.mg, .10*glv.mg, .10*glv.mg, 10.0*glv.min, 0.05, 15 * glv.min, 1.0e1 / glv.Re, 1.0e1 / glv.Re, 1.0e1);
	Qt.Set2(0.001*glv.dpsh, 0.001*glv.dpsh, 0.001*glv.dpsh, 1.0*glv.ugpsHz, 1.0*glv.ugpsHz, 1.0*glv.ugpsHz, 0.0, 0.0, 0.0,
		0.0*glv.dphpsh, 0.0*glv.dphpsh, 0.0*glv.dphpsh, 0.0*glv.ugpsh, 0.0*glv.ugpsh, 0.0*glv.ugpsh, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	Xmax.Set(INF, INF, INF, INF, INF, INF, INF, INF, INF, 0.1*glv.dph, 0.1*glv.dph, 0.1*glv.dph, 200.0*glv.ug, 200.0*glv.ug, 200.0*glv.ug, 1 * glv.deg, 0.05, 1 * glv.deg, INF, INF, INF);
	Rt.Set2(0.2, 0.2, 0.6, 10.0 / glv.Re, 10.0 / glv.Re, 30.0, 10.1 / RE, 10.1 / RE, 10.1);*/

//	Pmax.Set2(10.0*glv.deg, 10.0*glv.deg, 30.0*glv.deg,
//		50.0, 50.0, 50.0,
//		1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4,
//		10.0*glv.dph, 10.0*glv.dph, 10.0*glv.dph,
//		10.0*glv.mg, 10.0*glv.mg, 10.0*glv.mg,
//		1 * glv.deg, 0.01, 1 * glv.deg,
//		1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4);
//
//	Pmin.Set2(0.01*glv.min, 0.01*glv.min, 0.1*glv.min,
//		0.01, 0.01, 0.1,
//		1.0 / glv.Re, 1.0 / glv.Re, 0.1,
//		0.001*glv.dph, 0.001*glv.dph, 0.001*glv.dph,
//		5.0*glv.ug, 5.0*glv.ug, 15.0*glv.ug,
//		1 * glv.min, 0.0001, 0.2*glv.min,
//		1.0 / glv.Re, 1.0 / glv.Re, 1.0);
//
//	Pk.SetDiag2(1.0*glv.deg, 1.0*glv.deg, 1.0*glv.deg,
//		1.0, 1.0, 1.0,
//		100.0 / glv.Re, 100.0 / glv.Re, 100.0,
//		0.01*glv.dph, 0.01*glv.dph, 0.01*glv.dph,
//		.10*glv.mg, .10*glv.mg, .10*glv.mg,
//		10.0*glv.min, 0.05, 15 * glv.min,
//		1.0e1 / glv.Re, 1.0e1 / glv.Re, 1.0e1);
//
//	Qt.Set2(0.001*glv.dpsh, 0.001*glv.dpsh, 0.001*glv.dpsh,
//		1.0*glv.ugpsHz, 1.0*glv.ugpsHz, 1.0*glv.ugpsHz,
//		0.0, 0.0, 0.0,
//		0.0*glv.dphpsh, 0.0*glv.dphpsh, 0.0*glv.dphpsh,
//		0.0*glv.ugpsh, 0.0*glv.ugpsh, 0.0*glv.ugpsh,
//		0.0, 0.0, 0.0,
//		0.0, 0.0, 0.0);
//
//	Xmax.Set(INF, INF, INF,
//		INF, INF, INF,
//		INF, INF, INF,
//		0.1*glv.dph, 0.1*glv.dph, 0.1*glv.dph,
//		200.0*glv.ug, 200.0*glv.ug, 200.0*glv.ug,
//		1 * glv.deg, 0.05, 1 * glv.deg,
//		INF, INF, INF);
//
//	Rt.Set2(0.2, 0.2, 0.6,
//		10.0 / glv.Re, 10.0 / glv.Re, 30.0,
//		10.1 / RE, 10.1 / RE, 10.1);

	Pmax.Set2(10.0*glv.deg, 10.0*glv.deg, 30.0*glv.deg,
		50.0, 50.0, 50.0,
		1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4,
		10.0*glv.dph, 10.0*glv.dph, 10.0*glv.dph,
		10.0*glv.mg, 10.0*glv.mg, 10.0*glv.mg,
		5 * glv.deg, 0.1, 5 * glv.deg,
		1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4);

	Pmin.Set2(0.01*glv.min, 0.01*glv.min, 0.1*glv.min,
		0.01, 0.01, 0.1,
		1.0 / glv.Re, 1.0 / glv.Re, 0.1,
		0.001*glv.dph, 0.001*glv.dph, 0.001*glv.dph,
		5.0*glv.ug, 5.0*glv.ug, 15.0*glv.ug,
		1 * glv.min, 0.0001, 0.2*glv.min,
		1.0 / glv.Re, 1.0 / glv.Re, 1.0);

	Pk.SetDiag2(1.0*glv.deg, 1.0*glv.deg, 1.0*glv.deg,
		0.5, 0.5, 0.5,
		5.0 / glv.Re, 5.0 / glv.Re, 10.0,
		0.01*glv.dph, 0.01*glv.dph, 0.01*glv.dph,
		.10*glv.mg, .10*glv.mg, .10*glv.mg,
		10.0*glv.min, 0.05, 15 * glv.min,
		5 / glv.Re, 5 / glv.Re, 10);

	Qt.Set2(0.001*glv.dpsh, 0.001*glv.dpsh, 0.001*glv.dpsh,
		1.0*glv.ugpsHz, 1.0*glv.ugpsHz, 1.0*glv.ugpsHz,
		0.0, 0.0, 0.0,
		0.0*glv.dphpsh, 0.0*glv.dphpsh, 0.0*glv.dphpsh,
		0.0*glv.ugpsh, 0.0*glv.ugpsh, 0.0*glv.ugpsh,
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0);

	Xmax.Set(INF, INF, INF,
		INF, INF, INF,
		INF, INF, INF,
		0.1*glv.dph, 0.1*glv.dph, 0.1*glv.dph,
		200.0*glv.ug, 200.0*glv.ug, 200.0*glv.ug,
		10 * glv.deg, 0.1, 10 * glv.deg,
		INF, INF, INF);

	Rt.Set2(0.2, 0.2, 0.6,
		10.0 / glv.Re, 10.0 / glv.Re, 30.0,
		0.5 / RE, 0.5 / RE, 2);

	//Pmax.Set2(10.0*glv.deg, 10.0*glv.deg, 30.0*glv.deg, 50.0, 50.0, 50.0, 1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4,
	//	10.0*glv.dph, 10.0*glv.dph, 10.0*glv.dph, 10.0*glv.mg, 10.0*glv.mg, 10.0*glv.mg, 0 * glv.deg, 0.0, 0 * glv.deg, 1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4);
	//Pmin.Set2(0.01*glv.min, 0.01*glv.min, 0.1*glv.min, 0.01, 0.01, 0.1, 1.0 / glv.Re, 1.0 / glv.Re, 0.1,
	//	0.001*glv.dph, 0.001*glv.dph, 0.001*glv.dph, 5.0*glv.ug, 5.0*glv.ug, 15.0*glv.ug, 0 * glv.min, 0.000, 0*glv.min, 1.0 / glv.Re, 1.0 / glv.Re, 1.0);
	//Pk.SetDiag2(1.0*glv.deg, 1.0*glv.deg, 1.0*glv.deg, 1.0, 1.0, 1.0, 100.0 / glv.Re, 100.0 / glv.Re, 100.0,
	//	0.01*glv.dph, 0.01*glv.dph, 0.01*glv.dph, .10*glv.mg, .10*glv.mg, .10*glv.mg, 0.0*glv.min, 0.0, 0 * glv.min, 1.0e1 / glv.Re, 1.0e1 / glv.Re, 1.0e1);
	//Qt.Set2(0.001*glv.dpsh, 0.001*glv.dpsh, 0.001*glv.dpsh, 1.0*glv.ugpsHz, 1.0*glv.ugpsHz, 1.0*glv.ugpsHz, 0.0, 0.0, 0.0,
	//	0.0*glv.dphpsh, 0.0*glv.dphpsh, 0.0*glv.dphpsh, 0.0*glv.ugpsh, 0.0*glv.ugpsh, 0.0*glv.ugpsh, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	//Xmax.Set(INF, INF, INF, INF, INF, INF, INF, INF, INF, 0.1*glv.dph, 0.1*glv.dph, 0.1*glv.dph, 200.0*glv.ug, 200.0*glv.ug, 200.0*glv.ug, 0 * glv.deg, 0.0, 0 * glv.deg, INF, INF, INF);
	//Rt.Set2(0.2, 0.2, 0.6, 10.0 / glv.Re, 10.0 / glv.Re, 30.0, 10.1 / RE, 10.1 / RE, 10.1);

	//Pmax.Set2(10.0*glv.deg, 10.0*glv.deg, 30.0*glv.deg,  
	//		  50.0, 50.0, 50.0,
	//	      1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4,
	//		  10.0*glv.dph, 10.0*glv.dph, 10.0*glv.dph, 
	//		  1000.0*glv.mg, 1000.0*glv.mg, 1000.0*glv.mg, 
	//	      10 * glv.deg, 1, 10 * glv.deg, 
	//	      1.0e4 / glv.Re, 1.0e4 / glv.Re, 1.0e4);
	//
	//Pmin.Set2(0.01*glv.min, 0.01*glv.min, 0.1*glv.min,
	//		  0.001, 0.001, 0.001, 
	//		  0.001 / glv.Re, 0.001 / glv.Re, 0.001,
	//		  0.0001*glv.dph, 0.0001*glv.dph, 0.0001*glv.dph, 
	//		  1.0*glv.ug, 1.0*glv.ug, 1.0*glv.ug,
	//		  0.0 * glv.min, 0.00000, 0.0*glv.min, 
	//		  0.001 / glv.Re, 0.001 / glv.Re, 0.001);
	//
	//Pk.SetDiag2(0.1*glv.deg, 0.1*glv.deg, 0.1*glv.deg, 
	//			0.1, 0.1, 0.1, 
	//			1.0 / glv.Re, 1.0 / glv.Re, 1.0,
	//			0.01*glv.dph, 0.01*glv.dph, 0.01*glv.dph,
	//		    .10*glv.mg, .10*glv.mg, .10*glv.mg, 
	//		    0*glv.min, 0.00, 0 * glv.min,
	//		    1.0 / glv.Re, 1.0 / glv.Re, 1.0);
	//
	//Qt.Set2(0.001*glv.dpsh, 0.001*glv.dpsh, 0.001*glv.dpsh,
	//		1.0*glv.ugpsHz, 1.0*glv.ugpsHz, 1.0*glv.ugpsHz,
	//		0.0, 0.0, 0.0,
	//		0.0*glv.dphpsh, 0.0*glv.dphpsh, 0.0*glv.dphpsh, 
	//		0.0*glv.ugpsh, 0.0*glv.ugpsh, 0.0*glv.ugpsh,
	//		0.0, 0.0, 0.0,
	//		0.0, 0.0, 0.0);
	//
	//Xmax.Set(INF, INF, INF,
	//		 INF, INF, INF,
	//		 INF, INF, INF,
	//		 1*glv.dph, 1*glv.dph, 1*glv.dph, 
	//		 200.0*glv.mg, 200.0*glv.mg, 200.0*glv.mg,
	//		 0 * glv.deg, 0, 0 * glv.deg, 
	//		 INF, INF, INF);
	//
	///*Rt.Set2(0.2, 0.2, 0.6,
	//		10.0 / glv.Re, 10.0 / glv.Re, 30.0, 
	//		5.0 / RE, 5.0 / RE, 5.0,
	//		0.1*glv.deg	);*/
	//Rt.Set2(0.1, 0.1, 0.1,
	//	1 / glv.Re, 1 / glv.Re, 5,
	//	0.1 / RE, 0.1 / RE, 1,
	//	0.5*glv.deg);
	////Rt.Set2(0, 0, 0,
	////	0 / glv.Re, 0 / glv.Re, 0,
	////	1 / RE, 1 / RE, 1,
	////	0*glv.deg);
	Rmax = Rt * 100;  Rmin = Rt * 0.01;  Rb = 0.9;
	FBTau.Set(1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0);
	//	FBTau.Set(1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, INF, INF, INF);
}

void CSINSOD::SetFt(int nnq)
{
	CSINSKF::SetFt(15);
	CMat3 MvkD = norm(sins.vn)*CMat3(-sins.Cnb.e02, sins.Cnb.e01, sins.Cnb.e00,
		-sins.Cnb.e12, sins.Cnb.e11, sins.Cnb.e10, -sins.Cnb.e22, sins.Cnb.e21, sins.Cnb.e20);
	Ft.SetMat3(18, 0, sins.Mpv*askew(sins.vn));
	Ft.SetMat3(18, 15, sins.Mpv*MvkD);
	Ft.SetMat3(18, 18, sins.Mpp);
}

void CSINSOD::SetMeas(void)
{
	if (measGPSvnValid) { SetMeasFlag(000007); }	// SINS/GPS_VEL
	if (measGPSposValid) { SetMeasFlag(000070); }	// SINS/GPS_POS
	if (measODValid) { SetMeasFlag(000700); }	// SINS/DR
	if (measMAGyawValid) { SetMeasFlag(001000); }	// SINS/MAG_YAW
	measGPSvnValid = measGPSposValid = measODValid = measMAGyawValid = 0;
}

void CSINSOD::Feedback(double fbts)
{
	CSINSKF::Feedback(fbts);
	Cbo = Cbo * a2mat(CVect3(FBXk.dd[15], 0.0, FBXk.dd[17]));
	Kod *= 1 - FBXk.dd[16];
	posDR -= *(CVect3*)(&FBXk.dd[18]);
}

void CSINSOD::SetMeasGPS(const CVect3 &pgps, const CVect3 &vgps)
{
	if (!IsZero(pgps, EPS))
	{
		*(CVect3*)&Zk.dd[3] = sins.pos - pgps;
		measGPSposValid = 1;
	}
	if (!IsZero(vgps, EPS))
	{
		*(CVect3*)&Zk.dd[0] = sins.vn - vgps;
		measGPSvnValid = 1;
	}
}

void CSINSOD::SetMeasOD(double dSod, double ts)
{
	CVect3 dSn = sins.Cnb*(Cbo*CVect3(0, dSod*Kod, 0));
//    CVect3 dSn = sins.Cnb*(CVect3(0, dSod*Kod, 0));
 	posDR = posDR + sins.eth.vn2dpos(dSn, 1.0);
	tODInt += ts;
	sins.lever(lvOD);
	//if (tODInt > 0.03)
	//	{
			*(CVect3*)&Zk.dd[6] = sins.posL - posDR;
			tODInt = 0.0;
			measODValid = 1;
	//	}
	//else
	//{
	//	tODInt = 0.0;
	//}
	//CVect3 dSn = sins.Cnb*(Cbo*CVect3(0, dSod*Kod, 0));
	//posDR = posDR + sins.eth.vn2dpos(dSn, 1.0);
	//tODInt += ts;
	//if (fabs(sins.wnb.k) < .5*glv.dps)
	//{
	//	if (tODInt > 1.0)
	//	{
	//		*(CVect3*)&Zk.dd[6] = sins.pos - posDR;
	//		tODInt = 0.0;
	//		measODValid = 1;
	//	}
	//}
	//else
	//{
	//	tODInt = 0.0;
	//}
}

void CSINSOD::SetMeasYaw(double ymag)
{
	if (ymag > EPS || ymag < -EPS)  // !IsZero(yawGPS)
		if (sins.att.i<30 * DEG && sins.att.i>-30 * DEG)
		{
			*(CVect3*)&Zk.dd[9] = -diffYaw(sins.att.k, ymag);
			measMAGyawValid = 1;
		}
}

int CSINSOD::Update(const CVect3 *pwm, const CVect3 *pvm, int nn, double ts)
{
	int res = TDUpdate(pwm, pvm, nn, ts, 5);
	vnRes = sins.vn + sins.Cnb*askew(sins.web)*lvOD;
	posRes = sins.pos + sins.MpvCnb*lvOD;
	return res;
}

int CSINSOD::TDUpdate(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts, int nStep)
{
	//	if(sins.tk>99)
	//		int debugi = 1;

	sins.Update(pwm, pvm, nSamples, ts);
	Feedback(sins.nts);
	for (int j = 0; j < nr; j++) measlost.dd[j] += sins.nts;

	measRes = 0;

	if (nStep <= 0 || nStep >= maxStep) { nStep = maxStep; }
	tdStep = nStep;

	tdts += sins.nts; kftk = sins.tk;
	meanfn = meanfn + sins.fn; ifn++;
	for (int i = 0; i < nStep; i++)
	{
		if (iter == -2)			// -2: set measurements
		{
			if (ifn == 0)	break;
			CVect3 vtmp = meanfn * (1.0 / ifn); meanfn = O31; ifn = 0;
			sins.fn = vtmp; SetFt(nq); sins.fn = vtmp;
			SetMeas(); SetHk(nq);
		}
		else if (iter == -1)			// -1: discrete
		{
			Fk = ++(Ft*tdts); // Fk = I+Ft*ts
			Qk = Qt * tdts;
			Xk = Fk * Xk;
			//			RtFading(tdts);
			meantdts = tdts; tdts = 0.0;
		}
		else if (iter < nq)		// 0 -> (nq-1): Fk*Pk
		{
			int row = iter;
			RowMul(Pk1, Fk, Pk, row);
		}
		else if (iter < 2 * nq)		// nq -> (2*nq-1): Fk*Pk*Fk+Qk
		{
			int row = iter - nq;
			RowMulT(Pk, Pk1, Fk, row);
			Pk.dd[nq*row + row] += Qk.dd[row];
			//			if(row==nq-1) {	Pk += Qk; }
		}
		else if (iter < 2 * (nq + nr))	// (2*nq) -> (2*(nq+nr)-1): sequential measurement updating
		{
			if (measstop > 0) measflag = 0;
			int row = (iter - 2 * Ft.row) / 2;
			int flag = measflag & (0x01 << row);
			if (flag)
			{
				//				if((iter-2*Ft.row)%2==0)
				if (iter % 2 == 0)
				{
					Hi = Hk.GetRow(row);
					Pxz = Pk * (~Hi);
					Pz0 = (Hi*Pxz)(0, 0);
					innovation = Zk(row) - (Hi*Xk)(0, 0);
					adptOKi = 1;
					if (Rb.dd[row] > EPS)
						adptOKi = RAdaptive(row, innovation, Pz0);
					double Pzz = Pz0 + Rt(row) / rts(row);
					Kk = Pxz * (1.0 / Pzz);
				}
				else
				{
					measflag ^= flag;
					if (adptOKi)
					{
						measRes |= flag;
						Xk += Kk * innovation;
						Pk -= Kk * (~Pxz);
						measlost.dd[row] = 0.0;
					}
					if (Zfd0.dd[row] < INF / 2)
					{
						RPkFading(row);
					}
				}
			}
			else
			{
				nStep++;
			}
			if (iter % 2 == 0)
				RtFading(row, meantdts);
		}
		else if (iter == 2 * (nq + nr))	// 2*(nq+nr): Xk,Pk constrain & symmetry
		{
			XPConstrain();
			symmetry(Pk);
		}
		else if (iter >= 2 * (nq + nr) + 1)	// 2*(nq+nr)+1: Miscellanous
		{
			Miscellanous();
			iter = -3;
		}
		iter++;
	}
	SecretAttitude();
	if (measstop > -1000000) measstop--;

	measflaglog |= measRes;
	return measRes;
}

CVect3 lla2enu(double latitude, double longtitude, double altitude, double latitude0, double longtitude0, double altitude0)
{
    double Re=6378137;
    double f=1/298.257;
    double pi=3.14159265359;
    double deg2rad, e, e2, s1, c1, s12, sq, sq2, RMh, RNh, c1RNh;
    double tmp_latitude0, tmp_longtitude0;
    double x,y,z;
    deg2rad=pi/180;
    //latitude=latitude*deg2rad;
    tmp_latitude0=latitude0;
    //longtitude=longtitude*deg2rad;
    tmp_longtitude0=longtitude0;
    e=sqrt(2*f-f*f);
    e2=e*e;
    s1=sin(latitude);
    c1=cos(latitude);
    s12=s1*s1;
    sq=1-e2*s12;
    sq2=sqrt(sq);
    RMh=Re*(1-e2)/sq/sq2+altitude;
    RNh=Re/sq2+altitude;
    c1RNh=c1*RNh;
    x=(longtitude-tmp_longtitude0)*c1RNh;
    y=(latitude-tmp_latitude0)*RMh;
    z=altitude-altitude0;
    return CVect3(x,y,z);
}