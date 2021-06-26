#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

#include <time.h>
#include <iostream>
#include <malloc.h>
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include <string>
#include <fstream>
#include <iomanip>
#include "calibration.h"
#include "zupt.h"
#include "strapdown.h"
#include "KFApp.h"
#include "PSINS.h"
#include "std_msgs/String.h"
#include <sstream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "odsins.h"

using namespace std;

ros::Publisher nav_pub;

void TimeupdateCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    pImuGps->wm.i=msg->angular_velocity.x * TS;
    pImuGps->wm.j=msg->angular_velocity.y * TS;
    pImuGps->wm.k=msg->angular_velocity.z * TS;
    pImuGps->vm.i=msg->linear_acceleration.x * TS;
    pImuGps->vm.j=msg->linear_acceleration.y * TS;
    pImuGps->vm.k=msg->linear_acceleration.z * TS;
    if (aligncnt < Aligni0T * FRQ)
    {
        aln.Update(&pImuGps->wm, &pImuGps->vm, 1, TS);
        alignatt = q2att(aln.qnb);
        alignatt *= (180 / PI);
        aligntxt<<alignatt<<endl;
        ROS_INFO("alignyaw: [%lf]", alignatt.k);
    }
    else if ( aligncnt == Aligni0T * FRQ )
    {
        /*pos0.i = 0.596638466053064;
        pos0.j = 1.902709569997353;
        pos0.k = 5.390300000000000e+02;*/
        kf.lvOD = CVect3(1, 1, 0.1);
        kf.Init(CSINS(aln.qnb, O31, pos0, 0));
        kf.Pk.dd[440]=0;
    }
    else
    {
        kf.Update(&pImuGps->wm, &pImuGps->vm, 1, TS);
        //ROS_INFO("navyaw: [%lf]", kf.sins.att.k);
    }
    aligncnt++;
}

void MeasureupdateCallback(const std_msgs::Float64::ConstPtr& msg)
{
    if ( aligncnt > ( Aligni0T * FRQ + 10 ) )
    {
        nav.header.frame_id = "nav";
        ros::Time currentTime = ros::Time::now();
        nav.header.stamp = currentTime;
        nav.header.seq = outcnt;
        double vd, shiftX=0, shiftY=0, shiftZ=0;
        vd = msg->data;
        kf.SetMeasOD(vd, TS);

        navigation << fixed << setprecision(16) << kf.posRes.i << endl;

        nav.pose.pose.orientation.w=kf.sins.qnb.q0;
        nav.pose.pose.orientation.x=kf.sins.qnb.q1;
        nav.pose.pose.orientation.y=kf.sins.qnb.q2;
        nav.pose.pose.orientation.z=kf.sins.qnb.q3;
        CVect3 enu=lla2enu(kf.posRes.i, kf.posRes.j, kf.posRes.k, latitude0, longtitude0, altitude0);
        nav.pose.pose.position.x=enu.i;
        nav.pose.pose.position.y=enu.j;
        nav.pose.pose.position.z=enu.k;

        nav_pub.publish(nav);
        //ROS_INFO("high: [%lf]", nav.pose.position.z);
        outcnt++;
    }
}

int main(int argc, char **argv) {

    /* vector<string> imudata;
     LoadImu("/home/hx/od_sins_realtime/imu.txt",imudata);*/

    ros::init(argc, argv, "od_sins_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/fogtopic", 1000, TimeupdateCallback);
    ros::Subscriber od_sub = nh.subscribe("/od", 1000, MeasureupdateCallback);

    nav_pub=nh.advertise<nav_msgs::Odometry>("nav",1000);
    /*ros::Publisher out_pub;
    out_pub=nh.advertise<geometry_msgs::PoseStamped>("nav");*/

    ros::spin();

    return 0;
}
	/*int debug = 0;
	int detector = 0;
	int detectnum = -1;
	double gyro[3], fib[3], odom;
	vector<double>vex;
	vector<double>vey;
	vector<double>vez;*/

	//pos0.i = 20*PI/180;
	//pos0.j = 110*PI/180;
	//pos0.k = 380;
	// initial alignment
	//printf("Initial alignment...\n");
	/*for (int ii = 0; ii < Aligni0T*FRQ; ii++)
	{
		//fimu.load(1);
        Loadoneline(imudata, pImuGps, ii);
		aln.Update(&pImuGps->wm, &pImuGps->vm, 1, TS);
		alignatt = q2att(aln.qnb);
		alignatt *= (180 / PI);
//		if (ii==4999)
//        {
//		    int jjj=0;
//        }
		//fres << alignatt;
		//align<<alignatt.i<<"\t"<<alignatt.j<<"\t"<<alignatt.k<<endl;
		//align<<alignatt<<endl;
	}*/
	//align.close();
   // CSINS si(aln.qnb, O31, pos0, pImuGps->t);
   // si.Update(&pImuGps->wm, &pImuGps->vm,1,0.008);
	/*CAlignkf alignkf;
	alignkf.Init(CSINS(aln.qnb, O31, pos0, pImuGps->t));
	for (int ii = 0; ii < AlignkfT*FRQ; ii++)
	{
        Loadoneline(imudata, pImuGps, ii);
	    //fimu.load(1);
		alignkf.Update(&pImuGps->wm, &pImuGps->vm, 1, TS);
		alignatt = alignkf.sins.att;
		alignatt *= (180 / PI);
		//fres << alignatt;
	}*/

//    kf.lvOD = CVect3(0, 0, 0);
	//kf.Init(CSINS(alignkf.sins.qnb, O31, pos0, pImuGps->t));
	/*for (jj = 0; jj < ln; jj++)
	{
		//fimu.load(1);
        Loadoneline(imudata, pImuGps, jj+12500);

		//�ٶȷֽ�
		double vd, alpha, beta, vdx, vdy, vdz;
		vd = pImuGps->odom;

		if(jj==40000)
        {
		    int jjj=0;
        }
//		if ((vd != 0) && (kf.sins.wnb.k != 0))
//			alpha = asin(dl / (vd / kf.sins.wnb.k));
//		else
//			alpha = 0;
//		if ((vd != 0) && (kf.sins.wnb.i != 0))
//			beta = asin(dl / (2 * vd / kf.sins.wnb.i));
//		else
//			beta = 0;
//		vdx = vd * sin(alpha);
//		vdy = vd * cos(alpha)*cos(beta);
//		vdz = vd * sin(beta);
		//////////////////////////////////////////////////////////////////////////////
		kf.SetMeasOD(vd, TS);
		//if (jj % 4 == 0)
		//{
//			ffsn << kf.sins.att << kf.vnRes << kf.posRes << pImuGps->t << kf.sins.eb << kf.sins.db  <<  kf.posDR << kf.Kod << kf.Cbo;
//			//ffsn << kf.sins.att << kf.sins.vn << kf.sins.pos << kf.sins.eb << kf.sins.db << pImuGps->t << kf.posDR << kf.Kod << kf.Cbo;
//			kfxk << kf.Xk << kf.FBXk << vd;
//			kfpk << kf.Pk;
		//}
		//navigation << kf.posRes.i << "\t" << kf.posRes.j << "\t" << kf.posRes.k << endl;
        navigation << fixed << setprecision(16) << kf.posRes.i << endl;
		//navigation << kf.sins.att << kf.vnRes << kf.posRes << kf.sins.eb << kf.sins.db << kf.posDR << kf.Cbo << pImuGps->t << "\t" << kf.Kod << endl;
        //navigation << kf.sins.att << kf.vnRes << kf.posRes  << endl;
       //kfx << kf.Xk << kf.FBXk << kf.Zk << vd << endl;
	}*/
	//navigation.close();
	//kfx.close();
	//alignpitch = alignkf.sins.att.i;
	//alignroll = alignkf.sins.att.j;
	//phiwest2east(alignkf.sins.att.k);
	//��������
	//for (jj = 0; jj < ln; jj++)
	//{

	//	fimu.load(1);
	//	/*gyro[0] = data1 / cyzq;
	//	gyro[1] = data2 / cyzq;
	//	gyro[2] = data3 / cyzq;
	//	fib[0] = data4 / cyzq;
	//	fib[1] = data5 / cyzq;
	//	fib[2] = data6 / cyzq;*/
	//	gyro[0] = pImuGps->wm.i*FRQ;     //������>����
	//	gyro[1] = pImuGps->wm.j*FRQ;
	//	gyro[2] = pImuGps->wm.k*FRQ;
	//	fib[0] = pImuGps->vm.i*FRQ;          //������>���ٶ�
	//	fib[1] = pImuGps->vm.j*FRQ;
	//	fib[2] = pImuGps->vm.k*FRQ;
	//	vex.push_back(gyro[0]);
	//	vey.push_back(gyro[1]);
	//	vez.push_back(fib[2]);
	//	//ax.push(fib[0]);
	//	//ay.push(fib[1]);
	//	//az.push(fib[2]);
	//	if (vex.size() > windows_len)
	//	{
	//		vex.erase(vex.begin());
	//		vey.erase(vey.begin());
	//		vez.erase(vez.begin());
	//	}
	//	//�궨
	//	//if (jj <= (biaodingtime / cyzq))

	//	//{
	//	//	odom = data7;
	//		//��װ����
	//		pitcherr = 0;
	//		rollerr = 0;
	//		yawerr = 0;
	//	//	Calculate_Tmb();

	//	//	fprintf(savedata1, "%lf %lf %lf %lf %lf %lf\n", gyro[0], gyro[1], gyro[2], fib[0], fib[1], fib[2]);
	//	//	//SINS����
	//	//	strapdown(gyro[0], gyro[1], gyro[2], fib[0], fib[1], fib[2], odom, pitch0, roll0, yaw0);

	//	//	switch (inter)
	//	//	{
	//	//	case 0:
	//	//		break;
	//	//	case 1:
	//	//		break;
	//	//	case 2:
	//	//		fprintf(savedata2, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", attitude[0], attitude[1], attitude[2], hit_v[0], hit_v[1], hit_v[2], hit_x*Degree_Rad, hit_y*Degree_Rad, hit_z);

	//	//		inter = 0;
	//	//		debug++;
	//	//		if (debug == 1000)
	//	//			debug = 0;
	//	//		break;
	//	//	}

	//	//}
	//	//д�궨���
	//	/*if (jj == (biaodingtime / cyzq))
	//	{
	//		S1[0] = (hit_x - lon_0)*Rxp;
	//		S1[1] = (hit_y - phi_0)*Ryp;
	//		S1[2] = hit_z - high0;

	//		S2[0] = (lon_ref - lon_0)*Rxp;
	//		S2[1] = (phi_ref - phi_0)*Ryp;
	//		S2[2] = high_ref - high0;

	//		Calculate_theta(S1, S2, &pitcherr);
	//		Calculate_kd(S1, S2, &kod);
	//		Calculate_phi(S1, S2, &yawerr);
	//		rollerr = 0;
	//	}*/

	//	//����
	//	//if (jj > (biaodingtime / cyzq))
	//	//{
	//		//���㰲װ������
	//		Calculate_Tmb();
	//		//��̼ƶ���*�̶�ϵ�����
	//		kod = 0.01;
	//		odom = pImuGps->odom * (1 + kod);
	//		//fprintf(savedata1, "%lf %lf %lf %lf %lf %lf\n", gyro[0], gyro[1], gyro[2], fib[0], fib[1], fib[2]);
	//		//SINS����
	//		strapdown(gyro[0], gyro[1], gyro[2], fib[0], fib[1], fib[2], odom, alignkf.sins.att.i, alignkf.sins.att.j, alignkf.sins.att.k);
	//		//·��Լ��
	//		//dist = dist + odom;
	//		//if ((dist > 190) && (dist < 210))
	//		//{
	//		//	//���ٴ����
	//		//	if (ifdetect(vex, vey))
	//		//	{
	//		//		detector++;
	//		//	}
	//		//	if (detector > 10)
	//		//	{
	//		//		detectnum++;
	//		//		//hit_x y ����
	//		//		hit_x = speedbump_x[detectnum];
	//		//		hit_y = speedbump_y[detectnum];
	//		//		//����
	//		//		detector = 0;
	//		//		dist = 0;
	//		//	}
	//		//	//fprintf(savedata, "%lf %lf\n", bias, var);
	//		//}
	//		//if (dist > 210)
	//		//{
	//		//	detector = 0;
	//		//	detectnum++;
	//		//	dist = dist - 200;
	//		//}
	//		switch (inter)
	//		{
	//		case 0:
	//			break;
	//		case 1:
	//			break;
	//		case 2:
	//			//fprintf(savedata2, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", attitude[0], attitude[1], attitude[2], hit_v[0], hit_v[1], hit_v[2], hit_x*Degree_Rad, hit_y*Degree_Rad, hit_z);
	//			ffsn << attitude[0] << attitude[1] << attitude[2] << hit_v[0] << hit_v[1] << hit_v[2] << hit_x * Degree_Rad << hit_y * Degree_Rad << hit_z;

	//			inter = 0;
	//			debug++;
	//			break;
	//		}
	//	//}

	//	if (debug == 39101)
	//		debug = 0;

	//}


//void strapdown(double hit_shuju1, double hit_shuju2, double hit_shuju3, double hit_shuju4, double hit_shuju5, double hit_shuju6, double hit_shuju7, double hit_shuju8, double hit_shuju9, double hit_shuju10)
//{
//	double pitchT, rollT, yawT, dyaw, dpitch;
//	double wib[3], fb[3], fn_1[3], dv[3], dv1[3], hit_v1[3], hit_v2[3], hit_p[3], hit_p1[3], hit_p2[3], od, vd, alpha, beta;
//	//��ʼ��Ϣװ��
//	if (jj == 0)
//	{
//
//		phi = phi_0;
//		hit_x = lon_0;
//		hit_y = phi_0;
//		hit_z = high0;
//		for (int j = 0; j < 3; j++)
//		{
//			hit_v[j] = 0;
//		}
//		pitchT = hit_shuju8;
//		rollT = hit_shuju9;
//		yawT = hit_shuju10;
//		T[0] = cos(yawT)*cos(rollT) - sin(yawT)*sin(pitchT)*sin(rollT);
//		T[1] = -sin(yawT)*cos(pitchT);
//		T[2] = cos(yawT)*sin(rollT) + sin(yawT)*sin(pitchT)*cos(rollT);
//		T[3] = cos(yawT)*sin(pitchT)*sin(rollT) + sin(yawT)*cos(rollT);
//		T[4] = cos(pitchT)*cos(yawT);
//		T[5] = sin(yawT)*sin(rollT) - cos(yawT)*sin(pitchT)*cos(rollT);
//		T[6] = -cos(pitchT)*sin(rollT);
//		T[7] = sin(pitchT);
//		T[8] = cos(pitchT)*cos(rollT);
//
//		Update_q();
//		Normalize();
//		Calculate_T();
//		Attitude();
//
//		q[0] = sqrt(1 + T[0] + T[4] + T[8]) / 2.0;
//		q[1] = (T[7] - T[5]) / (4 * q[0]);
//		q[2] = (T[2] - T[6]) / (4 * q[0]);
//		q[3] = (T[3] - T[1]) / (4 * q[0]);
//
//		Attitude();
//
//		attitude_old[0] = attitude[0];
//		attitude_old[1] = attitude[1];
//		attitude_old[2] = attitude[2];
//	}
//
//	//����
//	wib[0] = hit_shuju1;// *Rad_Degree;
//	wib[1] = hit_shuju2;// *Rad_Degree;
//	wib[2] = hit_shuju3;// *Rad_Degree;
//	g = g_0 + 0.051799*sin(phi)*sin(phi);
//	fb[0] = hit_shuju4;// *g0;
//	fb[1] = hit_shuju5;// *g0;
//	fb[2] = hit_shuju6;// *g0;
//	od = hit_shuju7;
//
//	//����
//	inter = inter + 1;
//	switch (inter)
//	{
//	case 0:
//		for (int j = 0; j < 3; j++)
//		{
//			wib1[j] = wib[j];	fb1[j] = fb[j];
//		}
//		od1 = od;
//		break;
//	case 1:
//		for (int j = 0; j < 3; j++)
//		{
//			wib2[j] = wib[j];	fb2[j] = fb[j];
//		}
//		od2 = od;
//		break;
//	case 2:
//		for (int j = 0; j < 3; j++)
//		{
//			wib3[j] = wib[j];	fb3[j] = fb[j];
//		}
//		od3 = od;
//		Calculate_wpbb();
//		Rgkt4_q(q);
//		Normalize();
//		Calculate_T();
//		Attitude();
//		for (int j = 0; j < 3; j++)
//		{
//			wib1[j] = wib3[j];	fb1[j] = fb3[j];
//		}
//		od1 = od3;
//		//�ٶȸ���
//		vd = (od2 + od3) / cyzq / 2;
//		if ((attitude[2] - attitude_old[2]) < -90)
//			dyaw = (attitude[2] - attitude_old[2] + 360)* Rad_Degree / (2 * cyzq);
//		else if ((attitude[2] - attitude_old[2]) > 90)
//			dyaw = (attitude[2] - attitude_old[2] - 360)* Rad_Degree / (2 * cyzq);
//		else
//			dyaw = (attitude[2] - attitude_old[2])* Rad_Degree / (2 * cyzq);
//
//		dpitch = (attitude[0] - attitude_old[0])* Rad_Degree / (2 * cyzq);
//		if ((vd != 0) && (dyaw != 0))
//			alpha = asin(dl / (vd / dyaw));
//		else
//			alpha = 0;
//		if ((vd != 0) && (dpitch != 0))
//			beta = asin(dl / (2 * vd / dpitch));
//		else
//			beta = 0;
//		hit_v1[0] = vd * sin(alpha);
//		hit_v1[1] = vd * cos(alpha)*cos(beta);
//		hit_v1[2] = vd * sin(beta);
//
//		MultMatr(Tmb, hit_v1, hit_v2, 3, 3, 1);
//		MultMatr(T, hit_v2, hit_v, 3, 3, 1);
//		/*MultMatr(T, fb2, fn_1, 3, 3, 1);
//		dv[0] = fn_1[0] + (2 * wiep[2])*hit_v[1] - (2 * wiep[1] + wepp[1])*hit_v[2];
//		dv[1] = fn_1[1] - (2 * wiep[2])*hit_v[0] + (2 * wiep[0] + wepp[0])*hit_v[2];
//		dv[2] = fn_1[2] + (2 * wiep[1] + wepp[1])*hit_v[0] - (2 * wiep[0] + wepp[0])*hit_v[1] - g;*/
//		/*
//		hit_v1[0] = hit_v[0] + dv[0]  * cyzq;
//		hit_v1[1] = hit_v[1] + dv[1]  * cyzq;
//		hit_v1[2] = hit_v[2] + dv[2]  * cyzq;
//
//		dv[0] = fn_1[0] + (2 * wiep[2])*hit_v1[1] - (2 * wiep[1] + wepp[1])*hit_v1[2];
//		dv[1] = fn_1[1] - (2 * wiep[2])*hit_v1[0] + (2 * wiep[0] + wepp[0])*hit_v1[2];
//		dv[2] = fn_1[2] + (2 * wiep[1] + wepp[1])*hit_v1[0] - (2 * wiep[0] + wepp[0])*hit_v1[1] - g;
//		*/
//		/*hit_v2[0] = hit_v[0] + dv[0]  * 2 * cyzq;
//		hit_v2[1] = hit_v[1] + dv[1]  * 2 * cyzq;
//		hit_v2[2] = hit_v[2] + dv[2]  * 2 * cyzq;*/
//		//λ�ø���
//		hit_p1[0] = (od2 + od3) * sin(alpha);
//		hit_p1[1] = (od2 + od3) * cos(alpha)*cos(beta);
//		hit_p1[2] = (od2 + od3) * sin(beta);
//		MultMatr(Tmb, hit_p1, hit_p2, 3, 3, 1);
//		MultMatr(T, hit_p2, hit_p, 3, 3, 1);
//
//		hit_x = hit_x + hit_p[0] / Rxp;
//		hit_y = hit_y + hit_p[1] / Ryp;
//		hit_z = hit_z + hit_p[2];
//		phi = hit_y;
//
//		attitude_old[0] = attitude[0];
//		attitude_old[1] = attitude[1];
//		attitude_old[2] = attitude[2];
//		/*for (int j = 0; j < 3; j++)
//		{
//			hit_v[j] = hit_v2[j];
//		}*/
//		break;
//	}
//}
//
//int Transpon(double*A, double*B, int line, int column)
//{
//	int i, j;
//	double *TempA;
//
//	if (line == 0 || column == 0) return 0;
//
//	TempA = (double *)malloc(line*column*(sizeof(double)));
//	for (i = 0; i < line*column; i++) TempA[i] = A[i];
//
//	for (i = 0; i < column; i++)
//		for (j = 0; j < line; j++)
//			B[i*line + j] = TempA[j*column + i];
//	free(TempA);
//	return 1;
//}
//
////���� g
//
//int MultMatr(double*A, double*B, double*C, int line, int k, int column)
//{
//	int i, j, l;
//	double *TempA, *TempB;
//	if (line == 0 || k == 0 || column == 0) return 0;
//
//	TempA = (double *)malloc(line*k*(sizeof(double)));
//	TempB = (double *)malloc(k*column*(sizeof(double)));
//	for (i = 0; i < line*k; i++) TempA[i] = A[i];
//	for (i = 0; i < k*column; i++) TempB[i] = B[i];
//
//	for (i = 0; i < line; i++)
//		for (j = 0; j < column; j++)
//		{
//			C[i*column + j] = 0.0;
//			for (l = 0; l < k; l++) C[i*column + j] = C[i*column + j] + TempA[i*k + l] * TempB[l*column + j];
//		}
//
//	free(TempA);
//	free(TempB);
//	return 1;
//}
//
//int SumMatr(double*A, double*B, double*C, int line, int column, int sign)
//{
//	int i, j;
//	double *TempA, *TempB;
//	int Mult = -1;   if (sign) Mult = 1;
//
//	if (line == 0 || column == 0) return 0;
//
//	TempA = (double *)malloc(line*column*(sizeof(double)));
//	TempB = (double *)malloc(line*column*(sizeof(double)));
//	for (i = 0; i < line*column; i++) TempA[i] = A[i];
//	for (i = 0; i < line*column; i++) TempB[i] = B[i];
//
//	for (i = 0; i < line; i++)
//	{
//		for (j = 0; j < column; j++)
//			C[i*column + j] = TempA[i*column + j] + TempB[i*column + j] * Mult;
//	}
//	free(TempA);
//	free(TempB);
//	return 1;
//}
//
//int brinv(double *a, int n)
//{
//	int *is, *js, i, j, k, l, u, v;
//	double d, p;
//	is = (int*)malloc(n * sizeof(int));
//	js = (int*)malloc(n * sizeof(int));
//	for (k = 0; k <= n - 1; k++)
//	{
//		d = 0.0;
//		for (i = k; i <= n - 1; i++)
//			for (j = k; j <= n - 1; j++)
//			{
//				l = i * n + j; p = fabs(a[l]);
//				if (p > d) { d = p; is[k] = i; js[k] = j; }
//			}
//		if (d + 1.0 == 1.0)
//		{
//			free(is); free(js); printf("err**not inv\n");
//			return(0);
//		}
//		if (is[k] != k)
//			for (j = 0; j <= n - 1; j++)
//			{
//				u = k * n + j; v = is[k] * n + j;
//				p = a[u]; a[u] = a[v]; a[v] = p;
//			}
//		if (js[k] != k)
//			for (i = 0; i <= n - 1; i++)
//			{
//				u = i * n + k; v = i * n + js[k];
//				p = a[u]; a[u] = a[v]; a[v] = p;
//			}
//		l = k * n + k;
//		a[l] = 1.0 / a[l];
//		for (j = 0; j <= n - 1; j++)
//			if (j != k)
//			{
//				u = k * n + j; a[u] = a[u] * a[l];
//			}
//		for (i = 0; i <= n - 1; i++)
//			if (i != k)
//				for (j = 0; j <= n - 1; j++)
//					if (j != k)
//					{
//						u = i * n + j;
//						a[u] = a[u] - a[i*n + k] * a[k*n + j];
//					}
//		for (i = 0; i <= n - 1; i++)
//			if (i != k)
//			{
//				u = i * n + k; a[u] = -a[u] * a[l];
//			}
//	}
//	for (k = n - 1; k >= 0; k--)
//	{
//		if (js[k] != k)
//			for (j = 0; j <= n - 1; j++)
//			{
//				u = k * n + j; v = js[k] * n + j;
//				p = a[u]; a[u] = a[v]; a[v] = p;
//			}
//		if (is[k] != k)
//			for (i = 0; i <= n - 1; i++)
//			{
//				u = i * n + k; v = i * n + is[k];
//				p = a[u]; a[u] = a[v]; a[v] = p;
//			}
//	}
//	free(is); free(js);
//	return(1);
//}
//
//void SumVect(double*A, double*B, double*C, int n, int sign)
//{
//	int i;
//	int Mult = -1;   if (sign) Mult = 1;
//
//	for (i = 0; i < n; i++) C[i] = A[i] + Mult * B[i];
//}
//
////�¼Ӻ���
//
//void TransMatrix(double *a, double *b, int m, int n)
//{
//	int i, j;
//	for (i = 0; i < m; i++)
//	{
//		for (j = 0; j < n; j++)
//		{
//			b[j*m + i] = a[n*i + j];
//		}
//	}
//}
//
//void sum(double*a, double*b, int line, int column, int sign, double*c)
//{
//	int i, j;
//	int Mult = -1;   if (sign) Mult = 1;
//
//	if (line == 0 || column == 0) return;
//
//
//	for (i = 0; i < line; i++)
//	{
//		for (j = 0; j < column; j++)
//			c[i*column + j] = a[i*column + j] + b[i*column + j] * Mult;
//	}
//	return;
//}
//
//void mult(double *a, double *b, int m, int n, int k, double *c)
//{
//	int i, j, l, u;
//	for (i = 0; i < m; i++)
//	{
//		for (j = 0; j < k; j++)
//		{
//			u = i * k + j;  c[u] = 0.0;
//			for (l = 0; l < n; l++)
//			{
//				c[u] = c[u] + a[i*n + l] * b[l*k + j];
//			}
//		}
//	}
//	return;
//}
//
//void Update_q()
//{
//	q[1] = sqrt(1 + T[0] - T[4] - T[8]) / 2;
//	q[2] = sqrt(1 - T[0] + T[4] - T[8]) / 2;
//	q[3] = sqrt(1 - T[0] - T[4] + T[8]) / 2;
//	q[0] = sqrt(1 - q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//
//	if ((T[7] - T[5]) < 0) q[1] = -q[1];
//	if ((T[2] - T[6]) < 0) q[2] = -q[2];
//	if ((T[3] - T[1]) < 0) q[3] = -q[3];
//}
////����q��T
//
//void Rgktf_q(double *omega, double *q, double *fq)
//{
//	fq[0] = (0 * q[0] - omega[0] * q[1] - omega[1] * q[2] - omega[2] * q[3]) / 2.0;
//	fq[1] = (omega[0] * q[0] + 0 * q[1] + omega[2] * q[2] - omega[1] * q[3]) / 2.0;
//	fq[2] = (omega[1] * q[0] - omega[2] * q[1] + 0 * q[2] + omega[0] * q[3]) / 2.0;
//	fq[3] = (omega[2] * q[0] + omega[1] * q[1] - omega[0] * q[2] + 0 * q[3]) / 2.0;
//}
//
//void Attitude()
//{
//	attitude[0] = asin(T[7])*Degree_Rad;
//	attitude[1] = atan(-T[6] / T[8])*Degree_Rad;
//	attitude[2] = atan(T[1] / T[4])*Degree_Rad;
//	if (T[4] > 0)
//	{
//		if (attitude[2] < 0)
//			attitude[2] = attitude[2] + 360;
//	}
//	else
//		attitude[2] = attitude[2] + 180;
//	/*attitude[0] = asin(T[7])*Degree_Rad;
//	attitude[1] = atan(-T[6] / T[8])*Degree_Rad;
//	attitude[2] = atan(-T[1] / T[4])*Degree_Rad;
//	if (T[4] < 0)
//		attitude[2] = attitude[2] + 180;
//	else if (attitude[2] < 0)
//		attitude[2] = attitude[2] + 360;*/
//		/*{
//		if (attitude[2] < 0)
//			attitude[2] = attitude[2] + 360;
//	}
//	else
//		attitude[2] = attitude[2] + 180;*/
//
//}
////����attitude��Degree_Rad
//
//void Calculate_T()
//{
//	T[0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
//	T[1] = 2 * (q[1] * q[2] - q[0] * q[3]);
//	T[2] = 2 * (q[1] * q[3] + q[0] * q[2]);
//
//	T[3] = 2 * (q[1] * q[2] + q[0] * q[3]);
//	T[4] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
//	T[5] = 2 * (q[2] * q[3] - q[0] * q[1]);
//
//	T[6] = 2 * (q[1] * q[3] - q[0] * q[2]);
//	T[7] = 2 * (q[2] * q[3] + q[0] * q[1]);
//	T[8] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
//}
//
//void Normalize()
//{
//	double temp;
//	temp = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
//	q[0] = q[0] / temp;
//	q[1] = q[1] / temp;
//	q[2] = q[2] / temp;
//	q[3] = q[3] / temp;
//}
//
//void Calculate_wpbb()
//{
//	double tmp_T[9];
//	int i;
//	TransMatrix(T, tmp_T, 3, 3);
//	wiep[0] = 0;
//	wiep[1] = w_ie * cos(phi);
//	wiep[2] = w_ie * sin(phi);
//
//	Rxp = R_e * (1 + e_ * sin(phi)*sin(phi));
//	Ryp = R_e * (1 - 2 * e_ + 3 * e_*sin(phi)*sin(phi));
//	wepp[0] = -hit_v[1] / Ryp;
//	wepp[1] = hit_v[0] / Rxp;
//	wepp[2] = hit_v[0] * tan(phi) / Rxp;
//
//	wipp[0] = wiep[0] + wepp[0];
//	wipp[1] = wiep[1] + wepp[1];
//	wipp[2] = wiep[2] + wepp[2];
//
//	mult(tmp_T, wipp, 3, 3, 1, wipb);
//
//	for (i = 0; i < 3; i++)
//	{
//		wpbb1[i] = wib1[i] - wipb[i];
//		wpbb2[i] = wib2[i] - wipb[i];
//		wpbb3[i] = wib3[i] - wipb[i];
//	}
//}
////����wiep phi0 phi wepp hit_v wipp wipb wpbb1 wpbb2 wpbb3 wib1 wib2 wib3
//
//void Rgkt4_q(double *x)
//{
//	int i;
//	double f[4], k[4][4], y[4];
//
//	Rgktf_q(wpbb1, x, f);
//	for (i = 0; i < 4; i++)
//	{
//		k[0][i] = f[i];
//		y[i] = x[i];
//	}
//
//	for (i = 0; i < 4; i++) x[i] = y[i] + k[0][i] * cyzq;
//
//	Rgktf_q(wpbb2, x, f);
//	for (i = 0; i < 4; i++)   k[1][i] = f[i];
//
//	for (i = 0; i < 4; i++) x[i] = y[i] + k[1][i] * cyzq;
//
//	Rgktf_q(wpbb2, x, f);
//	for (i = 0; i < 4; i++)   k[2][i] = f[i];
//
//	for (i = 0; i < 4; i++) x[i] = y[i] + k[2][i] * cyzq*2.0;
//
//	Rgktf_q(wpbb3, x, f);
//	for (i = 0; i < 4; i++)   k[3][i] = f[i];
//
//	for (i = 0; i < 4; i++) x[i] = y[i] + cyzq * 2.0*(k[0][i] + 2 * k[1][i] + 2 * k[2][i] + k[3][i]) / 6.0;
//}
//
//void Calculate_Tmb()
//{
//	Tmb[0] = cos(yawerr)*cos(rollerr) - sin(yawerr)*sin(pitcherr)*sin(rollerr);
//	Tmb[1] = -sin(yawerr)*cos(pitcherr);
//	Tmb[2] = cos(yawerr)*sin(rollerr) + sin(yawerr)*sin(pitcherr)*cos(rollerr);
//	Tmb[3] = cos(yawerr)*sin(pitcherr)*sin(rollerr) + sin(yawerr)*cos(rollerr);
//	Tmb[4] = cos(pitcherr)*cos(yawerr);
//	Tmb[5] = sin(yawerr)*sin(rollerr) - cos(yawerr)*sin(pitcherr)*cos(rollerr);
//	Tmb[6] = -cos(pitcherr)*sin(rollerr);
//	Tmb[7] = sin(pitcherr);
//	Tmb[8] = cos(pitcherr)*cos(rollerr);
//}
//
//void phiwest2east(double phi)
//{
//	if ((phi > -pi) && (phi <= 0))
//		alignyaw = -phi;
//	else
//		alignyaw = 2 * pi - phi;
//}
//
//int ifdetect(vector<double>& vecx, vector<double>& vecy)
//{
//	vector<double>fn;
//	double  f;
//	int N;
//	for (int i = 0; i < vecx.size(); i++)
//	{
//		Cal_fn(vecx[i], vecy[i], f);
//		fn.push_back(f);
//	}
//	N = vecx.size();
//	N = (N - 1) / 2;
//	Cal_Bias(fn[N], bias);
//	Cal_Var(fn, var);
//	fn.clear();
//	if (((bias >= Gateb_min) && (bias <= Gateb_max)) || ((var >= Gatev_min) && (var <= Gatev_max)))
//		return 1;
//	else return 0;
//}