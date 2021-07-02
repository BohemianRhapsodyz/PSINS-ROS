#ifndef SRC_ODSINS_H
#define SRC_ODSINS_H

#endif //SRC_ODSINS_H

#include <fstream>
#include <iostream>
#include "KFApp.h"
#include "PSINS.h"
#include <iomanip>
#include <nav_msgs/Odometry.h>

#define Aligni0T		60
#define FRQ				125
#define TS				0.008

int aligncnt=0, outcnt=1;
double latitude0=0.596638466053064, longtitude0=1.902709569997353, altitude0=539.0;  //Please initialize your position properly
ImuGpsData *pImuGps = new ImuGpsData();
CVect3 pos0(latitude0,longtitude0,altitude0);
CAligni0 aln(pos0);
CSINSOD kf;
nav_msgs::Odometry nav;

