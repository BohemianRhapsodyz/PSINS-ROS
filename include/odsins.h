//
// Created by hx on 2021/3/8.
//

#ifndef SRC_ODSINS_H
#define SRC_ODSINS_H

#endif //SRC_ODSINS_H

#include <fstream>
#include <iostream>
#include "KFApp.h"
#include "PSINS.h"
#include <iomanip>
#include <nav_msgs/Odometry.h>

std::ofstream aligntxt("/home/hx/od_sins_realtime/align.txt");
std::ofstream navigation("/home/hx/od_sins_realtime/navigation.txt");
std::ofstream kfx("/home/hx/od_sins_realtime/kf.txt");

int aligncnt=0, outcnt=1;
double latitude0=0.596638466053064, longtitude0=1.902709569997353, altitude0=539;
ImuGpsData *pImuGps = new ImuGpsData();
CVect3 pos0(0.596638466053064,1.902709569997353,539), alignatt;
CAligni0 aln(pos0);
CSINSOD kf;

nav_msgs::Odometry nav;

