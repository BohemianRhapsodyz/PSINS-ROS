#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include "odsins.h"
#include "KFApp.h"
#include "PSINS.h"

using namespace std;

ros::Publisher nav_pub;

void TimeupdateCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    //Convert rate to increment
    pImuGps->wm.i=msg->angular_velocity.x * TS;
    pImuGps->wm.j=msg->angular_velocity.y * TS;
    pImuGps->wm.k=msg->angular_velocity.z * TS;
    pImuGps->vm.i=msg->linear_acceleration.x * TS;
    pImuGps->vm.j=msg->linear_acceleration.y * TS;
    pImuGps->vm.k=msg->linear_acceleration.z * TS;
    if (aligncnt < Aligni0T * FRQ)            // Initial alignment using inertial frame i0 method
    {
        aln.Update(&pImuGps->wm, &pImuGps->vm, 1, TS);
        /*alignatt = q2att(aln.qnb);
        alignatt *= (180 / PI);
        aligntxt<<alignatt<<endl;
        ROS_INFO("alignyaw: [%lf]", alignatt.k);*/
    }
    else if ( aligncnt == Aligni0T * FRQ )   //Initial alignment finish
    {
        kf.lvOD = CVect3(1, 1, 0.1);
        kf.Init(CSINS(aln.qnb, O31, pos0, 0));
    }
    else                                     //kf-->time update
    {
        kf.Update(&pImuGps->wm, &pImuGps->vm, 1, TS);
    }
    aligncnt++;
}

void MeasureupdateCallback(const std_msgs::Float64::ConstPtr& msg)
{
    if ( aligncnt > ( Aligni0T * FRQ ) )
    {
        nav.header.frame_id = "nav";
        ros::Time currentTime = ros::Time::now();
        nav.header.stamp = currentTime;
        nav.header.seq = outcnt;
        double vd;
        //Set kf-->measurement && measure update
        vd = msg->data;
        kf.SetMeasOD(vd, TS);

        nav.pose.pose.orientation.w=kf.sins.qnb.q0;
        nav.pose.pose.orientation.x=kf.sins.qnb.q1;
        nav.pose.pose.orientation.y=kf.sins.qnb.q2;
        nav.pose.pose.orientation.z=kf.sins.qnb.q3;
        CVect3 enu=lla2enu(kf.posRes.i, kf.posRes.j, kf.posRes.k, latitude0, longtitude0, altitude0);
        nav.pose.pose.position.x=enu.i;
        nav.pose.pose.position.y=enu.j;
        nav.pose.pose.position.z=enu.k;

        nav_pub.publish(nav);

        outcnt++;
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "od_sins_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu", 1000, TimeupdateCallback);
    ros::Subscriber od_sub = nh.subscribe("/encoder", 1000, MeasureupdateCallback);

    nav_pub=nh.advertise<nav_msgs::Odometry>("nav",1000);

    ros::spin();

    return 0;
}


