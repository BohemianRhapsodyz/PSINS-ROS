//
// Created by hx on 2021/3/9.
//
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
#include "odsins.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_pub_node");
    ros::NodeHandle hn;
    ros::Publisher nav_pub=hn.advertise<geometry_msgs::PoseStamped>("nav",1000);
    ros::Rate loop_rate(125);
    while (ros::ok())
    {
        //if( aligncnt > ( Aligni0T * FRQ + 10 ) )
        //{
            nav.header.frame_id = "nav";
            ros::Time currentTime = ros::Time::now();
            nav.header.stamp = currentTime;
            nav.header.seq = outcnt;

            ROS_INFO("high: [%lf]", nav.pose.position.z);
            //nav_pub.publish(nav);
            ros::spinOnce();
            loop_rate.sleep();

            outcnt++;
        //}
    }
    return 0;
}