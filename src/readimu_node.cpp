#include <ros/ros.h>
#include <string>
#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <unistd.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <cstdlib>
#include "KFApp.h"
#include "PSINS.h"
#include "odsins.h"

using namespace std;

void LoadImu(const string &trajectory, vector<string> &imu)
{
    ifstream fTimes;
    //read data file
    fTimes.open(trajectory.c_str());
    imu.reserve(5000);

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            imu.push_back(s);
        }
    }
}

vector<string> split(const string &str, const string &pattern)
{
    vector<string> res;
    if ("" == str)
        return res;

    string strs = str + pattern;

    size_t pos = strs.find(pattern);
    size_t size = strs.size();

    while (pos != string::npos)
    {
        string x = strs.substr(0, pos);
        res.push_back(x);
        strs = strs.substr(pos + 1, size);
        pos = strs.find(pattern);
    }
    return res;
}

void Loadoneline(vector<string> &imu, ImuGpsData *pImuGps, int i)
{
    string gyrox, gyroy, gyroz, accx, accy, accz, t, odom;
    vector<string> res=split(imu[i],"\t");
    gyrox=res[0];
    pImuGps->wm.i = atof(gyrox.c_str());
    gyroy=res[1];
    pImuGps->wm.j = atof(gyroy.c_str());
    gyroz=res[2];
    pImuGps->wm.k = atof(gyroz.c_str());
    accx=res[3];
    pImuGps->vm.i = atof(accx.c_str());
    accy=res[4];
    pImuGps->vm.j = atof(accy.c_str());
    accz=res[5];
    pImuGps->vm.k = atof(accz.c_str());
    t=res[6];
    pImuGps->t=atof(t.c_str());
    odom=res[7];
    pImuGps->odom=atof(odom.c_str());
}

ros::Publisher Publisher_imu;
ros::Publisher Publisher_od;

void Run_IMU()
{
    vector<string> imudata;
    LoadImu("/home/hx/od_sins_realtime/imu.txt",imudata);    //Replace by your path
    ImuGpsData *pImuGps = new ImuGpsData();

    try
    {
        ros::NodeHandle n;
        Publisher_imu = n.advertise<sensor_msgs::Imu>("imu", 1000);
        Publisher_od = n.advertise<std_msgs::Float64>("encoder", 1000);
        int nCnt = 0;
        ros::Rate r(125);
        while (ros::ok())
        {
            try
            {
                //----------------ROS Publish IMU Topics----------------------------
                Loadoneline(imudata, pImuGps, nCnt);
                float roll, pitch, yaw;
                roll = 0.0;
                pitch = 0.0;
                yaw = 0.0;
                geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
                geometry_msgs::Vector3 angular_velocity;
                angular_velocity.x = pImuGps->wm.i * FRQ;
                angular_velocity.y = pImuGps->wm.j * FRQ;
                angular_velocity.z = pImuGps->wm.k * FRQ;
                geometry_msgs::Vector3 linear_acceleration;
                linear_acceleration.x = pImuGps->vm.i * FRQ;
                linear_acceleration.y = pImuGps->vm.j * FRQ;
                linear_acceleration.z = pImuGps->vm.k * FRQ;

                sensor_msgs::Imu ImuData;
                ImuData.header.stamp = ros::Time::now();
                ImuData.header.frame_id = "frame_imu";
                ImuData.orientation = quat;
                ImuData.angular_velocity = angular_velocity;
                ImuData.linear_acceleration = linear_acceleration;
                ImuData.header.seq = nCnt;

                std_msgs::Float64 odom;
                odom.data=pImuGps->odom;

                Publisher_imu.publish(ImuData);
                Publisher_od.publish(odom);
                r.sleep();
                ros::spinOnce();
                ++nCnt;
            }
            catch (boost::system::system_error& e)
            {
                if (e.code().message() == "Interrupted system call")
                {  // ctrl+c
                    ros::shutdown();
                }
                else
                {
                    ROS_ERROR("ReadData Error: %s" ,e.what());
                }
            }
        }
        ros::spinOnce();
    }
    catch (std::exception& e)
    {
        ROS_ERROR("Run_IMU_Integrated_Nav Error: %s" ,e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "readimu_node");
    Run_IMU();
    return 0;
}
