#include <ros/ros.h>
#include <rosbag/bag.h>
#include <stdio.h>
#include "stdlib.h"
#include <string>
#include <math.h>
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <iomanip>
#include <sys/time.h>
#include <boost/lambda/lambda.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <unistd.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <cstdlib>
#include "calibration.h"
#include "zupt.h"
#include "strapdown.h"
#include "KFApp.h"
#include "PSINS.h"

using namespace std;

void LoadImu(const string &trajectory, vector<string> &imu)
{
    ifstream fTimes;
    // 数据流读取文件
    fTimes.open(trajectory.c_str());
    imu.reserve(5000);

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            // stringstream ss;
            // ss << s;
            //ROS_INFO("%s",s.c_str());
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
        res.push_back(x);//stoi(x)转整型
        strs = strs.substr(pos + 1, size);
        pos = strs.find(pattern);
    }
    return res;
}

void Loadoneline(vector<string> &imu, ImuGpsData *pImuGps, int i)
{
//    if (i > (imu.size() - 1)){
//        break;
//    }

    string gyrox, gyroy, gyroz, accx, accy, accz, t, odom;
    //char *strimu;
    // for (int j = 0; j < vposition.size(); j++)
    // {
    vector<string> res=split(imu[i],"\t");
    /*gyrox = imu[i].substr(0, 14); //substr就是截取字符，20指的是在第几个字符上，13从第20个数据向后截取的长度。
    pImuGps->wm.i = atof(gyrox.c_str());
    //X.push_back(atof(x.c_str())); //string类型转化为double型分为两步，先将string转为字符数组，然后数组转为double。
    //c_str将string转为数组，atof将数组转为double。
    gyroy = imu[i].substr(14, 14);
    pImuGps->wm.j = atof(gyroy.c_str());

    gyroz = imu[i].substr(32, 16);
    pImuGps->wm.k = atof(gyroz.c_str());

    accx = imu[i].substr(48, 16);
    pImuGps->vm.i = atof(accx.c_str());

    accy = imu[i].substr(64, 16);
    pImuGps->vm.j = atof(accy.c_str());

    accz = imu[i].substr(80, 16);
    pImuGps->vm.k = atof(accz.c_str());

    t = imu[i].substr(96, 16);
    pImuGps->t=atof(t.c_str());

    odom = imu[i].substr(112, 12);
    pImuGps->odom=atof(odom.c_str());*/
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

/*bool Open_IMU_Serial(std::string& rsPort, int nBaudRate, const std::string& rsTopic, const std::string rsFrame)
{
    using namespace boost::asio;
    io_service io;
    bool opened_serial_port = false;
    char chrBuffer[2000];
    int try_counter = 0;
    char arhs_cmd_hex_code_[5];
    arhs_cmd_hex_code_[0] = 0x55;
    arhs_cmd_hex_code_[1] = 0xb4;
    arhs_cmd_hex_code_[2] = 0x00;
    arhs_cmd_hex_code_[3] = 0x01;
    arhs_cmd_hex_code_[4] = '\0';
    while(!opened_serial_port && ros::ok())
    {
        ROS_INFO("Waiting open serial port.");
        try
        {
            serial_port port(io, rsPort);
            port.set_option(serial_port_base::baud_rate(nBaudRate));
            port.set_option(serial_port_base::character_size(8));
            port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
            port.set_option(serial_port_base::parity(serial_port_base::parity::none));
            port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            boost::system::error_code ec;

            port.write_some(boost::asio::buffer(arhs_cmd_hex_code_),ec);

            if(ec)
            {
                ROS_ERROR("write_some err,errmessage: %s",ec.message().c_str());
                continue;
            }
            break;
        }
        catch (boost::system::system_error& e)
        {
            if(try_counter > 10)
            {
                ROS_ERROR("Failed to open serial port.");
                return false;
            }
            if(rsPort == "/dev/ttyUSB0")
            {
                rsPort = "/dev/ttyUSB1";
            }
            else
            {
                rsPort = "/dev/ttyUSB0";
            }
            opened_serial_port = false;
            try_counter++;
            ros::Duration(0.1).sleep();
        }
    }

    ROS_INFO("Opened %s", rsPort.c_str());
    return true;
}*/

void Run_IMU()
{
    /* using namespace boost::asio;
    char chrBuffer[2000];*/
    vector<string> imudata;
    LoadImu("/home/hx/od_sins_realtime/imu.txt",imudata);
    ImuGpsData *pImuGps = new ImuGpsData();

    try
    {
        ros::NodeHandle n;
        Publisher_imu = n.advertise<sensor_msgs::Imu>("fogtopic", 1000);
        Publisher_od = n.advertise<std_msgs::Float64>("od", 1000);
        /*io_service io;
        serial_port port(io, rsPort);
        port.set_option(serial_port_base::baud_rate(nBaudRate));
        port.set_option(serial_port_base::character_size(8));
        port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
        port.set_option(serial_port_base::parity(serial_port_base::parity::none));
        port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

        boost::array<unsigned char, 2000> DataBuf;*/
        int nCnt = 0;
        ros::Rate r(125);
        while (ros::ok())
        {
            try
            {
                /*size_t usLength = boost::asio::read(port, boost::asio::buffer(DataBuf), boost::asio::transfer_at_least(1));
                boost::array<unsigned char, 2000>::iterator itr = DataBuf.begin();
                for (int i = 0; itr != DataBuf.end(); ++itr, ++i)
                {
                    chrBuffer[i] = *itr;
                    //std::cout << *itr << std::endl;
                }
                ST_AHRS_DATA AHRS;
                ST_AHRS_RAW_DATA RAW_AHRS;
                int get_data = 0;
                if (usLength > 0)
                {
                    CopeSerialData54(chrBuffer, usLength, AHRS, RAW_AHRS, get_data);
                }*/
                //----------------ROS Publish IMU Topics----------------------------
                Loadoneline(imudata, pImuGps, nCnt);
                float roll, pitch, yaw;
                /*roll = AHRS.st_euler_data.f_roll;
                pitch = AHRS.st_euler_data.f_pitch;
                yaw = AHRS.st_euler_data.f_yaw;*/
                roll = 0;
                pitch = 0;
                yaw = 0;
                geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
                geometry_msgs::Vector3 angular_velocity;
                /*angular_velocity.x = RAW_AHRS.st_gyro_data.f_axis_x * PI / 180;
                angular_velocity.y = RAW_AHRS.st_gyro_data.f_axis_y * PI / 180;
                angular_velocity.z = RAW_AHRS.st_gyro_data.f_axis_z * PI / 180;*/
                angular_velocity.x = pImuGps->wm.i * FRQ;
                angular_velocity.y = pImuGps->wm.j * FRQ;
                angular_velocity.z = pImuGps->wm.k * FRQ;
                geometry_msgs::Vector3 linear_acceleration;
                /*linear_acceleration.x = RAW_AHRS.st_acc_data.f_axis_x;
                linear_acceleration.y = RAW_AHRS.st_acc_data.f_axis_y;
                linear_acceleration.z =  RAW_AHRS.st_acc_data.f_axis_z;*/
                linear_acceleration.x = pImuGps->vm.i * FRQ;
                linear_acceleration.y = pImuGps->vm.j * FRQ;
                linear_acceleration.z = pImuGps->vm.k * FRQ;

                sensor_msgs::Imu ImuData;
                ImuData.header.stamp = ros::Time::now();
                //ImuData.header.stamp = ;
                ImuData.header.frame_id = "fogimu";
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
        ROS_ERROR("Run_GPS_IMU Error: %s" ,e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "readimu_node");
    Run_IMU();
    return 0;
}
