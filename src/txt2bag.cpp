#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/PointStamped.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
using namespace std;

void LoadImages(const string &trajectory, vector<string> &position_x)
{
    ifstream fTimes;
    // 数据流读取文件
    fTimes.open(trajectory.c_str());
    position_x.reserve(5000);

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            // stringstream ss;
            // ss << s;
            //ROS_INFO("%s",s.c_str());
            position_x.push_back(s);
        }
    }
}


int main(int argc, char **argv){

    vector<string> vposition;

    // LoadImages(string(argv[1]),vposition);
    LoadImages("/home/hx/od_sins/src/imu.txt",vposition);

    ros::init(argc,argv,"publish_02"); //初始化节点
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PointStamped>("clicked_point",1); //定义发送端
    ros::Rate loop_rate(10); //频率
    //vector<string>::iterator i = vposition.begin();
    //auto i = vposition.begin();

    geometry_msgs::PointStamped msg;
    msg.header.frame_id = "map";
    // int num = vposition.size();

    // printf("num = %d \n",num);
    int i = 0;
    while (ros::ok()){
        // if (i == (vposition.end() - 5)){
        if (i > (vposition.size() - 1)){
            break;
        }


        string x, y, z;

        // for (int j = 0; j < vposition.size(); j++)
        // {
        x = vposition[i].substr(0, 5); //substr就是截取字符，20指的是在第几个字符上，13从第20个数据向后截取的长度。

        msg.point.x = atof(x.c_str());
        //X.push_back(atof(x.c_str())); //string类型转化为double型分为两步，先将string转为字符数组，然后数组转为double。
        //c_str将string转为数组，atof将数组转为double。
        y = vposition[i].substr(6, 11);
        msg.point.y = atof(y.c_str());

        msg.point.z = 0;

        chatter_pub.publish(msg);

        ROS_INFO("i = %d \n",i);
        i = i + 1;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

