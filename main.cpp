#include <iostream>
#include <chrono>
#include <thread>
//#include <ros/ros.h>
//#include <std_msgs/String.h>

extern std::string cmd_body();

using namespace std;

int main( int argc, char** argv )
{
////创建节点
//    ros::init(argc, argv, "send_cmd");
//    ros::NodeHandle n;
//
////发布话题
//    ros::Publisher talkerPub = n.advertise<std_msgs::String>("/cerebellum_control", 1);
//
//
//    while(ros::ok())
//    {
//        std_msgs::String msg;
//
//        std::string cmd = cmd_body();
//        msg.data = cmd;         // String 转为 str 作为消息体的数据
//
//        ROS_INFO("%s", msg.data.c_str());
//
//        talkerPub.publish(msg);
//
//        ros::spinOnce();
//
//        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//    }
    string a = cmd_body();
//    cout << a;
    return 0;
}