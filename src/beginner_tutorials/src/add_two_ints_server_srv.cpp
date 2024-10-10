//
// Created by coswang on 24-10-9.
//

#include "add_two_ints_server.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
// 引用 ros.h 檔

int main(int argc, char** argv){
    ros::init(argc, argv, "hello_cpp_node");     // 初始化hello_cpp_node
    ros::NodeHandle handler;                     // node 的 handler
    while (ros::ok()){                           // 在 ros 順利執行時
        ROS_INFO("Hello World!");                // 印出 Hello World
        ros::Duration(1).sleep();                // 間隔 1 秒
    }
}