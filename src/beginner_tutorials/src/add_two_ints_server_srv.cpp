//
// Created by coswang on 24-10-9.
//
 #include "ros/ros.h"
 #include  "beginner_tutorials/my_srv.h"
 int k;

class qurr
        {
public:
    bool query(beginner_tutorials::my_srv::Request &req,
               beginner_tutorials::my_srv::Response &res);
        };

 bool query(beginner_tutorials::my_srv::Request& req,
            beginner_tutorials::my_srv::Response& res)
 {

   if (req.id == 74588)
   {
       res.name = "yhwang";
       res.gender = "male";
       res.age = k;
   }
    else
    {
       res.name = "bkwang";
       res.gender = "male";
       res.age = 40;
    }

//   ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
     ROS_INFO_STREAM("ID: " << req.id << " Name: " << res.name << " Gender: " << res.gender << " Age: " << res.age);
   return true;
 }

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "query_info_server"); //初始化node
   ros::NodeHandle n; // node handler
   beginner_tutorials::my_srv srv;
   qurr a;
   n.param("my_num", k, 30);

   ros::ServiceServer service = n.advertiseService("query_info", &qurr::query, &a); //定義service server以及callback function
   ROS_INFO("Ready to query info.");
   ros::spin(); //持續運行此node

  return 0;
 }