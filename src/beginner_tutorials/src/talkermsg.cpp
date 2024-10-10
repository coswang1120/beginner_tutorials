 #include "ros/ros.h"
 #include "beginner_tutorials/my_msg.h"

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "talker");
   ros::NodeHandle n;
   ros::Publisher chatter_pub = n.advertise<beginner_tutorials::my_msg>("chatter", 1000);
   ros::Rate loop_rate(10);
  int count = 0;
   while (ros::ok())
   {
     beginner_tutorials::my_msg msg;
     msg.id = count;
     msg.title = "hello";
     msg.content = "hello from c++";
     ROS_INFO_STREAM("ID: " << msg.id << " Title: " << msg.title << " Content: " << msg.content);
     chatter_pub.publish(msg);
     ros::spinOnce();
     loop_rate.sleep();
     ++count;
   }
   return 0;
 }