#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

void resetFlag(const bool& msg)
{
  ROS_INFO("I heard: [%s]", msg->data);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "visitedPointPublisher");
  ros::NodeHandle n;
  // ros::Publisher pointList = n.advertise<std_msgs::String>("/visited_point_list",1000);
  ros::Subscriber resetFlag = n.subscribe("resetFlag", 1, resetFlag);
  ros::Rate loop_rate(10);
  while (ros::ok()){
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pointList.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
