#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>






float rF = 0; //reset flag
geometry_msgs::Pose currentPose;
sensor_msgs::PointCloud2 visitedPointsList;

// sensor_msgs::PointCloud2 currentPose; //pose message

void resetFlag_cb(const std_msgs::Float64& msg){ //TODO: CHECK IF THIS WORKS
  rF = msg.data;
  ROS_INFO("I heard: [%f]", msg.data);
}

void imu_cb(const geometry_msgs::PoseStamped& msg){ // imu call back
  pcl::PointXYZ tempPoint(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
  currentPose = msg.pose;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "visitedPointPublisher");
  ros::NodeHandle n;
  ros::Publisher pointList = n.advertise<sensor_msgs::PointCloud2>("/visited_point_list",1); //TODO: CHECK IF THIS WORKS
  ros::Subscriber resetFlag = n.subscribe("/resetFlag", 1, resetFlag_cb);
  ros::Subscriber uavIMU_sub = n.subscribe("/ground_truth_to_tf/pose",1,imu_cb); //TODO: CHECK IF THIS WORKS
  ros::Rate loop_rate(1);
  // std::vector<geometry_msgs/points> visitedPointsList;

  while (ros::ok()){
    ros::spinOnce();
    if(!rF){
      visitedPointsList.data; //TODO: CHECK IF THIS WORKS
      // pointList.publish(visitedPointsList); //TODO: CHECK IF THIS WORKS
      // add (x,y,z) points to vector
      // publish vector
    }
    else{
      // visitedPointsList.clear(); //TODO: CHECK IF THIS WORKS
    }
    loop_rate.sleep();
  }
  return 0;
}
