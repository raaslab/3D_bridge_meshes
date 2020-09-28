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
pcl::PointCloud<pcl::PointXYZ>::Ptr visitedPointsList (new pcl::PointCloud<pcl::PointXYZ>); //TODO: check this

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
  ros::Publisher pointList_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/visited_point_list",1,true); //TODO: CHECK THIS
  ros::Subscriber resetFlag = n.subscribe("/resetFlag", 1, resetFlag_cb);
  ros::Subscriber uavIMU_sub = n.subscribe("/ground_truth_to_tf/pose",1,imu_cb);
  ros::Rate loop_rate(1);
  visitedPointsList->header.frame_id = "/world";
  ROS_INFO("visitedPoints_publisher");
  int count = 0;
  while (ros::ok()){
    ros::spinOnce();
    ROS_INFO("while loop: %d", count);
    visitedPointsList->width = count+1; visitedPointsList->height = 1; visitedPointsList->points.resize (visitedPointsList->width * visitedPointsList->height);
    if(!rF){
      visitedPointsList->points[count].x = currentPose.position.x; visitedPointsList->points[count].y = currentPose.position.y; visitedPointsList->points[count].z = currentPose.position.z; //TODO:check this
      if(visitedPointsList->size()){
        pointList_pub.publish(visitedPointsList);
        for(int i=0;i<visitedPointsList->size();i++){
          std::cout<<visitedPointsList->at(i)<<std::endl;
        }
      }
      count++;
    }
    else{
      ROS_INFO("Else");
      visitedPointsList->clear(); //TODO:check this
      count = 0;
      // visitedPointsList.clear(); //TODO: CHECK IF THIS WORKS
    }
    loop_rate.sleep();
  }
  return 0;
}
