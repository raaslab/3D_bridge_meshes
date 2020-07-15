#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <pcl/octree/octree_search.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <pcl/io/pcd_io.h>
#include "/home/user01/catkin_ws/src/octomap_mapping/octomap_server/include/octomap_server/kevin_functions.h"
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <hector_moveit_navigation/NavigationAction.h>
// #include "sensor_msgs/msg/NavSatFix"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

int main(int argc, char** argv){
	float num1 = 2.234;
	float num2 = 2.245;
	float minRadius = 8.0;
	float maxRadius = 50.1;
	float answer1 = abs(num1-num2);
	float answer2 = abs(num2-num1);
	std::cout<<checkIfFloatsAreTheSame(num1, num2, 100)<<std::endl;
}