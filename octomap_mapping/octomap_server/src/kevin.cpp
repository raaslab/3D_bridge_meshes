#define _USE_MATH_DEFINES
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
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
#include "../include/octomap_server/kevin_functions.h"
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <hector_moveit_navigation/NavigationAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <gtsp/Tour.h>
#include <gtsp/GTSPData.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <string>
#include <list>
#include <set>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <std_msgs/Float64MultiArray.h>


#define MAX_TOUR_SIZE 25


// global variables
int resolution = 1;
octomap::OcTree* fullOcTree = new octomap::OcTree(resolution);
octomap::OcTree* trimmedOcTree = new octomap::OcTree(resolution);
int countCB = 0;
int fileSave = 0;
int markerSize = 1;
float xData=0.0;
float yData=0.0;
float zData=0.0;
int positionCount = 0;
std::vector<int> tour;
double moveit_distance = -1;
bool tour_ready = false;
bool length_ready = false;
geometry_msgs::Pose currentPose;
pcl::PointCloud<pcl::PointXYZ>::Ptr runningVisitedVoxels (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloudOccTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<double> tempzFilteredSize;


void tourCallback(const gtsp::Tour::ConstPtr& msg){
  ROS_INFO("Got tour");
  tour = msg->tour;
  std::string output = "TOUR: ";
  for(int i = 0; i < tour.size(); i++)
  {
    output.append(std::to_string(tour[i]));
    output.append(" ");
  }
  ROS_INFO("%s", output.c_str());
  tour_ready = true;
}

void lengthCallback(const std_msgs::Float64::ConstPtr& msg){
  ROS_INFO("Got length");
  moveit_distance = msg->data;
  ROS_INFO("MoveIt Distance: %f", moveit_distance);
  length_ready = true;
}

void full_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
  // std::cout << "Octomap binary tree call back number:" << countCB << std::endl;
  octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);
  fullOcTree = dynamic_cast<octomap::OcTree*>(absTree);
  // countCB++;
}

void trimmed_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
  // std::cout << "Octomap binary tree call back number:" << countCB << std::endl;
  octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);
  trimmedOcTree = dynamic_cast<octomap::OcTree*>(absTree);
  // countCB++;
}

void imu_cb(const geometry_msgs::PoseStamped& msg){ // imu call back
  pcl::PointXYZ tempPoint(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
  currentPose = msg.pose;
}

void zFiltered_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
  tempCloudOccTrimmed->clear();
  BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points){
    tempCloudOccTrimmed->points.push_back(pcl::PointXYZ(pt.x,pt.y,pt.z));
  }
}

void zFilteredSize_cb(const std_msgs::Float64MultiArray& msg){
  tempzFilteredSize.clear();
  for(int i = 0;i<msg.data.size();i++){
    tempzFilteredSize.push_back(msg.data.at(i));
  }
}


int main(int argc, char** argv){
  std::ofstream myfile;
  myfile.open("/home/user01/testRunsBI/test1Info.txt");
  std::ofstream myfile1;
  myfile1.open("/home/user01/testRunsBI/test1Clusters.txt");
// initializing ROS everything
  ros::init(argc, argv, "kevin");
  actionlib::SimpleActionClient<hector_moveit_navigation::NavigationAction> ac("hector_navigator", true);
  ros::NodeHandle n;
  ros::Rate r(0.05); // less than 1 is slower
  ros::Publisher occArrayTrimmed_pub = n.advertise<visualization_msgs::MarkerArray>("/occArrayTrimmed_pub",1,true);
  ros::Publisher freeArrayTrimmed_pub = n.advertise<visualization_msgs::MarkerArray>("/freeArrayTrimmed_pub",1,true);
  ros::Publisher occArrayFull_pub = n.advertise<visualization_msgs::MarkerArray>("/occArrayFull_pub",1,true);
  ros::Publisher freeArrayFull_pub = n.advertise<visualization_msgs::MarkerArray>("/freeArrayFull_pub",1,true);
  ros::Publisher impossibleArray_pub = n.advertise<visualization_msgs::MarkerArray>("/impossibleArray_pub",1,true);
  ros::Publisher usableArray_pub = n.advertise<visualization_msgs::MarkerArray>("/usableArray_pub",1,true);
  ros::Publisher point_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("/gtsp_point_cloud", 1);
  ros::Publisher goal_distance_publisher = n.advertise<geometry_msgs::Point>("/compute_path/point", 1);
  ros::Publisher gtspData_pub = n.advertise<gtsp::GTSPData>("/gtsp_data", 1);

  ros::Subscriber fullTree_sub = n.subscribe("/octomap_full",1,full_cb);
  ros::Subscriber trimmedTree_sub = n.subscribe("/octomap_full_trimmed",1,trimmed_cb);
  ros::Subscriber uavIMU_sub = n.subscribe("/ground_truth_to_tf/pose",1,imu_cb);
  ros::Subscriber tour_sub = n.subscribe("/gtsp_tour_list", 1, tourCallback);
  ros::Subscriber distance_sub = n.subscribe("/compute_path/length", 1, lengthCallback);
  ros::Subscriber zFiltered_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/zFiltered",1,zFiltered_cb);
  ros::Subscriber zFilteredSize_sub = n.subscribe("/zFilteredSize", 1, zFilteredSize_cb);


// initializing arrays
  pcl::PointXYZ searchPoint;
  visualization_msgs::MarkerArray occArrayTrimmed;
  visualization_msgs::MarkerArray freeArrayTrimmed;
  visualization_msgs::MarkerArray occArrayFull;
  visualization_msgs::MarkerArray freeArrayFull;
  visualization_msgs::MarkerArray impossibleArray;
  visualization_msgs::MarkerArray usableArray;

// initializing variables and markers
  uint32_t shape = visualization_msgs::Marker::CUBE;
  int loopNumber = 0;
  float minRadius = 2;
  float maxRadius = 6;
  float thresholdOcc = 1.0;
  float thresholdFree = 0.0;
  float sizeOfUAV = 1.0;
  float tempID; int id4Markers; float markerSize;
  bool tspDone = false;
  int structureCovered = 0;
  pcl::PointXYZ searchPoint1;
  pcl::PointXYZ searchPoint2;
  octomap::OcTree::leaf_iterator it;
  octomap::OcTree::leaf_iterator endLeaf;
  myfile<<"minRadius: "<<minRadius<<std::endl;
  myfile<<"maxRadius: "<<maxRadius<<std::endl;
  myfile<<"sizeOfUAV: "<<sizeOfUAV<<std::endl;

  visualization_msgs::Marker fullOccupiedMarkers; fullOccupiedMarkers.header.frame_id = "/world"; fullOccupiedMarkers.header.stamp = ros::Time::now(); fullOccupiedMarkers.ns = "occupied_markers_full";
  fullOccupiedMarkers.type = shape; fullOccupiedMarkers.action = visualization_msgs::Marker::ADD;
  fullOccupiedMarkers.color.r = 1.0f; fullOccupiedMarkers.color.g = 0.0f; fullOccupiedMarkers.color.b = 0.0f; fullOccupiedMarkers.color.a = 1.0;
  visualization_msgs::Marker fullFreeMarkers; fullFreeMarkers.header.frame_id = "/world"; fullFreeMarkers.header.stamp = ros::Time::now(); fullFreeMarkers.ns = "free_markers_full";
  fullFreeMarkers.type = shape; fullFreeMarkers.action = visualization_msgs::Marker::ADD;
  fullFreeMarkers.color.r = 0.0f; fullFreeMarkers.color.g = 1.0f; fullFreeMarkers.color.b = 0.0f; fullFreeMarkers.color.a = 1.0;
  visualization_msgs::Marker fullUnknownMarkers; fullUnknownMarkers.header.frame_id = "/world"; fullUnknownMarkers.header.stamp = ros::Time::now(); fullUnknownMarkers.ns = "unknown_markers_full";
  fullUnknownMarkers.type = shape; fullUnknownMarkers.action = visualization_msgs::Marker::ADD;
  fullUnknownMarkers.color.r = 1.0f; fullUnknownMarkers.color.g = 1.0f; fullUnknownMarkers.color.b = 0.0f; fullUnknownMarkers.color.a = 1.0;
  visualization_msgs::Marker impossibleMarkers; impossibleMarkers.header.frame_id = "/world"; impossibleMarkers.header.stamp = ros::Time::now(); impossibleMarkers.ns = "impossible_markers";
  impossibleMarkers.type = shape; impossibleMarkers.action = visualization_msgs::Marker::ADD;
  impossibleMarkers.color.r = 0.4f; impossibleMarkers.color.g = 0.0f; impossibleMarkers.color.b = 0.8f; impossibleMarkers.color.a = 1.0;
  visualization_msgs::Marker usableMarkers; usableMarkers.header.frame_id = "/world"; usableMarkers.header.stamp = ros::Time::now(); usableMarkers.ns = "usable_markers";
  usableMarkers.type = shape; usableMarkers.action = visualization_msgs::Marker::ADD;
  usableMarkers.color.r = 0.0f; usableMarkers.color.g = 0.0f; usableMarkers.color.b = 1.0f; usableMarkers.color.a = 0.4;
  visualization_msgs::Marker trimmedOccupiedMarkers; trimmedOccupiedMarkers.header.frame_id = "/world"; trimmedOccupiedMarkers.header.stamp = ros::Time::now(); trimmedOccupiedMarkers.ns = "occupied_markers_trimmed";
  trimmedOccupiedMarkers.type = shape; trimmedOccupiedMarkers.action = visualization_msgs::Marker::ADD;
  trimmedOccupiedMarkers.color.r = 1.0f; trimmedOccupiedMarkers.color.g = 0.0f; trimmedOccupiedMarkers.color.b = 0.0f; trimmedOccupiedMarkers.color.a = 1.0;
  visualization_msgs::Marker trimmedFreeMarkers; trimmedFreeMarkers.header.frame_id = "/world"; trimmedFreeMarkers.header.stamp = ros::Time::now(); trimmedFreeMarkers.ns = "free_markers_trimmed";
  trimmedFreeMarkers.type = shape; trimmedFreeMarkers.action = visualization_msgs::Marker::ADD;
  trimmedFreeMarkers.color.r = 0.0f; trimmedFreeMarkers.color.g = 1.0f; trimmedFreeMarkers.color.b = 0.0f; trimmedFreeMarkers.color.a = 1.0;
  visualization_msgs::Marker trimmedUnknownMarkers; trimmedUnknownMarkers.header.frame_id = "/world"; trimmedUnknownMarkers.header.stamp = ros::Time::now(); trimmedUnknownMarkers.ns = "unknown_markers_trimmed";
  trimmedUnknownMarkers.type = shape; trimmedUnknownMarkers.action = visualization_msgs::Marker::ADD;
  trimmedUnknownMarkers.color.r = 1.0f; trimmedUnknownMarkers.color.g = 1.0f; trimmedUnknownMarkers.color.b = 0.0f; trimmedUnknownMarkers.color.a = 1.0;

  ros::Time totalRunTime = ros::Time::now();

  hector_moveit_navigation::NavigationGoal goal;
  ROS_INFO("Waiting for action server to start");
  ac.waitForServer();
  ROS_INFO("Action server started");
//taking off
  goal.goal_pose.position.x = 0;
  goal.goal_pose.position.y = 0;
  goal.goal_pose.position.z = 1;
  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = 0;
  goal.goal_pose.orientation.w = 1;
  ac.sendGoal(goal);
  
// checking takeoff
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  if(finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    // ROS_INFO("Takeoff finished: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Takeoff did not finish before the time out");
  goal.goal_pose.position.x = 0;
  goal.goal_pose.position.y = -1;
  goal.goal_pose.position.z = 3;
  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = 0;
  goal.goal_pose.orientation.w = 1;
  ac.sendGoal(goal);
  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  if(finished_before_timeout){
    actionlib::SimpleClientGoalState state = ac.getState();
    // ROS_INFO("Takeoff finished: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Takeoff did not finish before the time out");
  goal.goal_pose.position.x = 0;
  goal.goal_pose.position.y = -1;
  goal.goal_pose.position.z = 6;
  goal.goal_pose.orientation.x = 0;
  goal.goal_pose.orientation.y = 0;
  goal.goal_pose.orientation.z = 1;
  goal.goal_pose.orientation.w = 0;
  ac.sendGoal(goal);
  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  if(finished_before_timeout){
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Takeoff finished: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Takeoff did not finish before the time out");

  ROS_INFO("Finished all takeoff maneuvers");

  ROS_INFO("Waiting for gtsp_solver and distance_publisher to subscribe");
  ros::Rate poll_rate(100);
  while(point_cloud_publisher.getNumSubscribers() == 0)
    poll_rate.sleep();
  while(goal_distance_publisher.getNumSubscribers() == 0)
    poll_rate.sleep();

// problem is that GLNS will crash if empty set
  while (ros::ok()){
    ros::spinOnce();
// initializing variables
    id4Markers = 0; int countFreeFull = 0; int countOccFull = 0; int countUnknownFull = 0; markerSize = 0;
// getting sizes of free, occupied, and unknown for full octree
    for(it = fullOcTree->begin_leafs(),endLeaf = fullOcTree->end_leafs();it!=endLeaf;++it){
      markerSize = it.getSize();
      if(it->getValue()>thresholdOcc){
        fullOccupiedMarkers.pose.position.x = it.getX(); fullOccupiedMarkers.pose.position.y = it.getY(); fullOccupiedMarkers.pose.position.z = it.getZ();
        fullOccupiedMarkers.id = id4Markers;
        fullOccupiedMarkers.scale.x = markerSize; fullOccupiedMarkers.scale.y = markerSize; fullOccupiedMarkers.scale.z = markerSize;
        occArrayFull.markers.push_back(fullOccupiedMarkers);
        countOccFull = countOccFull + 1;
      }
      else if(it->getValue()<thresholdFree){
        fullFreeMarkers.pose.position.x = it.getX(); fullFreeMarkers.pose.position.y = it.getY(); fullFreeMarkers.pose.position.z = it.getZ();
        fullFreeMarkers.id = id4Markers;
        fullFreeMarkers.scale.x = markerSize; fullFreeMarkers.scale.y = markerSize; fullFreeMarkers.scale.z = markerSize;
        freeArrayFull.markers.push_back(fullFreeMarkers);
        countFreeFull = countFreeFull + 1;
      }
      else{
        fullUnknownMarkers.pose.position.x = it.getX(); fullUnknownMarkers.pose.position.y = it.getY(); fullUnknownMarkers.pose.position.z = it.getZ();
        fullUnknownMarkers.id = id4Markers;
        fullUnknownMarkers.scale.x = markerSize; fullUnknownMarkers.scale.y = markerSize; fullUnknownMarkers.scale.z = markerSize;
        countUnknownFull = countUnknownFull + 1;
      }
      id4Markers = id4Markers + 1;
    }
// getting sizes of free, occupied, and unknown for trimmed octree
    int countFreeTrimmed = 0; int countOccTrimmed = 0; int countUnknownTrimmed = 0; markerSize = 0;
    for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it){
      markerSize = it.getSize();
      if(it->getValue()>thresholdOcc){
        trimmedOccupiedMarkers.pose.position.x = it.getX(); trimmedOccupiedMarkers.pose.position.y = it.getY(); trimmedOccupiedMarkers.pose.position.z = it.getZ();
        trimmedOccupiedMarkers.id = id4Markers;
        trimmedOccupiedMarkers.scale.x = markerSize; trimmedOccupiedMarkers.scale.y = markerSize; trimmedOccupiedMarkers.scale.z = markerSize;
        occArrayTrimmed.markers.push_back(trimmedOccupiedMarkers);
        countOccTrimmed = countOccTrimmed + 1;
      }
      else if(it->getValue()<thresholdFree){
        trimmedFreeMarkers.pose.position.x = it.getX(); trimmedFreeMarkers.pose.position.y = it.getY(); trimmedFreeMarkers.pose.position.z = it.getZ();
        trimmedFreeMarkers.id = id4Markers;
        trimmedFreeMarkers.scale.x = markerSize; trimmedFreeMarkers.scale.y = markerSize; trimmedFreeMarkers.scale.z = markerSize;
        freeArrayTrimmed.markers.push_back(trimmedFreeMarkers);
        countFreeTrimmed = countFreeTrimmed + 1;
      }
      else{
        trimmedUnknownMarkers.pose.position.x = it.getX(); trimmedUnknownMarkers.pose.position.y = it.getY(); trimmedUnknownMarkers.pose.position.z = it.getZ();
        trimmedUnknownMarkers.id = id4Markers;
        trimmedUnknownMarkers.scale.x = markerSize; trimmedUnknownMarkers.scale.y = markerSize; trimmedUnknownMarkers.scale.z = markerSize;
        countUnknownTrimmed = countUnknownTrimmed + 1;
      }
      id4Markers = id4Markers + 1;
    }
// creating point clouds for free, occupied, and unknown voxels for full point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFreeFull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOccFull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudUnknownFull (new pcl::PointCloud<pcl::PointXYZ>);
    cloudFreeFull->width = countFreeFull; cloudFreeFull->height = 1; cloudFreeFull->points.resize (cloudFreeFull->width * cloudFreeFull->height);
    cloudOccFull->width = countOccFull; cloudOccFull->height = 1; cloudOccFull->points.resize (cloudOccFull->width * cloudOccFull->height);
    cloudUnknownFull->width = countUnknownFull; cloudUnknownFull->height = 1; cloudUnknownFull->points.resize (cloudUnknownFull->width * cloudUnknownFull->height);
    std::vector<double> freeSizeFull;
    std::vector<double> occSizeFull;
    countFreeFull = 0; countOccFull = 0; countUnknownFull = 0;
    for(it = fullOcTree->begin_leafs(),endLeaf = fullOcTree->end_leafs();it!=endLeaf;++it){
      if(it->getValue()>thresholdOcc){
        cloudOccFull->points[countOccFull].x = it.getX(); cloudOccFull->points[countOccFull].y = it.getY(); cloudOccFull->points[countOccFull].z = it.getZ();
        occSizeFull.push_back(it.getSize());
        countOccFull = countOccFull + 1;
      }
      else if(it->getValue()<thresholdFree){
        cloudFreeFull->points[countFreeFull].x = it.getX(); cloudFreeFull->points[countFreeFull].y = it.getY(); cloudFreeFull->points[countFreeFull].z = it.getZ();
        freeSizeFull.push_back(it.getSize());
        countFreeFull = countFreeFull + 1;
      }
      else{
        cloudUnknownFull->points[countUnknownFull].x = it.getX(); cloudUnknownFull->points[countUnknownFull].y = it.getY(); cloudUnknownFull->points[countUnknownFull].z = it.getZ();
        countUnknownFull = countUnknownFull + 1;
      }
    }
    
    ROS_INFO("\nRemoving previously seen voxels");
    std::cout<<"trimmed point cloud size: " << tempCloudOccTrimmed->size() << std::endl;
    std::cout<<"running visited voxel size: " << runningVisitedVoxels->size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOccTrimmed (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<double> zFilteredSize;
    int removeVoxel;
    for(int j=0;j<tempCloudOccTrimmed->size();j++){
      removeVoxel = 0;
      for(int i=0;i<runningVisitedVoxels->size();i++){
        if(tempCloudOccTrimmed->at(j).x==runningVisitedVoxels->at(i).x && tempCloudOccTrimmed->at(j).y==runningVisitedVoxels->at(i).y && tempCloudOccTrimmed->at(j).z==runningVisitedVoxels->at(i).z){
          removeVoxel = 1;
        }
      }
      if(removeVoxel == 0){
        cloudOccTrimmed->push_back(tempCloudOccTrimmed->at(j));
        zFilteredSize.push_back(tempzFilteredSize.at(j));
      }
    }
    std::cout<<"size after removing running visited from trimmed point cloud: " << cloudOccTrimmed->size() << std::endl;
    std::cout<<"size of zFilteredSize: " <<zFilteredSize.size()<<std::endl;

    float tempX1; float tempY1; float tempZ1; float tempX2; float tempY2; float tempZ2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusteredPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr viewPoints (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> point2ClusterMapping;
    std::vector<double> tempRes;
    int numOfCluster = 0;
    myfile1<<"===="<<loopNumber<<"===="<<std::endl;
    for(int j = 0; j<cloudOccTrimmed->size();j++){
      tempX1 = cloudOccTrimmed->points[j].x;
      tempY1 = cloudOccTrimmed->points[j].y;
      tempZ1 = cloudOccTrimmed->points[j].z;
      tempPoints->clear();
      findIfVoxelCanBeSeen(tempX1,tempY1,tempZ1,zFilteredSize.at(j),cloudFreeFull,freeSizeFull,minRadius,maxRadius,sizeOfUAV,tempPoints,&tempRes);
        myfile1<<j<<". view point: ";
        myfile1<<cloudOccTrimmed->points[j]<<std::endl;
      if(tempPoints->size()>0){
        viewPoints->push_back(cloudOccTrimmed->points[j]);
        numOfCluster++;
        myfile1<<"clustered points: ";
      }
      for(int i = 0; i<tempPoints->size();i++){
        clusteredPoints->push_back(tempPoints->points[i]);
        point2ClusterMapping.push_back(numOfCluster);
        myfile1<<tempPoints->points[i]<<" "<<tempRes[i]<<", ";
      }
      myfile1<<std::endl;
    }
    std::cout << "cloud free size: " << cloudFreeFull->size() << std::endl;
        
    if(numOfCluster<2){ // the structure has been covered
      ROS_INFO("No clusters");
      structureCovered = 1;
    }
    else{ // sending usable point cloud to Naik's code
      ROS_INFO("Sending usable graph to gtsp solver");
      gtsp::GTSPData gtsp_data;
      pcl::PCLPointCloud2 pcl_pc;
      pcl::toPCLPointCloud2(*clusteredPoints, pcl_pc);
      sensor_msgs::PointCloud2 usablePC2;
      pcl_conversions::fromPCL(pcl_pc, usablePC2);
      
      gtsp_data.points = usablePC2;
      gtsp_data.numClusters = numOfCluster;
      gtsp_data.pointClusterMapping = point2ClusterMapping;
      std::cout<<"Number of clusters: "<<numOfCluster<<"\nClustered points size: "<<clusteredPoints->size()<<"\nCluster mapping size: "<<point2ClusterMapping.size()<<std::endl;

      gtspData_pub.publish(gtsp_data);
      while(!tour_ready){
        ros::spinOnce();
        poll_rate.sleep();
      }
      tour_ready = false;
      ROS_INFO("Tour ready");
    }

// publishing voxel arrays
    occArrayTrimmed_pub.publish(occArrayTrimmed);
    freeArrayTrimmed_pub.publish(freeArrayTrimmed);
    occArrayFull_pub.publish(occArrayFull);
    freeArrayFull_pub.publish(freeArrayFull);

// executes tour based on GTSP output
    if(structureCovered == 1){ // going back to origin because no new viewpoints
      goal.goal_pose.position.x = 0;
      goal.goal_pose.position.y = 0;
      goal.goal_pose.position.z = 0;
      ROS_INFO("Moving to point: (%f,%f,%f)", goal.goal_pose.position.x,goal.goal_pose.position.y,goal.goal_pose.position.z);
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);
      finished_before_timeout = false;
      finished_before_timeout = ac.waitForResult(ros::Duration(60));
      if(finished_before_timeout){
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Last state reached: %s", state.toString().c_str());
        break;
      }
      else{
        ac.cancelAllGoals();
        ROS_INFO("Could not reach origin in time alloted");
        break;
      }
      ROS_INFO("Code done");
      break;
    }
    else{
      ros::Time startTime = ros::Time::now();
      ros::Duration elapsed = ros::Time::now()-startTime;
      pcl::KdTree<pcl::PointXYZ>::Ptr gtspTree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      gtspTree->setInputCloud(clusteredPoints);
      std::vector<int> nn_indices (numOfCluster);
      std::vector<float> nn_dists (numOfCluster);
      gtspTree->nearestKSearch(pcl::PointXYZ(currentPose.position.x, currentPose.position.y, currentPose.position.z), numOfCluster, nn_indices, nn_dists);
      std::cout<<std::endl;
      int currentPoint = 0;
      while(true){
        if(std::count(tour.begin(),tour.end(),nn_indices[currentPoint])){
          break;
        }
        else{
          currentPoint++;
        }
      }
      int countMoveit = 0;
      int orientationStatus = 0;
      tspDone = false;
      int currentPointNumber = -1;
      for(int j=0;j<tour.size();j++){
        if(tour.at(j)==nn_indices[currentPoint]){
          currentPointNumber = j;
          break;
        }
      }

      ROS_INFO("\nStart index in tour: %d\nStart point in tour: %d",currentPointNumber,tour[currentPointNumber]);
      while(elapsed.sec<60 && !tspDone){
        // goal point for moveit
        goal.goal_pose.position.x = clusteredPoints->points[tour[currentPointNumber]-1].x;
        goal.goal_pose.position.y = clusteredPoints->points[tour[currentPointNumber]-1].y;
        goal.goal_pose.position.z = clusteredPoints->points[tour[currentPointNumber]-1].z;
        // ROS_INFO("\nCurrent index in tour: %d\nCurrent point in tour: %d\nMoving to point: (%f,%f,%f)",currentPointNumber,tour[currentPointNumber],goal.goal_pose.position.x,goal.goal_pose.position.y,goal.goal_pose.position.z);
        std::cout<<"Current index in tour: "<<currentPointNumber<<std::endl;
        std::cout<<"Current point in tour: "<<tour[currentPointNumber]<<std::endl;
        std::cout<<"Moving to point: ("<<goal.goal_pose.position.x<<","<<goal.goal_pose.position.y<<","<<goal.goal_pose.position.z<<")"<<std::endl;
        tf2::Quaternion UAVOrientation;
        switch(orientationStatus){
          case 0:
            UAVOrientation.setRPY(0,0,0);
            orientationStatus++;
            break;
          case 1:
            UAVOrientation.setRPY(0,0,M_PI_2);
            orientationStatus++;
            break;
          case 2:
            UAVOrientation.setRPY(0,0,M_PI);
            orientationStatus++;
            break;
          case 3:
            UAVOrientation.setRPY(0,0,-M_PI_2);
            orientationStatus = 0;
            break;
          default:
            ROS_INFO("Error: goal orientation is not working");
        }
        goal.goal_pose.orientation.x = UAVOrientation.x();
        goal.goal_pose.orientation.y = UAVOrientation.y();
        goal.goal_pose.orientation.z = UAVOrientation.z();
        goal.goal_pose.orientation.w = UAVOrientation.w();

        // ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        finished_before_timeout = false;
        finished_before_timeout = ac.waitForResult(ros::Duration(30));
        if(finished_before_timeout){ // waiting for 30 seconds to reach goal before sending next point
          actionlib::SimpleClientGoalState state = ac.getState();
          int clusterNumber = point2ClusterMapping.at(tour[currentPointNumber]-1);
          pcl::PointXYZ tempPoint(viewPoints->at(clusterNumber-1).x,viewPoints->at(clusterNumber-1).y,viewPoints->at(clusterNumber-1).z);
          runningVisitedVoxels->push_back(tempPoint);
          ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else{
          ac.cancelAllGoals();
          ROS_INFO("Action did not finish before the time out.");
        }
        if(currentPointNumber >= tour.size()-1){ // check if the current tour point is the last one and loop
          currentPointNumber = 0;
        }
        else{
          currentPointNumber++;
        }
        if(tour.size()-1 == countMoveit){ // check if all tour points have been visited
          tspDone = true;
        }
        else{
          countMoveit++;
          elapsed = ros::Time::now()-startTime;
        }
      }
    }

    std::cout<<std::endl;
    ROS_INFO("Clearing all voxel maps stored");
    occArrayTrimmed.markers.clear();
    freeArrayTrimmed.markers.clear();
    occArrayFull.markers.clear();
    freeArrayFull.markers.clear();
    impossibleArray.markers.clear();
    usableArray.markers.clear();

    std::cout<<"=============loop number: "<<loopNumber+1<<"========================"<<std::endl; // outputting loop number
    loopNumber++;
    r.sleep();
  }

  ros::Duration elapsedTotalTime = ros::Time::now()-totalRunTime;
  ROS_INFO("Total run time: %d seconds",elapsedTotalTime.sec);
  myfile.close();
  myfile1.close();
  return 0;
}
