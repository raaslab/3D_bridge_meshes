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

void printData(octomap::OcTree tree){
  std::cout << "Number of leaf nodes: " << tree.getNumLeafNodes() << std::endl;
  std::cout << "Resolution: " << tree.getResolution() << std::endl;
  std::cout << "Root node: " << tree.getRoot() << std::endl;
  std::cout << "Tree depth: " << tree.getTreeDepth() << std::endl;
  std::cout << "Tree type: " << tree.getTreeType() << std::endl;
  std::cout << "Memory full grid: " << tree.memoryFullGrid() << std::endl;
  std::cout << "Memory usage: " << tree.memoryUsage() << std::endl;
  std::cout << "Memory usage node: " << tree.memoryUsageNode() << std::endl;
  std::cout << "Size: " << tree.size() << std::endl;
  std::cout << "Volume: " << tree.volume() << std::endl;
  std::cout << std::endl;
}

void printVector(std::string str, std::vector<int> vector2Print){
  for(int j = 0; j<vector2Print.size(); j++){
    std::cout << str << vector2Print.at(j);
  }
  std::cout << std::endl;
}

int radiusSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, pcl::PointXYZ searchPoint, float radius){ // checks if the points in "pointCloud" are in the "radius" of "searchPoint"
  float x1; float y1; float z1; float x2; float y2; float z2; float x; float y; float z; float sumOfAll; float distance; float counter=0;
  x2 = searchPoint.x; y2 = searchPoint.y; z2 = searchPoint.z;
  for(int i = 0; i<pointCloud->size();i++){
    x1 = pointCloud->points[i].x; y1 = pointCloud->points[i].y; z1 = pointCloud->points[i].z;
    x = x2-x1; y = y2-y1; z = z2-z1;
    sumOfAll = pow(x,2) + pow(y,2) + pow(z,2);
    distance = sqrt(sumOfAll);
    if(distance <= radius){
      counter = counter+1;
    }
  }
  return counter;
}

int checkIfPointsAreTheSame(pcl::PointXYZ point1, pcl::PointXYZ point2){ // checks if point1 == point2
  int output = 0;
  if(point1.x == point2.x && point1.y == point2.y && point1.z == point2.z){
    output = 1;
  }
  return output;
}

int checkIfFloatsAreTheSame(float num1, float num2, int precision){
  int new1 = num1*precision;
  int new2 = num2*precision;
  int output;
  if(new1 == new2){
    output = 1;
  }
  else{
    output = 0;
  }
  return output;
}
