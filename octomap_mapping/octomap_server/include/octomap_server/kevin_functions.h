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

void removeIfTwoValues(std::vector<float> &v){
  auto end = v.end();
  std::vector<int> doubleValues;
  std::vector<int>::iterator new_end;
  int counter=0;
  for (int i = 0;i<v.size();i++){
      for(int j = 0;j<v.size();j++){
          if(checkIfFloatsAreTheSame(v.at(i),v.at(j),100) && i!=j){
              doubleValues.push_back(counter);
              break;
          }
      }
      counter++;
  }
  for(int i=0;i<doubleValues.size();i++){
    v.erase(v.begin()+doubleValues.at(i)-i);
  }
}

int findIfVoxelCanBeSeen(float POIX,float POIY,float POIZ,double POIRes,pcl::PointCloud<pcl::PointXYZ>::Ptr freeData,std::vector<double> freeRes,pcl::PointCloud<pcl::PointXYZ>::Ptr occData,std::vector<double> occRes,float minRadius,float maxRadius,float UAVSize){
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints (new pcl::PointCloud<pcl::PointXYZ>);
  for(int i = 0; i<freeData->size();i++){
    float tempX = freeData->points[i].x;
    float tempY = freeData->points[i].y;
    float tempZ = freeData->points[i].z;
    int goodPoint = 0;
    std::vector<float> yPosMinMax;
    std::vector<float> yNegMinMax;
    std::vector<float> xPosMinMax;
    std::vector<float> xNegMinMax;
    if(tempZ-(freeRes[i]/2)<POIZ && POIZ<tempZ+(freeRes[i]/2)){
      if(tempX-(freeRes[i]/2)<POIX && POIX<tempX+(freeRes[i]/2)){
        if(tempY>POIY){
          if(tempY<POIY+maxRadius){
            yPosMinMax.push_back(tempY-(freeRes[i]/2));
            yPosMinMax.push_back(tempY+(freeRes[i]/2));
          }
          // TODO: Add code here to keep track of min and max of all free points that come in.
          // If there are only 2 then that means those are the min and max. If there are more than 2 that means the view is obstructed by an obstacle.
          // This will allow us to see whether or not there is line of sight and if there are usable voxels by taking the unique values of the set
          if(tempY-(freeRes[i]/2) > POIY+minRadius && tempY-(freeRes[i]/2)+UAVSize < POIY+maxRadius){
            goodPoint = 1;
          }
          else if(tempY+(freeRes[i]/2) < POIY+maxRadius && tempY+(freeRes[i]/2)-UAVSize > POIY+minRadius){
            goodPoint = 1;
          }
        }
        else if(tempY<POIY){
          if(tempY>POIY-minRadius){
            yNegMinMax.push_back(tempY-(freeRes[i]/2));
            yNegMinMax.push_back(tempY+(freeRes[i]/2));
          }
          if(tempY+(freeRes[i]/2) < POIY-minRadius && tempY+(freeRes[i]/2)-UAVSize > POIY-maxRadius){
            goodPoint = 1;
          }
          else if(tempY-(freeRes[i]/2) > POIY-maxRadius && tempY-(freeRes[i]/2)+UAVSize < POIY-minRadius){
            goodPoint = 1;
          }
        }
      }
      else if(tempY-(freeRes[i]/2)<POIY && POIY<tempY+(freeRes[i]/2)){
        if(tempX>POIX){
          if(tempX<POIX+maxRadius){
            xPosMinMax.push_back(tempX-(freeRes[i]/2));
            xPosMinMax.push_back(tempX+(freeRes[i]/2));
          }
          if(tempX-(freeRes[i]/2) > POIX+minRadius && tempX-(freeRes[i]/2)+UAVSize < POIX+maxRadius){
            goodPoint = 1;
          }
          else if(tempX+(freeRes[i]/2) < POIX+maxRadius && tempX+(freeRes[i]/2)-UAVSize > POIX+minRadius){
            goodPoint = 1;
          }
        }
        else if(tempX<POIX){
          if(tempX>POIX-minRadius){
            xNegMinMax.push_back(tempX-(freeRes[i]/2));
            xNegMinMax.push_back(tempX+(freeRes[i]/2));
          }
          if(tempX+(freeRes[i]/2) < POIX-minRadius && tempX+(freeRes[i]/2)-UAVSize > POIX-maxRadius){
            goodPoint = 1;
          }
          else if(tempX-(freeRes[i]/2) > POIX-maxRadius && tempX-(freeRes[i]/2)+UAVSize < POIX-minRadius){
            goodPoint = 1;
          }
        }
      }
    }
    removeIfTwoValues(yPosMinMax);
    removeIfTwoValues(yNegMinMax);
    removeIfTwoValues(xPosMinMax);
    removeIfTwoValues(xNegMinMax);
    if(yPosMinMax.size()<3){ //yPos has a possible value

    }
    if(yNegMinMax.size()<3){

    }
    if(xPosMinMax.size()<3){

    }
    if(xNegMinMax.size()<3){
      
    }
  }
  /*
  Here we take as input the center of a voxel and check if it can be seen by the free area.
  We need to check all 4 directions (+x,-x,+y,-y) and see if there is enough free space between the min-max range.
  Also need to check if line of sight is free and if UAV is far enough away from all obstacles in the environmnet.
  - Check 4 directions for free space
  - Check line of sight is free
  - Check if UAV will not crash into environment
  */
}

