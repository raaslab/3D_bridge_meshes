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
#include <algorithm>

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

void removeAboveValue(std::vector<float> vector,float upperBound,std::vector<float>* outputVector){
  for(int i = 0;i<vector.size();i++){
    if(vector.at(i)>upperBound){
      vector.erase(vector.begin()+i);
      i--;
    }
  }
  *outputVector = vector;
}

void removeBelowValue(std::vector<float> vector,float lowerBound,std::vector<float>* outputVector){
  for(int i = 0;i<vector.size();i++){
    if(vector.at(i)<lowerBound){
      vector.erase(vector.begin()+i);
      i--;
    }
  }
  *outputVector = vector;
}

void findIfVoxelCanBeSeen(float POIX,float POIY,float POIZ,float POIRes,pcl::PointCloud<pcl::PointXYZ>::Ptr freeData,std::vector<double> freeRes,float minRadius,float maxRadius,float UAVSize,pcl::PointCloud<pcl::PointXYZ>::Ptr outputPoints,std::vector<double>* outputRes){
  std::vector<float> yPosMinMax;
  std::vector<float> yNegMinMax;
  std::vector<float> xPosMinMax;
  std::vector<float> xNegMinMax;
  for(int i = 0; i<freeData->size();i++){
    float tempX = freeData->points[i].x;
    float tempY = freeData->points[i].y;
    float tempZ = freeData->points[i].z;
    int goodPoint = 0;
    // get max & min if free voxel is in z range, x range, & y is within maxRadius
    if(tempZ-(freeRes[i]/2)<POIZ && POIZ<tempZ+(freeRes[i]/2)){
      if(tempX-(freeRes[i]/2)<POIX && POIX<tempX+(freeRes[i]/2)){
        if(tempY>POIY){ // gets positive free values
          if(tempY-(freeRes[i]/2)<POIY+maxRadius){
            yPosMinMax.push_back(tempY-(freeRes[i]/2));
            yPosMinMax.push_back(tempY+(freeRes[i]/2));
          }
        }
        else if(tempY<POIY){ // gets negative free values
          if(tempY+(freeRes[i]/2)>POIY-maxRadius){
            yNegMinMax.push_back(tempY-(freeRes[i]/2));
            yNegMinMax.push_back(tempY+(freeRes[i]/2));
          }
        }
      }
    // get max & min if free voxel is in z range, y range, & x is within maxRadius
      else if(tempY-(freeRes[i]/2)<POIY && POIY<tempY+(freeRes[i]/2)){
        if(tempX>POIX){ // gets positive free values
          if(tempX-(freeRes[i]/2)<POIX+maxRadius){
            xPosMinMax.push_back(tempX-(freeRes[i]/2));
            xPosMinMax.push_back(tempX+(freeRes[i]/2));
          }
        }
        else if(tempX<POIX){ // gets negative free values
          if(tempX+(freeRes[i]/2)>POIX-maxRadius){
            xNegMinMax.push_back(tempX-(freeRes[i]/2));
            xNegMinMax.push_back(tempX+(freeRes[i]/2));
          }
        }
      }
    }
  }

  // checking if line of sight is free space
  if(yPosMinMax.size()>0){ // check yPos
    removeIfTwoValues(yPosMinMax);
    int yPosGood = 0;
    if(yPosMinMax.size()==2 && *min_element(yPosMinMax.begin(),yPosMinMax.end())<=POIY+POIRes && *max_element(yPosMinMax.begin(),yPosMinMax.end())>POIY+minRadius){
      yPosGood = 1;
    }
    if(yPosGood == 1){
      float yPosMax = *max_element(yPosMinMax.begin(),yPosMinMax.end());
      int numOfPointsyPos = abs((yPosMax-(POIY+minRadius))/POIRes);
      for(int j=0;j<numOfPointsyPos;j++){
        outputPoints->push_back({POIX,POIY+minRadius+(POIRes/2)+(POIRes*j),POIZ});
        outputRes->push_back(POIRes);
      }
    }
  }
  if(yNegMinMax.size()>0){ // check yNeg
    removeIfTwoValues(yNegMinMax);
    int yNegGood = 0;
    if(yNegMinMax.size()==2 && *min_element(yNegMinMax.begin(),yNegMinMax.end())<POIY-minRadius && *max_element(yNegMinMax.begin(),yNegMinMax.end())>=POIY-POIRes){
      yNegGood = 1;
    }
    if(yNegGood == 1){
      float yNegMin = *min_element(yNegMinMax.begin(),yNegMinMax.end());
      int numOfPointsyNeg = abs((yNegMin-(POIY-minRadius))/POIRes);
      for(int j=0;j<numOfPointsyNeg;j++){
        outputPoints->push_back({POIX,POIY-minRadius-(POIRes/2)-(POIRes*j),POIZ});
        outputRes->push_back(POIRes);
      }
    }
  }
  if(xPosMinMax.size()>0){ // check xPos
    removeIfTwoValues(xPosMinMax);
    int xPosGood = 0;
    if(xPosMinMax.size()==2 && *min_element(xPosMinMax.begin(),xPosMinMax.end())<=POIX+POIRes && *max_element(xPosMinMax.begin(),xPosMinMax.end())>POIX+minRadius){
      xPosGood = 1;
    }
    if(xPosGood==1){
      float xPosMax = *max_element(xPosMinMax.begin(),xPosMinMax.end());
      int numOfPointsxPos = abs((xPosMax-(POIX+minRadius))/POIRes);
      for(int j=0;j<numOfPointsxPos;j++){
        outputPoints->push_back({POIX+minRadius+(POIRes/2)+(POIRes*j),POIY,POIZ});
        outputRes->push_back(POIRes);
      }
    }
  }
  if(xNegMinMax.size()>0){ // check xNeg
    removeIfTwoValues(xNegMinMax);
    int xNegGood = 0;
    if(xNegMinMax.size()==2 && *min_element(xNegMinMax.begin(),xNegMinMax.end())<POIX-minRadius && *max_element(xNegMinMax.begin(),xNegMinMax.end())>=POIX-POIRes){
      xNegGood = 1;
    }
    if(xNegGood==1){
      float xNegMin = *min_element(xNegMinMax.begin(),xNegMinMax.end());
      int numOfPointsxNeg = abs((xNegMin-(POIX-minRadius))/POIRes);
      for(int j=0;j<numOfPointsxNeg;j++){
        outputPoints->push_back({POIX-minRadius-(POIRes/2)-(POIRes*j),POIY,POIZ});
        outputRes->push_back(POIRes);
      }
    }
  }
  return;
}

void testCode(std::vector<double>* outputRes){
  outputRes->push_back(1);
  outputRes->push_back(5);
  outputRes->push_back(3);
  outputRes->push_back(7);
}