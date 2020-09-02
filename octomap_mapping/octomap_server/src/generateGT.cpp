#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// #include <Vector3>
#include <octomap/octomap.h>
#include <octomap_server/OctomapServer.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeStamped.h>
// #include <octomath/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>


// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// ros::Publisher occArrayTrimmed_pub;
float thresholdOccW = 0.6;
int resolutionW = 0.5;
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOccFull (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr subset_cloud(new pcl::PointCloud<pcl::PointXYZ>);      
visualization_msgs::MarkerArray zGTmarkers;
// visualization_msgs::MarkerArray zPruned;
uint32_t shapeCube = visualization_msgs::Marker::CUBE;
float trimmedHeightGT = 0.5;
// int initstraySizeThresh = 2;
std::ofstream outputFile;


void trimmed_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
	octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);
	octomap::OcTree* trimmedOcTree = new octomap::OcTree(resolutionW);
	trimmedOcTree = dynamic_cast<octomap::OcTree*>(absTree);
	octomap::OcTree::leaf_iterator it;
	octomap::OcTree::leaf_iterator endLeaf;
	
	visualization_msgs::Marker zFilteredMarkers; zFilteredMarkers.header.frame_id = "/world"; zFilteredMarkers.header.stamp = ros::Time::now(); zFilteredMarkers.ns = "groundTruth";
	zFilteredMarkers.type = shapeCube; zFilteredMarkers.action = visualization_msgs::Marker::ADD;
	zFilteredMarkers.color.r = 0.0f; zFilteredMarkers.color.g = 1.0f; zFilteredMarkers.color.b = 0.0f; zFilteredMarkers.color.a = 1.0;
	int id4Markers = 0; int markerSize = 1; int sizeOfCloud = 0;
    int i = 0;
	std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;
	for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it) {
		if(it->getOccupancy()>thresholdOccW){
			// std::cout << it->getValue() << "\n";
			// octomap::point3d point(1.25, -634.25, -1.75);
			// if (it.inBBX(point)) {
			// 	std::cout << "Point is in Map!!!!!!\n";
			// }
			if(it.getZ()>trimmedHeightGT) {
                zFilteredMarkers.pose.position.x = it.getX(); zFilteredMarkers.pose.position.y = it.getY(); zFilteredMarkers.pose.position.z = it.getZ();
				zFilteredMarkers.id = i;

				outputFile << i << "," << it.getX() << "," << it.getY() << "," << it.getZ() << "\n";
				zFilteredMarkers.scale.x = markerSize; zFilteredMarkers.scale.y = markerSize; zFilteredMarkers.scale.z = markerSize;
				zGTmarkers.markers.push_back(zFilteredMarkers);
                i++;
			octomap::point3d pointx = trimmedOcTree->keyToCoord(it.getKey());

			std::cout << pointx.x() << " " << pointx.y() << " " << pointx.z() <<"\n";

			}
		} 
		// else {
			// keys.push_back(std::make_pair(it.getKey(), it.getDepth()));
		// }
	}
	// for(auto k:keys) {
    // 	trimmedOcTree->deleteNode(k.first, k.second);
	// }
	// octomap::point3d point(0.45, 1.75, 4.75);
	// octomap::OcTreeKey key;
	// unsigned int depth = trimmedOcTree->getTreeDepth();
	// std::cout << depth << "\n";
	// octomap::OcTreeNode* pointer = trimmedOcTree->search(point, 16);
	// if (pointer != NULL) {
	// 	std::cout << "Point is in Map!\n";
	// } else {
	// 	std::cout << "Point is NOT in Map!\n"

}

int main(int argc, char** argv){
	ros::init(argc, argv, "genGT");
	ros::NodeHandle n;
	// ros::Publisher output_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/zGT", 1, true);
	ros::Publisher zGT = n.advertise<visualization_msgs::MarkerArray>("/zGT", 1, true);
	// ros::Publisher zPruned_pub = n.advertise<visualization_msgs::MarkerArray>("/zPruned_Markers", 1, true);

	ros::Subscriber trimmedTree_sub = n.subscribe("/octomap_full", 1, trimmed_cb);

	ros::Rate loop_rate(1);
	ROS_INFO("Finished initializing all parts of ROS");
	// int loopNumber = 0;
	outputFile.open("exampleOutput.csv");

	while (ros::ok()) {
		// ROS_INFO("While loop");
		outputFile.clear();
		outputFile << "id," << "X," << "Y," << "Z,\n"; 
		ros::spinOnce();
		// pcl_conversions::toPCL(ros::Time::now(), subset_cloud->header.stamp);

		// if(zGTmarkers.markers.size()>2){
		zGT.publish(zGTmarkers);
			// zPruned_pub.publish(zPruned);
		loop_rate.sleep();
		zGTmarkers.markers.clear();
		// subset_cloud->clear();
	}
	outputFile.close();
	return 0;
}
