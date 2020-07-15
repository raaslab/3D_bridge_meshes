#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// ros::Publisher occArrayTrimmed_pub;
float thresholdOcc = 1.0;
int resolution = 1;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOccFull (new pcl::PointCloud<pcl::PointXYZ>);
visualization_msgs::MarkerArray zFiltered;
uint32_t shape = visualization_msgs::Marker::CUBE;
float trimmedHeight = 3;


void trimmed_cb(const octomap_msgs::Octomap& input){ // occupancy tree call back
	octomap::AbstractOcTree* absTree = octomap_msgs::fullMsgToMap(input);
	octomap::OcTree* trimmedOcTree = new octomap::OcTree(resolution);
	trimmedOcTree = dynamic_cast<octomap::OcTree*>(absTree);
	octomap::OcTree::leaf_iterator it;
	octomap::OcTree::leaf_iterator endLeaf;
	visualization_msgs::Marker zFilteredMarkers; zFilteredMarkers.header.frame_id = "/world"; zFilteredMarkers.header.stamp = ros::Time::now(); zFilteredMarkers.ns = "free_markers_trimmed";
	zFilteredMarkers.type = shape; zFilteredMarkers.action = visualization_msgs::Marker::ADD;
	zFilteredMarkers.color.r = 0.0f; zFilteredMarkers.color.g = 1.0f; zFilteredMarkers.color.b = 0.0f; zFilteredMarkers.color.a = 1.0;
	int id4Markers = 0; int markerSize = 1; int sizeOfCloud = 0;

	for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it){
		if(it->getValue()>thresholdOcc){
			if(it.getZ()>trimmedHeight){
				sizeOfCloud++;
			}
		}
	}

	cloudOccFull->header.frame_id = "/world";
	cloudOccFull->width = sizeOfCloud; cloudOccFull->height = 1; cloudOccFull->points.resize (cloudOccFull->width * cloudOccFull->height);
	for(it = trimmedOcTree->begin_leafs(),endLeaf = trimmedOcTree->end_leafs();it!=endLeaf;++it){
		if(it->getValue()>thresholdOcc){
			if(it.getZ()>trimmedHeight){
				cloudOccFull->points[id4Markers].x = it.getX(); cloudOccFull->points[id4Markers].y = it.getY(); cloudOccFull->points[id4Markers].z = it.getZ();

				zFilteredMarkers.pose.position.x = it.getX(); zFilteredMarkers.pose.position.y = it.getY(); zFilteredMarkers.pose.position.z = it.getZ();
				zFilteredMarkers.id = id4Markers;
				zFilteredMarkers.scale.x = markerSize; zFilteredMarkers.scale.y = markerSize; zFilteredMarkers.scale.z = markerSize;
				zFiltered.markers.push_back(zFilteredMarkers);
				id4Markers++;
			}
		}
	}
	ROS_INFO("Size of cloud: %zu", cloudOccFull->size());
}

int main(int argc, char** argv){
	ros::init(argc, argv, "zFilter");
	ros::NodeHandle n;
	ros::Publisher output_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/zFiltered",1,true);
	ros::Publisher zFiltered_pub = n.advertise<visualization_msgs::MarkerArray>("/zFiltered_Markers",1,true);

	ros::Subscriber trimmedTree_sub = n.subscribe("octomap_full_trimmed",1,trimmed_cb);

	ros::Rate loop_rate(1);
	ROS_INFO("Finished initializing all parts of ROS");
	// int loopNumber = 0;
	while (ros::ok()){
		// ROS_INFO("While loop");
		ros::spinOnce();
		pcl_conversions::toPCL(ros::Time::now(), cloudOccFull->header.stamp);
		if(cloudOccFull->size()){
			output_pub.publish(cloudOccFull);
		}
		if(zFiltered.markers.size()>2){
			zFiltered_pub.publish(zFiltered);
		}
		loop_rate.sleep();
		zFiltered.markers.clear();
		cloudOccFull->clear();
	}
}