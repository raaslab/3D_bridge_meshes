#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/CameraInfo.h>
#include <eigen_conversions/eigen_msg.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <iostream>

ros::Publisher pub_cloud;
ros::Publisher pub_map;

static int counter = 0;

void sync_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_trimmed,
                    const sensor_msgs::PointCloud2ConstPtr &input_cloud) {

        // cv::Mat image_cv_mat = cv_ptr->image;
        
        // Get the transformation between velodyne frame and image frame
        pcl::PCLPointCloud2 pcl_input_trimmed;
        pcl_conversions::toPCL(*input_cloud_trimmed, pcl_input_trimmed);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>)-----; 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_trimmed(new pcl::PointCloud<pcl::PointXYZRGB>);      
        pcl::fromPCLPointCloud2(pcl_input_trimmed, *temp_cloud_trimmed);

        pcl::PCLPointCloud2 pcl_input;
        pcl_conversions::toPCL(*input_cloud, pcl_input);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>)-----; 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);      
        pcl::fromPCLPointCloud2(pcl_input, *temp_cloud);

        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

        kdtree.setInputCloud (temp_cloud);


        // K nearest neighbor search

        int K = 1;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        // pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr subset_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);      

        for (int i = 0; i < temp_cloud_trimmed->points.size(); i++) {
            temp_cloud->points[pointIdxNKNSearch[0]].r = 255;
            temp_cloud->points[pointIdxNKNSearch[0]].g = 255;
            temp_cloud->points[pointIdxNKNSearch[0]].b = 255;

        }

        for (int i = 0; i < temp_cloud_trimmed->points.size(); i++) {

            if ( kdtree.nearestKSearch (temp_cloud_trimmed->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
                pcl::PointXYZRGB point = pcl::PointXYZRGB(255, 0 , 0);
                temp_cloud->points[pointIdxNKNSearch[0]].r = 255;
                temp_cloud->points[pointIdxNKNSearch[0]].g = 0;
                temp_cloud->points[pointIdxNKNSearch[0]].b = 0;
                // point.x = temp_cloud->points[pointIdxNKNSearch[0]].x;
                // point.y = temp_cloud->points[pointIdxNKNSearch[0]].y;
                // point.z = temp_cloud->points[pointIdxNKNSearch[0]].z;

                subset_cloud->points.push_back(temp_cloud->points[pointIdxNKNSearch[0]]);
            }
        }

        subset_cloud->is_dense = false;
        subset_cloud->header.frame_id = temp_cloud->header.frame_id;

        // // std::cout << inliers->indices.size() << "\n";
        // pcl::PassThrough<pcl::PointXYZRGB> pass_y;
        // pass_y.setInputCloud(temp_cloud);
        // pass_y.setFilterFieldName("y");
        // pass_y.setFilterLimits(-3, 2);
        // pass_y.filter(*temp_cloud);
        // pcl::PassThrough<pcl::PointXYZRGB> pass_x;
        // pass_x.setInputCloud(temp_cloud);
        // pass_x.setFilterFieldName("x");
        // pass_x.setFilterLimits(0, 50);
        // pass_x.filter(*temp_cloud);
        sensor_msgs::PointCloud2 ros_output_cloud;

        // pcl::toROSMsg(*temp_cloud, ros_output_cloud)----------------;
        pcl::toROSMsg(*subset_cloud, ros_output_cloud);
        pub_cloud.publish(ros_output_cloud);

        sensor_msgs::PointCloud2 ros_output_cloud_map;

        // pcl::toROSMsg(*temp_cloud, ros_output_cloud)----------------;
        pcl::toROSMsg(*temp_cloud, ros_output_cloud_map);
        pub_map.publish(ros_output_cloud_map);

}

int main(int argc, char** argv) {
    // Initialize topic and broadcaster.
    ros::init(argc, argv, "transform_cloud_to_cam_frame");

    ros::NodeHandle handler;

    // Pass the position and attitude topics through a filter,
    // so that they can by synced together.
 
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_acquire_trimmed
                                    (handler, "octomap_point_cloud_centers_trimmed", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> loam_acquire
    //                                 (handler, "/velodyne_cloud_registered", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_acquire
                                (handler, "octomap_point_cloud_centers", 1);
    // Object call for devising a policy or parameters on how to 
    // synchronize the topics
    typedef message_filters::sync_policies::ApproximateTime
                <sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> 
                SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync_params(SyncPolicy(10), 
                                              pcl_acquire_trimmed, pcl_acquire);
    // Using these parameters of the sync policy, get the messages, which are synced
    sync_params.registerCallback(boost::bind(&sync_callback, _1, _2));
    pub_cloud = handler.advertise<sensor_msgs::PointCloud2>("bridge_only_cloud", 1);
    pub_map = handler.advertise<sensor_msgs::PointCloud2>("fused_map", 1);

    ros::spin();
    return 0;
}