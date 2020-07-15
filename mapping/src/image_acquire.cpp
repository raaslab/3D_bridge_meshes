#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
static int counter = 0;
void image_callback(const sensor_msgs::ImageConstPtr image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imwrite("/home/kartikmadhira/Desktop/lab_updates/simulation/image_acquire/image" + std::to_string(counter++) + ".jpg", cv_ptr->image);
}

int main(int argc, char** argv) {
    ros::init (argc, argv, "image_acquire");
    ros::NodeHandle node_handler;
    // sensor_msgs::PointCloud2 output_pcl;
    // std::string topic = argv[1];
    // std::cout << "the topic bei/ng subscribed is " << topic << "\n";
    // ros::Subscriber sub = node_handler.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, boost::bind(&pcl_callback, _1, output_pcl));
    ros::Subscriber sub = node_handler.subscribe<sensor_msgs::ImageConstPtr>("/front_cam/camera/image", 1, image_callback);

    // pub = node_handler.advertise<sensor_msgs::PointCloud2>("filtered", 0.1);
    ros::spin();
}