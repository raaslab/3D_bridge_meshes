// Copyright 2019, RAAS lab
#define _USE_MATH_DEFINES
 
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char** argv) {
    // Initialize topic and broadcaster.
    ros::init(argc, argv, "velodyne_to_intel_tf_pub");
    tf::TransformBroadcaster bcast;
    
    // Set the orientation of the sensor w.r.t vehicle
    // Last used setup during the time this code was written 
    // was the M600 UAV for bridge inspection

    tf::Quaternion sensor_ros;
    sensor_ros.setRPY(0, 0, M_PI_2);
    // sensor_ros.setRPY(0, 0, 0);    
    sensor_ros.normalize();

    // Initialize a transform object to send geometry messages information
    tf::Transform tf;

    ros::Rate rate(10.0);
    ROS_DEBUG_STREAM("Publisher frequency now running at: " << 10);

    if (!ros::ok()) {
        ROS_FATAL_STREAM("ROS nodes are not running!");
    }


    while (ros::ok()) {
        tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf.setRotation(sensor_ros);        
        // Publish transformation between dji and velodyne lidar.
        bcast.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "velodyne", "cam_pose"));
        rate.sleep();
    }
    ros::spin();
    return 0;
}