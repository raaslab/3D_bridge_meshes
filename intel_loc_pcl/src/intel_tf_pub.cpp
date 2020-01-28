// Copyright 2019, RAAS lab

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

void intel_pose_callback(const nav_msgs::Odometry::ConstPtr &intel_pose) {
        static tf::TransformBroadcaster bcast;
        tf::Transform tf;
        tf::Quaternion rot;
        tf.setOrigin(tf::Vector3(intel_pose->pose.pose.position.x, intel_pose->pose.pose.position.y, 
                                    intel_pose->pose.pose.position.z));
        tf.setRotation(tf::Quaternion(intel_pose->pose.pose.orientation.x, intel_pose->pose.pose.orientation.y, 
                                    intel_pose->pose.pose.orientation.z, intel_pose->pose.pose.orientation.w));
        bcast.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "cam_pose", "world"));
        
    }

int main(int argc, char** argv) {
    // Initialize topic and broadcaster.
    ros::init(argc, argv, "intel_tf_pub");

    ros::NodeHandle handler;

    ros::Subscriber sub = handler.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, intel_pose_callback);

    
    ros::spin();
    return 0;
}