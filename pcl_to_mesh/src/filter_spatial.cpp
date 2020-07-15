// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <ctime>
#include "boost/date_time/posix_time/posix_time.hpp"
int
main (int argc, char** argv)
{


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("/home/kartikmadhira/catkin_ws/src/pcl_filter/pcd_outputs/pcd_output.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    // pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    int z_min, z_max, x_min, x_max, y_min, y_max;
    std::string axis;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZI>);
    // std::cout << topic << "\n";
    z_min = -6;
    z_max = 15;
    x_min = -20;
    x_max = -2;
    // z_min = -10;
    // z_max = +10;
    y_min = 3;
    y_max = 7;
    // x_min = -10;
    // x_max = +10;
    // z_min = -1;
    // z_max = 3;

    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min, x_max);
    pass_x.filter(*cloud_filtered_y);

    pcl::PassThrough<pcl::PointXYZI> pass_z;
    pass_z.setInputCloud(cloud_filtered_y);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min, z_max);
    pass_z.filter(*cloud_filtered_y);


    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pass_y.setInputCloud(cloud_filtered_y);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min, y_max);
    pass_y.filter(*cloud_filtered_y);

    pcl::io::savePCDFile("/home/kartikmadhira/catkin_ws/src/pcl_filter/pcd_outputs/filtered_bridge.pcd",
                        *cloud_filtered_y);

}