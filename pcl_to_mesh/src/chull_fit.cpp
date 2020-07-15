#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/impl/search.hpp>
#include <fstream>
#include "boost/date_time/posix_time/posix_time.hpp"
int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("/home/kartikmadhira/github/pcl_to_mesh/build/bridge_plane_2.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

  // // Build a filter to remove spurious NaNs
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0, 1.1);
  // pass.filter (*cloud_filtered);
  // std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setIndices (inliers);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Create a Convex Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.reconstruct (*cloud_hull);

  
  std::ofstream outfile("plane_fit_2.txt");
  outfile << "X" << "\t" << "Y" << "\t" << "Z" << "\n";
  for (int i = 0; i <= cloud_hull->points.size(); i++) {
    outfile << cloud_hull->points[i].x << " " << cloud_hull->points[i].y << " "
    << cloud_hull->points[i].z << "\n";
  }
  std::cerr << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("bridge_filtered_hull.pcd", *cloud_hull, false);

  return (0);
}