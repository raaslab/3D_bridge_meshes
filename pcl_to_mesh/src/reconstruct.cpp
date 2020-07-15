#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "boost/date_time/posix_time/posix_time.hpp"
using namespace pcl;
int main (int argc, char **argv)
{
 if (argc != 3)
 {
 PCL_ERROR ("Syntax: %s input.pcd output.ply\n", argv[0]);

 return -1;
 }
//  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
//  argv[0] = "/home/kartikmadhira/catkin_ws/src/pcl_filter/pcd_outputs/pcd_output.pcd output.vtk";

//  io::loadPCDFile ("/home/kartikmadhira/catkin_ws/src/pcl_filter/pcd_outputs/pcd_output.pcd", *cloud);

//  MovingLeastSquares<PointXYZ, PointXYZ> mls;
//  mls.setInputCloud (cloud);
//  mls.setSearchRadius(0.3);
//  mls.setPolynomialFit (true);
//  mls.setPolynomialOrder(2);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
//  mls.setUpsamplingRadius(0.00005);
//  mls.setUpsamplingStepSize(0.00001);
//  PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
//  mls.process (*cloud_smoothed);
//  NormalEstimationOMP<PointXYZ, Normal> ne;
//  ne.setNumberOfThreads(8);
//  ne.setInputCloud(cloud_smoothed);
//  ne.setRadiusSearch(0.0000001);
//  Eigen::Vector4f centroid;
//  compute3DCentroid(*cloud_smoothed, centroid);
//  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
//  PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
//  ne.compute (*cloud_normals);
//  for (size_t i = 0; i < cloud_normals->size (); ++i)
//  {
//  cloud_normals->points[i].normal_x *= -1;
//  cloud_normals->points[i].normal_y *= -1;
//  cloud_normals->points[i].normal_z *= -1;
//  }
//  PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
//  concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
//   Poisson<PointNormal> poisson;
//  poisson.setDepth(9);
//  poisson.setInputCloud
// (cloud_smoothed_normals);
//  PolygonMesh mesh;
//  poisson.reconstruct (mesh);
// //   compute (cloud, output, depth, solver_divide, iso_divide, point_weight);
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   viewer->addPolygonMesh(mesh, "meshes",0);
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//   cout << "problem\n";
//   while (!viewer->wasStopped ()){
//       viewer->spinOnce ();
//       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//   }
//  io::saveVTKFile (argv[2], mesh);
//  return 0;
}