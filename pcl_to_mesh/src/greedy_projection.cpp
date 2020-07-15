// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/surface/gp3.h>
// // #include <pcl/surface/3rdparty/poisson4/
// #include <pcl/io/vtk_io.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/PolygonMesh.h>
// // #include <pcl/io/ply/ply.h>
// // #include <pcl/io/vtk_lib_io.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/search/impl/search.hpp>
#include <pcl/surface/organized_fast_mesh.h>
// #include <fstream>
// #include <string>
// #include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/registration/icp.h>

#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <pcl/common/transforms.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/surface/qhull.h>

// // #include "boost/date_time/posix_time/posix_time.hpp"
#include <thread>
#include <chrono>
#include <memory>
#include <map>
#include <deque>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/filters/extract_indices.h>


struct VertexXYZ {
  float x;
  float y;
  float z;
  VertexXYZ() {}
  VertexXYZ(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
  VertexXYZ(const VertexXYZ &other) {
    x = other.x;
    y = other.y;
    z = other.z;
  }
  void operator=(const VertexXYZ &other) {
    x = other.x;
    y = other.y;
    z = other.z;
  }
  bool operator==(const VertexXYZ &other) const {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }
  bool operator!=(const VertexXYZ &other) const {
    return !((x == other.x) && (y == other.y) && (z == other.z));
  }
};

struct Quad {
  VertexXYZ c1;
  VertexXYZ c2;
  VertexXYZ c3;
  VertexXYZ c4;
  int id;
  std::vector<VertexXYZ> ver_list;
  // Quad() : id(5) {};
  Quad(VertexXYZ _c1, VertexXYZ _c2,
      VertexXYZ _c3, VertexXYZ _c4, int _id) : c1(_c1), c2(_c2), c3(_c3), c4(_c4), id(_id), ver_list({_c1, _c2, _c3, _c4}) {}

  Quad(const Quad &other) {
      c1 = other.c1;
      c2 = other.c2;
      c3 = other.c3;
      c4 = other.c4;
      id = other.id;
      ver_list = other.ver_list;
  }
  bool operator==(const Quad &other) const {
    return (c1 == other.c1) && (c2 == other.c2) \
            && (c3 == other.c3) && (c4 == other.c4);
  }
  bool operator!=(const Quad &other) const {
    return !((c1 == other.c1) && (c2 == other.c2) \
            && (c3 == other.c3) && (c4 == other.c4));
  }
};
bool comparePoints( pcl::PointXYZI &a,  pcl::PointXYZI &b) {
    return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}

double getDist(const VertexXYZ &a, const VertexXYZ &b) {
    return (pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2));
}

double getDistPoints(const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
  // cout << (pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2))  << "\n";
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2));
}


void removeUnwantedMeshes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PolygonMesh mesh_in,
                          pcl::PolygonMesh &mesh_out) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh_in.cloud, *mesh_cloud);
    mesh_out.header.frame_id = mesh_in.header.frame_id;
    mesh_out.header.seq= mesh_in.header.seq;
    mesh_out.header.stamp = mesh_in.header.stamp;
    mesh_out.cloud = mesh_in.cloud;
    for (int i = 0; i < mesh_in.polygons.size();i++) {
        for (int j = 0; j < cloud_in->points.size(); j++) {
            bool check = false;
            for (auto &each_point : mesh_in.polygons[i].vertices) {
                // mesh_cloud->points[each_point]
                // cout << mesh_cloud->points[each_point].x << " " << cloud_in->points[i].x << "\n";
                if (getDistPoints(mesh_cloud->points[each_point], cloud_in->points[j]) < 0.30) {
                    // cout << "true\n";
                    check = true;
                }

            }
            if (check) {
                mesh_out.polygons.push_back(mesh_in.polygons[i]);
            }
        }
    }
}





void getPolygons(pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud, pcl::PolygonMesh &polygons, 
                pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud) {
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (input_cloud);
  n.setInputCloud (input_cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::concatenateFields (*input_cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZINormal>);
  tree2->setInputCloud (cloud_with_normals);

  pcl::GridProjection<pcl::PointXYZINormal> gp3;
  // pcl::PolygonMesh triangles;

  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.setResolution(0.06);
  gp3.setPaddingSize(1);
  gp3.reconstruct(polygons);

  // pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(polygons.cloud, *out_cloud);
  // std::cout << triangles.polygons[0].vertices.size() << " vertices\n";
  // pcl::PointCloud<pcl::PointXYZI>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::fromPCLPointCloud2(polygons.cloud, *mesh_cloud);
}

void transformPolygonMesh(pcl::PolygonMesh* inMesh, Eigen::Matrix4f& transform)
{
    //NOTE: For testing purposes, the matrix is defined internally
    transform = Eigen::Matrix4f::Identity();
    float theta = 0; // The angle of rotation in radians
    transform (0,0) = cos (theta);
    transform (0,1) = -sin(theta);
    transform (1,0) = sin (theta);
    transform (1,1) = cos (theta);
    transform (0,3) = 2.5;

    //Important part starts here
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(inMesh->cloud, cloud);
    pcl::transformPointCloud(cloud, cloud, transform);
    pcl::toPCLPointCloud2(cloud, inMesh->cloud);
}
void visualizeMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Quad test, int ind) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
  viewer->addCoordinateSystem (1.0);
  cout << cloud->header.frame_id << "\n";

  viewer->addPointCloud<pcl::PointXYZ>(cloud, std::to_string(ind));
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, std::to_string(ind));
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, std::to_string(ind));

  viewer->addLine(pcl::PointXYZ(test.c1.x, test.c1.y, test.c1.z), pcl::PointXYZ(test.c2.x, test.c2.y, test.c2.z), 255,0,0,"line1"+ std::to_string(ind));
  viewer->addLine(pcl::PointXYZ(test.c2.x, test.c2.y, test.c2.z), pcl::PointXYZ(test.c3.x, test.c3.y, test.c3.z), 0, 255,0,"line2"+ std::to_string(ind));
  viewer->addLine(pcl::PointXYZ(test.c3.x, test.c3.y, test.c3.z), pcl::PointXYZ(test.c4.x, test.c4.y, test.c4.z), 0,0,255,"line3"+ std::to_string(ind));
  viewer->addLine(pcl::PointXYZ(test.c4.x, test.c4.y, test.c4.z), pcl::PointXYZ(test.c1.x, test.c1.y, test.c1.z), 123,123,123,"line4"+ std::to_string(ind));
  viewer->initCameraParameters ();
  viewer->setCameraPosition(-1.80, 1.80, -0.40, 2.5, 2.5, 2.5);
  viewer->saveScreenshot("screenshots/box" + std::to_string(ind) + ".png");
  while (!viewer->wasStopped ()){
      viewer->spinOnce (100);
  }
}


struct comp
{
	template<typename T>
	bool operator()(const T& l, const T& r) const
	{ 
		return l.first < r.first;
	}
};

void setSquare(Quad &square) {
    VertexXYZ first_point = square.ver_list[0];
    // square.ver_list.pop_back();
    std::set<std::pair<double, VertexXYZ>, comp> square_set = 
    {{getDist(square.ver_list[1], first_point), square.ver_list[1]}, 
    {getDist(square.ver_list[2], first_point), square.ver_list[2]}, 
    {getDist(square.ver_list[3], first_point), square.ver_list[3]}};
    int i = 1;
    for (auto &each_point : square_set) {
        square.ver_list[i] = each_point.second;
        // cout << each_po/int.first << "\n";
        i++;
    }
    // cout << "new\n";
    square.c1 = square.ver_list[0];
    square.c2 = square.ver_list[1];
    square.c3 = square.ver_list[3];
    square.c4 = square.ver_list[2];

}


Quad getCollation(Quad source, const Quad target) {
    bool ret_val = false;
    std::vector<VertexXYZ> copy_list;
    std::vector<int> ind;
    std::vector<int> ind_tar;

    int count = 0;
    int check = 0;
    // cout << source.ver_list.size() << ""
    for (auto it = source.ver_list.begin(); it != source.ver_list.end(); it++) {

        if (*it == target.c1) {
            ret_val = true;
            count++;
            copy_list.push_back(target.c1);
            ind.push_back(check);
            // ind_tar.push_
        } 
        if (*it == target.c2) {
            ret_val = true;
            count++;
            copy_list.push_back(target.c2);
            ind.push_back(check);

        } 
        if (*it == target.c3) {
            ret_val = true;
            count++;
            copy_list.push_back(target.c3);
            ind.push_back(check);

        } 
        if (*it == target.c4) {
            ret_val = true;
            count++;
            copy_list.push_back(target.c4);
            ind.push_back(check);
            // ind.push_back(4);
        }
        check++;
    }
    std::vector<VertexXYZ> make_quad_list(source.ver_list);
    if (ret_val && count == 2) {
      // cout << count << " \n";
        for (int i = 0; i < target.ver_list.size(); i++) {
            if ((copy_list[0] != target.ver_list[i]) && (copy_list[1] != target.ver_list[i])) {
                make_quad_list[ind.back()] = target.ver_list[i];
                ind.pop_back();
            } 
        }

        Quad ret_quad(make_quad_list[0], make_quad_list[1],
                  make_quad_list[2], make_quad_list[3], 2);
        setSquare(ret_quad);
        return ret_quad;
        
    }
    Quad temp(VertexXYZ(1,1,1), VertexXYZ(1,1,1), VertexXYZ(1,1,1), VertexXYZ(1,1,1), 0);
    return temp;
  }

bool checkCollation(Quad source, const Quad target) {
    bool ret_val = false;
    int count = 0;
    for (auto it = source.ver_list.begin(); it != source.ver_list.end(); it++) {
        if (*it == target.c1) {
            ret_val = true;
            count++;
        } 
        if (*it == target.c2) {
            ret_val = true;
            count++;

        } 
        if (*it == target.c3) {
            ret_val = true;
            count++;

        } 
        if (*it == target.c4) {
            ret_val = true;
            count++;

        } 
    }
    if (count == 2) {
      return true;
    }
    return false;
    // if (ret_val) {
    //   return true;
    // }
    // return false;
  }


void applyTransformation(Eigen::Matrix4f &transform, Quad &source_quad) {
    for (auto &corner : source_quad.ver_list) {
        Eigen::Vector4f corner_vec, corner_transformed;
        corner_vec << corner.x , corner.y, corner.z, 1;
        corner_vec = corner_vec.transpose();
        corner_transformed = transform*corner_vec;
        corner.x = corner_transformed(0);
        corner.y = corner_transformed(1);
        corner.z = corner_transformed(2);
    }
    source_quad.c1 = source_quad.ver_list[0];
    source_quad.c2 = source_quad.ver_list[1];
    source_quad.c3 = source_quad.ver_list[2];
    source_quad.c4 = source_quad.ver_list[3];

}





int main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("/home/kartikmadhira/github/pcl_to_mesh/build/bridge_plane_0.pcd", cloud_blob);
    // std::ofstream outfile1("quads_collated_2.txt");
    // std::ofstream outfile("tri_collated_2.txt");

  // pcl::io::loadPCDFile ("/home/kartikmadhira/catkin_ws/src/pcl_filter/pcd_outputs/pcd_output.pcd", cloud_blob);

  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud
  std::cout << cloud->height << " is the height of cloud\n";
  std::cout << cloud->width << " is the width of cloud\n";

  // Normal estimation*
// Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));

  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

pcl::PolygonMesh triangles;

//   pcl::GridProjection<pcl::PointNormal> gp3;
//   gp3.setInputCloud(cloud_with_normals);
//   gp3.setSearchMethod(tree2);
//   gp3.setResolution(0.05);
//   gp3.setPaddingSize(2);
//   // gp3.scaleInputDataPoint(4);
//   gp3.reconstruct(*triangles);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::fromPCLPointCloud2(triangles->cloud, *mesh_cloud);
// // Convert to Eigen format
// const int npts = static_cast <int> ((mesh_cloud->size()));

// Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_src (3, npts);
// Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_tgt (3, npts);

// for (int i = 0; i < npts; ++i)
// {
//       cloud_src (0, i) = mesh_cloud->points[i].x;
//       cloud_src (1, i) = mesh_cloud->points[i].y;
//       cloud_src (2, i) = mesh_cloud->points[i].z;

//       cloud_tgt (0, i) = cloud->points[i].x;
//       cloud_tgt (1, i) = cloud->points[i].y;
//       cloud_tgt (2, i) = cloud->points[i].z;
// }
// Eigen::Matrix4f T;

// // Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
// T = pcl::umeyama (cloud_src, cloud_tgt, true);

// pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::transformPointCloud(*mesh_cloud, *transformed, T/T(3,3));

// pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
// icp.setInputSource(transformed);
// icp.setInputTarget(cloud);

// pcl::PointCloud<pcl::PointXYZ> Final;
// icp.align(Final);


// // Create a Convex Hull representation of the projected inliers
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_grid (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::ConcaveHull<pcl::PointXYZ> chull_grid;
// // pcl::ConvexHull<pcl::PointXYZ>::MeshConstruction::
// // chull_grid.
// // chull_grid.per
// chull_grid.setInputCloud (cloud);
// chull_grid.reconstruct(mesh_out);

// // cout << "The grid projection mesh has an area of" << chull_grid.getTotalArea() << "\n";


// //   // Create a Convex Hull representation of the projected inliers
// // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_cloud (new pcl::PointCloud<pcl::PointXYZ>);
// // pcl::ConvexHull<pcl::PointXYZ> chull_cloud;
// // chull_cloud.setInputCloud (cloud);
// // // chull_cloud.
// // // chull_cloud.reconstruct();

// // cout << "The original point cloud has an area of" << chull_cloud.getTotalArea() << "\n";
// // float scale = chull_cloud.getTotalArea()/chull_grid.getTotalArea();
// // cout << "scale is " << chull_cloud.getTotalArea() << "\n";

// std::cout << "has converged:" << icp.hasConverged() << " score: " <<
// icp.getFitnessScore() << std::endl;
// Eigen::Matrix4f T2 = icp.getFinalTransformation();
// pcl::PointCloud<pcl::PointXYZ>::Ptr transformed2 (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::transformPointCloud(*transformed, *transformed2, (T2));




  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(4);
  poisson.setOutputPolygons(true);
  poisson.setInputCloud(cloud_with_normals);
  // poisson.
  poisson.setConfidence(true);
  poisson.setScale(1);
  poisson.setIsoDivide(6);
  pcl::PolygonMesh mesh_out;

  poisson.reconstruct(mesh_out);


// boost::shared_ptr<pcl::PolygonMesh> mesh_out (new pcl::PolygonMesh(*triangles));
removeUnwantedMeshes(cloud, mesh_out, triangles);



  // // gp3.sca
  // std::cout << triangles.polygons[0].vertices.size() << " vertices\n";
  // float default_iso_level = 0.0f;
  // int default_hoppe_or_rbf = 0;
  // float default_extend_percentage = 0.0f;
  // int default_grid_res = 50;
  // float default_off_surface_displacement = 0.01f;
  // pcl::MarchingCubesRBF<pcl::PointNormal> mc;
  // mc.setIsoLevel (0.8);
  // mc.setGridResolution (100, 100, 100);
  // // mc.setPercentageExtendGrid (extend_percentage);
  // mc.setInputCloud (cloud_with_normals);
  // mc.setOffSurfaceDisplacement(1.5);
  // mc.reconstruct(*triangles);





  // triangles.cloud
  // std::vector<pcl::Vertices> first_list(triangles.polygons.begin(), triangles.polygons.end());
  // std::vector<pcl::Vertices> sec_list(first_list.begin(), first_list.end());

  //  // Create the filtering object
  // pcl::PassThrough<pcl::PointXYZI> pass;
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.0, 1.0);
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*mesh_cloud);

//   Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(triangles.cloud, *mesh_cloud);
  // pcl::transformPointCloud (*cloud, *mesh_cloud, transform);
//   // transformPolygonMesh(&triangles, transform);
//   cout << transform(1,0) << transform(1,1) << transform(1,2) << transform(1,3) <<"\n";
//   cout << transform(2,0) << transform(2,1) << transform(2,2) << transform(2,3) <<"\n";
//   cout << transform(3,0) << transform(3,1) << transform(3,2) << transform(3,3) <<"\n";

  std::deque<Quad> q1;

//   // Store all the polygons int the Quad format in the respective ques;
//   // triangles.polygons[0].vertices[0].o
  int count = 0;

//   // applyANMS(mesh_cloud, 0.0004);
//   // pcl::PolygonMesh new_quads;
//   // pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZI>);
//   // getPolygons(mesh_cloud, new_quads, out_cloud);
//   // triangles.polygons[1] =  pcl::concatenateFields(triangles.polygons[1], triangles.polygons[2]);
  for (auto &each_quad : triangles.polygons) {
    if (each_quad.vertices.size() == 4) {
        VertexXYZ c1(mesh_cloud->points[each_quad.vertices[0]].x,
        mesh_cloud->points[each_quad.vertices[0]].y,
        mesh_cloud->points[each_quad.vertices[0]].z);

        VertexXYZ c2(mesh_cloud->points[each_quad.vertices[3]].x,
        mesh_cloud->points[each_quad.vertices[3]].y,
        mesh_cloud->points[each_quad.vertices[3]].z);

        VertexXYZ c3(mesh_cloud->points[each_quad.vertices[2]].x,
        mesh_cloud->points[each_quad.vertices[2]].y,
        mesh_cloud->points[each_quad.vertices[2]].z);

        VertexXYZ c4(mesh_cloud->points[each_quad.vertices[1]].x,
        mesh_cloud->points[each_quad.vertices[1]].y,
        mesh_cloud->points[each_quad.vertices[1]].z);
        // each_quad.vertices
        Quad make_quad(c1, c2, c3, c4, count++);
        q1.push_back(make_quad);
    }
  }

  std::deque<Quad> q2(q1);
// // cout << "q1 size is" << q1.size() << "\n";

      Quad first = q1.front();
  // cout << triangles.polygons.size() << "the polygons\n";
  while (!q1.empty()) {
    // cout << q1.size() << " "  << q2.size() << "\n";
      first = q1.front();
      q1.pop_front();
      std::deque<Quad> temp;
      bool collate_check = false;
      // cout <<  q1.size() << " is the q1 size \n";
      while (!q2.empty()) {
        Quad second = q2.front();
        // cout <<  q2.size() << " is the q2 size \n";
        if (first != second) {


                Quad temp1 = first;
                Quad temp2 = second;
            if(checkCollation(first, second)) {
                // cout << "Collated! The id is " << first.id << "\n";
                
                Quad first_dup = getCollation(first, second);
                Quad temp(VertexXYZ(1,1,1), VertexXYZ(1,1,1), VertexXYZ(1,1,1), VertexXYZ(1,1,1), 0);

                // cout << temp1.c1.x << " " << temp1.c1.y << " "  << temp1.c1.z << "\n";
                // cout << temp1.c2.x << " " << temp1.c2.y << " "  << temp1.c2.z << "\n"; 
                // cout << temp1.c3.x << " " << temp1.c3.y << " "  << temp1.c3.z << "\n"; 
                // cout << temp1.c4.x << " " << temp1.c4.y << " "  << temp1.c4.z << "\n";

                // cout << "-----------second-----------------\n";


                // cout << temp2.c1.x << " " << temp2.c1.y << " "  << temp2.c1.z << "\n";
                // cout << temp2.c2.x << " " << temp2.c2.y << " "  << temp2.c2.z << "\n"; 
                // cout << temp2.c3.x << " " << temp2.c3.y << " "  << temp2.c3.z << "\n"; 
                // cout << temp2.c4.x << " " << temp2.c4.y << " "  << temp2.c4.z << "\n";
                // cout << "-----------after-----------------\n";

                // cout << first.c1.x << " " << first.c1.y << " "  << first.c1.z << "\n";
                // cout << first.c2.x << " " << first.c2.y << " "  << first.c2.z << "\n"; 
                // cout << first.c3.x << " " << first.c3.y << " "  << first.c3.z << "\n"; 
                // cout << first.c4.x << " " << first.c4.y << " "  << first.c4.z << "\n"; 
                // cout << "-------------------------------\n";

                // q2.pop_front();
                if (first_dup != temp) {

                    std::deque<Quad> rem_sec_from_first;
                                // cout << q1.front().c1.x << "\n";
                    // if (second.x != )
                    // cout << second.c1.x << " " << second.c1.y << "\n";
                      while (q1.front() != second && !(q1.empty())) {
                    // cout << second.c1.x << " " << second.c1.y << "\n";

                          rem_sec_from_first.push_front(q1.front());
                          q1.pop_front();
                      }
                      if (!q1.empty()) {
                        q1.pop_front();
                      }
                      while(!rem_sec_from_first.empty()) {
                          q1.push_front(rem_sec_from_first.front());
                          rem_sec_from_first.pop_front();
                      }
                    collate_check = true;
                    first = first_dup;
                    // while ()
                    break;
                }
            }
        // }
        // temp = q2.front();
            temp.push_front(q2.front());
        }
            q2.pop_front();

      }
      // once q2 is emptied or a collation matures, merge temp and q2;
      Quad t2 = q2.front();
      if (collate_check) {
          q2.pop_front();
      }
      // Quad t2 = q2.front();

      while (!temp.empty()) {
        if (collate_check) {
            if (temp.front() != t2) {
                q2.push_front(temp.front());
                temp.pop_front();
            }
        } else {
            q2.push_front(temp.front());
            temp.pop_front();
        }
      }
      // if new collation was not possible push first onto q2;
      if (!collate_check) {
          q2.push_front(first);
      } else {
          q1.push_front(first);
      }
      // visualizeMesh(triangles)

  }

// // VertexXYZ c1(1, 0, 0);
// // VertexXYZ c2(2, 0, 0);
// // VertexXYZ c3(3, 0, 0);
// // VertexXYZ c4(4, 0, 0);

// // // Quad make_quad1(c1, c2, c3, c4);

// // VertexXYZ c11(23, 0, 0);
// // VertexXYZ c22(6, 0, 0);
// // VertexXYZ c33(7, 0, 0);
// // VertexXYZ c44(43, 0, 0);

// // // Quad make_quad2(c11, c22, c33, c44);


// // cout << "q2 size is" << q2.size() << "\n";

// // if (make_quad1 != make_quad2) {
//     // cout << " Quad Inequality runs properly\n";
// // }

// // if (checkCollation(make_quad1, make_quad2)) {
// //     for (auto &ver : make_quad1.ver_list) {
// //         cout << ver.x << " " << ver.y << " " << ver.z << "\n";
// //     }
// // }





//       // VertexXYZ c2(mesh_cloud->points[each_quad.vertices[3]].x,
//       // mesh_cloud->points[each_quad.vertices[3]].y,
//       // mesh_cloud->points[each_quad.vertices[3]].z);

//       // VertexXYZ c3(mesh_cloud->points[each_quad.vertices[2]].x,
//       // mesh_cloud->points[each_quad.vertices[2]].y,
//       // mesh_cloud->points[each_quad.vertices[2]].z);

//       // VertexXYZ c4(mesh_cloud->points[each_quad.vertices[1]].x,
//       // mesh_cloud->points[each_quad.vertices[1]].y,
//       // mesh_cloud->points[each_quad.vertices[1]].z);

//       // Quad make_quad(c1, c2, c3, c4)




// // cloud->points[triangles.polygons[0].vertices[0]].y  // for (auto &part : parts) {
//   //   std::cout << part;
//   // }
//   // std::ofstream outfile("triangles0.txt");
//   // std::ofstream outfile1("quads0.txt");

//   // outfile << "X \t Y\t Z \t \n";
//   // outfile1 << "X \t Y\t Z \t \n";

//   // for (int i=0;i<triangles.polygons.size(); ++i) {
//   //   // std::string 
//   //   // outfile << setprecision(2) <<cloud->points[triangles.polygons[i].vertices[0]].x << "\t" <<
//   //   // cloud->points[triangles.polygons[i].vertices[0]].y << "\t" <<
//   //   // cloud->points[triangles.polygons[i].vertices[0]].z << "\n";

//   //   // outfile << cloud->points[triangles.polygons[i].vertices[1]].x << "\t" <<
//   //   // cloud->points[triangles.polygons[i].vertices[1]].y << "\t" <<
//   //   // cloud->points[triangles.polygons[i].vertices[1]].z << "\n";

//   //   // outfile << cloud->points[triangles.polygons[i].vertices[2]].x << "\t" <<
//   //   // cloud->points[triangles.polygons[i].vertices[2]].y << "\t" <<
//   //   // cloud->points[triangles.polygons[i].vertices[2]].z << "\n";



//   //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[0]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[0]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[0]].z << "\n";

//   //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[3]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[3]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[3]].z << "\n";

//   //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[2]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[2]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[2]].z << "\n";

//   //   // outfile << "\n";
//   //   outfile << "\n";

//   //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[2]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[2]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[2]].z << "\n";

//   //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[1]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[1]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[1]].z << "\n";

//   //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[0]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[0]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[0]].z << "\n";

//   //   // outfile << "\n";
//   //   outfile << "\n";




//   //   outfile1 << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[0]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[0]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[0]].z << "\n";

//   //   outfile1 << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[3]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[3]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[3]].z << "\n";

//   //   outfile1 << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[2]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[2]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[2]].z << "\n";

//   //   outfile1 << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[1]].x << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[1]].y << "\t" <<
//   //   mesh_cloud->points[triangles.polygons[i].vertices[1]].z << "\n";

//   //   // outfile << "\n";
//   //   outfile1 << "\n";
  
//   // }
//   // pcl::geometry::toHalfEdgeMesh(triangles, quads);


//   // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   // viewer->setBackgroundColor (0, 0, 0);
//   // viewer->addPolygonMesh (triangles,"meshes",0);

//   // pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud (new pcl::PointCloud<pcl::PointXYZ>);

//   // viewer->addPointCloud<pcl::PointXYZI>(mesh_cloud);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
  viewer->addPolygonMesh(mesh_out , "meshses", 0); 
  // viewer->addPointCloud<pcl::PointXYZ>(transformed2, "0");
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "0");

  // viewer->addPointCloud<pcl::PointXYZ>(mesh_cloud, "1");
  int c = 0;
  int cl = 0;
  while (!q2.empty()) {
      Quad test = q2.front();
      // outfile1 << test.c1.x << " " << test.c1.y << " " << test.c1.z << "\n";
      // outfile1 << test.c2.x << " " << test.c2.y << " " << test.c2.z << "\n";
      // outfile1 << test.c3.x << " " << test.c3.y << " " << test.c3.z << "\n";
      // outfile1 << test.c4.x << " " << test.c4.y << " " << test.c4.z << "\n";
      // outfile1 << "\n";

      // outfile << test.c1.x << " " << test.c1.y << " " << test.c1.z << "\n";
      // outfile << test.c2.x << " " << test.c2.y << " " << test.c2.z << "\n";
      // outfile << test.c3.x << " " << test.c3.y << " " << test.c3.z << "\n";
      // // outfile << test.c4.x << " " << test.c4.y << " " << test.c4.z << "\n";
      // outfile << "\n";
      // outfile << test.c3.x << " " << test.c3.y << " " << test.c3.z << "\n";
      // outfile << test.c4.x << " " << test.c4.y << " " << test.c4.z << "\n";
      // outfile << test.c1.x << " " << test.c1.y << " " << test.c1.z << "\n";
      // outfile << "\n";
    // viewer->addLine(pcl::PointXYZ(test.c1.x, test.c1.y, test.c1.z), pcl::PointXYZ(test.c2.x, test.c2.y, test.c2.z), 255,0,0,"line1"+ std::to_string(cl));
    // viewer->addLine(pcl::PointXYZ(test.c2.x, test.c2.y, test.c2.z), pcl::PointXYZ(test.c3.x, test.c3.y, test.c3.z), 255, 0, 0, "line2"+ std::to_string(cl));
    // viewer->addLine(pcl::PointXYZ(test.c3.x, test.c3.y, test.c3.z), pcl::PointXYZ(test.c4.x, test.c4.y, test.c4.z), 255, 0, 0, "line3"+ std::to_string(cl));
    // viewer->addLine(pcl::PointXYZ(test.c4.x, test.c4.y, test.c4.z), pcl::PointXYZ(test.c1.x, test.c1.y, test.c1.z), 255, 0, 0,"line4"+ std::to_string(cl));
    // applyTransformation(transform, test);
    visualizeMesh(mesh_cloud, test, cl);

    if (!q2.empty()) {
      q2.pop_front();
    }
    cl++;

      // outfile1 << "\n";
      // q2.pop_front();
  }







//   // }
//   // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[0]], mesh_cloud->points[triangles.polygons[3].vertices[2]], 255,0,0,"line2");
//   // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[3]], mesh_cloud->points[triangles.polygons[3].vertices[2]], 255,0,0,"line3");
//   // // viewer->addCircle()
//   // viewer->addSphere(mesh_cloud->points[triangles.polygons[3].vertices[0]],0.002,255,0,0,"sphere1");
//   // viewer->addSphere(mesh_cloud->points[triangles.polygons[3].vertices[1]],0.002,0,255,0,"sphere2");
//   // viewer->addSphere(mesh_cloud->points[triangles.polygons[3].vertices[2]],0.002,0,0,255,"sphere3");

//   // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[1]], mesh_cloud->points[triangles.polygons[3].vertices[2]], 0,255,0, "poly1");
  // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[1]], mesh_cloud->points[triangles.polygons[3].vertices[0]], 0,255,0, "poly2");
  // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[0]], mesh_cloud->points[triangles.polygons[3].vertices[2]],  0,255,0,"poly3");
  // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[3]], mesh_cloud->points[triangles.polygons[3].vertices[0]],  0,255,0,"poly4");

  // viewer->addSphere(pcl::Vertices::vertices(q2.front().c1.x, q2.front().c1.y, q2.front().c1.z);

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(-1.80, 1.80, -0.40, 2.5, 2.5, 2.5);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "0");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "0");
  // viewer->addPointCloud()
  // viewer->saveScreenshot("screenshots/allQuads.png");
  while (!viewer->wasStopped ()){
      viewer->spinOnce ();
      std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
//  viewer->addLine(cloud->points[triangles.polygons[3].vertices[1]], cloud->points[triangles.polygons[3].vertices[2]], 0,255,0, "poly1");
//   viewer->addLine(cloud->points[triangles.polygons[3].vertices[2]], cloud->points[triangles.polygons[3].vertices[3]], 0,255,0, "poly2");
//   viewer->addLine(cloud->points[triangles.polygons[3].vertices[3]], cloud->points[triangles.polygons[3].vertices[0]],  0,255,0,"poly3");
//   viewer->addLine(cloud->points[triangles.polygons[3].vertices[0]], cloud->points[triangles.polygons[3].vertices[1]],  0,255,0,"poly4");

  
}