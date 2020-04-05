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
#include <pcl/surface/impl/organized_fast_mesh.hpp>

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

double getDist(const pcl::PointXYZI &a, const pcl::PointXYZI &b) {
    return (pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2));
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


void visualizeMesh(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Quad test, int ind,
                   boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer) {
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
  viewer->addCoordinateSystem (1.0);
  viewer->addPointCloud<pcl::PointXYZI>(cloud, std::to_string(ind));
  viewer->addLine(pcl::PointXYZ(test.c1.x, test.c1.y, test.c1.z), pcl::PointXYZ(test.c2.x, test.c2.y, test.c2.z), 255,0,0,"line1"+ std::to_string(ind));
  viewer->addLine(pcl::PointXYZ(test.c2.x, test.c2.y, test.c2.z), pcl::PointXYZ(test.c3.x, test.c3.y, test.c3.z), 0, 255,0,"line2"+ std::to_string(ind));
  viewer->addLine(pcl::PointXYZ(test.c3.x, test.c3.y, test.c3.z), pcl::PointXYZ(test.c4.x, test.c4.y, test.c4.z), 0,0,255,"line3"+ std::to_string(ind));
  viewer->addLine(pcl::PointXYZ(test.c4.x, test.c4.y, test.c4.z), pcl::PointXYZ(test.c1.x, test.c1.y, test.c1.z), 123,123,123,"line4"+ std::to_string(ind));
  viewer->initCameraParameters ();
  viewer->setCameraPosition(-1.25, 1.19, -0.29,1,1,1);
  while (!viewer->wasStopped ()){
      viewer->spinOnce (1000);
  }
}



// Function to apply ANMS to each of the square points so that you have one point in a specified spatial area
void applyANMS(pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud, double dist_tol) {
    // pcl::PointCloud<pcl::PointXYZI>::Ptr ret_cloud(input_cloud);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Take every point, and iterate through each other point to check if it's inside tolerance value
    double min = INT_MAX;
    std::unordered_map<int, int> smap;
    for (int i = 0; i < input_cloud->points.size(); i++) {
        for (int j = 0; j < input_cloud->points.size(); j++) {
            if (!comparePoints(input_cloud->points[i], input_cloud->points[j])) {
                min = std::min (getDist(input_cloud->points[i], input_cloud->points[j]), min);
                if (getDist(input_cloud->points[i], input_cloud->points[j]) < dist_tol) {
                    // auto point = std::find(ret_cloud->points.begin(), ret_cloud->points.end(), *it1);
                    // ret_cloud->points.erase(point);
                    // cout << "inliers\n";
                    if (smap.find(j) == smap.end()) {
                        inliers->indices.push_back(j);
                    }
                    smap[i] = 1; 
                }
            }
        }
    }
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*input_cloud);
    // cout << min << "\n";
    // return ret_cloud;
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
            ind.push_back(4);
        }
        check++;
    }
    std::vector<VertexXYZ> make_quad_list;
    if (ret_val && count == 2) {
      // cout << count << " \n";
        for (int i = 0; i < target.ver_list.size(); i++) {
            if ((copy_list[0] != target.ver_list[i]) && (copy_list[1] != target.ver_list[i])) {
                make_quad_list.push_back(target.ver_list[i]);
            } 
        }
        for (auto tar : source.ver_list) {
            if ((copy_list[0] != tar) && (copy_list[1] != tar)) {
                make_quad_list.push_back(tar);
            } 
     
        }
        Quad ret_quad(make_quad_list[0], make_quad_list[1],
                  make_quad_list[2], make_quad_list[3], 2);

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



int main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("/home/kartikmadhira/github/pcl_to_mesh/build/bridge_plane_2.pcd", cloud_blob);
    // std::ofstream outfile1("quads_collated_0.txt");
    // std::ofstream outfile("tri_collated_0.txt");

  // pcl::io::loadPCDFile ("/home/kartikmadhira/catkin_ws/src/pcl_filter/pcd_outputs/pcd_output.pcd", cloud_blob);

  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud
  std::cout << cloud->height << " is the height of cloud\n";
  std::cout << cloud->width << " is the width of cloud\n";

  // Normal estimation*
// Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZINormal>);
  tree2->setInputCloud (cloud_with_normals);

  pcl::GridProjection<pcl::PointXYZINormal> gp3;
  pcl::PolygonMesh triangles;

  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.setResolution(0.05);
  gp3.setPaddingSize(2);
  gp3.reconstruct(triangles);
  
  std::cout << triangles.polygons[0].vertices.size() << " vertices\n";
  pcl::PointCloud<pcl::PointXYZI>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(triangles.cloud, *mesh_cloud);
  
  // std::vector<pcl::Vertices> first_list(triangles.polygons.begin(), triangles.polygons.end());
  // std::vector<pcl::Vertices> sec_list(first_list.begin(), first_list.end());

  //  // Create the filtering object
  // pcl::PassThrough<pcl::PointXYZI> pass;
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.0, 1.0);
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*mesh_cloud);

  std::deque<Quad> q1;

  // Store all the polygons int the Quad format in the respective ques;
  // triangles.polygons[0].vertices[0].o
  int count = 0;

  // applyANMS(mesh_cloud, 0.0004);
  // pcl::PolygonMesh new_quads;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  // getPolygons(mesh_cloud, new_quads, out_cloud);
  // triangles.polygons[1] =  pcl::concatenateFields(triangles.polygons[1], triangles.polygons[2]);
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
// cout << "q1 size is" << q1.size() << "\n";

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

// VertexXYZ c1(1, 0, 0);
// VertexXYZ c2(2, 0, 0);
// VertexXYZ c3(3, 0, 0);
// VertexXYZ c4(4, 0, 0);

// // Quad make_quad1(c1, c2, c3, c4);

// VertexXYZ c11(23, 0, 0);
// VertexXYZ c22(6, 0, 0);
// VertexXYZ c33(7, 0, 0);
// VertexXYZ c44(43, 0, 0);

// // Quad make_quad2(c11, c22, c33, c44);


// cout << "q2 size is" << q2.size() << "\n";

// if (make_quad1 != make_quad2) {
    // cout << " Quad Inequality runs properly\n";
// }

// if (checkCollation(make_quad1, make_quad2)) {
//     for (auto &ver : make_quad1.ver_list) {
//         cout << ver.x << " " << ver.y << " " << ver.z << "\n";
//     }
// }





      // VertexXYZ c2(mesh_cloud->points[each_quad.vertices[3]].x,
      // mesh_cloud->points[each_quad.vertices[3]].y,
      // mesh_cloud->points[each_quad.vertices[3]].z);

      // VertexXYZ c3(mesh_cloud->points[each_quad.vertices[2]].x,
      // mesh_cloud->points[each_quad.vertices[2]].y,
      // mesh_cloud->points[each_quad.vertices[2]].z);

      // VertexXYZ c4(mesh_cloud->points[each_quad.vertices[1]].x,
      // mesh_cloud->points[each_quad.vertices[1]].y,
      // mesh_cloud->points[each_quad.vertices[1]].z);

      // Quad make_quad(c1, c2, c3, c4)




// cloud->points[triangles.polygons[0].vertices[0]].y  // for (auto &part : parts) {
  //   std::cout << part;
  // }
  // std::ofstream outfile("triangles0.txt");
  // std::ofstream outfile1("quads0.txt");

  // outfile << "X \t Y\t Z \t \n";
  // outfile1 << "X \t Y\t Z \t \n";

  // for (int i=0;i<triangles.polygons.size(); ++i) {
  //   // std::string 
  //   // outfile << setprecision(2) <<cloud->points[triangles.polygons[i].vertices[0]].x << "\t" <<
  //   // cloud->points[triangles.polygons[i].vertices[0]].y << "\t" <<
  //   // cloud->points[triangles.polygons[i].vertices[0]].z << "\n";

  //   // outfile << cloud->points[triangles.polygons[i].vertices[1]].x << "\t" <<
  //   // cloud->points[triangles.polygons[i].vertices[1]].y << "\t" <<
  //   // cloud->points[triangles.polygons[i].vertices[1]].z << "\n";

  //   // outfile << cloud->points[triangles.polygons[i].vertices[2]].x << "\t" <<
  //   // cloud->points[triangles.polygons[i].vertices[2]].y << "\t" <<
  //   // cloud->points[triangles.polygons[i].vertices[2]].z << "\n";



  //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[0]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[0]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[0]].z << "\n";

  //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[3]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[3]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[3]].z << "\n";

  //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[2]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[2]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[2]].z << "\n";

  //   // outfile << "\n";
  //   outfile << "\n";

  //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[2]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[2]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[2]].z << "\n";

  //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[1]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[1]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[1]].z << "\n";

  //   outfile << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[0]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[0]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[0]].z << "\n";

  //   // outfile << "\n";
  //   outfile << "\n";




  //   outfile1 << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[0]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[0]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[0]].z << "\n";

  //   outfile1 << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[3]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[3]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[3]].z << "\n";

  //   outfile1 << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[2]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[2]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[2]].z << "\n";

  //   outfile1 << setprecision(2) << mesh_cloud->points[triangles.polygons[i].vertices[1]].x << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[1]].y << "\t" <<
  //   mesh_cloud->points[triangles.polygons[i].vertices[1]].z << "\n";

  //   // outfile << "\n";
  //   outfile1 << "\n";
  
  // }
  // pcl::geometry::toHalfEdgeMesh(triangles, quads);


  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor (0, 0, 0);
  // viewer->addPolygonMesh (triangles,"meshes",0);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // viewer->addPointCloud<pcl::PointXYZI>(mesh_cloud);
// viewer->addPolygonMesh(new_quads, "meshses", 0);

  // viewer->addPointCloud<pcl::PointXYZI>(mesh_cloud);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
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
viewer->addLine(pcl::PointXYZ(test.c1.x, test.c1.y, test.c1.z), pcl::PointXYZ(test.c2.x, test.c2.y, test.c2.z), 255,0,0,"line1"+ std::to_string(cl));
  viewer->addLine(pcl::PointXYZ(test.c2.x, test.c2.y, test.c2.z), pcl::PointXYZ(test.c3.x, test.c3.y, test.c3.z), 0, 255,0,"line2"+ std::to_string(cl));
  viewer->addLine(pcl::PointXYZ(test.c3.x, test.c3.y, test.c3.z), pcl::PointXYZ(test.c4.x, test.c4.y, test.c4.z), 0,0,255,"line3"+ std::to_string(cl));
  viewer->addLine(pcl::PointXYZ(test.c4.x, test.c4.y, test.c4.z), pcl::PointXYZ(test.c1.x, test.c1.y, test.c1.z), 123,123,123,"line4"+ std::to_string(cl));
      // visualizeMesh(mesh_cloud, test, cl, viewer);
      q2.pop_front();
      cl++;

        // outfile1 << "\n";
        // q2.pop_front();
    }







  // }
  // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[0]], mesh_cloud->points[triangles.polygons[3].vertices[2]], 255,0,0,"line2");
  // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[3]], mesh_cloud->points[triangles.polygons[3].vertices[2]], 255,0,0,"line3");
  // // viewer->addCircle()
  // viewer->addSphere(mesh_cloud->points[triangles.polygons[3].vertices[0]],0.002,255,0,0,"sphere1");
  // viewer->addSphere(mesh_cloud->points[triangles.polygons[3].vertices[1]],0.002,0,255,0,"sphere2");
  // viewer->addSphere(mesh_cloud->points[triangles.polygons[3].vertices[2]],0.002,0,0,255,"sphere3");

  // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[1]], mesh_cloud->points[triangles.polygons[3].vertices[2]], 0,255,0, "poly1");
  // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[1]], mesh_cloud->points[triangles.polygons[3].vertices[0]], 0,255,0, "poly2");
  // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[0]], mesh_cloud->points[triangles.polygons[3].vertices[2]],  0,255,0,"poly3");
  // viewer->addLine(mesh_cloud->points[triangles.polygons[3].vertices[3]], mesh_cloud->points[triangles.polygons[3].vertices[0]],  0,255,0,"poly4");

  // viewer->addSphere(pcl::Vertices::vertices(q2.front().c1.x, q2.front().c1.y, q2.front().c1.z);

 viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()){
      viewer->spinOnce ();
      std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
//  viewer->addLine(cloud->points[triangles.polygons[3].vertices[1]], cloud->points[triangles.polygons[3].vertices[2]], 0,255,0, "poly1");
//   viewer->addLine(cloud->points[triangles.polygons[3].vertices[2]], cloud->points[triangles.polygons[3].vertices[3]], 0,255,0, "poly2");
//   viewer->addLine(cloud->points[triangles.polygons[3].vertices[3]], cloud->points[triangles.polygons[3].vertices[0]],  0,255,0,"poly3");
//   viewer->addLine(cloud->points[triangles.polygons[3].vertices[0]], cloud->points[triangles.polygons[3].vertices[1]],  0,255,0,"poly4");

  
}