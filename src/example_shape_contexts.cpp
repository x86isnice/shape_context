
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/impl/3dsc.hpp>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <vector>

using namespace pcl;

int main (int, char** argv)
{
  // std::string filename = argv[1];
  std::string filename = "/home/x86isnice/michael.pcd";


  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile <pcl::PointXYZ> (filename.c_str (), *cloud) == -1)
  // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }
  std::cout << "Loaded " << cloud->points.size () << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation.setSearchMethod (kdtree);

  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
 // normal_estimation.setRadiusSearch (0.05);
  normal_estimation.setKSearch(100);
  normal_estimation.compute (*normals);
  std::cout << " normals estimated completed!" << std::endl;
  // Setup the shape context computation
  pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> shape_context;

  // Provide the point cloud
  shape_context.setInputCloud (cloud);
  // Provide normals
  shape_context.setInputNormals (normals);
  // Use the same KdTree from the normal estimation
  shape_context.setSearchMethod (kdtree);
  std::cout << "shape context begin !" << std::endl;
  pcl::PointCloud<pcl::ShapeContext1980>::Ptr shape_context_features (new pcl::PointCloud<pcl::ShapeContext1980>);

  // The minimal radius is generally set to approx. 1/10 of the search radius, while the pt. density radius is generally set to 1/5
  //shape_context.setRadiusSearch (0.5);
  shape_context.setRadiusSearch (70);
  shape_context.setPointDensityRadius (0.04);
  shape_context.setMinimalRadius (0.05);
  //shape_context.setKSearch(100);

  std::cout << "shape context end !" << std::endl;
  // Actually compute the shape contexts
  shape_context.compute (*shape_context_features);

  pcl::io::savePCDFileASCII("../good.pcd" , *shape_context_features);

  std::cout << "3DSC output points.size (): " << shape_context_features->points.size() << std::endl;
  // Display and retrieve the shape context descriptor vector for the 0th point.
  for (std::size_t i = 0; i < shape_context_features->points.size(); i++ )
  {
  std::cout << shape_context_features->points[i] << std::endl;

  //float* first_descriptor = shape_context_features->points[0].descriptor; // 1980 elements
  }
  return 0;
}
