/* Revised from http://pointclouds.org/documentation/tutorials/voxel_grid.php
   Input: Pointcloud from a pcd file
   Output: Voxelgrid downsampled pointcloud pcd file
   Commandline type: ./voxel (file number)
  */
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  std::stringstream si;
  if(argc!=2)
  {
    std::cerr << "Not enough input, try again!" << std::endl;
    return -1;
  }
  si << "../strawberry_" << argv[1] << ".pcd";
  // Replace the path below with the path where you saved your file
  reader.read (si.str(), *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering objectvim str
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.001f, 0.001f, 0.001f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
  std::stringstream so;
  so << "../strawberry_" << argv[1] << "_downsampled" << ".pcd";
  writer.write (so.str(), *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}