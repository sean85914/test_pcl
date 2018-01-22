/* Revised from http://pointclouds.org/documentation/tutorials/voxel_grid.php
   Input: Pointcloud from a pcd file
   Output: Voxelgrid downsampled pointcloud pcd file
   Commandline type: ./voxel (file number)
  */
#include <iostream>
#include <fstream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  std::fstream fs;
  fs.open("voxel_grid.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  // Fill in the cloud data
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  std::stringstream si;
  if(argc!=3)
  {
    std::cerr << "Not enough input, try again!" << std::endl;
    return -1;
  }
  //si << "../strawberry_" << argv[1] << ".pcd";
  // Replace the path below with the path where you saved your file
  reader.read (argv[1], *cloud); // Remember to download the file first!

  //std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
  //     << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
  // Start calculate time
  int start_s = clock();
  // Create the filtering objectvim str
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.001f, 0.001f, 0.001f);
  sor.filter (*cloud_filtered);
  // End calculate time 
  int stop_s = clock();
  //std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
  //     << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
  //std::stringstream so;
  //so << "../strawberry_" << argv[1] << "_downsampled" << ".pcd";
  writer.write (argv[2], *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  fs << argv[1] <<" "<< cloud->width * cloud->height << " " << cloud_filtered->width * cloud_filtered->height << " " << (stop_s - start_s)/double(CLOCKS_PER_SEC) * 1000 << std::endl;
  fs.close();
  return (0);
}