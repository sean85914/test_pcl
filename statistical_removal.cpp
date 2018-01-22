/*
  Revised from: http://pointclouds.org/documentation/tutorials/statistical_outlier.php
  Input: Pointcloud from pcd file
  Output: Pointcloud pcd file that remove outliers from the origin one
  Commandline type: ./removal (file number)
*/
#include <iostream>
#include <fstream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::fstream fs;
  fs.open("removal.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  if(argc!=3)
  {
    std::cerr << "Not enough input, try again!" << std::endl;
    return -1;
  }
  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZRGB> (argv[1], *cloud);

  //std::cerr << "Cloud before filtering: " << std::endl;
  //std::cerr << *cloud << std::endl;
  int start_s = clock();
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  // Changeable coefficients
  sor.setMeanK (200);
  sor.setStddevMulThresh (0.2);

  sor.filter (*cloud_filtered);
  int stop_s = clock();
  //std::cerr << "Cloud after filtering: " << std::endl;
  //std::cerr << *cloud_filtered << std::endl;
  fs << argv[1] << " " << cloud->points.size() << " " << cloud_filtered->points.size() << " " << (stop_s - start_s)/double(CLOCKS_PER_SEC) * 1000 << std::endl;
  fs.close();
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> (argv[2], *cloud_filtered, false);
  /*  Outliers, not used here.
  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
  */
  return (0);
}
