#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
  if(argc != 4)
  {
    std::cerr << "Not enough inputs, try again!" << std::endl;
    return(-1);
  }
  // Read cloud_in
  reader.read(argv[1], *cloud_in);
  // Read cloud_out
  reader.read(argv[2], *cloud_out);
  /*
  // vg
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_vg (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_vg (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(0.001f, 0.001f, 0.001f);
  sor.filter(*in_vg);
  std::cerr << "cloud_in with " << in_vg->points.size() << " after voxel grid fiter." << std::endl;
  sor.setInputCloud(cloud_out);
  sor.filter(*out_vg);
  std::cerr << "cloud_out with " << out_vg->points.size() << " after voxel grid fiter." << std::endl;
  */
  /*
  // Segmentation
  // 1. Color
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_r (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_r (new pcl::PointCloud<pcl::PointXYZRGB>);
  int in_length = in_vg->points.size();
  int out_length = out_vg->points.size();
  int r_thres = 160;
  int in_count = 0;
  int out_count = 0;
  for(int i= 0; i< in_length; ++i)
  {
     if(in_vg->points[i].r > r_thres)
     {
        ++in_count;
     }
  }
  for(int i= 0; i< out_length; ++i)
  {
     if(out_vg->points[i].r > r_thres)
     {
        ++out_count;
     }
  }
  in_r->width = in_count;
  in_r->height = 1;
  in_r->points.resize(in_r->width * in_r->height);
  out_r->width = out_count;
  out_r->height = 1;
  out_r->points.resize(out_r->width * out_r->height);
  int j= 0;
  for(int i= 0; i< in_length; ++i)
  {
     if(in_vg->points[i].r > r_thres)
     {
        in_r->points[j].x = in_vg->points[i].x;
        in_r->points[j].y = in_vg->points[i].y;
        in_r->points[j].z = in_vg->points[i].z;
        in_r->points[j].r = in_vg->points[i].r;
        in_r->points[j].g = in_vg->points[i].g;
        in_r->points[j].b = in_vg->points[i].b;
        ++j;
     }
  }
  j= 0;
  for(int i= 0; i< out_length; ++i)
  {
     if(out_vg->points[i].r > r_thres)
     {
        out_r->points[j].x = out_vg->points[i].x;
        out_r->points[j].y = out_vg->points[i].y;
        out_r->points[j].z = out_vg->points[i].z;
        out_r->points[j].r = out_vg->points[i].r;
        out_r->points[j].g = out_vg->points[i].g;
        out_r->points[j].b = out_vg->points[i].b;
        ++j;
     }
  }
  // 2. SOR
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_sor (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_sor (new pcl::PointCloud<pcl::PointXYZRGB>);
  sor_.setInputCloud(in_r);
  sor_.setMeanK (200);
  sor_.setStddevMulThresh (0.2);
  sor_.filter (*in_sor);
  sor_.setInputCloud(out_r);
  sor_.setMeanK (200);
  sor_.setStddevMulThresh (0.2);
  sor_.filter (*out_sor);
  */
  // ICP
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.05);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (50);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1);
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  icp.align(*cloud_in);
  /*std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;*/
  std::cout << icp.getFinalTransformation() << std::endl;
  *cloud_out += *cloud_in;
  //std::cerr << "Final with " << Final.points.size() << " after voxel grid fiter." << std::endl; 
  // Save result
  writer.write<pcl::PointXYZRGB>(argv[3], *cloud_out, false);
  //std::cerr << "cloud_out with " << out_vg->points.size() << " after ICP." << std::endl; 
  return (0);
}
