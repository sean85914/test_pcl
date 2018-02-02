#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

int main (int argc, char** argv)
{
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_cp (new pcl::PointCloud <pcl::PointXYZRGB>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[1], *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
  *cloud_cp = *cloud;
  pcl::IndicesPtr indices (new std::vector <int>);

  pcl::MinCutSegmentation<pcl::PointXYZRGB> seg;
  seg.setInputCloud (cloud);
  //seg.setIndices (indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointXYZRGB point;
  point.x = 0.336;
  point.y = 0.006;
  point.z = -0.045;
  point.r = 255;
  point.g = 255;
  point.b = 255;
  foreground_points->points.push_back(point);
  seg.setForegroundPoints (foreground_points);

  seg.setSigma (0.25);
  seg.setRadius (0.03);
  seg.setNumberOfNeighbours (14);
  seg.setSourceWeight (0.8);

  std::vector <pcl::PointIndices> clusters;
  seg.extract (clusters);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr segmentation(new pcl::PointCloud<pcl::PointXYZRGB> ());

  std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  
  int count = 0;
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it_1 = colored_cloud->begin(); it_1 != colored_cloud->end(); it_1 ++)
  {
    //std::cout << std::distance(colored_cloud->begin(), it_1) <<std::endl;
    if(it_1->r == 255 && it_1->g== 255 && it_1->b == 255)
    {
      count++;
      pcl::PointXYZRGB point_to_add;
      point_to_add.x = it_1->x;
      point_to_add.y = it_1->y;
      point_to_add.z = it_1->z;
      point_to_add.r = it_1->r;
      point_to_add.g = it_1->g;
      point_to_add.b = it_1->b;
      segmentation->points.push_back(point_to_add);
    }
  }
  std::cout << count << std::endl;
  
  segmentation->width = count;
  segmentation->height = 1;
  segmentation->points.resize(count);
  /*int count = 0;

  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = colored_cloud->begin(); it != colored_cloud->end(); it ++)
  {
    if(it->r == 255 && it->g == 255 && it->b == 255)
    {
        count ++;
    }
  }*/
  //std::cout << "There are " << count << " points after segmentation." << std::endl;
  viewer.showCloud(foreground_points);
  while (!viewer.wasStopped ())
  {
  }
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB>("seg.pcd", *segmentation, false);
  return (0);
}