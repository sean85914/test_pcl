#include <iostream>	
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/min_cut_segmentation.h>
int main (int argc, char** argv)
{
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_color (new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud <pcl::PointXYZRGB>);
	if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[1], *cloud) == -1 )
  	{
    	std::cout << "Cloud reading failed." << std::endl;
    	return (-1);
  	}
  	int count = 0;
  	std::cout << cloud->points.size() << std::endl;

	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it=cloud->begin(); it!= cloud->end(); it++)
	{
		if(it->r<=255 && it->r >=86 && it->g>=0 && it->g <=70 && it->b >=0 && it->b <= 82)
		{
			pcl::PointXYZRGB point_to_add;
			point_to_add.x = it->x;
         	point_to_add.y = it->y;
          	point_to_add.z = it->z;
          	point_to_add.r = it->r;
          	point_to_add.g = it->g;
          	point_to_add.b = it->b;
          	cloud_color->points.push_back(point_to_add);
          	count ++;
		}
	}
	cloud_color->width = count;
	cloud_color->height = 1;
	cloud_color->points.resize(count);
	std::cout << count << std::endl;
	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  	sor.setInputCloud (cloud_color);
  	// Changeable coefficients
  	sor.setMeanK (200);
  	sor.setStddevMulThresh (0.2);

  	sor.filter (*cloud_filtered);
  	Eigen::Vector4f centroid; 
  	pcl::compute3DCentroid(*cloud_filtered, centroid);
  	std::cout << centroid << std::endl;
	
  	pcl::PCDWriter writer;
  	writer.write<pcl::PointXYZRGB>(argv[2], *cloud_filtered, false);
  	return (0);

}