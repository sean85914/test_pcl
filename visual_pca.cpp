#include <iostream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
int main(int argc, char** argv){
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZRGB> ());
	if(argc!=3)
	{
		std::cerr << "Not enough input, try again!" << std::endl;
		return 0;
	}
	reader.read (argv[1], *cloud);
	reader.read (argv[2], *cloud_ori);
	pcl::PCA<pcl::PointXYZRGB> pca;
	int start_s = clock();
	pca.setInputCloud(cloud);
	Eigen::Matrix3f eigen = pca.getEigenVectors();
	Eigen::Vector4f center = pca.getMean();

	pcl::PointXYZ point1, point2;
	point1.x = center(0);
	point1.y = center(1);
	point1.z = center(2);
	point2.x = center(0) + eigen(0, 0) * 0.1;
	point2.y = center(1) + eigen(0, 1) * 0.1;
	point2.z = center(2) + eigen(0, 2) * 0.1;
	int stop_s = clock();
	std::cerr << "Processed time:" << double(stop_s - start_s)/CLOCKS_PER_SEC * 1000 <<" ms" << std::endl;
	//writer.write<pcl::PointXYZRGB>(argv[3], *cloud_ori, false);
	pcl::visualization::PCLVisualizer viewer ("Strawberry with vector");
	viewer.addPointCloud (cloud_ori, "point_cloud");
	viewer.addArrow(point2, point1, 1, 0, 0, false);
	viewer.addSphere(point1, 0.005);

 	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
 	//viewer.setPosition(800, 400); // Setting visualiser window position

  	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    	viewer.spinOnce ();
  	}
	return 0;
}