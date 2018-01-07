#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
int main(int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZRGB> ());
	if(argc!=4)
	{
		std::cerr << "Not enough input, try again!" << std::endl;
		return 0;
	}
	reader.read (argv[1], *cloud);
	reader.read (argv[2], *cloud_ori);
	pcl::PCA<pcl::PointXYZRGB> pca;
	pca.setInputCloud(cloud);
	Eigen::Matrix3f eigen = pca.getEigenVectors();
	Eigen::Vector4f center = pca.getMean();
	//std::cout << eigen(0,0) << " " << eigen(0,1) << " " << eigen(0,2) << std::endl;
	std::cout<< cloud_ori->points.size() << std::endl;
	std::cout<< cloud_ori->width << std::endl;
	std::cout<< cloud_ori->height << std::endl;
	int len = 8;
	for(int i=0;i<len;++i)
	{
		pcl::PointXYZRGB point;
		point.x = center(0) + i* eigen(0,0) * 0.05;
		point.y = center(1) + i* eigen(0,1) * 0.05;
		point.z = center(2) + i* eigen(0,2) * 0.05;
		point.r = 0;
		point.g = 255;
		point.b = 255;
		cloud_ori->points.push_back(point);
	}
	//reader.read (argv[1], *cloud);
	cloud_ori->width += cloud_ori->points.size() + len;
	cloud_ori->height = 1;
	cloud_ori->resize(cloud_ori->width * cloud_ori->height);
	std::cout<< cloud_ori->points.size() << std::endl;
	std::cout<< cloud_ori->width << std::endl;
	std::cout<< cloud_ori->height << std::endl;

	writer.write<pcl::PointXYZRGB>(argv[3], *cloud_ori, false);

	return 0;
}