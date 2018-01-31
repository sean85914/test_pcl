#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read (argv[1], *cloud);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
  // Voxel grid
  /*// Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_vg); */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vg (new pcl::PointCloud<pcl::PointXYZRGB>);
  *cloud_vg = *cloud;
  std::cout << "PointCloud after filtering has: " << cloud_vg->points.size ()  << " data points." << std::endl; //*

  /*// Reject red
  int r_thres = atoi(argv[2]);
  std::cout << "Threshold: "<< r_thres << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rr (new pcl::PointCloud<pcl::PointXYZRGB>);
  int count = 0;
  for(int i= 0; i<cloud_vg->points.size(); ++i)
  {
    if(cloud_vg->points[i].r < r_thres)
    {
      ++count;
    }
  }
  cloud_rr->width = count;
  cloud_rr->height = 1;
  cloud_rr->points.resize(cloud_rr->width * cloud_rr->height);
  int k= 0;
  for(int i= 0; i<cloud_vg->points.size(); ++i)
  {
    if(cloud_vg->points[i].r < r_thres)
    {
      cloud_rr->points[k].x = cloud_vg->points[i].x;
      cloud_rr->points[k].y = cloud_vg->points[i].y;
      cloud_rr->points[k].z = cloud_vg->points[i].z;
      cloud_rr->points[k].r = cloud_vg->points[i].r;
      cloud_rr->points[k].g = cloud_vg->points[i].g;
      cloud_rr->points[k].b = cloud_vg->points[i].b;
      ++k;
    }
  }
  std::cout << "PointCloud after color filtering has: " << cloud_rr->points.size ()  << " data points." << std::endl; //*
  */
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylin (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
  //writer.write<pcl::PointXYZRGB> ("reject_red.pcd", *cloud_rr, false);
  // Compute normal
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_vg);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
  // Find model of type cylinder
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.005);
  //seg.setRadiusLimits(0, 0.001);
  seg.setInputCloud(cloud_vg);
  seg.setInputNormals(cloud_normals);
  seg.segment (*inliers, *coefficients);
  pcl::visualization::PCLVisualizer viewer ("Pointcloud with normal");
  viewer.addPointCloud (cloud_vg, "point_cloud");
  viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_vg, cloud_normals, 10, 0.005, "normals");
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
  // Extract cylinder
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud_vg);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_cylin);
  //writer.write("cylinder.pcd", *cloud_cylin, false);
  //Reject red
  int r_thres = atoi(argv[2]);
  std::cout << "Threshold: "<< r_thres << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rr (new pcl::PointCloud<pcl::PointXYZRGB>);
  int count = 0;
  for(int i= 0; i<cloud_cylin->points.size(); ++i)
  {
    if(cloud_cylin->points[i].r < r_thres)
    {
      ++count;
    }
  }
  cloud_rr->width = count;
  cloud_rr->height = 1;
  cloud_rr->points.resize(cloud_rr->width * cloud_rr->height);
  int k= 0;q
  for(int i= 0; i<cloud_cylin->points.size(); ++i)
  {
    if(cloud_vg->points[i].r < r_thres)
    {
      cloud_rr->points[k].x = cloud_cylin->points[i].x;
      cloud_rr->points[k].y = cloud_cylin->points[i].y;
      cloud_rr->points[k].z = cloud_cylin->points[i].z;
      cloud_rr->points[k].r = cloud_cylin->points[i].r;
      cloud_rr->points[k].g = cloud_cylin->points[i].g;
      cloud_rr->points[k].b = cloud_cylin->points[i].b;
      ++k;
    }
  }
  writer.write("cylinder.pcd", *cloud_rr, false);
  /*int i=0, nr_points = (int) cloud_rr->points.size ();
  //std::cout << "while loop" << std::endl;
  int cc =0;
  while (cloud_rr->points.size () > 0.3 * nr_points)
  {
    // Segment the largest stick component from the remaining cloud
    seg.setInputCloud (cloud_rr);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a stick model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_rr);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_stick);
    std::stringstream name;
    name << "stick" << cc << ".pcd";
    ++cc;
    writer.write(name.str(), *cloud_stick, false);
    std::cout << "PointCloud representing the stick component: " << cloud_stick->points.size () << " data points." << std::endl;
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_rr = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_rr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.003); // 2cm
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_rr);
  ec.extract (cluster_indices);
  int j = 0;
  //std::cout << "for loop" << std::endl;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_rr->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
  */
  return (0);
}