#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <stdlib.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

using namespace pcl;
using namespace std;
using namespace pcl::io;
using namespace pcl::visualization;

// Visualize point cloud (XYZ)

void VisualizeXYZ(PointCloud<PointXYZ>::Ptr cloud) {
  CloudViewer viewer("3d viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {
  }
}

void VisualizeXYZRGB(PointCloud<PointXYZRGB>::Ptr cloud) {
  CloudViewer viewer("Cluster viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {
  }
}

int main(int argc, char** argv) {
  // Read and show input

  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

  PointCloud<PointXYZ>::Ptr cloud_s1(
      new PointCloud<PointXYZ>);  // Segment 1 ~ Planar model

  loadPCDFile<PointXYZ>("region_growing_rgb_tutorial.pcd", *cloud);

  // VisualizeXYZ(cloud_filtered);

  ModelCoefficients::Ptr coefficients(new ModelCoefficients);
  PointIndices::Ptr inliers(new PointIndices);

  pcl::SACSegmentation<PointXYZ> seg;

  // Optional

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  //VisualizeXYZ(cloud);

  ExtractIndices<PointXYZ> extract;

  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_s1);

  std::vector<int> nans;
  pcl::removeNaNFromPointCloud(*cloud_s1, *cloud_s1, nans);
  std::cout << "Number of Points after removing NANs: "
            << cloud_s1->points.size() << std::endl;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_s1);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1);  // 2cm
  ec.setMinClusterSize(500);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_s1);
  ec.extract(cluster_indices);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud_s1->points[*pit]);  //*
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: "
              << cloud_cluster->points.size() << " data points." << std::endl;

    std::string str = boost::lexical_cast<std::string>(j);
    cout << j << str << endl;


      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_cluster, rand()%255,rand()%255, rand()%255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, single_color, str);

    j++;
  }

  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return (0);
}
