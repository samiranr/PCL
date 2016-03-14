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

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/filters/voxel_grid.h>

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
  // Read a colored cloud


  PointCloud<PointXYZRGB>::Ptr cloud_colored(new PointCloud<PointXYZRGB>);

  loadPCDFile<PointXYZRGB>("train_1.pcd", *cloud_colored);

 


  VisualizeXYZRGB(cloud_colored);


/*
//Print points one by one (for debugging)
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud_colored->points.size (); ++i)
    std::cerr << "    " << cloud_colored->points[i].x << " " 
                        << cloud_colored->points[i].y << " " 
                        << cloud_colored->points[i].z << std::endl;
  
*/




// Remove NANS

     cout << "Number of Points before removing NANs: "
            << cloud_colored->points.size() <<endl;       




  vector<int> nans;
  removeNaNFromPointCloud(*cloud_colored, *cloud_colored, nans);
  cout << "Number of Points after removing NANs: "
            << cloud_colored->points.size() <<endl;




// Downsample the cloud ( The parameters of all further processes depend of the downsampling factor)

            /*


cout<< "PointCloud before downsampling: " << cloud_colored->width * cloud_colored->height 
       << " data points (" << getFieldsList (*cloud_colored) << ")."<<endl;

  // Create the filtering object
  VoxelGrid<PointXYZRGB> downsample;
  downsample.setInputCloud (cloud_colored);
  downsample.setLeafSize (0.01f, 0.01f, 0.01f);
  downsample.filter (*cloud_colored);
cout<< "PointCloud after downsampling: " << cloud_colored->width * cloud_colored->height 
       << " data points (" << getFieldsList (*cloud_colored) << ")."<<endl;


*/




// Thresholds the z axis - going forward from the car

  PassThrough<PointXYZRGB> z_axis_threshold;
  z_axis_threshold.setInputCloud (cloud_colored);
  z_axis_threshold.setFilterFieldName ("z");
  z_axis_threshold.setFilterLimits (0.0, 15); // Decide range of z axis
  //pass.setFilterLimitsNegative (true);
  z_axis_threshold.filter (*cloud_colored);


// Thresholds the y axis - going towards the sky

   PassThrough<PointXYZRGB> y_axis_threshold;
  y_axis_threshold.setInputCloud (cloud_colored);
  y_axis_threshold.setFilterFieldName ("y"); // Decide range of y axis
  y_axis_threshold.setFilterLimits (0, 5);
  //pass.setFilterLimitsNegative (true);
  y_axis_threshold.filter (*cloud_colored);






// RANSAC plane/road extraction



  ModelCoefficients::Ptr coefficients(new ModelCoefficients);
  PointIndices::Ptr inliers(new PointIndices);
  SACSegmentation<PointXYZRGB> seg;


  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.1); // Closest distance to another point to be considered on the same plane // To be tuned 

  seg.setInputCloud(cloud_colored);
  seg.segment(*inliers, *coefficients);

  //VisualizeXYZ(cloud);

  ExtractIndices<PointXYZRGB> extract;

  extract.setInputCloud(cloud_colored);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_colored);


// RANSAC plane/road extraction ends here



  // Filtering outliers (Might be of use later)
  // Create the filtering object
/*
StatisticalOutlierRemoval<PointXYZRGB> sor;
  sor.setInputCloud (cloud_colored);
  sor.setMeanK (20);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_colored);

*/ 




  // Start Clustering based on Eucilidean Distance



  search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new search::KdTree<PointXYZRGB>);


  
  tree->setInputCloud(cloud_colored);

  vector<PointIndices> cluster_indices;
  EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance(0.1);  // 2cm  // Cluster tolerace // To be tuned
  ec.setMinClusterSize(500);   // Minimum Cluster size // To be tuned
  ec.setMaxClusterSize(25000);  // According to the need 
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_colored);
  ec.extract(cluster_indices);



// Visualize the clusters using different colors


  boost::shared_ptr<visualization::PCLVisualizer> viewer(
      new visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  int j = 0;
  for (vector<PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    PointCloud<PointXYZRGB>::Ptr cloud_cluster(
        new PointCloud<PointXYZRGB>);
    for (vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud_colored->points[*pit]);  //*
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    cout << "PointCloud representing the Cluster: "
              << cloud_cluster->points.size() << " data points." << std::endl;

    string str = boost::lexical_cast<std::string>(j);
    cout << j << str << endl;


      visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_cluster, rand()%255,rand()%255, rand()%255);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_cluster, single_color, str);

    j++;
  }



// Start the visualization

  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  //VisualizeXYZRGB(cloud_colored);

  return (0);
}
