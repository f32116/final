#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <string>
#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <iomanip>
#include <time.h>

using namespace boost::filesystem;

int WIDTH = 1280, HEIGHT = 720;
Eigen::Matrix4f l_t_c;
cv::Mat INTRINSIC_MAT(3, 3, cv::DataType<double>::type); // Intrinsics
cv::Mat DIST_COEFFS(5, 1, cv::DataType<double>::type); // Distortion vector

ros::Publisher plane_filtered_pub;
ros::Publisher cluster_pub;

void initializeGlobalParams() {
  l_t_c << 0.84592974185943604, 0.53328412771224976, -0.0033089336939156055, 0.092240132391452789,
           0.045996580272912979, -0.079141519963741302, -0.99580162763595581, -0.35709697008132935, 
           -0.53130710124969482, 0.84222602844238281, -0.091477409005165100, -0.16055910289287567,
           0, 0, 0, 1;

  INTRINSIC_MAT.at<double>(0, 0) = 698.939;
  INTRINSIC_MAT.at<double>(1, 0) = 0;
  INTRINSIC_MAT.at<double>(2, 0) = 0;

  INTRINSIC_MAT.at<double>(0, 1) = 0;
  INTRINSIC_MAT.at<double>(1, 1) = 698.939;
  INTRINSIC_MAT.at<double>(2, 1) = 0;

  INTRINSIC_MAT.at<double>(0, 2) = 641.868;
  INTRINSIC_MAT.at<double>(1, 2) = 385.788;
  INTRINSIC_MAT.at<double>(2, 2) = 1.0;

  DIST_COEFFS.at<double>(0) = -0.171466;
  DIST_COEFFS.at<double>(1) = 0.0246144;
  DIST_COEFFS.at<double>(2) = 0;
  DIST_COEFFS.at<double>(3) = 0;
  DIST_COEFFS.at<double>(4) = 0;
}


void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  fromROSMsg(*msg, *cloud);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*cloud_filtered);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.05);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);
  *cloud_filtered = *cloud_f;

  sensor_msgs::PointCloud2 filtered_cloud;
  pcl::toROSMsg(*cloud_filtered, filtered_cloud);
  filtered_cloud.header.frame_id = "velodyne";
  plane_filtered_pub.publish(filtered_cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance (1); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (1500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  std::cout << "Cluster size: " << cluster_indices.size() << std::endl;


  int j = 50;
  int k = 0;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZI>);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    
    // extract clusters and save as a single point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      cloud_filtered->points[*pit].intensity = j;
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    *cloud_clusters += *cloud_cluster;

    // compute cluster centroid and publish
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    j+=2;

  }

  sensor_msgs::PointCloud2 cluster_cloud;
  pcl::toROSMsg(*cloud_clusters, cluster_cloud);
  cluster_cloud.header.frame_id = "velodyne";
  cluster_pub.publish(cluster_cloud);
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  plane_filtered_pub = nh.advertise<sensor_msgs::PointCloud2 >("plane_filtered_pub_points", 1);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2 >("cluster_cloud", 1);

  ros::Subscriber lidar_sub = nh.subscribe("points_raw", 1, lidar_callback);
  
  initializeGlobalParams();
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }
}

