//******************************************  main  **********************************************
#include <time.h>
#include <string>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
//******************************************  ros  ***********************************************
#include <ros/ros.h>
//******************************************  msgs  **********************************************
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
//*****************************************  matker  *********************************************
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//******************************************  pcl  ***********************************************
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/ndt.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/approximate_voxel_grid.h>
//*****************************************  OpenCV  *********************************************
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>

ros::Publisher plane_filtered_pub;
ros::Publisher cluster_pub;

cv::Mat intrisicMat(3, 3, cv::DataType<float>::type); // Intrisic matrix
cv::Mat rMat(3, 3, cv::DataType<float>::type); // Rotation matrix
cv::Mat tVec(3, 1, cv::DataType<float>::type); // Translation vector
cv::Mat distCoeffs(5, 1, cv::DataType<float>::type);   // Distortion vector
int count = 0;

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

void PointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::vector<cv::Point3f> points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromROSMsg(*msg, *cloud);

    for(int i=0; i<=cloud->points.size(); i++)
    {
        points.push_back(cv::Point3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }

    if(count == 0)
    {
        rMat.at<float>(0, 0) = 0.845929;
        rMat.at<float>(1, 0) = 0.045996;
        rMat.at<float>(2, 0) = -0.531307;

        rMat.at<float>(0, 1) = 0.533284;
        rMat.at<float>(1, 1) = -0.079141;
        rMat.at<float>(2, 1) = 0.842226;

        rMat.at<float>(0, 2) = -0.003308;
        rMat.at<float>(1, 2) = -0.995802;
        rMat.at<float>(2, 2) = -0.091477;

        tVec.at<float>(0) = 0.092240;
        tVec.at<float>(1) = -0.357097;
        tVec.at<float>(2) = -0.160559;

        intrisicMat.at<float>(0, 0) = 698.939;
        intrisicMat.at<float>(1, 0) = 0;
        intrisicMat.at<float>(2, 0) = 0;
 
        intrisicMat.at<float>(0, 1) = 0;
        intrisicMat.at<float>(1, 1) = 698.939;
        intrisicMat.at<float>(2, 1) = 0;
 
        intrisicMat.at<float>(0, 2) = 641.868;
        intrisicMat.at<float>(1, 2) = 385.788;
        intrisicMat.at<float>(2, 2) = 1;

        distCoeffs.at<float>(0) = -0.171466;
        distCoeffs.at<float>(1) = 0.0246144;
        distCoeffs.at<float>(2) = 0;
        distCoeffs.at<float>(3) = 0;
        distCoeffs.at<float>(4) = 0;

        ROS_INFO("camera parameter set");
        count++;
    }

    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(points, rMat, tVec, intrisicMat, distCoeffs, projectedPoints);   

    /*Matrix4f T_imu2cam;
    T_imu2cam <<  0.845929,  0.533284, -0.003308,  0.092240,
                  0.045996, -0.079141, -0.995802, -0.357097,
                 -0.531307,  0.842226, -0.091477, -0.160559,
                         0,         0,         0,         1;

    pcl::transformPointCloud ( *cloud, *cloud_trans, T_imu2cam);*/
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "hw7_node");    

    ros::NodeHandle nh;
    cluster_pub = nh.advertise<sensor_msgs::PointCloud2 >("cluster_cloud", 1);
    plane_filtered_pub = nh.advertise<sensor_msgs::PointCloud2 >("plane_filtered_pub_points", 1);

    ros::Subscriber point_data = nh.subscribe("/cluster_cloud", 1, &PointCallback);
    ros::Subscriber lidar_sub = nh.subscribe("points_raw", 1, lidar_callback);
    
    /*image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/zed/left/image_raw_color", 1, imageCallback);
    image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);*/

    ros::Rate rate(100);
    while(ros::ok())
    {
      rate.sleep();
      ros::spinOnce();
    }
    ros::spin();
    return (0);
}
