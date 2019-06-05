//******************************************       ***********************************************
//#include <thread>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
//******************************************  ros  ***********************************************
#include <ros/ros.h>
#include <ros/package.h>
//******************************************  msgs  **********************************************
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
//******************************************  pcl  ***********************************************
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/filters/approximate_voxel_grid.h>



//************************************  pointcloud filter  ***************************************
void pointcloud_filter (pcl::PCLPointCloud2::Ptr cloud, bool passthrough)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
    voxel.setInputCloud (cloud);
    voxel.setLeafSize (0.2f, 0.2f, 0.2f);
    voxel.filter (*cloud_filtered);

    if(passthrough)
    {
      pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_trans);

      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud_trans);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits ( 0, 2);
      pass.filter (*cloud_filtered2);

      pcl::toPCLPointCloud2(*cloud_filtered2, *cloud);
    }
    else
    {
      *cloud = *cloud_filtered;
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "hw6_node");    

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map",1);

//**************************************  set variable  ******************************************
    sensor_msgs::PointCloud2 map_output;
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

//********************************  set cloud point variable  ************************************
    pcl::PCLPointCloud2::Ptr getpcd (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr map_trans (new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);

//**********************************  set icp parameter  *****************************************
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setMaximumIterations (300);

//**********************************  set initial guess  *****************************************
    //initial_guess(0,0) = cos( 140 * M_PI / 180 );
    //initial_guess(0,1) = -sin( 140 * M_PI / 180 );
    //initial_guess(1,0) = sin( 140 * M_PI / 180 );
    //initial_guess(1,1) = cos( 140 * M_PI / 180 );
    //initial_guess(0,3) = -2;
    //initial_guess(1,3) = 7;
    //initial_guess(2,3) = -3;

//********************************  get pcd package path  ****************************************
    std::string pkgpath = ros::package::getPath("hw6_0751915");

    ros::Rate rate(100);
    while(ros::ok())
    {

      for ( int i = 1 ; i <= 179 ; i = i+5 )
      {
      //*******************************  set pcd file path  **************************************
        std::string name = boost::to_string(i);
        std::string path = pkgpath + "/pcd/" + name + ".pcd";

      //*********************************  load pcd file  ****************************************
        pcl::PCDReader reader;
        reader.read( path , *getpcd);

      //********************************  pcd file filter  ***************************************
        pointcloud_filter (getpcd, true);

        if ( i != 1 )
        {
        //*******************************  set ndt source  ***************************************
          pcl::fromPCLPointCloud2( *getpcd, *input_cloud);
          icp.setInputSource (input_cloud);
          icp.setInputTarget (map);

        //*******************************  set ndt output  ***************************************
          ros::Time time = ros::Time::now();
          pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
          icp.align (*output_cloud, initial_guess);

          ROS_INFO("icp calculation score: %f ", icp.getFitnessScore ());
          ROS_INFO("icp calculation time: %f second", (ros::Time::now() - time).toSec());
        //********************************  transforming  ****************************************
          pcl::transformPointCloud (*input_cloud, *output_cloud, icp.getFinalTransformation ());
          initial_guess = icp.getFinalTransformation ();

        //******************************  combine two pcd  ***************************************
          *map += *output_cloud;

        //*******************************  map filtered  *****************************************
          pcl::toPCLPointCloud2( *map, *map_trans);
          pointcloud_filter (map_trans, false);
          pcl::fromPCLPointCloud2( *map_trans, *map);

        //*************************  transfer type and add header  *******************************
          pcl::toROSMsg (*map, map_output);
          map_output.header.frame_id = "map";

        //**********************************  pub map  *******************************************
          map_pub.publish(map_output);
          ros::spinOnce();
          rate.sleep();
        }
        else
        {
          pcl::fromPCLPointCloud2( *getpcd, *map);
        }
      }

    //************************************  save map  ********************************************
      pcl::io::savePCDFileASCII ( pkgpath + "/map.pcd", *map);
      ROS_INFO(" Map Saved !!! ");

      while(ros::ok())
      {
        map_pub.publish(map_output);
        ros::spinOnce();
        rate.sleep();
      }
    }
    ros::spin();
    return (0);
}
