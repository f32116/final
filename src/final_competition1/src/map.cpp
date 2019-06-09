//******************************************       ***********************************************
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
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

//************************************  pointcloud filter  ***************************************
void pointcloud_filter (pcl::PCLPointCloud2::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
    voxel.setInputCloud (cloud);
    voxel.setLeafSize (0.5f, 0.5f, 0.5f);
    voxel.filter (*cloud);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "map_node");    

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map",1);

//**************************************  set variable  ******************************************
    sensor_msgs::PointCloud2 map_output;
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();

//********************************  set cloud point variable  ************************************
    pcl::PCLPointCloud2::Ptr getpcd (new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_trans (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);

//**********************************  set initial guess  *****************************************
    //initial_guess(0,0) = cos( 140 * M_PI / 180 );
    //initial_guess(0,1) = -sin( 140 * M_PI / 180 );
    //initial_guess(1,0) = sin( 140 * M_PI / 180 );
    //initial_guess(1,1) = cos( 140 * M_PI / 180 );
    //initial_guess(0,3) = -2;
    //initial_guess(1,3) = 7;
    //initial_guess(2,3) = -3;

//********************************  get pcd package path  ****************************************
    std::string pkgpath = ros::package::getPath("final_competition1");

//**********************************  process fist pcd  ******************************************
    for ( int i = 4 ; i <= 7 ; i++ ) 
    {
    //********************************  set pcd file path  ***************************************
        std::string name = boost::to_string(i);
        std::string path = pkgpath + "/map/first-" + name + ".pcd";

    //**********************************  load pcd file  *****************************************
        pcl::PCDReader reader;
        reader.read( path , *getpcd);

    //*********************************  map filtered  *******************************************
        pointcloud_filter (getpcd);
        pcl::fromPCLPointCloud2( *getpcd, *pcd_trans);

    //********************************  combine two pcd  *****************************************
        *map += *pcd_trans;
    }

//*********************************  process submap pcd  *****************************************
    for ( int i = 15 ; i <= 20 ; i++ )
    {
        if( i != 16 and i != 17 )
        {
        //******************************  set pcd file path  *************************************
            std::string name = boost::to_string(i);
            std::string path = pkgpath + "/map/submap_" + name + ".pcd";

        //********************************  load pcd file  ***************************************
            pcl::PCDReader reader;
            reader.read( path , *getpcd);

        //*******************************  map filtered  *****************************************
            pointcloud_filter (getpcd);
            pcl::fromPCLPointCloud2( *getpcd, *pcd_trans);

        //******************************  combine two pcd  ***************************************
            *map += *pcd_trans;
        }
    }

//*****************************  transfer type and add header  ***********************************
    pcl::toROSMsg (*map, map_output);
    map_output.header.frame_id = "map";

//**************************************  save map  **********************************************
    pcl::io::savePCDFileASCII ( pkgpath + "/NCTU_map.pcd", *map);
    ROS_INFO(" Map Saved !!! ");

    ros::Rate rate(100);
    while(ros::ok())
    {
        map_pub.publish(map_output);
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    return (0);
}
