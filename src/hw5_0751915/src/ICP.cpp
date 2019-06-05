#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Output (new pcl::PointCloud<pcl::PointXYZ>);


Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity(); 

class hw5_node
{
  private:
    ros::NodeHandle nh, nh2;
    ros::Subscriber point_data;
    ros::Publisher scan_pub;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    sensor_msgs::PointCloud2 scan_output;


  public:
    hw5_node()
    {
    //*********************************  set pub and sub  ****************************************
        scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan",1);
        point_data = nh2.subscribe("/points_raw", 1, &hw5_node::PointCallback, this);

    //********************************  set initial guess  ***************************************
        initial_guess(0,0) = cos( 140 * M_PI / 180 );
        initial_guess(0,1) = -sin( 140 * M_PI / 180 );
        initial_guess(1,0) = sin( 140 * M_PI / 180 );
        initial_guess(1,1) = cos( 140 * M_PI / 180 );
        initial_guess(0,3) = -2;
        initial_guess(1,3) = 7;
        initial_guess(2,3) = -3;

    //********************************  set icp parameter  ***************************************
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(0.01);
	    icp.setMaximumIterations (200);
    }

    //*******************************  scan data callback  ***************************************
    void PointCallback(const sensor_msgs::PointCloud2::ConstPtr& pcloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2::Ptr cloud_trans1 (new pcl::PCLPointCloud2 ());
        pcl::PCLPointCloud2::Ptr cloud_filtered2 (new pcl::PCLPointCloud2 ());

        pcl_conversions::toPCL( *pcloud, *cloud_trans1);

        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloud_trans1);
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
        sor.filter (*cloud_filtered2);

        pcl::fromPCLPointCloud2(*cloud_filtered2, *cloud_trans2);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_trans2);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-1.5, 1);
        pass.filter (*tcloud);

    //*********************************  set icp source  *****************************************
        icp.setInputSource(tcloud);
        icp.setInputTarget(cloud_trans);

    //*************************************  do icp  *********************************************
        icp.align(*Output, initial_guess);
        initial_guess = icp.getFinalTransformation();
        //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        initial_guess = icp.getFinalTransformation();

    //************************************  pub scan  ********************************************
        pcl::toROSMsg (*tcloud, scan_output);
        scan_output.header.frame_id = "scan";
        scan_pub.publish(scan_output);

    //**********************************  set tf value  ******************************************
        static tf::TransformBroadcaster br;
        tf::Matrix3x3 rotation(initial_guess(0,0), initial_guess(0,1), initial_guess(0,2),
                               initial_guess(1,0), initial_guess(1,1), initial_guess(1,2),
                               initial_guess(2,0), initial_guess(2,1), initial_guess(2,2));
        tf::Quaternion q;
        rotation.getRotation(q);
        q.normalize();
        tf::Vector3 translation(initial_guess(0,3), initial_guess(1,3), initial_guess(2,3));
        tf::Transform transform(q, translation);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "scan"));
    }

    //******************************  shutdown sub and pub  **************************************
    ~hw5_node()
    {
        scan_pub.shutdown();
        point_data.shutdown();
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "hw5_node");    
    hw5_node hw5;

    ros::NodeHandle nh1;
    ros::Publisher map_pub = nh1.advertise<sensor_msgs::PointCloud2>("/map",1);
    sensor_msgs::PointCloud2 map_output;

//********************************  set cloud point variable  ************************************
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    
//**********************************  set pcd file path  *****************************************
    std::string pkgpath = ros::package::getPath("hw5_0751915");
    std::string mappath = "/map.pcd";
    std::string path = pkgpath + mappath;

//************************************  load pcd file  *******************************************
    pcl::PCDReader reader;
    reader.read( path , *cloud);

//**********************************  use voxel filter  ******************************************
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*cloud_filtered);

//************************  transfer pcd file type and add header  *******************************
    pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_trans);

//*****************************  transfer type and add header  ***********************************
    pcl::toROSMsg (*cloud_trans, map_output);
    map_output.header.frame_id = "map";

    ros::Rate rate(100);
    while(ros::ok())
    {
    //************************************  pub map  *********************************************
        map_pub.publish(map_output);

        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    return (0);
}
