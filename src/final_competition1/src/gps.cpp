#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
 
static double lat_origin = 24.7855644226;
static double lon_origin = 120.997009277;
static double alt_origin = 127.651;
static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
static GeographicLib::LocalCartesian proj = GeographicLib::LocalCartesian(lat_origin, lon_origin, alt_origin);

std::ofstream file;
std::string pkgpath, path;
int count = 0, count1 = 0;

class store_data
{
    public:
        int sec;
        int nsec;
        double x;
        double y;
        double z;
};

std::vector<store_data> gps_list;

ros::Publisher marker_pub;
visualization_msgs::Marker line_strip;

static void gps_callback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    double x, y, z;
 
    // Warning: if the measurement of altitude is wrong, then the result of projection will be wrong.
    proj.Forward(msg->latitude, msg->longitude, alt_origin, x, y, z);

    geometry_msgs::Point p;

    p.x = x;
    p.y = y;
    p.z = z;

    line_strip.points.push_back(p);
    if(line_strip.points.size()>15000)
    {
      line_strip.points.clear();
    }
    marker_pub.publish(line_strip);

    store_data gps;

    gps.sec = msg->header.stamp.sec;
    gps.nsec = msg->header.stamp.nsec;
    gps.x = x;
    gps.y = y;
    gps.z = z;
    gps_list.push_back(gps);
    count++;

    return;
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "gps_trans_node");
    ros::NodeHandle nh;

    pkgpath = ros::package::getPath("final_competition1");
    path = pkgpath + "/localization_ground_truth/localization_measurement.csv";
    file.open(path.c_str(), std::ios::trunc);
    if( !file )
    {
        ROS_INFO("Cannot open file first!");
    }
    file.close();

    ros::Subscriber gnss_sub = nh.subscribe("fix", 100000, gps_callback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/gps_marker",1);

    line_strip.header.frame_id = "/map";
    line_strip.ns = "linestrip";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = 1.0;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    while(ros::ok)
    {
        while(count > count1)
        {
            std::string s;
            std::stringstream ss(s);
            ss << gps_list[count1].sec << "." << gps_list[count1].nsec << "," << gps_list[count1].x << "," << gps_list[count1].y << "," << gps_list[count1].z << std::endl;

            if(count1 == 0)
            {
                file.open(path.c_str(), std::ios::out);
                if( !file )
                {
                    ROS_INFO("Cannot open file!");
                }
                else
                {
                    file << ss.str();
                }
                file.close();
            }
            else
            {
                file.open(path.c_str(), std::ios::app);
                if( !file )
                {
                    ROS_INFO("Cannot open file!");
                }
                else
                {
                    file << ss.str();
                }
                file.close();
            }
            count1++;
        }
        ros::spinOnce();
    }
    ros::spin();
}
