#include <string>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "ground_truth_node");    

    ros::NodeHandle nh;
    ros::Publisher marker_pub1 = nh.advertise<visualization_msgs::Marker>("ground_truth1", 1);
    ros::Publisher marker_pub2 = nh.advertise<visualization_msgs::Marker>("ground_truth2", 1);

//***********************************  set line strip  *******************************************
    visualization_msgs::Marker line_strip1, line_strip2;
    line_strip1.header.frame_id = line_strip2.header.frame_id = "/map";
    line_strip1.ns = line_strip2.ns = "linestrip";
    line_strip1.action = line_strip2.action = visualization_msgs::Marker::ADD;
    line_strip1.pose.orientation.w = line_strip2.pose.orientation.w = 1.0;
    line_strip1.id = 1;
    line_strip2.id = 2;
    line_strip1.type = line_strip2.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip1.scale.x = line_strip2.scale.x = 1.0;
    line_strip1.color.g = line_strip2.color.g = 1.0f;
    line_strip1.color.a = line_strip2.color.a = 1.0;

//********************************  get csv package path  ****************************************
    std::string pkgpath = ros::package::getPath("final_competition1");
    std::string path1 = pkgpath + "/localization_ground_truth/localization_ground_truth_1.csv";
    std::string path2 = pkgpath + "/localization_ground_truth/localization_ground_truth_2.csv";

//************************************  read csv data  *******************************************
    std::fstream file;
    std::string line, data;
    int count1;

    file.open(path1.c_str(), std::ifstream::in);
    if( !file )
    {
        ROS_INFO("Cannot open file!");
        return -1;
    }
    while(std::getline(file, line))
    {
        count1 = 0;
        std::istringstream templine1(line);
        geometry_msgs::Point point;
        while (getline( templine1, data,','))
	    {
	        if (count1 == 1)
                point.x = atof(data.c_str());
            else if (count1 == 2)
                point.y = atof(data.c_str());
            else if (count1 == 3)
                point.z = atof(data.c_str());
            count1++;
	    }
        line_strip1.points.push_back(point);
    }
    file.close();

    file.open(path2.c_str(), std::ifstream::in);
    if( !file )
    {
        ROS_INFO("Cannot open file!");
        return -1;
    }
    while(std::getline(file, line))
    {
        count1 = 0;
        std::istringstream templine2(line);
        geometry_msgs::Point point;
        while (getline( templine2, data,','))
	    {
	        if (count1 == 1)
                point.x = atof(data.c_str());
            else if (count1 == 2)
                point.y = atof(data.c_str());
            else if (count1 == 3)
                point.z = atof(data.c_str());
            count1++;
	    }
        line_strip2.points.push_back(point);
    }
    file.close();

    ros::Rate rate(100);
    while(ros::ok())
    {
        marker_pub1.publish(line_strip1);
        marker_pub2.publish(line_strip2);
        ros::spinOnce();
    }
    ros::spin();
    return (0);
}
