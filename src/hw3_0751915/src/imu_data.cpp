#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "Eigen/Dense"

using namespace Eigen;

int time1, time2, time1_store, time2_store, count = 0;
double store_pos[3000][3];
double dt, sigma;
Vector3d angular_velocity, linear_acceleration;
Vector3d Ag = Vector3d::Zero();
Vector3d Vg = Vector3d::Zero();
Vector3d Sg = Vector3d::Zero();
Vector3d g;
Matrix3d C;
Matrix3d I = Matrix3d::Identity();
Matrix3d B;

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    time1 = msg->header.stamp.sec;
    time2 = msg->header.stamp.nsec;

    angular_velocity(0) = msg->angular_velocity.x;
    angular_velocity(1) = msg->angular_velocity.y;
    angular_velocity(2) = msg->angular_velocity.z;

    linear_acceleration(0) = msg->linear_acceleration.x;
    linear_acceleration(1) = msg->linear_acceleration.y;
    linear_acceleration(2) = msg->linear_acceleration.z;

    if(count == 0)
    {
        g << linear_acceleration(0), linear_acceleration(1), linear_acceleration(2);
        dt = 0;
    }
    else if(time1 - time1_store == 1)
    {
        dt = (double)time2 * 0.000000001 + 1 - (double)time2_store * 0.000000001;
    }
    else if (time1 - time1_store == 0)
    {
        dt = ((double)time2 - (double)time2_store) * 0.000000001;
    }

    sigma = dt * (sqrt(angular_velocity(0)*angular_velocity(0)+angular_velocity(1)*angular_velocity(1)+angular_velocity(2)*angular_velocity(2)));

    B <<                       0, -angular_velocity(2)*dt,  angular_velocity(1)*dt,
          angular_velocity(2)*dt,                       0, -angular_velocity(0)*dt,
         -angular_velocity(1)*dt,  angular_velocity(0)*dt,                       0;
    
    if(count == 0)
    {
      C = I;
    }
    else
    {
      C = C * ( I + (sin(sigma)/sigma)*B + ((1-cos(sigma))/(sigma*sigma))*(B*B) );
    }
    Ag = C * linear_acceleration;
    
    Vg = Vg + dt * (Ag - g);
    Sg = Sg + dt * Vg;

    //ROS_INFO("dt = %f", dt);

    store_pos[count][0] = Sg(0);
    store_pos[count][1] = Sg(1);
    store_pos[count][2] = Sg(2);

    time1_store = time1;
    time2_store = time2;

    count++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hw3_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_data = nh.subscribe("/imu/data", 1000, ImuCallback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    ros::Rate loop_rate(30);
    
    visualization_msgs::Marker points, line_strip, line_list;

    while (ros::ok())
    {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "/map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        line_strip.scale.x = 0.1;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        for (int i = 0; i < count; i++)
        {
            geometry_msgs::Point p;
            p.x = store_pos[i][0];
            p.y = store_pos[i][1];
            p.z = store_pos[i][2];
  
            line_strip.points.push_back(p);
        }

        marker_pub.publish(line_strip);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
