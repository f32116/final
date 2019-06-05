#include "stdio.h"
#include "stdlib.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include "Eigen/Dense"

using namespace Eigen;

int time1, time2, time1_store, time2_store, count = 0, count1 = 0;
double store_odom_pos[3000][3], store_ekf_pos[3000][3];
Vector3d angular_velocity, linear_acceleration;
Quaterniond quaternion;
Matrix3d R_imu2cam, R_cam2odom, RM;
sensor_msgs::Imu imu_trans;

class hw4_node
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber visual_odometry, imu_sub, ekf_sub;
    ros::Publisher imu_pub;
  
  public:
    hw4_node()
    {
        visual_odometry = nh.subscribe("/zed/odom", 100, &hw4_node::OdomCallback, this);
        imu_sub = nh.subscribe("/imu/data", 100, &hw4_node::ImuCallback, this);
        ekf_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 100, &hw4_node::EkfCallback, this);
        
        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data", 100);
    }

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        store_odom_pos[count][0] = msg->pose.pose.position.x;
        store_odom_pos[count][1] = msg->pose.pose.position.y;
        store_odom_pos[count][2] = msg->pose.pose.position.z;

        count++;
    }

    void EkfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg2)
    {
        store_ekf_pos[count1][0] = msg2->pose.pose.position.x;
        store_ekf_pos[count1][1] = msg2->pose.pose.position.y;
        store_ekf_pos[count1][2] = msg2->pose.pose.position.z;

        count1++;
    }

    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg1)
    {
        imu_trans.header = msg1->header;

        R_imu2cam << -0.02252259, -0.99974486,  -0.0017194,
                     -0.06487645,  0.00317777, -0.99788824,
                       0.9976391, -0.02236348, -0.06493147;

        R_cam2odom <<  0,  0,  1,
                      -1,  0,  0,
                       0, -1,  0;

        angular_velocity(0) = msg1->angular_velocity.x;
        angular_velocity(1) = msg1->angular_velocity.y;
        angular_velocity(2) = msg1->angular_velocity.z;
        angular_velocity.transpose() = angular_velocity.transpose() * R_imu2cam * R_cam2odom;
        imu_trans.angular_velocity.x = angular_velocity(0);
        imu_trans.angular_velocity.y = angular_velocity(1);
        imu_trans.angular_velocity.z = angular_velocity(2);

        linear_acceleration(0) = msg1->linear_acceleration.x;
        linear_acceleration(1) = msg1->linear_acceleration.y;
        linear_acceleration(2) = msg1->linear_acceleration.z;
        linear_acceleration.transpose() = linear_acceleration.transpose() * R_imu2cam * R_cam2odom;
        imu_trans.linear_acceleration.x = linear_acceleration(0);
        imu_trans.linear_acceleration.y = linear_acceleration(1);
        imu_trans.linear_acceleration.z = linear_acceleration(2);

        quaternion.x() = msg1->orientation.x;
        quaternion.y() = msg1->orientation.y;
        quaternion.z() = msg1->orientation.z;
        quaternion.w() = msg1->orientation.w;
        RM = quaternion.toRotationMatrix();
        RM = RM * R_imu2cam * R_cam2odom;
        quaternion = RM;
        imu_trans.orientation.x = quaternion.x();
        imu_trans.orientation.y = quaternion.y();
        imu_trans.orientation.z = quaternion.z();
        imu_trans.orientation.w = quaternion.w();

        imu_pub.publish(imu_trans);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hw4_node");
    
    hw4_node hw4;
    
    ros::NodeHandle nh1;
    ros::Publisher marker_pub = nh1.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        visualization_msgs::Marker line_strip, line_strip1;
        line_strip.header.frame_id = line_strip1.header.frame_id = "/map";
        line_strip.header.stamp = line_strip1.header.stamp = ros::Time::now();
        line_strip.ns = line_strip1.ns = "points_and_lines";
        line_strip.action = line_strip1.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = line_strip1.pose.orientation.w = 1.0;
        line_strip.id = 0;
        line_strip1.id = 1;
        line_strip.type = line_strip1.type = visualization_msgs::Marker::LINE_STRIP;

        line_strip.scale.x = 0.03;
        line_strip.color.r = 1.0;
        line_strip.color.a = 1.0;

        line_strip1.scale.x = 0.03;
        line_strip1.color.g = 1.0f;
        line_strip1.color.a = 1.0;

        for (int i = 0; i < count; i++)
        {
            geometry_msgs::Point p;
            p.x = store_odom_pos[i][0];
            p.y = store_odom_pos[i][1];
            p.z = store_odom_pos[i][2];
  
            line_strip.points.push_back(p);
        }

        for (int j = 0; j < count1; j++)
        {
            geometry_msgs::Point p1;
            p1.x = store_ekf_pos[j][0];
            p1.y = store_ekf_pos[j][1];
            p1.z = store_ekf_pos[j][2];

            line_strip1.points.push_back(p1);
        }

        marker_pub.publish(line_strip);
        marker_pub.publish(line_strip1);
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
