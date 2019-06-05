#ifndef GPS_TRANSFORMER_H
#define GPS_TRANSFORMER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl_ros/point_cloud.h>//ros need
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>

// GeographicLib
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

static void gnss_callback();

#endif