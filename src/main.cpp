/********************************************************************
 * 
 *  Multi-Object Tracking system based on LiDAR and Radar for 
 *  Intelligent Vehicles Applications
 * 
 *  Authors: Santiago Montiel Marín and Carlos Gómez Huélamo
 * 
 *  RobeSafe Research Group
 *  Department of Electronics, University of Alcalá
 * 
 * ******************************************************************/

// -- General purpose includes
#include <iostream>
#include <vector>

// -- ROS modules includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

// -- Submodules includes
#include "../include/lidar.hpp"
#include "../include/object.hpp"

// -- ROS publishers definitions
ros::Publisher pub_LiDAR_FilteredCloud;
ros::Publisher pub_LiDAR_ObstaclesMarkers;
//ros::Publisher pub_LiDAR_BoundingBoxes;

// -- ROS subscribers definitions
ros::Subscriber sub_LiDAR_RawPointCloud;

// -- LiDAR callback
void LiDAR_CB (const sensor_msgs::PointCloud2::ConstPtr& LidarMsg) {

    // Auxiliar point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonFilteredCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr FilteredCloud;

    // 1. Transforming ROS message into PCL point cloud
    pcl::fromROSMsg(*LidarMsg, *nonFilteredCloud);

    // 2. Cloud filtering by XYZ and angle
    pcl::PointCloud<pcl::PointXYZ> auxFilteredCloud = CloudFiltering(nonFilteredCloud);
    *FilteredCloud = auxFilteredCloud;

    // 2.b. Publishing intermediate output 1 -> filtered cloud
    sensor_msgs::PointCloud2 msgFilteredCloud;
    pcl::toROSMsg(*FilteredCloud, msgFilteredCloud);
    msgFilteredCloud.header.frame_id = LidarMsg->header.frame_id;
    msgFilteredCloud.header.stamp = LidarMsg->header.stamp;
    pub_LiDAR_FilteredCloud.publish(msgFilteredCloud);

    // 3. Plane segmentation
    // 3.b. Publishing intermediate output 2 -> segmented plane

    // 4. Clustering extraction
    // 4.b. Publsihing clusters
    // 4.c. Publishing bounding boxes

}

// -- Main function
int main (int argc, char** argv) {

    // -- ROS initialization
    ros::init(argc, argv, "lidar_radar_mot_node");
    ros::NodeHandle nh;

    // -- ROS publishers initialization
    pub_LiDAR_FilteredCloud = nh.advertise<sensor_msgs::PointCloud2>("topic_que_tampoco_se_cual_es", 1, true);
    pub_LiDAR_ObstaclesMarkers = nh.advertise<visualization_msgs::Marker>("Otro_topic_cuyo_nombre_ya_decidire", 1, true);

    // -- ROS subscribers initialization
    sub_LiDAR_RawPointCloud = nh.subscribe("topic_que_no_conozco", 1, &LiDAR_CB);

    // -- ROS spin
    ros::spin();

}

