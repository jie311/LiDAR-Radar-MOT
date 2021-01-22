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

#include "../include/data.hpp"
#include "../include/ros.hpp"
#include "../include/lidar.hpp"

// Main function
int main (int argc, char **argv) {

    LidarFramework LiDAR;

    ros::init(argc, argv, "lidar_radar_mot_node");
    ros::NodeHandle N;

    std::cout << "starting program" << std::endl;

}

// Development of callbacks
void LiDAR_CB (const sensor_msgs::PointCloud2::ConstPtr& LidarMsg) {

    pcl::PointCloud<pcl::PointXYZ> nonFilteredCloud;
    pcl::PointCloud<pcl::PointXYZ> FilteredCloud;

    // Transforming ROS message into PCL point cloud
    pcl::fromROSMsg(*LidarMsg, *nonFilteredCloud);

    // Cloud filtering by XYZ and angle
    pcl::PointCloud<pcl::PointXYZ> auxFilteredCloud = LiDAR.CloudFiltering(nonFilteredCloud);
    *FilteredCloud = auxFilteredCloud;

    // Publishing coloured filtered cloud as a ROS msg
    sensor_msgs::PointCloud2 LidarCloud2;
    pcl::toROSMsg(*FilteredCloud, LidarCloud2);
    // TODO: publisher

    // Clustering extraction from filtered cloud


}