/********************************************************************
 * 
 *  Multi-Object Tracking system based on LiDAR and Radar for 
 *  Intelligent Vehicles Applications
 * 
 *  @authors: Santiago Montiel Marín and Carlos Gómez Huélamo
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
#include <std_msgs/ColorRGBA.h>
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

    // 1. Transforming ROS message into PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonFilteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*LidarMsg, *nonFilteredCloud);

    // 2. Cloud filtering by XYZ and angle
    pcl::PointCloud<pcl::PointXYZ> auxFilteredCloud = CloudFiltering(nonFilteredCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr FilteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
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
    std::vector<Object> Obstacles;
    int numObstacles = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ObstaclesCloud (new pcl::PointCloud<pcl::PointXYZ>);
    *ObstaclesCloud = auxFilteredCloud;
    ClusteringExtraction(ObstaclesCloud, 1, 10, 400, &Obstacles, &numObstacles);

    // -- Colours
    std_msgs::ColorRGBA red;
    red.a = 1.0, red.r = 1.0, red.g = 0.0, red.b = 0.0;

    // 4.b. Publsihing clusters
    for (int i = 0; i < Obstacles.size(); i++) {

		visualization_msgs::Marker ObstaclesMarker;

        ObstaclesMarker.header.frame_id = "/ego_vehicle/lidar/lidar1";                          // map == global coordinates. Base_link == local coordinates
        ObstaclesMarker.header.stamp = LidarMsg->header.stamp;
        ObstaclesMarker.ns = "map_manager_visualization";
        ObstaclesMarker.action = visualization_msgs::Marker::ADD;
        ObstaclesMarker.type = visualization_msgs::Marker::SPHERE;
        ObstaclesMarker.id = Obstacles[i].object_id;

        ObstaclesMarker.points.clear();

        ObstaclesMarker.color = red;
        ObstaclesMarker.scale.x = 0.50;
        ObstaclesMarker.scale.y = 0.50;
        ObstaclesMarker.scale.z = 0.50;
        ObstaclesMarker.lifetime = ros::Duration(0.40);

        std::cout << "XYZ: " << Obstacles[i].centroid_x << " " << Obstacles[i].centroid_y << " " << Obstacles[i].centroid_z << std::endl;
        ObstaclesMarker.pose.position.x = Obstacles[i].centroid_x;
        ObstaclesMarker.pose.position.y = Obstacles[i].centroid_y;
        ObstaclesMarker.pose.position.z = Obstacles[i].centroid_z;

		pub_LiDAR_ObstaclesMarkers.publish(ObstaclesMarker);
	}

    // 4.c. Publishing bounding boxes

}

// -- Main function
int main (int argc, char** argv) {

    // -- ROS initialization
    ros::init(argc, argv, "lidar_radar_mot_node");
    ros::NodeHandle nh;

    // -- ROS publishers initialization
    pub_LiDAR_FilteredCloud = nh.advertise<sensor_msgs::PointCloud2>("/t4ac_perception/perception/detection/filtered_cloud", 1, true);
    pub_LiDAR_ObstaclesMarkers = nh.advertise<visualization_msgs::Marker>("/t4ac_perception/perception/detection/obstacle_markers", 1, true);

    // -- ROS subscribers initialization
    sub_LiDAR_RawPointCloud = nh.subscribe("/carla/ego_vehicle/lidar/lidar1/point_cloud", 1, &LiDAR_CB);

    // -- ROS spin
    ros::spin();

}

