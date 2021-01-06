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

#include <ros/ros.h>
#include <std_msgs/String.h>

void LiDAR_CB (const t4ac_msgs::BEV_detections_list::ConstPtr& Lidar_Detections_Msg) {

    LiDAR.Xyz_Filter()
    LiDAR.Angle_Filter()
}


int main (int argc, char **argv) {

    LidarFramework LiDAR;

    ros::init(argc, argv, "lidar_radar_mot_node");
    ros::NodeHandle N;

    std::cout << "starting program" << std::endl;

}