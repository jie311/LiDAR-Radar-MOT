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

// #include <ros/ros.h>

#include "lidar.hpp"

int main (int argc, char **argv) {

    LidarFramework LiDAR;

    // ros::init(argc, argv, "lidar");
    // ros::NodeHandle N;

    // ros::Publisher PubLidar = N.advertise <std_msgs::string> ("topicazo", 1000);

    std::cout << "starting program" << std::endl;

    LiDAR.HelloWorld();
    

}