#include "lidar.hpp"

/****************************************************
 *  Constructor & Deconstructor
 * 
 *  I: -
 *  O: -
 * **************************************************/
LidarFramework::LidarFramework(void) {}

LidarFramework::~LidarFramework(void) {}

/****************************************************
 *  Hello World function
 * 
 *  I: -
 *  O: -
 * **************************************************/
void LidarFramework::HelloWorld() {

    std::cout << "lidar says hello world" << std::endl;

}

/****************************************************
 *  Ransac3D algorithm (plane fitting algorithm)
 * 
 *  I: Cloud, MaxIterations, Threshold
 *  O: InlierPoints
 * **************************************************/

std::unordered_set<int> LidarFramework::Ransac3d (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, int MaxIterations, float Threshold) {

    std::unordered_set<int> inlierPoints;
    while (MaxIterations--) {


    }

    return inlierPoints;

}

/*
LidarFramework::XyzFilter () {



    return FilteredCloud;

}

LidarFramework::AngleFilter () {



    return FilteredCloud;

}

LidarFramework::Ransac3D () {



    return Plane;

}
*/