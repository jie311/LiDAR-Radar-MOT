#include <iostream>

#include "lidar.hpp"

// Constructor
LidarFramework::LidarFramework(void) {}

// De-constructor
LidarFramework::~LidarFramework(void) {}

void LidarFramework::SayHello() {

    std::cout << "lidar says hello world" << std::endl;

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