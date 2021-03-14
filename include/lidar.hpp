#ifndef LIDAR_HPP_
#define LIDAR_HPP_

#include <iostream>
#include <cmath>
#include <chrono>
#include <unordered_set>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/impl/point_types.hpp>

#include <iostream>
#include <vector>

#include "../include/kdtree.hpp"
#include "../include/object.hpp"

    // Cloud filtering function
    pcl::PointCloud<pcl::PointXYZ> CloudFiltering (pcl::PointCloud<pcl::PointXYZ>::Ptr nonFilteredCloud);
    
    // Plane segmentation function with Ransac3D
    std::unordered_set<int> PlaneSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, int MaxIterations, float Threshold);

    // Cloud separation function (TO BE DONE)

    // Clustering extraction function
    void ClusteringExtraction (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float Tolerance, int MinSize, int MaxSize, std::vector<Object> *outputObjects, int *numOutputObjects);

#endif
