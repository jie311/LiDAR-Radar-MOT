#ifndef LIDAR_H_
#define LIDAR_H_

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

    class LidarFramework {

        private:

            // --- XyzFilter ---

            // --- AngleFilter ---

            // --- Ransac3d ---
            // int MaxIterations;
            // float Threshold;
            // pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;

        public:

            // Constructor
            LidarFramework();

            // De-constructor
            ~LidarFramework();

            void HelloWorld();
            pcl::PointCloud<pcl::PointXYZ> CloudFiltering (pcl::PointCloud<pcl::PointXYZ>::Ptr nonFilteredCloud);
            void CloudClustering (pcl::PointCloud<PointXYZ>::Ptr Cloud, float Tolerance, int MinSize, int MaxSize);
            std::unordered_set<int> Ransac3d (pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, int MaxIterations, float Threshold);
                                   
    };


#endif /* LIDAR_H_*/